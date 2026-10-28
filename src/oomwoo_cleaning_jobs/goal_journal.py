"""Durable Nav2 goal-UUID journal for crash recovery, fail-closed.

An action client can crash *after* a ROS2 goal request reaches Nav2 and
*before* its acceptance callback. Therefore the exact UUID must be
written atomically to disk BEFORE send_goal_async. A journal with a
nonterminal entry blocks new goal issuance until the old UUID is
reconciled. An accepted cancel ACK alone cannot mark it terminal.

No ROS2 dependency. A stable nonblocking Linux flock guards each
journal transition across cooperating processes sharing one path.
This does NOT replace an exclusive cross-process Nav2 action owner.
Special filesystem nodes never block journal admission: open uses O_NONBLOCK
before inode validation, then rejects every non-private regular file.
"""
from __future__ import annotations

import json
import fcntl
import stat
from contextlib import contextmanager
from functools import wraps
import os
from pathlib import Path
import re
import tempfile

from .restart_witness import OrphanNav2Witness

class GoalJournalError(RuntimeError):
    """Goal provenance missing, corrupted or unresolved."""


_UUID = re.compile(r"[0-9a-f]{32}\Z")
_TOKEN = re.compile(r"[0-9a-f]{32}-[1-9][0-9]*\Z")
_DIGEST = re.compile(r"[0-9a-f]{64}\Z")
_TERMINAL = frozenset(("succeeded","canceled","aborted","rejected"))



def _serialized_writer(method):
    """Serialize one durable transition across local cooperating processes."""
    @wraps(method)
    def execute(self, *args, **kwargs):
        with self._writer_lock():
            return method(self, *args, **kwargs)
    return execute


class GoalJournal:
    def __init__(self,path:Path):
        self.path=Path(path)
        # Always check even a dangling symlink: Path.exists() follows links
        # and would mistake a broken/substituted journal for a clean start.
        self.read()

    def read(self) -> dict | None:
        # Linux-only fail-closed read. A symlink to an unrelated terminal
        # journal must NEVER clear the unknown in-flight Nav2 UUID.
        try:
            fd=os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)
        except FileNotFoundError:
            return None
        except OSError as exc:
            raise GoalJournalError("goal journal inaccessible/linked: do NOT rearm Nav2") from exc
        try:
            with os.fdopen(fd,"rb") as source:
                info=os.fstat(source.fileno())
                if (not stat.S_ISREG(info.st_mode)
                    or info.st_nlink!=1
                    or stat.S_IMODE(info.st_mode)&0o077):
                    raise GoalJournalError("goal journal inode/permissions unsafe")
                # A single Nav2 goal record should be tiny. Bound corrupted
                # files before JSON parsing to avoid unbounded RAM use.
                raw=source.read(65537)
            if len(raw)>65536:
                raise GoalJournalError("goal journal too large: refuse rearm")
            obj=json.loads(raw)
        except (OSError,ValueError) as exc:
            raise GoalJournalError("goal journal unreadable: do NOT rearm Nav2") from exc
        if not isinstance(obj,dict):
            raise GoalJournalError("goal journal must be a JSON object")
        allowed={"schema","uuid","token","job_id","map_digest","target_cell","state","outcome","revision"}
        if set(obj)!=allowed:
            raise GoalJournalError("goal journal schema mismatch")
        if (type(obj["schema"]) is not int or obj["schema"]!=1
            or not isinstance(obj["uuid"],str) or not _UUID.fullmatch(obj["uuid"])
            or not isinstance(obj["token"],str) or not _TOKEN.fullmatch(obj["token"])
            or not isinstance(obj["job_id"],str) or not obj["job_id"]
            or not isinstance(obj["map_digest"],str) or not _DIGEST.fullmatch(obj["map_digest"])
            or type(obj["target_cell"]) is not int or obj["target_cell"]<0
            or not isinstance(obj["state"],str)
            or obj["state"] not in ("submitting","accepted","terminal")
            or type(obj["revision"]) is not int or obj["revision"]<1):
            raise GoalJournalError("invalid goal journal values")
        if (obj["state"]=="terminal") != (isinstance(obj["outcome"],str) and obj["outcome"] in _TERMINAL):
            raise GoalJournalError("terminal goal requires real outcome; active cannot contain it")
        return obj

    def _write(self,state:dict):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        fd,tmp=tempfile.mkstemp(prefix=".nav2-goal-",suffix=".tmp",dir=self.path.parent)
        try:
            with os.fdopen(fd,"w",encoding="utf-8") as f:
                json.dump(state,f,sort_keys=True,separators=(",",":"))
                f.write("\n");f.flush();os.fsync(f.fileno())
            os.replace(tmp,self.path)
            if hasattr(os,"O_DIRECTORY"):
                directory=os.open(self.path.parent,os.O_DIRECTORY)
                try:os.fsync(directory)
                finally:os.close(directory)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)


    @contextmanager
    def _writer_lock(self):
        """Stable lock inode guards journal read/compare/atomic replacement.

        Nonblocking; contention, link substitution or unsafe mode refuses
        the transition. All callers must share this filesystem/path.
        """
        self.path.parent.mkdir(parents=True, exist_ok=True)
        lock_path=self.path.with_name(self.path.name+".lock")
        flags=(os.O_RDWR | os.O_CREAT | getattr(os,"O_NOFOLLOW",0)
               | getattr(os,"O_NONBLOCK",0))
        try:
            fd=os.open(lock_path,flags,0o600)
        except OSError as exc:
            raise GoalJournalError("Nav2 journal lock unavailable: refuse action") from exc
        try:
            info=os.fstat(fd)
            if (not stat.S_ISREG(info.st_mode) or info.st_nlink!=1
                or stat.S_IMODE(info.st_mode)&0o077):
                raise GoalJournalError("Nav2 journal lock inode/permissions unsafe")
            try:
                fcntl.flock(fd,fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as exc:
                raise GoalJournalError("Nav2 journal lock busy/unavailable") from exc
            yield
        finally:
            os.close(fd)

    @_serialized_writer
    def begin(self,*,uuid_hex:str,token:str,job_id:str,
              map_digest:str,target_cell:int) -> dict:
        old=self.read()
        if old is not None and old["state"]!="terminal":
            raise GoalJournalError("previous Nav2 UUID unresolved: refuse new action")
        rev=old["revision"]+1 if old is not None else 1
        record={"schema":1,"uuid":uuid_hex,"token":token,"job_id":job_id,
                "map_digest":map_digest,"target_cell":target_cell,
                "state":"submitting","outcome":None,"revision":rev}
        # Validate BEFORE overwriting any proven/terminal record.
        if (not isinstance(uuid_hex,str) or not _UUID.fullmatch(uuid_hex)
            or not isinstance(token,str) or not _TOKEN.fullmatch(token)
            or not isinstance(map_digest,str) or not _DIGEST.fullmatch(map_digest)
            or not isinstance(job_id,str) or not job_id
            or type(target_cell) is not int or target_cell<0):
            raise GoalJournalError("invalid new Nav2 goal metadata")
        self._write(record)
        return record

    @_serialized_writer
    def accepted(self,*,token:str,uuid_hex:str):
        old=self.read()
        if old is None or old["token"]!=token or old["uuid"]!=uuid_hex:
            raise GoalJournalError("acceptance cannot be tied to durable sent goal")
        if old["state"]=="terminal":
            raise GoalJournalError("stale acceptance after terminal result")
        if old["state"]!="accepted":
            self._write({**old,"state":"accepted","revision":old["revision"]+1})

    @_serialized_writer
    def terminal(self,*,token:str,uuid_hex:str,outcome:str):
        old=self.read()
        if old is None or old["token"]!=token or old["uuid"]!=uuid_hex:
            raise GoalJournalError("terminal result cannot be tied to persisted Nav2 UUID")
        if outcome not in _TERMINAL:
            raise GoalJournalError("nonterminal Nav2 response")
        if old["state"]=="terminal":
            if old["outcome"]!=outcome:
                raise GoalJournalError("conflicting terminal outcomes")
            return
        self._write({**old,"state":"terminal","outcome":outcome,
                     "revision":old["revision"]+1})

    def orphaned_uuid(self)->str | None:
        entry=self.read()
        return entry["uuid"] if entry and entry["state"]!="terminal" else None

    @_serialized_writer
    def mark_reconciled_canceled(self,witness:OrphanNav2Witness)->None:
        """Only after direct trusted ROS2 observation yielded this witness.

        The ROS2 adapter MUST ALSO independently confirm that no OTHER
        active goals exist, including goals from other action clients.
        Never deserialize "proof" from arbitrary user-supplied JSON.
        """
        current=self.read()
        if current is None or current["state"]=="terminal":
            raise GoalJournalError("no unresolved UUID to reconcile")
        if not isinstance(witness,OrphanNav2Witness):
            raise GoalJournalError("must supply live verified ROS2 witness")
        proof=witness.require_ready()
        if (proof["known_old_goal_uuid"]!=current["uuid"]
            or proof["cancel_response_named_old_goal"] is not True
            or proof["terminal_canceled"] is not True):
            raise GoalJournalError("orphan witness and durable UUID do not match")
        # Do not re-enter a second file lock: the proof and journal
        # transition MUST be one atomic writer transaction.
        self._write({**current,"state":"terminal","outcome":"canceled",
                     "revision":current["revision"]+1})
