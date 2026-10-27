"""Write-ahead log for safely mirroring verified swept-floor observations.

A persistent long job may maintain CleaningJobs (public job state)
and SweepCampaign (route/room state) in *different* checkpoint files.
A crash between writing those files risks one believing progress is
saved while the other loses it. This fsync'd WAL records each TRUSTED
coverage event before either projection, then reconciles idempotently.

Only feed validated external SimCoverageBridge deltas from an ACTIVE,
accepted Nav2 goal. The journal itself cannot authenticate a ROS topic
or prove a physical vacuum motor was on. No Nav2/MCU commands here.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import tempfile

from .grid import Grid
from .engine import CleaningJobs
from .sweep_campaign import SweepCampaign


class CoverageLedgerError(RuntimeError):
    """Unresolved or mismatched externally measured coverage event."""


def _hash_cells(cells):
    return hashlib.sha256(json.dumps(sorted(cells),separators=(",",":")).encode()).hexdigest()


class CoverageWAL:
    def __init__(self,path:Path,*,grid:Grid,scope:frozenset[int],
                 producer:str,job_id:str,max_events:int=100000):
        if (not isinstance(grid,Grid) or not isinstance(scope,frozenset)
            or not scope or not scope<=grid.traversable
            or not isinstance(producer,str) or not re.fullmatch(r"[A-Za-z0-9_/-]{1,100}",producer)
            or not isinstance(job_id,str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}",job_id)
            or type(max_events) is not int or not 1<=max_events<=1000000):
            raise CoverageLedgerError("invalid journal source/job/scope/bounds")
        self.grid=grid
        self.scope=scope
        self.producer=producer
        self.job_id=job_id
        self.max_events=max_events
        self.path=Path(path)
        self.identity={"schema":1,"map_digest":grid.fingerprint(),
                       "scope_digest":_hash_cells(scope),
                       "producer":producer,"job_id":job_id}
        self._state=None
        if self.path.exists():
            self._state=self._read()
        else:
            self._write({**self.identity,"revision":1,"events":{}})

    def _validate(self,v):
        if (not isinstance(v,dict)
            or set(v)!=set(self.identity)|{"revision","events"}
            or any(v.get(k)!=a for k,a in self.identity.items())
            or type(v.get("revision")) is not int or v["revision"]<1
            or not isinstance(v.get("events"),dict)
            or len(v["events"])>self.max_events):
            raise CoverageLedgerError("WAL missing/incorrect map/producer/schema")
        for key,entry in v["events"].items():
            if (not isinstance(key,str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,100}",key)
                or not isinstance(entry,dict)
                or set(entry)!={"cells","status"}
                or entry["status"] not in ("pending","applied")
                or not isinstance(entry["cells"],list)
                or not entry["cells"] or len(entry["cells"])>10000
                or any(type(i) is not int for i in entry["cells"])
                or entry["cells"]!=sorted(set(entry["cells"]))
                or not set(entry["cells"])<=self.scope):
                raise CoverageLedgerError("WAL event corrupted or outside job scope")

    def _read(self):
        try:v=json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError,ValueError) as exc:
            raise CoverageLedgerError("WAL unreadable; refuse to acknowledge") from exc
        self._validate(v)
        return v

    def _write(self,v):
        self._validate(v)
        if self._state is not None:
            disk=self._read()
            if disk["revision"]!=self._state["revision"]:
                raise CoverageLedgerError("WAL modified by other writer")
        self.path.parent.mkdir(parents=True,exist_ok=True)
        fd,tmp=tempfile.mkstemp(prefix=".coverage-wal-",dir=self.path.parent)
        try:
            with os.fdopen(fd,"w",encoding="utf-8") as f:
                json.dump(v,f,ensure_ascii=False,sort_keys=True,separators=(",",":"))
                f.write("\n");f.flush();os.fsync(f.fileno())
            os.replace(tmp,self.path)
            if hasattr(os,"O_DIRECTORY"):
                dirfd=os.open(self.path.parent,os.O_DIRECTORY)
                try:os.fsync(dirfd)
                finally:os.close(dirfd)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)
        self._state=v

    def _change(self,**fields):
        self._write({**self._state,**fields,"revision":self._state["revision"]+1})

    def stage(self,event_id:str,cells:tuple[int,...],*,
              externally_validated:bool):
        """WAL fsync BEFORE acknowledging in either consumer checkpoint."""
        if (not isinstance(event_id,str)
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,100}",event_id)
            or type(cells) is not tuple or not cells or len(cells)>10000
            or any(type(i) is not int for i in cells)
            or not set(cells)<=self.scope
            or externally_validated is not True):
            raise CoverageLedgerError("unverified or out-of-scope measurement")
        data=sorted(set(cells))
        if event_id in self._state["events"]:
            old=self._state["events"][event_id]
            if old["cells"]!=data:
                raise CoverageLedgerError("same coverage event ID has conflicting payload")
            return
        if len(self._state["events"])>=self.max_events:
            raise CoverageLedgerError("WAL event budget exhausted")
        events={**self._state["events"],event_id:{"cells":data,"status":"pending"}}
        self._change(events=events)

    def pending(self):
        return tuple(k for k,e in self._state["events"].items()
                     if e["status"]=="pending")

    def project(self,event_id:str,job:CleaningJobs,campaign:SweepCampaign,
                *,simulate_crash_after_job:bool=False,
                simulate_crash_after_campaign:bool=False):
        """Idempotently project the WAL event to both durable ledgers.

        Test-only fault hooks intentionally raise after one of the
        separately fsynced projections. Never mark an event applied until
        BOTH ledgers agree. If a node restarts paused, first independently
        reconcile Nav2 and explicitly resume before replaying pending work.
        """
        event=self._state["events"].get(event_id)
        if event is None:
            raise CoverageLedgerError("unknown external coverage event")
        if event["status"]=="applied":
            return False
        cells=set(event["cells"])
        if (job.grid.fingerprint()!=self.grid.fingerprint()
            or campaign.grid.fingerprint()!=self.grid.fingerprint()
            or not cells<=set(job.state["targets"])
            or not cells<=campaign.scope):
            raise CoverageLedgerError("coverage projection source or scope differs")
        job_done=set(job.state["cleaned"])
        if not cells<=job_done:
            if job.status()["state"]!="running":
                raise CoverageLedgerError("job must be safely resumed before WAL replay")
            job.acknowledge_cleaned(tuple(sorted(cells)))
        if simulate_crash_after_job:
            raise CoverageLedgerError("TEST INJECTED CRASH AFTER JOB FSYNC")
        campaign_done=set(campaign.state["covered"])
        if not cells<=campaign_done:
            if campaign.summary().state!="running":
                raise CoverageLedgerError("campaign must be safely resumed before WAL replay")
            campaign.apply_external_coverage(tuple(sorted(cells)))
        if simulate_crash_after_campaign:
            raise CoverageLedgerError("TEST INJECTED CRASH AFTER CAMPAIGN FSYNC")
        if not cells<=set(job.state["cleaned"]) or not cells<=set(campaign.state["covered"]):
            raise CoverageLedgerError("one durable ledger did not record external evidence")
        events={**self._state["events"]}
        events[event_id]={**event,"status":"applied"}
        self._change(events=events)
        return True

    def require_drained(self):
        if self.pending():
            raise CoverageLedgerError("pending verified coverage receipts need reconciliation")
