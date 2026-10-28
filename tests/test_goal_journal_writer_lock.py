"""Linux multiprocess Nav2 UUID journal compare/write adversarial tests.

All actions are synthetic; no ROS2 graph, motors, or physical MCU involved.
"""
import fcntl
import multiprocessing
import os
from pathlib import Path
import queue
import tempfile
import unittest

from oomwoo_cleaning_jobs.goal_journal import GoalJournal, GoalJournalError

A="a1"*16
B="b2"*16
C="c3"*16
D="d4"*16
TOKEN1="e1"*16+"-1"
TOKEN2="e2"*16+"-2"
TOKEN3="e3"*16+"-3"
DIGEST="a4"*32

def _racing_begin(path, ident, token, trigger, barrier, ready, results):
    try:
        journal=GoalJournal(Path(path))
        original_read=journal.read
        def raced_read():
            previous=original_read()
            if previous is not None and previous["state"]=="terminal":
                try:barrier.wait(timeout=.45)
                except Exception:pass
            return previous
        journal.read=raced_read
        ready.put("ready")
        if not trigger.wait(6):
            results.put("timeout")
            return
        try:
            journal.begin(uuid_hex=ident,token=token,
                job_id="race-lab",map_digest=DIGEST,target_cell=2)
        except GoalJournalError:
            results.put("blocked")
        else:
            results.put("started")
    except BaseException as exc:
        results.put("error:"+repr(exc))

class GoalJournalWriterLockTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/"nav2.json"
        self.journal=GoalJournal(self.path)

    def test_double_begin_on_prior_terminal_is_serialized_across_processes(self):
        self.journal.begin(uuid_hex=A,token=TOKEN1,
            job_id="race-lab",map_digest=DIGEST,target_cell=1)
        self.journal.terminal(token=TOKEN1,uuid_hex=A,outcome="succeeded")
        ctx=multiprocessing.get_context("fork")
        trigger=ctx.Event()
        barrier=ctx.Barrier(2)
        ready=ctx.Queue()
        results=ctx.Queue()
        workers=[
            ctx.Process(target=_racing_begin,
                args=(str(self.path),id,token,trigger,barrier,ready,results))
            for id,token in ((B,TOKEN2),(C,TOKEN3))
        ]
        for child in workers:child.start()
        try:
            self.assertEqual([ready.get(timeout=6) for _ in workers],
                             ["ready","ready"])
            trigger.set()
            outcomes=sorted(results.get(timeout=8) for _ in workers)
            self.assertEqual(outcomes,["blocked","started"])
        finally:
            trigger.set()
            for child in workers:
                child.join(timeout=5)
                if child.is_alive():
                    child.terminate()
                    child.join(timeout=3)
        self.assertTrue(all(child.exitcode==0 for child in workers))
        final=GoalJournal(self.path).read()
        self.assertEqual(final["state"],"submitting")
        self.assertIn(final["uuid"],(B,C))
        self.assertEqual(final["revision"],3)

    def test_lock_contention_refuses_write_without_mutation(self):
        lock=self.path.with_name(self.path.name+".lock")
        # Establish a normal private inode before the contention test.
        self.journal.begin(uuid_hex=A,token=TOKEN1,job_id="lab",
                           map_digest=DIGEST,target_cell=1)
        before=self.path.read_bytes()
        fd=os.open(lock,os.O_RDWR)
        try:
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaises(GoalJournalError):
                self.journal.accepted(token=TOKEN1,uuid_hex=A)
            self.assertEqual(self.path.read_bytes(),before)
        finally:
            fcntl.flock(fd,fcntl.LOCK_UN)
            os.close(fd)

    def test_lock_symlink_rejected_before_goal_send(self):
        marker=Path(self.temp.name)/"unrelated"
        marker.write_text("untouched",encoding="utf-8")
        lock=self.path.with_name(self.path.name+".lock")
        lock.symlink_to(marker)
        with self.assertRaises(GoalJournalError):
            self.journal.begin(uuid_hex=A,token=TOKEN1,job_id="lab",
                map_digest=DIGEST,target_cell=1)
        self.assertFalse(self.path.exists())
        self.assertEqual(marker.read_text(encoding="utf-8"),"untouched")

    def test_lockfile_persists_with_private_permissions(self):
        self.journal.begin(uuid_hex=A,token=TOKEN1,job_id="lab",
                           map_digest=DIGEST,target_cell=1)
        lock=self.path.with_name(self.path.name+".lock")
        self.assertTrue(lock.is_file())
        self.assertEqual(lock.stat().st_mode&0o777,0o600)
        self.journal.accepted(token=TOKEN1,uuid_hex=A)
        self.journal.terminal(token=TOKEN1,uuid_hex=A,outcome="canceled")
        self.assertTrue(lock.exists())
        self.assertEqual(GoalJournal(self.path).read()["outcome"],"canceled")
