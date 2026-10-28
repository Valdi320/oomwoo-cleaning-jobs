"""Multiprocess lost-update and stale-owner guards; synthetic masks only."""
import fcntl
import multiprocessing
import os
from pathlib import Path
import queue
import tempfile
import time
import unittest

from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.zone_revision import ZoneUpdateError, ZoneUpdateTransaction


def _competing_writer(base, old, proposal, path, start, ready, results):
    try:
        tx=ZoneUpdateTransaction(base,old,Path(path))
        original=tx._read
        def slow_read():
            value=original()
            time.sleep(.12)
            return value
        tx._read=slow_read
        ready.put("ready")
        if not start.wait(8):
            results.put("timeout")
            return
        try:
            tx.stage(proposal,paused=True,goal_terminal=True,
                     verified_source_map=True)
        except ZoneUpdateError:
            results.put("stale")
        else:
            results.put("committed")
    except BaseException as exc:
        results.put(type(exc).__name__)


class ZoneWriterLockTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/"zone.json"
        self.base=Raster(8,8,.1,0.,0.,0.,(0,)*64)
        def mask(indices):
            pixels=[0]*64
            for i in indices:pixels[i]=100
            return Raster(8,8,.1,0.,0.,0.,tuple(pixels))
        self.old=mask((9,10))
        self.a=mask((17,18))
        self.b=mask((25,26))

    def test_stale_process_blocks_nav_after_other_process_stages(self):
        first=ZoneUpdateTransaction(self.base,self.old,self.path)
        stale=ZoneUpdateTransaction(self.base,self.old,self.path)
        self.assertFalse(stale.must_block_nav())
        first.stage(self.a,paused=True,goal_terminal=True,verified_source_map=True)
        self.assertTrue(first.must_block_nav())
        self.assertTrue(stale.must_block_nav())

    def test_lost_or_corrupted_durable_record_blocks_nav(self):
        tx=ZoneUpdateTransaction(self.base,self.old,self.path)
        self.path.unlink()
        self.assertTrue(tx.must_block_nav())
        self.path.write_text("invalid-json")
        self.assertTrue(tx.must_block_nav())

    def test_nonblocking_lock_contention_fails_closed(self):
        tx=ZoneUpdateTransaction(self.base,self.old,self.path)
        lock=self.path.with_name(self.path.name+".lock")
        fd=os.open(lock,os.O_RDWR)
        try:
            fcntl.flock(fd,fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(ZoneUpdateError,"lock"):
                tx.stage(self.a,paused=True,goal_terminal=True,
                         verified_source_map=True)
            self.assertEqual(tx.status().state,"active")
        finally:
            fcntl.flock(fd,fcntl.LOCK_UN)
            os.close(fd)
        tx.stage(self.a,paused=True,goal_terminal=True,
                 verified_source_map=True)
        self.assertTrue(tx.must_block_nav())

    def test_two_processes_cannot_both_commit_same_revision(self):
        ZoneUpdateTransaction(self.base,self.old,self.path)
        ctx=multiprocessing.get_context("fork")
        start=ctx.Event()
        ready=ctx.Queue()
        results=ctx.Queue()
        children=[
          ctx.Process(target=_competing_writer,args=(self.base,self.old,p,
                      str(self.path),start,ready,results))
          for p in (self.a,self.b)
        ]
        for process in children:process.start()
        try:
            self.assertEqual(sorted([ready.get(timeout=8) for _ in children]),
                             ["ready","ready"])
            start.set()
            outcomes=sorted(results.get(timeout=10) for _ in children)
            self.assertEqual(outcomes,["committed","stale"])
        finally:
            start.set()
            for process in children:
                process.join(timeout=5)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=2)
        self.assertTrue(all(process.exitcode==0 for process in children))
        self.assertEqual(ZoneUpdateTransaction(self.base,self.old,self.path)
                         .status().state,"paused_recovery")

    def test_private_stable_lockfile_permissions(self):
        ZoneUpdateTransaction(self.base,self.old,self.path)
        lock=self.path.with_name(self.path.name+".lock")
        self.assertTrue(lock.is_file())
        self.assertEqual(lock.stat().st_mode & 0o777,0o600)


if __name__=="__main__":
    unittest.main()
