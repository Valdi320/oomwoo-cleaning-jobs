"""Linux-only coverage WAL ownership, inode and concurrent-writer regressions.

Synthetic filesystem tests; no ROS2, no motor, no battery and no real robot.
"""
import fcntl
import multiprocessing
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.coverage_wal import CoverageWAL, CoverageLedgerError


def _race_stage(path, name, ready, go, barrier, results):
    try:
        grid = Grid(8, 8, (0,) * 64, resolution_m=.1)
        wal = CoverageWAL(Path(path), grid=grid, scope=grid.traversable,
                          producer="lab_meter", job_id="lab")
        original = wal._read
        def synchronize_read():
            item = original()
            try:
                barrier.wait(timeout=.7)
            except Exception:
                pass
            return item
        wal._read = synchronize_read
        ready.put(True)
        if not go.wait(6):
            results.put("no-start")
            return
        try:
            wal.stage(name, (18,), externally_validated=True)
        except CoverageLedgerError:
            results.put("blocked")
        else:
            results.put("accepted")
    except BaseException as exc:
        results.put("error:" + repr(exc))


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux flock/open required")
class CoverageWALWriterSafetyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "coverage.wal"
        self.grid = Grid(8, 8, (0,) * 64, resolution_m=.1)
        self.kw = dict(grid=self.grid, scope=self.grid.traversable,
                       producer="lab_meter", job_id="lab")
        self.wal = CoverageWAL(self.path, **self.kw)

    def test_exclusive_flock_contention_refuses_stage_without_mutation(self):
        lock = self.path.with_name(self.path.name + ".lock")
        fd = os.open(lock, os.O_RDWR | os.O_CREAT, 0o600)
        old = self.path.read_bytes()
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(CoverageLedgerError):
                self.wal.stage("receipt1", (18,), externally_validated=True)
            self.assertEqual(self.path.read_bytes(), old)
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)

    def test_two_processes_do_not_acknowledge_lost_receipts(self):
        ctx = multiprocessing.get_context("fork")
        go, barrier = ctx.Event(), ctx.Barrier(2)
        ready, results = ctx.Queue(), ctx.Queue()
        workers = [ctx.Process(target=_race_stage,
                   args=(str(self.path), e, ready, go, barrier, results))
                   for e in ("receiptA", "receiptB")]
        for proc in workers:
            proc.start()
        try:
            self.assertEqual([ready.get(timeout=8) for _ in workers],
                             [True, True])
            go.set()
            outcomes = sorted(results.get(timeout=8) for _ in workers)
            self.assertEqual(outcomes, ["accepted", "blocked"])
        finally:
            go.set()
            for proc in workers:
                proc.join(timeout=4)
                if proc.is_alive():
                    proc.terminate()
                    proc.join(timeout=3)
        self.assertTrue(all(proc.exitcode == 0 for proc in workers))
        journal = CoverageWAL(self.path, **self.kw)
        self.assertEqual(len(journal.pending()), 1)

    def test_stale_instance_cannot_claim_wal_drained(self):
        older = CoverageWAL(self.path, **self.kw)
        self.wal.stage("receipt1", (18,), externally_validated=True)
        with self.assertRaises(CoverageLedgerError):
            older.require_drained()

    def test_linked_wal_never_accepted_as_own_coverage_ledger(self):
        target = self.path.with_name("unrelated.json")
        self.path.rename(target)
        self.path.symlink_to(target)
        with self.assertRaises(CoverageLedgerError):
            CoverageWAL(self.path, **self.kw)
        self.assertTrue(self.path.is_symlink())
        self.assertTrue(target.is_file())

    def test_dangling_symlink_must_not_initialize_new_wal(self):
        self.path.unlink()
        self.path.symlink_to("absent.json")
        with self.assertRaises(CoverageLedgerError):
            CoverageWAL(self.path, **self.kw)
        self.assertTrue(self.path.is_symlink())

    def test_lock_symlink_refuses_writes(self):
        lock = self.path.with_name(self.path.name + ".lock")
        if lock.exists():
            lock.unlink()
        target = self.path.with_name("unrelated-lock")
        target.write_text("do not touch", encoding="utf-8")
        lock.symlink_to(target)
        old = self.path.read_bytes()
        with self.assertRaises(CoverageLedgerError):
            self.wal.stage("receipt1", (18,), externally_validated=True)
        self.assertEqual(self.path.read_bytes(), old)
        self.assertEqual(target.read_text(), "do not touch")

    @unittest.skipUnless(hasattr(os, "mkfifo"), "needs mkfifo")
    def test_fifo_wal_rejected_without_blocking(self):
        self.path.unlink()
        os.mkfifo(self.path, mode=0o600)
        program = ("from pathlib import Path\n"
                   "from oomwoo_cleaning_jobs import Grid\n"
                   "from oomwoo_cleaning_jobs.coverage_wal import CoverageWAL,CoverageLedgerError\n"
                   "g=Grid(8,8,(0,)*64,resolution_m=.1)\n"
                   "try: CoverageWAL(Path(__import__('sys').argv[1]),grid=g,"
                   "scope=g.traversable,producer='lab_meter',job_id='lab')\n"
                   "except CoverageLedgerError: print('REFUSED')\n"
                   "else: raise AssertionError('FIFO accepted')\n")
        result = subprocess.run([sys.executable, "-c", program, str(self.path)],
                                capture_output=True, text=True, timeout=2.5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "REFUSED")
        self.assertTrue(stat.S_ISFIFO(self.path.lstat().st_mode))


if __name__ == "__main__":
    unittest.main()
