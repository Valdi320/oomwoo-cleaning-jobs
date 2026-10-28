"""Regression for durable sweep checkpoint serialization and safe admission.

All tests use a temporary Linux filesystem and synthetic grid. No ROS2 or motors.
"""
from __future__ import annotations

import fcntl
import os
from pathlib import Path
import tempfile
import threading
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan
from oomwoo_cleaning_jobs.sweep_campaign import SweepCampaign, SweepStateError


class CampaignSerializationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "campaign.json"
        free = (0,) * 256
        self.grid = Grid(16, 16, free, resolution_m=.1, clearance_cells=1)
        self.raster = Raster(16, 16, .1, -.8, -.8, 0., free)
        pts = tuple(self.raster.centre(i) for i in (50, 65, 170))
        self.plan = adapt_upstream_plan(self.grid, self.raster, pts)
        self.scope = frozenset(self.grid.traversable)
        self.first = self.open()

    def open(self):
        return SweepCampaign(self.grid, self.raster, self.plan, self.path,
                             scope=self.scope, session_id="sweep_stress")

    def test_forced_interleaving_cannot_erase_competing_cancel(self):
        """Before lock: A reads old revision, B cancels, A overwrites cancel."""
        a, b = self.first, self.open()
        entered, release = threading.Event(), threading.Event()
        original_read = a._read
        errors = []

        def delayed_read():
            snapshot = original_read()
            entered.set()
            if not release.wait(timeout=4):
                raise RuntimeError("test timed out")
            return snapshot

        a._read = delayed_read

        def start_a():
            try:
                a.start()
            except Exception as error:
                errors.append(str(error))

        worker = threading.Thread(target=start_a, name="campaign-writer-a")
        worker.start()
        try:
            self.assertTrue(entered.wait(timeout=4), "writer did not read revision")
            with self.assertRaises(SweepStateError):
                b.cancel()
        finally:
            release.set()
            worker.join(timeout=4)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(self.first._read()["revision"], 2)
        self.assertEqual(self.first._read()["state"], "running")
        self.assertEqual(self.open().summary().state, "paused_recovery")

    def test_busy_sidecar_lock_refuses_transition_without_mutation(self):
        lock_path = Path(str(self.path) + ".lock")
        fd = os.open(lock_path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(SweepStateError):
                self.first.start()
        finally:
            os.close(fd)
        self.assertEqual(self.first._read()["state"], "ready")
        self.first.start()
        self.assertEqual(self.first._read()["state"], "running")

    def test_symlinked_checkpoint_is_not_trusted(self):
        other = self.path.with_name("other.json")
        self.path.rename(other)
        self.path.symlink_to(other)
        with self.assertRaises(SweepStateError):
            self.open()
        self.assertEqual(other.exists(), True)

    def test_hardlinked_checkpoint_is_not_trusted(self):
        other = self.path.with_name("other.json")
        os.link(self.path, other)
        with self.assertRaises(SweepStateError):
            self.open()

    def test_fifo_checkpoint_refused_without_blocking(self):
        self.path.unlink()
        os.mkfifo(self.path, 0o600)
        with self.assertRaises(SweepStateError):
            self.open()

    def test_unsafe_lock_symlink_refuses_write(self):
        other = self.path.with_name("lock-target")
        other.write_text("untouched")
        lock_path = Path(str(self.path) + ".lock")
        lock_path.unlink()
        lock_path.symlink_to(other)
        with self.assertRaises(SweepStateError):
            self.first.start()
        self.assertEqual(other.read_text(), "untouched")
        self.assertEqual(self.first._read()["state"], "ready")

    def test_world_readable_checkpoint_refused(self):
        os.chmod(self.path, 0o644)
        with self.assertRaises(SweepStateError):
            self.open()


if __name__ == "__main__":
    unittest.main()
