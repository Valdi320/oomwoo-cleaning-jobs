"""Transactional Nav2 keepout hot-edit acceptance and negative fault tests."""
from pathlib import Path
import tempfile
import unittest
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.zone_revision import ZoneUpdateTransaction,ZoneUpdateError


class TransactionalZoneEditTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        base=(0,)*20*20
        self.map=Raster(20,20,.1,-1.,-1.,0.,base)
        a=list(base);b=list(base)
        for j in (9*20+9,9*20+10,10*20+9,10*20+10):a[j]=100
        for j in (11*20+11,11*20+12,12*20+11,12*20+12):b[j]=100
        self.old=Raster(20,20,.1,-1.,-1.,0.,tuple(a))
        self.new=Raster(20,20,.1,-1.,-1.,0.,tuple(b))
        self.file=Path(self.tmp.name)/"zone-rev.json"
        self.tx=ZoneUpdateTransaction(self.map,self.old,self.file)
        self._sequence=0

    def stage(self):
        self.tx.stage(self.new,paused=True,goal_terminal=True,
                      verified_source_map=True)

    def verify(self,costmap=None,local=None):
        self._sequence+=1
        self.tx.verify_sample(
           actual_mask=self.new,global_costmap=costmap or self.new,
           local_costmap=local or self.new,filter_info_active=True,
           status_fresh=True,job_paused=True,
           global_sequence=self._sequence,local_sequence=self._sequence)

    def test_two_independent_costmap_proofs_before_commit(self):
        self.stage()
        self.assertTrue(self.tx.must_block_nav())
        self.verify()
        self.assertEqual(self.tx.status().confirmed_samples,1)
        with self.assertRaises(ZoneUpdateError):
            self.tx.commit(nav2_idle=True,fresh_pose=True,job_paused=True)
        self.verify()
        self.assertEqual(self.tx.status().state,"verified")
        self.tx.commit(nav2_idle=True,fresh_pose=True,job_paused=True)
        self.assertFalse(self.tx.must_block_nav())
        self.assertEqual(self.tx.current_mask,self.new)

    def test_duplicate_costmap_frame_cannot_count_as_two_proofs(self):
        self.stage()
        self.verify()
        self.assertEqual(self.tx.status().confirmed_samples,1)
        with self.assertRaisesRegex(ZoneUpdateError,"stale/replayed"):
            self.tx.verify_sample(actual_mask=self.new,
               global_costmap=self.new,local_costmap=self.new,
               filter_info_active=True,status_fresh=True,job_paused=True,
               global_sequence=1,local_sequence=1)
        self.assertEqual(self.tx.status().confirmed_samples,1)
        self.verify()
        self.assertEqual(self.tx.status().state,"verified")

    def test_new_global_but_stale_local_frame_refused(self):
        self.stage()
        self.verify()
        with self.assertRaisesRegex(ZoneUpdateError,"stale/replayed"):
            self.tx.verify_sample(actual_mask=self.new,
               global_costmap=self.new,local_costmap=self.new,
               filter_info_active=True,status_fresh=True,job_paused=True,
               global_sequence=2,local_sequence=1)
        self.assertEqual(self.tx.status().confirmed_samples,1)

    def test_live_mask_not_published_denies_activation(self):
        self.stage()
        with self.assertRaises(ZoneUpdateError):
            self.tx.verify_sample(actual_mask=self.old,
                global_costmap=self.new,local_costmap=self.new,
                filter_info_active=True,status_fresh=True,job_paused=True,
                global_sequence=1,local_sequence=1)

    def test_global_post_filter_costmap_must_be_lethal(self):
        self.stage()
        with self.assertRaises(ZoneUpdateError):
            self.verify(costmap=self.map)
        self.assertTrue(self.tx.must_block_nav())

    def test_local_rolling_costmap_must_be_lethal_in_observed_overlap(self):
        self.stage()
        with self.assertRaises(ZoneUpdateError):
            self.verify(local=self.map)
        self.assertTrue(self.tx.must_block_nav())

    def test_filter_info_server_loss_denies_activation(self):
        self.stage()
        with self.assertRaises(ZoneUpdateError):
            self.tx.verify_sample(actual_mask=self.new,
                global_costmap=self.new,local_costmap=self.new,
                filter_info_active=False,status_fresh=True,job_paused=True,
                global_sequence=1,local_sequence=1)

    def test_no_live_status_denies_activation(self):
        self.stage()
        with self.assertRaises(ZoneUpdateError):
            self.tx.verify_sample(actual_mask=self.new,
                global_costmap=self.new,local_costmap=self.new,
                filter_info_active=True,status_fresh=False,job_paused=True,
                global_sequence=1,local_sequence=1)

    def test_mask_changes_forbidden_while_nav_running(self):
        with self.assertRaises(ZoneUpdateError):
            self.tx.stage(self.new,paused=False,goal_terminal=True,
                verified_source_map=True)
        with self.assertRaises(ZoneUpdateError):
            self.tx.stage(self.new,paused=True,goal_terminal=False,
                verified_source_map=True)
        self.assertFalse(self.tx.must_block_nav())

    def test_reboot_during_edit_never_authorizes_nav(self):
        self.stage()
        resumed=ZoneUpdateTransaction(self.map,self.old,self.file)
        self.assertEqual(resumed.status().state,"paused_recovery")
        self.assertTrue(resumed.must_block_nav())
        with self.assertRaises(ZoneUpdateError):
            resumed.commit(nav2_idle=True,fresh_pose=True,job_paused=True)

    def test_wrong_map_origin_rejected(self):
        shifted=Raster(20,20,.1,-.8,-1.,0.,self.new.data)
        with self.assertRaises(ZoneUpdateError):
            self.tx.stage(shifted,paused=True,goal_terminal=True,
                          verified_source_map=True)

    def test_stale_writer_cannot_commit_after_file_modified(self):
        parallel=ZoneUpdateTransaction(self.map,self.old,self.file)
        self.stage()
        with self.assertRaises(ZoneUpdateError):
            parallel.stage(self.new,paused=True,goal_terminal=True,
                           verified_source_map=True)

    def test_identical_edit_is_not_new_version(self):
        with self.assertRaises(ZoneUpdateError):
            self.tx.stage(self.old,paused=True,goal_terminal=True,
                          verified_source_map=True)

    def test_resume_requires_fresh_pose_and_backend_idle(self):
        self.stage();self.verify();self.verify()
        with self.assertRaises(ZoneUpdateError):
            self.tx.commit(nav2_idle=False,fresh_pose=True,job_paused=True)
        with self.assertRaises(ZoneUpdateError):
            self.tx.commit(nav2_idle=True,fresh_pose=False,job_paused=True)

    def test_corrupt_transaction_is_quarantined(self):
        self.file.write_text('{"schema":1')
        with self.assertRaises(ZoneUpdateError):
            ZoneUpdateTransaction(self.map,self.old,self.file)

    def test_local_costmap_finer_centre_only_trick_rejected(self):
        self.stage()
        pixels = [0] * (40 * 40)
        for index, value in enumerate(self.new.data):
            if value == 100:
                x, y = self.new.centre(index)
                col = int((x + 1.) / .05)
                row = int((y + 1.) / .05)
                pixels[row * 40 + col] = 100
        fine = Raster(40, 40, .05, -1., -1., 0., tuple(pixels))
        # Legacy rule checked only these lethal centre samples, admitting
        # free 5 cm subcells in each 10 cm forbidden mask square.
        for i, value in enumerate(self.new.data):
            if value == 100:
                self.assertEqual(fine.data[fine.cell_at(*self.new.centre(i))], 100)
        with self.assertRaisesRegex(ZoneUpdateError, "local"):
            self.verify(local=fine)
        self.assertTrue(self.tx.must_block_nav())

    def test_local_costmap_finer_full_square_proofs_allow_two_samples(self):
        self.stage()
        pixels = [0] * (40 * 40)
        for i, value in enumerate(self.new.data):
            if value != 100:
                continue
            col = i % self.new.width
            row = i // self.new.width
            for dy in (0, 1):
                for dx in (0, 1):
                    pixels[(row * 2 + dy) * 40 + col * 2 + dx] = 100
        fine = Raster(40, 40, .05, -1., -1., 0., tuple(pixels))
        self.verify(local=fine)
        self.verify(local=fine)
        self.assertEqual(self.tx.status().state, "verified")
        self.tx.commit(nav2_idle=True, fresh_pose=True, job_paused=True)
        self.assertFalse(self.tx.must_block_nav())

if __name__=="__main__":
    unittest.main()
