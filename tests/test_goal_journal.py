"""Durable UUID-before-send journal regression, no ROS2 or hardware required."""
import tempfile
from pathlib import Path
import unittest

from oomwoo_cleaning_jobs.goal_journal import GoalJournal,GoalJournalError
from oomwoo_cleaning_jobs.restart_witness import OrphanNav2Witness


A="a1"*16
B="b2"*16
TOKEN="d3"*16+"-1"
TOKEN2="d4"*16+"-2"
DIGEST="e5"*32


class NavGoalJournalTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/"journal.json"
        self.j=GoalJournal(self.path)

    def start(self,*,hexid=A,token=TOKEN):
        return self.j.begin(uuid_hex=hexid,token=token,job_id="demo",
                            map_digest=DIGEST,target_cell=11)

    def test_unknown_goal_has_no_orphan(self):
        self.assertIsNone(self.j.read())
        self.assertIsNone(self.j.orphaned_uuid())

    def test_uuid_persisted_before_acceptance(self):
        v=self.start()
        self.assertEqual(v["state"],"submitting")
        self.assertTrue(self.path.is_file())
        j2=GoalJournal(self.path)
        self.assertEqual(j2.orphaned_uuid(),A)
        self.assertEqual(j2.read()["token"],TOKEN)

    def test_second_goal_blocked_until_terminal(self):
        self.start()
        with self.assertRaises(GoalJournalError):
            self.start(hexid=B,token=TOKEN2)

    def test_accepted_but_process_crash_keeps_uuid(self):
        self.start()
        self.j.accepted(token=TOKEN,uuid_hex=A)
        self.assertEqual(GoalJournal(self.path).read()["state"],"accepted")
        self.assertEqual(GoalJournal(self.path).orphaned_uuid(),A)

    def test_terminal_frees_next_goal(self):
        self.start()
        self.j.accepted(token=TOKEN,uuid_hex=A)
        self.j.terminal(token=TOKEN,uuid_hex=A,outcome="canceled")
        self.assertIsNone(self.j.orphaned_uuid())
        other=self.start(hexid=B,token=TOKEN2)
        self.assertEqual(other["uuid"],B)
        self.assertGreater(other["revision"],1)

    def test_outcome_not_guessed_from_cancel_request(self):
        self.start()
        with self.assertRaises(GoalJournalError):
            self.j.terminal(token=TOKEN,uuid_hex=A,outcome="cancel_request_sent")
        self.assertEqual(self.j.orphaned_uuid(),A)

    def test_wrong_uuid_or_token_never_mutates(self):
        self.start()
        before=self.path.read_bytes()
        with self.assertRaises(GoalJournalError):
            self.j.accepted(token=TOKEN,uuid_hex=B)
        with self.assertRaises(GoalJournalError):
            self.j.terminal(token=TOKEN2,uuid_hex=A,outcome="canceled")
        self.assertEqual(before,self.path.read_bytes())

    def test_duplicate_matching_terminal_is_idempotent(self):
        self.start()
        self.j.terminal(token=TOKEN,uuid_hex=A,outcome="rejected")
        before=self.path.read_bytes()
        self.j.terminal(token=TOKEN,uuid_hex=A,outcome="rejected")
        self.assertEqual(before,self.path.read_bytes())

    def test_conflicting_terminal_refused(self):
        self.start()
        self.j.terminal(token=TOKEN,uuid_hex=A,outcome="rejected")
        with self.assertRaises(GoalJournalError):
            self.j.terminal(token=TOKEN,uuid_hex=A,outcome="succeeded")

    def test_corrupt_json_disarms(self):
        self.path.write_text("{")
        with self.assertRaises(GoalJournalError):
            self.j.read()
        with self.assertRaises(GoalJournalError):
            GoalJournal(self.path)

    def test_non_object_or_invalid_schema_disarms(self):
        for bad in ("[]",'{"schema": 1}', '{"schema": 2}', '"hello"'):
            with self.subTest(bad=bad):
                self.path.write_text(bad)
                with self.assertRaises(GoalJournalError):
                    GoalJournal(self.path)

    def test_invalid_uuid_token_digest_index_rejected_before_write(self):
        specs=[
            {"uuid_hex":"invalid"},{"token":"wrong"},{"map_digest":"tiny"},
            {"target_cell":-1},{"target_cell":True},{"uuid_hex":"AA"*16}
        ]
        for data in specs:
            with self.subTest(data=data):
                args={"uuid_hex":A,"token":TOKEN,"job_id":"demo",
                      "map_digest":DIGEST,"target_cell":11}
                args.update(data)
                with self.assertRaises(GoalJournalError):
                    self.j.begin(**args)
                self.assertFalse(self.path.exists())

    def test_cannot_mark_reconciled_from_external_json(self):
        self.start()
        with self.assertRaises(GoalJournalError):
            self.j.mark_reconciled_canceled({
                "known_old_goal_uuid":A,
                "terminal_canceled":True,
                "verified_disarmed_stop":True,
            })
        self.assertEqual(self.j.orphaned_uuid(),A)

    def test_witness_must_be_ready_and_match_journal(self):
        self.start()
        unconfirmed=OrphanNav2Witness(A)
        unconfirmed.see_status(A,2)
        unconfirmed.cancel_response(0,(A,))
        with self.assertRaises(Exception):
            self.j.mark_reconciled_canceled(unconfirmed)
        self.assertEqual(self.j.orphaned_uuid(),A)
        wrong=OrphanNav2Witness(B)
        wrong.see_status(B,2)
        wrong.cancel_response(0,(B,))
        wrong.see_status(B,5)
        for k in range(12):
            wrong.observe(sim_time=10+k*.15,x=0.,y=0.,
                linear_mps=0.,angular_rps=0.)
        with self.assertRaises(GoalJournalError):
            self.j.mark_reconciled_canceled(wrong)
        self.assertEqual(self.j.orphaned_uuid(),A)

    def test_verified_witness_ends_unresolved_uuid(self):
        self.start()
        confirmed=OrphanNav2Witness(A)
        confirmed.see_status(A,2)
        confirmed.cancel_response(0,(A,))
        confirmed.see_status(A,5)
        for k in range(12):
            confirmed.observe(sim_time=10+k*.15,x=0.,y=0.,
                linear_mps=0.,angular_rps=0.)
        self.j.mark_reconciled_canceled(confirmed)
        self.assertIsNone(self.j.orphaned_uuid())
        self.assertEqual(self.j.read()["outcome"],"canceled")


if __name__=="__main__":
    unittest.main()
