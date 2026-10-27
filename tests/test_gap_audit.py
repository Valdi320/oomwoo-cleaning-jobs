"""Residual coverage audit including inaccessible and dynamically banned repairs."""
import unittest
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.gap_audit import GapAuditor,GapAuditError


def two_rooms_grid():
    width,height=32,16
    occupancy=[100]*(width*height)
    for row in range(2,14):
        for col in (*range(2,13),*range(19,30)):
            occupancy[row*width+col]=0
    for col in range(13,19):occupancy[8*width+col]=0
    return Grid(width,height,tuple(occupancy),resolution_m=.05)


class GapAuditTests(unittest.TestCase):
    def setUp(self):
        self.grid=two_rooms_grid()
        self.first=4*32+5
        self.second=4*32+25
        self.scope=self.grid.traversable
        self.a=GapAuditor(self.grid,self.scope,unresolved_upstream=(77,78))

    def test_all_external_cells_complete_despite_unadmitted_waypoints(self):
        report=self.a.audit(credited=self.scope,pose_cell=self.first)
        self.assertTrue(report.verified_complete)
        self.assertEqual(report.missing_cells,0)
        self.assertEqual(report.unresolved_upstream_waypoints,(77,78))

    def test_missing_cells_are_never_claimed_as_clean(self):
        credited=frozenset(i for i in self.scope if i not in
                            (self.first,self.second))
        report=self.a.audit(credited=credited,pose_cell=self.first)
        self.assertEqual(report.credited_cells,len(credited))
        self.assertEqual(report.missing_cells,2)
        self.assertFalse(report.verified_complete)
        self.assertEqual(len(report.reachable_gaps),2)

    def test_missing_adjacent_cells_form_one_gap(self):
        remain=frozenset((4*32+6,4*32+7,4*32+8))
        report=self.a.audit(credited=self.scope-remain,pose_cell=self.first)
        self.assertEqual(len(report.reachable_gaps),1)
        self.assertEqual(report.reachable_gaps[0].size,3)

    def test_altered_keepout_blocks_one_room_and_reports_unreachable(self):
        blocked=Grid(self.grid.width,self.grid.height,self.grid.occupancy,
                      keepout=frozenset([8*32+15]),resolution_m=.05)
        scope=frozenset(blocked.traversable)
        a=GapAuditor(blocked,scope)
        credited=frozenset(i for i in scope if i!=self.second)
        report=a.audit(credited=credited,pose_cell=self.first)
        self.assertTrue(report.require_manual_review)
        self.assertEqual(len(report.unreachable_gaps),1)
        self.assertEqual(report.unreachable_gaps[0].cells,(self.second,))

    def test_gap_keys_stable_and_sizes_auditable(self):
        cells=self.scope-frozenset((self.first,self.second))
        one=self.a.audit(credited=cells,pose_cell=self.first)
        two=self.a.audit(credited=cells,pose_cell=self.first)
        self.assertEqual(one,two)
        self.assertTrue(all(len(g.key)==12 for g in one.reachable_gaps))

    def test_repair_route_needs_live_admission(self):
        missed=frozenset((4*32+9,4*32+10))
        route=self.a.propose_repair(credited=self.scope-missed,
                                    pose_cell=self.first,admit_route=lambda r:True)
        self.assertTrue(route)
        self.assertEqual(route[0],self.first)
        self.assertTrue(all(i in self.scope for i in route))

    def test_repair_rejected_if_costmap_refuses(self):
        missed=frozenset((4*32+9,))
        with self.assertRaises(GapAuditError):
            self.a.propose_repair(credited=self.scope-missed,pose_cell=self.first,
                                   admit_route=lambda r:False)

    def test_missing_gap_when_none_only_no_repair(self):
        with self.assertRaises(GapAuditError):
            self.a.propose_repair(credited=self.scope,pose_cell=self.first,
                                  admit_route=lambda r:True)

    def test_outside_credited_scope_refused(self):
        scope=frozenset((self.first,))
        a=GapAuditor(self.grid,scope)
        with self.assertRaises(GapAuditError):
            a.audit(credited=frozenset((self.second,)),pose_cell=self.first)

    def test_invalid_pose_refused(self):
        with self.assertRaises(GapAuditError):
            self.a.audit(credited=frozenset(),pose_cell=-10)

    def test_invalid_gap_scope_refused(self):
        for scope in (frozenset(),frozenset((-1,)),[self.first]):
            with self.subTest(scope=scope),self.assertRaises(GapAuditError):
                GapAuditor(self.grid,scope)

    def test_status_has_honest_count(self):
        report=self.a.audit(credited=frozenset((self.first,)),
                            pose_cell=self.first)
        status=report.summary()
        self.assertEqual(status["missing_cells"],len(self.scope)-1)
        self.assertTrue(status["manual_review"])
        self.assertFalse(status["verified_complete"])

if __name__=="__main__":
    unittest.main()
