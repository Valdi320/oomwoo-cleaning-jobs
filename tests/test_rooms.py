"""Deterministic, fail-closed saved-map room segmentation regression tests."""
from __future__ import annotations
import json
from pathlib import Path
import tempfile
import unittest
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.rooms import (
    RoomPlan, RoomPlanError, segment_rooms,
)


def two_rooms(keepout=frozenset()):
    w,h=28,14
    cells=[100]*(w*h)
    for y in range(2,12):
        for x in (*range(2,11),*range(17,26)):
            cells[y*w+x]=0
    for x in range(11,17):
        cells[6*w+x]=0
    return Grid(w,h,tuple(cells),keepout=keepout,resolution_m=.05)


class RoomPartitionTests(unittest.TestCase):
    def test_detects_two_rooms_separated_by_doorway(self):
        grid=two_rooms()
        plan=segment_rooms(grid,bottleneck_distance_cells=2,min_seed_cells=10)
        self.assertEqual(len(plan.rooms),2)
        self.assertEqual(set(plan.rooms),{'room_001','room_002'})
        self.assertEqual(sum(map(len,plan.rooms.values())),len(grid.traversable))
        self.assertIn(5*grid.width+5,plan.rooms['room_001'])
        self.assertIn(5*grid.width+22,plan.rooms['room_002'])

    def test_labels_are_deterministic_and_disjoint(self):
        grid=two_rooms()
        first=segment_rooms(grid)
        second=segment_rooms(grid)
        self.assertEqual(dict(first.rooms),dict(second.rooms))
        a=set(first.rooms['room_001'])
        b=set(first.rooms['room_002'])
        self.assertFalse(a&b)
        self.assertEqual(a|b,grid.traversable)

    def test_single_open_room_remains_one(self):
        grid=Grid(15,15,tuple([0]*225))
        plan=segment_rooms(grid)
        self.assertEqual(len(plan.rooms),1)
        self.assertEqual(len(plan.rooms['room_001']),225)

    def test_tiny_room_falls_back_without_dropping_cells(self):
        grid=Grid(2,2,(0,0,0,0))
        plan=segment_rooms(grid)
        self.assertEqual(len(plan.rooms),1)
        self.assertEqual(set(plan.rooms['room_001']),{0,1,2,3})

    def test_disconnected_areas_still_get_separate_labels(self):
        grid=Grid(8,3,(100,)*8+(0,0,100,100,0,0,100,100)+(100,)*8)
        plan=segment_rooms(grid)
        self.assertEqual(len(plan.rooms),2)

    def test_keepout_is_never_assigned(self):
        k=frozenset((5*28+5,))
        grid=two_rooms(keepout=k)
        plan=segment_rooms(grid)
        self.assertFalse(any(k&set(cells) for cells in plan.rooms.values()))
        self.assertEqual(sum(map(len,plan.rooms.values())),len(grid.traversable))

    def test_unknown_cells_remain_unassigned(self):
        grid=Grid(5,5,tuple(0 if x not in (3,4) else -1
                            for y in range(5) for x in range(5)))
        plan=segment_rooms(grid)
        self.assertTrue(all(i not in cells for cells in plan.rooms.values()
                            for i in (3,4,8,9,13,14,18,19,23,24)))

    def test_room_labels_can_drive_existing_cleaning_jobs(self):
        grid=two_rooms()
        plan=segment_rooms(grid)
        labelled=plan.grid_with_rooms(grid)
        selected=labelled.targets(mode='rooms',rooms=('room_001',))
        self.assertEqual(selected,frozenset(plan.rooms['room_001']))
        self.assertFalse(selected & set(plan.rooms['room_002']))

    def test_source_map_fingerprint_mismatch_fails_closed(self):
        grid=two_rooms()
        plan=segment_rooms(grid)
        changed=two_rooms(keepout=frozenset((5*28+5,)))
        with self.assertRaises(RoomPlanError):
            plan.validate_for(changed)
        with self.assertRaises(RoomPlanError):
            plan.grid_with_rooms(changed)

    def test_atomic_json_roundtrip_and_changed_map_refusal(self):
        grid=two_rooms()
        plan=segment_rooms(grid)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'rooms.json'
            plan.save(path,grid)
            loaded=RoomPlan.load(path,grid)
            self.assertEqual(dict(plan.rooms),dict(loaded.rooms))
            self.assertEqual(plan.source_map_digest,loaded.source_map_digest)
            with self.assertRaises(RoomPlanError):
                RoomPlan.load(path,two_rooms(frozenset((5*28+5,))))

    def test_rejects_corrupted_json_without_overwriting(self):
        grid=two_rooms()
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'rooms.json'
            path.write_text('{')
            with self.assertRaises(RoomPlanError):
                RoomPlan.load(path,grid)

    def test_rejects_duplicate_or_overlapping_cells(self):
        grid=two_rooms()
        plan=segment_rooms(grid)
        rooms={k:tuple(v) for k,v in plan.rooms.items()}
        first=next(iter(rooms))
        second=next(k for k in rooms if k!=first)
        extra=rooms[first][0]
        rooms[second]=tuple(sorted(rooms[second]+(extra,)))
        bad=RoomPlan(plan.source_map_digest,rooms,frozenset(),2,8)
        with self.assertRaises(RoomPlanError):
            bad.validate_for(grid)

    def test_rejects_missing_cell_from_partition(self):
        grid=two_rooms()
        plan=segment_rooms(grid)
        rooms=dict(plan.rooms)
        first=next(iter(rooms))
        rooms[first]=rooms[first][1:]
        altered=RoomPlan(plan.source_map_digest,rooms,frozenset(),2,8)
        with self.assertRaises(RoomPlanError):
            altered.validate_for(grid)

    def test_operator_door_seal_must_be_safe(self):
        grid=two_rooms()
        with self.assertRaises(RoomPlanError):
            segment_rooms(grid,door_seals=frozenset((-1,)))
        with self.assertRaises(RoomPlanError):
            segment_rooms(grid,door_seals=frozenset((0,)))

    def test_door_seal_persists(self):
        grid=two_rooms()
        seals=frozenset((6*28+13,))
        plan=segment_rooms(grid,door_seals=seals)
        self.assertEqual(plan.door_seals,seals)
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'room-plan.json'
            plan.save(f,grid)
            reloaded=RoomPlan.load(f,grid)
            self.assertEqual(reloaded.door_seals,seals)

    def test_bad_segmentation_parameters_rejected(self):
        grid=two_rooms()
        for value in (0,-1,False,13):
            with self.subTest(value=value),self.assertRaises(RoomPlanError):
                segment_rooms(grid,bottleneck_distance_cells=value)
        for value in (0,-1,False,6000):
            with self.subTest(value=value),self.assertRaises(RoomPlanError):
                segment_rooms(grid,min_seed_cells=value)


    def test_human_labels_roundtrip_and_selection(self):
        grid=two_rooms()
        plan=segment_rooms(grid).renamed("room_001","Salón")
        plan=plan.renamed("room_002","Cocina")
        self.assertEqual(set(plan.rooms),{"Salón","Cocina"})
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"room_labels.json"
            plan.save(p,grid)
            restored=RoomPlan.load(p,grid)
            labelled=restored.grid_with_rooms(grid)
            self.assertEqual(
                labelled.targets(mode='rooms',rooms=('Salón',)),
                frozenset(plan.rooms['Salón']))

    def test_duplicate_unsafe_and_unknown_labels_refused(self):
        p=segment_rooms(two_rooms())
        for bad in ('','/etc','Kitchen\nOther',' Bad ','X'*61,'room_002'):
            with self.subTest(bad=bad),self.assertRaises(RoomPlanError):
                p.renamed('room_001',bad)
        with self.assertRaises(RoomPlanError):
            p.renamed('not_existing','Cocina')

    def test_existing_room_plan_for_changed_map_cannot_be_overwritten(self):
        a=two_rooms()
        b=two_rooms(frozenset((5*28+5,)))
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'room_plan.json'
            plan=segment_rooms(a)
            plan.save(out,a)
            old=out.read_bytes()
            with self.assertRaises(RoomPlanError):
                segment_rooms(b).save(out,b)
            self.assertEqual(out.read_bytes(),old)



if __name__=='__main__':
    unittest.main()
