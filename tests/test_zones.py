"""No-go polygon and virtual-wall map rasterization tests (no robot output)."""
from __future__ import annotations
import tempfile
import math
from pathlib import Path
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.zones import ZonePlan, ZoneShape, ZoneError
from oomwoo_cleaning_jobs.rooms import segment_rooms, RoomPlanError


class ZoneTests(unittest.TestCase):
    def setUp(self):
        self.map=Raster(10,10,.1,-.5,-.5,0.,tuple([0]*100))
        self.grid=Grid(10,10,tuple([0]*100),resolution_m=.1)

    def polygon(self):
        return ZoneShape('Sin entrar','polygon',
               ((-.2,-.2),(.2,-.2),(.2,.2),(-.2,.2)))

    def test_polygons_mask_exact_cell_centres(self):
        plan=ZonePlan.create(self.map,self.grid,(self.polygon(),))
        masked=plan.mask(self.map,self.grid)
        selected=[i for i,v in enumerate(masked.data) if v==100]
        self.assertEqual(len(selected),16)
        self.assertIn(5*10+5,selected)
        self.assertNotIn(0,selected)
        self.assertTrue(all(v in (0,100) for v in masked.data))

    def test_zone_changes_traversable_grid_without_touching_raw_map(self):
        plan=ZonePlan.create(self.map,self.grid,(self.polygon(),))
        masked=plan.grid_with_keepouts(self.map,self.grid)
        self.assertTrue(masked.keepout)
        self.assertEqual(len(masked.traversable),84)
        self.assertEqual(self.grid.occupancy,masked.occupancy)
        self.assertEqual(len(self.grid.keepout),0)

    def test_wall_across_map_separates_safe_components(self):
        wall=ZoneShape('Muro virtual','wall',((-.5,0.),(.5,0.)),width_m=.20)
        plan=ZonePlan.create(self.map,self.grid,(wall,))
        g=plan.grid_with_keepouts(self.map,self.grid)
        upper=g.reachable(1)
        self.assertNotIn(9*10+1,upper)
        self.assertTrue(len(upper)<len(g.traversable))

    def test_origin_yaw_world_polygon_points(self):
        rotated=Raster(5,5,.1,1.,-1.,math.pi/2,tuple([0]*25))
        g=Grid(5,5,tuple([0]*25),resolution_m=.1)
        x,y=rotated.centre(12)
        square=ZoneShape('Centro','polygon',
            ((x-.035,y-.035),(x+.035,y-.035),
             (x+.035,y+.035),(x-.035,y+.035)))
        z=ZonePlan.create(rotated,g,(square,))
        self.assertEqual([i for i,v in enumerate(z.mask(rotated,g).data)
                          if v==100],[12])

    def test_overlapping_shapes_are_union(self):
        shape=self.polygon()
        wall=ZoneShape('Línea','wall',((-.4,0.),(.4,0.)),width_m=.08)
        plan=ZonePlan.create(self.map,self.grid,(shape,wall))
        m=plan.mask(self.map,self.grid)
        self.assertGreaterEqual(sum(v==100 for v in m.data),16)
        self.assertEqual(sum(v in (0,100) for v in m.data),100)

    def test_invalid_self_intersection_or_degenerate_polygon(self):
        with self.assertRaises(ZoneError):
            ZoneShape('Bowtie','polygon',((0.,0.),(1.,1.),(0.,1.),(1.,0.)))
        with self.assertRaises(ZoneError):
            ZoneShape('Line','polygon',((0.,0.),(1.,0.),(2.,0.)))

    def test_invalid_names_and_widths(self):
        for name in ('','Bad/name',' Too much ', 'Bad\nLabel'):
            with self.subTest(name=name),self.assertRaises(ZoneError):
                ZoneShape(name,'polygon',((0.,0.),(1.,0.),(1.,1.)))
        for width in (0,-1,3,float('nan')):
            with self.subTest(width=width),self.assertRaises(ZoneError):
                ZoneShape('Wall','wall',((0.,0.),(1.,0.)),width_m=width)
        with self.assertRaises(ZoneError):
            ZoneShape('Wall','wall',((0.,0.),(0.,0.)),width_m=.15)

    def test_duplicate_zone_name_rejected(self):
        with self.assertRaises(ZoneError):
            ZonePlan.create(self.map,self.grid,(self.polygon(),self.polygon()))

    def test_map_origin_change_invalidates_zones(self):
        plan=ZonePlan.create(self.map,self.grid,(self.polygon(),))
        shifted=Raster(10,10,.1,-.4,-.5,0.,self.map.data)
        with self.assertRaises(ZoneError):
            plan.mask(shifted,self.grid)

    def test_occupancy_change_invalidates_zones(self):
        plan=ZonePlan.create(self.map,self.grid,(self.polygon(),))
        cells=[0]*100
        cells[0]=100
        updated_map=Raster(10,10,.1,-.5,-.5,0.,tuple(cells))
        updated_grid=Grid(10,10,tuple(cells),resolution_m=.1)
        with self.assertRaises(ZoneError):
            plan.mask(updated_map,updated_grid)

    def test_zone_mutation_invalidates_saved_room_partition(self):
        earlier=segment_rooms(self.grid)
        altered=ZonePlan.create(self.map,self.grid,(self.polygon(),))
        grid2=altered.grid_with_keepouts(self.map,self.grid)
        with self.assertRaises(RoomPlanError):
            earlier.grid_with_rooms(grid2)

    def test_persistence_roundtrip(self):
        original=ZonePlan.create(self.map,self.grid,(self.polygon(),))
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'zones.json'
            original.save(path,self.map,self.grid)
            loaded=ZonePlan.load(path,self.map,self.grid)
            self.assertEqual(loaded.mask(self.map,self.grid),
                             original.mask(self.map,self.grid))

    def test_stale_plan_cannot_be_overwritten(self):
        plan=ZonePlan.create(self.map,self.grid,(self.polygon(),))
        cells=[0]*100
        cells[1]=100
        new=Raster(10,10,.1,-.5,-.5,0.,tuple(cells))
        g=Grid(10,10,tuple(cells),resolution_m=.1)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'zones.json'
            plan.save(p,self.map,self.grid)
            raw=p.read_bytes()
            with self.assertRaises(ZoneError):
                ZonePlan.create(new,g,()).save(p,new,g)
            self.assertEqual(p.read_bytes(),raw)

    def test_corrupt_persisted_data_fails_closed(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'zones.json'
            p.write_text('not json')
            with self.assertRaises(ZoneError):
                ZonePlan.load(p,self.map,self.grid)


    def test_rotated_mask_bounds_match_exhaustive_cell_sample(self):
        raster=Raster(21,16,.1,2.0,-3.0,.65,tuple([0]*336))
        grid=Grid(21,16,tuple([0]*336),resolution_m=.1)
        cx,cy=raster.centre(7*21+10)
        shapes=(ZoneShape('No go','polygon',
                 ((cx-.15,cy-.2),(cx+.14,cy-.18),
                  (cx+.11,cy+.19),(cx-.17,cy+.2))),
                ZoneShape('Wall','wall',
                 ((cx-.3,cy-.25),(cx+.35,cy+.27)),width_m=.07))
        plan=ZonePlan.create(raster,grid,shapes)
        expected=tuple(100 if any(sh.contains_centre(*raster.centre(i),
                                                      raster.resolution)
                                  for sh in shapes) else 0
                       for i in range(336))
        self.assertEqual(plan.mask(raster,grid).data,expected)

    def test_nonfinite_persisted_origin_rejected(self):
        p=ZonePlan(self.grid.fingerprint(),(float('nan'),-.5,0.),
                   (self.polygon(),))
        with self.assertRaises(ZoneError):
            p.mask(self.map,self.grid)



if __name__=='__main__':
    unittest.main()
