"""Exact geometry equivalence and stable route selection after bounded dirty scoring."""
import hashlib
import random
import unittest
from oomwoo_cleaning_jobs.grid import Grid
from oomwoo_cleaning_jobs.sweep_target import _count_nearby_dirty, select_sweep_route

def _bruteforce(route, remaining, grid):
    coords=tuple(grid.xy(i) for i in route)
    return sum(1 for target in remaining
        if min((target%grid.width-x)**2+(target//grid.width-y)**2
               for x,y in coords)<=(0.11/grid.resolution_m)**2)

class NearbyDirtyCounterTests(unittest.TestCase):
    def test_random_equivalence_across_map_resolutions(self):
        rng=random.Random(29037)
        for resolution in (.015,.025,.05,.1,.2,1.0,.001,1e-6):
            grid=Grid(19,13,(0,)*247,resolution_m=resolution)
            for _ in range(30):
                route=tuple(rng.sample(range(247),rng.randint(1,35)))
                remaining=frozenset(rng.sample(range(247),rng.randint(1,120)))
                with self.subTest(resolution=resolution,route=len(route),dirty=len(remaining)):
                    self.assertEqual(_count_nearby_dirty(route,remaining,grid),
                                     _bruteforce(route,remaining,grid))
    def test_no_row_wrap_and_distinct_target_count(self):
        grid=Grid(12,12,(0,)*144,resolution_m=.05)
        route=(11,11,11)
        dirty=frozenset((11,12,10,23))
        self.assertEqual(_count_nearby_dirty(route,dirty,grid),3)
        self.assertEqual(_bruteforce(route,dirty,grid),3)
    def test_empty_route_rejected_and_empty_dirty_is_zero(self):
        grid=Grid(5,5,(0,)*25)
        with self.assertRaises(ValueError):
            _count_nearby_dirty((),frozenset({4}),grid)
        self.assertEqual(_count_nearby_dirty((4,),frozenset(),grid),0)
    def test_selected_route_and_admission_sequence_unchanged(self):
        width,height=120,68
        grid=Grid(width,height,(0,)*(width*height),resolution_m=.05,clearance_cells=1)
        remaining=frozenset(y*width+x for y in range(12,64) for x in range(75,116))
        callbacks=[]
        def admit(route):
            callbacks.append(route[-1])
            return True
        route=select_sweep_route(grid,5+5*width,remaining,admit=admit)
        self.assertEqual(len(callbacks),10)
        self.assertEqual(len(route),92)
        self.assertEqual(hashlib.sha256(",".join(map(str,route)).encode()).hexdigest(),
                         "7cd6403e5112d92f176013767892be660da8dc5d8ab684e4c299a41066f93184")
        self.assertTrue(set(route)<=grid.traversable)

if __name__=="__main__":
    unittest.main()
