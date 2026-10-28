"""Host-only differential and negative controls for costmap admission fast path.

All tests use synthetic costmaps, NOT ROS2 messages, Gazebo or hardware.
"""
import math
import random
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from oomwoo_cleaning_jobs import swept_footprint as geom


def raster(width=120, height=120, resolution=.02, yaw=0., points=()):
    co,si=math.cos(yaw),math.sin(yaw)
    origin_x=-.5*(co*width-si*height)*resolution
    origin_y=-.5*(si*width+co*height)*resolution
    data=[0]*(width*height)
    for x,y,cost in points:
        local_x=co*(x-origin_x)+si*(y-origin_y)
        local_y=-si*(x-origin_x)+co*(y-origin_y)
        col=math.floor(local_x/resolution)
        row=math.floor(local_y/resolution)
        assert 0 <= col < width and 0 <= row < height
        data[row*width+col]=cost
    return SimpleNamespace(width=width,height=height,resolution=resolution,
                           origin_x=origin_x,origin_y=origin_y,yaw=yaw,
                           data=tuple(data))


def reference_clear(grid,a,b,radius,max_cost):
    """Old all-pixel geometry ordering, kept independent for comparison."""
    co,si=math.cos(grid.yaw),math.sin(grid.yaw)
    def local(p):
        dx,dy=p[0]-grid.origin_x,p[1]-grid.origin_y
        return co*dx+si*dy,-si*dx+co*dy
    ax,ay=local(a);bx,by=local(b)
    if any(not math.isfinite(x) for x in (ax,ay,bx,by,radius)):
        return False
    res=grid.resolution
    x0=math.floor((min(ax,bx)-radius)/res)
    x1=math.floor((max(ax,bx)+radius)/res)
    y0=math.floor((min(ay,by)-radius)/res)
    y1=math.floor((max(ay,by)+radius)/res)
    if x0<0 or y0<0 or x1>=grid.width or y1>=grid.height:
        return False
    radius2=radius*radius
    for row in range(y0,y1+1):
        bottom=row*res
        for col in range(x0,x1+1):
            left=col*res
            if geom._segment_box2(ax,ay,bx,by,left,bottom,left+res,bottom+res)<=radius2+1e-12:
                cost=grid.data[row*grid.width+col]
                if cost<0 or cost>max_cost:
                    return False
    return True


class SweptFastPathTests(unittest.TestCase):
    def test_all_safe_map_skips_expensive_geometry(self):
        grid=raster()
        with patch.object(geom,"_segment_box2",side_effect=AssertionError(
                "known safe pixels must skip geometric distance")):
            self.assertTrue(geom.swept_disk_clear(grid,(-.3,0.),(.4,0.),.21,25))

    def test_unknown_and_high_cost_inside_are_still_blocked(self):
        for cost in (-1,26,100):
            with self.subTest(cost=cost):
                grid=raster(points=((0.,0.,cost),))
                self.assertFalse(geom.swept_disk_clear(
                    grid,(-.3,0.),(.4,0.),.21,25))

    def test_at_threshold_is_safe(self):
        grid=raster(points=((0.,0.,25),))
        self.assertTrue(geom.swept_disk_clear(grid,(-.3,0.),(.4,0.),.21,25))

    def test_hazard_in_bounding_box_outside_capsule_does_not_block(self):
        # Diagonal capsule bbox includes these hazard pixels, but robot does
        # not touch them. We must still compute true geometry before rejecting.
        a=(-.2,-.2);b=(.2,.2)
        grid=raster(points=((.39,-.39,100),))
        self.assertTrue(reference_clear(grid,a,b,.21,25))
        self.assertTrue(geom.swept_disk_clear(grid,a,b,.21,25))

    def test_outer_costmap_edge_still_fails_closed(self):
        grid=raster(width=45,height=45,resolution=.02)
        self.assertFalse(geom.swept_disk_clear(grid,(-.35,-.35),(.35,.35),.21,25))

    def test_nonfinite_pose_fails_closed(self):
        grid=raster()
        for v in (float("nan"),float("inf"),-float("inf")):
            self.assertFalse(geom.swept_disk_clear(grid,(v,0.),(.2,.2),.21,25))

    def test_seeded_differential_clear_sparse_and_rotated(self):
        rng=random.Random(20261028)
        for case in range(120):
            yaw=(0.,.3,-.7,math.pi/2)[case%4]
            points=[]
            for i in range(case%5):
                points.append((rng.uniform(-.45,.45),rng.uniform(-.45,.45),
                               rng.choice((-1,20,25,26,100))))
            grid=raster(width=180,height=180,resolution=.02,yaw=yaw,
                        points=points)
            a=(rng.uniform(-.25,.25),rng.uniform(-.25,.25))
            b=(rng.uniform(-.25,.25),rng.uniform(-.25,.25))
            radius=rng.choice((.05,.1,.21,.3))
            self.assertEqual(reference_clear(grid,a,b,radius,25),
                             geom.swept_disk_clear(grid,a,b,radius,25),
                             f"differential failed at seeded case {case}")

if __name__=="__main__":
    unittest.main()
