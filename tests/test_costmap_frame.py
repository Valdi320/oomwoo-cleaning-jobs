"""Time-fenced map<-odom transform tests for local rolling Nav2 costmaps."""
import math
from types import SimpleNamespace as N
import unittest
from oomwoo_cleaning_jobs.costmap_frame import rolling_costmap_in_map,CostmapFrameError

def stamp(t):
    sec=int(t)
    return N(sec=sec,nanosec=int(round((t-sec)*1_000_000_000)))

def quat(yaw):
    return N(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))

def grid(*,frame="odom",time=20.,origin=(0.,0.),angle=0.,w=10,h=10,values=None):
    p=N(x=origin[0],y=origin[1],z=0.)
    metadata=N(width=w,height=h,resolution=.10,
      origin=N(position=p,orientation=quat(angle)))
    return N(info=metadata,header=N(frame_id=frame,stamp=stamp(time)),
             data=[0]*(w*h) if values is None else values)

def frame(*,src="odom",dst="map",time=20.,shift=(0.,0.),yaw=0.):
    return N(header=N(frame_id=dst,stamp=stamp(time)),
             child_frame_id=src,
             transform=N(translation=N(x=shift[0],y=shift[1],z=0.),
                         rotation=quat(yaw)))

class LocalCostmapFrameTests(unittest.TestCase):
    def test_identity_frames_convert_local_into_map_coordinates(self):
        r=rolling_costmap_in_map(grid(),map_from_odom=frame(),sim_now_s=20.)
        self.assertEqual((r.origin_x,r.origin_y,r.yaw),(0.,0.,0.))
        self.assertEqual(r.cell_at(.05,.05),0)

    def test_translation_matches_exact_map_cell_world_coordinate(self):
        r=rolling_costmap_in_map(grid(origin=(-.5,-.5)),
                  map_from_odom=frame(shift=(1.,2.)),sim_now_s=20.)
        self.assertAlmostEqual(r.origin_x,.5)
        self.assertAlmostEqual(r.origin_y,1.5)
        self.assertEqual(r.cell_at(.55,1.55),0)

    def test_transform_yaw_composes_with_grid_yaw(self):
        q=math.pi/2
        r=rolling_costmap_in_map(grid(origin=(1.,0.),angle=.2),
            map_from_odom=frame(shift=(.4,.7),yaw=q),sim_now_s=20.)
        self.assertAlmostEqual(r.origin_x,.4,places=5)
        self.assertAlmostEqual(r.origin_y,1.7,places=5)
        self.assertAlmostEqual(r.yaw,q+.2,places=5)
        x,y=r.centre(15)
        self.assertEqual(r.cell_at(x,y),15)

    def test_map_frame_costmap_needs_no_odometry_tf(self):
        r=rolling_costmap_in_map(grid(frame="map"),sim_now_s=20.)
        self.assertEqual(r.frame,"map")

    def test_odom_frame_with_missing_tf_refused(self):
        with self.assertRaises(CostmapFrameError):
            rolling_costmap_in_map(grid(),sim_now_s=20.)

    def test_tf_direction_reversal_refused(self):
        with self.assertRaises(CostmapFrameError):
            rolling_costmap_in_map(grid(),map_from_odom=frame(src="map",dst="odom"),
                                   sim_now_s=20.)

    def test_stale_local_costmap_refused(self):
        with self.assertRaisesRegex(CostmapFrameError,"costmap stale"):
            rolling_costmap_in_map(grid(time=10.),map_from_odom=frame(),
                                   sim_now_s=20.)

    def test_stale_tf_refused(self):
        with self.assertRaisesRegex(CostmapFrameError,"TF stale"):
            rolling_costmap_in_map(grid(),map_from_odom=frame(time=8.),
                                   sim_now_s=20.)

    def test_nearly_four_degrees_of_roll_refused(self):
        t=frame()
        t.transform.rotation.x=.15
        with self.assertRaises(CostmapFrameError):
            rolling_costmap_in_map(grid(),map_from_odom=t,sim_now_s=20.)

    def test_unknown_frame_fails_closed(self):
        with self.assertRaises(CostmapFrameError):
            rolling_costmap_in_map(grid(frame="laser"),map_from_odom=frame(),
                                   sim_now_s=20.)

    def test_bounded_amcl_future_tolerance_accepted(self):
        r=rolling_costmap_in_map(grid(),map_from_odom=frame(time=25.),
                                 sim_now_s=20.)
        self.assertEqual(r.cell_at(.05,.05),0)

    def test_excessive_future_transform_rejected(self):
        with self.assertRaisesRegex(CostmapFrameError,"stale/future"):
            rolling_costmap_in_map(grid(),map_from_odom=frame(time=27.),
                                   sim_now_s=20.)

    def test_nonfinite_tf_refused(self):
        t=frame()
        t.transform.translation.x=float("nan")
        with self.assertRaises(CostmapFrameError):
            rolling_costmap_in_map(grid(),map_from_odom=t,sim_now_s=20.)

    def test_empty_rolling_grid_refused(self):
        with self.assertRaises(CostmapFrameError):
            rolling_costmap_in_map(grid(w=0,h=0),map_from_odom=frame(),
                                   sim_now_s=20.)

if __name__=="__main__":
    unittest.main()
