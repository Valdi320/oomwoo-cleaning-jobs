"""Host-only negative controls for map-frame OccupancyGrid origin rotation.

No live ROS2, Nav2, Gazebo or real hardware.
"""
import math
import unittest
from types import SimpleNamespace as N
from oomwoo_cleaning_jobs.map_admission import AdmissionError, Raster
from oomwoo_cleaning_jobs.costmap_frame import CostmapFrameError, rolling_costmap_in_map


def grid(q, *, frame="map"):
    return N(
        info=N(width=20, height=20, resolution=.1,
               origin=N(position=N(x=-1., y=-1., z=0.), orientation=q)),
        data=[0]*400,
        header=N(frame_id=frame, stamp=N(sec=20, nanosec=0)),
    )


def planar_quaternion(yaw):
    return N(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))


class RosGridOriginOrientationTests(unittest.TestCase):
    def test_zero_quaternion_cannot_be_an_identity_fallback(self):
        with self.assertRaisesRegex(AdmissionError, "quaternion"):
            Raster.from_ros(grid(N(x=0., y=0., z=0., w=0.)))

    def test_nonunit_scaled_quaternion_rejected(self):
        with self.assertRaisesRegex(AdmissionError, "non-unit"):
            Raster.from_ros(grid(N(x=0., y=0., z=.3, w=.3)))

    def test_normalized_but_nonplanar_quaternion_rejected(self):
        with self.assertRaisesRegex(AdmissionError, "non-planar"):
            Raster.from_ros(grid(N(x=.15, y=0., z=0.,
                                   w=math.sqrt(1.-.15**2))))

    def test_nonfinite_quaternions_rejected(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value), self.assertRaises(AdmissionError):
                Raster.from_ros(grid(N(x=0., y=0., z=value, w=1.)))

    def test_bool_quaternion_component_rejected(self):
        with self.assertRaises(AdmissionError):
            Raster.from_ros(grid(N(x=False, y=0., z=0., w=1.)))

    def test_missing_quaternion_component_rejected(self):
        with self.assertRaises(AdmissionError):
            Raster.from_ros(grid(N(x=0., z=0., w=1.)))

    def test_valid_identity_and_sign_equivalent_quaternions(self):
        for w in (-1., 1.):
            with self.subTest(w=w):
                r=Raster.from_ros(grid(N(x=0., y=0., z=0., w=w)))
                self.assertEqual(r.yaw, 0.)
                self.assertEqual(r.cell_at(*r.centre(210)), 210)

    def test_valid_rotated_maps_preserve_cell_coordinates(self):
        for yaw in (-math.pi, -math.pi/2, -.3, 0., .3,
                    math.pi/2, math.pi):
            with self.subTest(yaw=yaw):
                r=Raster.from_ros(grid(planar_quaternion(yaw)))
                self.assertAlmostEqual(math.sin(r.yaw), math.sin(yaw))
                self.assertAlmostEqual(math.cos(r.yaw), math.cos(yaw))
                for cell in (0, 19, 210, 399):
                    self.assertEqual(r.cell_at(*r.centre(cell)), cell)

    def test_map_frame_live_costmap_refuses_invalid_orientation(self):
        malformed=grid(N(x=0., y=0., z=0., w=0.))
        with self.assertRaisesRegex(CostmapFrameError,
                                    "invalid map-frame Nav2 costmap"):
            rolling_costmap_in_map(malformed, sim_now_s=20.)

    def test_odom_grid_still_disallowed_in_map_raster(self):
        with self.assertRaises(AdmissionError):
            Raster.from_ros(grid(planar_quaternion(0.), frame="odom"))


if __name__=="__main__":
    unittest.main()
