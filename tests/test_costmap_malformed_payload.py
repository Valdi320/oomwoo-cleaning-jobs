"""Pure Python duck-typed ROS message error-boundary tests. No hardware."""
import unittest
from types import SimpleNamespace as N
from oomwoo_cleaning_jobs.costmap_frame import rolling_costmap_in_map,CostmapFrameError

def grid(frame="map"):
    return N(header=N(frame_id=frame,stamp=N(sec=20,nanosec=0)),
             info=N(width=10,height=10,resolution=.1,
                    origin=N(position=N(x=0.,y=0.,z=0.),
                             orientation=N(x=0.,y=0.,z=0.,w=1.))),
             data=[0]*100)
def tf():
    return N(header=N(frame_id="map",stamp=N(sec=20,nanosec=0)),
             child_frame_id="odom",
             transform=N(translation=N(x=0.,y=0.,z=0.),
                         rotation=N(x=0.,y=0.,z=0.,w=1.)))
class MalformedBoundary(unittest.TestCase):
    def reject(self,g,t=None):
        with self.assertRaisesRegex(CostmapFrameError,"malformed"):
            rolling_costmap_in_map(g,map_from_odom=t,sim_now_s=20.)
    def test_missing_header(self):
        g=grid();del g.header;self.reject(g)
    def test_missing_stamp(self):
        g=grid();del g.header.stamp;self.reject(g)
    def test_missing_nanosec(self):
        g=grid();del g.header.stamp.nanosec;self.reject(g)
    def test_missing_frame_id(self):
        g=grid();del g.header.frame_id;self.reject(g)
    def test_missing_info(self):
        g=grid();del g.info;self.reject(g)
    def test_missing_origin(self):
        g=grid();del g.info.origin;self.reject(g)
    def test_missing_position(self):
        g=grid();del g.info.origin.position;self.reject(g)
    def test_none_grid_data(self):
        g=grid();g.data=None;self.reject(g)
    def test_missing_tf_header(self):
        t=tf();del t.header;self.reject(grid("odom"),t)
    def test_none_tf_rotation(self):
        t=tf();t.transform.rotation=None;self.reject(grid("odom"),t)
    def test_nonnumeric_tf_translation(self):
        t=tf();t.transform.translation.x=object();self.reject(grid("odom"),t)
    def test_nonnumeric_odom_origin(self):
        g=grid("odom");g.info.origin.position.y=object();self.reject(g,tf())
    def test_valid_map_grid(self):
        self.assertEqual(rolling_costmap_in_map(grid(),sim_now_s=20.).cell_at(.05,.05),0)
    def test_valid_odom_grid(self):
        self.assertEqual(rolling_costmap_in_map(grid("odom"),map_from_odom=tf(),sim_now_s=20.).cell_at(.05,.05),0)
    def test_known_stale_error_preserved(self):
        with self.assertRaisesRegex(CostmapFrameError,"stale"):
            rolling_costmap_in_map(grid(),sim_now_s=30.)
if __name__=="__main__":unittest.main()
