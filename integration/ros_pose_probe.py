"""ROS2 message-level checks for translated/rotated occupancy-grid goals."""
import math
from oomwoo_cleaning_jobs.nav_interlock import Goal
from oomwoo_cleaning_jobs.ros_nav2 import pose_for_cell

goal=Goal("frame-1","pose_case",6,(0,1,2,6),"0"*64)
p=pose_for_cell(goal,width=4,height=3,resolution=0.5,
                origin_x=1.,origin_y=-2.,origin_yaw=math.pi/2)
# Cell 6 is row 1 col 2 => local centre (1.25,0.75).
assert abs(p.pose.pose.position.x-0.25)<1e-10
assert abs(p.pose.pose.position.y-(-0.75))<1e-10
assert p.pose.header.frame_id=="map"
assert abs(p.pose.pose.orientation.z-math.sqrt(.5))<1e-10
assert abs(p.pose.pose.orientation.w-math.sqrt(.5))<1e-10
for kw in ({"width":0},{"origin_yaw":float("nan")},{"resolution":0}):
    args={"width":4,"height":3,"resolution":0.5,
          "origin_x":1.,"origin_y":-2.,"origin_yaw":0.}
    args.update(kw)
    try:pose_for_cell(goal,**args)
    except ValueError:pass
    else:raise AssertionError("unsafe map unexpectedly admitted")
print("ROS_POSE_PROBE PASS: 90-degree map transform and invalid input rejection")
