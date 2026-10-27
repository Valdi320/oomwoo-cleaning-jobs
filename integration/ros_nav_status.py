"""Inspect active Gazebo/Nav2 ROS graph using fresh rclpy, not ros2 daemon."""
import json,time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

rclpy.init()
node=Node("valdi_nav2_discovery_probe")
v={"nodes":[],"topics":[],"action_server":False,"map":None,"odom":None,
   "sim_time_sec":None, "cmd_vel":None}
def on_map(m):
 v["map"]={"width":m.info.width,"height":m.info.height,
           "resolution":m.info.resolution,
           "origin_x":m.info.origin.position.x,
           "origin_y":m.info.origin.position.y,
           "occ_count":len(m.data),
           "frame":m.header.frame_id}
def on_odom(m):
 v["odom"]={"x":m.pose.pose.position.x,"y":m.pose.pose.position.y,
            "linear_x":m.twist.twist.linear.x,"angular_z":m.twist.twist.angular.z,
            "frame":m.header.frame_id}
def on_clock(m):v["sim_time_sec"]=m.clock.sec+m.clock.nanosec*1e-9
def on_twist(m):v["cmd_vel"]={"linear_x":m.linear.x,"angular_z":m.angular.z}
q=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(OccupancyGrid,"/map",on_map,q)
node.create_subscription(Odometry,"/odom",on_odom,10)
node.create_subscription(Clock,"/clock",on_clock,10)
node.create_subscription(Twist,"/cmd_vel",on_twist,10)
client=ActionClient(node,NavigateToPose,"/navigate_to_pose")
t=time.monotonic()+15
while time.monotonic()<t:
 rclpy.spin_once(node,timeout_sec=.25)
 v["nodes"]=node.get_node_names()
 v["topics"]=node.get_topic_names_and_types()
 v["action_server"]=client.server_is_ready()
 if v["action_server"] and v["map"] and v["odom"] and v["sim_time_sec"]:break
print("NAV2_DISCOVERY "+json.dumps({**v,"topics":v["topics"][:70]},sort_keys=True))
node.destroy_node()
rclpy.shutdown()
