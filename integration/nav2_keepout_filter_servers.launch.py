"""Launch ONLY the isolated test KeepoutFilter metadata and mask servers.

These do not send robot motion commands. The ordinary OOMWOO simulator
and Nav2 test processes run separately in the same offline ROS domain.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="nav2_map_server", executable="map_server",
             name="valdi_keepout_mask_server",output="screen",
             parameters=[{"use_sim_time":True,
                 "yaml_filename":"/lab/keepout_test_room.yaml",
                 "topic_name":"/valdi/keepout_filter_mask",
                 "frame_id":"map"}]),
        Node(package="nav2_map_server", executable="costmap_filter_info_server",
             name="valdi_keepout_info_server",output="screen",
             parameters=[{"use_sim_time":True,
                 "type":0,
                 "filter_info_topic":"/valdi/keepout_costmap_filter_info",
                 "mask_topic":"/valdi/keepout_filter_mask",
                 "base":0.0,"multiplier":1.0}]),
        Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",
             name="valdi_keepout_lifecycle_manager",output="screen",
             parameters=[{"use_sim_time":True,
                 "autostart":True,"bond_timeout":0.0,
                 "node_names":["valdi_keepout_mask_server",
                               "valdi_keepout_info_server"]}]),
    ])
