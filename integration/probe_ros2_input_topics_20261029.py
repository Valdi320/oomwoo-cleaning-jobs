"""Real isolated ROS2 name resolution, full relay constructor, no Twist publishing."""
import argparse,json
from unittest.mock import patch
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from oomwoo_cleaning_jobs.lab_topic_guard import LabTopicGuardError
from oomwoo_cleaning_jobs.ros_dock_velocity_relay import ExperimentalDockVelocityRelay
CASES={
 "nominal":("/valdi/nav2_cmd_vel","/valdi/nav2_cmd_vel",True),
 "nav_to_dock":("/valdi/nav2_cmd_vel","/valdi/dock_drive_cmd_vel",False),
 "dock_to_nav":("/valdi/dock_drive_cmd_vel","/valdi/nav2_cmd_vel",False),
 "nav_to_cmd_vel":("/valdi/nav2_cmd_vel","/cmd_vel",False),
 "odom_to_other":("/odom","/another_odom",False),
 "battery_to_other":("/battery_state","/another_battery",False),
 "status_to_other":("/navigate_to_pose/_action/status","/another_status",False),
 "dock_state_to_other":("/dock_drive/state","/another_state",False),
 "docked_to_other":("/dock_drive/docked","/another_docked",False),
}
THREE_WAY_CASES={'three_nominal': ('/valdi/sweep_cmd_vel', '/valdi/sweep_cmd_vel', True), 'sweep_to_nav': ('/valdi/sweep_cmd_vel', '/valdi/nav2_cmd_vel', False), 'sweep_state_to_other': ('/coverage_planner/motion_state', '/another_phase', False), 'map_to_other': ('/map', '/another_map', False), 'clock_to_other': ('/clock', '/another_clock', False), 'unstable_to_other': ('/coverage_meter/sim_unstable', '/another_stability', False), 'tf_to_other': ('/tf', '/another_tf', False), 'tf_static_to_other': ('/tf_static', '/another_static_tf', False), 'three_output_to_motor': ('/valdi/lab_safe_cmd_vel_output', '/cmd_vel', False)}
CASES.update(THREE_WAY_CASES)
def main():
    a=argparse.ArgumentParser()
    a.add_argument("--case",choices=CASES,required=True)
    a.add_argument("--three-way",action="store_true")
    opts,ros_args=a.parse_known_args()
    nominal,expected,allowed=CASES[opts.case]
    rclpy.init(args=ros_args)
    probe=Node("valdi_lab_input_resolution_probe")
    relay=None
    try:
        sub=probe.create_subscription(Twist,nominal,lambda m:None,10)
        resolved=sub.topic_name
        assert resolved==expected,(nominal,resolved,expected)
        probe.destroy_subscription(sub)
        params=[Parameter("simulation_lab",value=True),Parameter("three_way",value=opts.three_way)]
        if allowed:
            relay=ExperimentalDockVelocityRelay(parameter_overrides=params)
            assert relay.safe.topic_name=="/valdi/lab_safe_cmd_vel_output"
            result="allowed"
        else:
            with patch.object(ExperimentalDockVelocityRelay,"create_timer",
                              side_effect=AssertionError("unsafe input armed timer")) as timer:
                try:
                    relay=ExperimentalDockVelocityRelay(parameter_overrides=params)
                except LabTopicGuardError:
                    assert not timer.called
                    result="denied_before_timer"
                else:raise AssertionError("remapped input was accepted")
        print("LAB_INPUT_REMAP "+json.dumps({
            "case":opts.case,"three_way":opts.three_way,"nominal":nominal,"resolved":resolved,
            "expected":expected,"result":result,
            "no_twists_published":True,"motor_hardware":False},sort_keys=True),flush=True)
    finally:
        if relay is not None:relay.destroy_node()
        probe.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=="__main__":main()
