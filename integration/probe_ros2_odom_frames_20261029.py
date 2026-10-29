"""Isolated real ROS2 Jazzy Odometry DDS ingress probe (NO motion commands).

Requires disposable networkless Podman runner. Publishes *only* nav_msgs/Odometry
on /odom, observes private relay state, and never requests velocity ownership.
"""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from oomwoo_cleaning_jobs.ros_dock_velocity_relay import ExperimentalDockVelocityRelay


def odom(stamp_ns, *, frame_id="odom", child_frame_id="base_footprint"):
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.twist.twist.linear.x = 0.0
    msg.twist.twist.angular.z = 0.0
    return msg


def deliver(executor, relay, publisher, stamp_ns, *, accepted,
            frame_id="odom", child_frame_id="base_footprint", failure=None):
    """Verify callback delivery and STOP on rejection using actual ROS graph."""
    relay._odom_last = None
    relay.mux.reset("probe pending odometry callback")
    packet = odom(stamp_ns,frame_id=frame_id,child_frame_id=child_frame_id)
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        publisher.publish(packet)
        executor.spin_once(timeout_sec=0.03)
        executor.spin_once(timeout_sec=0.03)
        if accepted and relay._odom_last is not None:
            assert relay.mux.owner == relay.mux.STOP
            return True
        if (not accepted and relay.mux.stop_reason ==
                (failure or "ROS odometry header stale/replayed/invalid")):
            assert relay._odom_last is None
            assert relay.mux.owner == relay.mux.STOP
            return True
    raise AssertionError("Odometry DDS callback or stamp policy not observed")


def main():
    rclpy.init()
    relay = ExperimentalDockVelocityRelay(
        parameter_overrides=[Parameter("simulation_lab",value=True)])
    sender = Node("valdi_stamped_odom_lab_sender")
    publisher = sender.create_publisher(Odometry,"/odom",10)
    executor = SingleThreadedExecutor()
    executor.add_node(relay)
    executor.add_node(sender)
    cases = []
    try:
        assert relay.safe.topic_name == "/valdi/lab_safe_cmd_vel_output"
        assert not sender.get_publishers_info_by_topic("/cmd_vel")
        # Wait briefly for pub/sub DDS graph to match, no locomotion issued.
        deadline=time.monotonic()+3.0
        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=0.05)
        assert publisher.get_subscription_count()>0,"/odom DDS graph never matched"

        def check(name,stamp_ns,expected,*,frame_id="odom",
                  child_frame_id="base_footprint",failure=None):
            deliver(executor,relay,publisher,stamp_ns,accepted=expected,
                    frame_id=frame_id,child_frame_id=child_frame_id,
                    failure=failure)
            cases.append({"case":name,"pass":True,"accepted":expected})

        now=relay.get_clock().now().nanoseconds
        check("fresh",now,True)
        check("duplicate",now,False)
        now=relay.get_clock().now().nanoseconds
        check("recovered_fresh",now,True)
        check("old_2s",now-2_000_000_000,False)
        now=relay.get_clock().now().nanoseconds
        check("future_2s",now+2_000_000_000,False)
        check("zero_stamp",0,False)
        now=relay.get_clock().now().nanoseconds
        check("new_valid",now,True)
        check("reordered",now-1_000_000,False)
        # Incorrect reference frames cannot refresh the odometry lease.
        fail="ROS odometry frame mismatch or missing"
        for name,parent,child in (
            ("missing_parent","","base_footprint"),
            ("missing_child","odom",""),
            ("map_parent","map","base_footprint"),
            ("base_link_child","odom","base_link"),
            ("slash_parent","/odom","base_footprint"),
            ("slash_child","odom","/base_footprint"),
            ("whitespace_child","odom","base_footprint ")):
            check(name,relay.get_clock().now().nanoseconds,False,
                  frame_id=parent,child_frame_id=child,failure=fail)
        check("recovered_correct_frame",
              relay.get_clock().now().nanoseconds,True)
        assert relay.mux.owner==relay.mux.STOP
        assert not sender.get_publishers_info_by_topic("/cmd_vel")
        print("ROS2_ODOM_FRAME_PROBE "+json.dumps(
            {"pass":len(cases),"total":len(cases),"cases":cases,
             "ros2_dds_odometry":True,"motor_topics_published":0,
             "nonzero_velocity_published":0,
             "physical_hardware_connected":False},sort_keys=True),
              flush=True)
    finally:
        executor.shutdown()
        sender.destroy_node()
        relay.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == "__main__":
    main()
