"""Real ROS2 middleware proof of Nav2-vs-dock cmd_vel exclusivity (NO MOTORS).

A lab publisher generates synthetic Nav2 and dock Twist inputs, odometry,
battery, Nav2 ActionServer status and dock state; the experimental relay
publishes ONLY to /valdi/lab_safe_cmd_vel_output. No Gazebo/physical
motors are connected, and no actual dock/charging is simulated here.
"""
from __future__ import annotations
import json,time,math,tempfile
from pathlib import Path
from oomwoo_cleaning_jobs.goal_journal import GoalJournal

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String,Bool
from action_msgs.msg import GoalStatusArray,GoalStatus
from oomwoo_cleaning_jobs_lab_interfaces.srv import ControlCleaning,GetCleaningStatus
from oomwoo_cleaning_jobs.ros_dock_velocity_relay import ExperimentalDockVelocityRelay


def main():
    rclpy.init()
    scratch=tempfile.TemporaryDirectory(prefix="valdi-real-ros2-lease-")
    journal_path=Path(scratch.name)/"uuid.json"
    journal=GoalJournal(journal_path)
    UUID1="c1"*16
    UUID2="c2"*16
    UUID3="c3"*16
    UUID4="c4"*16
    FOREIGN="df"*16
    tokens={
       UUID1:"a1"*16+"-1",
       UUID2:"a2"*16+"-2",
       UUID3:"a3"*16+"-3",
       UUID4:"a4"*16+"-4",
    }
    def begin(uuid):
        journal.begin(uuid_hex=uuid,token=tokens[uuid],
                      job_id="dock_lab",map_digest="ee"*32,target_cell=1)
        journal.accepted(token=tokens[uuid],uuid_hex=uuid)
    def terminal(uuid,kind):
        journal.terminal(token=tokens[uuid],uuid_hex=uuid,outcome=kind)
    relay=ExperimentalDockVelocityRelay(parameter_overrides=[
         Parameter("simulation_lab",value=True),
         Parameter("goal_journal_path",value=str(journal_path))])
    source=Node("oomwoo_lab_velocity_sources")
    outputs=[]
    source.create_subscription(Twist,"/valdi/lab_safe_cmd_vel_output",
           lambda m:outputs.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    nav=source.create_publisher(Twist,"/valdi/nav2_cmd_vel",10)
    dock=source.create_publisher(Twist,"/valdi/dock_drive_cmd_vel",10)
    odom=source.create_publisher(Odometry,"/odom",10)
    battery=source.create_publisher(BatteryState,"/battery_state",10)
    latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    nav_status=source.create_publisher(GoalStatusArray,"/navigate_to_pose/_action/status",latched)
    state=source.create_publisher(String,"/dock_drive/state",10)
    docked=source.create_publisher(Bool,"/dock_drive/docked",latched)
    control=source.create_client(ControlCleaning,"/valdi/relay/control")
    status=source.create_client(GetCleaningStatus,"/valdi/relay/status")
    shared={"nav":0.,"dock":0.,"battery":.92,"nav_status":(),"dock_state":"IDLE",
            "docked":False,"publish_nav":True,"publish_dock":True}
    def twist(value):
        m=Twist();m.linear.x=value;return m
    def emit():
        if shared["publish_nav"]:nav.publish(twist(shared["nav"]))
        if shared["publish_dock"]:dock.publish(twist(shared["dock"]))
        msg=Odometry()
        msg.twist.twist.linear.x=0.
        msg.twist.twist.angular.z=0.
        odom.publish(msg)
        bat=BatteryState();bat.percentage=shared["battery"]
        battery.publish(bat)
        arr=GoalStatusArray()
        arr.status_list=[]
        for uuid,code in shared["nav_status"]:
            row=GoalStatus()
            row.status=code
            row.goal_info.goal_id.uuid=list(bytes.fromhex(uuid))
            arr.status_list.append(row)
        nav_status.publish(arr)
        docked.publish(Bool(data=shared["docked"]))
        state.publish(String(data=shared["dock_state"]))
    source.create_timer(.05,emit)
    ex=SingleThreadedExecutor()
    ex.add_node(source);ex.add_node(relay)
    n=0;log=[]
    def until(condition,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            ex.spin_once(timeout_sec=.02)
            if condition():return True
        return False
    def stable(seconds):
        until(lambda:False,seconds)
    def call(operation,id=None,isolated=True):
        nonlocal n
        n+=1
        request=ControlCleaning.Request()
        request.request_id=id or "mux-"+str(n)
        request.operation=operation
        request.isolated_disposable_sim=isolated
        future=control.call_async(request)
        if not until(future.done,5):raise RuntimeError("typed ROS2 relay service no response")
        reply=future.result()
        if reply is None:raise RuntimeError("ROS2 relay response None")
        log.append({"op":operation,"ok":reply.ok,"state":reply.state,"error":reply.error})
        return reply
    def last():return outputs[-1][1] if outputs else None
    def observe(value,seconds=.20,tol=.002):
        return until(lambda:outputs and abs(last()-value)<tol,seconds)
    try:
        assert until(lambda:control.service_is_ready() and status.service_is_ready()
                     and outputs,6)
        assert observe(0.)
        unauthorized=call("claim_nav",id="unauthorized",isolated=False)
        assert not unauthorized.ok and relay.mux.owner=="stopped"
        stable(.91)
        assert call("claim_nav").ok
        begin(UUID1)
        shared["nav_status"]=((UUID1,2),)
        stable(.22)
        shared["nav"]=.23
        shared["dock"]=.41
        assert observe(.23,.6),"Nav2 owner output missing"
        shared["publish_nav"]=False
        assert observe(0.,.72),"stale Nav2 commands were not zeroed"
        shared["publish_nav"]=True
        shared["nav"]=0.
        terminal(UUID1,"succeeded")
        shared["nav_status"]=((UUID1,4),) # terminal SUCCEEDED on lab ActionServer status
        stable(.15)
        assert call("release_for_dock").ok
        assert observe(0.)
        shared["nav"]=.35
        assert observe(0.,.16),"Nav2 still commanding after release"
        shared["nav"]=0.
        stable(.84)
        assert call("claim_dock").ok
        shared["dock"]= -.08
        shared["nav"]=.37
        assert observe(-.08,.5),"Dock owner input did not pass"
        shared["publish_dock"]=False
        assert observe(0.,.7),"stale dock input was not zeroed"
        shared["publish_dock"]=True
        shared["dock"]=0.
        shared["dock_state"]="SEARCH"
        shared["docked"]=False
        stable(.20)
        shared["dock_state"]="DONE"
        shared["docked"]=True
        stable(.85)
        done=call("finish_dock")
        assert done.ok,log
        assert observe(0.)
        # An unrelated Nav2 action becoming active DURING a dock or
        # navigation velocity lease must revoke that lease immediately.
        shared["dock_state"]="IDLE"
        shared["docked"]=False
        stable(.90)
        assert call("claim_nav").ok
        shared["nav"]=.17
        # A latched terminal result from the PREVIOUS Nav2 goal must
        # NOT unlock the new velocity lease.
        assert observe(0.,.3)
        assert not relay._goal_witness.can_release_for_dock()
        begin(UUID2)
        shared["nav_status"]=((UUID2,2),)
        assert until(lambda:relay._goal_witness.current_active_matches(),.8)
        assert observe(.17,.5)
        # Fresh but EMPTY Nav2 status must not permit nonzero output
        # merely because this UUID was seen active earlier.
        shared["nav_status"]=()
        assert observe(0.,.35)
        shared["nav_status"]=((UUID2,2),)
        assert observe(.17,.55)
        # Another client's unexpected ACTIVE goal invalidates the lease.
        shared["nav_status"]=((FOREIGN,2),)
        assert until(lambda:relay.mux.owner=="stopped",.8)
        assert observe(0.,.5)
        terminal(UUID2,"canceled")
        shared["nav_status"]=((UUID2,5),)
        shared["nav"]=0.
        stable(.9)

        assert call("claim_nav").ok
        begin(UUID3)
        shared["nav_status"]=((UUID3,2),)
        shared["nav"]=.18
        assert until(lambda:relay._goal_witness.current_active_matches(),.8)
        assert observe(.18,.5)
        shared["battery"]=.04
        assert until(lambda:relay.mux.owner=="stopped",.8)
        assert observe(0.,.5)
        terminal(UUID3,"canceled")
        shared["nav_status"]=((UUID3,5),)
        shared["battery"]=.9
        shared["nav"]=0.
        stable(.9)

        assert call("claim_nav").ok
        begin(UUID4)
        shared["nav_status"]=((UUID4,2),)
        assert until(lambda:relay._goal_witness.current_active_matches(),.8)
        terminal(UUID4,"succeeded")
        shared["nav_status"]=((UUID4,4),)
        stable(.20)
        assert call("release_for_dock").ok
        stable(.90)
        assert call("claim_dock").ok
        shared["dock"]= -.07
        assert observe(-.07,.5)
        # A competing Nav2 action invalidates the DOCK lease too.
        shared["nav_status"]=((FOREIGN,2),)
        assert until(lambda:relay.mux.owner=="stopped",.85)
        assert observe(0.,.5)
        shared["nav_status"]=((UUID4,4),)
        shared["dock"]=0.
        with_replay=call("stop",id="replay1")
        assert with_replay.ok
        replay=call("claim_nav",id="replay1")
        assert not replay.ok and "different command" in replay.error
        assert relay.mux.owner=="stopped"
        req=GetCleaningStatus.Request();req.request_id="verify"
        future=status.call_async(req)
        assert until(future.done,4)
        d=json.loads(future.result().status_json)
        assert d["physical_dock_service_verified"] is False
        assert d["motor_hardware_connected"] is False
        result={"ok":True,"mode":"actual ROS2 middleware, SYNTHETIC Nav2/dock/odom and no physical motors",
                "ros2_typed_service":True,"final_owner":relay.mux.owner,
                "phases":log,"test_output_positive_nav2":.23,
                "test_output_negative_dock":-.08,
                "zero_on_each_stale_owner":True,
                "unknown_nav2_active_uuid_revokes_lease":True,
                "old_terminal_uuid_cannot_release_current_lease":True,
                "empty_status_revokes_nonzero_output":True,
                "low_battery_revokes_lease":True,
                "competing_nav2_revokes_dock_lease":True,
                "motor_hardware_connected":False,
                "docking_mechanically_confirmed":False,
                "charging_bin_mop_verified":False,
                "output_messages":len(outputs)}
        print("ROS2_DOCK_RELAY "+json.dumps(result,sort_keys=True),flush=True)
    finally:
        ex.shutdown();source.destroy_node();relay.destroy_node();rclpy.shutdown()
        scratch.cleanup()


if __name__=="__main__":main()
