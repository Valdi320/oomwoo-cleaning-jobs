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
    UUID5="c5"*16
    UUID6="c6"*16
    FOREIGN="df"*16
    tokens={
       UUID1:"a1"*16+"-1",
       UUID2:"a2"*16+"-2",
       UUID3:"a3"*16+"-3",
       UUID4:"a4"*16+"-4",
       UUID5:"a5"*16+"-5",
       UUID6:"a6"*16+"-6",
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
        msg.header.frame_id="odom"
        msg.child_frame_id="base_footprint"
        stamp_ns=relay.get_clock().now().nanoseconds
        msg.header.stamp.sec=stamp_ns//1_000_000_000
        msg.header.stamp.nanosec=stamp_ns%1_000_000_000
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
        # The real BatteryState callback must preserve fraction semantics.
        # Prove the other admission witnesses are ready before testing 100.
        shared["battery"]=100.
        assert until(lambda:relay._battery_fraction==100.,1.)
        assert relay._nav_idle() and relay._dock_state=="IDLE"
        assert relay._fresh(relay._odom_last,.35)
        assert relay.mux._quiet(time.monotonic())
        denied_battery=call("claim_nav")
        assert not denied_battery.ok and relay.mux.owner=="stopped"
        shared["battery"]=.92
        assert until(lambda:math.isclose(relay._battery_fraction,.92,
                                         abs_tol=1e-6),1.)
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

        # A percent-style value arriving over DDS during an accepted NAV
        # lease must revoke it, even while odom/status/commands stay fresh.
        assert call("claim_nav").ok
        begin(UUID5)
        shared["nav_status"]=((UUID5,2),)
        shared["nav"]=.18
        assert until(lambda:relay._goal_witness.current_active_matches(),.8)
        assert observe(.18,.5)
        shared["battery"]=100.
        assert until(lambda:relay._battery_fraction==100.
                     and relay.mux.owner=="stopped",.8)
        assert relay.mux.stop_reason=="unsafe resource telemetry during velocity lease"
        assert observe(0.,.5)
        terminal(UUID5,"canceled")
        shared["nav_status"]=((UUID5,5),)
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
        shared["battery"]=100.
        assert until(lambda:relay._battery_fraction==100.
                     and relay.mux.owner=="stopped",.8)
        assert relay.mux.stop_reason=="unsafe resource telemetry during velocity lease"
        assert observe(0.,.5)
        shared["battery"]=.9
        shared["nav"]=0.
        shared["dock"]=0.
        stable(.9)

        # Reconcile a new Nav2 UUID and terminal handoff before checking
        # the existing competing-action case on a fresh DOCK lease.
        assert call("claim_nav").ok
        begin(UUID6)
        shared["nav_status"]=((UUID6,2),)
        assert until(lambda:relay._goal_witness.current_active_matches(),.8)
        terminal(UUID6,"succeeded")
        shared["nav_status"]=((UUID6,4),)
        stable(.20)
        assert call("release_for_dock").ok
        stable(.9)
        assert call("claim_dock").ok
        shared["dock"]= -.07
        assert observe(-.07,.5)
        # A competing Nav2 action invalidates the DOCK lease too.
        shared["nav_status"]=((FOREIGN,2),)
        assert until(lambda:relay.mux.owner=="stopped",.85)
        assert observe(0.,.5)
        shared["nav_status"]=((UUID6,4),)
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
                "percent_style_battery_denies_admission":True,
                "percent_style_battery_revokes_nav_lease":True,
                "percent_style_battery_revokes_dock_lease":True,
                "battery_stop_reason_verified_for_both_owners":True,
                "competing_nav2_revokes_dock_lease":True,
                "motor_hardware_connected":False,
                "docking_mechanically_confirmed":False,
                "charging_bin_mop_verified":False,
                "output_messages":len(outputs)}
        print("ROS2_DOCK_RELAY "+json.dumps(result,sort_keys=True),flush=True)
    finally:
        ex.shutdown();source.destroy_node();relay.destroy_node();rclpy.shutdown()
        scratch.cleanup()



def main_three_way(*,keepout=False):
    """Exercise the same relay's three-way mode with real DDS, synthetic inputs."""
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import CostmapFilterInfo
    from rosgraph_msgs.msg import Clock as ClockMessage
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=[])
    scratch=tempfile.TemporaryDirectory(prefix="valdi-three-way-dds-")
    journal_path=Path(scratch.name)/"uuid.json"
    journal=GoalJournal(journal_path)
    relay=ExperimentalDockVelocityRelay(parameter_overrides=[
        Parameter("simulation_lab",value=True),Parameter("three_way",value=True),
        Parameter("require_keepout_filter",value=keepout),
        Parameter("use_sim_time",value=True),
        Parameter("goal_journal_path",value=str(journal_path))])
    source=Node("oomwoo_three_way_sim_observations")
    producers=[Node(name) for name in ("controller_server","coverage_planner","dock_drive")]
    ex=SingleThreadedExecutor()
    for node in [relay,source,*producers]:ex.add_node(node)
    latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    publishers={role:node.create_publisher(Twist,topic,10) for role,node,topic in zip(
        ("nav","sweep","dock"),producers,
        ("/valdi/nav2_cmd_vel","/valdi/sweep_cmd_vel","/valdi/dock_drive_cmd_vel"))}
    odom=source.create_publisher(Odometry,"/odom",10)
    battery=source.create_publisher(BatteryState,"/battery_state",10)
    clock=source.create_publisher(ClockMessage,"/clock",10)
    tf=source.create_publisher(TFMessage,"/tf",10)
    map_pub=source.create_publisher(OccupancyGrid,"/map",latched)
    if keepout:
        mask_pub=source.create_publisher(OccupancyGrid,"/oomwoo/keepout_mask",latched)
        info_pub=source.create_publisher(CostmapFilterInfo,"/oomwoo/keepout_filter_info",latched)
        costmap_pub=source.create_publisher(OccupancyGrid,"/global_costmap/costmap",10)
    unstable=source.create_publisher(Bool,"/coverage_meter/sim_unstable",10)
    nav_status=source.create_publisher(GoalStatusArray,"/navigate_to_pose/_action/status",latched)
    sweep_state=producers[1].create_publisher(String,"/coverage_planner/motion_state",10)
    dock_state=producers[2].create_publisher(String,"/dock_drive/state",10)
    docked=producers[2].create_publisher(Bool,"/dock_drive/docked",latched)
    control=source.create_client(ControlCleaning,"/valdi/relay/control")
    outputs=[];metadata=[];checks=[];calls=[];reset_trace=[]
    source.create_subscription(Twist,"/valdi/lab_safe_cmd_vel_output",
        lambda m:outputs.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    def tap(msg,info):
        if not metadata:
            metadata.append({"type":type(info).__name__,"keys":sorted(info),
                "source_timestamp_positive":info.get("source_timestamp",0)>0,
                "publisher_gid_bytes":len(info.get("publisher_gid",[])),
                "publication_sequence_number":info.get("publication_sequence_number")})
    source.create_subscription(Twist,"/valdi/sweep_cmd_vel",tap,10)
    shared={"nav":0.,"sweep":0.,"dock":0.,"battery":.92,"tf":False,"clock":True,
        "sweep_state":"IDLE","dock_state":"IDLE","docked":False,
        "status":(),"changed_map":False,"publish_sweep":True,"publish_status":True,
        "tf_x":0.,"publish_costmap":True,"postfiltered":True,"costmap_last":0.}
    grid=OccupancyGrid();grid.header.frame_id="map"
    grid.info.width=grid.info.height=20;grid.info.resolution=.25
    grid.info.origin.position.x=grid.info.origin.position.y=-2.5
    grid.info.origin.orientation.w=1.;grid.data=[0]*400
    if keepout:
        import copy
        mask=copy.deepcopy(grid);mask.data[10*20+14]=100
        filter_info=CostmapFilterInfo();filter_info.type=0
        filter_info.base=0.;filter_info.multiplier=1.
        filter_info.filter_mask_topic="/oomwoo/keepout_mask"
    def emit():
        if shared["clock"]:
            message=ClockMessage();message.clock=source.get_clock().now().to_msg();clock.publish(message)
        stamp=relay.get_clock().now().to_msg()
        for role,publisher in publishers.items():
            if role=="sweep" and not shared["publish_sweep"]:continue
            message=Twist();message.linear.x=shared[role];publisher.publish(message)
        message=Odometry();message.header.frame_id="odom";message.child_frame_id="base_footprint"
        message.header.stamp=stamp;odom.publish(message)
        bat=BatteryState();bat.percentage=shared["battery"];battery.publish(bat)
        if shared["tf"]:
            transform=TransformStamped();transform.header.frame_id="map";transform.child_frame_id="base_link"
            transform.header.stamp=stamp;transform.transform.rotation.w=1.
            transform.transform.translation.x=shared["tf_x"]
            tf.publish(TFMessage(transforms=[transform]))
        grid.data[1]=100 if shared["changed_map"] else 0
        map_pub.publish(grid)
        if keepout:
            mask_pub.publish(mask);info_pub.publish(filter_info)
            # Reuse this source's existing timer; the real Nav2 config is2Hz.
            if shared["publish_costmap"] and time.monotonic()-shared["costmap_last"]>=.5:
                costmap=copy.deepcopy(mask if shared["postfiltered"] else grid)
                costmap.header.stamp=stamp;costmap_pub.publish(costmap)
                shared["costmap_last"]=time.monotonic()
        unstable.publish(Bool(data=False))
        sweep_state.publish(String(data=shared["sweep_state"]))
        dock_state.publish(String(data=shared["dock_state"]))
        docked.publish(Bool(data=shared["docked"]))
        arr=GoalStatusArray()
        for uuid,code in shared["status"]:
            row=GoalStatus();row.status=code;row.goal_info.goal_id.uuid=list(bytes.fromhex(uuid))
            arr.status_list.append(row)
        if shared["publish_status"]:nav_status.publish(arr)
    source.create_timer(.05,emit)
    def until(condition,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            ex.spin_once(timeout_sec=.01)
            if condition():return True
        return False
    def stable(seconds):until(lambda:False,seconds)
    def output(value,seconds=.8):
        return until(lambda:outputs and math.isclose(outputs[-1][1],value,abs_tol=.002),seconds)
    def call(operation):
        request=ControlCleaning.Request();request.request_id="three-"+str(len(calls))
        request.operation=operation;request.isolated_disposable_sim=True
        future=control.call_async(request)
        assert until(future.done,3.),"typed service timed out"
        reply=future.result();calls.append({"op":operation,"ok":reply.ok,"state":reply.state,"error":reply.error})
        return reply
    def quiet():
        shared.update(nav=0.,sweep=0.,dock=0.,sweep_state="IDLE",dock_state="IDLE",docked=False)
        stable(1.05)
    def sweep():
        reply=call("claim_sweep");assert reply.ok,(calls,relay._motion_health(),relay.mux.stop_reason,metadata)
        shared.update(sweep=.18,sweep_state="SWEEP")
        assert output(.18),(calls,relay._motion_health(),relay.mux.stop_reason,metadata,reset_trace)
    if keepout:
        original_reset=relay._reset
        def traced_reset(reason):
            if len(reset_trace)<100:
                health=getattr(relay.mux,"_health",None)
                reset_trace.append({"wall":time.monotonic(),"reason":reason,
                    "calls":len(calls),"owner_before":relay.mux.owner,
                    "prior_reason":relay.mux.stop_reason,
                    "token_matches":relay._motion_token==relay.mux.token,
                    "health":vars(health) if health is not None else None,
                    "costmap_age_ns":relay.get_clock().now().nanoseconds-relay._keepout_costmap_stamp,
                    "costmap_dds_age_ns":time.time_ns()-relay._keepout_costmap_dds,
                    "costmap_sequence":relay._keepout_costmap_sequence})
            original_reset(reason)
        relay._reset=traced_reset
    started=time.monotonic()
    try:
        assert until(lambda:control.service_is_ready() and outputs,4.)
        stable(1.1)
        if keepout:
            assert until(relay._keepout_ready,3.),"post-filter witnesses did not initialize"
        assert not call("claim_sweep").ok,"missing TF admitted motion"
        checks.append("missing_tf_denies_sweep")
        shared["tf"]=True;quiet();sweep()
        checks.append("sweep_nonzero_through_real_dds")
        if keepout:
            shared["tf_x"]=.7
            assert until(lambda:relay.mux.owner==relay.mux.STOP,1.) and output(0.)
            assert relay.mux.stop_reason=="projected motion intersects keepout"
            proof=relay._motion_health()
            assert proof.pose_fresh and proof.map_current and proof.isolated_sources
            assert proof.resources_ok and proof.safety_loop_ok
            checks.append("virtual_mask_blocks_direct_command_with_other_witnesses_healthy")
            shared["tf_x"]=0.;quiet();sweep()
            shared["publish_costmap"]=False
            assert until(lambda:relay.mux.owner==relay.mux.STOP,1.6) and output(0.)
            assert not relay._keepout_ready()
            checks.append("stale_original_postfilter_costmap_revokes_sweep")
            shared["publish_costmap"]=True;quiet();sweep()
            shared["postfiltered"]=False
            assert until(lambda:relay.mux.owner==relay.mux.STOP,1.2) and output(0.)
            assert not relay._keepout_ready()
            checks.append("fresh_but_unenforced_mask_revokes_sweep")
            shared["postfiltered"]=True;quiet();sweep()
        shared["nav"]=.12
        assert until(lambda:relay.mux.owner==relay.mux.STOP,1.)
        assert output(0.)
        checks.append("foreign_nav_command_revokes_sweep")
        quiet();sweep();shared["battery"]=100.
        assert until(lambda:relay.mux.owner==relay.mux.STOP,1.) and output(0.)
        shared.update(nav=0.,sweep=0.,sweep_state="IDLE");stable(1.)
        assert not call("claim_sweep").ok
        shared["battery"]=.92;quiet();sweep()
        checks.append("percent_battery_revokes_and_denies_sweep")
        shared.update(sweep=0.,sweep_state="TRANSIT");stable(.16)
        assert call("release_motion").ok and output(0.)
        uuid="d7"*16;token="a8"*16+"-1"
        journal.begin(uuid_hex=uuid,token=token,job_id="three_way_lab",
            map_digest=relay._map_hash,target_cell=1)
        stable(1.)
        assert call("claim_nav").ok,calls
        journal.accepted(token=token,uuid_hex=uuid)
        shared["status"]=((uuid,2),);stable(.15);shared["nav"]=.23
        assert output(.23),(calls,relay.mux.stop_reason)
        checks.append("sweep_to_exact_uuid_nav_handoff")
        shared["publish_status"]=False
        assert until(lambda:relay.mux.owner==relay.mux.STOP,1.5), \
            "expired Nav2 status retained nonzero ownership"
        assert output(0.)
        checks.append("missing_nav_status_revokes_active_navigation")
        # Losing the lease cannot release the old goal into docking. Reconcile
        # its terminal result and obtain a new exact-UUID NAV claim first.
        shared["nav"]=0.
        journal.terminal(token=token,uuid_hex=uuid,outcome="succeeded")
        shared.update(status=((uuid,4),),publish_status=True)
        stable(1.)
        assert not call("release_nav").ok
        uuid="d8"*16;token="a9"*16+"-2"
        journal.begin(uuid_hex=uuid,token=token,job_id="three_way_lab",
            map_digest=relay._map_hash,target_cell=1)
        assert call("claim_nav").ok,calls
        journal.accepted(token=token,uuid_hex=uuid)
        shared["status"]=((uuid,2),);stable(.15);shared["nav"]=.23
        assert output(.23),(calls,relay.mux.stop_reason)
        shared["nav"]=0.;stable(.12)
        journal.terminal(token=token,uuid_hex=uuid,outcome="succeeded")
        shared["status"]=((uuid,4),)
        assert until(relay._nav_confirmed_terminal,1.)
        assert call("release_nav").ok,calls
        stable(1.)
        assert call("claim_dock").ok,calls
        shared.update(dock=-.08,dock_state="SEARCH")
        assert output(-.08),(calls,relay.mux.stop_reason)
        assert until(lambda:relay._dock_started,1.)
        checks.append("exact_uuid_nav_to_dock_handoff")
        shared.update(dock=0.,dock_state="DONE",docked=True);stable(.16)
        assert call("release_motion").ok and output(0.)
        checks.append("software_dock_phase_release_not_physical_contact")
        quiet();sweep();shared["tf"]=False
        assert until(lambda:relay.mux.owner==relay.mux.STOP,1.3) and output(0.)
        assert until(lambda:relay._motion_health().pose_fresh is False,.8)
        checks.append("tf_dropout_revokes_sweep")
        shared["tf"]=True;quiet();sweep()
        shared["publish_sweep"]=False
        assert until(lambda:relay.mux.owner==relay.mux.STOP,1.) and output(0.)
        checks.append("command_lease_expiry_is_stop")
        shared["publish_sweep"]=True;quiet();sweep();shared["sweep"]=.35
        assert until(lambda:relay.mux.owner==relay.mux.STOP,1.) and output(0.)
        checks.append("overspeed_rejected_without_raising_limits")
        quiet();sweep()
        duplicate=source.create_publisher(Twist,"/valdi/sweep_cmd_vel",10)
        assert until(lambda:relay.mux.owner==relay.mux.STOP,1.) and output(0.)
        source.destroy_publisher(duplicate)
        checks.append("second_writer_revokes_ownership")
        quiet();sweep();shared["clock"]=False
        previous=len(outputs);stable(.7)
        assert len(outputs)>previous+3,"paused ROS clock suspended relay timer"
        assert relay.mux.owner==relay.mux.STOP and output(0.)
        assert relay._motion_health().safety_loop_ok is False
        checks.append("paused_ros_clock_does_not_suspend_output_watchdog")
        shared["clock"]=True;quiet();sweep();shared["changed_map"]=True
        assert until(lambda:relay._map_fault,1.) and output(0.)
        shared["changed_map"]=False;quiet()
        assert not call("claim_sweep").ok
        checks.append("changed_map_latches_reconciliation_required")
        assert source.count_publishers("/cmd_vel")==0
        assert source.count_publishers("/valdi/lab_safe_cmd_vel_output")==1
        print("ROS2_THREE_WAY_RELAY "+json.dumps({"ok":True,"checks":checks,"calls":calls,
            "keepout_required":keepout,
            "reset_trace":reset_trace,
            "output_messages":len(outputs),"metadata":metadata,"elapsed_s":round(time.monotonic()-started,2),
            "final_owner":relay.mux.owner,"limits":{"max_linear":relay.mux.max_linear,"max_angular":relay.mux.max_angular},
            "scope":"actual DDS/services/TF cache; synthetic clock,map,phase,resource,Nav2 status and commands; no Gazebo/physical robot",
            "actual_nav2_action_server":False,"physical_contact_or_charging":False},sort_keys=True),flush=True)
    except Exception:
        print("ROS2_THREE_WAY_FAILURE "+json.dumps({
            "checks_completed":checks,"calls":calls,"reset_trace":reset_trace,
            "keepout_required":keepout,"owner":relay.mux.owner,
            "stop_reason":relay.mux.stop_reason},sort_keys=True),flush=True)
        raise
    finally:
        ex.shutdown()
        for node in [*producers,source,relay]:node.destroy_node()
        scratch.cleanup()
        rclpy.shutdown()


if __name__=="__main__":
    import sys
    main_three_way(keepout="--keepout" in sys.argv) if "--three-way" in sys.argv else main()
