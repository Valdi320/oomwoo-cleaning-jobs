"""Experimental persistent ROS2 cleaning-job control node for disposable Gazebo.

This is intentionally NOT the upstream canonical ROS2 action/service API.
The private /valdi/cleaning_jobs command/status topics are for isolated CI
and simulation only; the executable fails closed unless explicitly opted
in as simulation_lab. No physical motor safety or swept-area feedback.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import BatteryState
from action_msgs.msg import GoalStatusArray, GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import CostmapFilterInfo
from rosgraph_msgs.msg import Clock

from .engine import CleaningJobs, JobError, Resources
from .grid import Grid, MapError
from .map_admission import Raster, CostmapChecker, AdmissionError
from .nav_interlock import NavigationInterlock
from .ros_nav2 import AsyncNav2Transport, pose_for_cell
from .sim_coverage import SimCoverageBridge, CoverageError
from .sweep_target import select_sweep_route, NoSafeSweep
from .nav_keepout_admission import VerifiedGlobalKeepout, KeepoutNotEnforced
from .goal_journal import GoalJournal
from .ros_restart import AsyncOrphanRecovery


_ACTIVE = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
           GoalStatus.STATUS_CANCELING}


class ExperimentalCleaningNode(Node):
    """ROS2 single-executor sandbox exposing a deliberately private JSON API."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__("controller", namespace="/valdi/cleaning_jobs",
                         parameter_overrides=parameter_overrides)
        self.declare_parameter("simulation_lab", False)
        self.declare_parameter("require_nav2_lifecycle", True)
        self.declare_parameter("checkpoint_path", "/tmp/valdi_cleaning_job.json")
        self.declare_parameter("clearance_cells", 5)
        self.declare_parameter("costmap_max_cost", 25)
        self.declare_parameter("coverage_mode", "disabled")
        self.declare_parameter("require_keepout_filter", False)
        self.declare_parameter("auto_continue", False)
        self.declare_parameter("max_no_progress_attempts", 4)
        self._sim_lab = self.get_parameter("simulation_lab").value is True
        self._require_nav2_lifecycle = (
            self.get_parameter("require_nav2_lifecycle").value is True)
        self._checkpoint = Path(self.get_parameter("checkpoint_path").value)
        # A crash must not lose the exact ROS2 goal UUID. Each new action is
        # journaled before send_goal_async so old goals can be reconciled.
        self._nav2_goal_journal=GoalJournal(
            Path(str(self._checkpoint)+".nav2-goal.json"))
        self._clearance = int(self.get_parameter("clearance_cells").value)
        self._max_cost = int(self.get_parameter("costmap_max_cost").value)
        self._coverage_mode = str(self.get_parameter("coverage_mode").value)
        self._require_keepout=(self.get_parameter("require_keepout_filter").value is True)
        if self._coverage_mode not in ("disabled", "sim_ground_truth"):
            raise ValueError("unknown coverage_mode")
        self._auto_continue = self.get_parameter("auto_continue").value is True
        self._max_no_progress=int(self.get_parameter("max_no_progress_attempts").value)
        if not 1<=self._max_no_progress<=8:
            raise ValueError("max_no_progress_attempts must be within 1..8")
        if self._auto_continue and self._coverage_mode != "sim_ground_truth":
            raise ValueError("auto_continue requires sim_ground_truth coverage evidence")

        self._map: Raster | None = None
        self._map_hash: str | None = None
        self._costmap: Raster | None = None
        self._costmap_received: float | None = None
        self._keepout_mask: Raster | None = None
        self._keepout_info: CostmapFilterInfo | None = None
        self._last_keepout_error: str | None = None
        self._clock_sec: float | None = None
        self._clock_received: float | None = None
        self._battery: Resources | None = None
        self._battery_received: float | None = None
        self._active_status: tuple[int, ...] | None = None
        self._frozen: str | None = None
        self._fresh_sim_explicitly_armed = False
        self._ever_sent_goal = False
        self._job: CleaningJobs | None = None
        self._interlock: NavigationInterlock | None = None
        self._port: AsyncNav2Transport | None = None
        self._nav_client = (self.create_client(GetState, "/bt_navigator/get_state")
                            if self._require_nav2_lifecycle else None)
        self._nav_active = False
        self._nav_last_checked: float | None = None
        self._nav_query = None
        self._nav_query_started = None
        self._last_command = {"op": "startup", "ok": True}
        self._last_terminal: str | None = None
        self._recovery: AsyncOrphanRecovery | None = None
        self._recovered_orphan_proof = None
        self._sent_count = 0
        self._accepted_count = 0
        self._meter: SimCoverageBridge | None = None
        self._meter_unstable: bool | None = None
        self._meter_stability_received: float | None = None
        self._meter_received: float | None = None
        self._meter_last_state: str | None = None
        self._meter_accepted_token: str | None = None
        self._next_after_succeeded: float | None = None
        self._no_progress_successes = 0
        self._sweep_tried: set[int] = set()
        self._cleaned_at_dispatch = 0
        self._acknowledged_measured_cells = 0
        self._post_goal_measured = 0
        self._meter_received_at_success: int | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._on_costmap, QoSProfile(depth=5))
        if self._require_keepout:
            self.create_subscription(OccupancyGrid,"/valdi/keepout_filter_mask",
                                     self._on_keepout_mask,latched)
            self.create_subscription(CostmapFilterInfo,
                                     "/valdi/keepout_costmap_filter_info",
                                     self._on_keepout_info,latched)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                 self._on_status, latched)
        if self._coverage_mode == "sim_ground_truth":
            self.create_subscription(OccupancyGrid,
                                     "/coverage_meter/covered_grid",
                                     self._on_covered_grid, latched)
            self.create_subscription(Bool, "/coverage_meter/sim_unstable",
                                     self._on_meter_unstable, latched)
        self.create_subscription(String, "command", self._on_command, 10)
        self._status_pub = self.create_publisher(String, "status", latched)
        self.create_timer(.6, self._poll_nav2_lifecycle)
        self.create_timer(0.35, self._tick)
        self._publish()

    def _now(self) -> float:
        return time.monotonic()

    def _on_clock(self, m):
        self._clock_sec = m.clock.sec + m.clock.nanosec * 1e-9
        self._clock_received = self._now()

    def _on_status(self, m):
        self._active_status = tuple(s.status for s in m.status_list)

    def _nav2_ready(self) -> bool:
        if not self._require_nav2_lifecycle:
            return True  # Only for a test ActionServer, not genuine Nav2.
        return bool(self._nav_active and self._nav_last_checked is not None
                    and self._now()-self._nav_last_checked < 2.5)

    def _poll_nav2_lifecycle(self):
        if not self._require_nav2_lifecycle:
            return
        if self._nav_query is not None:
            if self._now()-self._nav_query_started < 3.5:
                return
            # Pending service call took too long. Ignore late result.
            self._nav_query = None
            self._nav_active = False
        if not self._nav_client.service_is_ready():
            self._nav_active = False
            return
        try:
            future=self._nav_client.call_async(GetState.Request())
            self._nav_query=future
            self._nav_query_started=self._now()
            future.add_done_callback(self._nav_state_callback)
        except Exception:
            self._nav_active=False

    def _nav_state_callback(self, future):
        if self._nav_query is not future:
            return
        self._nav_query=None
        try:
            result=future.result()
            self._nav_active=(result.current_state.label == "active"
                              and result.current_state.id == 3)
        except Exception:
            self._nav_active=False
        self._nav_last_checked=self._now()

    def _signature(self, raster: Raster) -> str:
        payload = json.dumps(
            [raster.width, raster.height, raster.resolution, raster.origin_x,
             raster.origin_y, raster.yaw, raster.data],
            separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def _on_map(self, m):
        try:
            raster = Raster.from_ros(m)
            digest = self._signature(raster)
            if self._map_hash is not None and digest != self._map_hash:
                self._freeze("MAP_CHANGED")
                return
            self._map = raster
            self._map_hash = digest
            if self._meter is None and self._coverage_mode == "sim_ground_truth":
                baseline_grid = Grid(raster.width, raster.height, raster.data,
                     resolution_m=raster.resolution,clearance_cells=self._clearance)
                self._meter = SimCoverageBridge(raster, baseline_grid)
            if self._checkpoint.exists() and self._job is None:
                # Reopening a persistent job NEVER rearms Nav2 automatically.
                # An orphaned UUID must pass actual asynchronous ROS2
                # CancelGoal + server-status + odometry reconciliation first.
                try:
                    recovery_grid=Grid(raster.width,raster.height,raster.data,
                          resolution_m=raster.resolution,clearance_cells=self._clearance)
                    restored=CleaningJobs(recovery_grid,self._checkpoint)
                    self._attach_job(restored)
                except Exception as exc:
                    self._freeze("CHECKPOINT_RELOAD_REJECTED")
                    self.get_logger().error("failed to restore persisted job: "+str(exc))
        except (AdmissionError, ValueError) as exc:
            self._freeze("MAP_INVALID")
            self.get_logger().error(str(exc))

    def _on_costmap(self, m):
        try:
            self._costmap = Raster.from_ros(m)
            self._costmap_received = self._now()
        except AdmissionError:
            self._costmap = None
            self._costmap_received = None

    def _on_meter_unstable(self, m):
        self._meter_unstable = bool(m.data)
        self._meter_stability_received = self._now()
        if self._meter_unstable and self._job is not None:
            self._freeze("COVERAGE_SIM_UNSTABLE")

    def _on_covered_grid(self, m):
        if self._meter is None or self._clock_sec is None:
            return
        if self._meter_unstable is None or self._meter_stability_received is None:
            return  # Never treat missing meter stability as "stable".
        try:
            observed=Raster.from_ros(m)
            stamp=m.header.stamp.sec + m.header.stamp.nanosec*1e-9
            job_state=self._job.status()["state"] if self._job else None
            active_token=(self._interlock.snapshot()["active_goal"]
                          if self._interlock is not None else None)
            live=bool(job_state=="running" and active_token
                      and active_token==self._meter_accepted_token
                      and self._frozen is None)
            tail_token=(self._interlock.eligible_coverage_tail_token()
                        if self._interlock is not None and not live
                        else None)
            # The official 1Hz meter can deliver its final swept sample
            # AFTER Nav2's terminal success. The interlock allows only a
            # time-limited, non-paused completed-success token as evidence.
            active=bool(job_state=="running" and self._frozen is None
                        and (live or tail_token))
            scope=(frozenset(self._job.state["targets"]) if self._job
                   else self._meter.grid.traversable)
            result=self._meter.consume(observed,stamp_s=stamp,
                current_sim_s=self._clock_sec,
                simulation_unstable=self._meter_unstable,
                active=active,nav_goal_accepted=active,
                allowed_targets=scope)
            self._meter_last_state=result.state
            self._meter_received=self._now()
            if result.new_cells:
                if self._interlock is None or not (live or tail_token):
                    raise CoverageError("no accepted active/recent-success goal token")
                producer="oomwoo_sim_ground_truth_meter"
                digest=self._job.grid.fingerprint()
                # Only EXTERNAL measured floor, never Nav2 success, updates
                # durable coverage. A bounded late meter sample can complete
                # an already terminal Nav2 goal without fake retry navigation.
                if live:
                    status=self._interlock.acknowledge_coverage(
                        token=active_token,cells=result.new_cells,
                        producer=producer,map_digest=digest)
                else:
                    status=self._interlock.acknowledge_post_goal_coverage(
                        token=tail_token,cells=result.new_cells,
                        producer=producer,map_digest=digest)
                    self._post_goal_measured+=len(result.new_cells)
                self._acknowledged_measured_cells+=len(result.new_cells)
                self._sweep_tried.clear()
                if status["state"]=="completed":
                    self._next_after_succeeded=None
            self._publish()
        except (CoverageError,AdmissionError,JobError) as exc:
            self._meter_last_state="invalid: "+str(exc)
            if self._job is not None and self._job.status()["state"]=="running":
                self._freeze("COVERAGE_EVIDENCE_INVALID")
            self.get_logger().warning("coverage meter rejected: "+str(exc))

    def _on_keepout_mask(self, m):
        try:
            self._keepout_mask=Raster.from_ros(m)
            self._last_keepout_error=None
        except AdmissionError as exc:
            self._keepout_mask=None
            self._last_keepout_error=str(exc)
            self._freeze("KEEPOUT_INVALID_MASK")

    def _on_keepout_info(self,m):
        self._keepout_info=m
        if (m.type!=0 or m.filter_mask_topic!="/valdi/keepout_filter_mask"):
            self._freeze("KEEPOUT_INVALID_INFO")

    def _keepout_gate(self) -> VerifiedGlobalKeepout | None:
        if not self._require_keepout:
            return None
        if (self._keepout_mask is None or self._keepout_info is None
            or self._keepout_info.type != 0
            or self._keepout_info.filter_mask_topic!="/valdi/keepout_filter_mask"):
            raise JobError("keepout mask or type-0 filter_info missing")
        if (not self.get_publishers_info_by_topic("/valdi/keepout_filter_mask")
            or not self.get_publishers_info_by_topic("/valdi/keepout_costmap_filter_info")):
            raise JobError("keepout source publishers absent")
        if (self._map is None or self._costmap is None
            or self._costmap_received is None
            or self._now()-self._costmap_received>6.0):
            raise JobError("keepout post-filter global costmap not fresh")
        try:
            return VerifiedGlobalKeepout(self._map,self._keepout_mask,self._costmap)
        except KeepoutNotEnforced as exc:
            raise JobError("global KeepoutFilter is not enforcing all masked cells") from exc

    def _on_battery(self, m):
        # Unknown/invalid battery percentage must not authorize motion.
        f = m.percentage
        if not isinstance(f, (float, int)) or not math.isfinite(f) or not 0 <= f <= 1:
            self._battery = None
            self._battery_received = None
            if self._interlock is not None and self._interlock.can_dispatch():
                self._freeze("BATTERY_TELEMETRY_INVALID")
            return
        self._battery = Resources(float(f))
        self._battery_received = self._now()
        if self._interlock is not None and self._job.status()["state"] == "running":
            try:
                self._interlock.observe_resources(self._battery)
            except JobError as exc:
                self._freeze("RESOURCE_TRANSITION_FAILURE")
                self.get_logger().error(str(exc))

    def _on_terminal(self, token: str, outcome: str):
        self._last_terminal = outcome
        if self._interlock is None:
            self._frozen = "ORPHAN_NAV2_RESULT"
            return
        accepted=self._interlock.on_goal_result(token=token, outcome=outcome)
        if accepted and self._meter_accepted_token==token:
            self._meter_accepted_token=None
        if (accepted and self._job is not None
            and self._job.status()["cleaned_cells"] > self._cleaned_at_dispatch):
            self._sweep_tried.clear()
        if (accepted and outcome=="succeeded" and self._auto_continue
            and self._job is not None and self._job.status()["state"]=="running"):
            # Let the 1 Hz external coverage meter publish one final sample.
            self._next_after_succeeded=self._now()+1.2
            self._meter_received_at_success=self._meter.received if self._meter else None
        # Navigation success itself NEVER credits cleaned cells.
        self._publish()

    def _on_fault(self, token: str, reason: str):
        self._frozen = reason
        if self._interlock is not None:
            try:
                self._interlock.on_transport_fault(token=token, reason=reason)
            except JobError as exc:
                self.get_logger().error("fault gate: " + str(exc))
        self._publish()

    def _on_accepted(self, token: str):
        self._accepted_count += 1
        self._meter_accepted_token=token
        self._publish()

    def _freeze(self, reason: str):
        self._frozen = reason
        if self._interlock is not None and self._job.status()["state"] == "running":
            try:
                self._interlock.pause_for_recovery(reason)
            except JobError as exc:
                self.get_logger().error("freeze: " + str(exc))

    def _fresh_pose(self) -> tuple[float, float]:
        if (self._clock_sec is None or self._clock_received is None
            or self._now()-self._clock_received > 3.0):
            raise JobError("stale or missing simulator clock")
        t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if abs(self._clock_sec-stamp) > 2.5:
            raise JobError("stale map-to-robot transform")
        return (t.transform.translation.x, t.transform.translation.y)

    def _ready(self) -> tuple[Raster, Grid, int, Resources]:
        if not self._sim_lab:
            raise JobError("simulation_lab disabled: refuse to send navigation")
        if not self._nav2_ready():
            raise JobError("Nav2 BT navigator lifecycle is not active")
        if self._frozen:
            raise JobError("controller frozen: " + self._frozen)
        if not self._fresh_sim_explicitly_armed:
            raise JobError("fresh disposable simulator not armed")
        if self._map is None:
            raise JobError("map not available")
        if (self._costmap is None or self._costmap_received is None
            or self._now()-self._costmap_received > 6.0):
            raise JobError("global Nav2 costmap missing or stale")
        if (self._battery is None or self._battery_received is None
            or self._now()-self._battery_received > 15):
            raise JobError("battery status missing or stale")
        if self._battery.needed():
            raise JobError("resource hold: " + self._battery.needed())
        if self._coverage_mode=="sim_ground_truth":
            if (self._meter is None or not self._meter.ready
                or self._meter_received is None
                or self._now()-self._meter_received>4.0
                or self._meter_unstable is not False
                or self._meter_stability_received is None
                or self._now()-self._meter_stability_received>4.0):
                raise JobError("ground-truth coverage meter baseline/stability not fresh")
        self._keepout_gate()  # refuse to arm unless mask is lethal in Nav2
        x,y=self._fresh_pose()
        cell=self._map.cell_at(x,y)
        grid = Grid(self._map.width, self._map.height, self._map.data,
                    resolution_m=self._map.resolution,
                    clearance_cells=self._clearance)
        if cell not in grid.traversable:
            raise JobError("localized pose fails static footprint clearance")
        return self._map, grid, cell, self._battery

    def _build_nav_goal(self, goal):
        m=self._map
        if m is None:
            raise JobError("no map")
        return pose_for_cell(
            goal,width=m.width,height=m.height,resolution=m.resolution,
            origin_x=m.origin_x,origin_y=m.origin_y,origin_yaw=m.yaw,
            stamp=self.get_clock().now().to_msg())

    def _dispatch(self, cell: int, *, thru_sweep: bool = False):
        if self._interlock is None or self._port is None or self._map is None:
            raise JobError("no active interlock")
        if not self._port.client.server_is_ready() or not self._nav2_ready():
            raise JobError("Nav2 not lifecycle-active / action server not ready")
        if (self._costmap is None or self._costmap_received is None
            or self._now()-self._costmap_received > 6):
            raise JobError("Nav2 costmap no longer fresh")
        checker=CostmapChecker(self._map,self._costmap, max_cost=self._max_cost)
        keepout=self._keepout_gate()
        def admit(route):
            return (checker.route_admitted(route) and
                    (keepout is None or keepout.accept_route(route)))
        proposed=None
        if thru_sweep and self._coverage_mode=="sim_ground_truth":
            if self._job is None:
                raise JobError("no job for measured sweep")
            remaining=frozenset(set(self._job.state["targets"])-
                                set(self._job.state["cleaned"]))
            try:
                proposed=select_sweep_route(
                    self._job.grid,cell,remaining,
                    admit=admit,
                    previously_tried=frozenset(self._sweep_tried))
            except NoSafeSweep as exc:
                self._freeze("COVERAGE_NO_SAFE_SWEEP")
                raise JobError("no admissible through-sweep goal") from exc
        goal=self._interlock.dispatch(
            pose_cell=cell, admit_route=admit,
            route_override=proposed)
        if proposed is not None:
            self._sweep_tried.add(proposed[-1])
        self._sent_count += 1
        self._cleaned_at_dispatch=self._job.status()["cleaned_cells"]
        return goal.token

    def _attach_job(self, job: CleaningJobs):
        """Attach a job and journaled real Nav2 ActionClient, DISARMED."""
        if self._job is not None or self._port is not None or self._interlock is not None:
            raise JobError("job already attached")
        port=AsyncNav2Transport.from_ros2(self,self._build_nav_goal,
             on_terminal=self._on_terminal,on_fault=self._on_fault,
             on_accepted=self._on_accepted,journal=self._nav2_goal_journal)
        self._job=job
        self._port=port
        self._interlock=NavigationInterlock(job,port,
             coverage_source=("oomwoo_sim_ground_truth_meter"
                              if self._coverage_mode=="sim_ground_truth"
                              else "unconfigured_no_credit"),
             cancel_timeout_s=12)

    def _handle(self, request: dict) -> dict:
        op=request.get("op")
        if not isinstance(op,str) or op not in (
            "arm_fresh_sim", "start", "pause", "resume", "cancel", "status",
            "reconcile_orphan"):
            raise JobError("unknown command")
        if op=="status":
            return {"ok":True, "op":op}
        if op=="arm_fresh_sim":
            if (not self._sim_lab or request.get("isolated_disposable_sim") is not True
                or self._checkpoint.exists() or self._ever_sent_goal
                or self._job is not None or self._interlock is not None
                or self._port is not None):
                raise JobError("arm only allowed with NEW empty checkpoint in disposable isolated sim")
            if self._active_status is not None and any(x in _ACTIVE for x in self._active_status):
                raise JobError("Nav2 already executing another goal")
            self._fresh_sim_explicitly_armed=True
            return {"ok":True, "op":op, "lab_only":True}
        if op=="reconcile_orphan":
            if not self._sim_lab or request.get("isolated_disposable_sim") is not True:
                raise JobError("reconcile permitted only in exclusive disposable sim")
            if (self._job is None or self._interlock is None
                or self._job.status()["state"]!="paused_recovery"):
                raise JobError("no persisted paused-recovery job")
            if self._recovery is not None:
                raise JobError("recovery already started")
            uuid=self._nav2_goal_journal.orphaned_uuid()
            if uuid is None:
                raise JobError("no unresolved durable Nav2 UUID")
            if self._frozen is not None:
                raise JobError("controller frozen; no recovery")
            if not self._nav2_ready():
                raise JobError("Nav2 lifecycle not active")
            self._recovery=AsyncOrphanRecovery(self,uuid,max_wall_s=30.)
            return {"ok":True,"op":op,"phase":"observing_old_uuid"}
        if op=="start":
            if self._job is not None or self._checkpoint.exists():
                raise JobError("one job per checkpoint: restart requires separate reconciliation")
            if request.get("mode")!="spot":
                raise JobError("prototype supports spot target only, not full cleaning")
            job_id=request.get("job_id")
            x=request.get("x"); y=request.get("y")
            if type(x) not in (float,int) or type(y) not in (float,int):
                raise JobError("spot x/y numeric coordinates required")
            m,grid,pose,battery=self._ready()
            spot=m.cell_at(float(x),float(y))
            radius_m=request.get("radius_m",0.0)
            if (type(radius_m) not in (float,int) or not math.isfinite(radius_m)
                or not 0<=radius_m<=0.5):
                raise JobError("spot radius_m must be finite and within 0..0.5m")
            px,py=m.centre(pose);gx,gy=m.centre(spot)
            if math.hypot(gx-px,gy-py)>5.0:
                raise JobError("spot beyond 5m lab test limit")
            checker=CostmapChecker(m,self._costmap,max_cost=self._max_cost)
            # Verify route before committing the new job checkpoint.
            target_cells=grid.targets(mode="spot",spot=spot,radius_m=radius_m)
            route=grid.path_to_nearest(pose,target_cells)
            keepout=self._keepout_gate()
            if not checker.route_admitted(route) or (keepout is not None and
                 not keepout.accept_target(spot)):
                raise JobError("dynamic costmap/keepout rejected proposed target or route")
            job=CleaningJobs(grid,self._checkpoint)
            job.start(job_id,mode="spot",start_cell=pose,spot=spot,
                      radius_m=radius_m,resources=battery)
            self._attach_job(job)
            port=self._port
            if not port.client.server_is_ready():
                job.suspend_recovery("NAV2_SERVER_UNAVAILABLE")
                raise JobError("Nav2 not ready; checkpoint left safely paused")
            self._interlock.confirm_backend_idle(verified_no_outstanding_goals=True)
            self._ever_sent_goal=True
            token=self._dispatch(pose)
            return {"ok":True,"op":op,"goal_token":token}
        if self._interlock is None or self._job is None:
            raise JobError("no active cleaning job")
        if op=="pause":
            self._next_after_succeeded=None
            self._interlock.pause()
            return {"ok":True,"op":op}
        if op=="cancel":
            self._next_after_succeeded=None
            self._interlock.cancel_job()
            return {"ok":True,"op":op}
        if op=="resume":
            _,_,pose,battery=self._ready()
            self._interlock.resume(pose_cell=pose,resources=battery)
            self._next_after_succeeded=None
            self._no_progress_successes=0
            token=self._dispatch(pose, thru_sweep=self._auto_continue)
            return {"ok":True,"op":op,"goal_token":token}
        raise JobError("invalid command")

    def _on_command(self, m):
        try:
            if len(m.data)>2048:
                raise JobError("oversized command")
            request=json.loads(m.data)
            if not isinstance(request,dict):
                raise JobError("command must be JSON object")
            result=self._handle(request)
            result["request_id"]=request.get("request_id")
        except Exception as exc:
            result={"ok":False, "error":str(exc),
                    "op":request.get("op") if isinstance(locals().get("request"),dict) else None,
                    "request_id":request.get("request_id") if isinstance(locals().get("request"),dict) else None}
        self._last_command=result
        self._publish()

    def _keepout_currently_verified(self):
        if not self._require_keepout:
            return False
        try:
            self._keepout_gate()
            return True
        except JobError as exc:
            self._last_keepout_error=str(exc)
            return False

    def _tick(self):
        if self._interlock is not None:
            self._interlock.check_timeout()
        if self._recovery is not None and self._recovery.phase not in ("verified","failed"):
            try:
                if self._recovery.step():
                    if (not self._nav2_ready() or
                        self._map is None or self._costmap is None or
                        self._costmap_received is None or
                        self._now()-self._costmap_received>6 or
                        self._battery is None or self._battery_received is None or
                        self._now()-self._battery_received>15 or
                        self._battery.needed() is not None):
                        raise JobError("recovery confirmed but resources/Nav2 stale")
                    if self._require_keepout:
                        self._keepout_gate()
                    if self._coverage_mode=="sim_ground_truth":
                        if self._meter is None or not self._meter.ready:
                            raise JobError("meter not ready after restart")
                    self._fresh_pose()  # Must also be fresh AFTER quiet proof.
                    if self._job is None or self._interlock is None:
                        raise JobError("job missing in recovery")
                    if self._job.grid.fingerprint()!=self._nav2_goal_journal.read()["map_digest"]:
                        raise JobError("journal map does not match restored checkpoint")
                    self._nav2_goal_journal.mark_reconciled_canceled(
                        self._recovery.witness)
                    self._interlock.confirm_backend_idle(
                        verified_no_outstanding_goals=True)
                    self._fresh_sim_explicitly_armed=True
                    self._recovered_orphan_proof=self._recovery.result()
                    self.get_logger().info("known orphan canceled and stationary; manual resume now allowed")
            except Exception as exc:
                self._frozen="NAV2_ORPHAN_RECONCILE_FAILED"
                self.get_logger().error("recovery refused: "+str(exc))
        if self._recovery is not None and self._recovery.phase=="failed":
            self._frozen="NAV2_ORPHAN_RECONCILE_FAILED"
        if self._job is not None and self._job.status()["state"]=="running":
            if not self._nav2_ready():
                self._freeze("NAV2_LIFECYCLE_NOT_ACTIVE")
            elif self._clock_received is None or self._now()-self._clock_received>3:
                self._freeze("SIM_CLOCK_LOST")
            elif self._costmap_received is None or self._now()-self._costmap_received>6:
                self._freeze("COSTMAP_STALE")
            elif self._battery_received is None or self._now()-self._battery_received>15:
                self._freeze("BATTERY_TELEMETRY_STALE")
            elif self._require_keepout and not self._keepout_currently_verified():
                self._freeze("KEEPOUT_FILTER_UNVERIFIED")
            elif (self._coverage_mode=="sim_ground_truth"
                  and (self._meter_received is None
                       or self._now()-self._meter_received>4.0
                       or self._meter_unstable is not False
                       or self._meter_stability_received is None
                       or self._now()-self._meter_stability_received>4.0)):
                self._freeze("COVERAGE_METER_STALE")
            else:
                try:
                    self._fresh_pose()
                except Exception:
                    self._freeze("LOCALIZATION_STALE")
        if (self._auto_continue and self._next_after_succeeded is not None
            and self._now()>=self._next_after_succeeded
            # Wait until at least ONE post-terminal external meter sample.
            # Do not discard a late 1Hz frame just to issue another goal.
            and (self._meter is None or self._meter_received_at_success is None
                 or self._meter.received>self._meter_received_at_success)):
            self._next_after_succeeded=None
            if (self._job is not None and self._interlock is not None
                and self._interlock.can_dispatch() and self._frozen is None):
                progress=self._job.status()["cleaned_cells"]
                if progress==self._cleaned_at_dispatch:
                    self._no_progress_successes+=1
                else:
                    self._no_progress_successes=0
                if self._no_progress_successes>=self._max_no_progress:
                    self._freeze("COVERAGE_NO_PROGRESS")
                else:
                    try:
                        _,_,pose,_=self._ready()
                        self._dispatch(pose, thru_sweep=True)
                    except Exception as exc:
                        self._freeze("AUTONOMOUS_NEXT_GOAL_FAILED")
                        self.get_logger().warning("next verified goal unavailable: "+str(exc))
        self._publish()

    def snapshot(self) -> dict:
        snap={"experimental":True,"simulation_lab":self._sim_lab,
              "armed_disposable_sim":self._fresh_sim_explicitly_armed,
              "frozen_reason":self._frozen,
              "nav2_lifecycle_required":self._require_nav2_lifecycle,
              "nav2_active":self._nav2_ready(),
              "keepout_filter_required":self._require_keepout,
              "keepout_filter_verified":self._keepout_currently_verified(),
              "keepout_filter_error":self._last_keepout_error,
              "map_ready":self._map is not None,
              "costmap_fresh":self._costmap_received is not None
                  and self._now()-self._costmap_received<=6,
              "clock_fresh":self._clock_received is not None
                  and self._now()-self._clock_received<=3,
              "battery_fresh":self._battery_received is not None
                  and self._now()-self._battery_received<=15,
              "sent_goals":self._sent_count,"accepted_goals":self._accepted_count,
              "unresolved_nav2_uuid":self._nav2_goal_journal.orphaned_uuid(),
              "recovery_phase":self._recovery.phase if self._recovery is not None else None,
              "recovery_error":self._recovery.error if self._recovery is not None else None,
              "recovered_orphan_verified":self._recovered_orphan_proof is not None,
              "last_terminal":self._last_terminal,
              "coverage_mode":self._coverage_mode,
              "coverage_ready":self._meter is not None and self._meter.ready
                   and self._meter_unstable is False,
              "coverage_last_state":self._meter_last_state,
              "coverage_observed_cells":self._meter.observed if self._meter else None,
              "coverage_credited_cells":self._acknowledged_measured_cells,
              "post_success_measured_cells":self._post_goal_measured,
              "auto_continue":self._auto_continue,
              "no_progress_successes":self._no_progress_successes,
              "max_no_progress_attempts":self._max_no_progress,
              "sweep_endpoints_tried":len(self._sweep_tried),
              "last_command":self._last_command}
        if self._interlock is not None:
            snap["interlock"]=self._interlock.snapshot()
        elif self._job is not None:
            snap["job"]=self._job.status()
        else:
            snap["job"]=None
        return snap

    def _publish(self):
        try:
            m=String()
            m.data=json.dumps(self.snapshot(),sort_keys=True)
            self._status_pub.publish(m)
        except Exception as exc:
            self.get_logger().error("failed to publish status: "+str(exc))

    def request_shutdown_cancel(self):
        """Stop requesting motion; caller spins for terminal result before teardown."""
        if self._interlock is not None and self._interlock.snapshot()["active_goal"]:
            if self._job.status()["state"]=="running":
                self._interlock.pause_for_recovery("NODE_SHUTDOWN")
            return True
        return False


def main():
    import sys
    # No action without an explicit named parameter and isolated ROS graph.
    rclpy.init(args=sys.argv)
    node=ExperimentalCleaningNode()
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pending=node.request_shutdown_cancel()
            deadline=time.monotonic()+7
            while pending and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.05)
                pending=node._interlock.snapshot()["active_goal"] is not None
            if pending:
                node.get_logger().error("shutdown with unconfirmed Nav2 cancel: STOP NOT VERIFIED")
        finally:
            executor.shutdown()
            if node._recovery is not None:
                node._recovery.destroy()
            node.destroy_node()
            rclpy.shutdown()


if __name__=="__main__":
    main()
