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

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import BatteryState
from action_msgs.msg import GoalStatusArray, GoalStatus
from lifecycle_msgs.srv import GetState
from rosgraph_msgs.msg import Clock

from .engine import CleaningJobs, JobError, Resources
from .grid import Grid, MapError
from .map_admission import Raster, CostmapChecker, AdmissionError
from .nav_interlock import NavigationInterlock
from .ros_nav2 import AsyncNav2Transport, pose_for_cell


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
        self._sim_lab = self.get_parameter("simulation_lab").value is True
        self._require_nav2_lifecycle = (
            self.get_parameter("require_nav2_lifecycle").value is True)
        self._checkpoint = Path(self.get_parameter("checkpoint_path").value)
        self._clearance = int(self.get_parameter("clearance_cells").value)
        self._max_cost = int(self.get_parameter("costmap_max_cost").value)

        self._map: Raster | None = None
        self._map_hash: str | None = None
        self._costmap: Raster | None = None
        self._costmap_received: float | None = None
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
        self._sent_count = 0
        self._accepted_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._on_costmap, QoSProfile(depth=5))
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                 self._on_status, latched)
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
        self._interlock.on_goal_result(token=token, outcome=outcome)
        # Navigation success is NOT proof of measured floor coverage.
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

    def _dispatch(self, cell: int):
        if self._interlock is None or self._port is None or self._map is None:
            raise JobError("no active interlock")
        if not self._port.client.server_is_ready() or not self._nav2_ready():
            raise JobError("Nav2 not lifecycle-active / action server not ready")
        if (self._costmap is None or self._costmap_received is None
            or self._now()-self._costmap_received > 6):
            raise JobError("Nav2 costmap no longer fresh")
        checker=CostmapChecker(self._map,self._costmap, max_cost=self._max_cost)
        goal=self._interlock.dispatch(
            pose_cell=cell, admit_route=checker.route_admitted)
        self._sent_count += 1
        return goal.token

    def _handle(self, request: dict) -> dict:
        op=request.get("op")
        if not isinstance(op,str) or op not in (
            "arm_fresh_sim", "start", "pause", "resume", "cancel", "status"):
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
            px,py=m.centre(pose);gx,gy=m.centre(spot)
            if math.hypot(gx-px,gy-py)>5.0:
                raise JobError("spot beyond 5m lab test limit")
            checker=CostmapChecker(m,self._costmap,max_cost=self._max_cost)
            # Verify route before committing the new job checkpoint.
            route=grid.path_to_nearest(pose,frozenset([spot]))
            if not checker.route_admitted(route):
                raise JobError("dynamic footprint/costmap rejected proposed route")
            job=CleaningJobs(grid,self._checkpoint)
            job.start(job_id,mode="spot",start_cell=pose,spot=spot,
                      radius_m=0.0,resources=battery)
            self._job=job
            port=AsyncNav2Transport.from_ros2(self, self._build_nav_goal,
                 on_terminal=self._on_terminal, on_fault=self._on_fault,
                 on_accepted=self._on_accepted)
            self._port=port
            self._interlock=NavigationInterlock(job,port,coverage_source="unconfigured_no_credit",
                                                cancel_timeout_s=12)
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
            self._interlock.pause()
            return {"ok":True,"op":op}
        if op=="cancel":
            self._interlock.cancel_job()
            return {"ok":True,"op":op}
        if op=="resume":
            _,_,pose,battery=self._ready()
            self._interlock.resume(pose_cell=pose,resources=battery)
            token=self._dispatch(pose)
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

    def _tick(self):
        if self._interlock is not None:
            self._interlock.check_timeout()
        if self._job is not None and self._job.status()["state"]=="running":
            if not self._nav2_ready():
                self._freeze("NAV2_LIFECYCLE_NOT_ACTIVE")
            elif self._clock_received is None or self._now()-self._clock_received>3:
                self._freeze("SIM_CLOCK_LOST")
            elif self._costmap_received is None or self._now()-self._costmap_received>6:
                self._freeze("COSTMAP_STALE")
            elif self._battery_received is None or self._now()-self._battery_received>15:
                self._freeze("BATTERY_TELEMETRY_STALE")
            else:
                try:
                    self._fresh_pose()
                except Exception:
                    self._freeze("LOCALIZATION_STALE")
        self._publish()

    def snapshot(self) -> dict:
        snap={"experimental":True,"simulation_lab":self._sim_lab,
              "armed_disposable_sim":self._fresh_sim_explicitly_armed,
              "frozen_reason":self._frozen,
              "nav2_lifecycle_required":self._require_nav2_lifecycle,
              "nav2_active":self._nav2_ready(),
              "map_ready":self._map is not None,
              "costmap_fresh":self._costmap_received is not None
                  and self._now()-self._costmap_received<=6,
              "clock_fresh":self._clock_received is not None
                  and self._now()-self._clock_received<=3,
              "battery_fresh":self._battery_received is not None
                  and self._now()-self._battery_received<=15,
              "sent_goals":self._sent_count,"accepted_goals":self._accepted_count,
              "last_terminal":self._last_terminal,
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
            node.destroy_node()
            rclpy.shutdown()


if __name__=="__main__":
    main()
