"""Crash-recovery evidence checker for a SINGLE known Nav2 orphaned goal.

A restarted process no longer owns the old NavigateToPose goal handle.
ROS2 CancelGoal may request cancellation without that handle, but its ACK
does NOT prove the robot stopped. This class requires:
  * the specific old UUID was OBSERVED active;
  * the cancel response named that UUID;
  * the same UUID reached a terminal CANCELED status;
  * a continuously fresh robot pose/velocity record stayed quiet
    for a bounded window while ROS simulation time ADVANCED.

Test/simulator only. This is NOT MCU safety, not a general cancel-all
policy for multi-client robots. Any uncertainty stays unapproved.
"""
from __future__ import annotations
import math


class RestartNotSafe(RuntimeError):
    """Insufficient evidence of canceled old motion after a restart."""


class OrphanNav2Witness:
    ACTIVE = frozenset((1,2,3))
    CANCELED = 5

    def __init__(
        self, expected_goal_hex: str, *,
        min_quiet_sim_s: float = 1.2,
        max_linear_mps: float = .025,
        max_angular_rps: float = .04,
        max_drift_m: float = .07,
        max_gap_sim_s: float = .35,
        min_samples: int = 6,
    ):
        if (not isinstance(expected_goal_hex,str)
            or len(expected_goal_hex)!=32
            or any(c not in "0123456789abcdef" for c in expected_goal_hex)):
            raise ValueError("known Nav2 UUID must be 32 lowercase hex digits")
        values=(min_quiet_sim_s,max_linear_mps,max_angular_rps,
                max_drift_m,max_gap_sim_s)
        if any(type(x) not in (int,float) or not math.isfinite(x)
               or x<=0 for x in values):
            raise ValueError("quiet-window thresholds must be positive finite numbers")
        if type(min_samples) is not int or not 2<=min_samples<=1000:
            raise ValueError("invalid min_samples")
        self.expected=expected_goal_hex
        self.min_quiet=float(min_quiet_sim_s)
        self.max_linear=float(max_linear_mps)
        self.max_angular=float(max_angular_rps)
        self.max_drift=float(max_drift_m)
        self.max_gap=float(max_gap_sim_s)
        self.min_samples=min_samples
        self.active_seen=False
        self.cancel_acknowledged=False
        self.terminal_canceled=False
        self._canceled_seen_pre_ack=False
        self._quiet_start=None
        self._quiet_prev=None
        self._quiet_origin=None
        self._quiet_count=0
        self._quiet_span=0.0
        self._fault=None

    def see_status(self,goal_hex:str,status:int) -> None:
        if goal_hex!=self.expected or self._fault:
            return
        if status in self.ACTIVE and not self.terminal_canceled:
            self.active_seen=True
        elif status==self.CANCELED:
            if not self.active_seen:
                self._fault="terminal observed without known active orphan"
                return
            if not self.cancel_acknowledged:
                # DDS status callbacks may arrive BEFORE the service response,
                # even if the cancellation request was already accepted.
                # Store provisional result, never authorize recovery yet.
                self._canceled_seen_pre_ack=True
                return
            self.terminal_canceled=True
        elif status in (4,6):
            self._fault="old goal did not terminate CANCELED"
        elif status not in (0,1,2,3,4,5,6):
            self._fault="invalid ROS2 goal status"

    def cancel_response(self,return_code:int,canceled_goal_hexes:tuple[str,...]) -> None:
        if self._fault:
            raise RestartNotSafe(self._fault)
        if self.cancel_acknowledged:
            raise RestartNotSafe("duplicate cancel acknowledgment")
        if not self.active_seen:
            raise RestartNotSafe("old goal was never observed active")
        if return_code!=0 or self.expected not in canceled_goal_hexes:
            self._fault="cancel-all response did not name old goal"
            raise RestartNotSafe(self._fault)
        self.cancel_acknowledged=True
        if self._canceled_seen_pre_ack:
            self.terminal_canceled=True

    def observe(self, *, sim_time:float, x:float,y:float,
                linear_mps:float, angular_rps:float) -> None:
        if self._fault:
            return
        values=(sim_time,x,y,linear_mps,angular_rps)
        if any(type(v) not in (float,int) or not math.isfinite(v)
               for v in values):
            self._fault="non-finite position, clock or velocity"
            return
        if not self.terminal_canceled or not self.cancel_acknowledged:
            return
        if (abs(linear_mps)>self.max_linear
            or abs(angular_rps)>self.max_angular):
            self._clear_window()
            return
        if self._quiet_prev is not None:
            dt=sim_time-self._quiet_prev
            if dt<0:
                self._fault="simulation time went backwards"
                return
            if dt>self.max_gap:
                self._clear_window()
        if self._quiet_start is None:
            self._quiet_start=sim_time
            self._quiet_origin=(x,y)
            self._quiet_count=0
        if (math.hypot(x-self._quiet_origin[0],y-self._quiet_origin[1])
                > self.max_drift):
            self._clear_window()
            return
        self._quiet_prev=sim_time
        self._quiet_count+=1
        self._quiet_span=max(0,sim_time-self._quiet_start)

    def _clear_window(self) -> None:
        self._quiet_start=self._quiet_prev=self._quiet_origin=None
        self._quiet_count=0
        self._quiet_span=0.0

    @property
    def ready(self) -> bool:
        return bool(not self._fault and self.active_seen
            and self.cancel_acknowledged and self.terminal_canceled
            and self._quiet_count>=self.min_samples
            and self._quiet_span>=self.min_quiet)

    def require_ready(self) -> dict:
        if not self.ready:
            raise RestartNotSafe(self._fault or "old Nav2 goal is not proven terminal and quiet")
        return {"known_old_goal_uuid":self.expected,
                "observed_active":True,"cancel_response_named_old_goal":True,
                "terminal_canceled":True,"quiet_sim_s":round(self._quiet_span,3),
                "quiet_pose_samples":self._quiet_count,
                "verified_disarmed_stop":True}

    @property
    def fault(self):
        return self._fault
