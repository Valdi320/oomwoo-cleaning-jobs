"""Fail-closed exclusive wheel-command handoff between Nav2 and OOMWOO dock_drive.

OOMWOO's upstream dock_drive publishes geometry_msgs/Twist to cmd_vel even
when IDLE (zero velocity). Nav2 also has a cmd_vel chain. Running both
without a verified mux is UNSAFE. This digital contract chooses exactly
one command source at a time and emits zero when any lease expires.

THIS PURE PYTHON COMPONENT DOES NOT CONTROL REAL MOTORS. It must not be
installed as a physical safety watchdog; MCU cliff/bumper/stop remain
independent. ROS2 deployment requires EXPLICIT source remappings and proof
that only the mux publishes to the robot's actual cmd_vel topic.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

class DockHandoffError(RuntimeError):
    pass

@dataclass(frozen=True)
class Velocity:
    linear:float=0.
    angular:float=0.

class ExclusiveVelocityMux:
    """Ephemeral command owner; a process restart always begins STOPPED."""

    STOP="stopped"
    NAV="navigation"
    DOCK="dock_drive"

    def __init__(self,*,max_age_s:float=.30,quiet_required_s:float=.75):
        if (type(max_age_s) not in (float,int) or not .1<=max_age_s<=1.
            or type(quiet_required_s) not in (float,int)
            or not .4<=quiet_required_s<=3.):
            raise ValueError("invalid bounded mux timing")
        self.owner=self.STOP
        self.max_age=float(max_age_s)
        self.quiet_required=float(quiet_required_s)
        self._sent={self.NAV:None,self.DOCK:None}
        self._last_clock=None
        self._quiet_since=None
        self._last_odom=None
        self._dock_enabled=False
        self._dock_started=False
        self._dock_done=False
        self._docked=False
        self._stop_reason="startup: no Nav2 or dock owner verified"

    def _clock(self,now:float):
        if type(now) not in (int,float) or not math.isfinite(now) or now<0:
            raise DockHandoffError("invalid monotonic time")
        if self._last_clock is not None and now<self._last_clock:
            self._stop_reason="backwards clock"
            self.owner=self.STOP
            raise DockHandoffError("clock moved backwards")
        self._last_clock=now

    @property
    def stop_reason(self):
        return self._stop_reason

    def reset(self,reason:str):
        if not isinstance(reason,str) or not reason:
            raise DockHandoffError("invalid stop reason")
        self.owner=self.STOP
        self._stop_reason=reason
        self._sent={self.NAV:None,self.DOCK:None}
        self._dock_enabled=False
        self._dock_started=False
        self._dock_done=False
        self._docked=False
        self._quiet_since=None

    def observe_odom(self,*,now:float,linear_mps:float,angular_rps:float):
        self._clock(now)
        if (type(linear_mps) not in (int,float) or not math.isfinite(linear_mps)
            or type(angular_rps) not in (int,float) or not math.isfinite(angular_rps)):
            self.reset("untrusted odometry")
            raise DockHandoffError("invalid odometry")
        if abs(linear_mps)<=.025 and abs(angular_rps)<=.04:
            if self._quiet_since is None:self._quiet_since=now
        else:
            self._quiet_since=None
        self._last_odom=now

    def _quiet(self,now:float)->bool:
        return (self._quiet_since is not None
                and self._last_odom is not None
                and 0<=now-self._last_odom<=self.max_age
                and now-self._quiet_since>=self.quiet_required)

    def claim_nav(self,*,now:float,nav2_idle:bool,pose_fresh:bool,
                  dock_process_inactive:bool,resources_verified:bool):
        self._clock(now)
        if (self.owner!=self.STOP or nav2_idle is not True
            or pose_fresh is not True or dock_process_inactive is not True
            or resources_verified is not True or not self._quiet(now)):
            raise DockHandoffError("nav owner cannot arm without idle motor and dock proof")
        self.owner=self.NAV
        self._stop_reason=None
        self._sent[self.NAV]=None

    def release_for_dock(self,*,now:float,nav2_terminal:bool,
                         action_uuid_reconciled:bool,
                         nav2_input_stopped:bool):
        self._clock(now)
        if (self.owner!=self.NAV or nav2_terminal is not True
            or action_uuid_reconciled is not True
            or nav2_input_stopped is not True):
            raise DockHandoffError("cannot hand cmd_vel to dock while Nav2 may own it")
        self.reset("Nav2 terminal; wait for observed zero odometry before dock")

    def claim_dock(self,*,now:float,nav2_relinquished:bool,
                   dock_drive_input_isolated:bool):
        self._clock(now)
        if (self.owner!=self.STOP or nav2_relinquished is not True
            or dock_drive_input_isolated is not True or not self._quiet(now)):
            raise DockHandoffError("dock cmd_vel cannot overlap Nav2")
        self.owner=self.DOCK
        # The quiet time BEFORE handing over to the dock cannot prove
        # the robot is still stationary AFTER the docking manoeuvre.
        self._quiet_since=None
        self._last_odom=None
        self._dock_enabled=True
        self._dock_started=False
        self._dock_done=False
        self._docked=False
        self._stop_reason=None
        self._sent[self.DOCK]=None

    def observe_dock(self,*,now:float,drive_state:str,docked:bool):
        self._clock(now)
        if self.owner!=self.DOCK:
            raise DockHandoffError("cannot credit docking while not dock owner")
        if docked is not True and docked is not False:
            raise DockHandoffError("invalid docked message")
        if drive_state=="SEARCH" and not docked:
            self._dock_started=True
        elif drive_state=="DONE" and docked and self._dock_started:
            self._dock_done=True
            self._docked=True
        elif drive_state in ("IDLE","GOTO","TURN","BACK","REGROUP","SEARCH"):
            if not docked:self._docked=False
        else:
            raise DockHandoffError("unrecognized or stale dock confirmation")

    def finish_dock(self,*,now:float,dock_drive_terminated:bool):
        self._clock(now)
        if (self.owner!=self.DOCK or not self._dock_done or not self._docked
            or dock_drive_terminated is not True or not self._quiet(now)):
            raise DockHandoffError("docking geometry/stop not proven")
        self.reset("mechanically docked; station service still unverified")
        return {"geometric_docked":True,
                "electrical_charging_confirmed":False,
                "dustbin_emptied_confirmed":False,
                "mop_washed_confirmed":False}

    def accept_twist(self,source:str,velocity:Velocity,*,now:float):
        self._clock(now)
        if source not in (self.NAV,self.DOCK) or not isinstance(velocity,Velocity):
            raise DockHandoffError("invalid velocity source")
        if any(type(v) not in (int,float) or not math.isfinite(v)
               for v in (velocity.linear,velocity.angular)):
            self.reset("malformed wheel command")
            raise DockHandoffError("nonfinite wheel velocity")
        if abs(velocity.linear)>.55 or abs(velocity.angular)>1.4:
            self.reset("unbounded wheel command")
            raise DockHandoffError("velocity outside experimental limits")
        self._sent[source]=(now,velocity)

    def output(self,*,now:float)->Velocity:
        self._clock(now)
        if self.owner==self.STOP:return Velocity()
        stream=self._sent[self.owner]
        if stream is None:return Velocity()
        stamp,data=stream
        if not 0<=now-stamp<=self.max_age:
            return Velocity()
        return data
