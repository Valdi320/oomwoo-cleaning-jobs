"""Fresh ROS2 dock_drive geometry witness, NOT a charger or station service.

OOMWOO's upstream dock_drive publishes /dock_drive/state and latched
/dock_drive/docked during its actual LiDAR docking manoeuvre.
These messages are useful to know whether its GEOMETRIC manoeuvre
finished. They are NOT proof of physical charging contacts, vacuum
emptying or mop washing; no such feedback is exposed by that package.

Every observation must belong to an explicit NEW navigation->dock
exclusive-command handoff. Latched old docked True must never be enough.
"""
from __future__ import annotations
import math

class DockGeometryError(RuntimeError):
    pass

class DockGeometryWitness:
    def __init__(self,*,max_sample_age_s:float=.6,quiet_s:float=.8):
        if not .2<=max_sample_age_s<=2.0 or not .4<=quiet_s<=3.:
            raise ValueError("invalid docking witness policy")
        self.max_age=max_sample_age_s
        self.quiet_s=quiet_s
        self.epoch=None
        self.active=False
        self.seen_search=False
        self.seen_done=False
        self.latest_docked=False
        self.latest_docked_at=None
        self.quiet_started=None
        self.latest_odom_at=None
        self._previous_at=None
        self.done_at=None
        self.fail_reason=None

    def _time(self,now):
        if (type(now) not in (int,float) or not math.isfinite(now) or now<0
            or (self._previous_at is not None and now<self._previous_at)):
            self.active=False
            self.fail_reason="invalid or regressing monotonic clock"
            raise DockGeometryError(self.fail_reason)
        self._previous_at=now

    def arm(self,*,now,epoch:str,nav2_terminal:bool,
            isolated_dock_cmd_owner:bool):
        self._time(now)
        if (not isinstance(epoch,str) or not 1<=len(epoch)<=64
            or nav2_terminal is not True
            or isolated_dock_cmd_owner is not True):
            raise DockGeometryError("cannot observe a dock attempt without exclusive motor owner")
        self.epoch=epoch
        self.active=True
        self.seen_search=False
        self.seen_done=False
        self.latest_docked=False
        self.latest_docked_at=None
        self.quiet_started=None
        self.latest_odom_at=None
        self.done_at=None
        self.fail_reason=None

    def state(self,*,now,mode:str):
        self._time(now)
        if not self.active:raise DockGeometryError("no active dock lease")
        if mode=="SEARCH":
            self.seen_search=True
            self.seen_done=False
            self.latest_docked=False
            self.done_at=None
            self.quiet_started=None
        elif mode=="DONE":
            if not self.seen_search:
                raise DockGeometryError("stale dock DONE without a fresh SEARCH")
            self.seen_done=True
            self.done_at=now
        elif mode in ("IDLE","GOTO","TURN","BACK","REGROUP"):
            if mode=="IDLE" and self.seen_search:
                self.active=False
                self.fail_reason="dock returned to idle before verified completion"
            self.seen_done=False
            self.done_at=None
        else:
            raise DockGeometryError("unknown dock drive state")

    def docked(self,*,now,value:bool):
        self._time(now)
        if not self.active or type(value) is not bool:
            raise DockGeometryError("unexpected docked boolean")
        if value and (not self.seen_done or not self.seen_search):
            self.latest_docked=False
            return
        self.latest_docked=value
        self.latest_docked_at=now

    def odom(self,*,now,linear_mps:float,angular_rps:float):
        self._time(now)
        if not self.active:raise DockGeometryError("no active dock lease")
        if (type(linear_mps) not in (float,int) or type(angular_rps) not in (float,int)
            or not math.isfinite(linear_mps) or not math.isfinite(angular_rps)):
            self.fail_reason="invalid odometry"
            self.active=False
            raise DockGeometryError(self.fail_reason)
        self.latest_odom_at=now
        if (self.seen_done and self.latest_docked
            and abs(linear_mps)<=.025 and abs(angular_rps)<=.04):
            if self.quiet_started is None:self.quiet_started=now
        else:
            self.quiet_started=None

    def geometry_confirmed(self,*,now:float)->bool:
        self._time(now)
        return bool(self.active and self.seen_search and self.seen_done
                    and self.latest_docked and self.done_at is not None
                    and self.latest_docked_at is not None
                    and self.latest_docked_at>=self.done_at
                    and self.quiet_started is not None
                    and now-self.quiet_started>=self.quiet_s
                    and self.latest_odom_at is not None
                    and 0<=now-self.latest_odom_at<=self.max_age
                    and self.fail_reason is None)

    def report(self,*,now:float):
        return {"epoch":self.epoch,
                "geometry_docked":self.geometry_confirmed(now=now),
                "physical_station_contact_proven":False,
                "electrical_charging_proven":False,
                "bin_empty_proven":False,
                "mop_wash_proven":False}
