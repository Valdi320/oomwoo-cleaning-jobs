"""Conservative physical-coverage admission contract; NO physical deployment.

ROS2 Odometry and pose history alone do NOT prove cleaning. Any credit
requires a trusted upstream estimator, positive and recent actuator
telemetry, localization, and an active, accepted navigation goal.
This policy only screens already measured cell IDs; no fake cells are
generated and no motor commands are issued.

Upstream robot firmware/sensor trust remains to be implemented. Unlike
the Gazebo ground-truth meter this cannot establish physical dirt removal
without independent vacuum / mop actuation measurements.
"""
from __future__ import annotations
from dataclasses import dataclass
import math


class PhysicalCoverageNotVerified(RuntimeError):
    """Do not persist any physical cleaning credit."""


@dataclass(frozen=True)
class ActuatorReading:
    stamp_s:float
    vacuum_enabled:bool
    vacuum_current_a:float
    mop_enabled:bool
    mop_engaged:bool


@dataclass(frozen=True)
class LocalizationReading:
    stamp_s:float
    frame:str
    position_stddev_m:float
    yaw_stddev_rad:float


class PhysicalCoverageAdmission:
    """No physical credit from stale, uncertain, off or conflicting inputs."""

    def __init__(self, *, max_age_s:float=.65,
                 max_position_stddev_m:float=.09,
                 max_yaw_stddev_rad:float=.17,
                 minimum_vacuum_current_a:float=.12):
        vals=(max_age_s,max_position_stddev_m,max_yaw_stddev_rad,
              minimum_vacuum_current_a)
        if any(type(v) not in (float,int) or not math.isfinite(v) or v<=0
               for v in vals):
            raise ValueError("invalid robot cleaning admission limits")
        self.max_age=float(max_age_s)
        self.max_pos=float(max_position_stddev_m)
        self.max_yaw=float(max_yaw_stddev_rad)
        self.min_current=float(minimum_vacuum_current_a)

    def admit(self, *,now_s:float,cells:tuple[int,...],actuator:ActuatorReading,
              localization:LocalizationReading,goal_accepted:bool,
              job_running:bool,mode:str,
              trusted_coverage_source:bool,source_map_matches:bool) -> tuple[int,...]:
        if (type(now_s) not in (int,float) or not math.isfinite(now_s)
            or type(cells) is not tuple or not cells
            or any(type(i) is not int or i<0 for i in cells)
            or len(cells)>10000 or len(set(cells))!=len(cells)):
            raise PhysicalCoverageNotVerified("invalid measured cells/robot clock")
        if (not isinstance(actuator,ActuatorReading)
            or not isinstance(localization,LocalizationReading)
            or trusted_coverage_source is not True
            or source_map_matches is not True
            or goal_accepted is not True or job_running is not True):
            raise PhysicalCoverageNotVerified("untrusted source, map or inactive cleaning job")
        if mode not in ("vacuum","mop","vacuum_and_mop"):
            raise PhysicalCoverageNotVerified("unknown cleaning actuator mode")
        timestamps=(actuator.stamp_s,localization.stamp_s)
        if any(type(t) not in (int,float) or not math.isfinite(t)
               or not 0<=now_s-t<=self.max_age for t in timestamps):
            raise PhysicalCoverageNotVerified("stale/future cleaning or localization telemetry")
        if localization.frame!="map":
            raise PhysicalCoverageNotVerified("pose not in known map frame")
        noise=(localization.position_stddev_m,localization.yaw_stddev_rad)
        if any(type(v) not in (float,int) or not math.isfinite(v) or v<0 for v in noise):
            raise PhysicalCoverageNotVerified("invalid localization uncertainty")
        if noise[0]>self.max_pos or noise[1]>self.max_yaw:
            raise PhysicalCoverageNotVerified("too uncertain about robot position")
        vac=mode in ("vacuum","vacuum_and_mop")
        mop=mode in ("mop","vacuum_and_mop")
        if vac and (actuator.vacuum_enabled is not True
                    or type(actuator.vacuum_current_a) not in (float,int)
                    or not math.isfinite(actuator.vacuum_current_a)
                    or actuator.vacuum_current_a<self.min_current):
            raise PhysicalCoverageNotVerified("vacuum power not independently confirmed")
        if mop and (actuator.mop_enabled is not True
                    or actuator.mop_engaged is not True):
            raise PhysicalCoverageNotVerified("mop actuator/contact not independently confirmed")
        return cells
