"""Persistent single-owner room/whole-map Nav2 campaign binding (simulation only).

The REAL OOMWOO CoveragePlanner is used strictly for offline waypoint
geometry. This adapter owns NO ROS2 publisher or action client. Only the
existing ExperimentalCleaningNode's NavigationInterlock may dispatch Nav2.
Trusted coverage comes exclusively from a validated, accepted-goal
SimCoverageBridge delta and is written FIRST to CoverageWAL.

A "room" is a persisted label of conservative map cells. On test_room,
that is a digitally defined area, NOT proof of a physically walled room.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .engine import CleaningJobs
from .grid import Grid
from .map_admission import Raster
from .multiroom_program import MultiRoomProgram
from .coverage_wal import CoverageWAL
from .upstream_sweep_adapter import adapt_upstream_plan,SafeSweepPlan
from .measured_sweep_selector import SweepChoice
from .gap_audit import GapAuditor,GapAuditError
from .line_gap_route import select_line_through_route
from .sweep_target import NoSafeSweep
from .multiroom_program import RoomProgramError
from .sweep_campaign import SweepStateError
from .line_gap_route import select_line_through_route
from .sweep_target import NoSafeSweep


class RoomBridgeError(RuntimeError):
    """Cannot proceed with this mapped room safely."""


@dataclass(frozen=True)
class CampaignRoute:
    cells:tuple[int,...]
    kind:str
    choice:SweepChoice|None
    room:str
    target:int


def imported_offline_plan(grid:Grid,raster:Raster,source:Path,
                          *,max_shift_m:float=.24)->SafeSweepPlan:
    try:
        doc=json.loads(Path(source).read_text("utf-8"))
        if (doc.get("never_spun_planner") is not True
            or doc.get("no_nav2_goal_sent") is not True
            or doc.get("frame")!="map"
            or doc.get("source_map_size")!=[grid.width,grid.height]
            or abs(doc.get("resolution",0)-raster.resolution)>1e-6
            or not isinstance(doc.get("target_poses"),list)
            or not 1<=len(doc["target_poses"])<=2000):
            raise RoomBridgeError("not an isolated upstream planner geometry export")
        # The source map was exported with the original room-less Grid
        # and a specified conservative clearance, before room labels
        # were added. The original *occupancy* itself remains identical.
        original=Grid(grid.width,grid.height,grid.occupancy,
                      resolution_m=grid.resolution_m,clearance_cells=4)
        if original.fingerprint()!=doc.get("source_map_digest"):
            raise RoomBridgeError("the original map changed after plan export")
        poses=tuple(tuple(p) for p in doc["target_poses"])
        return adapt_upstream_plan(grid,raster,poses,max_shift_m=max_shift_m)
    except (KeyError,ValueError,TypeError,OSError) as e:
        raise RoomBridgeError("upstream source plan missing or malformed") from e


class PersistentRoomBridge:
    """One room campaign + one separate signed-claim external coverage WAL."""

    def __init__(self, grid:Grid,raster:Raster,source_plan:SafeSweepPlan,
                 directory:Path,*,job_id:str,rooms:tuple[str,...]):
        self.grid=grid
        self.raster=raster
        self.job_id=job_id
        self.program=MultiRoomProgram(grid,raster,source_plan,directory,
                        program_id=job_id,selected_rooms=rooms,max_attempts=4)
        self.wal=CoverageWAL(Path(directory)/"verified_coverage.wal",
                    grid=grid,scope=self.program.scope,
                    producer="oomwoo_sim_ground_truth_meter",job_id=job_id)
        self.last_goal_token=None
        self.last_mode=None
        self.last_measured=None
        self.queued_repairs=0
        self._prefer_gap=False
        self._credited_at_dispatch=0
        self._tried_repair_endpoints:set[int]=set()

    def start(self):
        if self.program.campaign.summary().state!="ready":
            raise RoomBridgeError("campaign already started; recovery required")
        self.program.campaign.start()

    def choose(self,measured:Raster,robot_cell:int,admit_route)->CampaignRoute:
        if self.program.campaign.summary().state!="running":
            raise RoomBridgeError("campaign not running")
        if robot_cell not in self.grid.traversable:
            raise RoomBridgeError("robot pose outside current safe grid")
        if not callable(admit_route):
            raise RoomBridgeError("live costmap admission unavailable")
        self.last_measured=measured
        room=self.program.active_room()
        if room is None:
            raise RoomBridgeError("no uncleaned room remains")
        try:
            proposed=(None if self._prefer_gap
                      else self.program.choose_next(measured))
        except SweepStateError as exc:
            raise RoomBridgeError("prior Nav2 goal or campaign recovery blocks a new goal") from exc
        except RoomProgramError:
            proposed=None
        if proposed is not None:
            _,choice=proposed
            target=choice.target_cell
            try:
                route=self.grid.path_to_nearest(robot_cell,frozenset((target,)))
                if admit_route(route) is True:
                    return CampaignRoute(route,"upstream",choice,room,target)
            except Exception:
                pass
        # Before a generic gap corridor, prefer a deterministic segment
        # that passes THROUGH and beyond an unmeasured corner cell.
        # Nav2's terminal-goal tolerance can stop it short otherwise.
        missing=frozenset(self.program.by_room[room]-
                        set(self.program.campaign.state["covered"]))
        if missing:
            try:
                through=select_line_through_route(
                    self.grid,robot_cell,missing,admit=admit_route,
                    previously_tried=frozenset(self._tried_repair_endpoints))
                if admit_route(through) is True:
                    self.queued_repairs+=1
                    return CampaignRoute(through,"repair",None,room,through[-1])
            except NoSafeSweep:
                pass
        # Deliberately do not lower the strict clearance when original
        # plan endpoints cannot be admitted. Repair only with fresh
        # independent evidence of uncovered in-scope cells.
        try:
            audit=GapAuditor(self.grid,self.program.by_room[room],
                             unresolved_upstream=self.program.plan.rejected)
            missing=frozenset(self.program.by_room[room]-
                  set(self.program.campaign.state["covered"]))
            try:
                # Aim THROUGH the missed cells so Nav2's goal tolerance
                # does not stop the robot just before the sweep disk.
                # All candidate grid routes still require CURRENT costmap
                # admission; only the independent meter may credit floor.
                route=select_line_through_route(
                     self.grid,robot_cell,missing,admit=admit_route,
                     previously_tried=frozenset(self._tried_repair_endpoints),
                     overshoot_m=.43,sweep_radius_m=.10)
            except NoSafeSweep:
                route=audit.propose_repair(
                    credited=frozenset(self.program.campaign.state["covered"])
                             &self.program.by_room[room],
                    pose_cell=robot_cell,admit_route=admit_route,
                    already_tried=frozenset(self._tried_repair_endpoints))
            if not route or admit_route(route) is not True:
                raise RoomBridgeError("repair corridor inadmissible")
            self.queued_repairs+=1
            return CampaignRoute(route,"repair",None,room,route[-1])
        except (GapAuditError,ValueError) as exc:
            raise RoomBridgeError("room has unmeasured cells but no safe Nav2 waypoint") from exc

    def after_dispatch(self,route:CampaignRoute,token:str):
        if self.program.campaign.summary().state!="running":
            raise RoomBridgeError("cannot record a Nav2 action in paused campaign")
        if route.choice is not None:
            self.program.campaign.reserve_goal(route.choice,token)
        else:
            self._tried_repair_endpoints.add(route.target)
        self._credited_at_dispatch=self.program.campaign.summary().verified_covered_cells
        self.last_goal_token=token
        self.last_mode=route.kind

    def after_terminal(self,token:str,outcome:str):
        if self.last_goal_token!=token:return
        credited_now=self.program.campaign.summary().verified_covered_cells
        if self.last_mode=="upstream":
            self.program.campaign.on_terminal(token,outcome)
        elif (outcome in ("succeeded","canceled")
              and credited_now==len(self.program.scope)
              and self.program.campaign.summary().state=="running"):
            # The original Nav2 target may have been CANCELED precisely
            # because the independently measured FULL room completed while
            # the route was still active. This is a safe terminal CANCELED,
            # not an unfinished job. Finish BOTH durable ledgers.
            self.program.finish_if_measured(self.last_measured)
        elif outcome!="succeeded" and self.program.campaign.summary().state=="running":
            self.program.campaign.pause("NAV2_"+outcome.upper())
        if self.program.campaign.summary().state=="running":
            # Only a FULL externally acknowledged scope can finish
            # independently of upstream waypoint count/rejections.
            if credited_now==len(self.program.scope):
                self.program.finish_if_measured(self.last_measured)
        if outcome=="succeeded":
            self._prefer_gap=(credited_now==self._credited_at_dispatch)
            if credited_now>self._credited_at_dispatch:
                self._tried_repair_endpoints.clear()
        self.last_goal_token=None
        self.last_mode=None

    def accept_meter_delta(self,*,frame_id:str,cells:tuple[int,...],
                           job:CleaningJobs,verified_active_goal:bool):
        if (verified_active_goal is not True or not isinstance(frame_id,str)
            or not frame_id or not cells):
            raise RoomBridgeError("measurement not bound to verified accepted Nav2 goal")
        if not set(cells)<=self.program.scope:
            raise RoomBridgeError("coverage cells outside room program")
        self.wal.stage(frame_id,cells,externally_validated=True)
        # The caller uses NavigationInterlock.acknowledge_coverage before
        # project(): that is where live action-token and map checks occur.
        return frame_id

    def finish_meter_projection(self,frame_id:str,job:CleaningJobs):
        self.wal.project(frame_id,job,self.program.campaign)
        self.wal.require_drained()

    def status(self):
        summary=self.program.campaign.summary()
        return {"state":summary.state,
                "verified_cells":summary.verified_covered_cells,
                "selected_scope_cells":summary.eligible_cells,
                "coverage_fraction":summary.measured_fraction,
                "active_room":self.program.active_room(),
                "room_progress":[p.__dict__ for p in self.program.progress()],
                "pending_coverage_events":len(self.wal.pending()),
                "source_waypoints":summary.remaining_upstream_waypoints,
                "unresolved_original_waypoints":summary.unresolved_upstream_waypoints,
                "repair_routes_issued":self.queued_repairs,
                "last_route_mode":self.last_mode}
