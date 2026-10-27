"""Multi-room cleaning program with ONE measured-credit ledger and ONE Nav2 owner.

Room partition geometry is labelled offline and this program binds a
durable sweep campaign to an ordered set of selected rooms. A room can
be called "completed" only when every eligible cell was independently
measured (not when upstream Nav2 announces arrival).

This is an experimental pure-Python coordinator contract. Real ROS2
movement, dynamic costmap checks, and real hardware dock services are
separate explicitly validated components.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re

from .grid import Grid
from .map_admission import Raster
from .upstream_sweep_adapter import SafeSweepPlan
from .sweep_campaign import SweepCampaign,SweepStateError
from .gap_audit import GapAuditor,GapAudit,GapAuditError
from .dock_cycle import DockCycle,DockCycleError


class RoomProgramError(RuntimeError):
    pass


def _sha(contents):
    return hashlib.sha256(json.dumps(contents,sort_keys=True,separators=(",",":"),
             ensure_ascii=False).encode()).hexdigest()


@dataclass(frozen=True)
class RoomProgress:
    name: str
    total: int
    credited: int
    remaining: int
    verified_complete: bool


class MultiRoomProgram:
    """Selected-room / whole-home campaigns with explicit service pauses."""

    def __init__(self,grid:Grid,raster:Raster,plan:SafeSweepPlan,
                 directory:Path,*,program_id:str,
                 selected_rooms:tuple[str,...]|None=None,
                 max_attempts:int=3):
        if (not isinstance(program_id,str)
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}",program_id)
            or not grid.rooms):
            raise RoomProgramError("invalid program identity / missing room layout")
        if selected_rooms is None:
            selected_rooms=tuple(sorted(grid.rooms))
        if (not isinstance(selected_rooms,tuple) or not selected_rooms
            or len(selected_rooms)>100 or len(set(selected_rooms))!=len(selected_rooms)
            or any(not isinstance(name,str) or name not in grid.rooms
                   for name in selected_rooms)):
            raise RoomProgramError("selected room labels must exist and be unique")
        self.grid=grid;self.raster=raster;self.plan=plan
        self.selected_rooms=selected_rooms
        self.by_room={name:frozenset(grid.rooms[name]) for name in selected_rooms}
        self.scope=frozenset().union(*self.by_room.values())
        if not self.scope or not self.scope<=grid.traversable:
            raise RoomProgramError("selected rooms include occupied or no-go cells")
        self.root=Path(directory)
        self.root.mkdir(parents=True,exist_ok=True)
        self.manifest=self.root/"program.json"
        spec={"schema":1,"program_id":program_id,
              "map_digest":grid.fingerprint(),
              "plan_digest":plan.plan_digest,
              "selected_rooms":list(selected_rooms),
              "rooms_digest":_sha({k:sorted(v) for k,v in self.by_room.items()}),
              "scope_digest":_sha(sorted(self.scope))}
        if self.manifest.exists():
            try:loaded=json.loads(self.manifest.read_text("utf-8"))
            except (OSError,ValueError) as e:
                raise RoomProgramError("stored room program damaged") from e
            if loaded!=spec:
                raise RoomProgramError("program rooms/map/plan changed: refuse restart")
        else:
            from .engine import Checkpoint
            Checkpoint(self.manifest).write(spec)
        self.campaign=SweepCampaign(grid,raster,plan,
             self.root/"sweep.json",scope=self.scope,session_id=program_id,
             max_attempts=max_attempts)
        self.dock=DockCycle(self.root/"dock.json",
             job_id=program_id,map_digest=grid.fingerprint())

    def progress(self)->tuple[RoomProgress,...]:
        credited=set(self.campaign.state["covered"])
        return tuple(RoomProgress(
             name,len(self.by_room[name]),len(self.by_room[name]&credited),
             len(self.by_room[name]-credited),
             self.by_room[name]<=credited)
             for name in self.selected_rooms)

    def active_room(self)->str|None:
        for p in self.progress():
            if p.remaining>0:return p.name
        return None

    def choose_next(self,measured:Raster):
        if self.dock.state["state"] not in ("idle","ready_to_resume"):
            raise RoomProgramError("dock service pending: navigation disabled")
        room=self.active_room()
        if room is None:
            return None
        choice=self.campaign.next_candidate(measured,only_scope=self.by_room[room])
        if choice is None:
            raise RoomProgramError(
                "room contains unverified cells but no safe upstream goal: gap review")
        return (room,choice)

    def measured_delta(self,real_external_verified_cells:tuple[int,...]):
        # Only verified external producer is allowed by caller.
        return self.campaign.apply_external_coverage(real_external_verified_cells)

    def audit_room(self,name:str,pose_cell:int)->GapAudit:
        if name not in self.by_room:
            raise RoomProgramError("unknown room")
        audit=GapAuditor(self.grid,self.by_room[name],
                         unresolved_upstream=self.plan.rejected)
        return audit.audit(credited=frozenset(self.campaign.state["covered"])
                         &self.by_room[name],pose_cell=pose_cell)

    def station_interrupt(self,*,battery_low:bool,bin_full:bool,
                          mop_dirty:bool,goal_terminal:bool):
        if not goal_terminal:
            raise RoomProgramError("must wait for Nav2 terminal before dock")
        if self.campaign.state["state"]!="running":
            raise RoomProgramError("station request requires running program")
        # Durable campaign is paused BEFORE any physical dock service
        # is requested, preventing unsafe continuation on crash.
        self.campaign.pause("DOCK_SERVICE")
        self.dock.request(battery_low=battery_low,bin_full=bin_full,
                          mop_dirty=mop_dirty,job_paused=True,goal_terminal=True)

    def continue_after_service(self,*,nav2_idle_verified:bool,
                               fresh_pose:bool,resources_healthy:bool):
        self.dock.release(nav2_idle_verified=nav2_idle_verified,
                          fresh_pose=fresh_pose,resources_healthy=resources_healthy)
        self.campaign.record_service()
        self.campaign.resumed_after_verified_idle(idle_verified=nav2_idle_verified)

    def finish_if_measured(self,measured:Raster):
        if self.dock.state["state"]!="idle":
            raise RoomProgramError("cannot finalize with pending station cycle")
        status=self.campaign.finalize(measured)
        if status.state=="completed":
            if any(not row.verified_complete for row in self.progress()):
                raise RoomProgramError("room source inconsistent after completion")
        return status
