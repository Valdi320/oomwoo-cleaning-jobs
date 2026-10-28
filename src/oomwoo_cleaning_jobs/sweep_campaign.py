"""Durable sweep campaign shared by whole-floor, selected rooms and manual zones.

IMPORTANT: no navigation success is interpreted as floor coverage. A
separate validated coverage producer must submit fresh measured cell IDs.
No ROS2 dependency; the ROS2 adapter MUST gate each selected route with
fresh Nav2 costmaps and a single Nav2 action owner.

This is a durable orchestration PLAN. It does not control physical motors,
does not infer vacuum suction and does not silently declare complete
when the imported upstream plan omits/misses target cells.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Mapping

from .grid import Grid
from .map_admission import Raster
from .upstream_sweep_adapter import SafeSweepPlan, raster_geometry_digest
from .measured_sweep_selector import MeasuredSweepSelector, SweepChoice


class SweepStateError(RuntimeError):
    """Cannot advance an unverified or incompatible sweep session."""


def _digest(values):
    return hashlib.sha256(
        json.dumps(values, separators=(",",":"),sort_keys=True).encode()).hexdigest()


@dataclass(frozen=True)
class SweepProgress:
    state: str
    eligible_cells: int
    verified_covered_cells: int
    measured_fraction: float
    remaining_cells: int
    remaining_upstream_waypoints: int
    unresolved_upstream_waypoints: int
    service_cycles: int
    revision: int


class SweepCampaign:
    """One writer, one campaign, one durable active action slot.

    The original OOMWOO sweep plan is a geometry source, never a second
    Nav2 motor/action issuer. The caller supplies a verified measured
    OccupancyGrid for choosing candidate destinations, and fresh measured
    deltas from SimCoverageBridge for crediting floor cells.
    """

    SCHEMA=1
    STATES=frozenset(("ready","running","paused","paused_recovery",
                      "manual_review","completed","cancelled"))

    def __init__(
        self, grid:Grid, raster:Raster, plan:SafeSweepPlan,
        path:Path, *, scope:frozenset[int],
        session_id:str,
        skip_when_measured:float=.80,
        max_attempts:int=3,
    ):
        if not isinstance(grid,Grid) or not isinstance(raster,Raster):
            raise SweepStateError("missing map")
        if (not isinstance(plan,SafeSweepPlan)
            or plan.map_digest!=grid.fingerprint()
            or plan.raster_geometry_digest!=raster_geometry_digest(raster)):
            raise SweepStateError("imported plan source map geometry mismatches campaign")
        if (raster.width,raster.height)!=(grid.width,grid.height) or raster.data!=grid.occupancy:
            raise SweepStateError("raster and grid occupancy disagree")
        if (not isinstance(scope,frozenset) or not scope or not scope<=grid.traversable
            or not isinstance(session_id,str) or not re.fullmatch(r"[a-zA-Z0-9_-]{1,64}",session_id)
            or type(max_attempts) is not int or not 1<=max_attempts<=8):
            raise SweepStateError("invalid cleaning scope / identity / attempt budget")
        self.grid=grid
        self.raster=raster
        self.plan=plan
        self.path=Path(path)
        self.scope=scope
        self.session_id=session_id
        self.max_attempts=max_attempts
        self.selector=MeasuredSweepSelector(grid,raster,plan,
                                             skip_when_covered_fraction=skip_when_measured)
        self._metadata={
            "map_digest":grid.fingerprint(),
            "plan_digest":plan.plan_digest,
            "scope_digest":_digest(sorted(scope)),
            "session_id":session_id,
            "max_attempts":max_attempts,
        }
        self._state=None
        if self.path.exists():
            v=self._read()
            # Surviving an unexpected process death never means the
            # previously sent Nav2 goal was cancelled: re-arm externally.
            if v["state"]=="running" or v["active_index"] is not None:
                v={**v,"state":"paused_recovery","reason":"INTERRUPTED_UNKNOWN_GOAL",
                   "revision":v["revision"]+1}
                self._write(v)
            self._state=v
        else:
            state=dict(self._metadata,
                schema=self.SCHEMA,state="ready",reason=None,
                revision=1,covered=[],attempts={},active_index=None,
                active_token=None,credited_before_dispatch=0,
                dispatched_indices=[],service_cycles=0)
            self._write(state)
            self._state=state

    def _validate(self,v):
        if (not isinstance(v,dict)
            or set(v)!=set(self._metadata)|{"schema","state","reason",
              "revision","covered","attempts","active_index",
              "active_token","credited_before_dispatch",
              "dispatched_indices","service_cycles"}
            or any(v.get(k)!=value for k,value in self._metadata.items())
            or type(v.get("schema")) is not int or v["schema"]!=self.SCHEMA
            or not isinstance(v.get("state"),str) or v["state"] not in self.STATES
            or type(v.get("revision")) is not int or v["revision"]<1
            or (v.get("reason") is not None and
                (not isinstance(v["reason"],str) or len(v["reason"])>100))
            or type(v.get("covered")) is not list
            or any(type(i) is not int for i in v["covered"])
            or v["covered"]!=sorted(set(v["covered"]))
            or not set(v["covered"])<=self.scope
            or type(v.get("attempts")) is not dict
            or type(v.get("dispatched_indices")) is not list
            or any(type(i) is not int or not 0<=i<len(self.plan.selected)
                   for i in v["dispatched_indices"])
            or type(v.get("credited_before_dispatch")) is not int
            or v["credited_before_dispatch"]<0
            or type(v.get("service_cycles")) is not int or v["service_cycles"]<0):
            raise SweepStateError("campaign corrupt or original source differs")
        for key,n in v["attempts"].items():
            if (not isinstance(key,str) or not key.isdecimal()
                or not 0<=int(key)<len(self.plan.selected)
                or type(n) is not int or not 1<=n<=self.max_attempts):
                raise SweepStateError("invalid attempt counts")
        index=v["active_index"]
        token=v["active_token"]
        if ((index is None)!=(token is None)
            or (index is not None and
                (type(index) is not int or not 0<=index<len(self.plan.selected)
                 or not isinstance(token,str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}",token)))):
            raise SweepStateError("inconsistent active Nav2 goal slot")
        if v["state"]=="completed" and len(v["covered"])!=len(self.scope):
            raise SweepStateError("completed campaign without verified full scope")
        if v["state"]=="running" and v["reason"] is not None:
            raise SweepStateError("running campaign contains pause reason")
        if v["state"] in ("paused","paused_recovery","manual_review") and not v["reason"]:
            raise SweepStateError("paused/review campaign missing cause")

    def _read(self):
        try:
            v=json.loads(self.path.read_text("utf-8"))
        except (OSError,ValueError) as exc:
            raise SweepStateError("missing/corrupt campaign: refuse Nav2 work") from exc
        self._validate(v)
        return v

    def _write(self,v):
        self._validate(v)
        if self._state is not None:
            disk=self._read()
            if disk["revision"]!=self._state["revision"]:
                raise SweepStateError("another writer modified campaign checkpoint")
        self.path.parent.mkdir(parents=True,exist_ok=True)
        fd,name=tempfile.mkstemp(prefix=".sweep-campaign-",suffix=".tmp",dir=self.path.parent)
        try:
            with os.fdopen(fd,"w",encoding="utf-8") as stream:
                json.dump(v,stream,separators=(",",":"),sort_keys=True)
                stream.write("\n")
                stream.flush();os.fsync(stream.fileno())
            os.replace(name,self.path)
            if hasattr(os,"O_DIRECTORY"):
                dfd=os.open(self.path.parent,os.O_DIRECTORY)
                try:os.fsync(dfd)
                finally:os.close(dfd)
        finally:
            if os.path.exists(name):os.unlink(name)
        self._state=v

    def _transition(self,**changes):
        self._write({**self._state,**changes,"revision":self._state["revision"]+1})

    def summary(self)->SweepProgress:
        s=self._state
        covered=len(s["covered"])
        return SweepProgress(s["state"],len(self.scope),covered,
                covered/len(self.scope),len(self.scope)-covered,
                len(self.plan.selected),len(self.plan.rejected),
                s["service_cycles"],s["revision"])

    @property
    def state(self):
        """Read-only state snapshot for status/test, not state mutation."""
        return json.loads(json.dumps(self._state))

    def start(self):
        if self._state["state"]!="ready":
            raise SweepStateError("new sweep starts only once")
        self._transition(state="running")

    def pause(self,reason="USER_PAUSE"):
        if self._state["state"]!="running":
            raise SweepStateError("pause requires running campaign")
        if not isinstance(reason,str) or not reason:
            raise SweepStateError("invalid pause reason")
        self._transition(state="paused",reason=reason)

    def resumed_after_verified_idle(self, *, idle_verified:bool):
        if idle_verified is not True:
            raise SweepStateError("Nav2 backend is not proven idle")
        if self._state["state"] not in ("paused","paused_recovery","manual_review"):
            raise SweepStateError("resume requires paused/review campaign")
        if self._state["active_index"] is not None:
            raise SweepStateError("old goal token unresolved; confirm terminal first")
        self._transition(state="running",reason=None)

    def reconcile_old_goal(self,*,token:str,terminal_confirmed:bool):
        if self._state["state"]!="paused_recovery":
            raise SweepStateError("old goal reconcile requires recovery state")
        if terminal_confirmed is not True or self._state["active_token"]!=token:
            raise SweepStateError("cannot clear old goal without matched Nav2 terminal")
        # Caller must independently establish terminal result and motion idle.
        self._transition(active_index=None,active_token=None,
                         credited_before_dispatch=len(self._state["covered"]))

    def next_candidate(self,measured:Raster, *,
                       only_scope:frozenset[int]|None=None)->SweepChoice|None:
        if self._state["state"]!="running" or self._state["active_index"] is not None:
            raise SweepStateError("next waypoint blocked by job state/old goal")
        if only_scope is not None:
            if (not isinstance(only_scope,frozenset) or not only_scope
                or not only_scope<=self.scope):
                raise SweepStateError("room target outside selected job scope")
            scope=only_scope
        else:
            scope=self.scope
        exhausted=frozenset(self.plan.selected[int(k)]
            for k,n in self._state["attempts"].items()
            if n>=self.max_attempts)
        return self.selector.select(measured,scope=scope,excluded=exhausted)

    def reserve_goal(self,choice:SweepChoice,token:str):
        if self._state["state"]!="running" or self._state["active_index"] is not None:
            raise SweepStateError("active Nav2 goal cannot be overlapped")
        if not isinstance(choice,SweepChoice) or not 0<=choice.index<len(self.plan.selected):
            raise SweepStateError("invalid candidate index")
        if self.plan.selected[choice.index]!=choice.target_cell:
            raise SweepStateError("candidate and plan mismatch")
        if not isinstance(token,str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}",token):
            raise SweepStateError("invalid goal token")
        key=str(choice.index)
        if self._state["attempts"].get(key,0)>=self.max_attempts:
            raise SweepStateError("retry budget exhausted")
        attempts={**self._state["attempts"],key:self._state["attempts"].get(key,0)+1}
        self._transition(active_index=choice.index,active_token=token,
                         credited_before_dispatch=len(self._state["covered"]),
                         attempts=attempts,
                         dispatched_indices=self._state["dispatched_indices"]+[choice.index])

    def apply_external_coverage(self,cells:tuple[int,...]):
        if self._state["state"]!="running":
            raise SweepStateError("coverage only while active job running")
        if type(cells) is not tuple or not cells or any(type(i) is not int for i in cells):
            raise SweepStateError("must provide verified cell tuple")
        if not set(cells)<=self.scope:
            raise SweepStateError("unscoped external measurement")
        new=sorted(set(self._state["covered"])|set(cells))
        self._transition(covered=new)

    def on_terminal(self,token:str,outcome:str):
        if (self._state["active_token"]!=token or
            outcome not in ("succeeded","canceled","aborted","rejected")):
            raise SweepStateError("unknown/stale/Nav2 nonterminal result")
        # When an EXTERNAL coverage meter has ALREADY proved every
        # required cell, NavigationInterlock intentionally cancels the
        # still-moving Nav2 goal. A terminal CANCELED in that precise case
        # means safe completion, not an interrupted unfinished room.
        if (self._state["state"]=="running"
            and len(self._state["covered"])==len(self.scope)):
            self._transition(state="completed",reason=None,
                             active_index=None,active_token=None)
        elif outcome!="succeeded" and self._state["state"]=="running":
            self._transition(state="paused_recovery",reason="NAV2_"+outcome.upper(),
                             active_index=None,active_token=None)
        else:
            self._transition(active_index=None,active_token=None)

    def record_service(self):
        if self._state["state"]!="paused":
            raise SweepStateError("service counted only for paused job")
        self._transition(service_cycles=self._state["service_cycles"]+1)

    def finalize(self,measured:Raster)->SweepProgress:
        if self._state["state"]!="running" or self._state["active_index"] is not None:
            raise SweepStateError("cannot finalize while motion pending or paused")
        if len(self._state["covered"])==len(self.scope):
            # Every required cell was credited only by a verified external
            # coverage producer. The original upstream path may have
            # rejected waypoints, but there is no unmeasured floor left.
            self._transition(state="completed",reason=None)
        elif self.next_candidate(measured) is not None:
            raise SweepStateError("some upstream waypoints remain to inspect")
        else:
            self._transition(state="manual_review",reason="UNVERIFIED_COVERAGE_GAPS")
        return self.summary()

    def cancel(self):
        if self._state["state"] in ("completed","cancelled"):
            raise SweepStateError("terminal campaign cannot be canceled")
        self._transition(state="cancelled",reason="USER_CANCEL")
