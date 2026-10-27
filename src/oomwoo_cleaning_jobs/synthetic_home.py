"""Deterministic end-to-end, SYNTHETIC multi-room cleaning benchmark.

No Gazebo, Nav2 or real hardware. We simulate a robot traversing 4-neighbour
routes, generate a separate moving cleaning-disk OccupancyGrid, validate
the evidence via SimCoverageBridge, and persist it to MultiRoomProgram.
Inject battery/bin/mop dock-service cycles and one controlled crash/restart.
This bench verifies composition; it is NOT proof a physical floor was cleaned.
"""
from __future__ import annotations

from dataclasses import asdict,dataclass
import json,math,tempfile
from pathlib import Path

from .grid import Grid
from .map_admission import Raster
from .upstream_sweep_adapter import adapt_upstream_plan
from .multiroom_program import MultiRoomProgram,RoomProgramError
from .sim_coverage import SimCoverageBridge
from .gap_audit import GapAuditor,GapAuditError


class ScenarioFailure(RuntimeError):
    pass


@dataclass(frozen=True)
class HomeResult:
    completed:bool
    rooms:tuple[str,...]
    eligible:int
    covered:int
    goals:int
    coverage_frames:int
    dock_cycles:int
    crash_restarts:int
    repaired_routes:int
    keepout_entries:int
    map_cells:int

    def as_dict(self):return asdict(self)


def make_home(seed:int=0):
    """Connected three-zone 30x18 map with seed-specific no-go furniture."""
    if type(seed) is not int or not 0<=seed<=99:raise ValueError("seed 0..99")
    w,h=30,18
    occ=(0,)*(w*h)
    keepout=set()
    for region in range(3):
        x=region*10+5
        y=4+(seed+region*3)%9
        keepout|={y*w+x,(y+1)*w+x}
    for y in range(3,9):keepout.add(y*w+w//2)
    initial=Grid(w,h,occ,keepout=frozenset(keepout),resolution_m=.12)
    names=("Salón","Cocina","Dormitorio")
    rooms={}
    for idx,name in enumerate(names):
        rooms[name]=tuple(sorted(i for i in initial.traversable
                                 if idx*10<=i%w<(idx+1)*10))
    grid=Grid(w,h,occ,rooms=rooms,keepout=frozenset(keepout),resolution_m=.12)
    raster=Raster(w,h,.12,-1.8,-1.08,0.,occ)
    source=tuple(raster.centre(i) for i in sorted(grid.traversable))
    plan=adapt_upstream_plan(grid,raster,source,max_shift_m=.18)
    if plan.rejected:raise ScenarioFailure("synthetic source plan incorrectly rejected safe cells")
    return grid,raster,plan


class DiskMeter:
    def __init__(self,raster):
        self.raster=raster
        self.covered=set()
        self.time=100.
        self.frames=0

    def scan(self):
        self.frames+=1;self.time+=.18
        m=self.raster
        return Raster(m.width,m.height,m.resolution,m.origin_x,m.origin_y,m.yaw,
                      tuple(100 if i in self.covered else 0
                            for i in range(m.width*m.height)))

    def move_cleaner(self,cell):
        m=self.raster
        x,y=m.centre(cell)
        col,row=cell%m.width,cell//m.width
        for yy in range(max(0,row-2),min(m.height,row+3)):
            for xx in range(max(0,col-2),min(m.width,col+3)):
                target=yy*m.width+xx
                a,b=m.centre(target)
                if m.data[target]==0 and math.hypot(a-x,b-y)<=.161:
                    self.covered.add(target)


def run_home(seed:int=0,*,selected_rooms:tuple[str,...]|None=None,
             service_interruptions=True,crash_once=True,
             max_goals:int=1500,workdir:Path|None=None)->HomeResult:
    if type(max_goals) is not int or not 1<=max_goals<=5000:
        raise ValueError("bounded max_goals required")
    if workdir is None:
        with tempfile.TemporaryDirectory(prefix="oomwoo-home-bench-") as d:
            return run_home(seed,selected_rooms=selected_rooms,
                       service_interruptions=service_interruptions,
                       crash_once=crash_once,max_goals=max_goals,workdir=Path(d))
    grid,raster,plan=make_home(seed)
    names=(("Salón","Cocina","Dormitorio")
           if selected_rooms is None else selected_rooms)
    root=Path(workdir)
    program=MultiRoomProgram(grid,raster,plan,root,
                  program_id="scenario_"+str(seed),selected_rooms=names)
    if program.campaign.summary().state=="ready":program.campaign.start()
    meter=DiskMeter(raster)
    bridge=SimCoverageBridge(raster,grid)
    last=meter.scan()
    bridge.consume(last,stamp_s=meter.time,current_sim_s=meter.time,
          simulation_unstable=False,active=False,nav_goal_accepted=False,
          allowed_targets=program.scope)
    position=min(program.scope)
    goals=0;crashes=0;repairs=0;keepout_hits=0;services=set()
    while program.campaign.summary().state=="running":
        if goals>=max_goals:raise ScenarioFailure("bounded navigation budget exceeded")
        room=program.active_room()
        if room is None:
            result=program.finish_if_measured(last)
            if result.state!="completed":
                raise ScenarioFailure("no active room but full completion refused")
            break
        try:
            _,choice=program.choose_next(last)
        except RoomProgramError:
            choice=None
        if choice is None:
            audit=program.audit_room(room,position)
            if not audit.missing_cells:continue
            try:
                route=GapAuditor(grid,program.by_room[room]).propose_repair(
                    credited=frozenset(program.campaign.state["covered"])&
                            program.by_room[room],pose_cell=position,
                    admit_route=lambda path:all(c in grid.traversable for c in path))
            except GapAuditError:
                # The synthetic kinematic bench can deliberately drive
                # exactly to an uncovered centre after a conservative
                # route check. Real Nav2 may stop short: NEVER promote
                # this synthetic fallback into a hardware command.
                left=frozenset(program.by_room[room]-
                               set(program.campaign.state["covered"]))
                route=grid.path_to_nearest(position,left)
            repairs+=1
            token=None
        else:
            route=grid.path_to_nearest(position,frozenset((choice.target_cell,)))
            token="simulated-nav-"+str(goals)
            # Durable reservation BEFORE any synthetic trajectory.
            program.campaign.reserve_goal(choice,token)
            if crash_once and not crashes and goals>=5:
                reboot=MultiRoomProgram(grid,raster,plan,root,
                  program_id="scenario_"+str(seed),selected_rooms=names)
                if reboot.campaign.state["state"]!="paused_recovery":
                    raise ScenarioFailure("simulated crashed Nav2 token was not quarantined")
                reboot.campaign.reconcile_old_goal(token=token,terminal_confirmed=True)
                # This is synthetic test-only idle evidence, not real Nav2.
                reboot.campaign.resumed_after_verified_idle(idle_verified=True)
                program=reboot
                crashes+=1
                continue
        if not route or route[0]!=position or not set(route)<=grid.traversable:
            raise ScenarioFailure("admitted route not safe")
        goals+=1
        for cell in route:
            if cell in grid.keepout:
                keepout_hits+=1
                raise ScenarioFailure("synthetic robot entered no-go cell")
            position=cell
            meter.move_cleaner(cell)
            last=meter.scan()
            fresh=bridge.consume(last,stamp_s=meter.time,
                current_sim_s=meter.time,simulation_unstable=False,
                active=True,nav_goal_accepted=True,allowed_targets=program.scope)
            if fresh.new_cells:program.measured_delta(fresh.new_cells)
        if token is not None:
            program.campaign.on_terminal(token,"succeeded")
        percentage=program.campaign.summary().measured_fraction
        flags=((.22,dict(battery_low=True,bin_full=False,mop_dirty=False)),
               (.53,dict(battery_low=False,bin_full=True,mop_dirty=False)),
               (.78,dict(battery_low=False,bin_full=False,mop_dirty=True)))
        if service_interruptions and program.campaign.state["state"]=="running":
            for n,(threshold,reason) in enumerate(flags):
                if n in services or percentage<threshold:continue
                program.station_interrupt(**reason,goal_terminal=True)
                dock=program.dock
                dock.confirm_dock_arrival(docked=True,nav2_terminal=True,
                                           station_connected=True)
                dock.start_service(station_ack_nonce=dock.state["nonce"],
                                   station_ready=True)
                for observation in range(1,3):
                    dock.observe(battery_fraction=.95,bin_full=False,mop_dirty=False,
                         station_completed=tuple(dock.state["needed"]),
                         station_ack_nonce=dock.state["nonce"],docked=True,
                observation_seq=observation)
                program.continue_after_service(nav2_idle_verified=True,
                       fresh_pose=True,resources_healthy=True)
                services.add(n)
                break
    result=program.campaign.summary()
    return HomeResult(
        completed=result.state=="completed",
        rooms=names,eligible=result.eligible_cells,covered=result.verified_covered_cells,
        goals=goals,coverage_frames=meter.frames,
        dock_cycles=program.dock.state["cycles"],
        crash_restarts=crashes,repaired_routes=repairs,
        keepout_entries=keepout_hits,map_cells=grid.width*grid.height)


if __name__=="__main__":
    import argparse
    p=argparse.ArgumentParser()
    p.add_argument("--seed",type=int,default=0)
    a=p.parse_args()
    data=run_home(a.seed)
    print(json.dumps(data.as_dict(),ensure_ascii=False,sort_keys=True))
    if not data.completed:raise SystemExit(1)
