"""Deterministic headless example. This is NOT a ROS2/Gazebo or hardware run."""
from __future__ import annotations
import json
from pathlib import Path
import tempfile
from . import CleaningJobs, Grid, Resources


def run_demo() -> dict:
    # Two labelled rooms connected by an open corridor (7x5).
    w,h=7,5
    occupancy=[0]*(w*h)
    for y in (0,4):
        for x in range(w): occupancy[y*w+x]=100
    for y in (1,3): occupancy[y*w+3]=100
    room_a=tuple(y*w+x for y in range(1,4) for x in range(1,3))
    room_b=tuple(y*w+x for y in range(1,4) for x in range(4,6))
    g=Grid(w,h,tuple(occupancy),rooms={'bedroom':room_a,'kitchen':room_b},
           keepout=frozenset({1*w+5}))
    start=2*w+1
    with tempfile.TemporaryDirectory() as scratch:
        cp=Path(scratch)/'state.json'
        job=CleaningJobs(g,cp)
        job.start('demo_whole',mode='whole',start_cell=start,
                  resources=Resources(battery_fraction=0.85))
        current=start
        n=0
        # Accept cells only after explicit simulated coverage feedback.
        while job.status()['state']!='completed':
            if n==4:
                job.observe_resources(Resources(battery_fraction=0.15))
                assert job.status()['state']=='paused_service'
                job=CleaningJobs(g,cp)  # demonstrate durable checkpoint across restarts
                job.resume(pose_cell=current,resources=Resources(battery_fraction=0.90))
            route=job.waypoint(current)
            assert all(cell in g.traversable for cell in route)
            current=route[-1]
            job.acknowledge_cleaned((current,))
            n+=1
        result=job.status()
        result.update({'no_go_excluded':1*w+5 not in g.traversable,
                       'all_targets_completed':result['coverage_fraction']==1.0,
                       'example':'headless (no ROS2, no robot)'})
        return result


if __name__=='__main__':
    print(json.dumps(run_demo(),sort_keys=True,indent=2))
