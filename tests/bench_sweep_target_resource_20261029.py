"""SYNTHETIC ONLY: deterministic sweep target CPU/RAM benchmark, no ROS2/ARM/robot."""
from __future__ import annotations
import hashlib, json, os, platform, resource, statistics, time
from oomwoo_cleaning_jobs.grid import Grid
from oomwoo_cleaning_jobs.sweep_target import select_sweep_route

def main():
    if hasattr(os,"sched_getaffinity"):
        cpus=os.sched_getaffinity(0)
        os.sched_setaffinity(0,{min(cpus)})
        affinity=sorted(os.sched_getaffinity(0))
    else:
        affinity=None
    width,height=120,68
    occupancy=(0,)*(width*height)
    grid=Grid(width,height,occupancy,resolution_m=.05,clearance_cells=1)
    start=5+5*width
    remaining=frozenset(y*width+x for y in range(12,64) for x in range(75,116))
    assert remaining<=grid.traversable
    runs=[]
    for _ in range(3):
        hits=[0]
        def admit(route):
            hits[0]+=1
            return True
        cpu0=time.process_time()
        t0=time.perf_counter()
        route=select_sweep_route(grid,start,remaining,admit=admit)
        elapsed=time.perf_counter()-t0
        cpu=time.process_time()-cpu0
        assert route and set(route)<=grid.traversable
        runs.append({"wall_seconds":round(elapsed,6),"process_cpu_seconds":round(cpu,6),
                     "candidate_admissions":hits[0],
                     "path_length":len(route),
                     "route_sha256":hashlib.sha256(",".join(map(str,route)).encode()).hexdigest()})
    if len({(r["path_length"],r["route_sha256"],r["candidate_admissions"]) for r in runs})!=1:
        raise AssertionError("Non-deterministic result")
    print(json.dumps({"scope":"synthetic Python 120x68 5cm grid, 2132 dirty cells; NO full ROS2, ARM, power or hardware",
       "python":platform.python_version(),"architecture":platform.machine(),
       "affinity":affinity,"runs":runs,
       "median_wall_seconds":round(statistics.median(r["wall_seconds"] for r in runs),6),
       "median_process_cpu_seconds":round(statistics.median(r["process_cpu_seconds"] for r in runs),6),
       "max_rss_kib":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss},sort_keys=True))
if __name__=="__main__":main()
