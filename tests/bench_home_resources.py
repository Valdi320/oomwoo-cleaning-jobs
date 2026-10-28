"""SYNTHETIC ONLY resource benchmark: one deterministic 30x18 three-room program.
Invoked from repo root: PYTHONPATH=src python3 tests/bench_home_resources.py --seed 0 --one-core
Not a Nav2, Gazebo, SBC or physical hardware minimum.
"""
import argparse, json, os, platform, resource, time
from oomwoo_cleaning_jobs.synthetic_home import run_home

def run():
    ap=argparse.ArgumentParser()
    ap.add_argument("--seed",type=int,default=0)
    ap.add_argument("--one-core",action="store_true")
    args=ap.parse_args()
    cpus=None
    if args.one_core and hasattr(os,"sched_getaffinity"):
        cpus=os.sched_getaffinity(0)
        os.sched_setaffinity(0,{min(cpus)})
    cpu_cores=len(os.sched_getaffinity(0)) if hasattr(os,"sched_getaffinity") else None
    before=resource.getrusage(resource.RUSAGE_SELF)
    start=time.perf_counter()
    result=run_home(args.seed)
    after=resource.getrusage(resource.RUSAGE_SELF)
    if not (result.completed and result.eligible==result.covered and result.keepout_entries==0):
        raise RuntimeError("Synthetic suite did not complete safely")
    return {
        "scope":"synthetic three-room Python controller, 30x18 cells; no ROS2/Gazebo/hardware",
        "seed":args.seed,"python":platform.python_version(),
        "architecture":platform.machine(),"affinity_cores":cpu_cores,
        "wall_seconds":round(time.perf_counter()-start,5),
        "user_cpu_seconds":round(after.ru_utime-before.ru_utime,5),
        "system_cpu_seconds":round(after.ru_stime-before.ru_stime,5),
        "max_rss_kib":after.ru_maxrss,
        "result":result.as_dict(),
    }

if __name__=="__main__":
    print(json.dumps(run(),ensure_ascii=False,sort_keys=True))
