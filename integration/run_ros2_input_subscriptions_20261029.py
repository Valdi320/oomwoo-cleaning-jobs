"""Networkless, already-cached ROS2 Jazzy lab only."""
import datetime,json,platform,subprocess,sys,time
from pathlib import Path
from run_real_full_relay_startup_20261029 import ARGS as BASE_ARGS
ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=ROOT/"docs/evidence/ros2_lab_input_remap_guard_20261029.json"
ARGS=list(BASE_ARGS)
OLD="/work/integration/full_relay_matrix_container_20261029.py"
NEW="/work/integration/input_subscriptions_matrix_20261029.py"
assert ARGS[-1].count(OLD)==1
ARGS[-1]=ARGS[-1].replace(OLD,NEW)
def main():
    start=time.monotonic()
    try:p=subprocess.run(ARGS,capture_output=True,text=True,timeout=130)
    except (OSError,subprocess.TimeoutExpired) as exc:
        print(json.dumps({"pass":False,"error":str(exc)}));return 1
    marker="LAB_INPUT_MATRIX "
    lines=[s[len(marker):] for s in p.stdout.splitlines() if s.startswith(marker)]
    try:out=json.loads(lines[0]) if len(lines)==1 else None
    except ValueError:out=None
    ok=p.returncode==0 and isinstance(out,dict) and out.get("pass")==9 and out.get("total")==9
    record={"timestamp_utc":datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "host_arch":platform.machine(),"image":"docker.io/makerspet/oomwoo:jazzy-dev",
        "scope":"real ROS2 Jazzy remap resolution, full relay ctor, no Twist, no motors",
        "isolation":"network none, caps all dropped, read-only, no image pull",
        "cpu_limit":3,"memory_limit":"2g","elapsed_s":round(time.monotonic()-start,2),
        "pass":ok,"returncode":p.returncode,"results":out,
        "error_tail":"" if ok else p.stderr[-1400:]+p.stdout[-1400:]}
    EVIDENCE.write_text(json.dumps(record,indent=2,sort_keys=True)+"\n")
    print(json.dumps({"pass":ok,"passed":out.get("pass") if isinstance(out,dict) else 0,
        "total":9,"seconds":record["elapsed_s"],"evidence":str(EVIDENCE),
        "error":record["error_tail"][:1200]},sort_keys=True))
    return 0 if ok else 1
if __name__=="__main__":sys.exit(main())
