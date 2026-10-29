"""One ROS2 process per subscriber remap; no Twist sent anywhere."""
import json,subprocess,sys
PROBE="/work/integration/probe_ros2_input_topics_20261029.py"
CASES={
 "nominal":None,
 "nav_to_dock":"/valdi/nav2_cmd_vel:=/valdi/dock_drive_cmd_vel",
 "dock_to_nav":"/valdi/dock_drive_cmd_vel:=/valdi/nav2_cmd_vel",
 "nav_to_cmd_vel":"/valdi/nav2_cmd_vel:=/cmd_vel",
 "odom_to_other":"/odom:=/another_odom",
 "battery_to_other":"/battery_state:=/another_battery",
 "status_to_other":"/navigate_to_pose/_action/status:=/another_status",
 "dock_state_to_other":"/dock_drive/state:=/another_state",
 "docked_to_other":"/dock_drive/docked:=/another_docked",
}
THREE_WAY_CASES={'three_nominal': None, 'sweep_to_nav': '/valdi/sweep_cmd_vel:=/valdi/nav2_cmd_vel', 'sweep_state_to_other': '/coverage_planner/motion_state:=/another_phase', 'map_to_other': '/map:=/another_map', 'clock_to_other': '/clock:=/another_clock', 'unstable_to_other': '/coverage_meter/sim_unstable:=/another_stability', 'tf_to_other': '/tf:=/another_tf', 'tf_static_to_other': '/tf_static:=/another_static_tf', 'three_output_to_motor': '/valdi/lab_safe_cmd_vel_output:=/cmd_vel'}
MANAGED_NAV_CASES={"managed_nominal":None,
 "feedback_to_other":"/navigate_to_pose/_action/feedback:=/foreign_feedback",
 "lifecycle_to_other":"/bt_navigator/get_state:=/foreign_lifecycle"}
def main():
    rows=[]
    managed="--managed-nav" in sys.argv
    three_way="--three-way" in sys.argv or managed
    cases=MANAGED_NAV_CASES if managed else THREE_WAY_CASES if three_way else CASES
    for name,rule in cases.items():
        args=[sys.executable,PROBE,"--case",name]
        if three_way:args.append("--three-way")
        if managed:args.append("--managed-nav")
        if rule:args+=["--ros-args","-r",rule]
        p=subprocess.run(args,capture_output=True,text=True,timeout=17)
        lines=[s.partition("LAB_INPUT_REMAP ")[2] for s in p.stdout.splitlines()
               if s.startswith("LAB_INPUT_REMAP ")]
        try:obj=json.loads(lines[0]) if len(lines)==1 else None
        except ValueError:obj=None
        passed=(p.returncode==0 and isinstance(obj,dict)
                and obj.get("resolved")==obj.get("expected")
                and obj.get("result")==("allowed" if rule is None else "denied_before_timer")
                and obj.get("no_twists_published") is True
                and obj.get("three_way") is three_way
                and obj.get("managed_nav2") is managed)
        rows.append({"case":name,"pass":passed,"result":obj,
                     "returncode":p.returncode,"error_tail":"" if passed else
                     (p.stderr[-800:]+p.stdout[-300:])})
    result={"total":len(rows),"pass":sum(int(x["pass"]) for x in rows),
            "cases":rows,"real_ros2":True,"velocity_published":False,
            "hardware":False,"gazebo":False}
    print("LAB_INPUT_MATRIX "+json.dumps(result,sort_keys=True),flush=True)
    if result["pass"]!=result["total"]:sys.exit(1)
if __name__=="__main__":main()
