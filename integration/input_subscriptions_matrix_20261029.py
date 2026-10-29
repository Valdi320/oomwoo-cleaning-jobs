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
def main():
    rows=[]
    for name,rule in CASES.items():
        args=[sys.executable,PROBE,"--case",name]
        if rule:args+=["--ros-args","-r",rule]
        p=subprocess.run(args,capture_output=True,text=True,timeout=17)
        lines=[s.partition("LAB_INPUT_REMAP ")[2] for s in p.stdout.splitlines()
               if s.startswith("LAB_INPUT_REMAP ")]
        try:obj=json.loads(lines[0]) if len(lines)==1 else None
        except ValueError:obj=None
        passed=(p.returncode==0 and isinstance(obj,dict)
                and obj.get("resolved")==obj.get("expected")
                and obj.get("result")==("allowed" if rule is None else "denied_before_timer")
                and obj.get("no_twists_published") is True)
        rows.append({"case":name,"pass":passed,"result":obj,
                     "returncode":p.returncode,"error_tail":"" if passed else
                     (p.stderr[-800:]+p.stdout[-300:])})
    result={"total":len(rows),"pass":sum(int(x["pass"]) for x in rows),
            "cases":rows,"real_ros2":True,"velocity_published":False,
            "hardware":False,"gazebo":False}
    print("LAB_INPUT_MATRIX "+json.dumps(result,sort_keys=True),flush=True)
    if result["pass"]!=result["total"]:sys.exit(1)
if __name__=="__main__":main()
