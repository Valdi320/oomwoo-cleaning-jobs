"""Fail-closed ROS2 lifecycle test: no active BT Navigator means no job goal."""
import tempfile
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter

from oomwoo_cleaning_jobs.ros_experimental_node import ExperimentalCleaningNode
from oomwoo_cleaning_jobs.engine import JobError


def main():
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix="valdi-lifecycle-") as scratch:
        checkpoint=Path(scratch)/"fresh.json"
        node=ExperimentalCleaningNode(parameter_overrides=[
            Parameter("simulation_lab",value=True),
            Parameter("use_sim_time",value=True),
            Parameter("require_nav2_lifecycle",value=True),
            Parameter("checkpoint_path",value=str(checkpoint))])
        try:
            assert not node.snapshot()["nav2_active"]
            assert node._handle({"op":"arm_fresh_sim","isolated_disposable_sim":True})["ok"]
            try:
                node._handle({"op":"start","job_id":"rejected","mode":"spot","x":1.,"y":0.})
                raise AssertionError("goal start improperly accepted before Nav2 active")
            except JobError as exc:
                assert "lifecycle is not active" in str(exc),str(exc)
            assert not checkpoint.exists()
            assert node.snapshot()["sent_goals"]==0
            print("ROS_LIFECYCLE_PROBE PASS: no active BT navigator -> no job or Nav2 command")
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__=="__main__":
    main()
