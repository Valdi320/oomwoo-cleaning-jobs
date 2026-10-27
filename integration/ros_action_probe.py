"""Real ROS2 NavigateToPose ActionClient↔ActionServer integration test.

Uses ROS2 middleware and real action types, but a harmless fake action server.
Not an official Nav2/Gazebo motion validation; that is a separate next gate.
"""
from __future__ import annotations
import json
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose

from oomwoo_cleaning_jobs import CleaningJobs, Grid, Resources
from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock
from oomwoo_cleaning_jobs.ros_nav2 import AsyncNav2Transport, pose_for_cell


class TestNav2Server(Node):
    def __init__(self):
        super().__init__("oomwoo_action_test_server")
        self.started = threading.Event()
        self.cancel_seen = threading.Event()
        self.active = 0
        self.lock = threading.Lock()
        self.server = ActionServer(
            self, NavigateToPose, "/navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda h: CancelResponse.ACCEPT,
        )

    def execute(self, handle):
        with self.lock:
            self.active += 1
        self.started.set()
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    self.cancel_seen.set()
                    handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(.025)
            handle.abort()
            return NavigateToPose.Result()
        finally:
            with self.lock:
                self.active -= 1


def wait_for(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.02)
    return False


def main():
    rclpy.init()
    server = TestNav2Server()
    client_node = Node("oomwoo_action_test_client")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        with tempfile.TemporaryDirectory() as temp:
            grid = Grid(12, 12, tuple([0] * 144))
            job = CleaningJobs(grid, Path(temp) / "job.json")
            job.start("ros_middleware_smoke", mode="spot", start_cell=0,
                      spot=143, radius_m=0.0)
            events, faults = [], []
            gate_box = {}

            def terminal(token, outcome):
                events.append((token, outcome))
                gate_box["gate"].on_goal_result(token=token, outcome=outcome)

            def fault(token, reason):
                faults.append((token, reason))
                gate_box["gate"].on_transport_fault(token=token, reason=reason)

            port = AsyncNav2Transport.from_ros2(
                client_node,
                lambda g: pose_for_cell(
                    g, width=12, height=12, resolution=0.15,
                    origin_x=-0.9, origin_y=-0.9, stamp=client_node.get_clock().now().to_msg()),
                on_terminal=terminal, on_fault=fault,
            )
            gate = NavigationInterlock(job, port, coverage_source="real_swept_area")
            gate_box["gate"] = gate
            assert port.wait_for_server(8.0), "ROS2 ActionServer not discovered"
            assert server.active == 0
            gate.confirm_backend_idle(verified_no_outstanding_goals=True)
            goal = gate.dispatch(pose_cell=0, admit_route=lambda cells: True)
            assert server.started.wait(timeout=8), "real ROS2 server never received goal"
            assert not gate.can_dispatch()
            gate.pause()
            assert not gate.can_dispatch(), "pause unexpectedly rearmed goal"
            assert server.cancel_seen.wait(timeout=8), "real server never saw cancel"
            assert wait_for(lambda: len(events)==1, 8), "no terminal cancel from action server"
            assert events[0] == (goal.token, "canceled"), events
            assert wait_for(lambda: server.active==0, 2)
            assert gate.snapshot()["active_goal"] is None
            assert job.status()["state"] == "paused_user"
            assert port.outstanding_tokens() == ()
            assert not faults, faults
            gate.resume(pose_cell=0, resources=Resources(.8))
            assert gate.can_dispatch()
            # No automatic new goal: explicit dispatch is required.
            assert len(events)==1
            result={
                "ok":True,
                "real_ros2_action":"/navigate_to_pose",
                "goal_accepted_and_executed":True,
                "cancel_request_received_by_action_server":True,
                "terminal_outcome":events[0][1],
                "interlock_disarmed_during_cancel":True,
                "new_goal_requires_explicit_dispatch":True,
                "faults":faults,
            }
            print("ROS_ACTION_PROBE "+json.dumps(result,sort_keys=True), flush=True)
    finally:
        executor.shutdown()
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=5)


if __name__ == "__main__":
    main()
