# ROS2 Jazzy + real Nav2 + Gazebo cancellation validation

**Date:** 2026-10-27. **Hardware:** none. **Status:** three headless
Gazebo regressions passed on the official OOMWOO test-room map.

Machine-readable evidence:
[evidence/gazebo_nav2_20261027.json](evidence/gazebo_nav2_20261027.json).

## What was actually validated

1. A genuine rclpy ActionClient sends NavigateToPose actions to the
   **real Nav2 BT Navigator**, operating the simulated OOMWOO robot.
2. From a fresh TF map-to-base_link pose, a reachable cell is selected
   in the official test_room map. The Nav2 global/local costmaps handle
   path planning and robot-footprint collision avoidance.
3. The action is accepted, and then the job interlock requests cancel
   while refusing to dispatch a second goal until the Nav2 terminal
   result. A mere cancel acknowledgement is never treated as terminal.
4. The result is **canceled** in all cases. The simulated robot settles
   in odometry, no next goal is sent, and no unverified area is
   credited as cleaned.

| Case | Accepted | Motion before cancel | Terminal | Max displacement after stop (1.2s simulated) | State |
|---|---|---:|---|---:|---|
| User pause during motion | yes | 0.0714 m | canceled | 0.000 m | paused_user |
| Low battery during motion | yes | 0.0714 m | canceled | 0.000 m | paused_service: LOW_BATTERY |
| Cancel before goal acceptance callback | yes | 0.000 m | canceled | 0.03001 m | paused_user |

One measured run per scenario, not a statistical guarantee or physical
safety certification. In the early cancellation scenario the robot moved
~3 cm during the post-cancel window, within the 10 cm acceptance threshold;
a cancel result is not proof of instantaneous mechanical stopping.

A second integration test, integration/ros_action_probe.py, uses
the **real ROS2 middleware and action types** against a harmless
NavigateToPose test ActionServer. That separates the real-client
callback protocol from actual robot movement in Gazebo.

## Reproduce in an isolated Podman container

Requirements: Linux x86_64, Podman, approximately 4 available CPU cores,
8 GiB available RAM and sufficient disk space to extract the official
OOMWOO development image (~9.5 GiB image; potentially more unpacked).
Run from this repository root. Do not connect to real robot hardware.
The shared read-only SELinux mount (:ro,z) is intentional: separate
containers with private :Z labels can relabel the source tree and
prevent an already running container from reading it.

~~~sh
podman pull docker.io/makerspet/oomwoo:jazzy-dev
podman run -d --pull=never --name oomwoo-nav2-validation \
  --network=none --cpus=4 --memory=8g --pids-limit=1300 \
  --shm-size=512m --security-opt=no-new-privileges --cap-drop=ALL \
  -e ROS_DOMAIN_ID=184 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_QPA_PLATFORM=offscreen \
  -e PYTHONPATH=/workspace/valdi-cleaning-jobs/src \
  -v "$PWD":/workspace/valdi-cleaning-jobs:ro,z \
  docker.io/makerspet/oomwoo:jazzy-dev sleep infinity

podman exec -d oomwoo-nav2-validation bash -lc \
  'source /ros_ws/install/setup.bash &&
   exec ros2 launch oomwoo_gazebo world.launch.py \
   world:=/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/worlds/test_room.world \
   x_pose:=0.0 y_pose:=0.0 robot_model:=oomwoo_one \
   headless:=true enable_cameras:=false >/tmp/valdi-world.log 2>&1'

podman exec -d oomwoo-nav2-validation bash -lc \
  'source /ros_ws/install/setup.bash &&
   exec ros2 launch oomwoo_bringup navigation.launch.py \
   robot_model:=oomwoo_one use_sim_time:=true slam:=False \
   localization:=amcl auto_localize:=true x_pose:=0.0 y_pose:=0.0 \
   map:=/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/maps/test_room.yaml \
   >/tmp/valdi-nav.log 2>&1'
~~~

Allow the launch to become ready before testing. The first probe
checks fresh map-to-base_link TF (not a stale AMCL cached pose), a valid
map, and a safe, reachable start and target without moving the robot.

~~~sh
podman exec oomwoo-nav2-validation bash -lc \
  'source /ros_ws/install/setup.bash && python3 \
  /workspace/valdi-cleaning-jobs/integration/gazebo_nav2_probe.py --mode preflight'
~~~

Only once preflight reports ok=true, perform all three cases:

~~~sh
for mode in cancel battery early; do
  podman exec oomwoo-nav2-validation bash -lc \
    "source /ros_ws/install/setup.bash && python3 \
    /workspace/valdi-cleaning-jobs/integration/gazebo_nav2_probe.py --mode $mode"
done
~~~

Each run prints a JSON GAZEBO_NAV2_PROBE record; nonzero exit means failure,
and the reader should inspect details rather than reporting a pass.
Clean up this specific test container once finished:

~~~sh
podman rm -f oomwoo-nav2-validation
~~~

The standalone real ROS2 transport test can also run in the same image:

~~~sh
podman exec oomwoo-nav2-validation bash -lc \
  'source /ros_ws/install/setup.bash && python3 \
  /workspace/valdi-cleaning-jobs/integration/ros_action_probe.py'
~~~

## Limitations / production requirements

- The real ROS2 ActionClient and Gazebo Nav2 cancellation are now tested,
  but this package is not a long-running public ROS2 cleaning-jobs node
  implementing start/pause/resume/cancel/status interfaces.
- The interlock does not physically operate motor safety, cliff, bumper,
  wheel-drop or current limiting. Those must remain independently MCU-owned.
- The test assumes a fresh exclusive simulation container for initial idle
  confirmation. On a real restarted robot, local empty action handles
  **do not** establish that an old Nav2 goal has stopped. Implement
  backend reconciliation before arming.
- Tested on a single official test_room map; not a living-room,
  multi-room, docking or cleaning-completeness regression.
- Production integration still needs footprint-aware admission with live
  keepout costmaps, fresh pose and map-origin checking, trusted measured
  cleaning progress, and agreement on the public job API.

This is a verified cancellation-integration milestone, not a completed
robot or finished cleaning-jobs RFC.
