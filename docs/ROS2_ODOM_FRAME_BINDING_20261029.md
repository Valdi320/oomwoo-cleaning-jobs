# Strict odometry frames at experimental ROS2 relay boundary

## Defect / rationale
A timestamp-fresh Odometry message with zero twist in an unrelated
coordinate frame previously passed the lab freshness check. This could
create false quiet-robot evidence for an experimental Nav2/dock handoff.
ROS2 nav_msgs/Odometry semantics are pose in header.frame_id and twist
in child_frame_id. OOMWOO-one documents:
odom -> base_footprint -> base_link.
The lab accepts only header.frame_id = odom AND child_frame_id =
base_footprint. No aliases or normalization.

## Behavior and limits
On wrong/missing frame, revoke odometry freshness, reset mux to STOP,
and do not advance accepted timestamp sequence. After a corrected valid
sample, recovery is possible but ownership must be reclaimed.
All output remains isolated to /valdi/lab_safe_cmd_vel_output.
Frame strings are NOT proof of publisher identity, TF correctness,
positional consistency, covariance, physical braking or hardware safety.
Some publishers legitimately use base_link; those remain refused until
an explicitly reviewed transform adapter exists. No real robot use.

## Reproduce
  PYTHONPATH=src python3 -m unittest discover -s tests -p test_ros_odom_frame_gate_20261029.py -q
  PYTHONPATH=src python3 -m unittest discover -s tests -p test_*.py -q
  PYTHONPATH=src python3 -O -m unittest discover -s tests -p test_*.py -q
  python3 integration/run_ros2_odom_frame_gate_20261029.py

The ROS2 runner inherits the cached Jazzy image, no networking, no
image pull, no devices, no privileges, read-only root and cgroup bounds.
Only zero Odometry is published; relay stays STOP on its lab topic.
Machine results in docs/evidence/ros2_odom_frame_gate_dds_20261029.json.

## Integration / revert
Contributor-owned experimental package; upstream PR #79 untouched.
Reverting this commit reinstates the false quiet-input exposure.
Still required: authenticated ROS2 inputs, TF coherence, integrated
Gazebo handoff, physical watchdog and ARM measurements. Apache-2.0.
