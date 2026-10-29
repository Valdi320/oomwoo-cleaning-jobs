# Resolved ROS2 input source isolation (experimental lab only)

Defect: even with the output publisher pinned to /valdi/lab_safe_cmd_vel_output,
ROS2 remapping of the NAV subscription onto the DOCK subscription topic causes
two nominally distinct source callbacks to receive the same input.
A similar remap on /odom or Nav2 status can invalidate safety-witness meaning.

Fix: inspect the actual seven rclpy Subscription.topic_name properties before
creating any control service or output timer. Refuse missing, remapped, repeated
or uninspectable handles and destroy subscriptions on failure.

Reproduce from the source commit:
- PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_lab_input_topic_guard_20261029.py' -q
- PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
- PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
- python3 integration/run_ros2_input_subscriptions_20261029.py

The ROS2 probe uses pre-cached official makerspet/oomwoo:jazzy-dev in an
offline unprivileged Podman container (2 GiB/3 CPU cgroup), and nine independent
middleware remap cases. No Twist messages are published. Evidence in
docs/evidence/ros2_lab_input_remap_guard_20261029.json.

Limits: exact names are NOT publisher authentication; a malicious publisher on
the correct input can still spoof data. Not Gazebo, hardware, MCU watchdog or
the maintainer's intended three-way reactive sweep executor. Any launch that
intentionally remaps these seven topics must now be reviewed/reworked.
Existing unmapped lab configurations retain their safety STOP default.
No upstream files modified; reverting this single private commit restores old
lab-only behavior. Leave upstream pointer PR #79 untouched during review.
