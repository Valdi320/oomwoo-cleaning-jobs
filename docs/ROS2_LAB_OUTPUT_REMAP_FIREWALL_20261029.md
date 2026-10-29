# Isolated ROS2 velocity-topic remap firewall — 2026-10-29

**Status:** contributor-owned Apache-2.0 safety hardening of an **experimental** ROS2/Nav2→dock velocity relay. It is **not** an upstream OOMWOO integration, a completed reactive executor, or a motor e-stop. Maintainer's direction: Nav2 for inter-zone transits, docking/localization; room sweeping through a separate reactive `/cmd_vel` executor. No physical device has been connected or flashed.

## Verified defect (before fix)

The existing `ExperimentalDockVelocityRelay` checked the **literal** parameter `output_topic="/valdi/lab_safe_cmd_vel_output"` before creating its ROS2 publisher, but ROS2 launch/CLI remapping can change the **resolved** publisher topic afterward. A real ROS2 Jazzy probe (networkless container, publisher created/destroyed, **no Twist published**) observed:

| ROS2 CLI arguments | Nominal topic | Actual publisher topic |
|---|---|---|
| none | `/valdi/lab_safe_cmd_vel_output` | `/valdi/lab_safe_cmd_vel_output` |
| `--ros-args -r /valdi/lab_safe_cmd_vel_output:=/cmd_vel` | `/valdi/lab_safe_cmd_vel_output` | **`/cmd_vel`** |

An unintended remapping could therefore defeat the nominal parameter allowlist. This matters especially while contributors experiment with velocity ownership and launch files. It does **not** establish that any real robot received commands.

## Change

- New `src/oomwoo_cleaning_jobs/lab_topic_guard.py` checks the actual `rclpy.Publisher.topic_name` against the *one* fully-qualified, isolated test output. Missing, non-string, remapped, relative or malformed names raise `LabTopicGuardError`.
- `ros_dock_velocity_relay.py` invokes that check **immediately after** `create_publisher`, destroys a rejected publisher, and raises before installing input subscriptions or its output timer. The pre-existing parameter allowlist stays in place; this is defense in depth.
- `tests/test_lab_topic_guard_20261029.py`: fake publishers cover accidental motor aliases, namespaces, near-miss names, broken publisher introspection and guard placement.
- `integration/probe_lab_topic_remapping_20261029.py` + `integration/run_lab_output_remap_matrix_20261029.py`: real ROS2 topic-resolution test with five scenarios, no messages emitted. The runner forces `--network none`, `--cap-drop ALL`, nonprivileged read-only container, and `--pull=never`; it refuses to proceed if the image is not cached.

## Reproduce

From the root of **this contributor branch**:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_lab_topic_guard_20261029.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
python3 integration/run_lab_output_remap_matrix_20261029.py
git diff --check
```

The matrix uses the already-cached `docker.io/makerspet/oomwoo:jazzy-dev` image; it does **not** download one, run Gazebo, publish `Twist`, attach hardware or launch the full relay.

**Observed on Bazzite x86_64 / Python 3.14.3:** 10/10 new focused synthetic tests PASS; entire inherited suite 776/776 PASS normally and with `python3 -O`; 5/5 real ROS2 Jazzy publisher/remapping tests PASS. Four remapping/namespace cases plus baseline: unsafe `/cmd_vel`, motor controller alias and unrelated topic were all rejected; exact isolated lab topic remained allowed. Saved machine report: [evidence/lab_ros2_resolved_output_firewall_20261029.json](evidence/lab_ros2_resolved_output_firewall_20261029.json). CI for this **new branch** must be observed separately after publication; local pass does not imply CI PASS.

## Limits and acceptance before upstream integration

This real ROS2 test verifies `rclpy` resolved publisher names and the safety guard, **not full relay startup**, a true action/odom handoff, Gazebo robot motion, global ROS2 graph exclusivity, or hardware braking. The live relay still requires its custom `oomwoo_cleaning_jobs_lab_interfaces` build and full end-to-end launch fault injection. An external ROS2 relay can deliberately copy the lab topic to `/cmd_vel`; source identities and safety witnesses still lack authentication. ROS2 topic whitelisting cannot replace MCU emergency stop, bumper/cliff safety or a physical watchdog. A brief publisher endpoint can exist prior to immediate rejection, but it emits **zero messages** and is destroyed before timer registration.

No published claims of measured SBC minimum, energy reduction, stopping distance, battery safety or robot deployment.

**Review checklist:** [x] genuine remap reproduced before fix; [x] resolved-topic check and cleanup; [x] bad remaps rejected in real ROS2; [x] synthetic full regressions; [ ] full relay startup in isolated ROS2 with rosidl built; [ ] single-writer ROS2 graph audit; [ ] Gazebo fault-injection; [ ] MCU physical interlock and real braking.

**Compatibility/revert:** opt-in experimental relay only; no upstream files, ROS2 launch defaults, Nav2, microcontroller firmware, PCB, batteries or hardware were changed. If this lab safeguard causes trouble, revert only this contributor commit (remove its import and post-creation check, new guard/probes/tests/docs). The [upstream cleaning-jobs PR #79](https://github.com/makerspet/oomwoo/pull/79) must stay unmodified while reviewed; this is a separate safety milestone, with no new maintainer comments.
