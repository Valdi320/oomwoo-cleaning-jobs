# Dock completion is revocable (isolated simulation only)

## Reproduction and fix

Previously ExclusiveVelocityMux.observe_dock accepted SEARCH(False) -> DONE(True) -> SEARCH(False) without revoking _dock_done/_docked. After quiet odometry, finish_dock could falsely return geometric_docked=True despite the later undocked report. IDLE(False) and contradictory GOTO(True) also retained stale _dock_done.

Now any state/contact contradicting already proven DONE(True) resets the velocity lease to STOP, clears completion evidence and raises DockHandoffError. Repeated DONE(True) stays valid. Legitimate intermediate states before DONE remain valid; retry requires a fresh claim.

The lab ROS2 relay also forwards docked=False after a completed DONE to the pure-Python mux (it previously skipped it). It still waits safely for the initial DONE before separate docked=True.

Reproduce:
  PYTHONPATH=src python3 -m unittest discover -s tests -p test_dock_proof_revocation_20261029.py -v
  PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py'
  PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py'
  git diff --check

Before: three focal failures reproduced against parent SHA 4519fca8dd932262b35ad063aaf05e7be0f47a7c (post-DONE SEARCH, IDLE and GOTO contradictions).

Limits: deterministic pure Python/Linux. ROS2 rclpy callbacks were inspected and edited but not executed in a real ROS2/Gazebo graph. Dock state and docked ROS2 messages are not authenticated. This does not prove physical position, contact, charging, cleaning, or MCU hardware motor STOP. Independent motor safety and physical validation still required.

Compatibility: a contradictory post-DONE message now stops the experimental mux and requires fresh arbitration; unstable ROS2 lab telemetry may need to retry. No motor topics changed. No third-party code modified.

Integration/revert: maintainers may cherry-pick owner commit after replaying ROS2 recorded bags; do not bridge lab output to real /cmd_vel. To revert, use git revert on the owner branch (never force-push). Further TODO: real ROS2/Gazebo replay of separate state/contact callback order and independent hardware interlock.
