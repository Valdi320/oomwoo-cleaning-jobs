# Exact-UUID Nav2 cancellation acknowledgement

Owner: Valdi320 experimental `oomwoo-cleaning-jobs` (Apache-2.0). Date: 2026-10-29.

## Blocker and negative evidence

`OrphanNav2Witness.cancel_response` previously accepted `ERROR_NONE (0)` when the old UUID appeared *anywhere* in `goals_canceling`. A canceled *foreign* UUID, the same UUID twice, a Python boolean `False` return code, a list replacing the tuple, or foreign ACK arriving after an early terminal state could pass. Five deterministic pre-fix tests failed as expected; positive exact-UUID control passed.

## Fix / contract

The ROS2 [CancelGoal service](https://github.com/ros2/rcl_interfaces/blob/rolling/action_msgs/srv/CancelGoal.srv) specifies: nonzero goal ID + zero timestamp -> cancel **only** that goal. The experimental adapter sends the exact previous UUID; therefore its successful reply must be `int(0)` and exactly `(known_uuid,)`. On any other shape the witness permanently faults and cannot authorize rearm. Existing active-observation, terminal CANCELED, quiet odometry, clock progression and latest full GoalStatusArray checks remain necessary.

No interface, durable journal schema, ROS2 topic, or runtime dependency changed. On unexpected service behavior, recovery intentionally stays disarmed.

## Reproduction

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_orphan_cancel_reply_exclusive_20261029.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_restart_witness.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
```

Execution counters and limits are saved in [evidence](evidence/nav2_exact_cancel_reply_20261029.json). These are **synthetic Linux tests**, not ROS2/Gazebo execution or physical verification.

## Integration / rollback / maintainer checklist

Apply only this narrow `restart_witness.py` change to the designated **sole** Nav2 action owner after reviewing the precise ROS2 CancelGoal request defaults and response semantics; run physical stop-chain independent checks and real Jazzy/Gazebo cancel/duplicate-packet replay. On a false negative leave the robot disarmed, investigate DDS and cancellation service. Revert the patch via an ordinary git revert if incompatible; never override the blocked recovery on live hardware.

- [ ] Verify CancelGoal's request stamp is zero when exact UUID is set.
- [ ] Replay extra, duplicate, and wrong UUID cancel replies in ROS2 Jazzy.
- [ ] Verify Gazebo no rearm on ambiguous ACK, valid ACK still requires status and quiet window.
- [ ] Obtain maintainer decision on global Nav2 goal ownership and MCU disarm.
- [ ] Keep low-cost ARM SBC profiling as separate follow-up.

No changes to upstream runtime, electronics, PCB, CAD or ROS2 nodes. No claims of full-stack or hardware safety.
