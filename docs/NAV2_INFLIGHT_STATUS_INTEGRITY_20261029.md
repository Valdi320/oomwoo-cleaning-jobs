# Nav2 in-flight fallback: reject mixed, duplicate and UNKNOWN status arrays

**Contributor / scope:** Valdi320-owned experimental Apache-2.0 cleaning-jobs.
**Base:** \`694aaf3bb401a90973d4624fb711d7cb78b4973a\`.
**Upstream link:** https://github.com/makerspet/oomwoo/pull/79

## Failure reproduced before fix

The lab ROS2 node's lifecycle-service fallback checked only whether the
durable UUID appeared ACTIVE using \`any(...)\`. Three synthetic
counterexamples returned true even when the same GoalStatusArray contained
(1) another ACTIVE UUID, (2) a duplicate UUID with terminal+active results,
or (3) an UNKNOWN status for a foreign UUID. Such a malformed packet must
never be used to keep the in-flight Nav2 execution lease healthy.

## Narrow change

\`only_own_nav2_goal_active()\` reuses the existing strict status snapshot
validator, demands exactly one ACTIVE UUID matching the durable journaled
UUID, and tolerates well-formed terminal history of other goals.
The experimental node's bounded service-jitter fallback now uses it.
Normal fresh lifecycle success path and new-goal preflight are unchanged.

## Reproduce

\`PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_nav2_inflight_status_integrity_20261029.py' -q\`

\`PYTHONPATH=src python3 -m unittest discover -s tests -q\`

\`PYTHONPATH=src python3 -O -m unittest discover -s tests -q\`

Results on local Linux: focal **8/8 PASS**,
normal **641/641 PASS**, optimized Python
**641/641 PASS**. Machine evidence:
[JSON](evidence/nav2_inflight_status_integrity_20261029.json).
GitHub Actions must be checked against the exact published commit.

## Integration / rollback / limits

Review and selectively import the two source edits and focused tests into
any authorized isolated ROS2 cleaning node; rerun Nav2/Gazebo and hardware
integration testing separately. If behavior regresses, revert the imported
commit normally. It does not change the public JSON checkpoint schema.

A valid but spoofed/stale ROS2 packet may still be untrusted: freshness is
checked by the caller's local monotonic timestamp, not by this helper.
Nav2 global single-owner arbitration, secure ROS2 transport and independent
MCU safety remain unresolved. This is a **synthetic Linux/Python + AST**
validation only; no actual ROS2/Gazebo, motor, battery or robot was used.
