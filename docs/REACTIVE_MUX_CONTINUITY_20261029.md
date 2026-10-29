# Reactive three-way velocity mux: continuity hardening (2026-10-29)

**State: LOCAL experimental contribution, not pushed; no upstream runtime changes.** Worktree `contrib/reactive-three-way-mux-lab-20261029-1018`, based on source `3f3cd7025ab242564d9a940f15009b2edaabca63`. Existing upstream [PR #79](https://github.com/makerspet/oomwoo/pull/79) is intentionally unchanged while the maintainer reviews its short pointer.

## Two unsafe state-machine assumptions found and corrected

1. **False stationary witness across odometry dropout.** The existing time-window check used the first zero-velocity sample to count quiet time, but did not require continued sampling. An 850 ms gap followed by a zero sample could satisfy a 750 ms "quiet" handoff, even though the robot's motion during the gap was unknown. `observe_odom` now resets the quiet witness if the inter-sample interval exceeds the policy's `max_age_s` (default 300 ms). Every new quiet window must be continuously observed; this is not a claim that samples guarantee physical immobility.
2. **Resurrected Nav2 command after terminal/idle transition.** Previously an ACTIVE→IDLE→ACTIVE sequence using the same goal UUID could re-enable a cached nonzero `Twist`. Now the first IDLE after observed ACTIVE latches a one-way inactive boundary and clears the command. A subsequent ACTIVE with that UUID revokes the lease and raises `ReactiveMuxError`, returning zero on output.

The normal successful ACTIVE→IDLE→exact verified terminal release→fresh quiet→SWEEP handoff still passes. No ROS2 middleware, real velocity publishers, robot hardware, batteries, firmware, or permanent processes were touched.

## Reproduce on the contributor's Linux host

```bash
cd /path/to/oomwoo-cleaning-jobs
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_reactive_mux_handoff_continuity_20261029.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_reactive_velocity_20261029.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
git diff --check
```

**Observed, synthetic Linux x86:** red baseline 3/4 FAIL + 1 PASS; after code fix new focal 4/4 PASS, prior mux 33/33 PASS, whole suite 728/728 PASS normally and with `-O`; whitespace diff PASS. No CI run has been triggered for this local commit.

## Integration and safety limits

The policy trusts its caller's claimed `MotionHealth`, timestamps, Nav2 UUID and source-isolation flags; these are **not authenticated ROS2 proofs**. External adapter must validate topic graph exclusivity, clock/time correspondence, action status and real motor interlock. Rejected Twist or returned zero cannot physically stop another ROS2 publisher. Next substantive milestone: isolated ROS2/Gazebo test-topic adapter with confirmed single-writer topology, fault injection and runtime measurements, before any upstream PR for the new architecture. No ARM minimum or hardware savings may be inferred.
