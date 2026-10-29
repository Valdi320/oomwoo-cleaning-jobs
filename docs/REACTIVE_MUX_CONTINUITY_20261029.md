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

## Callback ordering follow-up (2026-10-29)

An expired lease could survive if fresh odometry, health and command
callbacks ran before the next `output()` call. They replaced the expired
timestamps before the existing watchdog checked them. With a last command
at 1.94 s, callbacks at 2.30/2.31/2.32 s could reuse the same token despite
the 300 ms budget. The red regression reproduces this for NAV, SWEEP and
DOCK independently.

The existing expiry checks now run in the common monotonic-clock path,
before any callback can replace old evidence. Their duplicate code was
removed from `output()`. A missed health, odometry or command deadline
revokes the owner and token at the next public timed operation. Fresh
callbacks alone cannot restore ownership: the caller must reconcile the
backend, provide a new continuously observed quiet interval and claim a
new token. Timely callbacks retain their current token and motion.

No watchdog, node or timer was added. This policy does not execute while
its process is suspended and cannot certify a physical stop during a gap.
The two-way `ExclusiveVelocityMux` and ROS2 adapter are unchanged by this
follow-up; these results apply to `ThreeWayVelocityMux` and its existing
sequenced facade.

Focused host Python 3.14 checks pass, each as a separate module:

```bash
PYTHONPATH=src:tests python3 -m unittest test_reactive_velocity_20261029 -v
PYTHONPATH=src:tests python3 -m unittest test_sequenced_motion_ingress_20261029 -v
PYTHONPATH=src:tests python3 -m unittest test_reactive_mux_handoff_continuity_20261029 -v
PYTHONPATH=src:tests python3 -m unittest test_reactive_keepout_envelope_20261029 -v
```

Results: 42 + 18 + 4 + 22 PASS (86 focused tests, not a full suite).
Seven new test methods cover three owners, separate expired channels,
timely renewal, explicit recovery, and valid forward-sequenced callbacks.
The machine lock was LIBRE before each invocation; processes were scoped
to 4 GiB memory, 512 MiB swap and 50% CPU. Full reproduction output is in
`evidence/reactive_lease_refresh_codex_20261029.json`. No ROS2/DDS, Gazebo,
hardware, CI or physical stopping-distance validation was performed here.


## Local handoff consolidation — 29 October 2026

After the owner stopped the former contributor thread, this private validation
copy becomes the clean combined lab baseline. Its parent remains the colleague's
`b7a0154` resolved-input guard. The already-applied quiet-odometry, battery,
transit-reservation and pre-refresh lease fixes are consolidated locally with
the battery DDS probe and their existing evidence. Seven policy/test/probe
files match the current contributor branch `d455625` byte-for-byte; the relay
also retains b7a0154's input guard. No new runtime behavior is added by this
consolidation. Earlier results remain 121 focused Python passes, 9 resolved
input cases, 7 startup cases and the active battery/UUID DDS probe; these were
not rerun just to create a commit. Provenance is recorded in the guarded
handoff and the existing dated evidence. The original colleague repository,
remote branches and public PRs remain untouched. Three-way runtime integration
is still unfinished, and the owner confirms design/simulation only, no hardware.
