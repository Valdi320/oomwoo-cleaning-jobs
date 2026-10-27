# Measured small-spot completion: official OOMWOO meter + real Nav2/Gazebo

Validation date: 2026-10-27. Scope: an isolated OOMWOO One
test_room.world running Gazebo, Nav2 Jazzy and the UNMODIFIED
oomwoo_sim_support/coverage_meter_node (which uses simulator ground-truth
pose to stamp swept cells). Battery inputs in the test harness are synthetic.

## Key result

**3 out of 3 independent, cold-started simulator trials completed all
9/9 eligible cell-centre targets** in a deliberately small test spot.
Each used **two real Nav2 goals**, received measured coverage from
/coverage_meter/covered_grid, and finished only after confirming Nav2's
terminal canceled result for any remaining action.

- No preexisting meter coverage was counted as this job's work.
- Exactly 9 newly marked, in-scope cells were durably acknowledged.
- A goal ending successfully never counted as proof of cleaning.
- The final external meter sample may arrive after NavigateToPose
  returns terminal succeeded. A strictly bounded **3-second post-success
  tail** accepts ONLY independently measured coverage from that last
  completed action, if job remains running and no newer goal has been
  dispatched.
- Pause, cancel, abort, timeout, stale token, changed map, unexpected source
  and new dispatch invalidate that tail.
- Completion does not restart navigation, and the terminal cancel barrier
  remains enforced when there is still an active Nav2 action.

Primary evidence:
[evidence/ros_verified_small_spot_repeats_20261027.json](evidence/ros_verified_small_spot_repeats_20261027.json).
Earlier partial/failing runs are preserved in the local test investigation;
3/3 success is a LIMITED reproducibility sample, not statistical proof of
long-term reliability or generalization to any house layout.

## Failure discovered and resolved

The old executor could end a route with some cells marked 100 in the
official external meter's covered grid, yet keep them "uncovered" in our
job checkpoint. It had assumed a goal must still be actively executing
at the moment the once-per-second meter published its next grid.

That assumption was too strict and caused redundant retries, eventual
COVERAGE_NO_PROGRESS, and results such as 6/9 or 7/9.

The fix explicitly separates:
1. **navigation lifecycle**: goal accepted, succeeded, canceled, etc.;
2. **external measurement lifecycle**: when the independent ground-truth
   coverage grid actually publishes newly swept cells.

The controller can accept a measured sample in a short validated window
after terminal success without ever equating success to cleaning. The
interlock enforces the token, time, map digest, source, job state and
absence of a newer action; full completion is still proofed by the
external grid plus its state checks.

The bounded through-and-beyond target selector additionally drives to
a reachable location BEYOND missed cells so that Nav2's ~0.15m goal
tolerance does not leave the cleaning disk short of a target.
It checks the live costmap footprint and refuses inadmissible routes.
A configurable no-progress bound remains enabled if measured coverage
does not grow.

## Reproduction

See [MEASURED_COVERAGE_AND_AUTO_CONTINUE.md](MEASURED_COVERAGE_AND_AUTO_CONTINUE.md)
for container/ROS2 topic wiring, and
[EXPERIMENTAL_ROS_NODE.md](EXPERIMENTAL_ROS_NODE.md) for controller launch.

With Gazebo, Nav2, ground_truth, coverage_meter and the experimental
controller running **in one private offline ROS_DOMAIN_ID**, from this
checkout mounted read-only at /workspace/valdi-cleaning-jobs:

~~~sh
PYTHONPATH=/workspace/valdi-cleaning-jobs/src \
python3 /workspace/valdi-cleaning-jobs/integration/gazebo_small_spot_probe.py
~~~

For a truly independent trial, discard that dedicated **simulator
container** (not any other containers) and start a new one, with a new
job checkpoint and fresh Nav2. The probe intentionally selects a safe
test spot, requests 100% measured scope completion and checks that
the final action is terminal, not just cancellation-requested.

Additional real ROS2 middleware protocol regression:

~~~sh
PYTHONPATH=/workspace/valdi-cleaning-jobs/src \
python3 /workspace/valdi-cleaning-jobs/integration/ros_delayed_meter_tail_probe.py
~~~

This middleware probe has synthetic coverage and a fake ActionServer.
It is **not** a substitute for the three real Gazebo trials.

## Strict limits

This test does **not** establish whole-room or whole-home coverage,
multi-floor mapping, physical removal of dirt, charging/dock hardware,
MCU cliff/bumper watchdog safety, or a shipping ROS2 public job API.
The upstream coverage meter uses simulator ground truth, not a robot sensor.
An implementation integrated with upstream's existing boustrophedon
coverage planner must be planned with a SINGLE exclusive owner of
Nav2 goals (never two simultaneous autonomous planners).

The 9/9 result proves a defined small, unobstructed zone in the
particular official test_room and only the stated simulator software.
