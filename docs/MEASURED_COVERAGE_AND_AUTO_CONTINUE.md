# Measured coverage + automatic next action (Phase 4)

**2026-10-27.** A simulation-first incremental contribution to OOMWOO
cleaning-jobs. The ROS2 controller can now consume OOMWOO's external coverage
measurement, persist newly swept cells, and schedule a bounded follow-up
navigation action. It is NOT a complete multi-room cleaning orchestrator.

## Real upstream interfaces, verified from the source

The unmodified upstream
[oomwoo_sim_support/coverage_meter_node.py](https://github.com/makerspet/oomwoo-ros2-tools/blob/jazzy/src/oomwoo_sim_support/oomwoo_sim_support/coverage_meter_node.py)
publishes:

| Topic | Type | Actual semantics |
|---|---|---|
| /coverage_meter/covered_grid | nav_msgs/msg/OccupancyGrid | 100 for swept free cells, 0 otherwise; from simulator ground-truth pose + cleaning-disk geometry |
| /coverage_meter/sim_unstable | std_msgs/msg/Bool | True if suspicious ground-truth pose teleport makes simulated coverage unreliable |
| /coverage_meter/ratio | std_msgs/msg/Float32 | Whole-map cleanable area covered (not a room/job-specific ratio) |

The meter stamps the disk **even before its cleaning_active flag is true**:
cleaning_active only gates efficiency/path-length accounting.
This matters: a job manager must not credit the preexisting disk as if
its own job just cleaned that floor.

It is also NOT a physical dirt sensor or vacuum-power confirmation.
The grid comes from the simulator's ground-truth pose and assumes an
effective simulated cleaning disk. Hardware must have a separately
evaluated pose-based coverage estimator and actuator-state gating.

## What is implemented

### New source: src/oomwoo_cleaning_jobs/sim_coverage.py

SimCoverageBridge is a standard-library, testable delta adapter:
1. Requires an explicit stable meter signal, matching map frame,
   dimensions, resolution, origin and yaw; a recent monotonic ROS sim stamp.
2. Takes the first snapshot as a **baseline**: zero new job credit.
3. Only newly flipped (0-to-100) cells may count, intersected with
   the active job's eligible targets.
4. No credit if the job is paused, the action goal has not actually been
   accepted, or the expected action token is no longer active. Floor swept
   during a pause becomes part of the next baseline, not future credit.
5. Rejects invalid/unknown/occupied-map coverage, non-0/100 values,
   time rollback, meter reset/bit reversal, future/stale samples or a
   simulator-unstable report.
6. Credits durable CleaningJobs checkpoints ONLY via
   NavigationInterlock.acknowledge_coverage. A Nav2 "succeeded" result
   independently **does not** count any floor.

Mapped-free fringe can be swept by the cleaning disk although the robot's
centre cannot safely enter it; such cells are accepted as a measurement
but cannot become a centre-cell goal or be credited outside the scope.

### Experimental ROS2 integration

The controller opts in with:
- simulation_lab:=true (still refuses physical use)
- coverage_mode:=sim_ground_truth
- auto_continue:=true

It subscribes to the two official meter topics and refuses a job start
until a fresh baseline and stability status exist. The meter staying silent
or becoming unstable during motion pauses and requests Nav2 cancellation.

Spot commands may set radius_m between 0 and 0.5 metres. Only
**small experimental spots**, not whole-home room cleaning, are supported
by this ROS2 adapter.

After a terminal Nav2 success with uncleaned eligible cells remaining,
the node waits for an external measurement frame, then may issue the next
admissible goal, subject to new TF/costmap/battery and lifecycle checks.
A bounded, configurable number of successful arrivals without
externally measured progress (default four; two in the synthetic
no-progress test) triggers COVERAGE_NO_PROGRESS, not an endless loop.
After Nav2 terminal succeeded, a **3-second token-fenced measurement
tail** accepts a late upstream 1 Hz coverage frame while the job is still
running; this never grants credit for Nav2 success alone.

This uses a simple nearest-uncovered-cell prototype. The production
full-floor executor should reuse upstream oomwoo_coverage's
boustrophedon planner instead of replacing it with a naive one-cell search.
Only one executor may own Nav2 actions at a time.

## Results actually measured

**A. Real OOMWOO Gazebo + REAL Nav2 + ORIGINAL coverage_meter**

[Machine evidence](evidence/ros_real_meter_20261027.json)

- The meter had 29 covered cells **before** our test job: job credit was 0.
- Job scope was a 0.24-metre radius spot containing 69 eligible centre cells.
- A new externally marked cell was durably credited (1/69).
- Synthetic 4% battery input paused the job AFTER that credit; the
  controller canceled the Nav2 goal and preserved coverage across resume.
- Synthetic battery was restored and navigation resumed. There were
  3 accepted Nav2 goals; the test ended with explicit cancellation.
- Outcome: PASS for measured-data ingestion, checkpoint retention,
  resource interruption and resume. NOT PASS for full spot completeness:
  only 1/69 target cells was verified swept in this run.

The official test_room has no functional BatteryState publisher. The
battery values were explicitly **synthetic ROS2 test input**. The
coverage grid itself came from the unmodified upstream ground-truth
meter, NOT a hand-authored fake grid.

**B. Complete small-spot validation with original Gazebo coverage meter**

See [SMALL_SPOT_MEASURED_COMPLETION.md](SMALL_SPOT_MEASURED_COMPLETION.md)
and [three independently restarted successful runs](evidence/ros_verified_small_spot_repeats_20261027.json).

- Exactly **9/9 measured cell-centre targets** completed in **3/3** cold
  simulator runs, two real Nav2 goals per run, verified terminal cancel.
- The original failure was that the upstream 1 Hz meter could report
  the last swept cells *after* Nav2 said goal succeeded. The prior code
  discarded those samples, leaving cells marked 100 by the meter but 0
  in our checkpoint.
- A constrained 3 s post-success tail, with token, map, source,
  job-state and no-new-goal fences, solves this without inventing credit.
- The selector also drives through/beyond dirty centres to compensate for
  Nav2 goal tolerance. It still observes real costmap footprint checks.
- The results establish **one 9-cell spot only**, not complete rooms or
  physical vacuuming. Initial failures are part of the diagnostic record.

**C. Real ROS2 middleware; synthetic meter + fake ActionServer**

integration/ros_verified_meter_probe.py:

- First NavigateToPose goal reports succeeded, but there is NO cleaning
  credit without external evidence.
- The scheduler submits a second action automatically.
- The fake meter then marks one new eligible target cell; the job
  transitions to completed, the active Nav2 action is canceled,
  and the preexisting baseline cell is never credited.
- This validates the state/action/coverage wiring. It does NOT prove
  simulated physical movement because this ActionServer is a stub.

integration/ros_no_progress_probe.py (max_no_progress_attempts=2):

- Two consecutive Nav2 successes with no new covered-grid cells.
- Third navigation goal is NOT sent; controller pauses with reason
  COVERAGE_NO_PROGRESS. No cleaning credit is invented.

**D. Pure-Python regressions:** tests/test_sim_coverage.py checks
baseline, paused updates, source stability, geometry, replay,
meter reset and scope exclusion; all other job and Nav2 tests remain.

Further middleware regression integration/ros_delayed_meter_tail_probe.py
delivers a synthetic coverage update **after** a genuine ROS2 ActionServer
reports a successful terminal goal. The job becomes completed without
issuing a second goal. All simulator-only versus synthetic inputs are
named in the test output.

## Reproduce in a dedicated, offline OOMWOO development container

Use a dedicated Podman container with the original
docker.io/makerspet/oomwoo:jazzy-dev image, --network=none,
a private ROS_DOMAIN_ID and a read-only source bind mount (:ro,z
for a shared SELinux label when multiple test containers access the
same tree).

Launch the official test_room world and Nav2 as in
[ROS2_GAZEBO_VALIDATION.md](ROS2_GAZEBO_VALIDATION.md).
Do NOT start the upstream coverage_planner concurrently:
it would be a *second* owner of Nav2 goals.

In that container, launch only the upstream ground-truth pose and meter:

~~~sh
ros2 run oomwoo_sim_support ground_truth --ros-args \
  -p use_sim_time:=true -p spawn_x:=0.0 -p spawn_y:=0.0

ros2 run oomwoo_sim_support coverage_meter --ros-args \
  -p use_sim_time:=true -p cleaning_radius:=0.16 -p robot_radius:=0.1745
~~~

Run the experimental controller in a third terminal with a **new**
checkpoint path (all inside the same isolated ROS graph):

~~~sh
PYTHONPATH=/workspace/valdi-cleaning-jobs/src \
python3 -m oomwoo_cleaning_jobs.ros_experimental_node --ros-args \
  -p simulation_lab:=true -p use_sim_time:=true \
  -p coverage_mode:=sim_ground_truth -p auto_continue:=true \
  -p checkpoint_path:=/tmp/valdi-test-measured-cleaning.json
~~~

The isolated test harness sends commands and explicit synthetic battery
status, while observing the REAL OOMWOO meter and Nav2 action client:

~~~sh
PYTHONPATH=/workspace/valdi-cleaning-jobs/src \
python3 /workspace/valdi-cleaning-jobs/integration/gazebo_measured_coverage_probe.py
~~~

It emits GAZEBO_MEASURED_COVERAGE with a JSON report. A nonzero process
exit is a test failure; never interpret timeout as success.

ROS2 middleware tests (no Gazebo, with a fake action server/coverage input):

~~~sh
PYTHONPATH=/workspace/valdi-cleaning-jobs/src \
python3 /workspace/valdi-cleaning-jobs/integration/ros_verified_meter_probe.py

PYTHONPATH=/workspace/valdi-cleaning-jobs/src \
python3 /workspace/valdi-cleaning-jobs/integration/ros_no_progress_probe.py
~~~

## Next required milestone

Use a **single owner of Nav2** and an agreed producer/consumer interface
to integrate the existing oomwoo_coverage sweep planner, room/zone jobs,
trusted coverage and correct pause/resume with dock-cycle. Run multiple
Gazebo worlds with moving obstacles, no-go zones and forced restarts.
Fix goal reconciliation across process crashes before deploying on hardware.
For real hardware add actuator-state-gated coverage estimation and
separate MCU-safe stop verification.

This work proves a valuable interface and measured incremental progress.
It does not claim completed cleaning of the whole floor or a finished RFC.
