# Nav2 action handoff interlock (Phase 1, headless)

This is a tested ROS2/Nav2-facing state-machine and protocol contract. A
real ActionClient transport is now available in ros_nav2.py and has been
exercised against real Nav2 in Gazebo. It is NOT a persistent ROS2 cleaning-job
node and is not a safety-rated robot controller.
It complements the Phase-0 cleaning-jobs core.
Source: src/oomwoo_cleaning_jobs/nav_interlock.py.
Tests: tests/test_nav_interlock.py.

## The problem solved

Without a cancellation barrier, a job manager can mark itself paused while
Nav2 continues an already accepted route. If it then resumes and sends another
goal, the old action may still be executing. A late callback can be mistaken
for new work. This interlock gates all new goal submissions until terminal
confirmation, with token fencing for stale callbacks.

## Source/sink boundary to implement in ROS 2

| Role | OOMWOO interface | Adapter responsibility |
| --- | --- | --- |
| Navigate | /navigate_to_pose (nav2_msgs/action/NavigateToPose) | Adapt GoalTransport.send_goal(Goal) to ActionClient.send_goal_async, track acceptance and actual handle, forward terminal result |
| Cancel | Nav2 action cancellation | request_cancel(token) must cancel the matching goal. Cancel request or acceptance is NOT a terminal result. Continue waiting for a terminal result |
| Localization | /odom, /tf | Convert fresh localized pose to grid cell; reject stale/unknown pose |
| Map | /map (nav_msgs/msg/OccupancyGrid) | Preserve resolution, dimensions, occupancy, origin transform and frame. Current Phase-0 grid digest does NOT include map origin: validate separately |
| Safety | MCU independent hardware safety path | Hardware bumper/cliff/wheel-drop/current safety MUST remain operable with ROS2 down. On uncertain cancel, this core only blocks NEW goal issuance, it cannot stop motors |
| No-go | Nav2 costmap KeepoutFilter and inflation | admit_route(route) must check footprint, inflated keepout, map version. The grid route alone is advisory |
| Cleaning | agreed clean-and-map measured coverage stream | Only verified cleaning feedback may call acknowledge_coverage with matching token/source/map. Never treat Nav2 arrival as swept floor |
| Resources | /battery_state, future bin/mop topics | Update status; on service need pause and cancel active goal before attempting dock services |
| UI | future cleaning-jobs action/service API | Wait for maintainer interface choice; no duplicate global ROS2 API invented |

### Correct action lifecycle

1. The interlock starts disarmed on every process boot.
2. ROS2 adapter must prove that no previous Nav2 action can still move the robot.
   Lost goal handles after restart are NOT proof of safety.
3. After independent verification, call confirm_backend_idle(True). Restarted
   cleaning jobs also need fresh pose and resource-gated manual/system resume.
4. Before each dispatch, the adapter supplies real footprint/costmap admission.
   The port sends only the proposed target pose to Nav2 and records token to
   actual Nav2 action handle. Do not bypass Nav2 path planning.
5. Terminal result callbacks invoke on_goal_result(token, outcome) with one
   of succeeded, canceled, aborted, rejected. Nonterminal acceptance is not enough.
6. Pause/resource/recovery requests gate further goals and request cancel.
   check_timeout() uses a monotonic clock: if no terminal result appears,
   freeze scheduling and request verified independent stop/safety escalation.
7. A stale token is rejected. Matching coverage source strings do not by
   themselves authenticate data: verify it independently.

### Race: cancel before Nav2 has accepted a goal

In rclpy, send_goal_async yields a goal handle later. If pause is requested
while acceptance is pending, the adapter must remember cancel-on-acceptance.
When a handle eventually arrives, cancel that goal, wait for terminal result.
Do not mark action stopped simply because a cancellation request was sent.

### Race: process restarts while Nav2 still has a goal

A crashed manager loses the local handle but Nav2 might continue following
the accepted goal. The durable checkpoint resumes paused_recovery, and a new
interlock refuses to arm until backend idle is independently verified.
If the adapter cannot prove all old goals are terminal, stay disarmed.
The state machine cannot magically cancel an unknown old goal.

## Typical adapter flow (pseudocode; standalone ROS2/Gazebo harness in integration/)

~~~python
jobs = CleaningJobs(grid, checkpoint)
gate = NavigationInterlock(jobs, nav2_port, coverage_source="verified_sweep")
gate.confirm_backend_idle(verified_no_outstanding_goals=True) # ONLY AFTER PROOF

# If recovering an interrupted job:
gate.resume(pose_cell=fresh_pose_cell, resources=latest_resources)
goal = gate.dispatch(pose_cell=fresh_pose_cell, admit_route=costmap_admission)

# Actual trusted measured sweep feedback WHILE the goal is active:
gate.acknowledge_coverage(
    token=goal.token, cells=verified_cells, producer="verified_sweep",
    map_digest=grid.fingerprint())

# On pause:
gate.pause()  # sends cancel request, not proof of stopped motors
# Callback from Nav2 once that goal is terminal:
gate.on_goal_result(token=goal.token, outcome="canceled")
~~~

The snippet omits ROS2 wiring, safety path, handling initial job starts and
resource subscriptions, and is not suitable to drive actual hardware.

## Reproduce headless

~~~sh
PYTHONPATH=src python -m unittest discover -s tests -q
~~~

Test coverage: 104 pure-Python regression tests, including original job-state and action-handoff fault injection cases,
including uncertain goal send, lost cancel, cancellation timeout, stale callbacks,
invalid coverage evidence, late success after pause, restart, independent idle
confirmation, concurrent write, and no double dispatch.

Real ROS2 ActionClient integration and headless OOMWOO Nav2/Gazebo cancel
regressions have now run successfully; see ROS2_GAZEBO_VALIDATION.md.
Real physical hardware, MCU stop paths, real floor-cleaning evidence, and
production ROS2 public job/action orchestration remains unimplemented. A private
long-running experimental ROS2 node is now tested against the actual OOMWOO
Gazebo/Nav2 robot; see EXPERIMENTAL_ROS_NODE.md.
