# Phase 3 — isolated, long-running ROS2 cleaning-job controller

## Scope and status

An **experimental ROS2 Jazzy node** now runs throughout a cleaning job and
listens for start/pause/resume/cancel commands on private ROS2 topics. It
uses OOMWOO's REAL Nav2 NavigateToPose ActionClient for navigation and checks
the real simulated OOMWOO odometry in Gazebo.

**Not an upstream public API, not a finished cleaning-jobs module, and not a
safety-rated controller.** The sole purpose is to demonstrate and test the
integration boundary before maintainers settle the canonical ROS2 job
action/service interface. No physical robot or MCU has been tested.

Source: src/oomwoo_cleaning_jobs/ros_experimental_node.py.

## Private test interface (NOT OOMWOO canonical API)

- Command: /valdi/cleaning_jobs/command — std_msgs/msg/String JSON, depth 10.
- Status: /valdi/cleaning_jobs/status — std_msgs/msg/String JSON,
  reliable + transient-local, depth 1.
- Dependencies: /map (latched), /global_costmap/costmap, /clock, /tf,
  /battery_state (BatteryState percentage), /navigate_to_pose Nav2 ActionServer,
  /bt_navigator/get_state (lifecycle_msgs/srv/GetState).
- ROS2 executor: SingleThreadedExecutor, so the interlock and ActionClient
  callbacks do not race in different threads.

Example commands (JSON strings are passed in the std_msgs String data field):

~~~json
{"op":"arm_fresh_sim","isolated_disposable_sim":true,"request_id":"a"}
{"op":"start","mode":"spot","job_id":"test_spot","x":2.0,"y":0.0,"request_id":"b"}
{"op":"pause","request_id":"c"}
{"op":"resume","request_id":"d"}
{"op":"cancel","request_id":"e"}
{"op":"status","request_id":"f"}
~~~

The status message contains last_command including request_id, job state,
interlock barrier state, outstanding goal token, and Nav2 lifecycle state.
Invalid commands return ok:false and an error. Start accepts **spot mode only**
and limits target distance to 5 metres; full-home jobs must NOT be claimed as
implemented by this node.

## Safety and reliability gates

- **Simulation-only opt-in:** simulation_lab defaults to false.
  The private arm command requires isolated_disposable_sim:true, a brand-new
  checkpoint and no prior goal. The developer must also isolate DDS traffic
  with Podman network=none and a dedicated ROS_DOMAIN_ID. This is not an
  authorization or production robot control system.
- **BT Navigator lifecycle:** default require_nav2_lifecycle=true. A
  GetState service client verifies state ACTIVE (ID 3) and caches only a
  fresh reply. Nav2 can advertise the ActionServer before its lifecycle
  is active; this was reproduced and corrected. A stale/inactive lifecycle
  holds or pauses scheduling.
- **Costmap admission:** before each dispatch, use an independently received
  Nav2 global costmap (not just the base map); reject unknown cells, cost
  greater than 25, and cells along an advisory route whose sampled
  0.21 m footprint intersects obstacles/unknown/outer bounds.
  Nav2 remains responsible for actual collision monitoring and replanning.
- **Localization:** require a recent map-to-base_link TF transform with a
  fresh simulator clock; map geometry and origin changes freeze scheduling.
- **Battery:** require a fresh, finite BatteryState percentage. Losing the
  signal blocks new navigation and freezes a running job. Battery below 20%
  triggers a service pause and cancellation request. In the official OOMWOO
  test_room run there was no working battery publisher; the test harness
  supplied explicitly SYNTHETIC ROS2 battery telemetry and triggered 4%.
- **Optional live KeepoutFilter gate:** require_keepout_filter:=true
  requires an active type=0 FilterInfoServer, current mask publishers and
  a live global post-filter costmap that reports every prohibited mask
  cell as lethal. The controller refuses forbidden destinations even
  though Nav2 may return a seemingly successful *truncated* path.
  If the filter source disappears, new requests remain blocked; this
  was tested with the real OOMWOO Nav2/Gazebo. This does NOT yet enable
  the filter in OOMWOO's default config. See
  [REAL_NAV2_KEEPOUT_GAZEBO.md](REAL_NAV2_KEEPOUT_GAZEBO.md).
- **Cancellation:** do not send another goal until Nav2 returns a terminal
  action outcome. Uncertain send/cancel states latch a fault; the node
  cannot claim a physical stop merely from a cancel response.
- **Durability:** checkpoint writes are atomic; no second job can reuse the
  same checkpoint. After an interrupted process, automatic rearming is
  forbidden until an independent old-goal reconciliation exists.
- **Cleaning:** NO cells are credited for Nav2 arrival. A real trusted
  clean-and-map coverage producer is not integrated yet.

Hard stops for bumpers, cliffs, wheel-drop and overcurrent **must remain in
independent MCU firmware**. This test node never substitutes for them.

## Reproduce

Inside the official OOMWOO image with ROS2 Jazzy, launch the existing
test_room world and navigation. See:
[ROS2_GAZEBO_VALIDATION.md](ROS2_GAZEBO_VALIDATION.md) for the complete
isolated Podman setup and the world/navigation launch arguments. Use the
same named container, with this source tree mounted read-only.

Start the node after the Gazebo world and Nav2 launches:

~~~sh
podman exec -d oomwoo-nav2-validation bash -lc \
 'source /ros_ws/install/setup.bash &&
  export PYTHONPATH=/workspace/valdi-cleaning-jobs/src:$PYTHONPATH &&
  python3 -m oomwoo_cleaning_jobs.ros_experimental_node \
     --ros-args -p simulation_lab:=true -p use_sim_time:=true \
     -p checkpoint_path:=/tmp/valdi-job-phase3.json'
~~~

Wait for Nav2 lifecycle ACTIVE (the node refuses to start a job earlier).
Run the **self-contained operator/measurement harness**, which publishes
synthetic BatteryState because the upstream test room does not:

~~~sh
podman exec oomwoo-nav2-validation bash -lc \
 'source /ros_ws/install/setup.bash &&
  export PYTHONPATH=/workspace/valdi-cleaning-jobs/src:$PYTHONPATH &&
  python3 /workspace/valdi-cleaning-jobs/integration/gazebo_persistent_probe.py'
~~~

It sends start/pause/resume, simulates low battery followed by replenishment,
resumes, and cancels. It measures robot motion from OOMWOO Gazebo /odom,
checks terminal action cancellation, and asserts no unearned coverage.

A fast independent ROS2 middleware test (no Gazebo needed):

~~~sh
python3 integration/ros_persistent_probe.py
python3 integration/ros_lifecycle_guard_probe.py
~~~

Run these inside the official ROS2 image, not plain host Python.
The first creates fake map, costmap, TF, clock, battery and an
actual ROS2 NavigateToPose ActionServer, then exercises the long-running node
through its actual topics. This fake server has no motor output; the second
tests fail-closed behavior when Nav2 lifecycle is absent. Only the real
Gazebo harness validates simulated movement.

Machine-readable result:
[evidence/ros_persistent_gazebo_20261027.json](evidence/ros_persistent_gazebo_20261027.json).
It contains the exact image digest, 3 real Nav2 accepted actions, manual pause,
automatic synthetic low-battery pause and final cancel. All returned terminal
canceled and no unverified floor was counted.

## Still needed to complete the OOMWOO cleaning-jobs RFC

- Maintainer-approved **public ROS2 job action/service** and persistent
  deployed package/launch file (private JSON is for prototyping only).
- Real trusted clean-and-map area-coverage feedback and full whole-map,
  room/zone and spot cleaning orchestration, not just single Nav2 goals.
- Persisted automatic room segmentation and actual Nav2 KeepoutFilter masks.
- Dock-cycle recharge/mop/bin service orchestration and multi-room regressions
  that prove end-to-end resumed coverage without prohibited-area entry.
- After a controller crash, robust Nav2 active-goal reconciliation before
  rearming, then full MCU/physical robot validation.

The new node is a substantial integration milestone, not a claim that
the module or robot is completed.
