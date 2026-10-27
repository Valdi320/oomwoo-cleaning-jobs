# Real Nav2 KeepoutFilter + OOMWOO Gazebo acceptance (isolated, 2026-10-27)

This is a **tested simulator-only integration** of Nav2 KeepoutFilter in
both the global and local costmaps, using the official OOMWOO One
test_room Gazebo world, a private Nav2 Jazzy bringup with the existing
OOMWOO config as its base, and an independent application interlock.

**Not merged into OOMWOO's default navigation launch. Not physical safety.**

## Critical finding: virtual-wall drawings alone are not enforced

OOMWOO's stock simulation nav2_params.yaml has static/obstacle/voxel/
inflation costmap layers but no KeepoutFilter. Offline zone masks are
therefore only *proposals* until Nav2 subscribes to:
1. An active nav2_map_server publishing OccupancyGrid mask with 0/100 cells.
2. An active costmap_filter_info_server publishing CostmapFilterInfo type=0.
3. A KeepoutFilter AND separate filter inflation layer in **both**
   global and local costmap filters. Inflation helps account for robot
   footprint, since filtered zones are not inflated by the normal layers.

Official Nav2 Jazzy references:
- https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_filters/keepout_filter/
- https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/map_server/configuring_costmap_filter_info_server/

## New code and test-only isolation

- integration/generate_keepout_lab.py reads the installed official
  test_room.yaml + PGM; builds a rectangular experimental no-go mask
  via the existing ZonePlan module; serializes the PGM in ROS map
  orientation; and writes a **private** Nav2 param overlay into
  .local-sim/keepout-lab/. It preserves OOMWOO's original maps and
  Nav2 config. It records the occupancy/map fingerprint.
- integration/nav2_keepout_filter_servers.launch.py starts a private
  mask server, a type=0 FilterInfoServer and their lifecycle manager.
- integration/probe_real_nav2_keepout.py samples the **post-filter**
  global/local costmaps and asks the **real ComputePathToPose action
  server** for a route around the obstacle and one goal inside it.
- integration/probe_keepout_gazebo_motion.py sends real
  NavigateToPose to OOMWOO One in Gazebo, records map-to-base_link
  TF poses and refuses any mask entry.
- src/oomwoo_cleaning_jobs/nav_keepout_admission.py adds an
  application-side VerifiedGlobalKeepout: mask geometry must match
  the map and EVERY prohibited cell must already be lethal in the
  actual published global costmap. A forbidden target is rejected
  even if Nav2 would return a partial successful plan.
- The private ROS2 node has an **opt-in**
  require_keepout_filter:=true parameter. It subscribes to the
  actual mask and type=0 FilterInfo and refuses motion if publishers
  disappear, if the mask/costmap disagree, or if a candidate path
  enters a banned cell. When a running job loses verified filter
  conditions, it pauses and requests cancellation. The default
  remains false for existing unrelated Phase-3 tests.
- integration/probe_real_keepout_controller.py verifies forbidden
  goal refusal before any Nav2 action; a safe goal was accepted and
  canceled terminally. It supplied **synthetic BatteryState only**:
  the official test-room has no valid battery publisher.
- integration/probe_keepout_filter_loss.py deliberately terminates
  ONLY the named private FilterInfoServer **after a job is paused**.
  Resume is refused; the paused job and old goal count are retained.
  In-motion server loss is not verified by this particular test.

## Actually measured results

| Assertion | Outcome |
| --- | --- |
| Source filter mask | 220 prohibited map cells |
| Global costmap: inside / outside | 100 / 0 |
| Local costmap: inside / outside | 100 / 0 |
| ComputePathToPose around rectangle | SUCCESS, 108 poses, **0 mask crossings** |
| Goal requested inside rectangle | Nav2 returned success but **stopped 0.40 m short** |
| Independent job admission of forbidden goal | REJECTED before send |
| Gazebo movement test #1 | 1.731 m displacement, 226 sampled poses, **0 entries** |
| Minimum robot-centre-to-rectangle distance #1 | 0.2315 m |
| Gazebo movement test #2, cold restart | 1.736 m, 230 sampled poses, **0 entries** |
| Minimum distance #2 | 0.2636 m |
| Real filtered-node live refusal/cancel | PASS, exactly one allowed goal, terminal canceled |
| FilterInfoServer loss after job paused | PASS, resumption rejected, zero new Nav2 goals |

Machine-readable reports:

- [First filtered Nav2 and Gazebo motion](evidence/real_nav2_keepout_gazebo_20261027.json)
- [Cold-repeat + live controller + FilterInfo loss](evidence/real_nav2_keepout_repeat_20261027.json)
- [Private controller goal-refusal test](evidence/real_keepout_controller_20261027.json)

The *initial* attempt to use generic nav2_bringup failed because
OOMWOO's upstream config specified the literal map.yaml instead of
the actual saved-map path. The isolated generator now sets the absolute
test_room.yaml and AMCL's initial pose at the known simulator spawn
(0,0,0). Later passing runs used **real AMCL initialized at that
known spawn**, not an artificial static map-to-odom publisher.
This was checked via lifecycle state ACTIVE and process inspection.
No SLAM/relocalization edge cases were exercised.

## Reproduction in an isolated offline Podman container

Run from this repository root on an x86-64 Linux system with a local
docker.io/makerspet/oomwoo:jazzy-dev image and sufficient CPU/RAM/disk.
Use a fresh test container, *not* an active external robot DDS domain.
Generated files stay inside the ignored .local-sim directory:

~~~sh
mkdir -p .local-sim/keepout-lab
podman run --rm --pull=never --network=none \
  -e PYTHONPATH=/repo/src \
  -v "$PWD":/repo:ro,z \
  -v "$PWD/.local-sim/keepout-lab":/lab:rw,z \
  docker.io/makerspet/oomwoo:jazzy-dev \
  python3 /repo/integration/generate_keepout_lab.py --out /lab

podman run -d --pull=never --name oomwoo-keepout-validation \
  --network=none --cpus=4 --memory=8g --pids-limit=1300 \
  --shm-size=512m --security-opt=no-new-privileges --cap-drop=ALL \
  -e ROS_DOMAIN_ID=196 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e PYTHONPATH=/repo/src -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e QT_QPA_PLATFORM=offscreen \
  -v "$PWD":/repo:ro,z \
  -v "$PWD/.local-sim/keepout-lab":/lab:ro,z \
  docker.io/makerspet/oomwoo:jazzy-dev sleep infinity
~~~

Start three launches in the same container, in their respective
terminal sessions. For clarity, the commands below use podman exec;
keep the sessions alive. Redirect logs if launching in detached mode.

~~~sh
podman exec -d oomwoo-keepout-validation bash -lc \
 'source /ros_ws/install/setup.bash &&
  ros2 launch oomwoo_gazebo world.launch.py \
  world:=/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/worlds/test_room.world \
  x_pose:=0.0 y_pose:=0.0 robot_model:=oomwoo_one \
  headless:=true enable_cameras:=false >/tmp/world.log 2>&1'

podman exec -d oomwoo-keepout-validation bash -lc \
 'source /ros_ws/install/setup.bash &&
  ros2 launch /repo/integration/nav2_keepout_filter_servers.launch.py \
  >/tmp/filter.log 2>&1'

podman exec -d oomwoo-keepout-validation bash -lc \
 'source /ros_ws/install/setup.bash &&
  ros2 launch nav2_bringup bringup_launch.py \
  map:=/ros_ws/install/oomwoo_sim_support/share/oomwoo_sim_support/maps/test_room.yaml \
  params_file:=/lab/nav2_keepout_params.yaml \
  slam:=False use_sim_time:=True use_composition:=False autostart:=True \
  >/tmp/nav2.log 2>&1'
~~~

Once Nav2 lifecycle managers report active and mask/filter topics are
present, first run the **read-only** costmap/planner test:

~~~sh
podman exec oomwoo-keepout-validation bash -lc \
 'source /ros_ws/install/setup.bash &&
  python3 /repo/integration/probe_real_nav2_keepout.py'
~~~

Only in this isolated simulation, run the simulated motion test:

~~~sh
podman exec oomwoo-keepout-validation bash -lc \
 'source /ros_ws/install/setup.bash &&
  python3 /repo/integration/probe_keepout_gazebo_motion.py'
~~~

Optional extra guarded-job test: launch the private node with
simulation_lab:=true, require_keepout_filter:=true, use_sim_time:=true
and a **fresh checkpoint**; then run
integration/probe_real_keepout_controller.py. Its Operator publishes
explicitly synthetic battery telemetry. The last FilterInfo-loss test
intentionally terminates one server and must be the final test in a
fresh isolated container; do not try to continue navigating afterward.

Cleanup only your own exact named test container:

~~~sh
podman rm -f oomwoo-keepout-validation
~~~

## Remaining to reach the full RFC

The upstream default launch/config is deliberately **unchanged**.
Integrating the validated KeepoutFilter into the actual OOMWOO
bringing-up workflow needs maintainer approval, mask editor API,
change propagation/replanning tests, and a single Nav2 action owner.
These tests use one static rectangle and a known OOMWOO map; they
do not prove every polygon, dynamic map, moved robot pose or home.

Whole-room and multi-room coverage, an official start/pause/resume
action/service contract, dock-cycle integration and hardware-level
MCU safety remain outstanding. The robot's real localization noise,
cliff/drop sensors, wheel slip and physical collision safety have
not been validated.
