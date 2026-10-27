# Live Nav2 KeepoutFilter editing verified (OOMWOO Jazzy/Gazebo, 27 Oct 2026)

## Tested result: PASS in an isolated robot simulator

The OOMWOO test_room simulation was started with an EXPERIMENTAL Nav2
configuration, not the upstream default navigation launch. A test-only
mask_server and type-0 CostmapFilterInfoServer supplied a forbidden
polygon to the KeepoutFilter plugins of BOTH global and local costmaps.

While no Nav2 goal was active, the test loaded a different keepout
OccupancyGrid using the real nav2_msgs/srv/LoadMap service:

| Item | Actual observed result |
| --- | --- |
| Old prohibited mask | 220 cells |
| New prohibited mask | 198 cells |
| LoadMap return code | 0 (success) |
| Distinct post-filter confirmations | 2 |
| Global costmap updates after reload | at least 2 |
| Local costmap updates after reload | at least 3 |
| Map costs at new forbidden centre (-0.8, 0) | 100, lethal |
| Map costs at old forbidden centre (+0.85, 0) | 0, free |
| Motor/navigation commands during update | 0 |
| Software transaction committed | Yes, after both costmaps were proven |
| Stock OOMWOO navigation launch modified | NO |

Machine evidence: evidence/dynamic_keepout_hot_reload_verified_20261027.json.

## Important discovered coordinate-frame pitfall

OOMWOO's global costmap and KeepoutFilter mask use the 'map' frame,
but its rolling LOCAL costmap uses 'odom', at a DIFFERENT resolution
and dynamically moving origin. Directly treating both as map coordinates
is unsafe, even if the masks appear to line up in a particular world.

The new src/oomwoo_cleaning_jobs/costmap_frame.py transforms the
odom-frame costmap origin, yaw and sampling grid into the map frame.
The ROS2 test retrieves the actual TF2 map<-odom transform and requires
matching frame IDs, finite planar rotation and bounded ROS sim time.

In this OOMWOO test_room, AMCL publishes the latest transform about
5 simulated seconds IN THE FUTURE (its transform tolerance).
The conversion accepts at most 6 seconds of bounded future lead, and
rejects costmaps or transforms whose timestamps are too old or wildly
inconsistent. This tolerance must be reviewed with real robot localization.

ZoneUpdateTransaction now requires two strictly increasing REAL receive
sequence counters from EACH costmap. Replaying the same latched
OccupancyGrid cannot satisfy both confirmations. A change stays blocked
until its new zone is actually lethal where observed.

This is an additional ROS2 software gate, not a hardware collision
stop. A runtime user-interface for live zone updates remains future work.

## Follow-up with the actual Nav2 planner

After changing the keepout, we used the REAL ComputePathToPose action
without sending any NavigateToPose or motor commands:

- Formerly prohibited map point (0.85, 0): 32-pose planned path,
  endpoint error 0 m, zero intersections with the new forbidden mask.
- Newly prohibited point (-0.8, 0): Nav2 returned a 15-pose
  'successful' PARTIAL plan stopping 0.4 m BEFORE the requested
  point. Zero intersections with the new forbidden mask.

This is NOT proof that Nav2 rejects every forbidden destination on
its own. It demonstrates why the earlier job-level goal-admission
interlock is necessary: success from the planner can mean a truncated
path, not reaching the requested prohibited goal.

Machine report: evidence/post_hot_keepout_nav2_planner_20261027.json.

## Reproduction in the private, OFFLINE OOMWOO image

1. Build a fresh private Podman instance of
   docker.io/makerspet/oomwoo:jazzy-dev with network=none and an
   isolated ROS_DOMAIN_ID. Do NOT run these tests in a shared robot DDS
   graph or modify any upstream OOMWOO source tree.
2. Use integration/generate_keepout_lab.py to create private
   .local-sim/keepout-lab/keepout_test_room.yaml and the Nav2
   KeepoutFilter config overlay. Run the official Gazebo test_room,
   then launch integration/nav2_keepout_filter_servers.launch.py and
   Nav2 bringup with the generated overlay. Wait for lifecycle ACTIVE
   and valid AMCL TF before any tests. The tested setup delayed Nav2
   launch until Gazebo publishers had settled (roughly 12 seconds);
   earlier concurrent launch attempts stalled the planner lifecycle.
3. Build the replacement mask inside the same sandbox using
   integration/generate_keepout_revision.py --root /lab.
4. With no active Nav2 action, run
   integration/probe_dynamic_keepout.py. A true PASS must include
   correct LoadMap result, updated mask topic, two distinct lethal
   global and local costmap observations using a real TF transform,
   and committed software transaction state.
5. Without commanding robot motion, run
   integration/probe_post_hot_keepout_route.py. It verifies the
   change took effect inside the real ROS2 route planner.

Do not use this as a recipe to bypass independent MCU/bumper/cliff
safety. No moving-robot hot edit, dynamic SLAM map update, hardware
actuator or real vacuum dirt coverage was tested.

## Historical failed attempts retained

Earlier experiments stalled Nav2 planner lifecycle or incorrectly
attempted to interpret the local odom costmap directly as 'map'.
Their failure is retained for auditing in
evidence/dynamic_keepout_hot_reload_blocked_20261027.json.
The passing experiment is a later, separate cold-start simulation run.

## Still to complete for OOMWOO

Maintainers must decide how to integrate the verified filter overlay
into the STANDARD OOMWOO bringup, define the actual room/zone editing
API, handle mid-motion change requests with mandatory cancel/stop
barriers, reroute around newly blocked areas, and test pose loss,
transforms crossing time discontinuities, obstacles and restart.
Only a controlled static rectangle-to-rectangle edit while IDLE is
verified here, and the project still has no robot hardware safety
certification.
