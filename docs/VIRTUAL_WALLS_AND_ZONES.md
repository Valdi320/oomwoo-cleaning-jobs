# Virtual walls and persisted no-go zones (offline OOMWOO prototype)

This is a simulation-first source/geometry component of cleaning-jobs,
NOT a functioning Nav2 safety filter or a hardware cliff/bumper system.

## Implemented

src/oomwoo_cleaning_jobs/zones.py defines:

- No-go polygons in ROS2 map-frame metres. The mask uses each
  OccupancyGrid cell centre with the map's actual translation/yaw.
  Concave polygons are accepted; degenerate or self-crossing polygons,
  non-finite inputs, duplicate names and oversized shapes are rejected.
- Virtual walls specified by two endpoints and a finite thickness.
  Rasterized cell-centre distance from a segment creates an explicit
  no-go band at the map's resolution.
- Union of multiple zones as a 0/100 OccupancyGrid-style Raster.
  Raster 100 indicates *candidate prohibited cell* (not a collision
  sensor reading).
- Conservative planning Grid with rasterized zone indices added
  to Grid.keepout. Grid clearance inflates around them for the robot
  centre. The original map/occupancy remains immutable.
- Versioned, atomic JSON storage with exact source map/geometry/
  keepout fingerprint binding. Changed maps and labels refuse
  stale output. The room plan built before a zone edit also refuses
  the changed map, requiring room reassessment.

The source produces mask candidates only. Nothing is published
to Nav2, no-go editing UI has been built, and no actual robot
is sent through/around these boundaries in this test suite.

## Real official OOMWOO map check

integration/ros_zone_mask_probe.py subscribes READ-ONLY to /map
in the isolated OOMWOO Jazzy/Gazebo test_room. On 2026-10-27:

- Input: 140x140 map, 0.05m/cell, 12,394 conservative safe cells.
- One short virtual wall and one rectangular no-go polygon
  proposed in world coordinates.
- 116 base mask cells marked 100.
- Conservative safe set decreased to 11,761 cells once the
  configured five-cell clearance inflation was applied.
- JSON save/reload and stale room plan rejection: PASS.
- Real Nav2 goal/KeepoutFilter enforcement: **NOT TESTED**.

Machine-readable observation:
[evidence/zone_mask_test_room_20261027.json](evidence/zone_mask_test_room_20261027.json).

The upstream OOMWOO simulation parameter file at
https://github.com/makerspet/oomwoo-ros2-tools/blob/jazzy/src/oomwoo_sim_support/config/nav2_params.yaml
lists static/obstacle/voxel/inflation costmap layers but not
the Nav2 KeepoutFilter mask pipeline. A planning-side mask is NOT
proof that the Nav2 local/global costmaps will enforce it.
Before authorizing production movement, validate a configured
Nav2 KeepoutFilter/CostmapFilterInfo/mask server for BOTH costmaps,
and separately retain independent MCU safety.

## Plain Python offline example

~~~python
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.zones import ZoneShape, ZonePlan

base = Grid(10, 10, (0,)*100, resolution_m=.1)
raster = Raster(10, 10, .1, -.5, -.5, 0.0, (0,)*100)
zones = ZonePlan.create(raster, base, (
    ZoneShape("Sin entrar", "polygon",
              ((-.2,-.2),(.2,-.2),(.2,.2),(-.2,.2))),
    ZoneShape("Pared virtual", "wall",
              ((-.4,0.),(.4,0.)), width_m=.10),
))
mask = zones.mask(raster, base)
safer_grid = zones.grid_with_keepouts(raster, base)
~~~

No ROS2 output is emitted by this example. To persist, use
zones.save(path, raster, base) and ZonePlan.load(path,raster,base);
any source-map change invalidates the saved plan.

## Tests and remaining work

The standard Python regression suite includes tests/test_zones.py
and tests/test_room_job_flow.py. The latter verifies that a
polygon-excluded cell is not included in a selected room job.
Additional tests verify rotated maps, walls cutting connectivity,
overlapping masks, stale map refusal and atomic persistence.

Outstanding production features:
- Nav2 KeepoutFilter configured and tested (actual goal must never
  cross banned cells, including during replanning/recovery).
- Operator-editable floor plan UI, mask server lifecycle and QoS,
  Home Assistant-facing zone selection, coverage and docking jobs.
- Map updates/SLAM corrections, robot footprint uncertainty,
  dynamic obstacle avoidance and real robot sensor safety.

Do not interpret this as a completed, field-ready avoidance system.
