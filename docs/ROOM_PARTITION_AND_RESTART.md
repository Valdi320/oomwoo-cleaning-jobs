# Saved-map room partitioning, labels and restart checkpoint

A conservative, offline contribution to the OOMWOO cleaning-jobs RFC.
This is NOT arbitrary-house semantic understanding, a production Nav2
KeepoutFilter implementation, or a physical safety system.

## Delivered

The pure-Python module src/oomwoo_cleaning_jobs/rooms.py provides:

- Deterministic four-connected proposals from occupancy maps. A
  distance-to-wall field identifies wide interiors and bottlenecks;
  tiny disconnected areas are not silently dropped.
- Optional manual doorway seal cells to resolve ambiguous boundaries.
- Stable room identifiers and persistent user labels such as Salón
  and Cocina, independently editable from occupancy geometry.
- Exact partitions: no overlapping room cells, no unknown or no-go
  cells assigned to a cleaning room.
- Atomic, versioned JSON plans bound to the exact input map
  fingerprint, including clearance cells, occupancy and keepouts.
  Changed maps or zones refuse stale room labels.
- RoomPlan.grid_with_rooms(grid) returns a labeled Grid compatible
  with the original CleaningJobs.start(mode="rooms", ...) core.

Limitations: an open-plan space may be one room; a thin furniture gap
may create false splits. Humans should review room proposals and
doors before treating them as real architectural boundaries.

## Regression tests

~~~sh
PYTHONPATH=src python -m unittest discover -s tests -q
~~~

Synthetic maps include two open rooms joined through a narrow doorway,
unknown cells, no-go areas, corrupted or stale plans, duplicate labels,
manual room renaming, and exact coverage accounting.

tests/test_room_job_flow.py validates a fully completed chosen-room
job using synthetic coverage events, with simulated low battery,
goal cancellation, durable checkpoint, a new CleaningJobs instance
after restart, a new interlock that starts DISARMED, explicit
test-only backend-idle confirmation and subsequent resume to
100% of the selected grid-centre targets.

It never credits the other room or no-go cells. This proves pure
state/partition semantics, not actual ROS2 motion or dirt removal.

## Official OOMWOO test-room map check

integration/ros_room_partition_probe.py is a strictly read-only
ROS2 map client, never sending navigation commands.

The 2026-10-27 headless official test_room map had 140x140
cells, 0.05 metres per cell and 12,394 conservative safe cells.
For distance thresholds 2, 3 and 4 the room proposal was one
contiguous zone covering exactly all 12,394 safe cells,
about 0.25 seconds per threshold.

That is expected for the mostly open test_room, NOT proof the
algorithm correctly identifies rooms on arbitrary complex maps.

## Plain Python example

~~~python
from pathlib import Path
from oomwoo_cleaning_jobs import Grid, CleaningJobs
from oomwoo_cleaning_jobs.rooms import RoomPlan, segment_rooms

grid = Grid(15, 15, tuple([0] * 225), resolution_m=0.05)
plan = segment_rooms(grid).renamed("room_001", "Salón")
plan.save(Path("/tmp/rooms.json"), grid)
reloaded = RoomPlan.load(Path("/tmp/rooms.json"), grid)
labelled = reloaded.grid_with_rooms(grid)
jobs = CleaningJobs(labelled, Path("/tmp/room_job.json"))
jobs.start("room_salon", mode="rooms", rooms=("Salón",), start_cell=112)
~~~

Each new job requires its own fresh checkpoint. This snippet does
not publish ROS2 commands or make physical coverage claims.

## Not yet ready for deployment

The offline segmenter is not integrated with the private
experimental ROS2 node's full-room scheduling. That node only
runs small spot trials in a dedicated offline simulator.

The production scope still requires: exclusive Nav2 ownership,
integration with upstream OOMWOO boustrophedon coverage planner,
real Nav2 KeepoutFilter execution, dynamic obstacles, door
footprint checks, real room labels, docking/charging/mop services
and safe restart reconciliation of unknown old Nav2 actions.

No test has certified physical MCU bumper/cliff/overcurrent
safety or proven a robot physically vacuumed a real home.
