# Swept-disk admission: hazard-first pixel fast path (2026-10-28)

**Owner:** Valdi320, Apache-2.0 independent cleaning-jobs source; ChatGPT-assisted.
**Base:** `8d58ea220f745c68949e3936f916169b5eccf15b`.
**Upstream pointer:** https://github.com/makerspet/oomwoo/pull/79 (same existing scope, no separate PR).

## Blocker and change

The new *exact swept-disk* costmap admission loop from the preceding
commit called a geometry-heavy segment/rectangle distance test on **every**
observed cell of the bounding box, even when the cell was unambiguously
safe (`0 <= cost <= max_cost`). On a 1 cm grid this is thousands of calls
per sub-metre segment, unnecessary CPU work for inexpensive processors.

Move the inexpensive cost check **before** the segment/rectangle
intersection test. Known-safe pixels need no geometry. **Unsafe, lethal,
UNKNOWN, or over-threshold cells still invoke the exact original
segment/rectangle test**, and a touching hazard still blocks. No changes
to the radius, map-edge rejection, coordinate transform, occupancy
threshold, or fail-closed result. This is an exact predicate-preserving
reordering for validated `Raster` occupancy values (`-1..100`).

## Reproduction and evidence

From the root of the own `Valdi320/oomwoo-cleaning-jobs` repository:

```sh
PYTHONPATH=src python -m unittest discover -s tests -p test_swept_fastpath.py -v
PYTHONPATH=src python -m unittest discover -s tests -q
PYTHONPATH=src python -O -m unittest discover -s tests -q
PYTHONPATH=src python tests/bench_swept_fastpath.py
```

The 7 new tests cover safety threshold, UNKNOWN/lethal cells,
non-finite inputs, map edge, hazard in bbox outside disk, the
all-known-safe no-geometry optimization, and 120 seeded differential
cases including rotated costmaps. The 9 existing swept-footprint tests
also passed in the isolated host copy. The all-known-safe optimization
negative control **FAILED as expected** on the original parent code.
See `docs/evidence/swept_costmap_fastpath_20261028.json`.

In one *Linux x86-64 CPython 3.13.5 container*, the reproducible
30-iteration median synthetic microbenchmarks improved from 30.667 ms to
0.5908 ms (**51.91x**) on a completely safe 1-cm map, and 29.0811 ms
to 0.6765 ms (**42.99x**) with 40 lethal pixels in the segment bbox but
outside the capsule. Timings vary across hosts; these are **not** Pi 5
benchmarks, energy measurements, or end-to-end ROS2 frame budgets.

## Limits

**HOST-ONLY PYTHON / SYNTHETIC COSTMAPS.** No ROS2/Gazebo, physical
robot, MCU, batteries, LiDAR, motors, or docking was accessed in this
iteration. The copy used for 16 local tests contained the two relevant
production source modules and 9 prior tests, **not** the full repository;
check full regression separately with GitHub Actions. This predicate
remains a *preflight advisory guard*, not an MCU collision interlock.

New bottlenecks: unknown/lethal *dense* costmaps still invoke expensive
geometry for every candidate hazard. Optimize only after instrumenting
Gazebo, including stale TF, incoming costmap frame frequency, CPU usage
and latency on actual target compute. Safe robot operation must still
rely on independent hardware cliff/bumper/current safety.

## Validation on contributor PC before publication

- focused: PASS (see versioned JSON evidence).
- full: PASS (see versioned JSON evidence).
- optimized: PASS (see versioned JSON evidence).
- demo: PASS (see versioned JSON evidence).
- bench: PASS (see versioned JSON evidence).
These tests use the host CPU and synthetic data, not Gazebo or physical motors.
