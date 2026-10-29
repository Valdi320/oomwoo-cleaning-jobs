# Exact bounded grid clearance dilation — 29 October 2026

Owned contribution: Valdi320/oomwoo-cleaning-jobs, Apache-2.0. Upstream makerspet navigation, CAD, PCB, firmware, motor interfaces and safety interlocks are untouched.

## Problem / algorithm

Previously Grid.blocked expanded every occupied, unknown or keepout cell in a nested Python loop over its Chebyshev clearance square. Large clearance or cluttered floor maps inflated CPU cost. The revision preserves **precisely the same set of blocked cells** via adaptive strategies:

- Full-boundary case (twice the inflation radius reaches the map width or height): return all cells.
- Sparse obstacles: contiguous per-row integer ranges, without large scratch memory.
- Dense obstacles: four-corner rectangle difference array followed by an in-place 2D prefix sum. Temporary signed 32-bit integer array uses 4*(w+1)*(h+1) bytes on the tested platform (~0.19 MiB at 256x192). The Grid class already caps maps at 4,000,000 cells, preventing counter overflow.
- Select sparse when (blocked_count * min(w,2r+1) * min(h,2r+1)) <= 3*(w*h); otherwise dense. Both preserve unknown, keepout, obstacle margins and exterior boundary safety exactly.

Grid.blocked's public frozenset API, Grid.traversable behavior, cache cutoff and cleaning-credit security policy are unchanged.

## Reproducible commands

In the contribution repository, run:

    PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_grid_inflation_resource_20261029.py' -q
    PYTHONPATH=src python3 -m unittest discover -s tests -q
    PYTHONPATH=src python3 -O -m unittest discover -s tests -q
    PYTHONPATH=src:tests python3 tests/bench_grid_inflation_resource_20261029.py
    git diff --check

Observed PASS: 5/5 new focal tests (300 seeded reference-parity grids; dense/sparse path assertions; keepouts and map boundaries), 669/669 full regressions, 669/669 Python -O, diff --check. The two benchmark cases have matching SHA256 hashes and blocked-cell counts, with no change of navigation geometry.

## CPU benchmark, synthetic x86 only

Platform Linux-6.17.7-ba29.fc43.x86_64-x86_64-with-glibc2.42, Python 3.14.3; three runs per variant; same process; unpinned process CPU medians. Raw wall/CPU repetitions and exact SHA256 are preserved in [machine evidence](evidence/grid_clearance_resource_20261029.json).

- Synthetic room with four furniture obstacles, 256x192 cells, clearance 7, 3894 source obstacles and 14562 inflated blocked cells: **0.060451 s -> 0.010890 s (81.99% less CPU)**.
- Synthetic dense stress map, 256x192 cells, clearance 10, 13405 source obstacles and 49152 inflated cells: **0.353394 s -> 0.020558 s (94.18% less CPU)**.

These measurements do NOT characterize complete ROS2/Nav2/OOMWOO CPU, actual robot speed, ARM SBC, RAM savings, power, temperature or physical safety. The process high-water RSS is recorded without a before/after inference. Larger maps with frequent recalculation still require memory/latency profiling.

## Reviewer checklist / limits / revert

- [ ] Compare exact reference parity in tests and the two hashes with the parent revision [36bf425](https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/36bf42537194d2593240d7db76c5f726273c7ff3).
- [ ] Review src/oomwoo_cleaning_jobs/grid.py and tests/test_grid_inflation_resource_20261029.py.
- [ ] Run ROS2 Jazzy + real Nav2/Gazebo keepout, map changes and restart tests before integrating into production; this iteration is synthetic Python Linux only.
- [ ] Record SBC CPU/RAM and deadlines plus actual MCU safety independently before selecting an economic compute platform.
- [ ] Preserve exclusive Nav2 ownership pending upstream maintainer decision; never infer vacuum/actuator safety from this optimization.

No new dependency beyond standard-library array. The small isolated commit can be reverted normally with its tests; original inflated semantics remain the acceptance criterion.
