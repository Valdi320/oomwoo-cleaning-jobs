# OOMWOO bounded dirty corridor scoring: measured source contribution

**Scope:** Independent Valdi320 Apache-2.0 experimental cleaning-jobs gap-route selector, not the official OOMWOO Nav2 runtime. No motors, batteries, firmware, costmaps or ROS2 configuration changed.

## Problem and exact correction

The previous fallback selector scored each route candidate by scanning every dirty cell against every point in the candidate path: O(candidates × dirty cells × path length). Large rooms increase Python CPU requirements and may obstruct deployment on inexpensive single-board computers.

The new private route scorer checks cells belonging to a mathematically identical **0.11 m corridor disk** around each route coordinate using bounded 2D integer offsets, then counts distinct dirty targets once. For unusual map resolutions with corridor radius greater than 32 cells, it retains the original exact scan to avoid an oversized neighborhood allocation. This changes only scoring cost; it never issues goals, credits cleaning or relaxes the existing literal costmap-admission predicate.

## Reproduce (exact commit linked in upstream PR #79)

- Focused equivalence: PYTHONPATH=src python3 -m unittest discover -s tests -p test_sweep_target_resource_20261029.py -q
- Previous target checks: PYTHONPATH=src python3 -m unittest discover -s tests -p test_sweep_target.py -q
- Full: PYTHONPATH=src python3 -m unittest discover -s tests -q
- Optimized Python: PYTHONPATH=src python3 -O -m unittest discover -s tests -q
- Benchmark: PYTHONPATH=src python3 tests/bench_sweep_target_resource_20261029.py

## Repeatable local x86_64 baseline (Python 3.14.3, one pinned desktop core)

- One synthetic 120 × 68 map at 5 cm/cell; **2,132** dirty targets, 10 admission callbacks; three repetitions per implementation.
- Median process CPU seconds: **0.354458 before → 0.071538 after**, an observed **79.82% reduction** in this isolated microbenchmark.
- Median elapsed seconds: **0.356756 before → 0.075373 after**.
- Process max RSS: **23184 KiB before; 22444 KiB after**; these are small, noisy whole-process high-water values and do **not** establish memory savings.
- Identical selection: **92-cell route**, SHA256 7cd6403e5112d92f176013767892be660da8dc5d8ab684e4c299a41066f93184, same callback count.
- Verified **240 seeded reference equivalence cases** over eight map resolutions, including very fine-grid fallback. New focal **4/4 PASS**, previous target tests **10/10 PASS**, full regression **659/659 PASS**, Python -O **659/659 PASS**, compileall and git diff --check PASS.
- [Complete before/after machine evidence](evidence/sweep_target_resource_20261029.json); [parent stable source commit](https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/7c783d884c60940fb8a734b709b0dca93395f7a8).

## Limits, reviewer checklist, integration/revert

This measures one **pure Python x86_64 synthetic path selector**, not full-stack ROS2, Gazebo, SLAM, Nav2, ARM SBC, electrical power, battery life, safety hardware or real vacuuming. It does not identify the minimum system RAM/CPU. No direct hardware testing was performed.

- [ ] Review the one-file functional change and deterministic full-route hash.
- [ ] Verify the existing dynamic keepout, costmap admission and physical interlocks on real ROS2/Gazebo before integration.
- [ ] Profile the full stack on an inexpensive ARM SBC under representative SLAM/Nav2/cleaning load, including worst-case latency/RAM and thermal margin.
- [ ] Decide global exclusive Nav2 owner and canonical OOMWOO API with maintainer.

Integration is selective: import/cherry-pick the private source change into the approved component, run all unit and live ROS2/Gazebo tests. Revert by a normal inverse commit if it conflicts, without weakening the safety checks. The versioned PR #79 pointer holds exact commit, test/CI URLs and review status. Do not infer hardware suitability from this benchmark.
