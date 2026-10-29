# Local residual gap sweeper CPU reduction — synthetic, 2026-10-29

**Ownership and scope.** Independent Apache-2.0 Valdi320/oomwoo-cleaning-jobs contribution. It does not alter makerspet firmware, CAD, motor authorization, charging, upstream robot navigation or any real hardware.

## Issue and fix

The experimental residual-gap line-through planner scanned *all* uncovered cells for every geometric sample and again for every admitted route cell, and repeatedly regenerated the full traversable grid for candidate shortest paths. That used CPU unnecessarily on prospective low-cost SBCs.

- Enumerate only the exact swept-disk cell neighborhood, using the original inclusive Euclidean distance test on candidates. An unusually fine map (radius >32 cells) retains the exact full-scan fallback to avoid massive allocation.
- Check admitted-route proximity against constant-size Manhattan-radius-three offsets rather than route × all dirty cells. The **existing Nav2/costmap admission callback remains mandatory** and is not relaxed.
- Memoize the Grid's immutable traversable frozenset **per Grid instance only when the entire map is at most 65,536 cells**. Larger maps are deliberately not pinned in RAM. Frozen input snapshots and original keepout inflation are preserved.
- Cleaning credit still comes exclusively from the independent measured meter, never from a proposed route or a Nav2 success.

## Reproducible focal, regression and benchmark

Run in the source repository:

```sh
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_line_gap_neighborhood_20261029.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_line_gap_route.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
PYTHONPATH=src python3 tests/bench_line_gap_neighborhood_20261029.py
```

**Measured on one pinned desktop x86_64 core, Python 3.14.3:** one 120 × 90 map at 5cm and 630 simulated uncovered cells; three repetitions on the same host and benchmark program. Before/after reference is [parent e843df5](https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/e843df534205684f0b9b498d0fd1fc331e997e2a).

- Median process CPU: **1.220287 s before → 0.029714 s after; 97.56% lower** for this selected synthetic workload.
- Median wall clock: **1.228272 s → 0.029842 s**.
- Same **42-cell path**, same SHA256 `65037a5b74e8105fbbf8f5612ea5dadb09248f19319dcd486ff9253ae1f4de83`, same **10 admission callbacks** and same candidate-goal choice.
- Process max RSS: **22576 → 22312 KiB** (no RAM-saving inference; distinct Python processes, noisy high-water metric).
- **5/5 new focal PASS** (525 seeded geometric parity cases, 300 route proximity parity cases; cache-bound, map edge and exact path contract), prior line-gap 6/6 PASS; **664/664 full and 664/664 Python -O PASS**, compileall and git diff --check PASS.
- [Machine evidence including each repetition](evidence/line_gap_neighborhood_resource_20261029.json).

## Limits / risks / review / revert

This is an isolated pure-Python x86 microbenchmark. **No full-stack ROS2/Nav2/Gazebo run, ARM SBC test, thermal/energy measurement, hardware stop-chain validation, actual dirt removal or minimum computer price claim.** Performance and memory elsewhere in OOMWOO remain unknown. Cache lifetime adds an immutable frozenset to small Grid objects, constrained per object by the 65,536-cell cutoff; long-lived collections of many Grids still merit profile tests.

- [ ] Review the two source files, exact route hash, seeded reference parity and keepout coverage.
- [ ] Run with real ROS2/Nav2/Gazebo (keepout hot reload, navigation preemption, recovery and costmap availability).
- [ ] Profile actual full-stack CPU, peak RSS, deadline latency and power on candidate cheap ARM hardware before recommending a board.
- [ ] Maintainer must decide canonical cleaning-jobs integration and exclusive global Nav2 owner.

Integrate only into a maintainer-approved component after its live safety testing. Revert both source changes with an ordinary inverse commit if integration causes regressions; do not override upstream motor/stop interlocks. No upstream runtime changed here.
