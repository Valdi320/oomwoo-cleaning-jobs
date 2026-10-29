# OOMWOO contributor experiment: reactive sweep keepout envelope (2026-10-29)

**Status:** isolated Apache-2.0 contributor-owned Python prototype. Never used to drive physical motors. No ROS2 node, Gazebo run, MCU safety, or upstream code change in this commit. Built from local 39d87156c8d43c34f19b06b5112b4080f92a716d. Do **not** assume this has been accepted into OOMWOO. The maintainer's 2026-10-29 direction keeps Nav2 for inter-area transits/docking/localization and gives room sweeps to a reactive /cmd_vel executor.

## Why this closes a real gap

Nav2's KeepoutFilter does NOT prevent a separate reactive executor from publishing directly to /cmd_vel across a forbidden region. The existing three-way velocity mux validates ownership, health and command bounds, but has **no geometric collision preview**. On a simulated safe health witness, it would pass a valid sweep command without knowing that its next 0.8 s path intersects a virtual wall. Thus ROS2 keepout configuration alone is insufficient.

The new file src/oomwoo_cleaning_jobs/reactive_keepout_envelope.py adds an **opt-in, sweep-only final output gate**. Before forwarding a nonzero command, it independently checks (1) a trusted, fresh pose, (2) aligned and recently attested static map and keepout mask, (3) live costmap, and (4) the full swept circular robot footprint along a bounded differential-drive arc, using the project's existing exact swept_disk_clear raster primitive. Any failed gate returns ZERO and revokes the active synthetic motion lease until externally rearmed. Missing data, unknown cells, map-edge truncation, swapped forbidden mask or out-of-date evidence all fail closed.

Each arc is broken into segments of at most 0.08 s and 0.10 radians. Checking each straight chord with an additional radius of |linear_velocity * angular_velocity| * segment_duration² / 8 bounds the unicycle curve's lateral deviation. The configured footprint radius (example 0.21 m), additional stop/reaction margin (example 0.06 m) and horizon (0.8 s) are **illustrative only**: they are not measured stopping-distance or collision guarantees for any physical robot.

The mask is validated as binary 0/100 and geometrically aligned to the saved map. An **unchanged immutable Raster mask** is fully validated only once instead of rescanning every pixel every output tick; a newly provided mask object is always revalidated, while geometry and evidence freshness are checked every tick. This reduces per-cycle CPU overhead in this specific synthetic workload without skipping new masks.

## Reproduce exact Linux tests

    PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_reactive_keepout_envelope_20261029.py' -q
    PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
    PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
    PYTHONPATH=src python3 benchmarks/bench_reactive_keepout_20261029.py
    git diff --check

**Actual x86_64 Fedora/Bazzite Python 3.14.3 results:** new focused **22/22 PASS**; full regression **766/766 PASS**; optimized Python **766/766 PASS**. All are synthetic; CI for this new branch is not yet run.

Same-process 400×400 (160,000-cell) pure-Python steady-output microbenchmark, 100 calls/block ×3, shows median **3.22561 ms/call** when deliberately forcing a full binary-mask re-scan every time versus **0.23311 ms/call** with immutable-mask reuse (**13.84× reduction of this output microbenchmark**). These are measured on the user's x86_64 PC, **not** robot throughput, ARM SBC capacity, energy, minimum RAM, Gazebo performance or BOM savings. Evidence and exact commands: [JSON record](evidence/reactive_keepout_envelope_20261029.json). The synthetic benchmark holds one logical timestamp fixed, so it does not measure ROS2 cadence, callback scheduling or real timing jitter.

## Proposed integration contract — pending work (not executed)

1. A future trusted ROS2 adapter must consume *original* odometry pose timestamps in a consistent monotonic clock domain and attest versions/freshness of actual static/live maps and keepout mask. Never fabricate freshness from callback receipt time or trust self-declared source labels.
2. Every final **reactive sweep** output path must call this gate. The underlying mux may not simultaneously publish to /cmd_vel; exactly **one** authorized output publisher must own the physical command topic. Nav2 and dock outputs require their own explicit interlocked owner paths.
3. Maps used in the output decision must refer to a coherent revision and robot pose; this module does not prove cross-topic consistency, footprint calibration, map-to-odom TF, ROS2 publisher identity or lifecycle state. When map revisions change, the adapter must provide a *new immutable mask instance*.
4. Add an isolated ROS2 Jazzy/Gazebo probe on a **test-only velocity topic** proving that a mask-only forbidden obstacle blocks a direct reactive command; test dropped/delayed/rotated maps, single output writer, and controller restart. Never connect this prototype to live motors.
5. MCU watchdog, e-stop, cliff/bumper safety, demonstrated physical braking distance, actuator delay, lateral slip, localization covariance, obstacle inflation and real battery/docking are not provided by this software. A software STOP result does **not** certify a physical motor stop.

**Revert/disable:** do not instantiate ReactiveKeepoutEnvelope; remove its new module, tests, benchmark and documentation. All pre-existing mux APIs, Nav2/Gazebo launch files, firmware and default execution paths remain unchanged. The upstream [PR #79](https://github.com/makerspet/oomwoo/pull/79) is intentionally untouched pending maintainer review; this is a separate reactive architecture experiment, not a silent PR #79 extension.

**Review checklist:** [ ] Proven true map/odom timestamp provenance and version coherence; [ ] configurable physical stop horizon; [ ] one ROS2 output publisher; [ ] real Gazebo keepout and odometry faults; [ ] no motion until safe; [ ] ARM SBC load; [ ] maintainer API/architecture approval.
