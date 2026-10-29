# OOMWOO: sequenced evidence ingress for three-way motion ownership (2026-10-29)

**Status: local experimental contributor branch, NOT published, NOT integrated with ROS2, Gazebo or motors.** Owner: Valdi320; independent Apache-2.0 code based on `664c5c06a9c969c5607c219ff53d08204ef143d7`. Maintainer's 2026-10-29 direction: reactive room coverage owns sweep commands; Nav2 is retained for transits, docking, localization. No upstream repository file or PR #79 is changed.

## Concrete blocker and remediation

`ThreeWayVelocityMux.observe_health(now=..., proof=...)` and `observe_odom(now=...)` originally treated callback receipt time as freshness. An upstream adapter could accidentally re-submit an old odometry/health sample with a fresh receipt timestamp, indefinitely renewing a motion lease even though measurements were stale. A sensor reboot could also replay a previously observed sequence.

New opt-in `SequencedMotionMux` in `src/oomwoo_cleaning_jobs/sequenced_motion_ingress.py` wraps the three-way mux and requires `EvidenceEnvelope(source, sequence, sampled_at)` for **both** health and odometry. It rejects and STOPs upon:
- expired or future sample timestamps (max 200 ms default, configurable but not wider than the mux watchdog);
- non-increasing per-source sequence IDs or measurement times, including replay immediately after STOP;
- unrecognized source IDs, non-integer/boolean/negative/oversize sequence IDs and invalid/nonfinite timestamps;
- unsafe health changes and out-of-range commands through the unchanged downstream mux policy.

Two sources have separately monotonic sample streams. This is deliberately conservative: a dropped sample, source restart or clock discontinuity may require manual re-reconciliation. It does **not** itself authenticate the declared source.

## Reproduce (Linux Python, no ROS2 required)

```bash
cd /path/to/oomwoo-cleaning-jobs
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_sequenced_motion_ingress_20261029.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
git diff --check
```

**Executed on PC x86 Linux:** focused **16/16 PASS**, full **744/744 PASS**, optimized Python **744/744 PASS**; `git diff --check` PASS. Exact command/result record: `docs/evidence/sequenced_motion_ingress_20261029.json`. Compared with previous local suite of 728 tests, 16 new safety tests are added. No CI result can be claimed from this local commit.

## Strict integration contract

1. An external trusted ROS2 adapter must derive the sample's timestamp in the same monotonic domain as `now`, using validated clock translation if ROS time is used; never substitute callback receipt time for the original measurement time.
2. Publisher identity and message sequence must be verified before ingress; a self-declared string `source` and a steadily increasing counter do **not** prove sensor authenticity.
3. Nav2 UUID/terminal confirmations, obstacle maps, physical safety signals and isolation of all other `/cmd_vel` writers must be validated independently. The `MotionHealth` flags are still caller claims.
4. A subsequent isolated ROS2 proof should publish only to a **sim/test-specific topic**, audit the topic graph for exactly one genuine output writer, inject delayed/reordered messages and record dynamics. No real `/cmd_vel` publisher in this change.
5. MCU watchdog, physical e-stop, inertial/encoder checks, collision behavior and actual battery/dock interfaces remain untested.

**Revert:** revert only the future commit adding the three files below; this is a new opt-in class, so legacy `ThreeWayVelocityMux` behavior and prior tests are unchanged. No published remote branch has been altered. PR #79 is deliberately quiet pending maintainer review. No ARM performance, RAM, power or BOM reduction is claimed.
