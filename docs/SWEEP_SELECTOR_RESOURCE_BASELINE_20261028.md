# OOMWOO measured sweep: one-traversability snapshot and resource baseline

Synthetic scope only. The independently supplied 0/100 coverage meter remains the sole basis for skipping goals: there is **no automatic cleaning credit** from a Nav2 completion.

## Problem and fix
Each sampled cell previously evaluated `self.grid.traversable`, which constructs a new traversable set from the entire map. A 30x18 three-room simulation repeated this during every scan of a large waypoint list. The updated selector takes **one local immutable snapshot per selection**, after checking the same source map, and uses it for scope, exclusion and swath checks. There is no global stateful cache or altered keepout policy.

## Reproduce
`PYTHONPATH=src python3 tests/bench_home_resources.py --seed 0 --one-core`

`PYTHONPATH=src python3 -m unittest discover -s tests -q`

`PYTHONPATH=src python3 -O -m unittest discover -s tests -q`

## Measured host-specific baseline (two runs each)
- Same original source SHA: `ac4ccd7c83691cab589ad21653886a906b6473eb`; same deterministic seed and one pinned x86_64 host CPU core.
- Median wall time: **9.104s before**, **0.796s after** (observed **91.3% reduction**).
- Peak process RSS (largest per run): **23168 KiB before**, **23832 KiB after**.
- Synthetic three-room output identical: 530 verified cells, 125 synthetic goals, 0 keepout violations.
- Full machine evidence: [JSON](evidence/selector_one_snapshot_resource_20261028.json).
- Regression suites: normal and -O **PASS**; 3 new focused tests PASS.

## Limits for reviewer
This is a **CPU/RAM measurement of a pure-Python synthetic program**, not full OOMWOO Nav2/Gazebo or any physical controller. Single-core affinity on a desktop is **not** a Raspberry Pi or ARM benchmark. No power, thermal, MCU or robot safety measurements. Actual minimum SBC must be established using Nav2+SLAM+ROS2+hardware drivers, repeatability and peak memory margin.

Integration: review this small source diff and cherry-pick it into the appropriate independent module. Roll back via normal git revert if tests fail; do not force push. Maintainer approval of interfaces remains pending.
