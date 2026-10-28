# Full-mask-area admission proof — 2026-10-28

Owner: Valdi320; separate, Apache-2.0 cleaning-jobs implementation. AI-assisted design. Based on commit `22e1a857fae3f1b74c58b48e0c5e4625d6c1f5d0`. Proposed upstream pointer is already-open PR #79: https://github.com/makerspet/oomwoo/pull/79.

## Actual defect

In the existing `VerifiedGlobalKeepout` the global Nav2 costmap was tested at each forbidden mask-cell **centre only**. At mask resolution 0.10 m and global costmap resolution 0.05 m, only one of four subcells can be lethal while the guard claims the whole mask cell is enforced. An observed costmap may also contain a forbidden-cell centre but not its complete area. This is our experimental verifier's defect, not a demonstrated Nav2/motor safety bug.

## Proposed change

Use positive-area geometric overlap between every forbidden mask cell rectangle and all observed global costmap cells in map coordinates, including rotated grids. Require every overlapping observed cell to meet the lethal threshold; reject incomplete observation. The aligned-grid fast path avoids geometry loops when both grids are exactly identical; a two-million-candidate budget fails closed rather than blocking indefinitely. No dependencies or hardware cost.

## Test protocol

From the target repository root **on a new owner branch** after applying the package and integration edits:

```
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_keepout_raster_proof.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_nav_keepout_admission.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
```

The standalone geometry helper was executed locally: **9/9 PASS** and **9/9 under `python3 -O`**, syntax PASS. A centre-only negative control using exactly one lethal 0.05 m subcell demonstrates the old acceptance defect; the new full-area gate rejects it. JSON evidence is under `docs/evidence/`. After integration in the Valdi320 branch on the owner's Linux PC, focal geometry 9/9, integrated keepout admission 14/14, full 569/569, Python -O 569/569, syntax and synthetic demo all PASS. CI is separately verified after publication.

## Limits

No real ROS2/Gazebo, motor, MCU, physical occupancy or automated charging test in this iteration. No source authentication, no MCU-controlled stop. `ZoneUpdateTransaction.verify_sample` still has a separate local rolling-costmap centre-only check that remains for a future audited change. The area-verification budget needs Raspberry Pi profiling, and conservative failure may block an otherwise safe rotated/scaled filter. Do not use this as a stand-alone hardware safety system.
