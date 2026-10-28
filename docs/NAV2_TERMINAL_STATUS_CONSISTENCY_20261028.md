# Nav2 terminal status consistency (2026-10-28)

Owner: Valdi320. License: Apache-2.0. Parent: f7f475f1b93ac23d44f4338888def781567b3186.
Existing upstream pointer PR: https://github.com/makerspet/oomwoo/pull/79

## Bug

An exact journaled Nav2 UUID was observed ACTIVE (status 2), then both
the persistent GoalJournal and the ROS2 snapshot said CANCELED (5). The
isolated relay witness correctly permitted software dock handoff.
But if a later status for the SAME UUID contradicted this with SUCCEEDED
(4), the witness ignored the new outcome after its first terminal event.
The old software handoff permit remained true.

## Change

On each later snapshot naming our terminal UUID, compare the observed
terminal kind to the previously witnessed kind. Contradiction latches
a fault and prevents handoff. Returning to the prior status cannot
rearm. Matching repeats and terminal statuses for other UUIDs remain
valid. Existing journal validation and unknown/duplicate checks remain.

## Reproduce (ordinary Python, no ROS2 dependency)

    PYTHONPATH=src python3 -m unittest discover -s tests -p test_nav_lease_witness.py -v
    PYTHONPATH=src python3 -m unittest discover -s tests -q
    PYTHONPATH=src python3 -O -m unittest discover -s tests -q
    PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo

Five new regressions: canceled->succeeded, succeeded->canceled,
aborted->succeeded, unchanged terminal with unrelated completed goal,
and latched fault when the original terminal returns.
Before applying the patch the new negative tests reproduce the defect;
machine results are in docs/evidence/terminal_status_consistency_20261028.json.

## Limits

Only pure-Python host tests. No live ROS2/Nav2/Gazebo, no physical
motors, battery, MCU, real docking or dirt removal. ROS2 status producer
is not authenticated and this is NOT a physical safety controller.
No first-party OOMWOO runtime or third-party source changed.
