# OOMWOO persistent room coverage and dock integration — 2026-10-27

## Verified with real Nav2 and Gazebo

The private, long-running ROS2 cleaning controller can now start a persisted, operator-labelled digital room. It uses the original OOMWOO sweep-planner geometry WITHOUT spinning its competing action executor, and grants cleaned-cell credit ONLY after the original Gazebo ground-truth coverage meter reports newly covered cells.

Actual post-run inspection in the isolated official OOMWOO test_room:
- 171 of 171 eligible target cells verified in the CleaningJobs checkpoint; state completed.
- 171 of 171 cells independently confirmed in the persisted SweepCampaign; state completed.
- 11 NavigateToPose requests in the real Nav2 log, with final terminal canceled result after externally measured completion.
- The goal UUID journal reached a terminal canceled state, and the WAL/campaign credited-cell sets matched.
- The selected area Zona_simulada_A is an OPERATOR-DEFINED VIRTUAL ROOM on a physically OPEN test-room floor. It is not an architectural room.

Machine-readable checkpoint snapshot: docs/evidence/real_room_partial_20261027.json (historical filename; content states completed). No real vacuum/dirt removal or MCU safety was tested.

## Later: one persistent two-zone REAL Gazebo completion

After this initial single-zone 171/171 test, the same persistent
controller completed TWO separately labelled operator-defined
virtual areas in one job: **80/80 + 40/40 = 120/120** cells from the
original OOMWOO external simulated ground-truth meter. There were
18 accepted real Nav2 goals, 12 recorded gap repairs and both
CleaningJobs/SweepCampaign durable ledgers reached completed with
zero pending WAL events. The last Nav2 goal ended terminal canceled
after the externally measured scope completed.

Evidence: [two-zone report](REAL_TWO_ZONES_AND_DOCK_RELAY_20261027.md).

These remain two digital regions on an open simulated floor, NOT
physically separated real rooms or proof of real dirt removal.

## Real existing docking software and limitations

The upstream oomwoo_dock package contains a dock detector estimating dock mouth geometry and a dock-drive node that controls cmd_vel and publishes a latched docked Bool. We have tested conservative geometry and exclusive cmd_vel source-handoff state machines in pure Python.

The OOMWOO dock-drive and Nav2 must NEVER both command the real robot cmd_vel. The candidate handoff contract first requires terminal Nav2 action/UUID reconciliation and quiet observed odometry, then an exclusive and verified motor-command mux. A docked Bool alone does not confirm charging, dust emptying or mop washing.

Our durable DockCycle service requests and current resource-telemetry gates are tested with SYNTHETIC station acknowledgements. There is no demonstrated charging power, bin-empty sensor or mop-washing hardware provider in the OOMWOO test_room. Thus these actions cannot be validated physically in digital-only testing and are NOT 100% complete.

## Remaining engineering

Verify physically divided/furnished multi-room maps, docking integration using a REAL ROS2 exclusive command mux, dynamic recovery from localization failures, real charging/empty/wash provider protocols, and physical MCU watchdog/cliff/bumper/charge tests. The independent source still needs maintainer review and PR merge.

This report distinguishes demonstrated digital room completion from unresolved real dock/hardware requirements.
