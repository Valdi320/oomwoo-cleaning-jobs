# Nav2 live goal-journal recheck (2026-10-28)

Owner: Valdi320; separate Apache-2.0 cleaning-jobs repo; base e82f29ca01da87b9e18f4798023c6eedbe93dc9b.

Failure: an observed ROS2 ACTIVE status remained cached after durable journal became terminal. current_active_matches() used memory flags only, potentially relaying nonzero experimental Nav2 velocity until another status callback.

Fix: on each command gate call, re-read the fsync'd exact UUID GoalJournal, require matching UUID and accepted state; terminal returns False immediately; missing/corrupted/replaced state latches fault. Terminal journal alone NEVER authorizes dock handoff.

Reproduce: PYTHONPATH=src python -m unittest discover -s tests -p test_nav_lease_witness.py -v

Regress all: PYTHONPATH=src python -m unittest discover -s tests -q

Optimized: PYTHONPATH=src python -O -m unittest discover -s tests -q

Demo: PYTHONPATH=src python -m oomwoo_cleaning_jobs.demo

Negative control (new tests vs old code): expected FAIL reproduced. After fix: focused/full/optimized/demo PASS, local Python only.

No actual ROS2/Gazebo or motor/MCU/hardware testing. This is an experimental isolated output mux: ROS2 status is not authenticated; no hardware interlock/actuator stop certified. The durable journal may itself be stale relative to server status, so cannot replace fresh trusted action checks.
