# Reject unknown Nav2 GoalStatus as idle (2026-10-28)

Owner: Valdi320; isolated Apache-2.0 contribution, parent 08511a1b275e695d7cc04dfa1c5294bf345bcb3b.

Defect: the old isolated ROS2 relay checked only status 1/2/3 for active goals. A fresh GoalStatus.STATUS_UNKNOWN=0 was mistakenly treated as idle, and NavLeaseWitness ignored it when an existing lease was ACTIVE. A negative control reproduced both unsafe assumptions in the previous source before applying this patch.

Fix: validate unique exact 32-lowercase-hex goal UUIDs and status integers 1..6 for the whole snapshot; reject malformed, duplicate, 0/unknown, out-of-range or bool status, fail closed during Nav2 witnessing and dock ownership. The ROS2 adapter uses the same tested pure-Python idle predicate. Empty array still means no known active goal; terminal release separately requires own UUID witness.

Reproduce on Linux without ROS2:

    PYTHONPATH=src python -m unittest discover -s tests -p test_nav_lease_witness.py -v
    PYTHONPATH=src python -m unittest discover -s tests -q
    PYTHONPATH=src python -O -m unittest discover -s tests -q
    PYTHONPATH=src python -m oomwoo_cleaning_jobs.demo

Verified local: pre-fix negative control PASS (unsafe handling reproduced), focused 19/19, full 475/475, optimized 475/475, demo PASS.

Limitations: pure Python on host only; NO ROS2/Nav2/Gazebo live test this iteration, NO physical motor/battery/MCU tests, no authenticated ROS2 producer, no certified watchdog. Not production safety. No shared upstream runtime or third-party code changed.
