# Fresh Nav2 status preflight for experimental cleaning-jobs (2026-10-28)

**Owner:** Valdi320; Apache-2.0; ChatGPT-assisted independent module.
**Base:** 6dca3bbd0ad708f53683f14eeee7dd626593ab38.
**Upstream pointer:** https://github.com/makerspet/oomwoo/pull/79
**Scope:** contributor's experimental Python ROS2 adapter/node and pure-Python helper. No makerspet runtime, firmware, CAD, PCB, or physical robot changed.

## Defect and boundary

The private simulation-only node previously passed confirm_backend_idle(verified_no_outstanding_goals=True) during start without a fresh Nav2 action status admission check. The spot/room dispatch paths could send a new goal even if the most recently received status reported a different client's active/canceling goal. The transport itself had no final pre-send callback. This was a code-level **admission assumption**, not a demonstrated conflict in live Gazebo.

## Small fix

- fresh_nav2_status_idle requires a well-formed fresh (UUID, status) snapshot from one monotonic timebase (maximum 2.5 s), disallows unknown/duplicated UUIDs and all ACTIVE/EXECUTING/CANCELING goals; future-dated and nonfinite clocks are denied.
- Before persisting a NEW spot or rooms job, the node requires this status preflight.
- The same check runs before every room/spot dispatch, plus immediately before AsyncNav2Transport.send_goal builds the request, writes its journal, or invokes send_goal_async. Standalone transports retain optional pre_send for backward compatibility; the private node wires it explicitly.
- Failure does not send a new ROS2 goal; no speculative or time-based automatic rearming.

## Reproduce without ROS2 (Linux Python 3.10+)

    PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_nav2_fresh_status_preflight_20261028.py' -v
    PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal*.py' -q
    PYTHONPATH=src python3 -m unittest discover -s tests -q
    PYTHONPATH=src python3 -O -m unittest discover -s tests -q
    python3 -m compileall -q src tests
    PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo

**Observed on user's Linux PC, Python 3.14.3:** focal 13/13 PASS; goal journal 30/30 PASS; full suite 614/614 PASS; Python -O 614/614 PASS; syntax and synthetic demo PASS. Machine record: [docs/evidence/nav2_fresh_status_preflight_20261028.json](evidence/nav2_fresh_status_preflight_20261028.json).

Focal tests use synthetic Nav2 statuses and a fake ActionClient. A source AST regression ensures the real ROS2 node keeps the preflight at spot/room dispatch sites and the transport send boundary. No actual ROS2 middleware calls in this iteration.

## Integration and precise limits

This is NOT a durable global Nav2 single-owner broker. ROS2 status can be delayed, incomplete, misrouted, or change after checking; there is no trusted DDS source. Live ROS2/Gazebo validation (especially status heartbeat at idle), multi-client collision tests, and durable arbitration remain necessary. An absent status now refuses new navigation rather than guessing. This cannot stop a physical motor and does not replace MCU emergency stops.

Maintainer decisions: canonical cleaning API and single authority for Nav2 action ownership. To roll back after selective import, use a normal revert commit, repeat regression, do not erase unresolved goal journals or force-push others' history.
