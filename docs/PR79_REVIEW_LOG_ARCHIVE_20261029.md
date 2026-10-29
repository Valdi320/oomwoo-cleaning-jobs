# Archived PR #79 history as of 2026-10-29

Historical reviewer pointer, preserved in the contributor-owned repository on maintainer request. Not a live CI claim.

---

# Valdi320 — cleaning-jobs: current reviewer pointer

**Independent source (Apache-2.0):** https://github.com/Valdi320/oomwoo-cleaning-jobs

**Latest tested source SHA:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/4085115e7a4bc488bf013353c8ac4cc22cd1a5ed

## Maintainer quick review (2026-10-29): concurrent WAL receipt safety

- **Problem:** concurrent writers could independently acknowledge durable coverage receipts even though one atomic JSON replacement discarded the other's data; a stale instance could falsely declare a drained ledger. Existing symlink WALs were accepted, and FIFO reads could hang.
- **Fix:** private persistent nonblocking `flock` covering initialization, `stage`, and `project`; on-disk WAL compared to the full snapshot under that lock; stale `pending/require_drained` refuses success. No-follow/nonblocking regular/private inode validation and 64-MiB file limit. No shared OOMWOO runtime or motor/charging code changed.
- **Tested source SHA:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/4085115e7a4bc488bf013353c8ac4cc22cd1a5ed
- **Exact comparison:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/e3689b687329ac2470fcbcf091b2ac55c992ba17...4085115e7a4bc488bf013353c8ac4cc22cd1a5ed
- **Own branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-wal-writer-20261029-0744
- **Reproduce:** check out the exact source commit, then `PYTHONPATH=src python3 -m unittest discover -s tests -p test_coverage_wal_writer_safety_20261029.py -v`; run `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q`; repeat with `python3 -O`. The pure Python tests use Linux; no physical robot, firmware flash or motor commands.
- **Evidence:** before patch 0/7 new focal PASS (6 FAIL, 1 FIFO timeout ERROR); after patch **7/7 focal PASS, 683/683 full PASS, 683/683 Python -O PASS**, diff check PASS. [Review and limits](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/4085115e7a4bc488bf013353c8ac4cc22cd1a5ed/docs/WAL_WRITER_SAFETY_20261029.md) and [machine evidence](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/4085115e7a4bc488bf013353c8ac4cc22cd1a5ed/docs/evidence/wal_writer_safety_20261029.json).
- **Exact-source CI:** https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40832127341 — completed/success (Pure-Python regression).
- **Compatibility/risk:** busy locks fail closed; stale instances must reopen and reconcile. Rejects insecure/special/oversized WALs and requires a stable `.lock` file on one local filesystem. This does not secure hostile same-user races, remote network filesystems or Nav2 ownership; not measured on Gazebo, an ARM SBC or hardware.
- **Maintainer review:** [ ] inspect inode/locking semantics, [ ] replay real Gazebo ROS2 receipt and restart cases, [ ] benchmark ARM memory/CPU, [ ] ensure global Nav2 ownership and physical motor watchdog. Integrate only via approved selective cherry-pick; revert with `git revert 4085115e7a4bc488bf013353c8ac4cc22cd1a5ed` if unsuitable. Review pending; no PR duplicate or routine comment.

---

## Previous review snapshot (2026-10-29): revoke stale dock-completion proof

- **Safety defect:** experimental exclusive velocity mux accepted SEARCH(False) -> DONE(True) -> SEARCH(False) but retained the earlier dock-complete flags. Subsequent quiet odometry could let finish_dock return geometric_docked=True after the new undocked report. Contradictory IDLE(False) and GOTO(True) also left stale completion evidence.
- **Minimal owner-only fix:** after a witnessed DONE(True), any changed dock-drive state or false dock-contact retracts completion, resets the mux to STOP, and raises DockHandoffError. Repeated DONE(True) stays valid; normal intermediate states before DONE remain valid. The experimental ROS2 relay forwards docked=False after confirmed DONE, instead of silently ignoring that callback. No output topic, motor interface or upstream runtime changed.
- **Full owner source SHA:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/e3689b687329ac2470fcbcf091b2ac55c992ba17
- **Exact comparison:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/4519fca8dd932262b35ad063aaf05e7be0f47a7c...e3689b687329ac2470fcbcf091b2ac55c992ba17
- **Contributor branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-dock-proof-revocation-20261029-0644
- **Reproduce:** check out exact source commit; run PYTHONPATH=src python3 -m unittest discover -s tests -p test_dock_proof_revocation_20261029.py -v; then PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py', and repeat with python3 -O. Full clean worktree and git diff --check.
- **Evidence:** before the fix, 3/5 new focal tests failed as expected on source parent 4519fca (post-DONE SEARCH, IDLE, GOTO). After: **7/7 focused PASS**, existing docking regression **13/13 PASS**, full Python **676/676 PASS**, Python -O **676/676 PASS**, diff check PASS. [Source verification and limitations](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/e3689b687329ac2470fcbcf091b2ac55c992ba17/docs/DOCK_PROOF_REVOCATION_20261029.md).
- **Exact-source CI:** https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40825840894 **completed/success** (Python 3.10 and 3.12 jobs PASS).
- **Limitations:** pure Python deterministic Linux tests; ROS2 callback adapter is only code-reviewed, **NOT run in a live ROS2/Gazebo graph**, on an ARM SBC or real robot. Dock status/Bool topics remain unauthenticated. This does not validate charging, physical docking, station services, battery/motor watchdog or hardware stop.
- **Compatibility/risk:** a contradictory post-DONE message now fail-closes to STOP; replayed/disordered dock telemetry may force a fresh docking attempt. The isolated lab mux remains deliberately unusable directly on real /cmd_vel.
- **Review and integration:** [ ] review post-DONE revocation and callbacks; [ ] replay ROS2 Jazzy bag with state/contact reordering; [ ] verify no other cmd_vel publisher or Nav2 owner; [ ] validate motor-safe hardware interlock; [ ] test on a budget SBC when approved. Cherry-pick only the owner commit into a selected cleaning-jobs implementation after maintainer approval; do not change the main upstream sources. **Revert:** ordinary git revert e3689b687329ac2470fcbcf091b2ac55c992ba17 in the owner implementation; no history rewrite. Maintainer review pending; no new PR and no routine comment.

---

## Previous review snapshot (2026-10-29): exact bounded grid-clearance CPU improvement

- **Problem:** per-obstacle square inflation of occupied/unknown/keepout grid cells incurred repeated Python work with dense furniture maps and high conservative clearance, an avoidable cost for prospective inexpensive SBC hardware.
- **Minimal contributor-only fix:** Grid.blocked now adaptively uses exact per-row spans (sparse maps), 2D rectangular difference/prefix sum (dense maps), or full-boundary short circuit; original Chebyshev inflation, no-go geometry and outer-map margins preserved. Dense temporary integer array is bounded to 4*(w+1)*(h+1) bytes on the tested platform; no new external dependencies, ROS2/Nav2/hardware paths or motor decisions changed.
- **Exact owner commit:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/4519fca8dd932262b35ad063aaf05e7be0f47a7c
- **Review comparison:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/36bf42537194d2593240d7db76c5f726273c7ff3...4519fca8dd932262b35ad063aaf05e7be0f47a7c
- **Owner-only branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-grid-inflation-resource-20261029-0549
- **Synthetic x86_64 Python 3.14.3 CPU benchmarks, three runs each, matching SHA256 blocked sets:** furnished 256x192 room (clearance 7): median process CPU **0.060451s original -> 0.010890s revised (81.99% less)**; adversarial dense 256x192 map (clearance 10): **0.353394s -> 0.020558s (94.18% less)**. First map retains 34,590 traversable cells. These are local microbenchmarks, **NOT whole robot CPU/power/RAM savings**.
- **Tests:** 5/5 new focused PASS, including 300 seeded map reference parity cases, dense and sparse branch checks, blocked keepouts and map-boundary equivalence; full regressions **669/669 PASS**, Python -O **669/669 PASS**, git diff --check PASS.
- **Exact-source CI:** https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40820575099 **completed/success** (unit (3.10), unit (3.12)).
- **Reproduce:** checkout exact commit; run PYTHONPATH=src python3 -m unittest discover -s tests -p test_grid_inflation_resource_20261029.py -q; run PYTHONPATH=src python3 -m unittest discover -s tests -q and again with python3 -O; benchmark via PYTHONPATH=src:tests python3 tests/bench_grid_inflation_resource_20261029.py.
- **Machine evidence:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/4519fca8dd932262b35ad063aaf05e7be0f47a7c/docs/evidence/grid_clearance_resource_20261029.json ; [explanation, compatibility, risk and revert](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/4519fca8dd932262b35ad063aaf05e7be0f47a7c/docs/GRID_CLEARANCE_RESOURCE_20261029.md).
- **Limits/risk:** pure-Python Linux x86 only, not real ROS2/Jazzy/Gazebo, low-cost ARM SBC or physical robot; dense path uses bounded temporary RAM; no thermal, battery, power, deadline-latency or dirt-removal claims. No source runtime or licensing of upstream makerspet projects changed.
- **Review/integration/revert:** [ ] verify exact blocked-map parity and keepout, [ ] run real Jazzy/Nav2/Gazebo restart/keepout cases, [ ] profile full-stack RAM/CPU/latency/power on budget ARM, [ ] settle sole Nav2 owner and canonical cleaning API. Integrate only after maintainer approval and live testing; revert the isolated commit normally if incompatible. **No new PR or routine comment.**

---

## Previous review snapshot (2026-10-29): residual-gap CPU optimization, 97.56% in one synthetic benchmark

- **Problem:** a residual-gap line-through sweep scanned every dirty cell for each sample and repeated full-map traversability reconstruction for each candidate route, wasting CPU on prospective budget hardware.
- **Scope/fix:** independent Apache-2.0 selector now checks only the exact local Euclidean sweep disk and constant Manhattan-radius-three route neighborhood; Grid retains immutable traversable snapshots only for maps up to 65,536 cells. Original costmap-admission callback, clearance, keepout and measured-clean-credit requirements are unchanged. No hardware or motor-control behavior modified.
- **Source commit:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/36bf42537194d2593240d7db76c5f726273c7ff3
- **Review diff:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/e843df534205684f0b9b498d0fd1fc331e997e2a...36bf42537194d2593240d7db76c5f726273c7ff3
- **Contributor branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-line-gap-neighborhood-20261029-0444
- **Benchmark:** desktop x86_64 Python 3.14.3 pinned to one core, 120x90 at 5 cm with 630 dirty cells, 3 runs/version. Median process CPU 1.220287s before to 0.029714s after (97.56% lower); wall 1.228272s to 0.029842s. Identical 42-cell path SHA256 65037a5b74e8105fbbf8f5612ea5dadb09248f19319dcd486ff9253ae1f4de83 and 10 admission callbacks. **No RAM reduction claimed.**
- **Validation:** 5/5 new focal PASS (525 seeded exact-disk equivalence cases, 300 Manhattan parity cases, cache/map edge/route contract), 6/6 previous line-gap PASS, 664/664 full Python PASS, 664/664 Python -O PASS, compileall/diff check PASS. [Exact-source CI](https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40814920088): completed/success, Python 3.10 and Python 3.12 jobs PASS.
- **Reproduce:** checkout exact source SHA; run PYTHONPATH=src python3 -m unittest discover -s tests -p test_line_gap_neighborhood_20261029.py -q; then PYTHONPATH=src python3 -m unittest discover -s tests -q; also repeat with python3 -O. Benchmark: PYTHONPATH=src python3 tests/bench_line_gap_neighborhood_20261029.py.
- **Evidence:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/36bf42537194d2593240d7db76c5f726273c7ff3/docs/evidence/line_gap_neighborhood_resource_20261029.json ; [design, risks and rollback](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/36bf42537194d2593240d7db76c5f726273c7ff3/docs/LINE_GAP_NEIGHBORHOOD_RESOURCE_20261029.md).
- **Limits and risks:** synthetic pure Python x86 benchmark **only**, no live Nav2/ROS2/Gazebo, ARM CPU/RAM/energy, full-stack thermal tests or physical robot. The bounded cache uses extra resident memory on each small Grid; multi-Grid memory profiling is pending. Do not extrapolate 97.56% to the whole robot.
- **Review / integration / revert:** [ ] verify exact distance/route parity and no-go admission; [ ] run Jazzy/Nav2/Gazebo keepout and restart tests; [ ] measure full-stack CPU/RSS/latency/power on affordable ARM; [ ] resolve exclusive Nav2 owner and public job interface. Selectively import the two module changes only after maintainer approval and real safety validation, and revert by ordinary inverse commit if incompatible. Upstream runtime files remain untouched.

---

## Previous review snapshot (2026-10-29): 79.82% less CPU for synthetic uncovered-route scoring

- **Problem:** fallback gap sweeper compared every dirty cell against every route point per candidate, raising CPU demand on prospective affordable hardware.
- **Correction:** private Apache-2.0 module now enumerates the exact 0.11m corridor neighborhood and counts each dirty cell once. At ultra-fine resolution the original exact algorithm remains as a safe resource-bounded fallback. No change in costmap admission, deterministic route, Nav2 control, safety interfaces, charging or motor authority.
- **Exact source SHA:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/e843df534205684f0b9b498d0fd1fc331e997e2a
- **Small source diff:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/7c783d884c60940fb8a734b709b0dca93395f7a8...e843df534205684f0b9b498d0fd1fc331e997e2a
- **Contributor branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-sweep-target-resource-20261029-0342
- **Verification on Linux:** focal equivalence 4/4 PASS (240 deterministic geometric reference cases across eight resolutions), prior sweep-target 10/10 PASS, full regressions 659/659 PASS, optimized Python 659/659 PASS; compileall and diff check PASS. The selected route SHA256, 92 cells and 10 admission callbacks match the original.
- **Measured local one-core x86_64 Python 3.14.3 benchmark:** 120x68 5cm cells, 2,132 dirty cells; three repeated runs per implementation. Median process CPU 0.354458s before, 0.071538s after: 79.82% lower in this isolated synthetic benchmark. RSS high-water 23184 KiB before / 22444 KiB after; NO memory-saving inference.
- **Exact-SHA CI:** https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40809197987 completed/success; jobs: unit (3.12) PASS, unit (3.10) PASS.
- **Reproduce:** check out exact source SHA; run PYTHONPATH=src python3 -m unittest discover -s tests -p test_sweep_target_resource_20261029.py -q, then PYTHONPATH=src python3 -m unittest discover -s tests -q and PYTHONPATH=src python3 -O -m unittest discover -s tests -q; benchmark via PYTHONPATH=src python3 tests/bench_sweep_target_resource_20261029.py.
- **Raw evidence:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/e843df534205684f0b9b498d0fd1fc331e997e2a/docs/evidence/sweep_target_resource_20261029.json
- **Reviewer guide, limitations, revert:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/e843df534205684f0b9b498d0fd1fc331e997e2a/docs/SWEEP_TARGET_RESOURCE_20261029.md
- **Scope:** pure Python x86_64 synthetic, not ROS2/Nav2/Gazebo or ARM SBC, power/heat/memory budget of the complete stack, physical cleaning, motors or MCU safety. No upstream runtime change. Global Nav2 owner still undecided.
- **Checklist:** [ ] review exact disk/route equivalence, [ ] replay with measured coverage + dynamic keepouts in live Jazzy/Gazebo, [ ] measure full-stack CPU/RAM/latency/power on budget ARM SBC, [ ] settle Nav2 sole owner and official interface. Selectively integrate the private module fix after approval and run full regressions; revert using a normal inverse commit if incompatible, leaving safety gates active. No new PR or comment.

---

## Maintainer quick review (2026-10-29): exact-UUID cancel service response is exclusive

- **Problem / 5 reproduced pre-fix negative controls:** during orphan Nav2 recovery, a successful `CancelGoal` reply was accepted if it included the old UUID among **other canceled UUIDs**, duplicates, a malformed container or boolean return code. The ROS2 service contract for nonzero UUID + zero timestamp targets exactly one goal; accepting a broader reply could mask interference with another owner.
- **Minimal fix:** `OrphanNav2Witness.cancel_response` now requires integer `ERROR_NONE (0)` and exactly `(known_old_uuid,)`; mismatches latch a permanent recovery fault and **never authorize rearm**. Exact valid reply still requires observed old ACTIVE, terminal CANCELED, fresh status and bounded quiet odometry.
- **Latest tested code SHA:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/7c783d884c60940fb8a734b709b0dca93395f7a8
- **Isolated comparison (one source file + 1 focal test + docs/evidence):** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/e5f4ccbc9b4eae2571563e95adef99854c050349...7c783d884c60940fb8a734b709b0dca93395f7a8
- **Owner branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-exact-cancel-response-20261029-0244
- **Executed local Linux checks:** pre-fix 5/6 focal tests failed as expected, post-fix **6/6 PASS**, previous restart **20/20 PASS**, previous orphan **8/8 PASS**, full **655/655 PASS** and `python -O` **655/655 PASS**; compile and diff checks PASS.
- **Exact-SHA CI:** [Python 3.10 and Python 3.12 / run 40802966980](https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40802966980) **2/2 PASS**, confirmed completed/success.
- **Reproduce:** checkout exact SHA; `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_orphan_cancel_reply_exclusive_20261029.py' -q`, then `PYTHONPATH=src python3 -m unittest discover -s tests -q` and `PYTHONPATH=src python3 -O -m unittest discover -s tests -q`.
- **Machine evidence:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/7c783d884c60940fb8a734b709b0dca93395f7a8/docs/evidence/nav2_exact_cancel_reply_20261029.json ; **scope / ROS2 spec / review / revert:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/7c783d884c60940fb8a734b709b0dca93395f7a8/docs/NAV2_EXACT_CANCEL_REPLY_20261029.md
- **Compatibility/risk:** no public API or durable schema change; strict rejection may halt automatic recovery when ROS2 service replies are inconsistent. **Synthetic Python Linux only**; no live ROS2/Nav2/Gazebo, ARM SBC, energy, motor safety, or DDS adversarial tests. Global single-owner Nav2 arbitration and independent MCU stop remain unverified.
- **Integrate / revert:** cherry-pick/select the isolated witness fix only with maintainer approval, run Jazzy/Gazebo exact-cancel request/response and restart regressions before physical use; normally revert the source commit if incompatible while leaving the robot disarmed.
- **Maintainer checklist:** [ ] approve the sole Nav2 goal owner and exact-cancel semantics; [ ] replay malformed/extra acknowledgment on Gazebo; [ ] keep physical motor-stop chain MCU-owned; [ ] characterize inexpensive ARM compute after full-stack validation. No upstream runtime edit or new discussion comment.

---

## Maintainer quick review (2026-10-29): require current orphan UUID cancellation proof

- **Problem (4 reproducible pre-fix counterexamples):** the isolated crash-recovery controller could rearm after an old-UUID cancel and a quiet odometry window even when the *latest* GoalStatusArray was empty, contained only unrelated terminal history, carried UNKNOWN(0), or duplicated contradictory statuses. Another transient foreign ACTIVE UUID could be forgotten after disappearing from subsequent status packets.
- **Fix:** validate each complete ROS2 status snapshot (well-formed unique UUID/code, no UNKNOWN or foreign ACTIVE), latch ambiguous evidence as a recovery fault, and require the **exact old UUID currently CANCELED** at the final Nav2 rearm boundary. Keep existing exact cancel ACK, fresh status receipt, simulation clock and continuous quiet odometry checks. A terminal history for other nonactive goals remains permitted.
- **Source SHA:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/e5f4ccbc9b4eae2571563e95adef99854c050349
- **Small diff (2 production files, 1 focal test, docs/evidence):** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/69b01e3d09fae953aefca031c4f58a337f259da7...e5f4ccbc9b4eae2571563e95adef99854c050349
- **Owner branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-orphan-snapshot-integrity-20261029-0140
- **Focal/restart/total tests (local Linux):** **8/8 PASS**, **20/20 PASS**, **649/649 PASS** normal, **649/649 PASS** `python -O`; `py_compile` and `git diff --check` PASS.
- **Exact SHA CI:** [Python 3.10 + 3.12](https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40796162506) **2/2 PASS**.
- **Reproduce:** checkout the SHA and run `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_orphan_full_status_snapshot_20261029.py' -q`, then `PYTHONPATH=src python3 -m unittest discover -s tests -q` and `PYTHONPATH=src python3 -O -m unittest discover -s tests -q`.
- **Evidence:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/e5f4ccbc9b4eae2571563e95adef99854c050349/docs/evidence/nav2_orphan_full_snapshot_integrity_20261029.json ; **scope, integration and rollback:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/e5f4ccbc9b4eae2571563e95adef99854c050349/docs/NAV2_ORPHAN_FULL_SNAPSHOT_INTEGRITY_20261029.md
- **Compatibility/risk:** no public API or disk schema change; stricter, so Nav2 status history expiration can **prevent automatic rearm** until safe explicit reconciliation. All new tests are **synthetic Python Linux**; ROS2 wiring is AST-checked only. Not tested in live ROS2/Gazebo/ARM SBC, not physical motors, no MCU safety; no global Nav2 ownership or DDS trust guarantee.
- **Maintainer review checklist:** [ ] confirm orphan controller is allowed to be the sole Nav2 action owner; [ ] review two small source files; [ ] replay cold ROS2/Gazebo restart with stale/ambiguous status; [ ] retain MCU-owned stop on hardware. **Integrate** selectively into the approved isolated controller, rerun ROS2 full-stack; **revert** the source commit normally if incompatible, maintaining disarmed state. No PR comment or upstream runtime edit.

---

## Previous review snapshot (2026-10-29): reject contradictory in-flight Nav2 statuses

- **Problem:** during a temporary Nav2 lifecycle-service delay, an in-flight goal was treated as healthy if the current UUID appeared active ANYWHERE in a GoalStatusArray, including packets containing another active UUID, a duplicate contradictory UUID, or unknown states. Three such original acceptance counterexamples were reproduced in pure Python.
- **Fix:** validate the *entire* status snapshot and require exactly one ACTIVE UUID equal to the fsynced durable UUID. Valid terminal history for other goals is tolerated. No new dispatch, timer, checkpoint schema or physical motor behavior is enabled.
- **Source SHA:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/69b01e3d09fae953aefca031c4f58a337f259da7
- **Small source comparison:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/694aaf3bb401a90973d4624fb711d7cb78b4973a...69b01e3d09fae953aefca031c4f58a337f259da7
- **Branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-nav2-inflight-status-20261029
- **Verified local Linux tests:** focal **8/8 PASS**, regression **641/641 PASS**, optimized Python **641/641 PASS**, diff check PASS.
- **Exact-SHA CI:** [Pure-Python regression](https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40789782794) completed/success
- **Reproduce:** check out source SHA; run `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_nav2_inflight_status_integrity_20261029.py' -q`, then `PYTHONPATH=src python3 -m unittest discover -s tests -q `and `PYTHONPATH=src python3 -O -m unittest discover -s tests -q`.
- **Evidence:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/69b01e3d09fae953aefca031c4f58a337f259da7/docs/evidence/nav2_inflight_status_integrity_20261029.json ; **review notes:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/69b01e3d09fae953aefca031c4f58a337f259da7/docs/NAV2_INFLIGHT_STATUS_INTEGRITY_20261029.md
- **Compatibility / risk:** no public API or schema change; intentionally stricter during Nav2 lifecycle fallback and may freeze an already-running simulation when ROS2 statuses are ambiguous.
- **Scope:** synthetic Linux Python and AST wiring ONLY; no real Nav2/ROS2/Gazebo validation for this patch, no DDS authentication, global Nav2 lock, physical motor/MCU/charging or low-cost ARM benchmark.
- **Integration / rollback:** review narrow source diff, selectively adopt into the canonical Nav2 owner after agreement, run real ROS2 lifecycle fault replay and motor interlocks before any physical deployment; revert imported commit normally if necessary.
- **TODO for maintainer:** agree canonical cleaning API, sole Nav2 owner, separate MCU stop, and full-stack cheap SBC benchmark. No GitHub comment added.

---

## Previous review snapshot (2026-10-28): reject replaced Nav2 journal writer lock inode

- **Safety defect reproduced:** opening and flock-locking a private `goal.json.lock` did not establish that its *pathname still named the locked inode*. A same-user lock rotation between open and acquisition could let another process lock a replacement while the original holder still issued a different Nav2 UUID. Of three deterministic new controls **two FAILED before the fix**, while stable-lock control passed.
- **Minimal fix:** after nonblocking exclusive flock, compare named lock `st_dev/st_ino` with the original descriptor and fail closed on rename, substitution or disappearance, **before** modifying the goal journal. Existing durable UUID-before-send ordering, Linux advisory lock and JSON schema unchanged.
- **Exact code:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/694aaf3bb401a90973d4624fb711d7cb78b4973a
- **Isolated comparison:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/f8e0e6413a8105f877f971c085b65daafa185ba9...694aaf3bb401a90973d4624fb711d7cb78b4973a
- **Contributor branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-goal-lock-identity-20261028
- **Verification (Linux, executed):** new focal **3/3 PASS**, prior lock **4/4 PASS**, journal **15/15 PASS**, full regression **633/633 PASS**, `python3 -O` **633/633 PASS**, `git diff --check` PASS; **2 original counterexamples reproduced as failing pre-fix controls**.
- **CI:** [exact-SHA CI](https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40782540933) status: completed; conclusion: success; jobs: unit (3.12):success, unit (3.10):success
- **Minimum reproduction:** checkout exact source SHA; `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal_lock_identity_20261028.py' -q`; then `PYTHONPATH=src python3 -m unittest discover -s tests -q` and `PYTHONPATH=src python3 -O -m unittest discover -s tests -q`.
- **Machine evidence:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/694aaf3bb401a90973d4624fb711d7cb78b4973a/docs/evidence/goal_journal_lock_identity_20261028.json ; [review and exact limits](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/694aaf3bb401a90973d4624fb711d7cb78b4973a/docs/GOAL_JOURNAL_LOCK_IDENTITY_20261028.md).
- **Risk/compatibility:** no API or journal schema changes; relies on cooperating Linux processes and private local filesystem. A malicious actor replacing the name **after** the check, distinct journal paths, network FS and full global Nav2 arbitration remain unproven/unprotected.
- **Boundary:** SYNTHETIC Linux filesystem test **only**, not ROS2/Nav2/Gazebo, SBC/CPU power, real motors, MCU braking, charger or physical cleaning.
- **Integrate / revert:** review the isolated diff; import selectively into a canonical maintainer-owned cleaning controller after agreeing exclusive Nav2 owner and single journal path, then rerun Linux tests and ROS2 end-to-end. Revert an imported patch with a normal inverse commit; never force-merge or discard a live/orphaned Nav2 UUID.
- **Review / TODO:** [ ] decide canonical Nav2 single owner and durable journal location; [ ] run real ROS2 restart/rename recovery; [ ] measure full-stack low-cost ARM hardware; [ ] MCU-owned physical stop remains mandatory. No upstream runtime edits, no new PR, no new routine comment.

---

## Previous review snapshot (2026-10-28): durable sweep campaign cross-process serialization

- **Problem (reproduced):** two SweepCampaign writers can both read revision 1 and one erase a durable cancellation with a stale RUNNING write; a symlink/hardlink checkpoint is also accepted; FIFO substitution can hang startup. Three safe pre-fix negative controls observed 3 expected FAIL; the FIFO was not executed against vulnerable code.
- **Fix:** contributor-only stable private Linux flock sidecar around revision read/compare/fsync/atomic rename; startup recovery checks the exact revision and new creation cannot overwrite an unexpectedly existing checkpoint. Bounded (16 MiB), nonblocking no-follow regular-single-link file reads reject swapped links/special inodes.
- **Exact source:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/f8e0e6413a8105f877f971c085b65daafa185ba9
- **Focused compare:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/b778d81786ab6d71684a455f00486e9fbe23d3af...f8e0e6413a8105f877f971c085b65daafa185ba9
- **Source branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-sweep-campaign-serialization-20261028
- **Tests on Linux Python:** new focal **7/7 PASS**, previous SweepCampaign **18/18 PASS**; full suite **630/630 PASS** and Python -O **630/630 PASS**; git diff --check PASS. [Exact-SHA GitHub CI](https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40775706627) **PASS** (Python 3.10 and 3.12).
- **Reproduce:** checkout new source SHA; PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_sweep_campaign_serialization_20261028.py' -q; then PYTHONPATH=src python3 -m unittest discover -s tests -q and PYTHONPATH=src python3 -O -m unittest discover -s tests -q.
- **Evidence:** [machine-readable 7+630 results and pre-fix controls](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/f8e0e6413a8105f877f971c085b65daafa185ba9/docs/evidence/sweep_campaign_serialization_20261028.json); [review/spec/compatibility](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/f8e0e6413a8105f877f971c085b65daafa185ba9/docs/SWEEP_CAMPAIGN_SERIALIZATION_20261028.md).
- **Compatibility and risk:** schema-1 JSON is unchanged. File and lock must be private single-link regular files; legacy permissive checkpoint files will fail closed until owner inspects/repairs permissions. Busy lock rejects transitions instead of silently rearming. Local Linux advisory flock does NOT provide global exclusive Nav2 goal ownership across distinct processes/paths or protect against a malicious noncooperating writer.
- **Integrate / revert:** examine isolated source/test diff, import only into the canonical owned cleaning-job controller after agreeing checkpoint ownership, rerun the suite and real ROS2/Gazebo recovery; revert imported commit normally if required. Do not delete unresolved Nav2 journals. This contribution does not touch main, upstream runtime, firmware, CAD, motor or charging control.
- **Boundary:** SYNTHETIC local Linux filesystem tests only for this patch; NOT ROS2/Nav2/Gazebo, ARM SBC, physical hardware, energy or real cleaning.
- **Review decisions/TODO:** [ ] canonical cleaning-job API; [ ] unique Nav2 action owner; [ ] real ROS2/Gazebo concurrency/crash test; [ ] eventual MCU-owned physical interlocks.

---

## Previous review snapshot (2026-10-28): map-frame provenance safety

- **Problem reproduced before patch:** 3/5 newly added tests FAILED: when an OccupancyGrid kept identical pixels but changed world origin (x/y) or yaw, the same adapted cell-index sweep plan was accepted, including a durable campaign that wrote a session. A cell index alone does **not** identify the same physical floor point after map-origin changes.
- **Fix:** SHA-256 geometry binding (frame, grid size, exact resolution, origin x/y and yaw) added to `SafeSweepPlan` and its plan digest; reject stale plans in the measured selector and durable campaign **before** choosing goals or writing a checkpoint.
- **Source commit:** https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/b778d81786ab6d71684a455f00486e9fbe23d3af
- **Focused diff:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/8123edd8b30e01fcbb46470af4d3470aa4326473...b778d81786ab6d71684a455f00486e9fbe23d3af
- **Independent source branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-plan-frame-binding-20261028
- **PASS (Linux x86-64, Python 3.14.3):** 6/6 focused post-fix tests; 623/623 full regressions normal and 623/623 under `python3 -O`; `git diff --check` PASS. Pre-fix negative controls: 3 expected FAIL, 2 PASS (not regressions on fixed SHA).
- **Exact source CI:** [GitHub Actions 40766758124](https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40766758124), Python 3.10 and 3.12 **2/2 PASS**.
- **Reproduce:** checkout source commit above; `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_sweep_plan_geometry_provenance_20261028.py' -v`; then `PYTHONPATH=src python3 -m unittest discover -s tests -q` and `PYTHONPATH=src python3 -O -m unittest discover -s tests -q`.
- **Machine evidence:** [JSON](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/b778d81786ab6d71684a455f00486e9fbe23d3af/docs/evidence/sweep_plan_frame_binding_20261028.json); [review/replay constraints](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/b778d81786ab6d71684a455f00486e9fbe23d3af/docs/SWEEP_PLAN_FRAME_PROVENANCE_20261028.md).
- **Compatibility/risk:** plan-digest format intentionally changes; old checkpoints and direct `SafeSweepPlan` construction cannot silently migrate. Re-adapt from current validated map and reconcile outstanding Nav2 actions externally before new jobs. Exact float checks may reject equivalent transforms after lossy round-trip; that is conservative.
- **Integration/revert:** evaluate isolated source patch, apply/cherry-pick into an explicitly owned module only after canonical ROS2 interface/owner review, rerun focal + full tests; revert imported commit normally if necessary. No shared upstream runtime, firmware, CAD, charging logic or motor endpoints were modified.
- **Evidence boundary:** **synthetic/headless pure Python only** for this patch; NOT a ROS2/Gazebo, ARM SBC, physical hardware, cleaning-quality, MCU or battery validation.
- **Reviewer checklist / TODO:** [ ] confirm chosen cleaning job API; [ ] establish a single Nav2 action owner; [ ] verify map geometry provenance at real ROS2 lifecycle/restarts; [ ] verify Nav2/Gazebo integration and eventual safe physical hardware under MCU interlocks.

The previous measured CPU resource optimization remains documented below; no performance improvement is claimed for this safety patch.

---

## Previous review snapshot (2026-10-28): measured resource fast path

- **Problem:** a pure-Python measured-sweep selector recalculated all traversable cells repeatedly during disk inspection. This creates avoidable full-map work, especially in a long multi-room job.
- **Fix:** calculate an immutable local traversability snapshot once per select call, reuse the same snapshot for scope, exclusions and every sampled sweep cell; preserve all coverage evidence and keepout checks.
- **Diff:** https://github.com/Valdi320/oomwoo-cleaning-jobs/compare/ac4ccd7c83691cab589ad21653886a906b6473eb...8123edd8b30e01fcbb46470af4d3470aa4326473
- **Owner source branch:** https://github.com/Valdi320/oomwoo-cleaning-jobs/tree/contrib/cleaning-jobs-selector-resource-baseline-20261028
- **Synthetic PC benchmark:** identical 30×18, three virtual rooms, 530/530 credited floor cells, 125 synthetic goals, no forbidden cells entered; two trials per revision, one x86 desktop core pinned: median wall time **9.104s before / 0.796s after (91.3% less)**. Process peak RSS **23168 KiB before / 23832 KiB after**; **NO memory saving claimed**.
- **Validation:** 3/3 additional focused tests PASS; full local Python test suite **617/617 PASS**, Python -O **617/617 PASS**; [exact-SHA CI Python 3.10 and 3.12, both PASS](https://github.com/Valdi320/oomwoo-cleaning-jobs/actions/runs/40757977174).
- **Machine evidence:** https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/8123edd8b30e01fcbb46470af4d3470aa4326473/docs/evidence/selector_one_snapshot_resource_20261028.json
- **Reproduce:** checkout the source SHA; run `PYTHONPATH=src python3 tests/bench_home_resources.py --seed 0 --one-core` and `PYTHONPATH=src python3 -m unittest discover -s tests -q`, then repeat with `python3 -O`.
- **Limits:** host-specific two-trial synthetic Python measurement, **not full ROS2, Nav2, Gazebo, low-cost ARM SBC or physical robot**. No validated power, thermal, minimum RAM, battery, motors, MCU watchdog or cleaning quality; cheap hardware selection must wait for a full-stack representative workload. Runtime behavior is not integrated into official OOMWOO.
- **Integration / rollback:** reviewer may import the isolated contributor source change after inspecting the small diff and verifying tests; normal `git revert` can back it out. No forced merge, upstream Nav2 runtime or third-party files changed.
- **Outstanding maintainer decisions:** canonical cleaning-job API and exclusive Nav2 action owner; measurement of full-stack CPU, RAM, latency and power on affordable hardware.

Historical milestones below are snapshots, not the current benchmark or CI result.

---

## Major verified deliverables

- The UNMODIFIED OOMWOO boustrophedon waypoint generator is reused
  without starting its own navigation executor: 409 original targets,
  391 conservatively admitted distinct waypoints, five original
  unsafe/unresolved targets explicitly flagged (not falsely completed).
- A bounded REAL OOMWOO Gazebo + Nav2 action trial issued exactly six
  waypoints, all succeeded, with 348 NEW cells from the ORIGINAL
  simulated ground-truth coverage meter; 246 cells were durably
  retained across a SYNTHETIC battery/bin/mop station service pause.
  This does NOT prove an entire room was cleaned.
- Persistent multi-step sweep campaigns, selected-room/whole-home
  orchestration, gap analysis, fixed-size retry bounds, checkpoint
  restart handling and write-ahead coverage journals were developed
  and tested with deliberate interrupted-write fault injection.
- 12/12 separate deterministic SYNTHETIC three-room home scenarios
  completed all eligible cells with no forbidden-zone entries, while
  exercising simulated dock cycles and a controller restart.
  These are NOT 12 Nav2/Gazebo full-home simulations.
- An experimental typed ROSIDL StartCleaning / ControlCleaning /
  GetCleaningStatus package builds offline and passes actual ROS2
  middleware calls against a FAKE Nav2 action server, including
  duplicate-request and cancel/result gating.
- REAL Nav2 Jazzy/Gazebo KeepoutFilter originally passed two static
  prohibited-zone robot movements. This follow-up additionally
  demonstrates LIVE static-zone replacement using LoadMap:
  old 220-cell zone replaced with new 198-cell zone, confirmed by
  two distinct global/local post-filter updates and time-fenced
  TF2 map<-odom conversion; NO motor/navigation command during edit.
- Subsequent actual ComputePathToPose accepted a 32-pose route to
  the former forbidden zone but stopped 0.40 m short of the new
  forbidden destination on a truncated 15-pose path. The application
  must reject forbidden goals before dispatch, regardless of Nav2's
  success code.
- 383/383 pure-Python regressions PASS, including Python -O and
  Python 3.12, plus separate real ROS2 protocol and Nav2/Gazebo tests.

## Reproducible reports

- [Ten milestones, reality labels and limits](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/main/docs/TEN_SUBSTANTIAL_MILESTONES_20261027.md)
- [Verified live KeepoutFilter edit](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/main/docs/REAL_NAV2_DYNAMIC_KEEPOUT_20261027.md)
- [Six real Nav2 goals / synthetic station](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/main/docs/evidence/upstream_sweep_6goals_hardened_20261027.json)
- [12 synthetic homes](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/main/docs/evidence/synthetic_multihome_12cases_20261027.json)

## Critical remaining work / maintainer decisions

These are independent test modules; a production full-room or
multi-room queue is NOT integrated into the persistent OOMWOO ROS2
node. The typed services are an isolated PRIVATE prototype, not a
canonical approved OOMWOO interface. The verified KeepoutFilter
overlay is NOT in OOMWOO's stock bringup. Real dock/recharge hardware,
MCU safety, physical dirt removal, actual sensor-gated cleaning and
multi-room long-duration Nav2/Gazebo validation remain outstanding.

Please advise (1) the canonical job API and (2) the exclusive Nav2
action owner so this can be integrated without two autonomous planners.

ChatGPT-assisted, reproducibly tested on Linux in an isolated offline
OOMWOO Jazzy/Gazebo environment. Upstream changes ONLY this owned
contribution pointer; shared runtime and other contributors stay unchanged.


### Latest follow-up: measured virtual-room completion in REAL Gazebo

Independent source commit: https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/1cd6b5eb5753a832113e6a3c416e6fc084ecde80

The opt-in persistent ROS2 controller now schedules the **original OOMWOO
sweep-plan geometry** under one Nav2 owner, saves the per-room plan and
verifies coverage from the unmodified official Gazebo ground-truth meter.
For one **operator-defined digital region** in an OPEN test_room (NOT a
physically separated apartment room), actual checkpoints show
**171/171 eligible floor cells externally measured**, BOTH durable
CleaningJobs and SweepCampaign states COMPLETED, 11 Nav2 requests in
the log and the final UUID terminal CANCELED.

Added guarded gap-route proposals (Nav2 endpoint tolerance can leave
a narrow dirty fringe) and a software-only dock geometry / exclusive
cmd_vel owner handoff contract. 425/425 pure Python regressions PASS.
This is NOT proof that the real robot can safely hand motors between
Nav2 and dock-drive, charge, empty its dustbin, wash mop or remove dirt.

Report: https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/main/docs/REAL_ROOM_AND_DOCK_BOUNDARY_20261027.md

Physical station service, real motor mux/Nav2 dock arbitration, MCU
safety and whole multi-room Gazebo endurance remain for integration.

## Follow-up 2026-10-27: TWO COMPLETE digital zones in REAL OOMWOO Gazebo

**Published implementation and tests:**
https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/471a483f51fed1ba727c5befa49b39e54ac93fc3

The same persistent simulation-only cleaning controller completed two
different operator-defined, disjoint virtual regions on the official
OOMWOO test_room map in ONE selected-room job. These regions are on the
SAME OPEN FLOOR, not separated by architectural walls.

- Area A: **80/80** externally measured cells.
- Area B: **40/40** externally measured cells.
- Total: **120/120** independently measured eligible cells, 100%
  of exactly the selected virtual map scope.
- REAL OOMWOO Nav2 Jazzy: **18 accepted goals**; original OOMWOO
  ground-truth meter: **200 observed frames**; **12** bounded
  gap-repair routes proposed (no fabricated cleaning credit).
- Both durable CleaningJobs and SweepCampaign checkpoints reached
  COMPLETED with identical 120-cell credited sets. CoverageWAL had
  20 applied events and zero pending. The final UUID was terminal
  CANCELED after external completion; no active goal remained.
- Earlier 117/120 partial/timeout runs are preserved separately and
  were not misreported as successful.

An experimental isolated ROS2 dock velocity relay now uses the fsync'd
CURRENT Nav2 goal UUID, observed ACTIVE during this lease and then
matching terminal before transferring from Nav2 to dock_drive. Old
terminal statuses cannot authorize a new run. Foreign Nav2 action,
battery-low, empty/stale status and expired owner commands force STOP.
Actual ROS2 middleware/typed control test PASS, with SYNTHETIC
status/odometry/battery and NO MOTOR CONNECTIONS. The separate
UNMODIFIED upstream OOMWOO 2D docking harness also passed 7/7
simulated geometric approaches, not physical charging.

**437/437** headless regressions PASS on host, optimized Python and
Python 3.12; existing ROS2 probe matrix remains green.

- [Actual two-zone Gazebo evidence](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/main/docs/evidence/real_two_virtual_zones_complete_20261027.json)
- [ROS2 exact-UUID dock relay evidence](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/main/docs/evidence/ros2_exact_uuid_dock_relay_20261027.json)
- [Integration and production limits](https://github.com/Valdi320/oomwoo-cleaning-jobs/blob/main/docs/REAL_TWO_ZONES_AND_DOCK_RELAY_20261027.md)

**Production remainers:** physically divided furnished-room navigation,
real exclusive robot cmd_vel remaps and MCU safety; charge current,
auto-empty and mop-wash provider protocols plus hardware sensors; actual
vacuum/mop actuator-state coverage, full-home endurance, and canonical
OOMWOO ROS2 API/maintainer approval. Lab relay publishes to a SAFE,
NON-MOTOR topic only. This is not 100% of a hardware cleaning robot.


### Additional nonhardware physical coverage-admission contract

Source: https://github.com/Valdi320/oomwoo-cleaning-jobs/commit/0bf06d549374df5c55eb6c76a453f8b28adf9b6c

PhysicalCoverageAdmission is a fail-closed, pure-Python policy that
rejects credit for floor cells unless fresh and map-matching
localization, an accepted active navigation goal, independently
verified vacuum current and/or mop actuator contact, and a trusted
coverage source all agree. It has 22 additional negative/positive
tests; the entire regression suite passes 459/459 tests in normal,
optimized and Python 3.12-container modes.

**No real vacuum-current sensor, mop-switch, source authentication or
physical actuator has been connected**. This does NOT certify that
the hardware actually cleans. Production must supply independent
signed/trusted sensor metadata and MCU-owned safety interlocks.
