# Durable sweep checkpoint: serialize writers and reject unsafe filesystem nodes (2026-10-28)

**Contributor:** Valdi320. **Source:** independent Apache-2.0 cleaning-jobs repository. **Upstream:** OOMWOO PR #79 links independent code; no upstream runtime file is edited here.

## Defect, reproduced before the fix

SweepCampaign._write() previously compared disk revisions and then replaced the JSON file **without an inter-process lock**. Two writers could both observe revision 1, one permanently cancel the campaign and the other overwrite that cancellation with a running revision 2. Startup recovery also needed explicit revision compare.

Path.read_text() followed symlinks, accepted hard-linked JSON and could block on a substituted FIFO. The checkpoint decides whether a cleaning campaign may resume, so it must fail closed.

**Before fix:** three deterministic negative controls demonstrated the defects (all three FAIL as expected): lost cancel under forced writer interleaving, symlink admission and hard-link admission. The old FIFO read was not executed because it could hang.

## Solution

- Stable private campaign.json.lock inode with nonblocking Linux flock, protecting the **entire read/revision-check/atomic-replace** transaction. Contention rejects a write immediately; the caller must reconcile before further Nav2 actions.
- Constructor creation refuses to clobber unexpected checkpoints. Startup recovery now requires the exact on-disk revision it read.
- Checkpoint admission uses O_NOFOLLOW | O_NONBLOCK, regular single-link inode checks, private file mode and bounded 16 MiB read. Rejects symlink, hardlink, FIFO and corrupt/oversized checkpoints.
- Preserve schema 1 and existing fsync/atomic-replace durability. Sidecar lock is mode 0600; do not remove while another process may be active.

## Reproduce on Linux

    export PYTHONPATH="$PWD/src"
    python3 -m unittest discover -s tests -p 'test_sweep_campaign_serialization_20261028.py' -q
    python3 -m unittest discover -s tests -p 'test_sweep_campaign.py' -q
    python3 -m unittest discover -s tests -q
    python3 -O -m unittest discover -s tests -q

**Observed:** focal 7/7 PASS; existing campaign 18/18 PASS; whole suite 630/630 PASS; Python -O 630/630 PASS on local Linux. Evidence: docs/evidence/sweep_campaign_serialization_20261028.json. CI should be checked against the exact published SHA.

## Compatibility and boundaries

Existing schema-1 checkpoints are readable if regular, private, single-link files. Files with group/world-readable bits must be inspected by their owner and secured manually (chmod 600) before use, not silently rewritten. A lock-busy exception must fail the command closed.

The lock protects **cooperating Linux processes sharing one local filesystem path**. It does not enforce global single-owner Nav2 arbitration across different paths, protect against malicious noncooperating code, validate network filesystem locking, authorize real motors/charging or prove floor cleaning. Tests here are **synthetic Python/Linux filesystem only**; no ROS2/Nav2/Gazebo or hardware run for this change.

**Integration:** review the source and tests, selectively import into the canonical cleaning-job component after deciding checkpoint ownership; re-run regression and ROS2 simulation. **Rollback:** revert imported commit or deploy known-good prior implementation; never discard an unresolved Nav2 UUID to pretend idle. Public API and JSON schema are unchanged.

**Open decisions:** canonical cleaning API, exclusive global Nav2 owner, physical coverage evidence and MCU safety chain.
