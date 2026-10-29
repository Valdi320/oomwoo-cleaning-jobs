"""Offline repeated-output microbenchmark; NOT an OOMWOO hardware benchmark.

Run: PYTHONPATH=src python3 benchmarks/bench_reactive_keepout_20261029.py
No ROS2, Gazebo, motors or energy measurement. One 400x400 grid, identical
evidence snapshots, repeat calls at a fixed logical timestamp.
"""
from dataclasses import replace
import json
import platform
import statistics
import time

from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.dock_handoff import Velocity
from oomwoo_cleaning_jobs.reactive_velocity import MotionHealth
from oomwoo_cleaning_jobs.sequenced_motion_ingress import EvidenceEnvelope, SequencedMotionMux
from oomwoo_cleaning_jobs.reactive_keepout_envelope import (
    KeepoutEvidence, PoseEvidence, ReactiveKeepoutEnvelope)


def main():
    r = Raster(400, 400, .05, -10., -10., 0., (0,) * (400 * 400))
    maps = KeepoutEvidence(r, r, r, 1.90, 1.90, 1.90)
    pose = PoseEvidence(0., 0., 0., 1.92)
    m = SequencedMotionMux(odom_source="odom", health_source="health")
    for i in range(8):
        t = 1. + i*.12
        m.observe_odom(now=t, envelope=EvidenceEnvelope("odom", i, t-.01),
                       linear_mps=0., angular_rps=0.)
    health = MotionHealth(True, True, True, True, True, True, None, True, False)
    m.observe_health(now=1.85, envelope=EvidenceEnvelope("health", 0, 1.84),
                     proof=health)
    token = m.claim_sweep(now=1.90, nav_released=True, prior_terminal_uuid=None)
    m.observe_odom(now=1.92, envelope=EvidenceEnvelope("odom", 8, 1.91),
                   linear_mps=0., angular_rps=0.)
    m.observe_health(now=1.93, envelope=EvidenceEnvelope("health", 1, 1.92),
                     proof=health)
    m.accept_twist(source="reactive_sweep", token=token,
                   now=1.94, velocity=Velocity(.20, .2))
    g = ReactiveKeepoutEnvelope(m)
    def measure(*, disable_cache):
        timings = []
        for block in range(3):
            before = time.perf_counter()
            for k in range(100):
                if disable_cache:
                    # Equivalent to the original full binary mask scan per output.
                    g._validated_mask = None
                result = g.output(now=1.95, pose=pose, maps=maps)
                if result != Velocity(.20, .2):
                    raise RuntimeError("benchmark is not correctly authorized")
            timings.append(round((time.perf_counter()-before)*10, 5))
        return {"per_call_ms":timings,
                "median_ms":round(statistics.median(timings), 5)}
    baseline = measure(disable_cache=True)
    cached = measure(disable_cache=False)
    print(json.dumps({
        "machine":platform.machine(),"python":platform.python_version(),
        "raster_cells":len(r.data),"frames":"map","samples_per_block":100,
        "identical_timestamp_note":"fixed logical time, repeated steady-state call",
        "full_mask_scan_each_tick":baseline,
        "cached_immutable_mask":cached,
        "speedup":round(baseline["median_ms"]/cached["median_ms"],2),
        "scope":"pure-Python x86 synthetic output microbench; not ROS2, SBC or motor",
    },sort_keys=True))


if __name__ == "__main__":
    main()
