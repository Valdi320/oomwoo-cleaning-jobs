"""Optional CPython microbenchmark, NOT a ROS2 timing or hardware claim.

From repository root: PYTHONPATH=src python tests/bench_swept_fastpath.py
"""
import json
import platform
import statistics
import time
from types import SimpleNamespace

from test_swept_fastpath import reference_clear
from oomwoo_cleaning_jobs.swept_footprint import swept_disk_clear

W = H = 240
RES = .01
A, B = (-.35, -.35), (.35, .35)
RADIUS = .21
MAX_COST = 25


def sample_grid(off_path_hazards):
    pixels = [0] * (W * H)
    if off_path_hazards:
        for x in range(143, 183):
            pixels[71 * W + x] = 100  # y=-0.49m, inside bbox; outside capsule
    return SimpleNamespace(width=W, height=H, resolution=RES,
                           origin_x=-1.2, origin_y=-1.2, yaw=0.,
                           data=tuple(pixels))


def median_ms(fn, grid, repeats=30):
    assert fn(grid, A, B, RADIUS, MAX_COST)  # fail if behaviour differs
    results = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        result = fn(grid, A, B, RADIUS, MAX_COST)
        elapsed = time.perf_counter_ns() - start
        assert result
        results.append(elapsed / 1e6)
    return statistics.median(results)


def run():
    rows = {}
    for name, has_hazards in (("all_safe", False), ("off_path_hazards", True)):
        grid = sample_grid(has_hazards)
        median_ms(reference_clear, grid, 2)
        median_ms(swept_disk_clear, grid, 2)
        old_ms = median_ms(reference_clear, grid)
        new_ms = median_ms(swept_disk_clear, grid)
        rows[name] = {"reference_median_ms": round(old_ms, 4),
                      "optimized_median_ms": round(new_ms, 4),
                      "speedup_x": round(old_ms / new_ms, 2), "repeats": 30}
    print(json.dumps({"python": platform.python_version(),
                      "platform": platform.platform(),
                      "map": "240x240 cells at 1cm; 0.21m disk; 0.99m diagonal",
                      "type": "synthetic CPython host benchmark only",
                      "results": rows}, indent=2, sort_keys=True))


if __name__ == "__main__":
    run()
