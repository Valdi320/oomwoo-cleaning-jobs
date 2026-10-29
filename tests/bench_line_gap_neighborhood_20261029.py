"""Synthetic Python line-through residual-gap CPU benchmark, no ROS2 or hardware."""
from __future__ import annotations
import hashlib, json, os, platform, resource, statistics, time
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.line_gap_route import select_line_through_route

def main() -> None:
    if hasattr(os, "sched_getaffinity"):
        cpus = os.sched_getaffinity(0)
        os.sched_setaffinity(0, {min(cpus)})
        affinity = sorted(os.sched_getaffinity(0))
    else:
        affinity = None
    width, height = 120, 90
    grid = Grid(width, height, (0,) * (width * height),
                resolution_m=0.05, clearance_cells=3)
    start = 70 * width + 80
    missing = frozenset(row * width + col
                        for row in range(30, 60)
                        for col in range(45, 66))
    assert start in grid.traversable and missing <= grid.traversable
    runs = []
    for _ in range(3):
        admissions = [0]
        def admit(route):
            admissions[0] += 1
            return True
        t0, cpu0 = time.perf_counter(), time.process_time()
        route = select_line_through_route(grid, start, missing, admit=admit)
        elapsed, cpu = time.perf_counter() - t0, time.process_time() - cpu0
        assert route[0] == start and set(route) <= grid.traversable
        runs.append({"wall_s": round(elapsed, 6),
                     "cpu_s": round(cpu, 6), "route_len": len(route),
                     "admissions": admissions[0],
                     "route_sha256": hashlib.sha256(
                         ",".join(map(str, route)).encode()).hexdigest()})
    assert len({(r["route_len"], r["admissions"], r["route_sha256"])
                for r in runs}) == 1, "benchmark route nondeterministic"
    print(json.dumps({
        "scope": "single Python x86 synthetic 120x90 grid; 630 missing cells; NOT ROS2, ARM, power or hardware",
        "python": platform.python_version(), "machine": platform.machine(),
        "affinity": affinity, "runs": runs,
        "median_wall_s": round(statistics.median(r["wall_s"] for r in runs), 6),
        "median_cpu_s": round(statistics.median(r["cpu_s"] for r in runs), 6),
        "max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    }, sort_keys=True))

if __name__ == "__main__":
    main()
