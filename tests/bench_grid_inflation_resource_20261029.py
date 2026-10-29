"""Two reproducible synthetic maps, original vs bounded exact grid clearance."""
import hashlib
import json
import platform
import resource
import statistics
import sys
import time
from oomwoo_cleaning_jobs.grid import Grid
from test_grid_inflation_resource_20261029 import reference_blocked


def measure(label, w, h, r, occupied):
    grid = Grid(w, h, tuple(100 if (x, y) in occupied else 0
                             for y in range(h) for x in range(w)),
                keepout=frozenset((1200, 3400, 10771)), clearance_cells=r)
    fn = {'reference_original': reference_blocked,
          'bounded_hybrid': lambda g: g.blocked}
    times = {name: {'wall_s': [], 'cpu_s': []} for name in fn}
    digests = {}
    for _ in range(3):
        for name, call in fn.items():
            start_w, start_c = time.perf_counter(), time.process_time()
            result = call(grid)
            times[name]['cpu_s'].append(time.process_time()-start_c)
            times[name]['wall_s'].append(time.perf_counter()-start_w)
            digests[name] = {
                'blocked_count':len(result),
                'blocked_sha256':hashlib.sha256(
                    ','.join(map(str, sorted(result))).encode('utf-8')
                ).hexdigest(),
            }
    assert digests['reference_original'] == digests['bounded_hybrid']
    for rec in times.values():
        rec['median_cpu_s'] = statistics.median(rec['cpu_s'])
        rec['median_wall_s'] = statistics.median(rec['wall_s'])
    return {'label':label,'map':{'w':w,'h':h,'radius_cells':r,
            'obstacle_cells':len(occupied)},
            'identical_membership':True, 'digest':digests['reference_original'],
            'times':times}


def run():
    w,h=256,192
    # Synthetic room: closed exterior wall plus four furniture blocks.
    furniture = lambda x,y: (
        x==0 or y==0 or x==w-1 or y==h-1
        or (30<=x<80 and 42<=y<54)
        or (110<=x<130 and 75<=y<145)
        or (177<=x<204 and 18<=y<48)
        or (64<=x<70 and 137<=y<169))
    occupied_room={(x,y) for y in range(h) for x in range(w)
                   if furniture(x,y)}
    dense = {(x,y) for y in range(h) for x in range(w)
             if (x*37+y*17)%11<3}
    cases=[
        measure('synthetic_room_furniture',w,h,7,occupied_room),
        measure('adversarial_dense_unknown_cost',w,h,10,dense)]
    assert cases[0]['digest']['blocked_count'] < w*h, 'room should include free space'
    return {
        'platform':platform.platform(), 'arch':platform.machine(),
        'python':sys.version.split()[0], 'cases':cases,
        'process_peak_rss_kib_not_comparable':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        'scope':'pure Python x86 per-Grid inflation; not full ROS2/Nav2 or hardware'
    }


if __name__=='__main__':
    print(json.dumps(run(),sort_keys=True,indent=2))
