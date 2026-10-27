"""Single-writer job state machine. It never publishes motor or navigation commands.

The adapter owns (1) issuing a path to a navigation backend, (2) confirming
coverage from real coverage feedback, (3) keeping Nav2 keepout filters enabled,
and (4) sending MCU-approved safety stops when required. None are emulated here.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any

from .grid import Grid, MapError, UnreachableArea

STATES = {'running', 'paused_user', 'paused_service', 'paused_recovery',
          'completed', 'cancelled', 'error'}
TERMINAL = {'completed', 'cancelled', 'error'}
SCHEMA_VERSION = 1
JOB_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z')


class JobError(RuntimeError):
    """Invalid job transition / request. No state change unless documented."""


@dataclass(frozen=True)
class Resources:
    battery_fraction: float
    bin_full: bool = False
    mop_needs_wash: bool = False
    battery_minimum: float = 0.20

    def needed(self) -> str | None:
        import math
        if type(self.battery_fraction) not in (int,float) or not math.isfinite(self.battery_fraction) or not 0 <= self.battery_fraction <= 1:
            raise JobError('battery_fraction must be in [0,1]')
        if type(self.battery_minimum) not in (int,float) or not math.isfinite(self.battery_minimum) or not 0 < self.battery_minimum < 1:
            raise JobError('battery_minimum must be between 0 and 1')
        if type(self.bin_full) is not bool or type(self.mop_needs_wash) is not bool:
            raise JobError('resource flags must be bool')
        if self.battery_fraction <= self.battery_minimum: return 'LOW_BATTERY'
        if self.bin_full: return 'BIN_FULL'
        if self.mop_needs_wash: return 'MOP_SERVICE'
        return None


class Checkpoint:
    """Atomic durable JSON writes; deliberately single-writer (ROS node owns it).

    A crash after os.replace does not acknowledge missing coverage. A restore of
    a previously running job requires explicit manual/system resume + fresh pose.
    """
    def __init__(self, path: Path):
        self.path = Path(path)

    def read(self) -> dict[str, Any] | None:
        if not self.path.exists(): return None
        try:
            raw = json.loads(self.path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as ex:
            raise JobError('checkpoint missing/corrupted: refuse to arm') from ex
        if not isinstance(raw, dict):
            raise JobError('checkpoint not an object')
        return raw

    def write(self, obj: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix='.checkpoint-', suffix='.tmp',dir=self.path.parent)
        try:
            with os.fdopen(fd,'w',encoding='utf-8') as stream:
                json.dump(obj, stream, ensure_ascii=False, sort_keys=True, separators=(',',':'))
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name,self.path)
            if hasattr(os,'O_DIRECTORY'):
                dfd=os.open(self.path.parent,os.O_DIRECTORY)
                try: os.fsync(dfd)
                finally: os.close(dfd)
        finally:
            if os.path.exists(name): os.unlink(name)


class CleaningJobs:
    def __init__(self, grid: Grid, checkpoint: Path):
        self.grid = grid
        self.store = Checkpoint(Path(checkpoint))
        state = self.store.read()
        if state is not None:
            if state.get('map_digest') != self.grid.fingerprint():
                raise JobError('map/keepout/room definitions changed or checkpoint missing digest: resume forbidden')
            self._validate(state)
            if state['state'] == 'running':
                # No auto-motion after reboot; fail closed until user/adapter checks fresh pose.
                state = dict(state, state='paused_recovery', reason='RESTART_NEEDS_REVALIDATION',
                             revision=state['revision'] + 1)
                self.store.write(state)
        self.state: dict[str, Any] | None = state

    def _validate(self, state: dict[str, Any]) -> None:
        """Explicit guards (never `assert`, even when Python runs with -O)."""
        try:
            target = state['targets']
            done = state['cleaned']
            revision = state['revision']
            resume_count = state['resume_count']
            job_id = state['job_id']
            mode = state['mode']
            reason = state['reason']
            stage = state['state']
            digest = state['map_digest']
            spec_rooms = state['rooms']
            spot = state['spot']
            radius_m = state['radius_m']
            valid = (
                state['schema'] == SCHEMA_VERSION
                and isinstance(job_id, str) and JOB_ID.fullmatch(job_id) is not None
                and isinstance(stage, str) and stage in STATES
                and type(revision) is int and revision >= 1
                and type(resume_count) is int and resume_count >= 0
                and isinstance(digest, str) and len(digest) == 64
                and digest == self.grid.fingerprint()
                and type(target) is list and type(done) is list and bool(target)
                and all(type(i) is int for i in target + done)
                and target == sorted(set(target)) and done == sorted(set(done))
                and set(done) <= set(target) and set(target) <= self.grid.traversable
                and isinstance(mode, str) and mode in ('whole', 'rooms', 'spot')
                and type(spec_rooms) is list and all(isinstance(x,str) for x in spec_rooms)
                and (spot is None or type(spot) is int)
                and type(radius_m) in (int,float)
                and (reason is None or isinstance(reason, str))
            )
            if not valid:
                raise JobError('invalid checkpoint schema: refuse to arm')
            recomputed = self.grid.targets(mode=mode, rooms=tuple(spec_rooms),
                                           spot=spot, radius_m=radius_m)
            if frozenset(target) != recomputed:
                raise JobError('checkpoint scope differs from map: refuse to arm')
            if stage == 'completed' and done != target:
                raise JobError('invalid completed state')
            if stage != 'completed' and done == target:
                raise JobError('fully covered job must be completed')
            if stage == 'running' and reason is not None:
                raise JobError('running job has fault reason')
            if stage.startswith('paused_') and not reason:
                raise JobError('paused job missing reason')
        except (KeyError, TypeError, ValueError, MapError, OverflowError) as ex:
            raise JobError('invalid checkpoint schema: refuse to arm') from ex

    def _save(self, data: dict[str, Any]) -> None:
        self._validate(data)
        if self.state is not None:
            # No optimistic updates across two instances: single ROS writer by contract.
            disk = self.store.read()
            if disk is None or disk['revision'] != self.state['revision']:
                raise JobError('checkpoint modified by another writer; refuse to mutate')
        self.store.write(data)
        self.state = data

    def _require(self) -> dict[str, Any]:
        if self.state is None: raise JobError('no job')
        return self.state

    def start(self, job_id: str, *, mode: str, start_cell: int,
              rooms: tuple[str, ...] = (), spot: int | None = None,
              radius_m: float = 0, resources: Resources | None = None) -> dict[str, Any]:
        if not isinstance(job_id,str) or not JOB_ID.fullmatch(job_id):
            raise JobError('invalid job_id')
        if self.state is not None:
            raise JobError('checkpoint already contains a job; use a new checkpoint for a new job')
        targets = self.grid.targets(mode=mode, rooms=rooms, spot=spot, radius_m=radius_m)
        reachable = self.grid.reachable(start_cell)
        if not targets <= reachable:
            raise UnreachableArea(f'{len(targets-reachable)} target cells unreachable; job not started')
        need = resources.needed() if resources is not None else None
        s = {'schema':SCHEMA_VERSION,'job_id':job_id,'map_digest':self.grid.fingerprint(),
             'mode':mode,'rooms':list(rooms),'spot':spot,'radius_m':radius_m,
             'targets':sorted(targets),'cleaned':[],
             'state':'paused_service' if need else 'running',
             'reason':need,'revision':1,'resume_count':0}
        self._save(s)
        return self.status()

    def status(self) -> dict[str, Any]:
        s=self._require()
        return {'job_id':s['job_id'],'state':s['state'],'reason_code':s['reason'],
                'source':'oomwoo_cleaning_jobs','recoverable':s['state'].startswith('paused_'),
                'target_cells':len(s['targets']),'cleaned_cells':len(s['cleaned']),
                'coverage_fraction':len(s['cleaned'])/len(s['targets']),
                'revision':s['revision'],'resume_count':s['resume_count']}

    def waypoint(self, pose_cell: int) -> tuple[int, ...]:
        s=self._require()
        if s['state'] != 'running': raise JobError('job is not running: no waypoint')
        remaining=frozenset(set(s['targets']) - set(s['cleaned']))
        if not remaining: raise JobError('no work remaining')
        return self.grid.path_to_nearest(pose_cell,remaining)

    def acknowledge_cleaned(self, cells: tuple[int,...]) -> dict[str, Any]:
        s=self._require()
        if s['state'] != 'running': raise JobError('job is not running')
        if not isinstance(cells,tuple) or not cells or any(type(i) is not int for i in cells):
            raise JobError('acknowledgement must be nonempty tuple of grid indices')
        if not set(cells) <= set(s['targets']):
            raise JobError('acknowledged coverage outside target/inside keepout')
        cleaned=sorted(set(s['cleaned']) | set(cells))
        # Must be persisted before acknowledging feedback to caller.
        new=dict(s,cleaned=cleaned, state='completed' if len(cleaned)==len(s['targets']) else 'running',
                 reason=None,revision=s['revision']+1)
        self._save(new)
        return self.status()

    def observe_resources(self, resource: Resources) -> dict[str, Any]:
        s=self._require()
        need=resource.needed()
        if s['state']=='running' and need:
            self._save(dict(s,state='paused_service',reason=need,revision=s['revision']+1))
        return self.status()

    def pause(self) -> dict[str, Any]:
        s=self._require()
        if s['state'] == 'paused_user': return self.status()
        if s['state'] != 'running': raise JobError('pause only valid for running job')
        self._save(dict(s,state='paused_user',reason='USER_PAUSE',revision=s['revision']+1))
        return self.status()

    def resume(self, *, pose_cell: int, resources: Resources) -> dict[str, Any]:
        s=self._require()
        if not s['state'].startswith('paused_'):
            raise JobError('resume requires paused job')
        need=resources.needed()
        if need: raise JobError('cannot resume: '+need)
        remaining=frozenset(set(s['targets'])-set(s['cleaned']))
        # Fresh pose and a still-reachable remainder are mandatory before motion.
        accessible = self.grid.reachable(pose_cell)
        if not remaining <= accessible:
            raise UnreachableArea('remaining coverage unreachable from fresh pose')
        self._save(dict(s,state='running',reason=None, revision=s['revision']+1,
                        resume_count=s['resume_count']+1))
        return self.status()

    def suspend_recovery(self, reason: str) -> dict[str, Any]:
        """Fail-closed navigation/localization pause requiring deliberate resume.

        Hardware safety remains MCU-owned. This only gates future job commands.
        """
        if not isinstance(reason, str) or not re.fullmatch(r'[A-Z][A-Z0-9_]{2,63}', reason):
            raise JobError('reason must be stable machine-readable code')
        s = self._require()
        if s['state'] == 'paused_recovery':
            return self.status()  # Duplicate fault does not overwrite first cause.
        if s['state'] != 'running':
            raise JobError('recovery pause only valid for running job')
        self._save(dict(s, state='paused_recovery', reason=reason,
                        revision=s['revision'] + 1))
        return self.status()

    def cancel(self) -> dict[str, Any]:
        s=self._require()
        if s['state'] == 'cancelled': return self.status()
        if s['state'] in TERMINAL: raise JobError('cannot cancel terminal job')
        self._save(dict(s,state='cancelled',reason='USER_CANCELLED',revision=s['revision']+1))
        return self.status()

    def fail(self, reason: str) -> dict[str, Any]:
        s=self._require()
        if s['state'] in TERMINAL: raise JobError('cannot fail terminal job')
        if not isinstance(reason,str) or not re.fullmatch(r'[A-Z][A-Z0-9_]{2,63}',reason):
            raise JobError('reason must be stable machine-readable code')
        self._save(dict(s,state='error',reason=reason,revision=s['revision']+1))
        return self.status()
