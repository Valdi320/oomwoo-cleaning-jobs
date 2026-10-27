"""Durable station-service transaction contract for the cleaning-jobs RFC.

This is a simulation-first *orchestration* component. It sends NO commands
to a physical dock or motor. The upstream dock-cycle adapter must implement
real travel, charging, self-empty and wash jobs with independently observed
telemetry, then call these explicit transition gates.

Goal: no accidental resumption if the robot reached a dock but service
did not happen, if a station acknowledgment is stale, if sensor state
is missing or if a process restarted during a transaction.
"""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import secrets
import tempfile


class DockCycleError(RuntimeError):
    """Dock or service operation not safely proven."""


class DockCycle:
    STATES=frozenset(("idle","awaiting_dock","at_dock","servicing",
                      "ready_to_resume","paused_recovery","fault"))

    def __init__(self, path:Path,*,job_id:str,map_digest:str,
                 max_cycles:int=16):
        if (not isinstance(job_id,str) or not job_id or len(job_id)>64
            or not isinstance(map_digest,str) or len(map_digest)!=64
            or type(max_cycles) is not int or not 1<=max_cycles<=500):
            raise DockCycleError("invalid job/dock session")
        self.path=Path(path)
        self.job_id=job_id
        self.digest=map_digest
        self.max_cycles=max_cycles
        self._state=None
        if self.path.exists():
            item=self._read()
            if item["state"] not in ("idle","fault"):
                item={**item,"state":"paused_recovery","reason":"SERVICE_REVALIDATE",
                      "revision":item["revision"]+1}
                self._write(item)
            self._state=item
        else:
            item=dict(schema=1,job_id=job_id,map_digest=map_digest,
                      state="idle",reason=None,revision=1,cycles=0,
                      nonce=None,needed=[],completed=[],
                      station_ack=False,stable_samples=0,
                      last_observation_seq=0)
            self._write(item)
            self._state=item

    def _validate(self,d):
        keys={"schema","job_id","map_digest","state","reason","revision",
              "cycles","nonce","needed","completed","station_ack","stable_samples","last_observation_seq"}
        if (not isinstance(d,dict) or set(d)!=keys
            or type(d["schema"]) is not int or d["schema"]!=1
            or d["job_id"]!=self.job_id or d["map_digest"]!=self.digest
            or not isinstance(d["state"],str) or d["state"] not in self.STATES
            or type(d["revision"]) is not int or d["revision"]<1
            or type(d["cycles"]) is not int or not 0<=d["cycles"]<=self.max_cycles
            or type(d["station_ack"]) is not bool
            or type(d["stable_samples"]) is not int or not 0<=d["stable_samples"]<=100
            or type(d.get("last_observation_seq")) is not int
            or not 0<=d["last_observation_seq"]<=2**63-1
            or type(d["needed"]) is not list or type(d["completed"]) is not list
            or any(not isinstance(i,str) for i in d["needed"]+d["completed"])
            or d["needed"]!=sorted(set(d["needed"]))
            or d["completed"]!=sorted(set(d["completed"]))
            or not set(d["needed"])<=set(("recharge","empty_bin","wash_mop"))
            or not set(d["completed"])<=set(d["needed"])
            or (d["nonce"] is not None and
                (not isinstance(d["nonce"],str) or len(d["nonce"])!=32
                 or any(x not in "0123456789abcdef" for x in d["nonce"])))):
            raise DockCycleError("corrupted station transaction: refuse to resume")
        if (d["state"]=="ready_to_resume"
            and (d["stable_samples"]<2 or
                 set(d["needed"])!=set(d["completed"]) or
                 d["station_ack"] is not True)):
            raise DockCycleError("unproven ready-to-resume station state")
        if (d["state"]=="idle"
            and (d["nonce"] is not None or d["needed"] or d["completed"]
                 or d["station_ack"] or d["stable_samples"]
                 or d["last_observation_seq"])):
            raise DockCycleError("idle dock transaction retains unfinished service")

    def _read(self):
        try:
            v=json.loads(self.path.read_text("utf-8"))
        except (OSError,ValueError) as e:
            raise DockCycleError("missing/damaged dock transaction") from e
        self._validate(v)
        return v

    def _write(self,state):
        self._validate(state)
        if self._state is not None:
            if self._read()["revision"]!=self._state["revision"]:
                raise DockCycleError("concurrent dock transaction modified")
        self.path.parent.mkdir(parents=True,exist_ok=True)
        fd,temporary=tempfile.mkstemp(prefix=".dock-cycle-",suffix=".tmp",
                                      dir=self.path.parent)
        try:
            with os.fdopen(fd,"w",encoding="utf-8") as f:
                json.dump(state,f,sort_keys=True,separators=(",",":"))
                f.write("\n");f.flush();os.fsync(f.fileno())
            os.replace(temporary,self.path)
            if hasattr(os,"O_DIRECTORY"):
                fd_dir=os.open(self.path.parent,os.O_DIRECTORY)
                try:os.fsync(fd_dir)
                finally:os.close(fd_dir)
        finally:
            if os.path.exists(temporary):os.unlink(temporary)
        self._state=state

    def _set(self,**values):
        self._write({**self._state,**values,"revision":self._state["revision"]+1})

    @property
    def state(self):
        return json.loads(json.dumps(self._state))

    def request(self,*,battery_low:bool,bin_full:bool,mop_dirty:bool,
                job_paused:bool,goal_terminal:bool):
        if (self._state["state"]!="idle"
            or self._state["cycles"]>=self.max_cycles):
            raise DockCycleError("another station transaction in progress / budget exhausted")
        if job_paused is not True or goal_terminal is not True:
            raise DockCycleError("station operation requires paused job and terminal navigation")
        if any(type(x) is not bool for x in (battery_low,bin_full,mop_dirty)):
            raise DockCycleError("service flags must be real booleans")
        needed=sorted(k for k,flag in (("recharge",battery_low),
             ("empty_bin",bin_full),("wash_mop",mop_dirty)) if flag)
        if not needed:raise DockCycleError("no station service required")
        self._set(state="awaiting_dock",reason="SERVICE_REQUESTED",
                  needed=needed,completed=[],nonce=secrets.token_hex(16),
                  station_ack=False,stable_samples=0,last_observation_seq=0)

    def confirm_dock_arrival(self, *, docked:bool, nav2_terminal:bool,
                             station_connected:bool):
        if self._state["state"]!="awaiting_dock":
            raise DockCycleError("dock arrival unexpected")
        if docked is not True or nav2_terminal is not True or station_connected is not True:
            raise DockCycleError("dock location/contact/terminal not independently confirmed")
        self._set(state="at_dock",reason=None)

    def start_service(self,*,station_ack_nonce:str,station_ready:bool):
        if self._state["state"]!="at_dock":
            raise DockCycleError("robot not confirmed docked")
        if station_ready is not True or station_ack_nonce!=self._state["nonce"]:
            raise DockCycleError("dock not ready or stale transaction response")
        self._set(state="servicing",reason=None,station_ack=True)

    def observe(self,*,battery_fraction:float,
                bin_full:bool,mop_dirty:bool,
                station_completed:tuple[str,...],
                station_ack_nonce:str,docked:bool,observation_seq:int):
        if self._state["state"] not in ("servicing","ready_to_resume"):
            raise DockCycleError("station observations outside service session")
        if not isinstance(station_completed,tuple):
            raise DockCycleError("invalid completion metadata")
        if (docked is not True or station_ack_nonce!=self._state["nonce"]
            or type(battery_fraction) not in (int,float)
            or not math.isfinite(battery_fraction) or not 0<=battery_fraction<=1
            or type(bin_full) is not bool or type(mop_dirty) is not bool):
            raise DockCycleError("invalid station proof or resource telemetry")
        if (type(observation_seq) is not int or observation_seq<=self._state["last_observation_seq"]
            or observation_seq>2**63-1):
            raise DockCycleError("station telemetry replayed or out of order")
        if any(k not in self._state["needed"] for k in station_completed):
            raise DockCycleError("unexpected/forged station step")
        acknowledgments=set(station_completed)
        verified=[]
        if "recharge" in acknowledgments and battery_fraction>=.80:
            verified.append("recharge")
        if "empty_bin" in acknowledgments and not bin_full:
            verified.append("empty_bin")
        if "wash_mop" in acknowledgments and not mop_dirty:
            verified.append("wash_mop")
        merged=sorted(set(self._state["completed"])|set(verified))
        # A previously confirmed recharge/vacuum/mop condition is not
        # a permanent guarantee: each of the last TWO DISTINCT telemetry
        # frames must contain a CURRENT good reading and the station's
        # acknowledgments for EVERY required service. Regression resets
        # the consecutive-stable counter, even after we were ready.
        fresh_all=(set(verified)==set(self._state["needed"]))
        stable=min(self._state["stable_samples"]+1,100) if fresh_all else 0
        ready=stable>=2
        self._set(completed=merged,stable_samples=stable,
                  last_observation_seq=observation_seq,
                  state="ready_to_resume" if ready else "servicing")

    def release(self,*,nav2_idle_verified:bool,fresh_pose:bool,
                resources_healthy:bool)->str:
        if (self._state["state"]!="ready_to_resume"
            or set(self._state["completed"])!=set(self._state["needed"])
            or nav2_idle_verified is not True or fresh_pose is not True
            or resources_healthy is not True):
            raise DockCycleError("station service not durably verified; deny resume")
        token=self._state["nonce"]
        self._set(state="idle",reason=None,nonce=None,
                  needed=[],completed=[],station_ack=False,stable_samples=0,
                  last_observation_seq=0,cycles=self._state["cycles"]+1)
        return token

    def revalidate_after_restart(self, *, nav2_idle_verified:bool,
                                 docked:bool,station_connected:bool):
        if self._state["state"]!="paused_recovery":
            raise DockCycleError("dock revalidation only after interrupted transaction")
        if nav2_idle_verified is not True or docked is not True or station_connected is not True:
            raise DockCycleError("service restart cannot be reconciled without dock/robot proof")
        self._set(state="at_dock",reason="SERVICE_REISSUE_REQUIRED",
                  completed=[],station_ack=False,stable_samples=0,
                  last_observation_seq=0,nonce=secrets.token_hex(16))

    def fail(self,reason:str):
        if not isinstance(reason,str) or not reason:
            raise DockCycleError("missing failure reason")
        if self._state["state"] in ("idle","fault"):
            raise DockCycleError("cannot fail inactive service")
        self._set(state="fault",reason=reason)
