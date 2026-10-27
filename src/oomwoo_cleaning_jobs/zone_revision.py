"""Transactional, fail-closed editing of live Nav2 keepout-zone masks.

An offline ZonePlan/mask file is not an enforced no-go. Editing is
permitted ONLY with a paused job and terminal Nav2 goal. Before the
controller re-arms, the new mask must be observed on ROS2 and lethal in
the actual global costmap plus its locally observable overlap.

This is a software admission contract, not a realtime safety monitor or
an MCU emergency stop. The ROS2 driver must supply independently fresh
costmap/filter-info observations and must never use a force override.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile

from .map_admission import Raster,AdmissionError
from .nav_keepout_admission import VerifiedGlobalKeepout,KeepoutNotEnforced


class ZoneUpdateError(RuntimeError):
    pass


def _mask_digest(mask:Raster)->str:
    return hashlib.sha256(json.dumps({
        "size":[mask.width,mask.height],"resolution":mask.resolution,
        "origin":[mask.origin_x,mask.origin_y,mask.yaw],"cells":mask.data},
        sort_keys=True,separators=(",",":")).encode()).hexdigest()


@dataclass(frozen=True)
class AppliedZoneVersion:
    revision:int
    digest:str
    state:str
    confirmed_samples:int


class ZoneUpdateTransaction:
    """Persistent edit transaction from one proven mask to another."""

    def __init__(self,source_map:Raster,current_mask:Raster,path:Path):
        if (not isinstance(source_map,Raster) or not isinstance(current_mask,Raster)
            or source_map.frame!="map"
            or current_mask.width!=source_map.width
            or current_mask.height!=source_map.height
            or current_mask.data.count(100)==0):
            raise ZoneUpdateError("initial live no-go mask missing/invalid")
        self.source_map=source_map
        self.path=Path(path)
        self.current_mask=current_mask
        self.source_hash=_mask_digest(source_map)
        self.current_hash=_mask_digest(current_mask)
        self.pending_mask=None
        self._state=None
        if self.path.exists():
            loaded=self._read()
            if loaded["current_digest"]!=self.current_hash:
                raise ZoneUpdateError("saved live zone digest does not match caller")
            if loaded["state"]!="active":
                # Stale work is never resumed just because a process reopened.
                loaded={**loaded,"state":"paused_recovery","proof_samples":0,
                        "revision":loaded["revision"]+1}
                self._write(loaded)
            self._state=loaded
        else:
            self._write({
                "schema":2,"source_map_digest":self.source_hash,
                "current_digest":self.current_hash,
                "pending_digest":None,"revision":1,
                "state":"active","proof_samples":0,
                "last_global_seq":-1,"last_local_seq":-1
            })

    def _validate(self,v):
        if (not isinstance(v,dict)
            or set(v)!={"schema","source_map_digest","current_digest",
                         "pending_digest","revision","state","proof_samples",
                         "last_global_seq","last_local_seq"}
            or type(v["schema"]) is not int or v["schema"]!=2
            or v["source_map_digest"]!=self.source_hash
            or not isinstance(v["current_digest"],str) or len(v["current_digest"])!=64
            or (v["pending_digest"] is not None and
                (not isinstance(v["pending_digest"],str) or len(v["pending_digest"])!=64))
            or v["state"] not in ("active","waiting_filter","verified","paused_recovery","fault")
            or type(v["revision"]) is not int or v["revision"]<1
            or type(v["proof_samples"]) is not int or not 0<=v["proof_samples"]<=2
            or any(type(v.get(k)) is not int or not -1<=v[k]<=2**63-1
                   for k in ("last_global_seq","last_local_seq"))):
            raise ZoneUpdateError("corrupted/stale zone revision")

    def _read(self):
        try:record=json.loads(self.path.read_text("utf-8"))
        except (OSError,ValueError) as exc:
            raise ZoneUpdateError("unreadable live zone revision") from exc
        self._validate(record)
        return record

    def _write(self,item):
        self._validate(item)
        if self._state is not None:
            disk=self._read()
            if disk["revision"]!=self._state["revision"]:
                raise ZoneUpdateError("zone revision modified by competing writer")
        self.path.parent.mkdir(parents=True,exist_ok=True)
        fd,tmp=tempfile.mkstemp(prefix=".zone-update-",dir=self.path.parent)
        try:
            with os.fdopen(fd,"w",encoding="utf-8") as f:
                json.dump(item,f,separators=(",",":"),sort_keys=True)
                f.write("\n");f.flush();os.fsync(f.fileno())
            os.replace(tmp,self.path)
            if hasattr(os,"O_DIRECTORY"):
                dfd=os.open(self.path.parent,os.O_DIRECTORY)
                try:os.fsync(dfd)
                finally:os.close(dfd)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)
        self._state=item

    def _set(self,**changes):
        self._write({**self._state,**changes,"revision":self._state["revision"]+1})

    def status(self):
        return AppliedZoneVersion(self._state["revision"],
                   self._state["pending_digest"] or self._state["current_digest"],
                   self._state["state"],self._state["proof_samples"])

    def stage(self,new_mask:Raster, *, paused:bool,goal_terminal:bool,
              verified_source_map:bool):
        if self._state["state"]!="active":
            raise ZoneUpdateError("cannot overlap two zone edits")
        if (paused is not True or goal_terminal is not True or
            verified_source_map is not True):
            raise ZoneUpdateError("zone change requires paused job, terminal goal and current map")
        if (new_mask.width!=self.source_map.width or
            new_mask.height!=self.source_map.height or
            new_mask.resolution!=self.source_map.resolution or
            new_mask.origin_x!=self.source_map.origin_x or
            new_mask.origin_y!=self.source_map.origin_y or
            new_mask.yaw!=self.source_map.yaw or
            new_mask.frame!="map" or
            any(v not in (0,100) for v in new_mask.data)):
            raise ZoneUpdateError("candidate keepout geometry/values wrong")
        digest=_mask_digest(new_mask)
        if digest==self._state["current_digest"]:
            raise ZoneUpdateError("identical mask does not require an update")
        self.pending_mask=new_mask
        self._set(pending_digest=digest,state="waiting_filter",proof_samples=0,
                  last_global_seq=-1,last_local_seq=-1)

    def verify_sample(self,*,actual_mask:Raster,global_costmap:Raster,
                      local_costmap:Raster,filter_info_active:bool,
                      status_fresh:bool,job_paused:bool,
                      global_sequence:int,local_sequence:int):
        if self._state["state"] not in ("waiting_filter","verified"):
            raise ZoneUpdateError("no pending zone edit")
        if (self.pending_mask is None or job_paused is not True
            or filter_info_active is not True or status_fresh is not True):
            raise ZoneUpdateError("filter info stale or no paused job")
        # Require independent sequential post-filter costmap *frames*;
        # a replay of an old latched message is not a second proof.
        if (type(global_sequence) is not int or type(local_sequence) is not int
            or global_sequence<1 or local_sequence<1
            or global_sequence<=self._state["last_global_seq"]
            or local_sequence<=self._state["last_local_seq"]):
            raise ZoneUpdateError("stale/replayed global or local costmap frame")
        if _mask_digest(actual_mask)!=self._state["pending_digest"]:
            raise ZoneUpdateError("live ROS2 mask does not equal proposed mask")
        try:
            VerifiedGlobalKeepout(self.source_map,actual_mask,global_costmap)
        except KeepoutNotEnforced as exc:
            raise ZoneUpdateError("global costmap not updated to forbidden cells") from exc
        # Local costmap is rolling; demand lethal costs for mask cells
        # currently in its raster, and at least ONE known prohibited
        # cell in the local observation. Remote cells cannot be audited
        # locally from here, so the upcoming navigation checks local
        # costmap continuously.
        overlap=0
        for i,cost in enumerate(actual_mask.data):
            if cost!=100:continue
            x,y=actual_mask.centre(i)
            try:idx=local_costmap.cell_at(x,y)
            except AdmissionError:continue
            overlap+=1
            if local_costmap.data[idx]<95:
                raise ZoneUpdateError("local Nav2 costmap missed a prohibited cell")
        if overlap<1:
            raise ZoneUpdateError("local mask overlap never actually observed")
        samples=min(2,self._state["proof_samples"]+1)
        self._set(proof_samples=samples,
                  last_global_seq=global_sequence,
                  last_local_seq=local_sequence,
                  state="verified" if samples>=2 else "waiting_filter")

    def commit(self,*,nav2_idle:bool,fresh_pose:bool,job_paused:bool):
        if (self._state["state"]!="verified"
            or self.pending_mask is None
            or nav2_idle is not True or fresh_pose is not True or job_paused is not True):
            raise ZoneUpdateError("new mask cannot be adopted without two live costmap proofs")
        self.current_mask=self.pending_mask
        self.current_hash=self._state["pending_digest"]
        self.pending_mask=None
        self._set(current_digest=self.current_hash,pending_digest=None,
                  proof_samples=0,state="active",
                  last_global_seq=-1,last_local_seq=-1)

    def must_block_nav(self):
        return self._state["state"]!="active"
