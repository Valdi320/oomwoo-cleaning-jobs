"""Bounded idempotent request-handling for experimental ROS2 job services.

Retried ROS2 calls must never accidentally issue duplicate navigation
or restart a new job because a network/client response was delayed.
In-memory within one executor process; crashes additionally rely on
durable job checkpoints and Nav2 UUID journal for fail-closed safety.

This is not authentication/authorization for a physical robot. Private
services must run only in isolated simulation until a reviewed API exists.
"""
from __future__ import annotations

from collections import OrderedDict
import hashlib
import json
import re


class CommandReplayError(RuntimeError):
    pass


class RequestDedupe:
    def __init__(self,max_entries:int=128):
        if type(max_entries) is not int or not 8<=max_entries<=4096:
            raise ValueError("invalid request cache limit")
        self.limit=max_entries
        self._records=OrderedDict()

    def handle(self,request_id:str,request:dict,action):
        if (not isinstance(request_id,str)
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,100}",request_id)):
            raise CommandReplayError("request_id must be stable ASCII token")
        if not isinstance(request,dict) or not callable(action):
            raise CommandReplayError("invalid request or handler")
        try:
            canonical=json.dumps(request,sort_keys=True,separators=(",",":"),
                                  ensure_ascii=False,allow_nan=False)
        except (TypeError,ValueError) as e:
            raise CommandReplayError("unserializable or non-finite command") from e
        if len(canonical)>2048:
            raise CommandReplayError("oversized command")
        digest=hashlib.sha256(canonical.encode("utf-8")).hexdigest()
        if request_id in self._records:
            oldhash,outcome=self._records[request_id]
            if oldhash!=digest:
                raise CommandReplayError("request ID replayed with different command")
            self._records.move_to_end(request_id)
            return json.loads(json.dumps(outcome))
        outcome=action(request)
        if not isinstance(outcome,dict) or type(outcome.get("ok")) is not bool:
            raise CommandReplayError("handler did not return an explicit ok flag")
        self._records[request_id]=(digest,json.loads(json.dumps(outcome)))
        if len(self._records)>self.limit:
            self._records.popitem(last=False)
        return json.loads(json.dumps(outcome))

    def size(self):
        return len(self._records)
