"""Exact-UUID Nav2 goal evidence for ISOLATED ROS2 dock velocity handoff.

Seeing a stale terminal ROS2 GoalStatusArray is not proof that OUR
current Nav2 goal ended: the status array can also contain another
client's old result. Require an exact UUID (journaled before dispatch)
observed ACTIVE during THIS mux ownership lease, the matching terminal
status, and the same durable GoalJournal terminal result.

This source is not a physical motor watchdog and doesn't authenticate
ROS2 topics. It must not authorize physical docking without an
independent MCU/actuator layer and a controlled robot DDS graph.
"""
from __future__ import annotations

import re

from .goal_journal import GoalJournal, GoalJournalError


class NavLeaseError(RuntimeError):
    """Current Nav2 action UUID was not proved terminal by this lease."""


ACTIVE=frozenset((1,2,3))
TERMINAL={4:"succeeded",5:"canceled",6:"aborted"}

_GOAL_UUID = re.compile(r"[0-9a-f]{32}\Z")
_KNOWN_CODES = ACTIVE | frozenset(TERMINAL)


def valid_nav_status_snapshot(statuses) -> bool:
    """UNKNOWN=0 is not idle. Reject invalid states, UUIDs and duplicates."""
    if not isinstance(statuses, tuple):
        return False
    seen = set()
    for entry in statuses:
        if not isinstance(entry, tuple) or len(entry) != 2:
            return False
        uuid, code = entry
        if (type(uuid) is not str or _GOAL_UUID.fullmatch(uuid) is None
            or type(code) is not int or code not in _KNOWN_CODES
            or uuid in seen):
            return False
        seen.add(uuid)
    return True


def nav_status_idle(statuses) -> bool:
    """Conservative idle predicate, usable without ROS2 imports."""
    return (valid_nav_status_snapshot(statuses)
            and not any(code in ACTIVE for _, code in statuses))


class NavLeaseWitness:
    def __init__(self,journal:GoalJournal):
        if not isinstance(journal,GoalJournal):
            raise TypeError("requires a real durable goal journal")
        self.journal=journal
        self.reset()

    def reset(self):
        self.uuid=None
        self.saw_active=False
        self.saw_terminal=False
        self.terminal_kind=None
        self.fault=None

    def observe(self,statuses:tuple[tuple[str,int],...]):
        if not valid_nav_status_snapshot(statuses):
            self.fault="unknown, duplicated or malformed ROS2 goal status"
            return
        active=[uuid for uuid,code in statuses if code in ACTIVE]
        if len(set(active))>1:
            self.fault="multiple Nav2 action owners active"
            return
        if self.uuid is not None and any(uuid!=self.uuid for uuid in active):
            self.fault="unknown Nav2 goal took control"
            return
        try:
            record=self.journal.read()
        except GoalJournalError:
            self.fault="Nav2 goal journal missing/corrupted"
            return
        if active:
            current=active[0]
            if (record is None or record["uuid"]!=current
                or record["state"]=="terminal"
                or (self.uuid is not None and self.uuid!=current)
                or self.saw_terminal):
                self.fault="active Nav2 UUID is not our current journaled goal"
                return
            self.uuid=current
            self.saw_active=True
        if not self.saw_active:
            return
        if record is None or record["uuid"]!=self.uuid:
            self.fault="Nav2 goal UUID changed without terminal proof"
            return
        if self.saw_terminal:
            # ROS2 and the durable journal must keep agreeing on the SAME
            # terminal outcome for our UUID. A later contradictory status
            # may be stale/misrouted; never retain an old handoff permit.
            if record["state"]!="terminal" or record["outcome"]!=self.terminal_kind:
                self.fault="terminal Nav2 proof retracted"
                return
            own_status=next((code for uuid,code in statuses
                             if uuid==self.uuid),None)
            if own_status is not None and TERMINAL.get(own_status)!=self.terminal_kind:
                self.fault="conflicting terminal Nav2 status for current UUID"
            return
        terminal=[code for uuid,code in statuses
                  if uuid==self.uuid and code in TERMINAL]
        if terminal:
            code=terminal[-1]
            kind=TERMINAL[code]
            if record["state"]!="terminal" or record["outcome"]!=kind:
                # Callback can arrive before disk fsync; do not mark
                # ready until a further matching status confirms.
                return
            self.saw_terminal=True
            self.terminal_kind=kind

    def can_release_for_dock(self)->bool:
        if self.fault or not self.saw_active or not self.saw_terminal:
            return False
        try:
            record=self.journal.read()
        except GoalJournalError:
            return False
        return bool(record is not None and record["uuid"]==self.uuid
            and record["state"]=="terminal"
            and record["outcome"] in ("succeeded","canceled")
            and record["outcome"]==self.terminal_kind)

    def current_active_matches(self)->bool:
        # A cached ACTIVE ROS2 status must not outlive its durable journal.
        if self.fault or not self.saw_active or self.saw_terminal or self.uuid is None:
            return False
        try:
            record=self.journal.read()
        except GoalJournalError:
            self.fault="Nav2 goal journal missing/corrupted while active"
            return False
        if record is None or record["uuid"]!=self.uuid:
            self.fault="Nav2 goal journal changed while active"
            return False
        # A terminal journal stops velocity immediately, without authorizing
        # the later dock handoff (which still needs exact terminal status).
        if record["state"]=="terminal":
            return False
        if record["state"]!="accepted":
            self.fault="Nav2 goal no longer durably accepted"
            return False
        return True
