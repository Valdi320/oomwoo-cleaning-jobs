# Experimental three-way /cmd_vel policy — 29 October 2026

**Status: LOCAL WIP, not published and not production-integrated.**

## Blocker addressed

The OOMWOO maintainer clarified in upstream PR #79 that room coverage
must move to a reactive /cmd_vel executor while Nav2 handles transits,
docking and localization. Nav2/dock-drive already have an experimental
two-way relay; adding a third publisher directly would risk concurrent
wheel commands. This independent, opt-in pure-Python reference policy
models **one shared STOP/NAV/SWEEP/DOCK owner** rather than silently
introducing a separate sweep-only mux.

## Implementation

- `src/oomwoo_cleaning_jobs/reactive_velocity.py` supplies
  `ThreeWayVelocityMux` and immutable `MotionHealth`. This is a
  proof/command policy with NO ROS2 imports, publishers or motors.
- Each claimed owner receives a new in-memory token. Wrong owner or old
  token cannot emit nonzero output; conflicting nonzero commands STOP.
- No owner is armed without fresh health and quiet odometry. Typical
  synthetic gates: 0.30 s max sensor/command age, 0.75 s observed quiet
  interval, 0.25 m/s linear and 0.8 rad/s angular command limits.
  These are *experiment defaults*, not certified robot limits.
- NAV needs a 32-char lowercase Nav2 UUID and a matching observed active
  goal before nonzero output. Transitioning to idle mutes output; release
  requires an exact terminal UUID and independently verified completion.
- SWEEP requires Nav2 idle, source isolation and dock inactive; DOCK
  requires Nav2 idle and sweep inactive. Both require the fresh quiet
  interval after a handoff. Terminal evidence is one-shot.
- Every output call checks current health, odometry and command age.
  Inconsistent map/resource/localization/safety status, foreign Nav2
  action, stale command or backward time fails closed.

## Reproduce, in this contributor-owned local branch

Starting with source commit
`3f3cd7025ab242564d9a940f15009b2edaabca63`:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_reactive_velocity_20261029.py' -v
```

Historical initial snapshot: 33/33 focal PASS and a previously reported
724/724 full suite, normal and `-O`, on the user's Linux x86 host.
The full suite has NOT been rerun under the current machine instructions. This is entirely **synthetic
unit-test evidence**; no new ROS2/Gazebo, ARM CPU/energy or physical tests.
Machine-readable re-run report is in
`docs/evidence/reactive_mux_synthetic_20261029.json`.

## Safety limits, compatibility and next integration gate

`MotionHealth` flags and `nav_released` / `terminal_verified` are
**claims supplied by the caller, not independently verified proofs**.
Their adapter MUST validate the source and freshness of each signal,
track a persisted exact Nav2 goal UUID, detect foreign controllers,
verify real velocity-source remappings, provide a physical MCU watchdog
and confirm that only ONE mux publishes to the true robot `/cmd_vel`.
In ROS2, raw /cmd_vel messages have no authentication or durable lease
tokens; this Python token is NOT a security credential.

The current contributor tree now wires this policy into an explicit
`three_way=true` mode of the SAME `ExperimentalDockVelocityRelay`.
The default two-source mode remains available; its active DDS probe passes
again. The opt-in mode has the focused DDS evidence recorded below.
The reactive room-sweep algorithm itself, geometry/obstacle avoidance,
and canonical job API are not implemented by this module.

The isolated three-source DDS adapter now has recorded handoffs and injected
input faults. Remaining integration is a Gazebo run with ACTUAL planner/Nav2/
docking controllers and their phase/UUID coordination, keeping one safe test
output. Do not deploy this module to a real motor topic.

No battery, motor, charging station, firmware or continuous service
has been touched. No claim about cheaper minimum SBC or lower energy
consumption is made.

**Contribution hygiene:** Apache-2.0 owner branch only; upstream #79
remains unchanged for maintainer review. Only local contributor commits are delivered for owner review/integration;
no upstream runtime changes were made.


## Opt-in ROS2 adapter milestone — 29 October 2026

The existing relay selects one policy: the historical two-source default or
`three_way=true`. It retains one publisher on
`/valdi/lab_safe_cmd_vel_output` and one 50ms timer. In three-way mode that
existing timer uses steady time, so a paused ROS simulation clock cannot
suspend output/expiry checks. TF subscriptions use this same node; no extra
application node, timer, tool or worker is introduced. The probe and remap
matrix are extended in their existing files.

| Observed input | Meaning in this disposable simulation |
| --- | --- |
| TF, validated static Raster | Common map-to-base time at most500ms old and a free current cell; no cached-AMCL motion fallback or swept-footprint claim. |
| BatteryState | Finite fraction20% admission /15% active through100%; a simulation battery proxy, without bin/water/charging proof. |
| /clock, meter stability, phases | Advancing clock, recently observed stable meter and known fresh producer phases. This is simulator health, not an MCU safety loop. |
| Command-topic graph | One distinct endpoint per NAV/SWEEP/DOCK input, one self-named lab output, no /cmd_vel publishers; endpoint changes or duplicates stop as observed. |
| Native DDS metadata | Strict publication sequence and same-host wall timestamp, after the current claim; replay history survives STOP. |
| Nav2 status + GoalJournal | Preallocated exact UUID and matching map digest before NAV claim, observed active action and matching durable terminal before handoff. |

Jazzy's actual MessageInfo is a dict with source/received timestamps and
publication/reception sequence numbers. It supplies NO per-packet publisher
GID. The initial assumption failed in a real DDS run and was removed. The
adapter associates packets with unique observed graph endpoints; neither
that association nor a source timestamp is authentication, and DDS discovery
snapshots are not atomic wiring proof. This mode is restricted to the same-host
lab; clocks on separate machines have not been validated.

A native dropout test also found that caching an ACTIVE UUID while refreshing
aggregate health could prolong NAV ownership after status stopped arriving.
The ACTIVE UUID now expires under the existing one-second nonzero-NAV status
budget. The red case reproduces continued ownership before correction; the
final scenario passes **14 checks**, including forced STOP on missing status,
rejection of a release from the lost lease, and a fresh exact-UUID claim before
the subsequent NAV-to-DOCK handoff.

Final native scenario also checks initial missing TF, positive SWEEP/NAV/DOCK
output, foreign commands, invalid battery percentages, TF dropout, command
expiry, overspeed, duplicate writers, paused simulation clock and a latched
map-change fault. **Nine constructor/remap cases** pass, rejecting miswired
sweep/phase/map/clock/stability/TF inputs and motor-output remapping before the
output timer. Four separate focal Python modules pass **12 +6 +24 +10** cases;
six new metadata tests use the real relay methods/core with synthetic clocks,
graph and message metadata. The original active two-way DDS probe also passes.
No full suite was run. Exact commands, negative attempts, hashes and scopes:
[`evidence/three_way_ros2_adapter_codex_20261029.json`](evidence/three_way_ros2_adapter_codex_20261029.json).

Reproduce within the existing isolated Jazzy image with this source at `/work`
and the existing lab interfaces built/sourced (no host tool installation):

```bash
PYTHONPATH=/work/src:$PYTHONPATH python3 \
  /work/integration/ros_dock_velocity_relay_probe.py --three-way
PYTHONPATH=/work/src:$PYTHONPATH python3 \
  /work/integration/input_subscriptions_matrix_20261029.py --three-way
```

This is a real DDS/service/TF-cache bank with SYNTHETIC command, odometry,
clock, map, resource, producer-phase and Nav2-status inputs. It does not run a
Nav2 action server, Gazebo, a detector, a physical robot or actual charging.
In this mode `release_motion` releases software ownership on a fresh zero and
inactive software phase; DONE/Bool does not certify contact or charging.
The historical `finish_dock` operation remains in two-source mode only.

The actual planner still needs a producer phase signal and a pre-send journal/
lease handoff before its Nav2 action. Its0.35m/s and1.0rad/s defaults exceed the
unchanged mux limits0.25/0.8. Actual5Hz simulation cadence must be measured
against the300ms monotonic command budget, and the real Nav2 status/feedback
cadence must be checked before choosing its observation contract. Next work is
this real-controller/Gazebo integration and final single-writer bridge audit;
this milestone does not substitute for it. Hardware availability is explicitly
zero, and no physical strength, RF, charging or robot readiness is inferred.


## Actual-controller adapter work — 29 October 2026, local WIP

`managed_nav2=true` is an additional opt-in mode of this same relay and
requires `three_way=true`. It is for the private Gazebo integration only.
It does not enable a hardware motor topic or change the two-source default.
The owner confirms that the project has **no hardware: design and simulation
only**, so none of the evidence in this document is physical validation.

A passive test_room observation found six actual Nav2 velocity endpoints
(one controller and five behavior publishers), rather than one per node.
Its action status changes on events; ACTIVE status is not a periodic health
heartbeat. The initial managed experiment attempted to pin this multiwriter set and
use a subscription-wide reception sequence. Actual movement was rejected:
Fast DDS reports this sequence as UNSUPPORTED. The primary rmw_fastrtps
implementation confirms that fact. This attempted adapter is not accepted.
The managed launch now removes behavior_server and disables the unused
NavigateThroughPoses navigator. It uses Nav2's stock no-recovery behavior tree
with ComputePathToPose and FollowPath. Thus only controller_server can publish
NAV input; every input retains one pinned endpoint, strict publication
sequence and source timestamp checks. No new consolidator node/timer was
added. A transit requiring Nav2 recovery fails and requires reconciliation;
coverage's reactive bumper escape remains available while SWEEP owns motion.
All roles share the same existing final lab publisher and timer.

Fresh actual feedback can maintain an already witnessed exact ACTIVE UUID;
it requires the matching durable journal and a current pose stamp. Idle
reconciliation additionally requires a fresh ACTIVE lifecycle GetState reply
and the sole external NavigateToPose SendGoal client coverage_planner. Jazzy
also has an internal bt_navigator self-client: the primary implementation
forwards the RViz goal_pose topic through that client. The managed launch
remaps this alternative ingress to /valdi/disabled_nav2_goal_pose. The adapter
requires its one expected bt_navigator subscriber and no publishers on either
that topic or /goal_pose, with no subscriber remaining on /goal_pose. These
observations apply to common managed health during movement as well as idle.
DDS graph discovery is not access control or an atomic wiring guarantee.

Primary source: [Nav2 Jazzy navigate_to_pose.cpp](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp),
blob 2f7ed68aa3d68e984e6473db09a64091aaeb7866, configure/onGoalPoseReceived.
The actual runtime client inventory independently contained both nodes.
Fast DDS source: [rmw_take.cpp](https://github.com/ros2/rmw_fastrtps/blob/jazzy/rmw_fastrtps_shared_cpp/src/rmw_take.cpp),
blob 19d96fd61ef22992d8fbd4430de354aababccfe7, _assign_message_info.
The stock no-recovery tree was read from the cached Jazzy image before use.

Core odometry, leases, commands and health use the same ROS simulation clock
in this managed mode; wall-time receipt/TF age, lifecycle query timeout and
the independent clock-progress watchdog remain separate. The existing steady
tick retries a lifecycle query after its unanswered future expires at 1.5 s.
It never promotes an unanswered request to healthy. Terminal release still
requires exact status/journal agreement; an unknown or aborted result fails.

Focused actual-method tests cover quarantined self-client acceptance,
rejection of public ingress/extra clients, feedback UUID/pose/journal freshness,
consistent simulated lease time, and bounded lost startup query recovery.
The pre-movement diagnostic runs remain negative integration evidence, not a
successful arbitrated coverage run. Detailed final runtime evidence follows
when the actual-controller scene is completed.

The actual first transit reached its target and wrote a succeeded journal,
but its subsequent handoff failed in the initial single-NAV-source run.
Two event-order assumptions were corrected: the two-second initial ACTIVE
watchdog must not restart after an already observed ACTIVE goal terminates;
and a terminal status received before the result is fsynced must be correlated
with that journal once the durable callback completes. The same received
terminal snapshot is reconsidered only inside its original one-second
freshness window; its receipt timestamp is never refreshed. Release still
requires both exact matching sources. The actual-method regression uses a
real GoalJournal and NavLeaseWitness with terminal-before-fsync ordering.
A stale terminal event is not reconsidered as fresh.

Native remap checks found that rclpy Client.srv_name retains the nominal name.
The lifecycle and planner relay-client guards now use the public
Node.resolve_service_name API, which applies remaps. The final managed
constructor matrix passes three cases, including feedback and lifecycle
redirection rejected before the output timer; it sends no Twist.
Primary API source: [rclpy Node.resolve_service_name](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/node.py#L1495),
blob 1cace64a22670e3bb3cd9da7fc05e6d42faeff29. The corresponding failed native
attempt is retained as negative evidence rather than a successful check.


### Bounded actual-controller observation

The final 180-second diagnostic ended normally at the scorer wall-time bound,
with 22.55% partial measured coverage, no recorded relay fault, four acknowledged
NAV releases and five SWEEP claims. Actual status records contain three
successful goals and one canceled goal after the existing waypoint timeout.
The command graph has one controller_server NAV publisher, one coverage_planner
SWEEP publisher, one IDLE DockDrive publisher and one final relay writer consumed
by ros_gz_bridge; /cmd_vel and /cmd_vel_nav have no publishers. There were no
pose jumps or instability. Total collection wall time including setup/teardown:
201.98 s. This is **not a whole-room PASS** (bank exit 1, wall_timeout).
No escape entry occurred in this final scene, so its zero-on-entry correction
is supported by the three focal method cases, not by this trajectory.

Four separate focused modules pass 49 +31 +13 +8 cases; three native managed
constructor/remap cases pass. Flake8 and pep257 pass on the six changed ROS2
files. No full suite was run. See [exact scoped evidence and retained failures](evidence/managed_nav2_gazebo_codex_20261029.json).
The managed boot deadline now also covers the wait before plan_started, so a
lost Nav2 activation response cannot leave this mode waiting indefinitely.
Remaining acceptance: full-room performance, actual escape exercise, docking
trajectory/phase coordination and injected faults with the actual controllers.
All work remains local; the owner has no hardware.

The existing basic three-way DDS scenario also passes its 14 checks again
on this relay source; its generated motion/health/status inputs remain synthetic.


## Held contact during NAV — follow-up in progress

The first whole-room attempt on ec34c0e/c53a6d0 stopped at 75.31% coverage
(bank exit1, motion_fault, 561.21s including setup/teardown). Nine Nav2 goals
succeeded and ten releases completed before controller_server reported Failed
to make progress; the eleventh goal ended ABORTED. The managed adapter correctly
latched the existing fault. This is failed whole-room acceptance, not PASS.

Source inspection found that managed NAV returned before the existing held
bumper reflex could run. The follow-up asks cancellation once on held contact,
publishes TRANSIT_CANCEL to mute the existing relay NAV output, waits for the
actual result/journal and ordinary terminal/quiet/permission handoff, and only
then schedules the already existing contact reflex as SWEEP owner. It preserves
the observed direction/target through asynchronous acceptance and handoff.
An actual ABORTED result still faults; it never becomes a docking or recovery
permission. No output timer, implementation node, tool or worker is added.

CancelGoal must acknowledge exactly the current UUID with ERROR_NONE. Its
ERROR_GOAL_TERMINATED race grants nothing and only waits, at zero, for the actual
result. Rejected, foreign/duplicate UUID, transport and three-second unresolved
cancellation paths latch fault. Fault cancellation is bounded to the first fault
rather than repeated every tick; STOP transport errors cannot create a request
loop. Primary contract: [CancelGoal.srv](https://github.com/ros2/rcl_interfaces/blob/jazzy/action_msgs/srv/CancelGoal.srv),
blob 86afbdbf4eacb2c55c1faf8da3a17946c5f0fa9b.

Current focal evidence: 59 planner cases and33 real-method/metadata cases PASS,
including no escape before terminal plus SWEEP acknowledgement, cancel before
acceptance, cancellation/result race, bounded failures, continued ABORTED fault,
and actual relay output muting while retaining NAV ownership. The health phase
is accepted only in managed mode. Actual Gazebo confirmation of this follow-up
and whole-room acceptance remain pending at this note's entry.


### Complete test_room follow-up: scored PASS

The complete follow-up run finished sweep_complete with **98.66% coverage**,
**91.06% efficiency at the first90% crossing**, and70.49% final efficiency.
The existing90%/80% acceptance criteria were unchanged. The scorer reports605s
to target and total collection time857.12s including setup/teardown. Geometry
measurement remained valid with no pose jumps or simulator instability.

Actual status history contains21 distinct terminal UUIDs:17 SUCCEEDED and4
CANCELED. All21 NAV releases and22 SWEEP claims/releases were acknowledged;
the final producer phase was IDLE, with its final release before scoring ended.
Two actual held-contact episodes during NAV caused cancellation, exact terminal
release, quiet handoff and the existing reactive escape under SWEEP ownership.
No synthetic contact publisher or substitute controller was used in this run.
The final command graph still has one source per input and one final relay
publisher consumed by ros_gz_bridge,with no /cmd_vel publishers. No relay fault
was logged during the measured scene.

The first75.31% failed attempt remains preserved. This is one successful
follow-up,not a controlled A/B or a reliability estimate. See [full scoped
result,contact sequences,source hashes and retained errors](evidence/reactive_contact_full_room_codex_20261029.json).

**Shutdown limit:** the scored scene passed,but process shutdown was not clean.
After the final summary,the relay logged KeyboardInterrupt,the Gazebo bridge
exitedSIGABRT(-6),and Gazebo required the existing launchSIGTERM escalation.
The bridge abort cause remains open;the owned --rm container is terminal.
The existing sub-inscribed-inflation warnings and inner FollowPath halt-result
errors are also retained. Do not turn a scored PASS into a clean-log or physical
safety claim. Actual docking,combined active Keepout,managed fault injection,
shutdown diagnosis and broader-scene validation remain open.


### Managed simulation shutdown: UDP workaround and native Gazebo exit

The core dump from the98.66% run places heap-corruption detection in Fast DDS
2.14.7 shared-memory teardown. It does not identify the original corrupting
write. The managed launch now defaults to builtin UDPv4, retaining an explicit
parent transport choice and leaving the legacy launch mode unchanged. The
optional motion audit records that choice. This is a scoped workaround, not an
upstream memory repair.

The relay catches normal KeyboardInterrupt/ExternalShutdownException and still
cleans up. The existing regression launch gives its Gazebo transport a private
partition, then requests native /server_control stop when the scorer exits.
After a positive reply, the existing required-Gazebo exit closes the launch.
Sending Launch shutdown immediately after the reply interrupted native teardown;
that failed intermediate observation is retained. Missing/failed replies retain
the existing immediate shutdown. No new worker, node or timer was introduced.

The final bounded30s scene returned a positive native reply and every non-scorer
process exited cleanly, including Gazebo, the bridge and the relay. The scorer
correctly returned1 for wall_timeout; this partial scene did not pass coverage.
The previous bounded90s observation closed the bridge/relay cleanly but still
needed Gazebo SIGTERM. Four actual LaunchContext cases and lint on the two
changed ROS2 Python files pass. The existing14 DDS checks passed with UDP before
the relay exception-only change. See [source versions, all intermediate results,
exact exits and limitations](evidence/managed_shutdown_udp_codex_20261029.json).

The full post-change test_room passes: **98.51% coverage**, **91.33%
efficiency at the first 90% crossing**, 70.32% final efficiency, and 599.0 s to
target by the scorer. Total collection time was 855.96 s. The existing 90%/80%
criteria, geometry and motion limits were unchanged; no pose jumps or simulator
instability occurred. Status history contains 21 terminal UUIDs: 15 SUCCEEDED and
6 CANCELED. All 21 NAV releases and 22 SWEEP releases were acknowledged; final
phase was IDLE. One actual held-contact episode caused verified cancellation,
quiet handoff and the existing reactive escape. DockDrive remained IDLE.

The native stop returned data:true and **all 18 owned processes exited 0**,
including Gazebo, ros_gz_bridge and the relay. No SIGTERM escalation, bridge
SIGABRT or relay KeyboardInterrupt appeared. The graph still shows one input
publisher per role, one final relay output consumed by the bridge, and no
/cmd_vel publishers. This is one full successful observation after the change;
it does not prove an upstream memory fix, reliability, or controlled A/B gains.
The two sub-inscribed inflation errors and four inner FollowPath halt-result
errors remain recorded. A successful close is not a clean-log claim.

This consolidates the scoped shutdown milestone. Runtime source fingerprints
and the terminal/output/handoff evidence match. Repeating the green scene or
further transport polishing has low demonstrated return without new
evidence. The next required integration is active Keepout at the existing
final output: Nav2's mask topic is /oomwoo/keepout_mask, the planner currently
subscribes to keepout_filter_mask, and the relay does not yet consume the
mask/costmaps. The synthetic envelope also assumes a sequenced mux and 0.3 s map
freshness, while the current Nav2 costmaps publish at 2 Hz; it cannot simply be
attached or refreshed with invented sample times. Fuse existing components
and verify real masked movement before claiming that combination. Actual
docking remains pending.

The earlier 98.66% result stays tied to its original hashes. A positive native
ACK followed by a permanently hung Gazebo destructor has not been fault
injected; this uses the existing required-server exit without a new watchdog.
All evidence concerns software and simulation; no physical prototype exists.


### Keepout fusion in progress: direct command and transport evidence

The opt-in integration connects the native /oomwoo/keepout_mask to the planner
and folds the existing arc/swept-disk projection into the relay's sole output.
The synthetic envelope retains its original three-map checks. The live mask
path attests immutable aligned content, stable publishers and a recent original
post-filter global costmap; receiving static bytes does not refresh their age.
Masked coverage requires the existing relay/journal mode because a Nav2 sweep
can still issue a direct bumper/wedge escape. No extra output node or timer is
introduced. Ordinary obstacle/contact clearance retains its previous behavior;
keepout dilation now includes corners and its explicit margin.

The projection22, relay42 and planner70 focused cases pass. One existing DDS
probe run passes17 checks with required mask evidence, including direct virtual
intersection, stale post-filter map and fresh-but-unenforced map. It uses
synthetic clock/maps/commands, not an actual Nav2 server or Gazebo. A queued
zero from before a new lease is ignored without acquiring command/release proof;
old nonzero commands still fault. The first DDS rearm failure remains recorded
without a confirmed original cause. A separate diagnostic assertion was updated
because the output pose limit stops earlier than the common health witness.

The scorer's final13 cases and five-file lint/admission contexts pass. The
first masked Gazebo scene observed5181 poses with no disk/chord crossings, but
failed to release a goal canceled immediately after acceptance. A deterministic
case reproduced the lost ACTIVE observation between50ms output ticks. The
existing Nav2-status callback now passes its observed health to the mux, keeping
exact UUID, journal and common-health requirements. Its43-case module and17 DDS
checks passed. A later44-case module also checks that the first lease revocation
survives subsequent stopped commands in the existing bounded log/status record.

The next native run failed early with unexplained ownership loss and an invalid
or stale truth sample. Its original first cause was not retained; no clock or
transport cause is established. The first run also had Gazebo SIGSEGV139 after
a positive native stop reply. Two later clean closes do not repair that cause.
Both failed runs remain part of the evidence.

A final bounded180s scene observed8858 original ground-truth poses,8857 chords,
maximum20ms spacing and zero crossings for the0.18m circular proxy. Observed
SWEEP distance was9.74m, TRANSIT10.42m and ESCAPE0.28m. Native mask/info servers
and the post-filter global costmap were connected; the graph retained one final
output. Five Nav2 UUIDs succeeded and one was canceled. All20 auxiliary processes
closed cleanly. The scorer correctly exited1 for wall_timeout:16.37% coverage is
partial, not the full90%/80% acceptance. Final observed phase wasSWEEP, not a
completed sweep or an IDLE handoff.

Minimum sampled center-to-mask-bbox distance was0.206m. This supports the
reported circular-proxy observation, not a ground-truth0.06m margin, complete
URDF-footprint certificate, hardware stop or reliability claim. Eight poses
before mask arrival were counted but excluded from this observation. Full masked
coverage, intermittent-failure diagnosis, actual docking and physical validation
remain open. No hardware exists. See [exact source versions, all observations
and retained failures](evidence/reactive_keepout_fusion_codex_20261030.json).


The subsequent full masked acceptance **failed** at53.84% coverage after423.92s
wall time. Its19975 poses and19974 chords had20ms maximum spacing and zero
proxy-footprint crossings. Six Nav2 UUIDs succeeded and three were canceled;
eight of nine NAV releases were acknowledged. The first revocation was
"Nav2 goal journal missing/corrupted" during another immediate cancellation.
The final valid canceled journal does not erase that observed failure. A prior
rapid cancellation in the same scene did release successfully and resumed
SWEEP/escape, providing native evidence for the between-ticks callback repair.

The existing journal can independently reproduce a legitimate atomic-replace
race: after opening the old file but before fstat, accepted() replaces it; the
old descriptor has nlink0 and is rejected while the new named record is valid.
The native witness discards its exact read exception, so this remains a
reproduced candidate cause, not a proven explanation of the native failure.
At preparation, the bounded-reopen proposal and three focal cases were unapplied
pending explicit owner authorization under machine rule7. The subsequent
authorized application and its verification status are recorded below. Full masked acceptance
remains open. Gazebo also exited139 after its positive native ACK in this full
attempt;19 other auxiliary processes closed cleanly. This is distinct from the
previous FastDDS bridge issue and has no established repair. No hardware exists.


Offline inspection of the existing Gazebo core now localizes the shutdown
failure to iteration of the `events::Stop` callback map. RTTI confirms the
StopTag event; the main thread is in `std::thread::join` under `Server::~Server`.
Both owned failures have the same relative library offsets. The installed
Event.hh cleans pending connections under its mutex and then iterates the map
without that mutex. [Gazebo8.15.0 ServerPrivate](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/ServerPrivate.cc)
creates a Stop RPC thread and also calls Stop from destruction. This supports a
concurrent event-cleanup/lifetime hypothesis; the exact invalidating operation
and a repair remain unproven. No new simulation or runtime change was needed.

The first offline GDB trace could not unwind the Ubuntu libraries until its
analysis-only `/lib -> usr/lib` mapping was corrected. The final selected
frames and event RTTI are available, although219 other library inputs and
libthread_db remain absent. Nearest-export addr2line labels are misleading here:
SimulationRunner::Stop is only13bytes; the two failure offsets lie elsewhere.
The old CET-bounded metadata query also missed the UTC event; both core records
exist. Fingerprints, raw traces, limits and the existing sparse offline core are
referenced in the evidence. This is independent of the prior FastDDS bridge
workaround and does not establish a fix. Persistence was unchanged at that offline diagnosis. See the subsequent
authorized application below.


The owner subsequently answered "Autorizo" for the prepared journal patch.
It is now applied in the owned contribution copy. A read of a detached regular
inode closes that descriptor and reopens the named path once; a second detached
inode is refused. All original file, permission, size, schema and UUID checks
and the serialized writer remain in place. Three causal/security cases were
added to the existing inode module; they have NOT executed yet.

The first verification attempt did not launch Python unittest: the machine
guard reported that the owner was playing. This is an applied, unverified WIP,
not a tested fix or a successful native Keepout acceptance. The race is proven
independently; its identity with the native generic journal error is still
unproven. The next work is the listed focal modules, individually after LIBRE,
then a discriminating integration. No simulation or new scheduler was started.


## Authorized journal follow-up — 30 October 2026

The original bounded-reopen proposal at1ee0a44 now has focused validation in an
independent copy. A missing path on the second open is an error, preserving the
initial-missing cold-start behavior. No writer, lock or record-schema change is
made. Existing witness faults retain a bounded original exception instead of
only reporting missing/corrupted. The existing DDS probe wrapper also forwards
the relay's proof keyword; its earlier TypeError is retained as a probe failure.

Eleven inode/reopen cases and25 exact-UUID witness cases pass. The actual DDS
probe passes all17 checks with synthetic inputs and required Keepout evidence.
This does not establish the cause of the old native journal failure, complete
masked coverage, Gazebo shutdown repair or physical control. Next is one full
masked native case using unchanged90%/80% criteria, not more journal variants.
See [before/after sources, retained failures and limits](evidence/journal_reopen_delivery_20261030.json).


### Original-source TF selection across clock arrival order (2026-10-30)

A native masked-room run stopped because its newest TF stamp led the received
simulation clock by 5 ms, although a prior pose was still usable. The relay
now retains at most one admitted and one pending TF, as independent copies.
Health and the Keepout envelope share this temporal selector. A pending source
can be selected only after its original stamp is no longer future. The existing
500 ms health and 200 ms envelope budgets still apply. Reset discards both.

The original method fails the causal case. A first single-copy version passes
51 focused cases but fails a repeated-lead discriminator: it cannot promote
the preceding observation after the clock reaches it. The final version passes
52 cases, including successive 5 ms leads, both age boundaries plus 1 ns,
missing history/lookup, mutation of the incoming message, and reset. Earlier
negative and intermediate positive results remain in the evidence.

The existing DDS probe uses its same emitter timer and output subscription
to inject 1.2 seconds of 5 ms TF lead. All 18 checks pass in 47.218 seconds,
including its prior 17 checks. During 24 observations, newest TF age was
-5 ms while used-source age was 44.537–100.091 ms; each observed output stayed
at 0.18 m/s under the same lease. These are real ROS2 messages, TFBuffer and
services with synthetic inputs. There is no Nav2 action server, Gazebo scene
or physical robot. The change adds bounded message history to the same relay
without changing its interfaces, timers, journal or freshness thresholds.

Reproduce the focused file with the existing host Python environment:

    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m unittest discover \
      -s tests -p test_sequenced_motion_ingress_20261029.py -v

For native DDS, use the exact disposable-container command in
[evidence/pose_clock_skew_dds_20261030.json](evidence/pose_clock_skew_dds_20261030.json).
Machine ownership was checked; the owner's explicit compatible-resource
exception was used with CPU/memory limits and no GPU access. Full masked-room
acceptance remains pending. [Delivery and limits](evidence/pose_clock_skew_delivery_20261030.json).
