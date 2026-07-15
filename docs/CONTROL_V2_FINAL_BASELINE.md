# ControlV2 Formation — Final Verified Baseline

This document fixes a stable, fully-verified baseline of the ControlV2 formation-flight stack so the next
development phase can start from a known-good point. It records no new code — it describes what exists at the
commit below, what is proven, and what is explicitly **not** yet guaranteed.

## Baseline commits

| | commit | subject |
| --- | --- | --- |
| **Final HEAD** | `fc050c1` | `fix(test): handle prime identity mismatch in NPFG harness` |
| Phase F | `7f9d7b7` | `fix(control-v2): preserve legacy on rejected formation command` |
| Phase G | `fc050c1` | `fix(test): handle prime identity mismatch in NPFG harness` |

Branch `rebuild/planner-v2`. Phase G's parent is Phase F (`7f9d7b7`); neither commit was amended.

Lineage below the baseline: `fe7cd46` (arbiter) → `88cd720` (prime handoff) → `af4d731` (candidate
producer) → `b44b9ee` (operational mode) → `7f9d7b7` (Phase F) → `fc050c1` (Phase G).

## End-to-end data flow

A single operational command drives the whole stack; nothing auto-activates.

```
operator / BT / bridge
  │  UDP datagram (port 5010): control_mode, leader_name, slot_front/right/up_m,
  │                            command_sequence, command_timestamp  (+ existing guidance_mode etc.)
  ▼
AUDPControlReceiver::ReceiveUDPData → StoreOne (parse into FUavSetpoint, latest-wins per aircraft)
  ▼
AUDPControlReceiver::ApplyAutopilotToPawn(pawn, setpoint)
  │   JSBSim snapshot + health checks
  ▼
AUDPControlReceiver::RouteControlV2(pawn, setpoint, pawns)      ← the Phase F contract lives here
  │   resolves leader by label, finds/creates the per-aircraft runtime owner,
  │   pushes an operational request, and returns whether to SKIP the legacy writer this frame
  ▼
UFormationRuntimeOwnerV2 (UActorComponent on the follower; ticks after the movement component)
  │   ApplyOperationalRequest → validate / accept / reject; owns the state machine
  ▼
FFormationCandidateProducerV2   (real NPFG lateral guidance + TECS longitudinal + F16StickAdapterV2)
  │   once per genuine sim-time advance, submits a full candidate through the Prime/handoff contract
  ▼
MumtCommandArbiterV2   (process-global, production-bound at module startup)
  │   resolves LegacyOrManual vs FormationControlV2 at the CopyToJSBSim CONSUME boundary
  ▼
JSBSim FDM   (the arbiter's resolved block is what the FDM consumes — no producer writes surfaces directly)
```

Return value of `RouteControlV2` means **"skip the legacy inner-loop writer this frame"**, not "the packet
said formation". If it returns `false`, `ApplyAutopilotToPawn` falls through to the existing legacy guidance.

## Formation request: accept / reject and ownership

`RouteControlV2` returns `bAccepted || bRuntimeOwnsOrPending`, where `bRuntimeOwnsOrPending` is
`IsFormationRequested()` OR the phase is `Warming`/`Priming`/`AwaitingActivation`/`Active`. This yields four
cases:

| situation | `ApplyOperationalRequest` | owner state after | `RouteControlV2` | effect |
| --- | --- | --- | --- | --- |
| **Valid enable** (rising edge / idempotent hold / slot update / leader change) | accepted | requested / taking over | `true` | legacy writer skipped; ControlV2 engages |
| **Rejected from Idle** (stale / no-leader / non-finite / replayed, owner not yet owning) | rejected | Idle, `!IsFormationRequested` | **`false`** | **legacy writer keeps running — follower is never left un-commanded** |
| **Rejected while pending/Active** | rejected | still `IsFormationRequested` / Active | **`true`** | **ownership kept; no re-prime; bad values not applied; legacy does not re-enter** |
| **Explicit disable** (`control_mode != formation`) | disabled | Idle | `false` | immediate Legacy fallback; legacy writer runs the same frame |

The key invariant behind rows 2–3: a rejected `ApplyOperationalRequest` does **not** change
`bFormationRequested`. So a bad packet from Idle leaves the owner Idle (legacy runs), while a bad packet
under an Active owner leaves it Active (ownership kept). This is the Phase F fix — before it, `RouteControlV2`
returned `true` for any `control_mode=formation` packet and a rejected command from Idle suppressed the
legacy writer, leaving the follower flying a held command into the ground (observed fall to Alt=-27746).

## State machine

```
Idle ──(valid rising edge accepted)──▶ Warming ──▶ Priming ──▶ AwaitingActivation ──▶ Active
 ▲                                                                                       │
 └──────────────────── any fallback (disable / Falling / leader loss / identity /  ──────┘
                        world mismatch / sim-time discontinuity / stale) ───────────────▶ Idle
```

- **Warming** — run the NPFG/TECS/stick chain in shadow so the first real candidate is valid. Legacy writer
  is already skipped (the enable was accepted).
- **Priming** — `BeginHandoff`: submit the primed candidate; the exact-zero first consume is the stick
  latch's, continuous with the last resolved block.
- **AwaitingActivation** — prime submitted + activation requested; the arbiter confirms at the next consume
  boundary.
- **Active** — a fresh candidate is produced each advancing sim-time frame. Mode stays Legacy until the
  boundary confirms; there is no blend.

Any fallback drops immediately to Idle and requires a **new** explicit enable to resume — a held command
never silently re-activates Formation.

## Rejected packet while Active — ownership is kept

When the owner is Active and a **new-sequence but invalid** packet arrives (stale / non-finite slot /
missing leader / replay):

- the packet is **rejected** and **not applied** — the applied sequence, leader weak-pointer and slot are
  unchanged, so the producer keeps flying the previously-accepted leader/slot;
- there is **no re-prime** — the prime generation does not change;
- the runtime **stays Active** and the candidate generation keeps advancing;
- the legacy writer **does not re-enter** — `RouteControlV2` returns `true` because `IsFormationRequested()`
  still holds;
- no invalid value ever reaches the FCS.

## Explicit disable — immediate Legacy return

A `control_mode != formation` packet (explicit `legacy`, or absent) calls `ApplyOperationalRequest(false)`:
the owner falls back to Idle immediately, the producer is reset, `RouteControlV2` returns `false`, and the
legacy inner-loop writer runs the **same frame**. The next consume resolves Legacy. There is no blend and no
carry-over of the ControlV2 candidate.

## No direct JSBSim surface write

Producers never write `fcs/*-pos-rad` (or the JSBSim command block) directly. They submit a candidate to the
arbiter, which resolves exactly one block at the `CopyToJSBSim` consume boundary. In `LegacyOrManual` mode the
resolver leaves the legacy block untouched (`LegacyBlockMutationCount` must be 0); in `FormationControlV2`
mode it writes the resolved candidate. This is what lets Legacy and ControlV2 aircraft coexist in one world
with per-aircraft isolation, and what the ownership telemetry and arbiter invariants enforce.

## Verified tests at this baseline

All gates below were run sequentially (single UnrealEditor instance) at `fc050c1`.

| gate | result |
| --- | --- |
| `git diff --check` | clean |
| Operational formation (`run_formation_operational_v2.sh`) | **13/13** scenarios, `COMMAND_FORMATION_OPERATIONAL_V2_RESULT=PASS` |
| Formation candidate (`run_formation_candidate_integration_v2.sh`) | **8/8** scenarios, `COMMAND_FORMATION_CANDIDATE_V2_RESULT=PASS` |
| Command prime (`run_command_prime_v2.sh`) | `COMMAND_PRIME_V2_RESULT=PASS` |
| Command arbiter (`run_command_arbiter_v2.sh`) | `COMMAND_ARBITER_V2_RESULT=PASS` |
| Command ownership (`run_command_ownership_telemetry.sh`) | `COMMAND_OWNERSHIP_RESULT=PASS` |
| NPFG lateral closed loop (`build_verify_npfg_f16_lateral_closed_loop_v2.sh`) | `NPFG_F16_LATERAL_CLOSED_LOOP_V2_RESULT=PASS`, failures=0, non_finite_states=0, command_range_violations=0, writes_outside_owned_fdm=0 |
| NPFG caller contract (`build_verify_npfg_caller_contract_v2.sh`) | checks=**109**, failures=**0** |
| Near-field NPFG (`build_verify_nearfield_npfg_closed_loop_v2.sh`) | cases=3029, failures=**0** |
| `MUMT_SimEditor` `-ForceUnity` build | exit=0, errors=0, warnings=0 |

The 13 operational scenarios include the Phase F additions:
`RejectedFormationKeepsLegacyControl` (positive `InnerLoopAutopilot` write-count evidence that the legacy
writer keeps advancing through rejected commands, follower stays airborne) and
`RejectedPacketWhileActiveKeepsFormationOwnership` (ownership kept, no re-prime, no legacy re-entry, producer
keeps advancing). Phase F was additionally confirmed by a live `-game` reproduction: through stale /
missing-leader / non-finite commands the follower's legacy `[Inner]` guidance kept advancing and it stayed
airborne; a valid enable took over (legacy froze); a replay while Active kept ownership; and disable resumed
legacy.

## Performance tuning still open (not blocking correctness)

These are quality/tuning items, not safety or contract items — they do not affect the ownership, prime, or
isolation guarantees above:

- NPFG / TECS / F16StickAdapterV2 gains and limits are set for the pinned F-16 sweep policy, not tuned across
  the full flight regime; slot-tracking tightness and settling behavior are functional but not optimized.
- Leader-loss handling in the legacy path is a level-hold, not an optimized re-join.
- Formation entry transients (Warming→Active) are bounded and logged but not minimized.

## NOT guaranteed at this baseline

The following are explicitly out of scope and **not** proven by any gate here:

- **Stability across the full F-16 flight envelope.** Verified regimes are those exercised by the closed-loop
  and sweep harnesses, not every airspeed / altitude / load-factor combination.
- **Full proof of TECS bumpless continuity.** The exact-zero first consume is the stick latch's; the
  active-frame TECS deltas are logged, never required to be zero.
- **Fail-safe under sustained communications loss.** A held command is rejected as stale on the enabling
  edge, but there is no long-duration comms-loss fail-safe policy.
- **Multi-follower collision avoidance.** Per-aircraft isolation is proven; inter-follower deconfliction is
  not implemented.
- **Real-hardware flight validation.** All results are from the JSBSim-in-UE simulation, not a real airframe.

## Reproduce

```bash
# operational + candidate + prime + arbiter + ownership (each: single UnrealEditor instance, run in order)
bash Tools/planner_v2/run_formation_operational_v2.sh
bash Tools/planner_v2/run_formation_candidate_integration_v2.sh
bash Tools/planner_v2/run_command_prime_v2.sh
bash Tools/planner_v2/run_command_arbiter_v2.sh
bash Tools/planner_v2/run_command_ownership_telemetry.sh
# NPFG standalone harnesses (rc=0 + failures=0)
bash Tools/planner_v2/build_verify_npfg_f16_lateral_closed_loop_v2.sh
bash Tools/planner_v2/build_verify_npfg_caller_contract_v2.sh
bash Tools/planner_v2/build_verify_nearfield_npfg_closed_loop_v2.sh
# clean build
"$HOME/unreal/Engine/Build/BatchFiles/Linux/Build.sh" MUMT_SimEditor Linux Development \
  -project="$PWD/MUMT_Sim.uproject" -ForceUnity
```
