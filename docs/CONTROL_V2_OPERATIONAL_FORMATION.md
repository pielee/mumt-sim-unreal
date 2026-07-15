# Operational Formation mode (Phase D)

Phase C proved the real producer works when a test calls `BeginHandoff()`. Phase D connects the **real
operational command path** to it: a UDP command on port 5010 is parsed by the production
`AUDPControlReceiver`, routed to a per-aircraft `UFormationRuntimeOwnerV2`, which drives the
NPFG/TECS/stick producer through the Phase B/C Prime/handoff contract into the command arbiter.

Nothing auto-activates. Formation begins only on an explicit external `control_mode: formation` command.
With no such command the aircraft is bit-for-bit transparent Legacy, exactly as Phase A.

## Protocol — backward-compatible, minimal

The 5010 setpoint JSON already had `leader_name` and `slot_front_m` / `slot_right_m` / `slot_up_m`, so
those are **reused**. Three fields are **added**, all optional (absent → the old behavior):

| field | meaning |
| --- | --- |
| `control_mode` | `"legacy"` (default) or `"formation"`. Only `"formation"` engages ControlV2; unknown/absent → Legacy. This is **separate** from the existing `guidance_mode` (which still drives the legacy inner-loop formation). |
| `command_sequence` | monotonic per aircraft. `<=` the applied sequence is a replay and is refused, never re-executed. |
| `command_timestamp` | sender seconds. Too old on the enabling edge → the command is stale and refused. |

An old sender that omits these is completely unaffected — the same default-preserving pattern the existing
`guidance_mode` / `gun_firing` fields use. The receiver only parses and **routes**; no planner/prime/arbiter
logic lives in it.

## The runtime owner — a per-aircraft component

`UFormationRuntimeOwnerV2` is a `UActorComponent` on the follower aircraft, created lazily on the first
Formation request. It was chosen over a world manager or a map in the receiver for two reasons:

- **Tick ordering.** The producer must read the JSBSim snapshot *after* the movement component refreshes it
  (`CopyFromJSBSim`) and submit a candidate *before* the next consume (`CopyToJSBSim`). The component pins
  this with `AddTickPrerequisiteComponent(movement)` — no change to the JSBSim plugin, no reliance on
  registration order. A 60 Hz world timer could not guarantee it.
- **Cleanup.** Owned by the follower actor, it dies with the actor, so world teardown, actor destruction,
  aircraft removal and PIE end clean it and its producer up automatically. On disable / Falling / leader
  loss it stays but goes idle after an immediate Legacy fallback.

It owns, per aircraft: weak follower/leader/world identity, the producer, the requested/active mode, the
leader label, the slot, the applied sequence, the command timestamp, the last follower sim time, the last
producer result, the prime generation, the baseline consume sequence, the candidate generation, and the
fallback reason. All mutation is game-thread (`check(IsInGameThread())`).

### Cadence

The owner drives the producer **once per genuine sim-time advance**, with `dt = current follower
SimTimeSec - previous`. A non-finite / non-positive / too-large dt, a hold, or a reset is not fed to the
controllers; the producer never runs twice on the same `SimTimeSec`.

### State machine

`Idle → Warming` (shadow the chain so the first real candidate is valid) `→ Priming` (`BeginHandoff`)
`→ AwaitingActivation` (the arbiter confirms at the consume boundary) `→ Active` (a fresh candidate each
advancing frame). The exact-zero first consume is the stick latch's, unchanged from Phase B/C.

## Command semantics

- **Rising edge** (Legacy → Formation): validate identity / snapshot / health / timestamp / sequence, apply
  the slot, warm, prime, activate. Mode stays Legacy until the boundary confirms.
- **Repeated enable** (same sequence): idempotent — no re-prime, no re-handshake.
- **Slot update** (new sequence, same leader): `SetSlot` on the live producer, **no re-prime**.
- **Leader change**: immediate Legacy fallback + producer reset, then a fresh handshake against the new
  leader. The old ticket cannot be reused.
- **Disable** (`control_mode: legacy`): immediate Legacy fallback, producer reset, next consume is Legacy.
  No blend.
- **Safety fallback**: Falling / Crashed / UnknownHealth / leader lost / identity change / world mismatch /
  sim-time discontinuity / stale command all drop to Legacy immediately. Falling always outranks a handoff,
  and its throttle-0 / cutoff block reaches the FDM before any candidate. Any fallback requires a **new**
  explicit enable to resume — a held command never silently resumes Formation.

## The scripted `-FormationTest` guard

The headless `-FormationTest` flight harness rewrites the follower's setpoint every frame. A one-line guard
makes it skip a follower that already carries an operational `control_mode: formation` command, so
`RouteControlV2` can drive it. This affects only the test harness — production never passes `-FormationTest`.

## Rejected commands never suppress the legacy writer (Phase F)

`RouteControlV2`'s bool return means **"skip the legacy inner-loop writer this frame"** — it must NOT mean
"the packet said `control_mode: formation`". The two are different: a formation packet can be *rejected* (no
leader / stale / replayed / out-of-range slot). Returning `true` for every formation packet suppressed the
legacy writer even when ControlV2 declined the command, leaving the follower with no active controller (it
flew a held command into the ground). The return is now:

```
return bAccepted || bRuntimeOwnsOrPending;   // NOT: return control_mode == formation
```

- **Rejected from Idle** → `false`: the owner stays Idle / `!IsFormationRequested`, so the legacy guidance
  keeps running. The follower is never left un-commanded.
- **Rejected while pending/active** → `true`: `IsFormationRequested()` (or a Warming/Priming/AwaitingActivation/
  Active phase) still holds, so one bad packet cannot drop a live handoff or let the legacy writer re-enter
  under an active ControlV2. The bad packet's values are not applied and there is no re-prime.
- **Accepted** (valid enable / idempotent hold / slot update / leader change) → `true`, exactly as before.
- **Explicit disable** → the `control_mode != formation` branch resets the owner and returns `false`: the
  legacy writer runs the same frame.

## What is proven, and what is NOT

Proven, by `Tools/planner_v2/run_formation_operational_v2.sh` (13 scenarios, real UDP 5010, real producer,
live leader + follower):

- an old packet stays Legacy (backward-compat);
- an operational enable gives an **exact-zero** first consume on all five controlled fields, with a real
  candidate;
- the real producer keeps moving the controls (measured elevator/throttle to ~1.0), candidate generation
  advancing — not a fake that re-emits the baseline;
- repeated enable idempotent; slot update without re-prime; leader change forces a fresh handshake; disable
  is an immediate Legacy fallback (and the legacy **writer** provably re-advances); stale / replayed /
  non-finite / no-leader commands are refused;
  Falling preempts; per-aircraft isolation; world teardown leaves no runtime state;
- **rejected formation commands never suppress the legacy writer**: while a series of rejected commands is in
  flight the legacy inner-loop writer keeps advancing (positive `InnerLoopAutopilot` write-count evidence) and
  the follower stays airborne; and a rejected packet while Active keeps ControlV2 ownership (no re-prime, no
  legacy re-entry, the bad values never applied) while the producer keeps advancing.

**NOT** proven, and not claimed:

- **TECS bumpless continuity.** The exact-zero is the stick latch's; the active-frame deltas are logged,
  never required to be zero.
- **No production trigger beyond the explicit command.** There is no auto-activation; the default mode
  stays Legacy, and no direct JSBSim surface write or tuning change exists.

## Run it

```bash
bash Tools/planner_v2/run_formation_operational_v2.sh   # COMMAND_FORMATION_OPERATIONAL_V2_RESULT=PASS
```
