# BVRGym Autopilot (the outer-loop / low-level flight controller)

> Scope: the C++ autopilot that turns a navigation setpoint (heading / altitude / speed) into
> control-surface and throttle commands (aileron / elevator / throttle) that the JSBSim flight model flies.
> Confirmed from source: `BVRGymAutopilot.h` / `BVRGymAutopilot.cpp` and its call site in
> `UDPControlReceiver.cpp` (`ApplyAutopilotToPawn`).
>
> - Date: 2026-06-28 (git `25c5459`; the hard-turn hysteresis re-arm clamp in `BVRGymAutopilot.cpp` is an
>   uncommitted working-tree change)
> - Related: [README_UDP_Comms.md](README_UDP_Comms.md) (setpoint input), [README_F16_Actor.md](README_F16_Actor.md), [README_JSBSim.md](README_JSBSim.md)

---

## 0. TL;DR & where it lives

- **What it is:** a small, self-contained C++ PID autopilot **ported from the Python BVRGym project**
  (`autopilot.py`, `control.py`, `navigation.py`, `f16_config.py`). It is the controller that actually "flies the
  stick": given a target heading/altitude/speed, it outputs normalized aileron/elevator commands plus a throttle
  command from its speed-hold (autothrottle) loop.
- **Source files (this is the location):**
  - **`Source/MUMT_Sim/Public/BVRGymAutopilot.h`** — declarations (`FPID`, `FAutopilotNavParams`, `FAutopilotOutput`, `FAircraftAutopilot`)
  - **`Source/MUMT_Sim/Private/BVRGymAutopilot.cpp`** — the control-law implementation
  - Part of the **`MUMT_Sim` runtime module** (game C++, not a plugin).
- **Runtime location:** it is **not** a standalone Actor or Component. It exists only as **member objects**
  inside **`AUDPControlReceiver`** (the level Actor that listens on UDP). So at runtime the autopilot lives
  *inside the UDP receiver actor*, sitting between the JSON setpoint input (port 5010) and each aircraft's
  `UJSBSimMovementComponent`. There is **one `FAircraftAutopilot` instance per UAV**, held in
  `TMap<FString, FAircraftAutopilot> Autopilots` keyed by aircraft name (`FindOrAdd` on first setpoint), so
  every UAV keeps its **own** PID / hysteresis state and they never interfere.

```
  UDP 5010 setpoint ─► AUDPControlReceiver { TMap<name,FUavSetpoint>       Setpoints,  ─► UJSBSimMovementComponent.Commands
   {aircraft_name,...}                       TMap<name,FAircraftAutopilot> Autopilots }    (per matched pawn)
                          (60 Hz AutopilotTick — loops over every setpoint, drives each named pawn)
```

---

## 1. Where it sits in the control stack (answering "low-level controller location")

There are **three** control layers. The BVRGym autopilot is the **middle** one:

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 1 — Mission / high-level   "what to do"                                   │
│   Heading / Altitude / Throttle SETPOINT                                        │
│   source today: UDP 5010 binary packet (from ROS bridge or a script),          │
│                 or the debug UPROPERTYs. (Target diagram: the BT / Mission      │
│                 Autonomy would produce this.)                                   │
└───────────────────────────────┬────────────────────────────────────────────────┘
                                 ▼ ActiveHeadingDeg / ActiveAltitudeM / ActiveThrottle
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 2 — BVRGym Autopilot  ◄── THIS DOCUMENT  (outer-loop, "fly the stick")    │
│   FAircraftAutopilot::GetControlInput(diffHeading, diffAlt, phi, theta,          │
│                                       curSpeed, tgtSpeed)                        │
│     → {AileronCmd, ElevatorCmd, 0, ThrottleCmd}   (normalized; throttle 0..1)    │
└───────────────────────────────┬────────────────────────────────────────────────┘
                                 ▼ JSBSim->Commands.Aileron/Elevator + EngineCommands[0].Throttle
┌──────────────────────────────────────────────────────────────────────────────┐
│ LAYER 3 — JSBSim FCS (inner loop, inside the plugin)   "move the surfaces"      │
│   FGFCS channels: actuators / gains / filters → actual surface deflections,     │
│   then aero forces. (See README_JSBSim.md)                                      │
└──────────────────────────────────────────────────────────────────────────────┘
```

So:
- **Relative to the mission/BT layer**, this IS the low-level controller — it's the thing that converts a navigation
  goal into raw stick commands.
- **Relative to JSBSim's FCS**, it is an *outer loop* — it commands normalized aileron/elevator, and JSBSim's internal
  FCS is the truly innermost surface controller.

In the target architecture this autopilot is effectively the **"Action-level Autonomy"** block — already implemented,
but embedded in UE C++ rather than provided as an open-source ROS action layer.

---

## 2. Types (structure)

| Type | Kind | Purpose |
|---|---|---|
| `FPID` | `USTRUCT(BlueprintType)` | One PID channel: `Kp,Ki,Kd,IntegMin,IntegMax` + runtime `Derivator/Integrator`. `Update()`/`Reset()`/`SetGains()` |
| `FAutopilotNavParams` | `USTRUCT(BlueprintType)` | Navigation/mode thresholds & limits (act-spaces, RollMax, TanRef, dive/climb limits, …) |
| `FAutopilotOutput` | plain struct | `{Aileron, Elevator, Rudder, Throttle}` (Rudder always 0; Throttle `<0` = speed-hold off) |
| `FAircraftAutopilot` | plain C++ class (`MUMT_SIM_API`) | Holds the 4 PIDs (Roll/RollSec/Pitch/Throttle) + NavParams; `GetControlInput()` is the entry point |

Because `FPID`/`FAutopilotNavParams` are `BlueprintType` USTRUCTs, the gains are exposed as `UPROPERTY` configs on
`AUDPControlReceiver` and can be **edited live in the Details panel (PIE)** without recompiling.

---

## 3. The control law in detail

### 3.1 `FPID::Update` — the PID convention (BVRGym style)
```cpp
float FPID::Update(float CurrentValue) {
    const float Error = 0.f - CurrentValue;                 // SetPoint is ALWAYS 0
    const float P     = Kp * Error;
    const float D     = Kd * (Error - Derivator);  Derivator = Error;   // derivative of error (per-tick, no dt)
    Integrator        = Clamp(Integrator + Error, IntegMin, IntegMax);  // anti-windup clamp
    const float I     = Ki * Integrator;
    return P + I + D;
}
```
Notes that matter:
- **Setpoint is hardwired to 0**; the caller passes the *error signal* as `CurrentValue` (e.g. `RollRef - Phi`). The
  controller regulates that error to zero.
- **No `dt`** — gains are tuned per **60 Hz tick** (the rate the autopilot runs). Changing the tick rate changes the
  effective tuning.
- **`Ki = 0` for Roll/RollSec/Pitch** → those channels are effectively **PD controllers** with an anti-windup clamp
  kept ready. The new **`ThrottlePID` uses `Ki = 0.004`** (it is a real PI — its integrator carries the trim throttle).

**`FPID::SetGains(Cfg)`** copies only the gains (`Kp,Ki,Kd,IntegMin,IntegMax`), **resets `Derivator`, and deliberately
*preserves* `Integrator`**. `ApplyAutopilotToPawn` calls it on all four PIDs every tick to pick up live Details-panel
edits. This fixes a real bug: the old code copied the whole `FPID` struct each tick, which **zeroed the integrator
every frame** — so the autothrottle could never build its steady-state trim and was stuck P-only (throttle pinned
≈0.46, unable to hold speed). Roll/Pitch are `Ki=0` so resetting their integrators is harmless.

### 3.2 `GetControlInput` — two-mode outer loop + speed-hold
Inputs: `DiffHeadDeg` (signed heading error, [-180,180]), `DiffAltM` (target − current altitude, m), `CurrentPhiDeg`
(roll), `CurrentThetaDeg` (pitch), `CurrentSpeedMps`, `TargetSpeedMps` (last two default 0). Returns
`{Aileron, Elevator, 0, Throttle}`. The **lateral/vertical** law picks a mode from the heading/altitude error
magnitudes; the **speed-hold** (§3.5) then runs unconditionally and fills in `Throttle`.

**Hard-turn mode** — `|DiffHead| ≥ HeadActSpace` AND `|DiffAlt| ≤ AltActSpace`:
- Commit to a max-bank turn toward the target heading:
  `BankRef = sign(DiffHead) · RollMax (80°)` → `SetRollPID(BankRef)`.
- Hold altitude during the turn: `SetPitchPID(atan2(DiffAlt, TanRef))`.
- Hysteresis: widen `AltActSpace→max (2000)`, then **re-arm the stay-in-turn threshold to
  `HeadActSpace = min(HeadActSpaceMin, HeadActSpaceMax)`**. The stay-in threshold must be **≤** the enter
  threshold (`HeadActSpaceMax`); the `min(…)` clamp guarantees that even if a level's `NavParams` override sets
  `HeadActSpaceMin > Max` (which would invert the hysteresis, flicker hard-turn off every tick, and leave the
  aircraft never turning — the "UAV won't turn at corners" bug on the RL_2 map). **This clamp is currently an
  uncommitted working-tree change.**

**Precision mode** — otherwise (wings level, control altitude with pitch):
- `ThetaRef = clamp(atan2(DiffAlt, TanRef), DiveThetaMax −45°, ClimbThetaMax +45°)`, `SetRollPID(0)` (level wings),
  `SetPitchPID(ThetaRef)`.
- Three sub-states adjust `ThetaActSpace` for chatter-free behavior: **PREC-CLIMB** (`|DiffAlt|>1500`),
  **PREC-SETTLE** (`|DiffAlt|<1500` and `|Theta|>ThetaActSpace`), **PREC-LEVEL** (else).
- Hysteresis: widen `HeadActSpace→max (35)`.

The `*ActSpace` members (`AltActSpace`, `HeadActSpace`, `ThetaActSpace`) are **stateful hysteresis thresholds**
(min↔max) so the autopilot doesn't oscillate between hard-turn and precision modes.

### 3.3 Roll / pitch helpers
```cpp
SetRollPID(RollRef, bUseSecondary=false, Phi):
    Diff = RollCircleClip(clamp(RollRef,±180) − Phi)        // shortest angular error
    AileronCmd = clamp(−RollPID.Update(Diff), −1, 1)        // outer minus cancels the PID's internal sign

SetPitchPID(ThetaRef, Theta):
    ElevatorCmd = clamp(PitchPID.Update(clamp(ThetaRef,±90) − Theta), −1, 1)
```
- `bUseSecondary` would select `RollSecPID`, but it is **always called with `false`** → `RollSecPID` is currently
  **unused** (dead, but tunable).

### 3.4 `SetThrottlePID` — autothrottle / speed-hold (NEW)
```cpp
SetThrottlePID(CurrentSpeedMps, TargetSpeedMps):
    if (TargetSpeedMps <= 0)  { ThrottleCmd = -1; return; }      // speed-hold OFF → caller uses open-loop throttle
    ThrottleCmd = clamp(ThrottlePID.Update(CurrentSpeedMps − TargetSpeedMps), 0, 1)
```
- Runs **every tick, in both hard-turn and precision mode** (it is energy management, not a mode) — it is the last
  thing `GetControlInput` does before returning.
- **Sign trick:** `FPID::Update` computes `error = 0 − CurrentValue`, so passing `(V − Vtarget)` yields
  `error = (Vtarget − V)`. Slower than target → positive error → throttle **up**; faster → throttle **down**.
- `ThrottlePID = {0.02, 0.004, 0, 0, 250}` is a **PI** (no D). The integrator (clamped `[0, 250]`) supplies the
  steady-state trim: `Ki · IntegMax = 0.004 · 250 = 1.0`, i.e. it can drive throttle across the full `[0,1]` range.
  This is why `SetGains` must preserve the integrator (§3.1).
- **Disabled path:** `TargetSpeedMps ≤ 0` → `Throttle = −1`, signalling the caller to fall back to the open-loop
  `Setpoint.Throttle` (backward compatible with setpoints that don't request a speed).

### 3.5 Angle utilities
- `DeltaHeading(target, current)` = `((target−current+180) mod 360) − 180` → shortest signed heading error, [-180,180].
  Called by `UDPControlReceiver` to build `DiffHead`.
- `RollCircleClip(D)` wraps a roll difference into (−180, 180].

---

## 4. Default gains (from `f16_config.py`)

Set both in the `FAircraftAutopilot` constructor and (authoritatively at runtime) by `AUDPControlReceiver`'s configs:

| Channel / param | Kp | Ki | Kd | Integ min/max |
|---|---|---|---|---|
| `RollPID` | 0.01 | 0 | 0.9 | −0.2 / 0.2 |
| `RollSecPID` (unused) | 0.2 | 0 | 0.2 | −1.0 / 1.0 |
| `PitchPID` | 0.3 | 0 | 1.0 | −1.0 / 1.0 |
| `ThrottlePID` (autothrottle, NEW) | 0.02 | 0.004 | 0 | 0 / 250 |

| NavParam | Value | Used? |
|---|---|---|
| `AltActSpaceMin / Max` | 1000 / 2000 m | ✅ |
| `HeadActSpaceMin / Max` | 10 / 35° | ✅ |
| `ThetaActSpaceMin / Max` | 10 / 30° | ✅ |
| `RollMax` | 80° | ✅ (hard-turn bank) |
| `TanRef` | 2000 | ✅ (altitude→pitch slope) |
| `DiveThetaMax / ClimbThetaMax` | −45 / +45° | ✅ (pitch clamp) |
| `BankGain` | 0.8 | ❌ **declared but never referenced** in the control law |

---

## 5. How it is wired into `AUDPControlReceiver`

```
AUDPControlReceiver  (level Actor, owns: TMap<FString,FUavSetpoint>       Setpoints
                                          TMap<FString,FAircraftAutopilot> Autopilots)
│
├─ BeginPlay():  start a 60 Hz timer → AutopilotTick.  (Autopilots are created lazily, per UAV.)
│
├─ ReceiveSetpointData()  (each Tick, drains UDP 5010 JSON):
│      parses {aircraft_name, heading_deg, altitude_m, throttle_norm, target_speed_mps, launch_missile}
│      → Setpoints.FindOrAdd(aircraft_name).
│
└─ AutopilotTick()  (60 Hz):
       if bUseDebugSetpoint → inject a setpoint for CachedTargetPawn from DebugTarget* UPROPERTYs;
       if (Setpoints.Num() == 0) return;
       for each (name → setpoint):
         resolve the pawn (exact name match, else unique substring; ambiguous → warn & skip);
         └─ ApplyAutopilotToPawn(Pawn, name, setpoint):
              FAircraftAutopilot& AP = Autopilots.FindOrAdd(name);  // this UAV's own PID/hysteresis state
              AP.RollPID/RollSecPID/PitchPID/ThrottlePID.SetGains(...Config);   // gains only, keep integrators
              AP.NavParams = NavParams;
              S = JSBSim->AircraftState;
              AltM     = Pawn->GetActorLocation().Z / 100;            // UE world-Z (cm→m), NOT JSBSim ASL
              SpeedMps = S.TotalVelocityKts * 0.514444;
              DiffHead = DeltaHeading(setpoint.HeadingDeg, S.Yaw);    // compass error
              DiffAlt  = setpoint.AltitudeM − AltM;
              Out = AP.GetControlInput(DiffHead, DiffAlt, S.Roll, S.Pitch, SpeedMps, setpoint.TargetSpeedMps);
              ThrottleOut = (Out.Throttle >= 0) ? Out.Throttle : setpoint.Throttle;   // autothrottle else open-loop
              JSBSim->Commands.{Aileron,Elevator,Rudder} = Out.*;    // Rudder 0
              JSBSim->EngineCommands[0].Throttle = ThrottleOut;
              AutopilotAileron/Elevator = Out.*;                     // cached for HUD/Blueprint read
```

Key wiring facts:
- **One autopilot per UAV** — `Autopilots` is a `TMap` keyed by aircraft name; each named pawn gets its own
  PID/hysteresis/integrator state, so multiple UAVs never share controller state. (See [README_UDP_Comms.md]
  for the name-based addressing.)
- **Frames the autopilot sees:** *heading* is **compass** (`DiffHead` from `DeltaHeading`, fed a compass setpoint that
  the BT builds upstream as `atan2(Δx, −Δy)`); *altitude* uses **UE world `Location.Z/100`**, **not** JSBSim ASL —
  the same `z` value `BuildPawnState` publishes and the BT computes setpoints against. (Using JSBSim ASL here made the
  controller sit at `setpoint + origin-altitude offset`, so the BT never saw the target reached.)
- **Throttle:** when speed-hold is active (`Out.Throttle ≥ 0`) the **autothrottle output** is used; otherwise the
  **open-loop `Setpoint.Throttle`** is forwarded (backward compatible with speed-less setpoints).
- **State feedback** (Phi/Theta/Psi/speed) is read from the target pawn's JSBSim `AircraftState`.
- **Output is written directly** into `JSBSim->Commands` / `EngineCommands[0]` — it bypasses the pawn's `UDP_*`
  Blueprint variables (unlike the JSON 5005 path). The JSBSim plugin then pushes them into the FDM on its own tick.
- Runs at **60 Hz** (its own timer), independent of both the render frame rate and JSBSim's substep.

---

## 6. Notes / gaps / caveats

- **Roll/Pitch are PD only** — `Ki=0` on those channels; the integral path exists but is inert. **`ThrottlePID` is a
  real PI** (`Ki=0.004`) — its integrator carries the trim throttle, so `SetGains` preserves it (§3.1).
- **No `dt` term** — tuning is implicitly tied to the 60 Hz tick.
- **`RollSecPID` unused** (`bUseSecondary` is never true) and **`BankGain` unused** — both are tunable but dead.
- **Rudder always 0** — no yaw/coordination control; turns are bank-only.
- **Autothrottle is opt-in** — speed-hold only engages when the setpoint carries `target_speed_mps > 0`; otherwise the
  open-loop `throttle_norm` is forwarded. The BT requests speed via the `AircraftSetpoint.target_speed_mps` field
  (see [README_ROS_Bridge.md](README_ROS_Bridge.md)).
- **Hysteresis re-arm clamp is uncommitted** — `HeadActSpace = min(HeadActSpaceMin, HeadActSpaceMax)` in the hard-turn
  branch lives only in the working tree (not yet committed); without it a `Min>Max` NavParams override (RL_2) inverts
  the turn hysteresis and the UAV won't turn.
- **Multi-UAV** — `AutopilotTick` loops over every setpoint and drives each named pawn with its own autopilot. Running
  the JSON 5005 manual path and the 5010 autopilot path on the *same* pawn would still make them fight.

---

## 7. Relationship to the target architecture

This autopilot is the concrete **Action-level Autonomy** the target diagram wants — but it lives in UE C++ and is fed
by raw UDP, not by a ROS action interface. With the autothrottle in place it now accepts a full
`(heading, altitude, speed)` setpoint, which the BT supplies via `AircraftSetpoint`. The remaining integration step
(P1 in [ARCHITECTURE.md](ARCHITECTURE.md)) is to expose it as a `MoveTo(heading, altitude, speed)`-style ROS action that
a BT leaf maps onto, with a single source of truth so two autopilots never fight.

---

## 8. Key file paths

| Item | Path |
|---|---|
| Autopilot declarations | `Source/MUMT_Sim/Public/BVRGymAutopilot.h` |
| Autopilot implementation | `Source/MUMT_Sim/Private/BVRGymAutopilot.cpp` |
| Only owner / driver | `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` (`TMap Autopilots`, `AutopilotTick`, `ApplyAutopilotToPawn`) |
| Gain configs (UPROPERTY) | `Source/MUMT_Sim/Public/UDPControlReceiver.h` (`RollPIDConfig`, `RollSecPIDConfig`, `PitchPIDConfig`, `ThrottlePIDConfig`, `NavParams`) |
| Flight model it commands | `Plugins/JSBSimFlightDynamicsModel/.../JSBSimMovementComponent.{h,cpp}` |
| Upstream origin | Python BVRGym: `autopilot.py`, `control.py`, `navigation.py`, `f16_config.py` |
