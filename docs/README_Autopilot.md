# BVRGym Autopilot (the outer-loop / low-level flight controller)

> Scope: the C++ autopilot that turns a navigation setpoint (heading / altitude / throttle) into
> control-surface commands (aileron / elevator) that the JSBSim flight model flies.
> Confirmed from source: `BVRGymAutopilot.h` / `BVRGymAutopilot.cpp` and its single call site in
> `UDPControlReceiver.cpp`.
>
> - Date: 2026-06-23 (git `7632f63`)
> - Related: [README_UDP_Comms.md](README_UDP_Comms.md) (setpoint input), [README_F16_Actor.md](README_F16_Actor.md), [README_JSBSim.md](README_JSBSim.md)

---

## 0. TL;DR & where it lives

- **What it is:** a small, self-contained C++ PID autopilot **ported from the Python BVRGym project**
  (`autopilot.py`, `control.py`, `navigation.py`, `f16_config.py`). It is the controller that actually "flies the
  stick": given a target heading/altitude, it outputs normalized aileron/elevator commands.
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
│   FAircraftAutopilot::GetControlInput(diffHeading, diffAlt, phi, theta)         │
│     → AileronCmd, ElevatorCmd   (normalized −1..1)                              │
└───────────────────────────────┬────────────────────────────────────────────────┘
                                 ▼ JSBSim->Commands.Aileron/Elevator (+ throttle)
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
| `FPID` | `USTRUCT(BlueprintType)` | One PID channel: `Kp,Ki,Kd,IntegMin,IntegMax` + runtime `Derivator/Integrator`. `Update()`/`Reset()` |
| `FAutopilotNavParams` | `USTRUCT(BlueprintType)` | Navigation/mode thresholds & limits (act-spaces, RollMax, TanRef, dive/climb limits, …) |
| `FAutopilotOutput` | plain struct | `{Aileron, Elevator, Rudder}` (Rudder always 0) |
| `FAircraftAutopilot` | plain C++ class (`MUMT_SIM_API`) | Holds the 3 PIDs + NavParams; `GetControlInput()` is the entry point |

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
- **`Ki = 0` in every default config** → the integral term is currently inert; effectively a **PD controller** with
  an anti-windup clamp kept ready.

### 3.2 `GetControlInput` — two-mode outer loop
Inputs: `DiffHeadDeg` (signed heading error, [-180,180]), `DiffAltM` (target − current altitude, m), `CurrentPhiDeg`
(roll), `CurrentThetaDeg` (pitch). Picks a mode from error magnitudes:

**Hard-turn mode** — `|DiffHead| ≥ HeadActSpace` AND `|DiffAlt| ≤ AltActSpace`:
- Commit to a max-bank turn toward the target heading:
  `BankRef = sign(DiffHead) · RollMax (80°)` → `SetRollPID(BankRef)`.
- Hold altitude during the turn: `SetPitchPID(atan2(DiffAlt, TanRef))`.
- Hysteresis: widen `AltActSpace→max (2000)`, tighten `HeadActSpace→min (10)`.

**Precision mode** — otherwise (wings level, control altitude with pitch):
- `ThetaRef = clamp(atan2(DiffAlt, TanRef), DiveThetaMax −45°, ClimbThetaMax +45°)`, `SetRollPID(0)` (level wings),
  `SetPitchPID(ThetaRef)`.
- Three sub-states adjust `ThetaActSpace` for chatter-free behavior: **PREC-CLIMB** (`|DiffAlt|>1500`),
  **PREC-SETTLE** (`|DiffAlt|<1500` and `|Theta|>ThetaActSpace`), **PREC-LEVEL** (else).
- Hysteresis: widen `HeadActSpace→max (35)`.

The `*ActSpace` members (`AltActSpace`, `HeadActSpace`, `ThetaActSpace`) are **stateful hysteresis thresholds**
(min↔max) so the autopilot doesn't oscillate between hard-turn and precision modes. Returns `{Aileron, Elevator, 0}`.

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

### 3.4 Angle utilities
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
AUDPControlReceiver  (level Actor, owns: FAircraftAutopilot Autopilot)
│
├─ BeginPlay():  seed Autopilot.RollPID/RollSecPID/PitchPID/NavParams from the receiver's UPROPERTY configs;
│                start a 60 Hz timer → AutopilotTick.
│
├─ ReceiveSetpointData()  (each Tick, drains UDP 5010):
│      sets ActiveHeadingDeg / ActiveAltitudeM / ActiveThrottle (newest seq);
│      reset byte → Autopilot = FAircraftAutopilot() (re-seeded with configs);
│      sets bSetpointReceived = true.
│
└─ AutopilotTick()  (60 Hz):
       if bUseDebugSetpoint → use DebugTarget* UPROPERTYs instead of UDP;
       re-sync gains from configs (so live Details-panel edits take effect);
       if (!bSetpointReceived && !bUseDebugSetpoint) return;        // inactive until first setpoint
       Pawn = CachedTargetPawn (primary F16_UAV);
       └─ ApplyAutopilotToPawn(Pawn):
            JSBSim = Pawn->FindComponentByClass<UJSBSimMovementComponent>();
            S = JSBSim->AircraftState;
            DiffHead = DeltaHeading(ActiveHeadingDeg, S.Yaw);
            DiffAlt  = ActiveAltitudeM − S.AltitudeASLFt*0.3048;
            Out = Autopilot.GetControlInput(DiffHead, DiffAlt, S.Roll, S.Pitch);
            JSBSim->Commands.Aileron  = Out.Aileron;
            JSBSim->Commands.Elevator = Out.Elevator;
            JSBSim->Commands.Rudder   = Out.Rudder;          // 0
            JSBSim->EngineCommands[0].Throttle = ActiveThrottle;
            AutopilotAileron/Elevator = Out.*;               // cached for HUD/Blueprint read
```

Key wiring facts:
- **Inputs come from layer 1** (UDP 5010 setpoint or debug UPROPERTYs), throttle is **pass-through** (the autopilot
  does not compute throttle — it forwards `ActiveThrottle` straight to engine 0).
- **State feedback** (Phi/Theta/Psi/AltitudeASL) is read from the target pawn's JSBSim `AircraftState`.
- **Output is written directly** into `JSBSim->Commands` — it bypasses the pawn's `UDP_*` Blueprint variables (unlike
  the JSON 5005 path). The JSBSim plugin then pushes `Commands` into the FDM on its own tick.
- Runs at **60 Hz** (its own timer), independent of both the render frame rate and JSBSim's 120 Hz substep.

---

## 6. Notes / gaps / caveats

- **PD only today** — `Ki=0` everywhere; the integral path exists but is inert.
- **No `dt` term** — tuning is implicitly tied to the 60 Hz tick.
- **`RollSecPID` unused** (`bUseSecondary` is never true) and **`BankGain` unused** — both are tunable but dead.
- **Rudder always 0** — no yaw/coordination control; turns are bank-only.
- **Throttle is open-loop pass-through** — no speed hold (contrast with the ROS `bt_controller.yaml`, which *designs* a
  speed-scheduled throttle but isn't implemented).
- **Single pawn** — the autopilot drives only the primary controlled UAV; the JSON 5005 path is what handles multiple
  UAVs. Running both on the same pawn makes them fight.
- **Reset** (`Autopilot = FAircraftAutopilot()`) can only be triggered by the UDP setpoint reset byte, which the ROS
  bridge hardcodes to 0 — so a ROS-driven reset can't reach it (see [README_ROS_Bridge.md](README_ROS_Bridge.md)).

---

## 7. Relationship to the target architecture

This autopilot is the concrete **Action-level Autonomy** the target diagram wants — but it lives in UE C++ and is fed
by raw UDP, not by a ROS action interface. The clean integration path (P1 in [ARCHITECTURE.md](ARCHITECTURE.md)) is to
expose it as a `MoveTo(heading, altitude, speed)`-style action that a BT leaf maps onto, and to decide one source of
truth versus the (currently unimplemented) ROS-side controller in `bt_controller.yaml` so two autopilots never fight.

---

## 8. Key file paths

| Item | Path |
|---|---|
| Autopilot declarations | `Source/MUMT_Sim/Public/BVRGymAutopilot.h` |
| Autopilot implementation | `Source/MUMT_Sim/Private/BVRGymAutopilot.cpp` |
| Only owner / driver | `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` (member `Autopilot`, `AutopilotTick`, `ApplyAutopilotToPawn`) |
| Gain configs (UPROPERTY) | `Source/MUMT_Sim/Public/UDPControlReceiver.h` (`RollPIDConfig`, `RollSecPIDConfig`, `PitchPIDConfig`, `NavParams`) |
| Flight model it commands | `Plugins/JSBSimFlightDynamicsModel/.../JSBSimMovementComponent.{h,cpp}` |
| Upstream origin | Python BVRGym: `autopilot.py`, `control.py`, `navigation.py`, `f16_config.py` |
