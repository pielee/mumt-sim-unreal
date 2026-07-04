# JSBSim Plugin Data I/O Reference (full survey)

> Scope: every data path in and out of `Plugins/JSBSimFlightDynamicsModel`, plus every place our
> project code (`Source/MUMT_Sim`, Blueprints) reads or writes that data.
> - Date: 2026-07-02, surveyed at git `25c5459` + working tree
> - Parent docs: [ARCHITECTURE.md](ARCHITECTURE.md), [README_JSBSim.md](README_JSBSim.md) (plugin internals),
>   [README_UDP_Comms.md](README_UDP_Comms.md) (wire formats)
> - Upstream baseline for the diff section: JSBSim-Team/jsbsim commit `a222488` (2025-04-27,
>   `UnrealEngine/Plugins/JSBSimFlightDynamicsModel`), the closest ancestor of our copy.

**Path abbreviations used in citations**

| Abbrev | File |
|---|---|
| `MC.cpp` | `Plugins/JSBSimFlightDynamicsModel/Source/JSBSimFlightDynamicsModel/Private/JSBSimMovementComponent.cpp` |
| `MC.h` | `Plugins/JSBSimFlightDynamicsModel/Source/JSBSimFlightDynamicsModel/Public/JSBSimMovementComponent.h` |
| `FDM.h` | `Plugins/JSBSimFlightDynamicsModel/Source/JSBSimFlightDynamicsModel/Public/FDMTypes.h` |
| `FCS.h` | `Plugins/JSBSimFlightDynamicsModel/Source/ThirdParty/JSBSim/Include/models/FGFCS.h` |
| `UDP.cpp` / `UDP.h` | `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` / `Source/MUMT_Sim/Public/UDPControlReceiver.h` |
| `AP.cpp` / `AP.h` | `Source/MUMT_Sim/Private/BVRGymAutopilot.cpp` / `Source/MUMT_Sim/Public/BVRGymAutopilot.h` |

**Plugin code vs project code.** Everything under `Plugins/JSBSimFlightDynamicsModel` is third-party
plugin code (Epic/JSBSim-Team origin, with 4 local modifications — see §5). Everything under
`Source/MUMT_Sim` and `Content/Blueprints` is our project code.

---

## 1. Overview

```
                    ┌────────────────── external (other repos: ~/dev/mumt_ros_ws, py_bt_ros) ─────────────────┐
                    │  joystick ROS node → UDP JSON :5005      BT/controller → UDP JSON :5010                  │
                    │                     state JSON :5006  ←──────────────────────────────┐                   │
                    └───────────────┬──────────────────────────────┬───────────────────────┼───────────────────┘
                                    ▼                              ▼                       │
        ┌─────────────────────── AUDPControlReceiver (project C++, one actor per level) ───┴────────────────┐
        │  ReceiveUDPData (:5005, per-frame Tick)      ReceiveSetpointData (:5010, per-frame Tick)          │
        │        │ roll/pitch/yaw/throttle                    │ heading/alt/throttle/target_speed          │
        │        ▼ name-matched                               ▼ per-aircraft map                            │
        │  ApplyControlCommandToPawn ─────────┐        AutopilotTick (60 Hz timer)                          │
        │  (also sets BP vars UDP_*)          │               │ FAircraftAutopilot (BVRGym PID port)        │
        │                                     ▼               ▼                                              │
        │                    JSBSim->Commands.{Aileron,Elevator,Rudder}  +  EngineCommands[0].Throttle       │
        │                                                                                                    │
        │  SendStateToPython (20 Hz timer) ← AircraftState.{TotalVelocityKts,LocalEulerAngles}               │
        │                                    + Pawn actor transform (x,y,z UE-world)                         │
        └────────────────────────────────────────────┬───────────────────────────────────────────────────────┘
                                                     │ direct UPROPERTY member writes (no function call)
   F16_UAV / M_F16 pawn Blueprints (project) ────────┤ joystick axes → Commands.*, EngineCommands[0..].*
   UMG_BasicPrimaryFlightDisplay (project HUD) ◄─────┤ reads AircraftState.* every widget tick
                                                     ▼
        ┌────────────────── UJSBSimMovementComponent (plugin, one per aircraft pawn) ────────────────────────┐
        │ TickComponent:                                                                                     │
        │   Commands / EngineCommands ── CopyToJSBSim ──► FGFCS setters ──► property tree (fcs/*-cmd-norm)   │
        │   Exec->Run() × simloops (dt fixed 1/120 s)  ── JSBSim 6-DoF integration                           │
        │   FGPropagate/FGAuxiliary/FGFCS getters ── CopyFromJSBSim ──► AircraftState / EngineStates / Gears │
        │   ECEF → UE world (ESU tangent frame, Yaw−90°) ──► Parent->SetActorLocationAndRotation             │
        │   UE terrain raycast ◄── UEGroundCallback::GetAGLevel (JSBSim asks UE for ground)                  │
        └─────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

Key facts:

- The **only steady-state input surface** is two UPROPERTY structs on the component:
  `Commands` (`FFlightControlCommands`, MC.h:196-197) and `EngineCommands`
  (`TArray<FEngineCommand>`, MC.h:189-190). Both our C++ and our Blueprints write these **by direct
  member assignment**; no setter functions are involved on the UE side.
- The **only steady-state output surface** is `AircraftState` (`FAircraftState`, MC.h:199-200),
  plus the actor transform that the plugin applies itself (MC.cpp:404), plus `EngineStates` /
  `Gears` / `Tanks` arrays (MC.h:176-192).
- `CommandConsole` / `CommandConsoleBatch` / `PropertyManagerNode` / `SetWind` (MC.h:229-252) give raw
  property-tree access, but **no code in this project uses them** — not in C++ (no references under
  `Source/MUMT_Sim`) and not in any Blueprint (no `CommandConsole`/`SetWind` strings in any
  `Content/**/*.uasset` name table).

---

## 2. Input side (UE → JSBSim)

### 2.1 Flight-control commands — `FFlightControlCommands Commands`

Struct definition FDM.h:267-343; all fields `double`, dimensionless normalized commands.
Copied every game frame in `CopyToJSBSim()` (MC.cpp:684-742), which runs once per `TickComponent`
before the FDM step loop (MC.cpp:361). FGFCS setters store into members that FGFCS binds to the
property tree; the property names below are from the FGFCS doc block (FCS.h:123-133) and are consumed
by the F-16 FCS model (`fcs/aileron-cmd-norm` f16.xml:366, `fcs/elevator-cmd-norm` f16.xml:521,
`fcs/rudder-cmd-norm` f16.xml:698 in `Plugins/.../Resources/JSBSim/aircraft/f16/f16.xml`).

| UE field (FDM.h) | Type / unit / range | Entry point (plugin) | Transform on the way in | JSBSim property |
|---|---|---|---|---|
| `Aileron` (FDM.h:280) | double, dimensionless, −1..1 | `FCS->SetDaCmd(Commands.Aileron)` MC.cpp:687, setter FCS.h:382 | **none** (1:1) | `fcs/aileron-cmd-norm` (FCS.h:123) |
| `Elevator` (FDM.h:283) | double, dimensionless, −1..1 | `FCS->SetDeCmd` MC.cpp:689 | none | `fcs/elevator-cmd-norm` (FCS.h:124) |
| `Rudder` (FDM.h:286) | double, dimensionless, −1..1 | `FCS->SetDrCmd(-Commands.Rudder)` MC.cpp:691 | **sign inverted** | `fcs/rudder-cmd-norm` (FCS.h:125) |
| `Rudder` (again) | same value, **not** `Steer` | `FCS->SetDsCmd(Commands.Rudder)` MC.cpp:692 → forwards to FGGroundReactions (FCS.h:394) | none (positive) | `fcs/steer-cmd-norm` (FCS.h:126) |
| `YawTrim` (FDM.h:290) | double, −1..1 | `FCS->SetYawTrimCmd(-Commands.YawTrim)` MC.cpp:693 | **sign inverted** | `fcs/yaw-trim-cmd-norm` (FCS.h:132) |
| `PitchTrim` (FDM.h:293) | double, −1..1 | `FCS->SetPitchTrimCmd` MC.cpp:690 | none | `fcs/pitch-trim-cmd-norm` (FCS.h:130) |
| `RollTrim` (FDM.h:296) | double, −1..1 | `FCS->SetRollTrimCmd` MC.cpp:688 | none | `fcs/roll-trim-cmd-norm` (FCS.h:131) |
| `Steer` (FDM.h:302) | double, −1..1 | **never copied** — no reference in MC.cpp | — | — (dead field; steering is driven by `Rudder` above) |
| `LeftBrake` (FDM.h:305) | double, 0..1 | `FCS->SetLBrake(max(LeftBrake, ParkingBrake))` MC.cpp:699, setter FCS.h:521-523 | max() with parking brake | brake group value inside FGFCS; property name not present in local headers — 확인 불가 |
| `RightBrake` (FDM.h:308) | double, 0..1 | `FCS->SetRBrake(max(·, ParkingBrake))` MC.cpp:700 | max() with parking brake | same as above |
| `CenterBrake` (FDM.h:311) | double, 0..1 | `FCS->SetCBrake(max(·, ParkingBrake))` MC.cpp:701 | max() with parking brake | same as above |
| `ParkingBrake` (FDM.h:314) | double, 0..1 | folded into the three calls above | — | — |
| `GearDown` (FDM.h:318) | double, 0..1 (1=down) | `FCS->SetGearCmd` MC.cpp:702 | none | `gear/gear-cmd-norm` (FCS.h:133) |
| `Flap` (FDM.h:324) | double, 0..1 | `FCS->SetDfCmd` MC.cpp:694 | none | `fcs/flap-cmd-norm` (FCS.h:127) |
| `SpeedBrake` (FDM.h:327) | double, 0..1 | `FCS->SetDsbCmd` MC.cpp:695 | none | `fcs/speedbrake-cmd-norm` (FCS.h:128) |
| `Spoiler` (FDM.h:330) | double, 0..1 | `FCS->SetDspCmd` MC.cpp:696 | none | `fcs/spoiler-cmd-norm` (FCS.h:129) |

### 2.2 Engine commands — `TArray<FEngineCommand> EngineCommands`

Struct FDM.h:140-200. Applied per engine in `ApplyEnginesCommands()` (MC.cpp:1026-1088), called from
`CopyToJSBSim` (MC.cpp:704). The array is sized from the loaded model (`InitEnginesCommandAndStates`,
MC.cpp:1009-1024); the F-16 has one engine, so our code only touches index 0.

| UE field (FDM.h) | Type / unit / range | Entry point (plugin) | JSBSim binding |
|---|---|---|---|
| `Throttle` (FDM.h:153) | double, 0..1 | `FCS->SetThrottleCmd(i, ·)` MC.cpp:1038, setter FCS.h:423 ("normalized throttle command (0.0 - 1.0)") | `fcs/throttle-cmd-norm[i]` — consumed by the F-16 engine FCS channel (f16.xml:861) |
| `Mixture` (FDM.h:157) | double, 0..1 | `FCS->SetMixtureCmd(i, ·)` MC.cpp:1039 | per-engine mixture cmd (property name not in local headers — 확인 불가) |
| `PropellerAdvance` (FDM.h:164) | double, 0..1 | `FCS->SetPropAdvanceCmd` MC.cpp:1040 | per-engine advance cmd (확인 불가) |
| `PropellerFeather` (FDM.h:166) | bool | `FCS->SetFeatherCmd` MC.cpp:1041 | per-engine feather cmd (확인 불가) |
| `Starter` (FDM.h:160) | bool | `FGEngine::SetStarter` MC.cpp:1045 | direct engine-model state (not via property write) |
| `Running` (FDM.h:162) | bool | `FGEngine::SetRunning` MC.cpp:1046 | direct engine-model state |
| `Magnetos` (FDM.h:170) | enum 0-3 | `FGPiston::SetMagnetos` MC.cpp:1054 | piston only — N/A for F-16 |
| `Reverse`, `CutOff`, `Ignition`, `Augmentation`, `Injection` (FDM.h:182,184,178,174,176) | bool/int | `FGTurbine::Set*` MC.cpp:1061-1065 | direct turbine-model state (F-16 path) |
| `GeneratorPower`, `Condition` (FDM.h:188,190) | bool | `FGTurboProp::Set*` MC.cpp:1078-1081 | turboprop only — N/A for F-16 |

### 2.3 Other plugin inputs (one-shot / editor-driven)

| Input | Where | Notes |
|---|---|---|
| Initial conditions: actor pose → `IC->SetLongitudeDegIC/SetGeodLatitudeDegIC/SetAltitudeASLFtIC/SetPhiDegIC/SetPsiDegIC(+90°)/SetThetaDegIC` | MC.cpp:552-557 | BeginPlay only (`PrepareJSBSim`). Note **+90° yaw** here — inverse of the −90° output correction |
| `InitialCalibratedAirSpeedKts` → `IC->SetVcalibratedKtsIC` | MC.h:135, MC.cpp:587 | knots |
| `WindHeading`, `WindIntensityKts` → `IC->SetWindDirDegIC/SetWindMagKtsIC` | MC.h:145,151, MC.cpp:583-584 | deg / knots, IC only |
| `ControlFDMAtmosphere` + `TemperatureCelsius`/`PressureSeaLevelhPa` → `Atmosphere->SetTemperature/SetPressureSL` | MC.h:158-170, MC.cpp:571-580 | hPa→Pa ×100 at MC.cpp:574 |
| `FlapPositionAtStart` → `Commands.Flap` + `FCS->SetDfPos(ofNorm, ·)` | MC.h:129, MC.cpp:592-593 | the F16_UAV BP exposes/sets this (string `FlapPositionAtStart` in `Content/Blueprints/F16_UAV.uasset`) |
| `bStartWithGearDown` → `FCS->SetGearPos` | MC.h:117, MC.cpp:596-604 | |
| `bStartWithEngineRunning` → `Propulsion->InitRunning(-1)` + EngineCommands defaults | MC.h:123, MC.cpp:611-623 | sets Throttle 0, Mixture 1, Magnetos Both, Running true |
| Tanks: `FTank.FuelDensityPoundsPerGallon/ContentGallons` → `FGTank::SetDensity/SetContentsGallons` | FDM.h:29-31, MC.cpp:968-987 | **every frame** from `CopyToJSBSim` (MC.cpp:740) |
| `FuelFreeze` → `Propulsion->SetFuelFreeze` | MC.h:180, MC.cpp:1029 | every frame |
| Terrain: JSBSim ground queries → `GetAGLevel()` UE raycast | MC.h:219, MC.cpp:274-332; callback registered MC.cpp:492 | UE → JSBSim input of ground height/contact/normal (signed distance, meters→ft handled by callback in `UEGroundCallback.cpp`) |
| `SetWind(FSimpleWindState)` → `FGWinds::SetTurbType/SetTurbGain/SetTurbRate/SetWindNED/SetProbabilityOfExceedence` | MC.h:251-252, MC.cpp:199-232; struct FDM.h:457-501 (WindNED in knots, FDM.h:467-469) | BlueprintCallable — **unused in this project** (no C++ or BP references) |
| `CommandConsole(Property, InValue, Out)` — raw string get/set of any property-tree node | MC.h:237-238, MC.cpp:130-160 (Batch: 163-197) | **unused in this project** (no C++/BP references). This is the only generic path to properties not covered above (e.g. body rates `velocities/p-rad_sec`, alpha `aero/alpha-deg`) |

### 2.4 Project-side write chains (who writes `Commands` / `EngineCommands`)

Three writers exist; all end in the same direct member assignments.

**(a) UDP :5010 setpoints → BVRGym autopilot (UAVs; main autonomous path)**

1. External BT/controller sends JSON to port `SetpointListenPort` = 5010 (UDP.h:133-134).
   Fields parsed: `aircraft_name` (routing key), `heading_deg`, `altitude_m`, `throttle_norm`
   (clamped 0..1), `target_speed_mps`, `launch_missile`, `reset` (UDP.cpp:243-262). Batch form
   `{"setpoints":[...]}` supported (UDP.cpp:264-276).
2. Stored latest-wins in `TMap<FString,FUavSetpoint> Setpoints` (UDP.cpp:261; UDP.h:100).
3. `AutopilotTick()` on a **60 Hz timer** (UDP.cpp:80-84) resolves each key to a pawn — exact
   name match first, unique-substring fallback, ambiguous names skipped with a warning
   (UDP.cpp:302-341).
4. `ApplyAutopilotToPawn()` (UDP.cpp:344-401):
   - feedback read from the plugin: `AircraftState.LocalEulerAngles` Roll/Pitch/Yaw [deg] and
     `TotalVelocityKts` → m/s ×0.514444 (UDP.cpp:362-371, constant UDP.cpp:21);
   - **altitude feedback is UE world Z (cm→m), NOT JSBSim altitude** (UDP.cpp:370, rationale in the
     comment at UDP.cpp:366-369: the BT computes setpoints against the published UE-Z);
   - `FAircraftAutopilot::GetControlInput` (AP.cpp:55-126) produces Aileron/Elevator ∈ [−1,1]
     (clamps AP.cpp:134, 140-141), Rudder always 0 (AP.h:110), Throttle ∈ [0,1] from the speed-hold
     PI when `target_speed_mps > 0`, else −1 → open-loop `throttle_norm` used (AP.cpp:144-155,
     UDP.cpp:381);
   - writes: `JSBSim->Commands.Aileron/.Elevator/.Rudder = Out.*` and
     `JSBSim->EngineCommands[0].Throttle = ThrottleOut` (UDP.cpp:383-387).
5. Per-aircraft controller instances live in `TMap<FString,FAircraftAutopilot> Autopilots`
   (UDP.h:101); gains are re-synced from the actor's UPROPERTY configs each tick without wiping
   PID state (UDP.cpp:352-360, AP.cpp:27-34).

**(b) UDP :5005 raw stick commands (manned aircraft / joystick, and legacy Python control)**

1. JSON on port `ListenPort` = 5005 (UDP.h:106-107): either
   `{"commands":[{"aircraft_name","roll","pitch","yaw","throttle"},...]}` or a single flat object;
   legacy CSV `roll,pitch,yaw,throttle` fallback (UDP.cpp:445-527).
2. Per-frame `Tick` applies **name-matched commands only**: a pawn gets a command iff the pawn's
   name contains the command's `aircraft_name` (UDP.cpp:108-124). Pawn set = names matching
   `ControlledPawnNamePatterns`, capped at `MaxControlledUavs` (UDP.cpp:105, UDP.h:125,180-181).
3. `ApplyControlCommandToPawn` (UDP.cpp:633-660) writes both:
   - BP variables `UDP_Roll/UDP_Pitch/UDP_Yaw/UDP_Throttle` via reflection (UDP.cpp:641-644) — for
     Blueprints/HUD that read them;
   - directly `JSBSim->Commands.Aileron = roll`, `.Elevator = pitch`, `.Rudder = yaw`,
     `EngineCommands[0].Throttle = throttle` (UDP.cpp:650-656).
   No range clamping is done on this path — sender is trusted to keep −1..1 / 0..1.

**(c) Pawn Blueprints (keyboard/gamepad in-editor flying)**

`F16_UAV.uasset` / `M_F16.uasset` own the `JSBSimMovement` component and write
`Commands` members (Aileron/Elevator/Rudder + trims, brakes, flaps, gear, speedbrake) and
`EngineCommands` throttle from input events, prioritized via `PrioritizeDualUserInput`
(`S_DualInputCommand`). Evidence: uasset name tables contain `JSBSimMovementComponent`,
`FlightControlCommands`, `EngineCommands`, `AileronData/ElevatorData/ThrottleData`, input-action
events, and the comment string "Apply the prioritized control data to the flight commands".
Exact graph wiring is a binary asset — 확인 불가 beyond string evidence.

---

## 3. Per-frame processing (`TickComponent`, MC.cpp:335-422 — plugin code)

Order of operations each game frame (only when `AircraftLoaded` and not `Crashed`, MC.cpp:341-348):

1. **Fixed-step budget** — `simDtime = 120 × DeltaTime`; fractional part accumulates in `remainder`;
   `simloops = trunc(simDtime) + trunc(remainder)` (MC.cpp:352-355).
2. **`Exec->Setdt(1/120)`** — FDM dt is hard-fixed at 0.008333 s (MC.cpp:358).
3. **`CopyToJSBSim()`** (MC.cpp:361) — all §2.1/§2.2 writes, plus tanks/fuel-freeze.
4. **FDM step loop** — `Exec->Run()` executed `simloops` times (MC.cpp:364-367). Each Run advances
   JSBSim by exactly 1/120 s.
5. **`UpdateLocalTransforms()`** (MC.cpp:371) — recompute CG/EP/VRP local offsets (ft→cm ×30.48,
   inch→cm ×2.54; structural→actor axis flip X→−X) (MC.cpp:848-891).
6. **`CopyFromJSBSim()`** (MC.cpp:374) — all §4 reads into `AircraftState`, `EngineStates`,
   `Gears`, `Tanks`.
7. **Coordinate conversion & actor update** (MC.cpp:377-407):
   - ESU tangent transform built at the aircraft's ECEF location (local modification — see §5)
     (MC.cpp:380, 73-91);
   - **Yaw −90°**: JSBSim ψ is aero heading (0 = North), UE yaw 0 = East (MC.cpp:381-382);
   - rotation: `EngineRotationQuat = ESUTransform ⊗ LocalUERotation` (MC.cpp:383);
   - location: ECEF→UE via GeoReferencing, then subtract the world-rotated CG offset so the actor
     origin (not the CG) is placed (MC.cpp:385-392);
   - `UEForwardHorizontal` recomputed for the PFD (MC.cpp:394-395);
   - NaN in location/rotation ⇒ `CrashedEvent()` (suspends integration, sets `Crashed`, broadcasts
     `AircraftCrashed`) (MC.cpp:398-401, 1154-1159); otherwise
     **`Parent->SetActorLocationAndRotation(...)`** (MC.cpp:404).
8. Optional on-screen debug (MC.cpp:410-414).

**Input transforms — complete list** (everything that changes a value between UE and JSBSim):

| Where | Transform |
|---|---|
| MC.cpp:691 | Rudder sign inversion (`SetDrCmd(-Rudder)`) |
| MC.cpp:693 | YawTrim sign inversion (`SetYawTrimCmd(-YawTrim)`) |
| MC.cpp:699-701 | Brakes = `max(brake, ParkingBrake)` |
| MC.cpp:556 | IC heading **+90°** (UE yaw → aero ψ), inverse of the output −90° |
| MC.cpp:554 | IC altitude m→ft (`METER_TO_FEET`, FDM.h:10) |
| MC.cpp:574 | Pressure hPa→Pa (×100) |
| UDP.cpp:257 | `throttle_norm` clamp 0..1 (project code) |
| AP.cpp:134,140-141,153-154 | autopilot output clamps (aileron/elevator −1..1, throttle 0..1) (project code) |
| — | Aileron/Elevator/Flap/SpeedBrake/Spoiler/GearDown: **no transform** (verified 1:1 at MC.cpp:687-702) |

**120 Hz FDM ↔ game frame sync.** The plugin does *not* interpolate: it runs an integer number of
1/120 s steps per rendered frame and carries the fractional step in `remainder` (MC.cpp:352-355), so
simulated time tracks wall-clock on average but individual frames can run N or N+1 steps. The code
comment says the game should run at a fixed rate (MC.cpp:351); this project pins the engine to a
fixed 60 FPS (`bUseFixedFrameRate=True`, `FixedFrameRate=60.0`, `Config/DefaultEngine.ini:74-75`),
giving exactly **2 FDM steps per frame**. Project-side loops are on their own clocks: autopilot
60 Hz timer (UDP.cpp:80-84), state send every `StateSendInterval` = 0.05 s = 20 Hz (UDP.h:116,
UDP.cpp:87-91), UDP receive drained per frame (UDP.cpp:102-103). Commands written between FDM steps
are zero-order-held by JSBSim.

---

## 4. Output side (JSBSim → UE)

### 4.1 `FAircraftState` — filled in `CopyFromJSBSim()` (MC.cpp:744-796)

Struct FDM.h:347-435. Consumers (project code):
- **AP** = autopilot feedback, UDP.cpp:362-371
- **NET** = outbound state JSON on :5006, UDP.cpp:824-838
- **PFD** = `Content/Blueprints/PFD/UMG_BasicPrimaryFlightDisplay.uasset` (referenced by both pawn
  BPs; reads proven by uasset string table: `AltitudeASLFt`, `CalibratedAirSpeedKts`, `EulerRates`,
  `GroundSpeedKts`, `LocalEulerAngles(_Pitch/_Roll/_Yaw)`, `UEForwardHorizontal`, `VelocityNEDfps`)
- **XFORM** = consumed inside `TickComponent` itself for the actor transform

| Field (FDM.h) | Source getter (MC.cpp) | Type / unit | Consumers |
|---|---|---|---|
| `ElevatorPosition` (358) | `FCS->GetDePos(ofDeg)` :759 | double, **deg** | 미사용 (debug text only) |
| `LeftAileronPosition` (360) | `FCS->GetDaLPos(ofDeg)` :760 | double, deg | 미사용 |
| `RightAileronPosition` (362) | `FCS->GetDaRPos(ofDeg)` :761 | double, deg | 미사용 |
| `RudderPosition` (364) | `-1 × FCS->GetDrPos(ofDeg)` :762 | double, deg, **sign inverted back** (matches the input inversion) | 미사용 |
| `FlapPosition` (366) | `FCS->GetDfPos(ofDeg)` :763 | double, deg | 미사용 |
| `SpeedBrakePosition` (368) | `FCS->GetDsbPos(ofDeg)` :764 | double, deg | 미사용 |
| `SpoilersPosition` (370) | `FCS->GetDspPos(ofDeg)` :765 | double, deg | 미사용 |
| `CalibratedAirSpeedKts` (375) | `Auxiliary->GetVcalibratedKTS()` :768 (kts at source, FCS-side conversion in FGAuxiliary.h:164) | double, **kts (CAS)** | PFD |
| `GroundSpeedKts` (377) | `Auxiliary->GetVground() × 0.592484` :769 (fps→kts, FDM.h:15) | double, kts | PFD |
| `TotalVelocityKts` (379) | `Auxiliary->GetVt() × 0.592484` :770 | double, **kts (true airspeed)** | **AP** (→ m/s, UDP.cpp:371), **NET** (`speed_mps`, UDP.cpp:825,833-834), PFD 미표시 |
| `VelocityNEDfps` (381) | `Propagate->GetVel(eNorth/eEast/−eDown)` :771 | FVector, **ft/s**; ⚠ Z = **−vDown (up-positive)** despite the "NED" name | PFD |
| `AltitudeASLFt` (383) | `Propagate->GetAltitudeASL()` :772 | double, **ft ASL** | PFD; also AGL raycast length (MC.cpp:294). **Not** used by AP (uses UE-Z) or NET (sends UE-Z) |
| `AltitudeAGLFt` (385) | `Propagate->GetDistanceAGL()` :785 | double, ft AGL | 미사용 |
| `AltitudeRateFtps` (387) | `Propagate->Gethdot()` :773 (= −vDown, FGPropagate.h:418) | double, ft/s climb rate | 미사용 |
| `StallWarning` (389) | `Aerodynamics->GetStallWarn()` :774 | double 0..1 | 미사용 |
| `ECEFLocation` (394) | `Propagate->GetLocation() × 0.3048` :777-778 | FVector, **meters ECEF** (ft→m, FDM.h:9) | XFORM (MC.cpp:380,391) |
| `Latitude` (396) | `LocationVRP.GetGeodLatitudeDeg()` :779 | double, deg geodetic | 미사용 |
| `Longitude` (398) | `LocationVRP.GetLongitudeDeg()` :780 | double, deg | 미사용 |
| `LocalEulerAngles` (400) | `Propagate->GetEuler(ePsi/eTht/ePhi)` rad→deg :781-783 | FRotator, deg; **Yaw = aero heading (0=North, no −90 applied to the stored value)** | **AP** (UDP.cpp:363-365,373), **NET** (`pitch/roll/yaw`, UDP.cpp:826,835-837), PFD, XFORM (−90° copy at MC.cpp:381-382) |
| `EulerRates` (402) | `Auxiliary->GetEulerRates(ePhi/eTht/ePsi)` :784 | FVector (X=φ̇, Y=θ̇, Z=ψ̇) — **Euler-angle rates, not body rates p,q,r**; unit rad/s (JSBSim convention; local header doesn't state it — cf. rad/s rate fields in `Include/input_output/net_fdm.hxx:55-57`) | PFD |
| `UEForwardHorizontal` (404) | body X-axis → local tangent, Z zeroed :755-756, ESU-transformed :395 | FVector, UE world, unnormalized horizontal forward | PFD |
| `Crashed` (408) | set by `CrashedEvent()` :1157 (NaN transform only — AGL auto-crash disabled locally, :787-789) | bool | XFORM gate (MC.cpp:343); `AircraftCrashed` delegate (MC.h:203-204) 미사용 (no BP bindings found) |

### 4.2 Actor transform (the main "output" our whole project consumes)

`Parent->SetActorLocationAndRotation(EngineLocation, EngineRotationQuat)` (MC.cpp:404) — plugin
writes the pawn transform directly. Everything downstream that uses pawn position/rotation is
therefore consuming JSBSim output, notably:
- `BuildPawnState`: `x/y/z` = `Pawn->GetActorLocation()` (UE world cm), UDP.cpp:808,818-820 → :5006 JSON;
- autopilot altitude feedback `GetActorLocation().Z / 100` (UDP.cpp:370);
- anything in Blueprints/level logic reading pawn transforms (missiles, camera, etc.).

### 4.3 Other output arrays

| Output | Filled at | Contents / unit | Consumers |
|---|---|---|---|
| `EngineStates` (`FEngineState`, FDM.h:203-265) | `GetEnginesStates()` MC.cpp:1090-1150 | Thrust (lbf per JSBSim internal — unit not documented in local header, 확인 불가), RPM, N1/N2 %, flags | 미사용 (debug text only; not read by project C++ or BP name tables) |
| `Gears` (`FGear`, FDM.h:54-119) | `CopyGearPropertiesFromJSBSim()` MC.cpp:923-944 | positions (normalized), WOW, wheel speed m/s (ft→m :933), relative location/force (ft→cm :937-941) | 미사용 in project (debug draw MC.cpp:1249-1272 only) |
| `Tanks` (`FTank`, FDM.h:18-52) | `CopyTankPropertiesFromJSBSim()` MC.cpp:989-1005 | gallons, %, °C, lb/gal | 미사용 in project |
| Outbound UDP :5006 (project) | `SendStateToPython()` UDP.cpp:850-897, JSON built in `BuildPawnState` UDP.cpp:801-848 | per pawn: `aircraft_name`, `x/y/z` (UE cm), `speed_mps` (from TotalVelocityKts), `pitch/roll/yaw` (deg; yaw = aero heading), `throttle` (= **commanded** `EngineCommands[0].Throttle`, not achieved FCS position, UDP.cpp:827-829), `team`, `weapons{bullet_ammo,rocket_ammo}` via BP reflection | external ROS bridge / BT |

---

## 5. Local plugin modifications vs upstream (diff view)

Baseline: upstream `a222488` (2025-04-27) — smallest diff of all tags/commits checked (81 lines vs
1800+ for v1.2.x due to a later upstream line-ending change). Only `JSBSimMovementComponent.h/.cpp`
differ; `FDMTypes.*` and `UEGroundCallback.*` are byte-identical to upstream master. Our repo's git
history shows no plugin source edits after the initial import (only the `.uplugin` was added in
`3e6a23b`), so these deltas came in with the initial import.

| # | Modification | Location | Effect |
|---|---|---|---|
| 1 | **ESU tangent frame replaces `GetTangentTransformAtECEFLocation`** — new `BuildESUTangentTransform` (East, −North, Up basis) + `GetESUTransformAtECEFLocation/AtGeographicLocation`; used in TickComponent, PrepareJSBSim, GetAGLevel | MC.cpp:73-91, 380, 395, 547, 831-846; MC.h:330-331 | Correct attitude/heading mapping into UE's left-handed frame. Side effect: upstream's later "FlatPlanet" −180° yaw fix is absent (upstream added it after our baseline) |
| 2 | **Automatic crash-on-AGL<−10 ft disabled** (commented out) | MC.cpp:786-789 (active in upstream) | Aircraft can clip below terrain without triggering `Crashed`; only NaN transforms crash (MC.cpp:398-401) |
| 3 | Data-path setup uses absolute `FPaths::Combine` strings instead of upstream's `SGPath` relative paths | MC.cpp:509-516 | Functional equivalence for our layout; aircraft/engine/systems resolved under `Plugins/.../Resources/JSBSim` |
| 4 | Cosmetic/API: `LoadModel(TCHAR_TO_UTF8(...))` without explicit `std::string` (MC.cpp:243), `GetModelNameCString()` in the editor-only visualizer path (MC.cpp:1320) | — | No behavioral change |

Whether these were authored in-house or inherited from an intermediate fork: 확인 불가 (predates our
git history); none of them exist in any upstream revision checked (v1.2.0–v1.2.2, 2024-09→master).

---

## 6. Project-specific notes: unused fields & potential issues

**Unused / dead items**

- `Commands.Steer` is never forwarded to JSBSim (§2.1); steering silently reuses `Rudder`
  (MC.cpp:692). Writing `Steer` does nothing.
- `FUavSetpoint.LaunchMissile`: parsed from :5010 JSON (UDP.cpp:259) but **never read** anywhere —
  missile-launch setpoints are currently a no-op in UE.
- `CommandConsole` / `CommandConsoleBatch` / `PropertyManagerNode` / `SetWind`: exposed, unused (§2.3).
- `AircraftCrashed` delegate: no subscribers found (§4.1).
- `IndexedControlCommands` is populated (UDP.cpp:476) but only ever used as a fallback source for
  `BroadcastCommand` (UDP.cpp:493-496); `BroadcastCommand` itself is no longer applied to any pawn —
  it only feeds the read-only `Roll/Pitch/Yaw/Throttle` UPROPERTYs (UDP.cpp:498-501, UDP.h:210-220).
  Positional/broadcast control is effectively retired in favor of name matching (comment UDP.cpp:110-114).
- `AircraftState` fields with no consumer at all: all seven surface positions, `AltitudeAGLFt`,
  `AltitudeRateFtps`, `StallWarning`, `Latitude`, `Longitude` (§4.1). Surface positions would be the
  hook for control-surface animation if wanted later.

**Potential issues / gotchas**

1. **Two writers can fight over one aircraft.** The :5005 name-matched path writes
   `Commands`/`EngineCommands[0].Throttle` in `Tick` (per frame, UDP.cpp:108-124 → 650-656) while
   the :5010 autopilot writes the same fields on a 60 Hz timer (UDP.cpp:383-387). If both a named
   :5005 command and a :5010 setpoint exist for the same `aircraft_name`, last-writer-wins per
   frame with no defined priority.
2. **Setpoints never expire.** `Setpoints` entries persist until overwritten (UDP.cpp:261);
   `reset:true` recreates the PID controller (UDP.cpp:249-251) but does not remove the setpoint, so
   a UAV keeps flying the last setpoint forever after the BT stops publishing.
3. **`VelocityNEDfps` is not NED**: Z is up-positive (−vDown) (MC.cpp:771). Any future consumer
   assuming true NED will get inverted vertical speed.
4. **Heading conventions differ by surface**: `AircraftState.LocalEulerAngles.Yaw` and the :5006
   `yaw` field are aero/compass heading (0 = North), while the actor's UE yaw is that minus 90°
   (MC.cpp:381-382). BT-side code must use the compass convention (this is the
   `atan2(Δx,−Δy)`-style rule already documented in README_Autopilot.md).
5. **Altitude convention**: autopilot + :5006 use UE world Z, not JSBSim ASL/AGL (UDP.cpp:366-370);
   mixing them reintroduces the georeference origin offset bug the comment describes.
6. **No clamping on the :5005 path** (UDP.cpp:650-656): out-of-range roll/pitch/yaw/throttle go to
   JSBSim as-is (the :5010 path clamps throttle and the autopilot clamps surfaces).
7. **Throttle echo is the command, not the state**: :5006 `throttle` reports
   `EngineCommands[0].Throttle` (UDP.cpp:827-829), which our own autopilot wrote — a consumer
   can't distinguish achieved thrust from commanded throttle (N1/N2/Thrust exist in
   `EngineStates` but are unpublished).
8. **Crash detection is mostly off** (local mod #2, §5) — below-terrain flight won't raise events.
9. Debug-only: `FAircraftState::GetDebugMessage` pairs Yaw with `EulerRates.X` (=φ̇/roll rate) and
   Roll with `.Z` (=ψ̇) (FDM.h:419) — the on-screen debug rate labels are crossed; the stored data
   itself is ordered X=φ̇, Y=θ̇, Z=ψ̇ (MC.cpp:784).
10. `Propagate->DumpState()` runs every frame (MC.cpp:747) — upstream behavior, potential log noise;
    runtime cost/output destination 확인 불가 (implementation is in the prebuilt JSBSim library).

---

## 7. Input-surface catalog by role — and what we actually drive

JSBSim's true input surface is its **property tree**; the plugin hard-wires only a subset of it
(§2). This section groups every input by what it physically does, and marks usage.
Usage legend: ✅ = written with varying values at runtime, 🔒 = sent every frame but constant in
our setup, ⚙ = set once at start, ✖ = available but unused.

**Tier 1 — plugin-wired, sent every frame (`CopyToJSBSim`, MC.cpp:684-742)**

| Role | Input(s) | What it does | Our use |
|---|---|---|---|
| Primary stick | `Commands.Aileron/Elevator/Rudder` | roll / pitch / yaw surface commands (−1..1) into the F-16 FBW FCS | ✅ Aileron/Elevator (autopilot & joystick), 🔒 Rudder ≡ 0 on the autopilot path |
| Engine power | `EngineCommands[0].Throttle` | normalized thrust demand → `fcs/throttle-cmd-norm[0]` → F100 engine model | ✅ |
| Trim | `PitchTrim/RollTrim/YawTrim` | zero-point offsets added to stick channels | 🔒 pitch = ground-trim solution (MC.cpp:824), roll/yaw = 0 |
| High-lift / drag | `Flap`, `SpeedBrake`, `Spoiler` | low-speed lift; deceleration; lift-dump/roll assist | 🔒 Flap = `FlapPositionAtStart`, others 0 |
| Undercarriage | `GearDown`, `LeftBrake/RightBrake/CenterBrake/ParkingBrake`, `Steer` | gear extension (0/1), wheel brakes (0..1), nosewheel steering | 🔒 gear 1 (down), brakes 0; `Steer` is dead (never copied) — steering rides on `Rudder` (MC.cpp:692) |
| Engine management | `Starter`, `Running`, `CutOff`, `Ignition`, `Augmentation` (afterburner), `Injection`, `Reverse`, `Mixture`, `PropellerAdvance/Feather`, `Magnetos`, `GeneratorPower`, `Condition` | start/stop, fuel cutoff, AB, etc. — only the turbine subset applies to the F-16 | 🔒 Running=true, CutOff=false, rest defaults (MC.cpp:1044-1066) |
| Fuel | `Tanks[i].ContentGallons/FuelDensityPoundsPerGallon`, `FuelFreeze` | fuel quantity/density per tank; freeze consumption | 🔒 round-trips JSBSim's own values (§2.3); FuelFreeze=false |

**Tier 2 — plugin-wired, one-shot at BeginPlay (`PrepareJSBSim`, MC.cpp:525-652)**

| Role | Input(s) | Our use |
|---|---|---|
| Where/how the aircraft starts | IC position/attitude from the placed actor; `InitialCalibratedAirSpeedKts`; `StartOnGround` (ground trim + zero NED velocity); `bStartWithGearDown`; `bStartWithEngineRunning`; `FlapPositionAtStart` | ⚙ actor pose from the map; everything else C++ defaults (0 kts, on-ground, gear down, engine running) — confirmed via delta-serialization absence in the pawn BPs/maps |
| Environment at start | `WindHeading/WindIntensityKts` (IC wind), `ControlFDMAtmosphere` + temperature/pressure | ⚙ all defaults → zero wind, JSBSim standard atmosphere |

**Tier 3 — reachable only via `CommandConsole` (or `SetWind`) — all ✖ unused today**

| Role | Representative writable properties (verified in local sources) | Why you'd use it |
|---|---|---|
| Wind & turbulence | `SetWind()` (FGWinds setters, MC.cpp:199-232); milspec turbulence props `atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps`, `.../severity` (FGWinds.h:80-81) | wind disturbance / gust injection for robustness tests |
| Live fuel | `propulsion/tank[i]/contents-lbs` (FGTank.h:95) | refuel/leak scenarios, weight sweeps |
| Trim on demand | `simulator/do_trim` — write tLongitudinal(0)/tFull(1)/tGround(2)... triggers DoTrim (FGFDMExec.h:170-174) | re-trim mid-flight |
| F-16-specific switches | `fcs/fbw-override`, `fcs/hook-engage`, `fcs/canopy-engage` (declared in f16.xml:311-314) | disable the FBW damper loops, hook/canopy animation states |
| Direct state/overrides & everything else | full catalog via `PropertyManagerNode` (MC.h:229-230); surface-position overrides (`fcs/*-pos-*`, FCS.h:134-163), body rates/alpha reads, failures | fault injection, RL resets, reading p,q,r/alpha that `FAircraftState` lacks |

Tier 3 write-ability caveat: property registration lives in the prebuilt `libJSBSim.a` (`bind()`
implementations, no sources in-repo), so beyond the header-documented cases above, whether a given
node is writable is 확인 불가 statically — dump the runtime catalog to check.

**Bottom line.** Of the entire input surface, the values our project varies at runtime are exactly
four: `Commands.Aileron`, `Commands.Elevator`, `Commands.Rudder` (held at 0 by the autopilot), and
`EngineCommands[0].Throttle` (§2.4). Everything else is a constant re-send (Tier 1), a one-shot
default (Tier 2), or unreached (Tier 3). The terrain ground-callback (§2.3) is the one additional
per-step input JSBSim pulls on its own.

## 8. Verification points (requested cross-checks)

| Claim | Verdict | Evidence |
|---|---|---|
| `Commands.Aileron` etc. are normalized −1..1 stick commands passed unchanged via `SetDaCmd` to `fcs/aileron-cmd-norm` | **Confirmed** | Range doc FDM.h:278-280; pass-through MC.cpp:687 (no transform); setter FCS.h:382; property FCS.h:123; consumed by f16.xml:366 |
| `SetDrCmd(-Commands.Rudder)` rudder sign inversion exists | **Confirmed** | MC.cpp:691. Mirrored on output (`RudderPosition = −GetDrPos`, MC.cpp:762) and in trim readback (`Commands.Rudder = −GetDrCmd()` with upstream's own "Why this minus sign?" TODO, MC.cpp:826). `YawTrim` is likewise inverted (MC.cpp:693) |
| `FAircraftState` has no body-axis rates (p,q,r) and no angle of attack; `EulerRates` are Euler-angle rates | **Confirmed** | Full struct FDM.h:347-435 contains neither p/q/r nor alpha; `EulerRates` filled from `FGAuxiliary::GetEulerRates` (φ̇,θ̇,ψ̇) MC.cpp:784. **How our code copes:** the BVRGym autopilot uses no rate or alpha feedback at all — damping comes from the PID derivative term on the error signal (D = Kd·(e−e_prev), AP.cpp:14) and direct elevator schedules in hard-turn mode (AP.cpp:77-80); altitude uses UE-Z (UDP.cpp:370). Body rates/alpha *could* be fetched via `CommandConsole` (`velocities/p-rad_sec`, `aero/alpha-deg`) but no project code does |

