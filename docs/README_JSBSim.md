# JSBSim Plugin Structure (the library itself + Unreal integration)

> Scope: `Plugins/JSBSimFlightDynamicsModel` — the plugin that runs the JSBSim flight dynamics model in UE5.4.
> Two parts: **(1) the JSBSim library's own structure**, and **(2) how that structure is integrated into Unreal**.
>
> - Date: 2026-06-28 (git `25c5459`)
> - Parent doc: [ARCHITECTURE.md](ARCHITECTURE.md) (whole project)

---

## Part 1. JSBSim itself (the FDM library)

JSBSim is a standalone open-source **6-DoF flight dynamics library**, independent of Unreal. It is brought into this
project only as headers (`Source/ThirdParty/JSBSim/Include`) and **prebuilt binaries** (`Lib/Linux/libJSBSim.a`,
`Lib/JSBSim.dll`/`.lib`). So it is **header + static/dynamic library linkage**, not a source build.

### Core design: "data-driven + a property tree"

```
              ┌─────────────────────────── FGFDMExec (the executive/conductor) ────────────────────┐
              │  - Owns every model; each step Run() executes the models below once, in order        │
              │  - Manages dt (recommended 1/120s), Runge-Kutta integration, LoadModel(xml),         │
              │    RunIC(), Trim()                                                                    │
              │                                                                                       │
              │   ┌──────────────────────── FGPropertyManager (the property tree) ──────────────┐   │
              │   │  Central blackboard. Every JSBSim value exists as a string path:               │   │
              │   │   fcs/elevator-cmd-norm, fcs/throttle-cmd-norm[0],                             │   │
              │   │   position/h-sl-ft, velocities/vc-kts, attitude/phi-rad ...                    │   │
              │   │  → Models don't call each other directly; they read/write through this tree    │   │
              │   └───────────────────────────────────────────────────────────────────────────┘   │
              │                                                                                       │
              │   Models executed each Run() (each derives from FGModel):                             │
              │   FGInput → FGAtmosphere/FGWinds → FGFCS → FGMassBalance → FGPropulsion              │
              │     → FGAerodynamics → FGGroundReactions → FGExternalReactions → FGBuoyantForces     │
              │       → FGAircraft(sum forces/moments) → FGAccelerations → FGPropagate(integrate EOM) │
              │         → FGAuxiliary(derived quantities) → FGOutput                                  │
              └───────────────────────────────────────────────────────────────────────────────────┘
```

What each model does:

| Model | Role |
|---|---|
| **FGFCS** | Flight Control System. Maps pilot commands (da/de/dr/throttle…) through control channels (gains/filters/actuators in `systems/*.xml`) into **actual surface positions** |
| **FGAerodynamics** | Computes aero forces/moments from coefficient lookup tables in `<aerodynamics>` (vs alpha, Mach, etc.) |
| **FGPropulsion** | Engines (`FGEngine` → Piston/Turbine/Turboprop/Rocket/Electric) + fuel tanks (`FGTank`) + thrusters (`FGThruster`) |
| **FGMassBalance** | Mass, center of gravity (CG), inertia tensor. CG shifts over time as fuel burns |
| **FGGroundReactions** | Landing-gear (`FGLGear`) contact forces |
| **FGInertial** | Gravity + **ground-height query (FGGroundCallback)** — the Unreal integration hook (Part 2) |
| **FGPropagate** | Integrates the equations of motion → position (ECEF/geodetic lat-lon), attitude (Euler), velocity |
| **FGAuxiliary** | Derived quantities (Mach, CAS, α/β, Euler rates, etc.) |
| **FGAtmosphere / FGWinds** | Standard atmosphere; wind/turbulence models (None/Standard/Culp/Milspec/Tustin) |

### The aircraft is data (XML), not code

`Resources/JSBSim/aircraft/f16/f16.xml` shows the canonical JSBSim aircraft definition:

```
<fdm_config name="General Dynamics F-16A">   ← root
  <fileheader>          author/source/references
  <metrics>             wing area 300ft², span 30ft, chord 11.32ft ...
  <mass_balance>        empty weight, moments of inertia, CG location
  <ground_reactions>    landing gear (FGLGear units)
  <propulsion>          <engine file="F100-PW-229"> + <thruster file="direct">  ← references engine/ folder
  <flight_control> / <system file="pushback"/> <system file="hook"/>           ← references systems/ folder
  <aerodynamics>        aero coefficient tables (lookups vs alpha/Mach)
```

So the data folder is split three ways (registered via `Exec->SetAircraftPath/SetEnginePath/SetSystemsPath`):
- `Resources/JSBSim/aircraft/` — 61 aircraft (f16, various airliners…)
- `Resources/JSBSim/engine/` — engine catalog (F100-PW-229, CF6, CFM56 …)
- `Resources/JSBSim/systems/` — shared subsystems (autopilot, FCS-pitch/roll, accelerometers …)

> **Summary:** JSBSim = "an XML-described aircraft + a model pipeline that communicates via a property tree + the
> FGFDMExec conductor." From the outside it's a black box you drive with one cycle:
> `Setdt → CopyIn (write properties) → Run() → CopyOut (read properties)`.

---

## Part 2. How it's layered into Unreal (the plugin wrapper)

The plugin is a **thin wrapper** around that library. The `.uplugin` declares 3 modules + 1 dependency:

```
JSBSimFlightDynamicsModel.uplugin
├─ Module: JSBSimFlightDynamicsModel        (Runtime)  ← the core wrapper
├─ Module: JSBSimFlightDynamicsModelEditor  (Editor)   ← editor visualization (draws reference points)
├─ ThirdParty/JSBSim.Build.cs               (External) ← include headers + link libJSBSim
└─ Plugin dependency: GeoReferencing (required)        ← ECEF↔UE coordinate conversion
```

`Source/ThirdParty/JSBSim.Build.cs` only does per-platform linking — Linux links `libJSBSim.a` (static), Windows links
`JSBSim.lib` + stages `JSBSim.dll` at runtime, Mac links `.dylib`, Android links `.so`.

### The center of everything: `UJSBSimMovementComponent` (UActorComponent)

The single bridge between JSBSim and Unreal is **one class**, a movement component attached to the aircraft Actor
(M_F16 / F16_UAV). It holds the JSBSim objects as members:

```cpp
JSBSim::FGFDMExec* Exec;                          // the conductor (raw pointer, new/delete)
std::shared_ptr<FGFCS>        FCS;                // handles to the models owned by Exec
std::shared_ptr<FGPropagate>  Propagate;          // (shared_ptr). The header only forward-declares
std::shared_ptr<FGPropulsion> Propulsion;         // them so JSBSim headers don't leak outward
... FCS, MassBalance, Aircraft, Auxiliary, Inertial, Aerodynamics, GroundReactions ...
```

### Data mirroring: FDMTypes.h

Unreal/Blueprint can't see JSBSim's C++ types, so `FDMTypes.h` defines **Blueprint-facing mirror structs (USTRUCT
BlueprintType)**:
- Inputs: `FFlightControlCommands` (Aileron/Elevator/Rudder/Flap/Brake/Gear… normalized [-1..1]/[0..1]), `FEngineCommand` (Throttle/Mixture/Starter…)
- Outputs: `FAircraftState` (surface deg, CAS/ground kts, VelocityNED, Alt ASL/AGL, ECEF, lat/lon, Euler angles/rates), `FEngineState`, `FTank`, `FGear`
- Unit-conversion macros (`FEET_TO_METER`, `KNOT_TO_FEET_PER_SEC`, …) also live here

### Lifecycle

```
OnRegister / BeginPlay
   └─ Find the GeoReferencingSystem actor (error if absent — required dependency)
   └─ InitializeJSBSim()      new FGFDMExec → grab model pointers
   │                          → Inertial->SetGroundCallback(new UEGroundCallback(this))  ★registers ground link
   │                          → SetRootDir/AircraftPath/EnginePath/SystemsPath (Resources/JSBSim)
   └─ LoadAircraft()          Exec->LoadModel(AircraftModel)  ← parse XML, build engines/tanks/gears
   │                          → rebuild UE arrays (EngineCommands/Tanks/Gears)
   │                          ※ AircraftModel is a per-actor EditAnywhere UPROPERTY (default ""); every
   │                            aircraft here (manned M_F16 + every F16_UAV) sets it to "f16", so all
   │                            share one flight envelope / max speed
   └─ PrepareJSBSim()         Actor's UE Transform → geodetic → set JSBSim initial conditions (IC)
                              (lat/lon/alt/Psi+90/Theta/Phi, speed, wind, flaps, gear)
                              → RunIC() → start engines → DoTrim() (solve equilibrium via FGTrim)
```

### The per-frame core: `TickComponent` (JSBSimMovementComponent.cpp:335)

This is the heart of the Unreal integration:

```
1. Fixed 120Hz pacing  compute simloops + accumulate remainder → JSBSim steps at 1/120s regardless of FPS
                       Exec->Setdt(1/120)          (cpp:351-358)
                       ※ NOTE: full determinism also requires running the game at a fixed tick.
                         In practice the sim runs near realtime but can drop below realtime under heavy
                         load (e.g. many aircraft / terrain streaming).

2. CopyToJSBSim()      UE Commands → JSBSim properties  (cpp:684)
                       FCS->SetDaCmd(Aileron), SetDeCmd(Elevator), SetDrCmd(-Rudder),
                       SetThrottleCmd(i, Throttle), brakes/gear...   ★Rudder/Yaw/Steer are sign-flipped

3. for(simloops) Exec->Run()    the actual physics integration (cpp:364-367)

4. UpdateLocalTransforms()      recompute CG/Eye/VRP/gear positions (CG shifts as fuel burns) (cpp:848)

5. CopyFromJSBSim()  JSBSim properties → UE AircraftState  (cpp:744)
                     surface deg, CAS/Ground/Total kts, VelocityNED, Alt ASL/AGL,
                     ECEFLocation, lat/lon, Euler (Psi/Theta/Phi), rates
                     ※ TotalVelocityKts = Auxiliary->GetVt() (JSBSim true airspeed). UDPControlReceiver
                       reads this and computes SpeedMps = TotalVelocityKts * KnotToMetersPerSecond
                       (0.514444) → the airspeed fed to the autothrottle / speed-hold PI.

6. Coordinate transform + move the actor  (cpp:377-407)   ← ★the real heart of integration
```

### Integration hook ① coordinate transform (JSBSim ECEF ↔ Unreal/Cesium)

JSBSim computes in geocentric **ECEF/geodetic (WGS84)**; Unreal uses local cm coordinates (left-handed). The
`GeoReferencing` plugin's `AGeoReferencingSystem` bridges the gap:

- `ECEFToEngine()` / `EngineToECEF()` — ECEF ↔ UE
- Builds an **East-South-Up (ESU)** tangent frame (cpp:88; `-North` because UE is left-handed)
- Heading correction `Yaw -= 90` (cpp:382) — JSBSim heading 0 = north, UE 0 = east
- After a CG-offset correction, finally calls `Parent->SetActorLocationAndRotation()` to actually move the aircraft
- If the result is NaN → `CrashedEvent()`

> The aircraft sitting at the correct lat/lon on the Cesium terrain is thanks to this transform. (Cesium's georeference
> and this GeoReferencing system must share the same origin.)

### Integration hook ② terrain awareness (UEGroundCallback)

For landing/altitude (AGL), JSBSim asks "where is the ground beneath me?" Normally it uses its own ellipsoid, but this
plugin intercepts via `UEGroundCallback` and **raycasts into the Unreal world**:

```
JSBSim FGInertial → UEGroundCallback::GetAGLevel(ECEF location)
   → MovementComponent->GetAGLevel()  (cpp:274)
      → convert ECEF to UE → World->LineTraceSingleByObjectType() (hits terrain/Cesium tiles)
      → convert the hit point/normal back to ECEF and return to JSBSim
```

This is **how JSBSim physics "sees" the Unreal/Cesium terrain**. To avoid the AGL query starting below ground, it
starts `AGLThresholdMeters=15` above the point.

### Integration hook ③ generic property access

An escape hatch to read/write *any* JSBSim property by string, from Blueprint:
- `PropertyManagerNode(Catalog)` — list all properties
- `CommandConsole("gear/unit/wheel-speed-fps", InVal, OutVal)` / `CommandConsoleBatch(...)` — get/set any property

### Minor pieces

- **Module/logging**: `JSBSimModule.cpp` is an empty module + the `LogJSBSim` category. There's also an `LStream` to redirect `std::cout` into UE_LOG (disabled by default).
- **Build trick**: JSBSim headers trip UE's "warnings = errors" policy, so includes are wrapped in `#pragma warning(push/disable/pop)` (cpp:12-51).
- **Editor**: `PostEditChangeProperty` reloads the aircraft when AircraftModel changes; `JSBSimMovementCompVisualizer` draws CG/Eye/gear points in the editor.

---

## The whole thing in one picture (with MUMT_Sim context)

```
[UDP 5005/5010] → BVRGymAutopilot(PID) ─┐
[joystick/keyboard]  ─────────────────────┤  write
                                          ▼
        UJSBSimMovementComponent.Commands / EngineCommands   (FDMTypes mirror structs)
                                          │ CopyToJSBSim() — write properties
                                          ▼
   ┌──────────────── JSBSim libJSBSim.a (FGFDMExec) ────────────────┐
   │  FCS→Aero→Propulsion→GroundReactions→Aircraft→Accel→Propagate   │
   │         ▲ ground query                                          │
   └─────────┼──────────────────────────────────────────────────────┘
             │ UEGroundCallback → World LineTrace (Cesium terrain)
                                          │ CopyFromJSBSim() — read properties
                                          ▼
        UJSBSimMovementComponent.AircraftState (ECEF/lat-lon/attitude/speed)
                                          │ GeoReferencing: ECEF→UE, Yaw-90
                                          ▼
        Parent Actor.SetActorLocationAndRotation()  → flies on screen
                                          │
                                          └→ (UDPControlReceiver reads AircraftState, sends state out on 5006)
```

> **Summary:** JSBSim itself is the standalone "XML aircraft + property tree + model pipeline" library. The Unreal
> integration is a single `UJSBSimMovementComponent` that (a) mirrors inputs in → (b) Run()s at 120Hz → (c) mirrors
> outputs out → (d) converts ECEF→UE via GeoReferencing to move the actor, and (e) raycasts the Cesium terrain via
> GroundCallback.

---

## Key file paths

| Role | Path |
|---|---|
| Main binding component (header) | `Source/JSBSimFlightDynamicsModel/Public/JSBSimMovementComponent.h` |
| Main binding component (impl) | `Source/JSBSimFlightDynamicsModel/Private/JSBSimMovementComponent.cpp` |
| Blueprint mirror structs + unit macros | `Source/JSBSimFlightDynamicsModel/Public/FDMTypes.h` |
| Ground-query callback | `Source/JSBSimFlightDynamicsModel/Private/UEGroundCallback.cpp` (+ Public/.h) |
| Runtime module | `Source/JSBSimFlightDynamicsModel/Private/JSBSimModule.cpp` |
| ThirdParty link config | `Source/ThirdParty/JSBSim.Build.cs` |
| JSBSim headers | `Source/ThirdParty/JSBSim/Include/` (FGFDMExec.h, models/*.h …) |
| JSBSim binary | `Source/ThirdParty/JSBSim/Lib/Linux/libJSBSim.a` |
| Aircraft/engine/system data | `Resources/JSBSim/{aircraft,engine,systems}/` |
| F-16 definition | `Resources/JSBSim/aircraft/f16/f16.xml` |
