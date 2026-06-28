# The F-16 Actor in Unreal (Pawn structure & control flow)

> Scope: how the F-16 aircraft actor you placed in the level is built, what components and
> variables it has, and how it is driven (keyboard, UDP, autopilot) and observed (state out).
> Blueprints are binary `.uasset`, so structure here is extracted from asset strings, the text
> export `f16_copy.T3D`, and the C++ that drives the pawn — logic-level details are inferred from
> naming where the graph itself is binary.
>
> - Date: 2026-06-28 (git `25c5459`)
> - Related: [ARCHITECTURE.md](ARCHITECTURE.md) (whole project), [README_JSBSim.md](README_JSBSim.md) (flight model plugin)

---

## 0. TL;DR — there are TWO F-16 lineages

| Lineage | Assets | Flight model | Role |
|---|---|---|---|
| **JSBSim aircraft** (the real one) | `Content/Blueprints/M_F16`, `F16_UAV` | `UJSBSimMovementComponent` (true 6-DoF FDM) | What the project actually flies. `M_F16` = player, `F16_UAV` = remotely/AI controlled UAV |
| **Legacy arcade sample** | `Content/F16Control/blueprint/f16_copy`, `BP_GM`, `BP_Pilot`, `BP_Bullet`, `BP_rocket`, `UMG_menu` | Simple `AddActorWorldOffset`/rotation, **no JSBSim** | Old marketplace-style sample. Only its **art assets are reused** (mesh `sk_Jet`, anim `AB_Jet`, weapons `BP_Bullet`/`BP_rocket`) |

`Content/Blueprints/F_16.uasset` and `BP_Airliner.uasset` are **ObjectRedirectors that point to `M_F16`** (leftover renames) — not separate aircraft.

The text export `f16_copy.T3D` is an export of the **legacy arcade** pawn (it has `ForwardMovement`, `RotationControl`, `ShootBullet`, input axes `speed/yaw/pitch/roll`). It is useful to understand the legacy sample, **not** the JSBSim aircraft.

So when this doc says "the F-16 actor," it means the **JSBSim pawns `M_F16` / `F16_UAV`**.

---

## 1. Component hierarchy of the JSBSim F-16 pawn

`M_F16` is a Pawn (Blueprint) made of these components:

```
M_F16 (APawn, BlueprintGeneratedClass /Game/Blueprints/M_F16)
│
├── SkeletalMeshComponent  "Jet"      → mesh  /Game/F16Control/meshes/sk_Jet
│                                       → anim  /Game/F16Control/animation/AB_Jet  (AnimBlueprint)
│                                         (drives control-surface & gear animation from flight state)
│
├── UJSBSimMovementComponent           → the 6-DoF flight dynamics model (see README_JSBSim.md)
│                                         exposes .Commands (inputs) and .AircraftState (outputs)
│
├── SpringArmComponent                 → orbit boom (TargetArmLength) for chase camera
│   └── CameraComponent                → pilot/chase view (orbit via CameraOrbitRight/Up/Radial)
│
├── AudioComponent                     → engine sound (auto-activate)
├── ArrowComponent                     → editor gizmo / forward indicator
├── StaticMeshComponent                → weapon visual (sm_rocket mount)
└── WidgetComponent → UMG_BasicPrimaryFlightDisplay   (PFD HUD; "추가 Visualisation")
```

`F16_UAV` is a **leaner copy of the same skeleton**: SkeletalMesh `sk_Jet` + `UJSBSimMovementComponent` + SpringArm/Camera + PFD widget. It **drops the weapon/team payload** (no `BulletAmmo`/`RocketAmmo`/`Team` variables — see next section).

> The flight model itself lives entirely in `UJSBSimMovementComponent`. Everything else on the pawn is presentation (mesh, camera, audio, HUD) or gameplay payload (weapons/team). The pawn is essentially a **carrier for the JSBSim component plus a variable surface that external code reads/writes**.

---

## 2. Blueprint variables (the contract with C++)

The pawn exposes plain Blueprint variables that the native `AUDPControlReceiver` reads and writes **by name via reflection** (`FindPropertyByName`). This is the actual C++↔Blueprint interface.

### Inputs written INTO the pawn (command path)
| Variable | Type | Who writes | Meaning |
|---|---|---|---|
| `UDP_Roll` | float/double | C++ `ApplyControlCommandToPawn` | roll command from UDP JSON |
| `UDP_Pitch` | float/double | C++ | pitch command from UDP JSON |
| `UDP_Yaw` | float/double | C++ | yaw command from UDP JSON |
| `UDP_Throttle` | float/double | C++ | throttle command from UDP JSON |

Both `M_F16` and `F16_UAV` define `UDP_Roll/Pitch/Yaw/Throttle`. The pawn's own Blueprint graph is expected to read these each tick and push them into `JSBSimMovementComponent.Commands` / `EngineCommands`.

### Outputs read FROM the pawn (state path, all optional)
`AUDPControlReceiver::BuildPawnState` reads these **optionally** (null if the variable is absent on that pawn). Variable names are configurable UPROPERTYs on the receiver (defaults shown):
| State JSON key | Pawn variable (default) | Present on M_F16 | Present on F16_UAV |
|---|---|---|---|
| `team` | `Team` | ✅ | ❌ |
| `weapons.bullet_ammo` | `BulletAmmo` | ✅ | ❌ |
| `weapons.rocket_ammo` | `RocketAmmo` | ✅ | ❌ |

Other gameplay variables present on `M_F16` (referenced by the receiver's UPROPERTY name-mapping but read only where used): `isFiringBullet`, `RocketSpawnedID`, `shootingSpeed`, `Target`, `TargetArray`, `Throttle`. `LockTarget`/`IsLocked`/`isDead` mapping names exist on the receiver but the corresponding pawn variables are not all present.

> **Key consequence:** `M_F16` is the **full player aircraft** (flight + weapons + team + targeting). `F16_UAV` is a **flight-only remote aircraft** (flight + UDP vars + Target/TargetArray, no weapons/team). The state JSON for a UAV therefore reports `team`/ammo as `null`.

---

## 3. How the F-16 is driven — three control paths

There are **three independent ways** commands reach the JSBSim component, all addressed to a specific pawn:

```
(A) KEYBOARD  (player, M_F16)
    Input Action/Axis events in the BP graph
      Aileron/Elevator/Rudder, Throttle W/S, trims, Flaps Extend/Retract,
      Throttle Cut/Full, CycleMagnetos, ToggleEngines Running/Starters/Mixture/CutOff,
      SwitchCamera, ToggleDebugInfo, shoot_bullet (LMB), shoot_rocket (RMB)
        └─► JSBSimMovementComponent.Commands / EngineCommands

(B) UDP JSON  (remote, NAME-ADDRESSED)   — INDIRECT, via Blueprint variables
    AUDPControlReceiver::ReceiveUDPData (port 5005)
      → ParseJsonCommand  {commands:[{aircraft_name,roll,pitch,yaw,throttle}]}
        keyed into TMap<name,FRemoteControlCommand> NamedControlCommands
      → each tick: for every controlled pawn, apply the command whose aircraft_name
        is contained in the pawn's instance name (PawnName.Contains(key))
      → ApplyControlCommandToPawn(pawn)
          SetBlueprintNumber(pawn,"UDP_Roll"/"UDP_Pitch"/"UDP_Yaw"/"UDP_Throttle")
        └─► pawn BP graph reads UDP_* ──► JSBSimMovementComponent.Commands

(C) AUTOPILOT / SETPOINT  (remote, PER-UAV)   — DIRECT, bypasses Blueprint variables
    AUDPControlReceiver::ReceiveSetpointData (port 5010, JSON {aircraft_name,...})
      → store in TMap<name,FUavSetpoint> Setpoints  (latest-wins per aircraft)
      → AutopilotTick (60 Hz) loops setpoints, matches pawn by name, per-UAV FAircraftAutopilot
        (cascade PID: heading→bank→aileron, alt→pitch→elevator, speed→autothrottle)
      → ApplyAutopilotToPawn(pawn, name, setpoint)
          FindComponentByClass<UJSBSimMovementComponent>()
        └─► JSBSim->Commands.Aileron/Elevator/Rudder, EngineCommands[0].Throttle  (direct)
```

Which pawn each path hits — **everything is name-addressed by the pawn instance name** (`Pawn->GetName()`, e.g. `F16_UAV_C_2`, `M_F16_C_1`):
- The receiver finds candidate pawns **by name substring**: `ControlledPawnNamePatterns = {"F16_UAV","UAV","M_F16"}`, capped at `MaxControlledUavs = 4`. Note `M_F16` is now controllable too (joystick → manned).
- Path (B) applies **only name-matched commands** — a pawn responds solely to a command whose `aircraft_name` is contained in its instance name. There is **no positional-index or broadcast fallback**, so independent senders (joystick → manned `M_F16`, controller/BT → `F16_UAV`) can share the topic without cross-applying one vehicle's command to another.
- Path (C) drives **every UAV that has a setpoint**, each with its own `FAircraftAutopilot` (separate PID/hysteresis state, created lazily on first setpoint). A setpoint is matched to a pawn by **exact name first**, then a **unique substring** match (ambiguous matches are skipped and logged). The setpoint carries `{aircraft_name, heading_deg, altitude_m, throttle_norm, target_speed_mps, launch_missile}`; the autopilot outputs aileron/elevator and a throttle that is the **autothrottle** speed-hold output when `target_speed_mps > 0`, otherwise the open-loop `throttle_norm`.

> **Coordinate frame:** the world is `x = East, y = South, z = Up`, in cm (the JSBSim plugin's ESU tangent frame). Altitude both reported in state (`z`) and used by the autopilot is `GetActorLocation().Z / 100` (UE-Z metres), **not** JSBSim ASL — so `altitude_m` setpoints and `z/100` share one frame.

> **Important asymmetry:** Path (B) needs the pawn to have `UDP_*` variables AND a BP graph that forwards them. Path (C) needs nothing but a `UJSBSimMovementComponent` — it reaches into the component directly. The player's `M_F16` is flown by keyboard (A) or by a joystick whose commands come in over (B) addressed to `M_F16`; UAVs are flown remotely by (B) or (C).

---

## 4. State output (the F-16 → outside world)

`AUDPControlReceiver::SendStateToPython` runs on a timer (`StateSendInterval = 0.05s`, default 20 Hz; some levels override it lower) and, for every pawn matching `ObservedPawnNamePatterns = {"F16","UAV"}`, builds one JSON object via `BuildPawnState`:

```json
{ "message_type": "aircraft_state_batch", "count": N,
  "aircraft": [
    { "aircraft_name": "<Pawn->GetName(), e.g. F16_UAV_C_2 / M_F16_C_1>",
      "x": <UE cm, East>, "y": <UE cm, South>, "z": <UE cm, Up>,
      "speed_mps": <TotalVelocityKts * KnotToMetersPerSecond>,
      "pitch": <deg>, "roll": <deg>, "yaw": <deg>,     // from JSBSim AircraftState.LocalEulerAngles
      "throttle": <EngineCommands[0].Throttle>,
      "team": <Team var | null>,
      "weapons": { "bullet_ammo": <BulletAmmo | null>, "rocket_ammo": <RocketAmmo | null> } } ] }
```

Sent over UDP to `PythonIP:PythonStatePort = 127.0.0.1:5006`. Position is the **Unreal actor transform** (cm); attitude/speed come from the **JSBSim component**. The `aircraft_name` is the pawn instance name — it is the **routing key** that name-addressed commands (5005) and setpoints (5010) are matched against, so consumers should echo back the exact name they see here. The state-frame timer runs in game time and currently oversamples the ~60 Hz sim frame (raw send ≈180–200 Hz, ~3 duplicate frames each; effective distinct rate ≈60 Hz).

---

## 5. Animation & HUD

- **`AB_Jet` (AnimBlueprint on `sk_Jet`)**: drives the visual control surfaces (elevator/ailerons/rudder/flaps) and landing gear. It is expected to read `JSBSimMovementComponent.AircraftState` (surface positions in degrees, e.g. `ElevatorPosition`, `LeftAileronPosition`) and the gear `NormalizedPosition` — so the model surfaces move with the simulated FDM.
- **`UMG_BasicPrimaryFlightDisplay` (PFD widget)**: shows Calibrated Air Speed (kts), Altitude AGL/ASL (ft), heading, and an artificial horizon, fed from `AircraftState`. Shared widget class across both pawns (not per-vehicle instanced).

---

## 6. Game-mode wiring (how the player gets an F-16)

```
BP_JSBSimGameMode  (DefaultPawnClass = M_F16, PlayerControllerClass = BP_JSBSimPlayerController)
        │
        └─ on RL_30.umap (the Cesium world) — but note: DefaultEngine.ini still points
           GlobalDefaultGameMode at /Script/AirSim.AirSimGameMode and GameDefaultMap at the
           OpenWorld template (stale). The intended game mode is set per-map / in World Settings.

BP_JSBSimPlayerController  — possesses M_F16, adds Time-of-Day controls (TOD Dawn/Dusk/Noon)
F16_UAV                    — spawned at runtime via the "SpawnUAV" input action (P key), not possessed
```

A `UJSBSimMovementComponent` **requires an `AGeoReferencingSystem` actor in the level** (it logs an error and won't fly without one). On `RL_30` that georeference is shared with Cesium so the aircraft sits at the correct lat/lon on the tileset.

---

## 7. Legacy F16Control content (for reference, not the flight path)

`Content/F16Control/` is the old arcade sample whose **art is reused** by the JSBSim pawns:
- `blueprint/f16_copy` — arcade flight pawn (no JSBSim); `BP_GM`, `BP_Pilot`, `UMG_menu` — its game mode/menu; `BP_Bullet`, `BP_rocket` — weapon projectiles (these ARE referenced by `M_F16`'s shooting logic).
- `meshes/sk_Jet` (+ skeleton, physics asset), `animation/AB_Jet`, `meshes/sm_rocket` — the shared F-16 art.

---

## 8. Notes / gaps worth knowing

- **No C++ Pawn base class.** The F-16 is a pure Blueprint pawn; the only native code is `AUDPControlReceiver` (a separate level actor) and the JSBSim plugin component. All glue (reading `UDP_*`, forwarding to `Commands`) lives in the Blueprint graph.
- **Two F-16s with overlapping names.** `ObservedPawnNamePatterns={"F16","UAV"}` matches both `M_F16` and `F16_UAV`, so the player aircraft is also reported in the state batch.
- **Stale redirectors.** `F_16` and `BP_Airliner` redirect to `M_F16` — treat as aliases.
- **Friendly/enemy not modeled.** `Team` exists only as an optional string variable on `M_F16`; there is no enemy AI, damage, or lock logic wired into the JSBSim pawns. `Target`/`TargetArray` feed the camera, not combat.
- **Per-vehicle PFD not instanced** — both pawns share one PFD widget class.

---

## 9. Key file paths

| Item | Path |
|---|---|
| Player F-16 pawn | `Content/Blueprints/M_F16.uasset` |
| UAV F-16 pawn | `Content/Blueprints/F16_UAV.uasset` |
| Redirectors → M_F16 | `Content/Blueprints/F_16.uasset`, `Content/Blueprints/BP_Airliner.uasset` |
| Game mode / player controller | `Content/Blueprints/BP_JSBSimGameMode.uasset`, `BP_JSBSimPlayerController.uasset` |
| PFD HUD | `Content/Blueprints/PFD/UMG_BasicPrimaryFlightDisplay.uasset` |
| Mesh / anim / skeleton | `Content/F16Control/meshes/sk_Jet.uasset`, `animation/AB_Jet.uasset`, `meshes/sk_Jet_Skeleton.uasset` |
| Weapons | `Content/F16Control/blueprint/BP_Bullet.uasset`, `BP_rocket.uasset`, `meshes/sm_rocket` |
| Native driver (controls + state I/O) | `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` (+ `Public/UDPControlReceiver.h`) |
| Autopilot (PID) | `Source/MUMT_Sim/Public/BVRGymAutopilot.h`, `Private/BVRGymAutopilot.cpp` |
| Flight model component | `Plugins/JSBSimFlightDynamicsModel/.../JSBSimMovementComponent.{h,cpp}` |
| Legacy arcade export (reference) | `Content/Blueprints/f16_copy.T3D` |
