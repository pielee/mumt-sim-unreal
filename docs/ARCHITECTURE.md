# MUMT_Sim Architecture (current structure + gap analysis vs target)

> Subject: a BVR (Beyond Visual Range) air-combat simulator built on UE5.4 + JSBSim + Cesium.
> Every port, topic, and field name below was confirmed from the actual source. Missing / stubbed /
> hardcoded items are flagged honestly.
>
> - Analysis date: 2026-06-23 (git `7632f63`)
> - Target architecture: two Mission Autonomy blocks (friendly BT, enemy BT) + a central ROS-Unreal
>   block (SUB `/{ns}/cmd`, PUB `/{ns}/state`, plus extra visualisation) + a Joystick (Thrustmaster →
>   `/{manned_vehicle}/cmd`). Backbone = per-vehicle namespaced ROS2 topics.

---

## 1. The current structure at a glance (actual data flow)

The real backbone today is **raw UDP, not ROS topics**. Commands flow in two ways (JSON control / binary
setpoint); state flows out one way (JSON batch).

```
                              ┌─────────────────────────────────────────────────────────────┐
                              │                  ROS2 side (optional, partial)                │
   /aircraft/setpoint ───────▶│  bridge_node.py (node "mumt_bridge")                         │
   (custom_msgs/AircraftSetpoint)  _on_setpoint: json.dumps({aircraft_name,...}) ┐             │
                              │                                                  │             │
   /mumt/aircraft_commands ──▶│  _on_command: JSON UTF-8 passthrough ──┐         │             │
   (std_msgs/String, JSON)    │                                        │         │             │
                              │  /mumt/aircraft_states ◀── recv 5006 ──┼───┐     │             │
   (std_msgs/String, JSON) ◀──│  (50Hz drain)                          │   │     │             │
                              └────────────────────────────────────────┼───┼─────┼─────────────┘
                                                                        │   │     │
        Scripts/control_sender.py (NOT a joystick; autonomous UDP loop) │   │     │
        recv 0.0.0.0:5006 (state) ── control law ── send 127.0.0.1:5005 ┤   │     │ (both to the
        {"commands":[{aircraft_name,roll,pitch,yaw,throttle}]}          │   │     │  same UDP port)
                                                                        ▼   │     ▼
   ╔════════════════════════════════════════════════════════════════════════╪══════════════╗
   ║  Unreal Engine 5.4  (AUDPControlReceiver, Source/MUMT_Sim)              │              ║
   ║   ListenPort 5005      ◀── JSON control RX  {roll,pitch,yaw,throttle}   │              ║
   ║                            or {commands:[{...,aircraft_name?}]}         │              ║
   ║   SetpointListenPort 5010 ◀── JSON per-UAV setpoint RX ──────────────────┘              ║
   ║        {aircraft_name, heading_deg, altitude_m, throttle_norm, launch_missile}         ║
   ║        → TMap<name,FUavSetpoint> Setpoints / TMap<name,FAircraftAutopilot> Autopilots  ║
   ║          │  60Hz AutopilotTick — loops every setpoint, matches pawn by name            ║
   ║          ▼                                                                             ║
   ║   FBVRGymAutopilot (cascade PID)  heading→bank(BankGain0.8,RollMax80°)→Aileron         ║
   ║                                   altitude→pitch(PitchPID)→Elevator                    ║
   ║          ▼                                                                             ║
   ║   M_F16 / F16_UAV pawn ─▶ UJSBSimMovementComponent.Commands / EngineCommands           ║
   ║          ▼  120Hz fixed substep                                                        ║
   ║   JSBSim FGFDMExec (f16) ──▶ ECEF/geodetic ──(GeoReferencing)──▶ UE coordinates        ║
   ║          ▼                                                                             ║
   ║   Cesium World Terrain (RL_30.umap) + PFD(UMG_BasicPrimaryFlightDisplay)               ║
   ║          ▼  StateSendInterval 0.05s(20Hz) ─▶ 127.0.0.1:5006 ───────────────────────────╫─▶ (recv above)
   ║        {"message_type":"aircraft_state_batch","count":N,                               ║
   ║         "aircraft":[{aircraft_name,x,y,z,speed_mps,pitch,roll,yaw,throttle,            ║
   ║                      team,weapons:{bullet_ammo,rocket_ammo}}]}                         ║
   ╚════════════════════════════════════════════════════════════════════════════════════════╝

   Manual flight: keyboard/mouse only (DefaultInput.ini). No joystick/Thrustmaster bindings.
```

**Key point:** state DOES flow out of UE (UDP 5006, JSON batch). But when republished to ROS it goes onto a
single global topic `/mumt/aircraft_states`, not `/{ns}/state`.

---

## 2. Layer-by-layer detail

### (a) Unreal / C++ — `Source/MUMT_Sim`

**AUDPControlReceiver** (ports are hardcoded defaults in the `.h`):
- `ListenPort = 5005` — JSON control RX. Accepts two schemas:
  - top-level `{roll, pitch, yaw, throttle}` (broadcast to all)
  - `{commands:[{roll,pitch,yaw,throttle,aircraft_name?}, ...]}` (per-name/index; matched via `FindTargetPawns`)
- `SetpointListenPort = 5010` — **JSON per-UAV** setpoint RX `{aircraft_name, heading_deg, altitude_m, throttle_norm, launch_missile, reset?}`. Stored in `TMap<name,FUavSetpoint> Setpoints` (latest-wins per aircraft); `reset:true` drops that name's controller. Applied on the 60Hz `AutopilotTick`, which loops the map and drives each named pawn via its own `FAircraftAutopilot` (`Autopilots` map). A `{setpoints:[...]}` batch form is also accepted.
- `PythonIP = 127.0.0.1`, `PythonStatePort = 5006`, `StateSendInterval = 0.05` (20Hz) — state out. JSON fields: `aircraft_name, x, y, z (UE cm), speed_mps (knots×0.514444), pitch/roll/yaw (deg), throttle, team, weapons:{bullet_ammo, rocket_ammo}`. team/weapons read optionally from pawn variables.

**FBVRGymAutopilot** — cascade PID control law:
- Inputs: setpoint `heading_deg`, `altitude_m`, `throttle_norm`.
- Heading error → `RollPID` (+`RollSecPID`, `BankGain=0.8`, `RollMax=80°`) → target bank → `AileronCmd = Clamp(-Pid.Update(Diff), -1, 1)`.
- Altitude → target pitch (ThetaRef) → `PitchPID` → `ElevatorCmd`.
- Output `{AileronCmd, ElevatorCmd, 0}` (yaw=0) + throttle pushed to the pawn.
- Heading wrap: `((target-current+180)%360)-180`, `RollCircleClip`.

> Summary: **the autopilot lives inside UE C++**; what arrives from outside is either a high-level setpoint (heading/alt/throttle) or low-level surface commands (roll/pitch/yaw/throttle). I.e. the target diagram's "Action-level Autonomy" already exists, embedded in UE.

### (b) JSBSim flight dynamics — `Plugins/JSBSimFlightDynamicsModel`

- Pure in-process FDM. **No ROS/UDP/sockets** — all external I/O is the MUMT_Sim module's job.
- Control inputs `Commands` (FFlightControlCommands, BlueprintReadWrite): Aileron/Elevator/Rudder/YawTrim/PitchTrim/RollTrim/Steer ∈[-1,1], brakes/Flap/SpeedBrake/Spoiler/GearDown ∈[0,1]. **Throttle is a per-engine `EngineCommands` TArray**, not in Commands. Injected through FCS (`SetDaCmd/SetDeCmd/SetDrCmd(-Rudder)/SetThrottleCmd(i,…)`). Note sign flips on rudder/yawtrim.
- State outputs `AircraftState` (FAircraftState): surface deg, CAS/ground/total kts, VelocityNED, Altitude ASL/AGL ft, ECEF (m), lat/lon deg, LocalEulerAngles (Yaw/Pitch/Roll deg), EulerRates.
- Tick: a self-managed fixed-step accumulator (**120Hz, dt=1/120**). It does not use UE's FixedFrameRate, so full determinism requires fixing the game tick (acknowledged in a code comment). DefaultEngine.ini has `FixedFrameRate=200` but `bUseFixedFrameRate=False`.
- Coordinate transform: `AGeoReferencingSystem` (Epic GeoReferencing plugin, **hard dependency**) for ECEF↔engine. Tangent frame is East-South-Up (UE left-handed), heading correction `Yaw -= 90` (JSBSim 0=north, UE 0=east). Not Cesium-specific (works against any collidable terrain via `UEGroundCallback` line trace).
- Aircraft model: `AircraftModel` (FString, set in editor/BP). Data root is the plugin's `Resources/JSBSim` (the project-root `Config/JSBSim` is a separate folder holding only control.json).
- Gaps: much of the atmosphere/wind coupling is commented out (stub), crash detection is NaN-only, AGL raycast length/threshold (15m)/WGS84 constants hardcoded. **One component = one aircraft = one Actor**; no namespacing / multi-UAV interface.

### (c) ROS2 — `ros2/` (mumt_ros_bridge + custom_msgs)

- **The only implemented node**: `MumtBridgeNode` in `bridge_node.py` (node name `mumt_bridge`, console_script `bridge`). A pure UDP↔ROS relay.
  - SUB `/mumt/aircraft_commands` (String/JSON) → UDP 5005
  - SUB `/aircraft/setpoint` (custom_msgs/AircraftSetpoint) → JSON `{aircraft_name,...}` → UDP 5010
  - PUB `/mumt/aircraft_states` (String/JSON) ← UDP 5006 (50Hz drain)
  - Ports are ROS params (`unreal_ip/control_port/state_port/setpoint_port`) but **no launch/yaml overrides them**, so all default to 127.0.0.1.
- `custom_msgs/AircraftSetpoint.msg`: `aircraft_name (string), heading_deg, altitude_m, throttle_norm (float32), launch_missile (bool)`. `aircraft_name` is the per-UAV address: every BT (one per UAV) tags its own name so the single shared `/aircraft/setpoint` topic carries all UAVs' commands and UE routes each to the right pawn.
- **Behavior Tree: XML only, zero node implementations.** `behavior_trees/mumt_autopilot.xml` is BehaviorTree.CPP syntax (`main_tree_to_execute=MUMT_Autopilot`, Sequence: HasAircraftState → SelectControlledAircraft → Selector[RecoverIfBadAttitude, ParallelFlightControl] → PublishCommands). None of those node classes exist in Python or C++. **No generic MoveTo-style nodes, no blackboard keys.**
- launch: `bt_controller.launch.py` tries to start `executable='bt_controller'`, but setup.py only registers `bridge` → **launch fails ("executable not found")**.
- `bt_controller.yaml` holds detailed gains (pitch/roll/parallel-formation) that nothing reads.
- Framework mismatch: `package.xml` depends on `py_trees`, the XML is BehaviorTree.CPP syntax.

### (d) Input / control

- `Scripts/control_sender.py`: **not a joystick reader.** A 2-socket UDP autonomous control loop — recv `0.0.0.0:5006` (state) → compute → send `127.0.0.1:5005` `{"commands":[{aircraft_name,roll,pitch,yaw,throttle}]}`. Control law: `target_pitch=5°`, `throttle=1.0 if speed<150 else 0.6`, `pitch_cmd=clamp((5-pitch)*0.1,-1,1)` then negated; roll/yaw=0. Only aircraft whose name contains "UAV", first 2 only. ROS not used.
- `Config/JSBSim/control.json`: static defaults `{throttle:0.7, aileron/elevator/rudder:0}`. Field names differ from the UDP schema (roll/pitch/yaw) → no shared message definition.
- `Config/DefaultInput.ini`: flight control is **keyboard/mouse only** (Aileron Q/E, Elevator PgUp/PgDn, Rudder A/D, Throttle W/S, shoot_bullet LMB, shoot_rocket RMB, SpawnUAV P, gear G, brake Space). **No Thrustmaster/HOTAS/joystick bindings.** Gamepad/VR appear only as AxisConfig sensitivity defaults.

### (e) UE Content / Cesium

- Two live maps: `RL_30.umap` (the main Cesium world: Cesium3DTileset = World Terrain, CesiumGeoreference `DEFAULT_GEOREFERENCE`, CesiumIonServer = CesiumIonSaaS, `F16_UAV` placed), `RL_2.umap` (Epic GeoReferencing, non-Cesium).
- Pawns: `M_F16` (player F-16, JSBSimMovementComponent + PFD + weapons + `UDP_Pitch/Roll/Yaw/Throttle`), `F16_UAV` (spawned at runtime via SpawnUAV=P, `UDP_Pitch/Roll/Yaw/Throttle`). `F_16`·`BP_Airliner` are **ObjectRedirectors to M_F16** (not separate aircraft). See [README_F16_Actor.md](README_F16_Actor.md).
- Game mode: `BP_JSBSimGameMode` (PlayerController=BP_JSBSimPlayerController, DefaultPawn=M_F16) is what's actually used. But **DefaultEngine.ini has GameDefaultMap=`/Engine/Maps/Templates/OpenWorld`, GlobalDefaultGameMode=`/Script/AirSim.AirSimGameMode` (stale)** — inconsistent with RL_30/BP_JSBSimGameMode (misconfigured startup).
- PFD: `UMG_BasicPrimaryFlightDisplay` (CalibratedAirSpeedKts, AltitudeAGL/ASLFt, heading, horizon). Shared by both pawns, **not per-vehicle instanced**.
- **No enemy/friendly, team, target-lock, or health logic.** `F16_UAV`'s Target/TargetArray feed the SpringArm camera. No enemy AI/damage/seeker.
- Cesium ion token (JWT) is committed inside the `CesiumIonSaaS.uasset` binary (a secret exposed in source control).

---

## 3. Current vs target architecture — gap analysis

| Target element | Current state | Verdict |
|---|---|---|
| **Namespaced ROS topic `/{ns}/cmd`** | None. cmd is global `/mumt/aircraft_commands` or raw UDP 5005. Multi-aircraft is multiplexed by a JSON `aircraft` array / `aircraft_name`, not a ROS namespace | **Missing** (backbone is raw UDP) |
| **UE → ROS `/{ns}/state` publish** | State out exists (UE→UDP 5006→bridge→ROS), but on a single global `/mumt/aircraft_states` | **Partial** (no namespacing) |
| **BT model (friendly/enemy)** | XML/yaml/launch only. No node classes, executable, or blackboard. launch fails. Not connected to the bridge | **Missing (shell only)** |
| **Action-level Autonomy (MoveTo, open source)** | No generic MoveTo/Nav2. Instead UE C++ `FBVRGymAutopilot` turns heading/alt/throttle setpoints into surface commands via PID | **Partial** (autonomy is inside UE) |
| **Friendly vs enemy split** | No team distinction in pawns/blueprints. `team` is an optional output field only; no enemy AI / state subscription | **Mostly missing** |
| **Joystick (Thrustmaster) → `/{manned}/cmd`** | No joystick/HOTAS binding or node. Manual flight is keyboard/mouse only. control_sender.py is not a joystick either | **Fully missing** |
| **Extra Visualisation (PFD)** | `UMG_BasicPrimaryFlightDisplay` exists and works (shared widget) | **Present (partial)** |
| **ROS-Unreal single block (SUB cmd / PUB state)** | Effectively implemented via the UDP bridge (5005 cmd-in, 5006 state-out, 5010 setpoint-in). ROS is a thin global-topic relay | **Partial** (different topic model) |

**Direct answers**
- Is cmd a namespaced ROS topic? → **No** (global topics + raw UDP).
- Is there a UE→ROS `/{ns}/state` publish? → state-out path exists, but as a single global topic.
- Is the BT connected to the bridge? → **No** (no node implementations, can't run).
- Friendly/enemy split? → **None** (team is an optional output field only).
- Joystick wired? → **No** (keyboard/mouse only).
- Extra viz (PFD)? → works, but as a shared (not per-vehicle) widget.

---

## 4. Key observations & next steps

### Most important facts (5)
1. **The real backbone is raw UDP, not ROS** (5005 cmd-in / 5006 state-out / 5010 setpoint-in). The ROS bridge is a thin global-topic relay on top. The target's "ROS-Unreal SUB cmd/PUB state" is already satisfied at the transport level — **only the topic model differs**.
2. **Autonomy already exists but in the wrong layer** — `FBVRGymAutopilot` runs a cascade PID (heading→bank→aileron, alt→pitch→elevator) inside UE C++. The diagram's "Action-level Autonomy" is effectively here. The BT just needs to fill in that setpoint (heading/alt/throttle).
3. **The BT is a complete shell** — XML+yaml+launch look plausible but there are zero nodes, zero executables, the launch fails, and the framework is contradictory (py_trees vs BehaviorTree.CPP).
4. **Namespacing, friendly/enemy, and joystick are all absent** — multi-aircraft is JSON-array only, team is an optional string only, manual flight is keyboard only.
5. **Plenty of stale/hardcoded debt** — DefaultEngine's OpenWorld/AirSim game-mode misconfig, ObjectRedirector "fake" aircraft, committed Cesium token, missing reset/seq in AircraftSetpoint.msg, launch_missile ignored by UE, control.json field-name mismatch.

### Prioritized roadmap
1. **(P0) Namespace the topics** — rework bridge_node to be vehicle-id aware: map UE state JSON `aircraft_name` → ns and publish per-vehicle `/{ns}/state`, subscribe per-vehicle `/{ns}/cmd` → fan-in/out over the global UDP internally. (UE side need not change yet.) This is the core of the target backbone.
2. **(P0) Actually wire the BT** — pick one framework. (a) Keep BehaviorTree.CPP: implement the node classes (HasAircraftState/SelectControlledAircraft/Recover/Parallel/Publish) + add an executable to setup.py (or a C++ ament package), subscribe `/{ns}/state` → publish `/{ns}/cmd`. Or (b) rewrite in py_trees. **Leaf actions should target UE's setpoint path (heading/alt/throttle)** — the PID already consumes it.
3. **(P1) Clarify Action-level Autonomy** — expose `FBVRGymAutopilot`'s setpoint as a `MoveTo(heading,alt,speed)` interface mapped 1:1 to BT leaves (consider Nav2/open-source if desired).
4. **(P1) Split friendly/enemy into two Mission Autonomy blocks** — run the same BT as two instances with different ns/params (friendly publishes cmd only; enemy subscribes `/{ns}/state` + publishes `/{ns}/cmd`). Promote `team` to a first-class field (currently optional).
5. **(P1) Add a joystick (Thrustmaster) node** — a joy/HID node publishing `/{manned_vehicle}/cmd` (or bind directly via UE Enhanced Input). Extend the current keyboard-only mappings to joystick axes.
6. **(P2) Multi-aircraft spawn/manage + debt cleanup** — a per-vehicle spawn manager (currently single SpawnUAV on P), per-vehicle PFD instancing, fix DefaultEngine game-mode/map to RL_30/BP_JSBSimGameMode, add reset/seq to AircraftSetpoint.msg (or implement UE-side launch_missile), externalize the Cesium token, clean up the ObjectRedirector "fake" aircraft.

---

## Appendix: key file paths

- UE C++: `Source/MUMT_Sim/Private/UDPControlReceiver.cpp`(.h), `Source/MUMT_Sim/Private/BVRGymAutopilot.cpp`(.h)
- JSBSim plugin: `Plugins/JSBSimFlightDynamicsModel/Source/JSBSimFlightDynamicsModel/Private/JSBSimMovementComponent.cpp`, `Public/FDMTypes.h`
- ROS2: `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/bridge_node.py`, `.../behavior_trees/mumt_autopilot.xml`, `.../config/bt_controller.yaml`, `.../launch/bt_controller.launch.py`, `.../setup.py`, `mumt_ros_ws/src/custom_msgs/msg/AircraftSetpoint.msg`
- Input/control: `Scripts/control_sender.py`, `Config/JSBSim/control.json`, `Config/DefaultInput.ini`, `Config/DefaultEngine.ini`
- Content/Cesium: `Content/RL_30.umap`, `Content/Blueprints/BP_JSBSimGameMode.uasset`, `.../M_F16.uasset`, `.../F16_UAV.uasset`, `.../PFD/UMG_BasicPrimaryFlightDisplay.uasset`, `Content/CesiumSettings/CesiumIonServers/CesiumIonSaaS.uasset`
