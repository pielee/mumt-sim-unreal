# MUMT_Sim Architecture (current structure + gap analysis vs target)

> Subject: a BVR (Beyond Visual Range) air-combat simulator built on UE5.4 + JSBSim + Cesium.
> Every port, topic, and field name below was confirmed from the actual source. Missing / stubbed /
> hardcoded items are flagged honestly.
>
> - Analysis date: 2026-06-28 (git working tree, MUMT_Sim HEAD `25c5459`). One key uncommitted
>   working-tree item: `BVRGymAutopilot.cpp` hard-turn hysteresis clamp (re-arm `HeadActSpace =
>   min(HeadActSpaceMin, HeadActSpaceMax)`) — fixes "UAV won't turn at corners", not yet committed.
> - Target architecture: two Mission Autonomy blocks (friendly BT, enemy BT) + a central ROS-Unreal
>   block (SUB `/{ns}/cmd`, PUB `/{ns}/state`, plus extra visualisation) + a Joystick (Thrustmaster →
>   `/{manned_vehicle}/cmd`). Backbone = per-vehicle namespaced ROS2 topics.

---

## 1. The current structure at a glance (actual data flow)

The real backbone today is **raw UDP, not ROS topics**. Commands flow in two ways (JSON control / JSON
setpoint); state flows out one way (JSON batch). UE routes both inbound paths by **aircraft name**: each
command (5005) and each setpoint (5010) is applied to the pawn whose instance name contains its
`aircraft_name`, so joystick→manned and BT→UAV coexist on the shared topics simultaneously.

```
                              ┌─────────────────────────────────────────────────────────────┐
                              │                  ROS2 side (optional, partial)                │
   /aircraft/setpoint ───────▶│  bridge_node.py (node "mumt_bridge")                         │
   (custom_msgs/AircraftSetpoint)  _on_setpoint: json.dumps({aircraft_name,...,target_speed_mps}) ┐        │
                              │                                                  │             │
   /mumt/aircraft_commands ──▶│  _on_command: JSON UTF-8 passthrough ──┐         │             │
   (std_msgs/String, JSON)    │                                        │         │             │
                              │  /mumt/aircraft_states ◀── recv 5006 ──┼───┐     │             │
   (std_msgs/String, JSON) ◀──│  (60Hz drain timer)                    │   │     │             │
                              └────────────────────────────────────────┼───┼─────┼─────────────┘
                                                                        │   │     │
        Scripts/control_sender.py (NOT a joystick; autonomous UDP loop) │   │     │
        recv 0.0.0.0:5006 (state) ── control law ── send 127.0.0.1:5005 ┤   │     │ (both to the
        {"commands":[{aircraft_name,roll,pitch,yaw,throttle}]}          │   │     │  same UDP port)
                                                                        ▼   │     ▼
   ╔════════════════════════════════════════════════════════════════════════╪══════════════╗
   ║  Unreal Engine 5.4  (AUDPControlReceiver, Source/MUMT_Sim)              │              ║
   ║   ListenPort 5005      ◀── JSON control RX  {commands:[{roll,pitch,yaw,│              ║
   ║                            throttle,aircraft_name}]} → name-matched pawn │              ║
   ║   SetpointListenPort 5010 ◀── JSON per-UAV setpoint RX ──────────────────┘              ║
   ║        {aircraft_name, heading_deg, altitude_m, throttle_norm, target_speed_mps,       ║
   ║         launch_missile}                                                                ║
   ║        → TMap<name,FUavSetpoint> Setpoints / TMap<name,FAircraftAutopilot> Autopilots  ║
   ║          │  60Hz AutopilotTick — loops every setpoint, matches pawn by name            ║
   ║          ▼                                                                             ║
   ║   FAircraftAutopilot (cascade PID)  heading→bank(RollMax80°)→Aileron                   ║
   ║                                     altitude→pitch(PitchPID)→Elevator                  ║
   ║                                     target_speed_mps→ThrottlePID→Throttle (autothrottle,║
   ║                                       open-loop throttle_norm fallback when Vtgt<=0)    ║
   ║          ▼                                                                             ║
   ║   M_F16 / F16_UAV pawn ─▶ UJSBSimMovementComponent.Commands / EngineCommands           ║
   ║          ▼  120Hz fixed substep                                                        ║
   ║   JSBSim FGFDMExec (f16) ──▶ ECEF/geodetic ──(GeoReferencing)──▶ UE coordinates        ║
   ║          ▼                                                                             ║
   ║   Cesium World Terrain (RL_30.umap) + PFD(UMG_BasicPrimaryFlightDisplay)               ║
   ║          ▼  StateSendInterval 0.05s(20Hz default; RL_2 ≈0.005s, oversamples to ~180Hz) ╫─▶ 127.0.0.1:5006 (recv above)
   ║        {"message_type":"aircraft_state_batch","count":N,                               ║
   ║         "aircraft":[{aircraft_name,x,y,z,speed_mps,pitch,roll,yaw,throttle,            ║
   ║                      team,weapons:{bullet_ammo,rocket_ammo}}]}                         ║
   ╚════════════════════════════════════════════════════════════════════════════════════════╝

   Manned flight: F710 gamepad → joy_node → mumt_joystick → /mumt/aircraft_commands (50Hz) → bridge
   → 5005 → name-matched to M_F16. (UE keyboard/mouse bindings still exist; no in-UE joystick binding.)
```

**Key point:** state DOES flow out of UE (UDP 5006, JSON batch). But when republished to ROS it goes onto a
single global topic `/mumt/aircraft_states`, not `/{ns}/state`.

---

## 2. Layer-by-layer detail

### (a) Unreal / C++ — `Source/MUMT_Sim`

**AUDPControlReceiver** (ports are hardcoded defaults in the `.h`):
- `ListenPort = 5005` — JSON control RX. Accepts two schemas, but **applies name-matched commands only**:
  - `{commands:[{roll,pitch,yaw,throttle,aircraft_name}, ...]}` — each command is keyed into `NamedControlCommands` and applied (every Tick) to the pawn whose instance name *contains* its `aircraft_name`. There is **no positional/broadcast fallback** to a pawn, so independent senders (joystick→M_F16, BT/sender→F16_UAV) never cross-apply.
  - top-level `{roll, pitch, yaw, throttle}` is still parsed (fills `BroadcastCommand`/HUD vars) but is no longer broadcast to pawns.
- `SetpointListenPort = 5010` — **JSON per-UAV** setpoint RX `{aircraft_name, heading_deg, altitude_m, throttle_norm, target_speed_mps, launch_missile, reset?}` (was binary before; now plain JSON). Stored in `TMap<name,FUavSetpoint> Setpoints` (latest-wins per aircraft); `reset:true` drops that name's controller. Applied on the 60Hz `AutopilotTick`, which loops the map and routes each setpoint to a pawn by name (exact name first; single substring match as a spawn-suffix fallback, ambiguous → skip+warn), driving it via its own `FAircraftAutopilot` (`Autopilots` map). A `{setpoints:[...]}` batch form is also accepted.
- `PythonIP = 127.0.0.1`, `PythonStatePort = 5006`, `StateSendInterval = 0.05` (20Hz default; RL_2 overrides to ≈0.005s) — state out via a game-time timer. JSON batch fields per aircraft: `aircraft_name (=Pawn->GetName(), e.g. F16_UAV_C_2), x, y, z (UE cm), speed_mps (knots×0.514444), pitch/roll/yaw (deg), throttle, team, weapons:{bullet_ammo, rocket_ammo}`. team/weapons read optionally from pawn variables. Because the timer is game-time and oversamples the ~60Hz sim, raw output is ~180–200Hz with each distinct frame re-sent ~3× (effective distinct state rate ≈60Hz).

**FAircraftAutopilot** (`BVRGymAutopilot.*`) — cascade PID control law:
- Inputs: setpoint `heading_deg`, `altitude_m`, `target_speed_mps` (autothrottle), `throttle_norm` (fallback).
- Two modes with hysteresis: **hard-turn** (`|DiffHead|>=HeadActSpace && |DiffAlt|<=AltActSpace` → bank to `RollMax=80°`) vs **precision** (pitch to a climb/dive `ThetaRef`). Heading error → `RollPID` (+`RollSecPID`) → `AileronCmd = Clamp(-Pid.Update(Diff), -1, 1)`; altitude → `PitchPID` → `ElevatorCmd`.
- **Autothrottle (speed-hold PI)**: `SetThrottlePID` runs every tick. `target_speed_mps>0` → `ThrottleCmd = Clamp(ThrottlePID.Update(V - Vtgt), 0, 1)` (slower than target → throttle up; integrator carries the trim throttle). `target_speed_mps<=0` → output −1 and the caller falls back to the open-loop `throttle_norm`. `FPID::SetGains` resets the derivator but **preserves the integrator** so the trim accumulates across ticks.
- Output `{Aileron, Elevator, 0, Throttle}` (yaw=0). Heading wrap: `((target-current+180)%360)-180`, `RollCircleClip`.
- **Working-tree fix (uncommitted):** the hard-turn re-arm now clamps `HeadActSpace = min(HeadActSpaceMin, HeadActSpaceMax)`, so a level NavParams override with Min>Max can no longer invert the hysteresis and stop the UAV turning at waypoints.

> Summary: **the low-level autopilot lives inside UE C++**; what arrives from outside is either a high-level setpoint (heading/alt/speed) or name-matched surface commands (roll/pitch/yaw/throttle). The target diagram's "Action-level Autonomy" exists here, while the *formation* autonomy now lives in the BT (§c).

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
  - SUB `/aircraft/setpoint` (custom_msgs/AircraftSetpoint) → JSON `{aircraft_name,...,target_speed_mps}` → UDP 5010
  - PUB `/mumt/aircraft_states` (String/JSON) ← UDP 5006 (drained on a 60Hz timer)
  - Ports are ROS params (`unreal_ip/control_port/state_port/setpoint_port`) but **no launch/yaml overrides them**, so all default to 127.0.0.1.
- `custom_msgs/AircraftSetpoint.msg`: `aircraft_name (string), heading_deg, altitude_m, throttle_norm, target_speed_mps (float32), launch_missile (bool)`. `aircraft_name` is the per-UAV address: each BT tags its own name so the single shared `/aircraft/setpoint` topic carries all UAVs' commands and UE routes each to the right pawn. `target_speed_mps` is the new autothrottle target (preferred over open-loop `throttle_norm`).

- **Behavior Tree: now a working clean-milestone build (no longer a shell).** It is a separate py_trees-based repo (`~/dev/py_bt_ros`, scenario `scenarios/mumt`, run via `main.py --config scenarios/mumt/configs/mumt.yaml`), not the stale `behavior_trees/mumt_autopilot.xml` shipped in the bridge package (which remains BehaviorTree.CPP-syntax dead code).
  - Tree `default_bt.xml`: `ReactiveSequence[ GatherState(own="F16_UAV", leader="M_F16") → Sequence[ WaitForLeaderTakeoff → Takeoff → MaintainFormation ] ]`. M0–M2 variants are kept commented for stepwise testing.
  - Implemented nodes (`bt_nodes.py`): `GatherState` (parses the 5006 state batch, stores own/leader/spawn-altitude, True once both are seen), `WaitForLeaderTakeoff` (idle until leader climbs ≥ threshold above spawn), `Takeoff` (hold runway heading, climb relative to spawn), `MaintainFormation` (offset slot + closure; see §autonomy below), plus `HoldSetpoint` (M0 fixed-setpoint smoke test). All publish `AircraftSetpoint` (with `target_speed_mps`) on `/aircraft/setpoint`.
  - `configs/mumt.yaml`: namespace `/F16_UAV`, `bt_tick_rate: 10.0` → setpoints publish at 10Hz. Contract: one setpoint publisher per UAV (a duplicate BT → setpoint conflict → autopilot oscillation).
  - **Formation autonomy (`MaintainFormation`)**: slot = `leader_pos − aft·fwd + lat·right` (fwd = leader-heading unit, right = 90° right). **`aft_offset_m` NEGATIVE = slot in FRONT of the leader** (config is `aft=-80, lat=-40` → front-left); positive = behind. Heading blends from slot-bearing (far) to leader heading (near) to kill weaving; altitude = `max(leader_alt+voff, spawn+MIN_AGL)`; speed enforced via `target_speed_mps` (rendezvous catch-up beyond 1500m, else leader speed + along-track closure). On a momentary state gap it re-publishes the last setpoint and stays RUNNING (never FAILURE, so the parent Sequence does not fall back to Takeoff).
  - **Coordinate/heading/altitude frames** (shared by UE and the BT): UE world is x=East, y=South, z=Up (cm). Compass heading (0=N, 90=E) = `atan2(ΔEast, ΔNorth) = atan2(Δx, −Δy)` (math-frame `atan2(Δy,Δx)` would be 90° off). Altitude is **UE `Location.Z/100` (UE-Z, m)** — the same value state `z/100` carries and the autopilot feeds back — NOT JSBSim ASL; since ground can be negative, the BT uses spawn-relative climb thresholds.

### (d) Input / control

- **Joystick (manned) — now wired (in ROS, not in UE).** A Logitech **F710** gamepad → `joy_node` → `mumt_joystick` → `/mumt/aircraft_commands` (50Hz) → bridge → UDP 5005 → name-matched to the manned `M_F16`. One-shot launch: `ros2 launch mumt_ros_bridge manned_joystick.launch.py` (joy_node + mumt_joystick + bridge together; do not also run the bridge separately or 5006 conflicts). See [README_Joystick_Manned.md](README_Joystick_Manned.md).

- `Scripts/control_sender.py`: **not a joystick reader.** A 2-socket UDP autonomous control loop — recv `0.0.0.0:5006` (state) → compute → send `127.0.0.1:5005` `{"commands":[{aircraft_name,roll,pitch,yaw,throttle}]}`. Control law: `target_pitch=5°`, `throttle=1.0 if speed<150 else 0.6`, `pitch_cmd=clamp((5-pitch)*0.1,-1,1)` then negated; roll/yaw=0. Only aircraft whose name contains "UAV", first 2 only. ROS not used.
- `Config/JSBSim/control.json`: static defaults `{throttle:0.7, aileron/elevator/rudder:0}`. Field names differ from the UDP schema (roll/pitch/yaw) → no shared message definition.
- `Config/DefaultInput.ini`: in-UE flight control is **keyboard/mouse only** (Aileron Q/E, Elevator PgUp/PgDn, Rudder A/D, Throttle W/S, shoot_bullet LMB, shoot_rocket RMB, SpawnUAV P, gear G, brake Space). **No in-UE Thrustmaster/HOTAS/joystick bindings**; the F710 joystick path (above) reaches UE over the UDP command channel instead, not via UE Enhanced Input.

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
| **BT model (friendly/enemy)** | Working py_trees BT (`py_bt_ros/scenarios/mumt`): Ground → wait-for-leader → takeoff → MaintainFormation, publishing `/aircraft/setpoint`. Friendly (UAV) only; no enemy BT yet | **Present (friendly), partial** (one side only) |
| **Action-level Autonomy (MoveTo, open source)** | No generic MoveTo/Nav2. The BT emits heading/alt/speed setpoints; UE C++ `FAircraftAutopilot` turns them into surface commands + autothrottle via PID | **Present, split** (formation in BT, low-level autopilot in UE) |
| **Friendly vs enemy split** | No team distinction in pawns/blueprints. `team` is an optional output field only; no enemy AI / enemy BT | **Mostly missing** |
| **Joystick (Thrustmaster) → `/{manned}/cmd`** | F710 gamepad → `mumt_joystick` → `/mumt/aircraft_commands` (50Hz) → bridge → 5005 → name-matched to M_F16. Reaches a global topic + UDP, not a namespaced `/{manned}/cmd` | **Present** (different topic model; not Thrustmaster) |
| **Extra Visualisation (PFD)** | `UMG_BasicPrimaryFlightDisplay` exists and works (shared widget) | **Present (partial)** |
| **ROS-Unreal single block (SUB cmd / PUB state)** | Effectively implemented via the UDP bridge (5005 cmd-in, 5006 state-out, 5010 setpoint-in). ROS is a thin global-topic relay | **Partial** (different topic model) |

**Direct answers**
- Is cmd a namespaced ROS topic? → **No** (global topics + raw UDP).
- Is there a UE→ROS `/{ns}/state` publish? → state-out path exists, but as a single global topic.
- Is the BT connected to the bridge? → **Yes** (py_bt_ros → `/aircraft/setpoint` → bridge → 5010), for the friendly UAV.
- Friendly/enemy split? → **Friendly only** (no enemy BT; team is an optional output field only).
- Joystick wired? → **Yes** (F710 → `/mumt/aircraft_commands` → 5005 → M_F16), over a global topic rather than `/{manned}/cmd`.
- Extra viz (PFD)? → works, but as a shared (not per-vehicle) widget.

---

## 4. Key observations & next steps

### Most important facts (5)
1. **The real backbone is raw UDP, not ROS** (5005 cmd-in / 5006 state-out / 5010 setpoint-in). The ROS bridge is a thin global-topic relay on top. The target's "ROS-Unreal SUB cmd/PUB state" is already satisfied at the transport level — **only the topic model differs**.
2. **Autonomy is now split across two layers** — `FAircraftAutopilot` runs the low-level cascade PID + autothrottle (heading→bank→aileron, alt→pitch→elevator, target_speed→throttle) inside UE C++, while the *formation* autonomy (Ground → wait → takeoff → MaintainFormation) runs in the py_bt_ros BT and feeds it heading/alt/speed setpoints.
3. **The BT is real now (clean milestone build)** — `py_bt_ros/scenarios/mumt` implements GatherState / WaitForLeaderTakeoff / Takeoff / MaintainFormation as py_trees nodes publishing `/aircraft/setpoint` at 10Hz. (The old `behavior_trees/mumt_autopilot.xml` in the bridge package is unrelated dead code.)
4. **Name-based addressing replaces the old broadcast** — both 5005 commands and 5010 setpoints route to the pawn whose name contains `aircraft_name`, so joystick→M_F16 and BT→F16_UAV run on the shared topics at once. Still missing: ROS namespacing, an enemy BT (team is an optional string only).
5. **Plenty of stale/hardcoded debt** — DefaultEngine's OpenWorld/AirSim game-mode misconfig, ObjectRedirector "fake" aircraft, committed Cesium token, `launch_missile` still ignored by UE, `reset` accepted over UDP but not in AircraftSetpoint.msg, control.json field-name mismatch, and the 5006 state timer oversamples (~180Hz raw, ≈60Hz distinct).

### Communication quality (measured 2026-06-28, all 127.0.0.1)

| Link | Rate | Gap | Notes |
|---|---|---|---|
| `/mumt/aircraft_commands` (joystick) | 50 Hz | ~20 ms | 0 drops |
| `/aircraft/setpoint` (BT) | 10 Hz | ~100 ms | 0 drops (= `bt_tick_rate`) |
| `/mumt/aircraft_states` | ~180–200 Hz raw, **≈60 Hz distinct** | — | each frame re-sent ~3× (timer oversampling); ~0.82 KB/msg, ~50 KB/s, 0 drops |
| Round-trip (setpoint throttle → state echo) | ~48 ms avg (20–80 ms) | — | UDP Recv-Q drains to 0, no kernel backlog |

### Prioritized roadmap
1. **(P0) Namespace the topics** — rework bridge_node to be vehicle-id aware: map UE state JSON `aircraft_name` → ns and publish per-vehicle `/{ns}/state`, subscribe per-vehicle `/{ns}/cmd` → fan-in/out over the global UDP internally. (UE side need not change yet.) This is the core of the target backbone.
2. **(P0→done, harden) BT is wired** — the py_bt_ros `scenarios/mumt` BT now drives the friendly UAV via `/aircraft/setpoint`. Remaining: move it onto `/{ns}/cmd|state` if/when topics are namespaced, and delete the stale BehaviorTree.CPP `mumt_autopilot.xml` shell in the bridge package.
3. **(P1) Clarify Action-level Autonomy** — formalise the UE setpoint as a `MoveTo(heading,alt,speed)` interface mapped 1:1 to BT leaves (the autothrottle already consumes `target_speed_mps`); consider Nav2/open-source if desired.
4. **(P1) Split friendly/enemy into two Mission Autonomy blocks** — add an enemy BT instance with different ns/params (subscribes `/{ns}/state`, publishes `/{ns}/cmd`). Promote `team` to a first-class field (currently optional).
5. **(P1→done, polish) Joystick** — F710 → `mumt_joystick` → `/mumt/aircraft_commands` → M_F16 works. Remaining: optionally publish a namespaced `/{manned}/cmd` and/or bind a HOTAS directly via UE Enhanced Input.
6. **(P2) Multi-aircraft spawn/manage + debt cleanup** — a per-vehicle spawn manager (currently single SpawnUAV on P), per-vehicle PFD instancing, fix DefaultEngine game-mode/map to RL_30/BP_JSBSimGameMode, add reset/seq to AircraftSetpoint.msg (or implement UE-side launch_missile), externalize the Cesium token, clean up the ObjectRedirector "fake" aircraft.

---

## Appendix: key file paths

- UE C++: `Source/MUMT_Sim/Private/UDPControlReceiver.cpp`(.h), `Source/MUMT_Sim/Private/BVRGymAutopilot.cpp`(.h)
- JSBSim plugin: `Plugins/JSBSimFlightDynamicsModel/Source/JSBSimFlightDynamicsModel/Private/JSBSimMovementComponent.cpp`, `Public/FDMTypes.h`
- ROS2 bridge: `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/bridge_node.py`, `.../launch/manned_joystick.launch.py`, `mumt_ros_ws/src/custom_msgs/msg/AircraftSetpoint.msg` (stale BT shell: `.../behavior_trees/mumt_autopilot.xml`)
- BT (separate repo `~/dev/py_bt_ros`): `scenarios/mumt/bt_nodes.py`, `scenarios/mumt/default_bt.xml`, `scenarios/mumt/configs/mumt.yaml`
- Input/control: `Scripts/control_sender.py`, `Config/JSBSim/control.json`, `Config/DefaultInput.ini`, `Config/DefaultEngine.ini`
- Content/Cesium: `Content/RL_30.umap`, `Content/Blueprints/BP_JSBSimGameMode.uasset`, `.../M_F16.uasset`, `.../F16_UAV.uasset`, `.../PFD/UMG_BasicPrimaryFlightDisplay.uasset`, `Content/CesiumSettings/CesiumIonServers/CesiumIonSaaS.uasset`
