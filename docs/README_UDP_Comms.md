# UDP Communication Structure (external commands ↔ Unreal)

> Scope: the full UDP link that carries commands INTO Unreal and aircraft state OUT of Unreal —
> every socket, port, direction, rate, and message format, on all four endpoints.
> All facts confirmed from source: `UDPControlReceiver.cpp/.h`, `bridge_node.py`, `control_sender.py`,
> `AircraftSetpoint.msg`.
>
> - Date: 2026-06-23 (git `7632f63`)
> - Related: [ARCHITECTURE.md](ARCHITECTURE.md), [README_F16_Actor.md](README_F16_Actor.md)

---

## 0. TL;DR

Unreal speaks **only UDP** (no ROS). `AUDPControlReceiver` (a level Actor) opens **3 sockets**:
two for inbound commands (different abstraction levels) and one for outbound state.

| Port | Direction | Payload | UE socket | Peer that uses it |
|---|---|---|---|---|
| **5005** | external → UE | **JSON** low-level control `{roll,pitch,yaw,throttle}` | `ListenSocket` | `control_sender.py`, ROS bridge (`/mumt/aircraft_commands`) |
| **5010** | external → UE | **JSON** high-level setpoint `{aircraft_name,heading/alt/throttle}` (per-UAV) | `SetpointSocket` | ROS bridge (`/aircraft/setpoint`) |
| **5006** | UE → external | **JSON** state batch (pos/attitude/speed/weapons) | `SendSocket` | ROS bridge, `control_sender.py` |

All endpoints default to `127.0.0.1`. The two command channels exist because there are two control
abstraction levels: **raw surface commands (5005)** vs **navigation setpoint that an in-engine PID flies (5010)**.

---

## 1. Endpoint map

```
  ┌────────────────────────────┐                         ┌──────────────────────────────────────────┐
  │  control_sender.py         │   5005 JSON cmd  ─────►  │  Unreal: AUDPControlReceiver (level Actor)│
  │  (standalone autonomous)   │                         │                                            │
  │   recv 5006 ◄──────────────┼─────────  5006 JSON ───  │   SendSocket   → 127.0.0.1:5006 (20 Hz)   │
  │   send 5005                │                         │   ListenSocket ← :5005 (non-blocking)     │
  └────────────────────────────┘                         │   SetpointSocket ← :5010 (non-blocking)   │
                                                          │                                            │
  ┌────────────────────────────┐   5005 JSON cmd  ─────►  │   Tick:  drain 5005 + 5010, apply cmds     │
  │  ROS bridge_node.py        │   5010 binary sp ────►  │   Timer 60 Hz: BVRGym PID → setpoint pawn  │
  │  (mumt_bridge)             │                         │   Timer 20 Hz: build+send state → 5006     │
  │   recv 5006 ◄──────────────┼─────────  5006 JSON ───  └──────────────────────────────────────────┘
  │   /mumt/aircraft_commands  │
  │   /aircraft/setpoint       │   ROS topics on the other side (see ARCHITECTURE.md §2c)
  │   /mumt/aircraft_states    │
  └────────────────────────────┘
```

Note: **`bridge_node.py` and `control_sender.py` both bind `0.0.0.0:5006`** to receive state. Run only one at a
time — they otherwise compete for the same datagrams (the bridge sets `SO_REUSEADDR`, the sender does not).

---

## 2. The Unreal side — `AUDPControlReceiver`

A single `AActor` placed in the level. In `BeginPlay` it starts the three sockets and two timers; in `EndPlay`
it tears them down. Ports are `UPROPERTY` defaults (editable in the Details panel).

### Sockets
```cpp
ListenSocket   = FUdpSocketBuilder("UDP_Control_Receiver")
                   .AsNonBlocking().AsReusable().BoundToPort(ListenPort=5005)
                   .WithReceiveBufferSize(2 MB);
SetpointSocket = FUdpSocketBuilder("Autopilot_Setpoint_Receiver")
                   .AsNonBlocking().AsReusable().BoundToPort(SetpointListenPort=5010)
                   .WithReceiveBufferSize(64 KB);
SendSocket     = FUdpSocketBuilder("UDP_State_Sender")
                   .AsReusable().WithSendBufferSize(2 MB);   // sends to PythonIP:PythonStatePort = 127.0.0.1:5006
```

### Threading / rate model
| Driver | Rate | What it does |
|---|---|---|
| `Tick` (actor) | every frame | `ReceiveUDPData()` drains 5005; `ReceiveSetpointData()` drains 5010; then applies JSON commands to controlled pawns |
| `AutopilotTimerHandle` | **60 Hz** | `AutopilotTick()` → runs `FBVRGymAutopilot` PID → applies to the **primary** controlled pawn (only after first setpoint, or if `bUseDebugSetpoint`) |
| `StateSendTimerHandle` | **20 Hz** (`StateSendInterval=0.05`) | `SendStateToPython()` → builds + sends the state batch to 5006 |

Sockets are non-blocking and **fully drained** each Tick (`while HasPendingData`).

### Pawn targeting (by name substring)
- `ControlledPawnNamePatterns = {"F16_UAV","UAV"}`, capped at `MaxControlledUavs = 2` — who receives commands.
- `ObservedPawnNamePatterns = {"F16","UAV"}` — who is reported in the state batch (so the player `M_F16` is also reported).
- Fallback single target `TargetPawnName = "F16_UAV"`.

---

## 3. Channel 5005 — JSON low-level control (external → UE)

**Receiver:** `ReceiveUDPData` → `ParseCommand` → `ParseJsonCommand`, falling back to `ParseLegacyCsvCommand`.

### Accepted formats
1. **Top-level broadcast** (applies to all controlled pawns):
   ```json
   { "roll": 0.0, "pitch": -0.2, "yaw": 0.0, "throttle": 0.8 }
   ```
2. **Per-aircraft array** (named or indexed):
   ```json
   { "commands": [
       { "aircraft_name": "F16_UAV_0", "roll": 0.0, "pitch": -0.1, "yaw": 0.0, "throttle": 1.0 },
       { "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.6 }
   ] }
   ```
3. **Legacy CSV fallback** (if JSON parse fails): `"roll,pitch,yaw,throttle"` e.g. `"0,-0.2,0,0.8"`.

### Per-pawn resolution (in `Tick`) — name-matched only
Each controlled pawn responds **only** to a command whose `aircraft_name` is a substring of the pawn's
instance name (e.g. command `"M_F16"` → pawn `M_F16_C_0`; command `"F16_UAV_C_2"` → that exact UAV). There is
**no positional/broadcast fallback** — this lets independent senders run at the same time over the shared
topic (joystick → manned, controller → UAVs) without one vehicle's command leaking onto another. A pawn keeps
its last commanded `UDP_*` values until its next name-matched command arrives.
(Top-level broadcast `{roll,pitch,yaw,throttle}` and the legacy CSV form set the debug display fields but no
longer drive any pawn — only named array commands do.)

### How it reaches the flight model (INDIRECT)
`ApplyControlCommandToPawn` writes Blueprint variables via reflection (`SetBlueprintNumber`):
`UDP_Roll`, `UDP_Pitch`, `UDP_Yaw`, `UDP_Throttle`. The pawn's Blueprint graph then forwards those into
`JSBSimMovementComponent.Commands` / `EngineCommands`. So this channel **requires the pawn to have `UDP_*` variables
and a graph that uses them** (see [README_F16_Actor.md](README_F16_Actor.md)). `Roll/Pitch/Yaw/Throttle` are also
mirrored to read-only UPROPERTYs for HUD/debug.

---

## 4. Channel 5010 — JSON setpoint (external → UE, autopilot, **per-UAV**)

**Receiver:** `ReceiveSetpointData`. Each datagram is a UTF-8 JSON object carrying a high-level autopilot
setpoint **addressed to one aircraft by name** (so N UAVs share this single socket):

```json
{
  "aircraft_name":  "F16_UAV2",   // REQUIRED — routes to the matching pawn; empty/absent → packet ignored
  "heading_deg":    90.0,
  "altitude_m":     3000.0,
  "throttle_norm":  0.9,           // clamped to [0,1]
  "launch_missile": false,
  "reset":          false          // OPTIONAL — if true, drop this UAV's controller (fresh PID next tick)
}
```

A batch form is also accepted — `{ "setpoints": [ {…}, {…} ] }` — to drive several UAVs in one datagram.

Behavior:
- **Latest-wins per aircraft.** Setpoints are stored in `TMap<FString, FUavSetpoint> Setpoints` keyed by
  `aircraft_name`; a new packet for a name overwrites that name's slot only.
- `reset == true` → `Autopilots.Remove(name)`, so that UAV's controller is rebuilt clean on the next tick.
- The autopilot for a name stays inactive until that name's first setpoint arrives (no setpoint → no slot → not driven).

### How it reaches the flight model (DIRECT, per-UAV)
`AutopilotTick` (60 Hz) iterates `Setpoints`. For each entry it resolves the pawn by name — **exact
`GetName()` match first** (collision-free even when one name is a prefix of another, e.g. `F16_UAV` vs
`F16_UAV2`), then a **substring** fallback that is used **only when exactly one pawn contains the key**
(tolerates spawn suffixes like `"M_F16"` → `"M_F16_C_0"`; an ambiguous key is skipped + logged). It then looks up that aircraft's own
controller in `TMap<FString, FAircraftAutopilot> Autopilots` (`FindOrAdd`, so **each UAV keeps separate PID /
hysteresis state**). `ApplyAutopilotToPawn` then runs `FBVRGymAutopilot` (cascade PID: heading→bank→aileron,
altitude→pitch→elevator), grabs the `UJSBSimMovementComponent` via `FindComponentByClass`, and writes
**directly**: `JSBSim->Commands.Aileron/Elevator/Rudder` and `EngineCommands[0].Throttle`. No Blueprint variables.

> ⚠️ Per-pawn ownership still applies: if one pawn is both driven by a 5010 setpoint AND a 5005 JSON command,
> the two paths fight over its flight commands. Give each pawn a single controller (e.g. autopilot for UAVs,
> joystick/JSON for the manned M_F16).

---

## 5. Channel 5006 — JSON state batch (UE → external)

**Sender:** `SendStateToPython` (20 Hz). For every pawn matching `ObservedPawnNamePatterns`, `BuildPawnState`
emits one object; all are wrapped in a batch:

```json
{
  "message_type": "aircraft_state_batch",
  "count": 2,
  "aircraft": [
    {
      "aircraft_name": "F16_UAV_0",
      "x": 12345.6, "y": -789.0, "z": 30000.0,        // Unreal actor location, cm
      "speed_mps": 257.2,                              // AircraftState.TotalVelocityKts * 0.514444
      "pitch": 2.1, "roll": -0.4, "yaw": 178.9,        // AircraftState.LocalEulerAngles, deg
      "throttle": 0.8,                                 // EngineCommands[0].Throttle
      "team": null,                                    // optional pawn var "Team" (null if absent)
      "weapons": { "bullet_ammo": null, "rocket_ammo": null }  // optional pawn vars (null if absent)
    }
  ]
}
```

- Position is the **UE actor transform** (cm); attitude/speed come from the **JSBSim component**.
- `team` / `weapons.*` are read **optionally** from pawn variables (`Team`, `BulletAmmo`, `RocketAmmo`) — `null` when
  the pawn lacks them (e.g. `F16_UAV` has no weapons/team, so they report null; `M_F16` does).
- Variable names are configurable via `TeamVarName` / `BulletAmmoVarName` / `RocketAmmoVarName` UPROPERTYs.

---

## 6. The ROS bridge side — `bridge_node.py` (node `mumt_bridge`)

A pure UDP↔ROS adapter. ROS params (defaults): `unreal_ip=127.0.0.1`, `control_port=5005`, `state_port=5006`,
`setpoint_port=5010` (no launch/yaml overrides them).

| ROS topic | Type | UDP action |
|---|---|---|
| SUB `/mumt/aircraft_commands` | `std_msgs/String` (JSON inside) | validate JSON, then `sendto(unreal_ip, 5005)` — passthrough |
| SUB `/aircraft/setpoint` | `custom_msgs/AircraftSetpoint` | `json.dumps({aircraft_name, heading_deg, altitude_m, throttle_norm(clamped), launch_missile})` → `sendto(..., 5010)` |
| PUB `/mumt/aircraft_states` | `std_msgs/String` (JSON) | `recvfrom(65535)` on `0.0.0.0:5006` via a 50 Hz timer (`create_timer(0.02)`), validate JSON, publish |

Bridge-specific notes:
- The setpoint is forwarded as **JSON**, so the variable-length `aircraft_name` rides along and UE can route it
  to the right UAV. (Previously this was a fixed 17-byte binary packet with no name field — single-UAV only.)
- `launch_missile` is forwarded as a JSON bool (UE reads it into `FUavSetpoint`).

---

## 7. The standalone Python sender — `control_sender.py`

Not a joystick and not ROS. A self-contained autonomous controller:
- Binds `0.0.0.0:5006`, **blocking** `recvfrom(65535)` in a tight `while True` loop (no sleep; paced by the 20 Hz
  state stream from UE).
- Parses the state batch, picks aircraft whose name contains `"UAV"` (first 2), and for each computes:
  `target_pitch=5°`, `throttle = 1.0 if speed_mps<150 else 0.6`, `pitch_cmd = clamp((5-pitch)*0.1, -1, 1)`, then
  sends `pitch = -pitch_cmd` (roll/yaw = 0).
- Sends `{"commands":[ ... ]}` to `127.0.0.1:5005`.

Ports are hardcoded constants (`CONTROL_PORT=5005`, `STATE_PORT=5006`).

---

## 8. Full message-format reference

### A. 5005 inbound JSON (command)
```
{roll:float, pitch:float, yaw:float, throttle:float}                  // broadcast
{commands:[{aircraft_name?:str, roll,pitch,yaw,throttle:float}, ...]} // per aircraft
"roll,pitch,yaw,throttle"                                             // legacy CSV fallback
```
Ranges follow the pawn/FDM convention (surface cmds normalized; throttle 0..1). Field names are **roll/pitch/yaw**.

### B. 5010 inbound JSON (setpoint, per-UAV)
```
{aircraft_name:str, heading_deg:f, altitude_m:f, throttle_norm:f(0..1), launch_missile:bool, reset?:bool}
{setpoints:[{aircraft_name, heading_deg, altitude_m, throttle_norm, launch_missile}, ...]}   // batch form
```
`aircraft_name` is required and routes the setpoint to the pawn whose `GetName()` matches (exact, then substring).

### C. `custom_msgs/AircraftSetpoint.msg` (ROS side of the setpoint)
```
string  aircraft_name
float32 heading_deg
float32 altitude_m
float32 throttle_norm
bool    launch_missile
```
`aircraft_name` is the per-UAV address — each BT (one per UAV) sets it so the shared `/aircraft/setpoint` topic
can carry every UAV's commands. (`reset` is not in the ROS message; it's an optional UDP-only JSON field.)

### D. 5006 outbound JSON (state) — see §5.

### E. NOT a UDP format: `Config/JSBSim/control.json`
```json
{ "throttle": 0.7, "aileron": 0.0, "elevator": 0.0, "rudder": 0.0 }
```
This is a **JSBSim default snapshot**, not a UDP message. Note its field names (`aileron/elevator/rudder`) differ
from the UDP command schema (`roll/pitch/yaw`) — there is no single shared command schema across the project.

---

## 9. Gaps / pitfalls (honest notes)

1. **Two consumers bind 5006** — run only one of `bridge_node.py` / `control_sender.py` at a time.
2. **launch_missile ignored by UE** — present in msg + wire, never read on byte 13.
3. **No ROS-driven reset/seq** — `AircraftSetpoint.msg` lacks them; bridge forces reset=0.
4. **Field-name mismatch** — 5005 uses roll/pitch/yaw; control.json uses aileron/elevator/rudder.
5. **Two command paths can conflict** on the same pawn (5005 via `UDP_*` vs 5010 via direct autopilot).
6. **All hardcoded to 127.0.0.1** — UE ports are editable UPROPERTYs, bridge ports are ROS params (un-overridden), sender ports are constants.
7. **Not namespaced** — multi-aircraft is multiplexed inside the JSON `aircraft`/`commands` arrays by `aircraft_name`, not via per-vehicle ROS namespaces. (Target architecture wants `/{ns}/cmd`, `/{ns}/state`.)

---

## 10. Relationship to the target architecture

Today the UDP layer already provides the transport the target diagram calls "ROS-Unreal (SUB cmd / PUB state)":
5005/5010 = command-in, 5006 = state-out. What's missing is the **topic model**: the bridge relays onto single
global topics (`/mumt/aircraft_commands`, `/mumt/aircraft_states`) instead of per-vehicle `/{ns}/cmd` and
`/{ns}/state`. The cleanest next step (P0 in [ARCHITECTURE.md](ARCHITECTURE.md)) is to make `bridge_node.py`
vehicle-id aware — map `aircraft_name` ↔ namespace and fan in/out over this same UDP link — without changing the
Unreal side.

---

## 11. Key file paths

| Endpoint | Path |
|---|---|
| Unreal UDP receiver/sender + autopilot glue | `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` (+ `Public/UDPControlReceiver.h`) |
| Autopilot (PID consuming 5010 setpoints) | `Source/MUMT_Sim/Public/BVRGymAutopilot.h`, `Private/BVRGymAutopilot.cpp` |
| ROS bridge (UDP↔ROS) | `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/bridge_node.py` |
| Setpoint ROS message | `mumt_ros_ws/src/custom_msgs/msg/AircraftSetpoint.msg` |
| Standalone autonomous sender | `Scripts/control_sender.py` |
| JSBSim default snapshot (not UDP) | `Config/JSBSim/control.json` |
