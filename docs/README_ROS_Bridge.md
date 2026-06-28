# ROS 2 Bridge & Package (`ros2/`)

> Scope: the ROS 2 side of MUMT_Sim — what is actually implemented, what is only scaffolding, and how
> it connects to Unreal over UDP. Confirmed from source: `bridge_node.py`, `joystick_node.py`, both
> `package.xml` / `setup.py` / `CMakeLists.txt`, `manned_joystick.launch.py`, `bt_controller.yaml`,
> `bt_controller.launch.py`, `AircraftSetpoint.msg`.
>
> - Date: 2026-06-28 (mumt_ros_ws HEAD `ad75eb0`)
> - Related: [README_UDP_Comms.md](README_UDP_Comms.md) (the UDP link), [ARCHITECTURE.md](ARCHITECTURE.md)

---

## 0. TL;DR — implemented vs scaffolding

| Component | Status | Notes |
|---|---|---|
| **`bridge_node.py`** (UDP↔ROS adapter) | ✅ **Implemented & runnable** | Relays commands→UE, setpoints→UE, and state→ROS |
| **`joystick_node.py`** (`mumt_joystick`) | ✅ **Implemented** | Joy → `/mumt/aircraft_commands` (manned control — see [README_Joystick_Manned.md](README_Joystick_Manned.md)) |
| **`custom_msgs/AircraftSetpoint.msg`** | ✅ **Implemented** | Proper `ament_cmake` interface package (now carries `aircraft_name` + `target_speed_mps`) |
| **`manned_joystick.launch.py`** | ✅ **Implemented** | One-shot stack: `joy_node` + `mumt_joystick` + `bridge` |
| **BT controller node** (`mumt_bt_controller`) | ❌ **Not implemented here** | Autonomy now lives in the separate `py_bt_ros` repo; the stale `mumt_autopilot.xml` + `bt_controller.yaml` + `bt_controller.launch.py` artifacts remain but are unused |

**Bottom line:** the ROS side is a thin UDP↔ROS relay plus the manned joystick node. The BT "autopilot brain" was
moved out into the standalone `py_bt_ros` workspace, which talks to UE through these same bridge topics (it
publishes `/aircraft/setpoint` and reads `/mumt/aircraft_states`). The old in-package BT artifacts are dead.

---

## 1. Package layout

The ROS 2 side now lives in a **separate sibling colcon workspace** `~/dev/mumt_ros_ws/` (moved out of the UE
project on 2026-06-24 for clean separation). The whole system is **3 independent components**:

```
~/dev/
├── MUMT_Sim/                    ← UE5.4 project (talks UDP only)
├── mumt_ros_ws/                 ← THIS doc — colcon workspace (UDP↔ROS bridge + joystick)
│   └── src/
│       ├── custom_msgs/         (ament_cmake — ROS interfaces)
│       │   ├── msg/AircraftSetpoint.msg
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       └── mumt_ros_bridge/     (ament_python)
│           ├── mumt_ros_bridge/{bridge_node.py, joystick_node.py, __init__.py}
│           ├── behavior_trees/mumt_autopilot.xml   ← stale BT design (unused)
│           ├── config/{joystick.yaml, bt_controller.yaml}
│           ├── launch/{manned_joystick.launch.py, bt_controller.launch.py}
│           ├── setup.py    entry_points: bridge, joystick
│           └── package.xml  depends: rclpy, std_msgs, sensor_msgs, custom_msgs, joy, py_trees
└── py_bt_ros/                   ← behavior tree (Python, OWN git repo, not colcon)
                                    connects via ROS topics (publishes /aircraft/setpoint)
```

Boundaries: **UDP** (MUMT_Sim ↔ mumt_ros_bridge) and **ROS topics** (mumt_ros_bridge ↔ py_bt_ros).
Build dependency order: **`custom_msgs` first**, then `mumt_ros_bridge`.

> Cruft note: nested `mumt_ros_bridge/mumt_ros_bridge/{build,install,log}/` folders (each with `COLCON_IGNORE`)
> are accidentally-committed colcon artifacts; they are not part of the package.

---

## 2. The bridge node — `bridge_node.py` (IMPLEMENTED)

Node class `MumtBridgeNode`, node name **`mumt_bridge`**, console_script **`bridge`**. A pure UDP↔ROS adapter
(Unreal speaks only UDP — see [README_UDP_Comms.md](README_UDP_Comms.md)).

### Parameters (defaults)
| Param | Default | Meaning |
|---|---|---|
| `unreal_ip` | `127.0.0.1` | UE host |
| `control_port` | `5005` | JSON command → UE |
| `state_port` | `5006` | state ← UE (bridge binds `0.0.0.0:5006`) |
| `setpoint_port` | `5010` | JSON per-UAV setpoint → UE |

(No launch/yaml overrides these, so they always run at defaults.)

### Topics ↔ UDP mapping
| ROS topic | Type | Direction | UDP action |
|---|---|---|---|
| `/mumt/aircraft_commands` | `std_msgs/String` (JSON inside) | SUB | validate JSON → `sendto(unreal_ip, 5005)` (passthrough) |
| `/aircraft/setpoint` | `custom_msgs/AircraftSetpoint` | SUB | `json.dumps({aircraft_name, heading_deg, altitude_m, throttle_norm(clamped 0..1), target_speed_mps, launch_missile})` → `sendto(..., 5010)` |
| `/mumt/aircraft_states` | `std_msgs/String` (JSON) | PUB | 50 Hz timer (`create_timer(0.02)`) → `recvfrom(65535)` on 5006 → validate JSON → publish |

### Behavior details
- **Three UDP sockets**: `_cmd_sock` (→5005), `_sp_sock` (→5010), `_recv_sock` (bound `0.0.0.0:5006`, `SO_REUSEADDR`, non-blocking).
- **`_recv_state`** (50 Hz): drains the recv socket in a loop (`while True … except BlockingIOError: break`), so it forwards every queued datagram per tick and does not cap state throughput; validates each datagram as JSON, republishes as a `String`. Invalid JSON → warn + skip.
- **`_on_command`**: validates the incoming String is JSON, then forwards the **raw bytes** to UE (no transform — the ROS String already carries the JSON the UE side expects).
- **`_on_setpoint`**: serializes the typed message to a JSON object (`aircraft_name`, `heading_deg`, `altitude_m`, `throttle_norm` clamped to [0,1], `target_speed_mps`, `launch_missile`) and sends it to 5010. The name carries through so UE routes the setpoint to the addressed UAV; `target_speed_mps` drives the UE-side autothrottle (≤0 disables it, falling back to open-loop `throttle_norm`).

### What it does NOT do
- No namespacing of the bridge **topics** — `/aircraft/setpoint` and `/mumt/aircraft_commands` are shared global topics. Multi-UAV addressing is done **in the message** via `aircraft_name`, not via per-UAV topics. This is what lets the manned joystick (M_F16) and the BT-driven UAVs (F16_UAV) coexist on the same topics at once — UE routes each command/setpoint only to the pawn whose instance name contains the `aircraft_name`. (Each BT runs under its own `--ns` and tags its own name.)
- No QoS tuning (depth-10 default), no rate limiting on commands, no command echo/ack.
- `launch_missile` is forwarded as a JSON bool (UE reads it into `FUavSetpoint`).

---

## 3. `custom_msgs` (IMPLEMENTED)

A standard `ament_cmake` + `rosidl` interface package generating one message:

```
# AircraftSetpoint.msg
string  aircraft_name      # which UAV this setpoint addresses (token match in UE)
float32 heading_deg
float32 altitude_m
float32 throttle_norm       # open-loop throttle, used when autothrottle is off
float32 target_speed_mps    # autothrottle target; <=0 disables it
bool    launch_missile
```

`CMakeLists.txt` → `rosidl_generate_interfaces(custom_msgs "msg/AircraftSetpoint.msg")`.
**Note:** `aircraft_name` carries the addressing in-band (no per-UAV topic). `target_speed_mps` was added for the
UE-side speed-hold autothrottle. The message still has **no `reset` / `seq`** fields — to control reset from ROS,
they would need adding here.

---

## 4. Stale in-package BT artifacts (NOT USED)

Three artifacts (`behavior_trees/mumt_autopilot.xml`, `config/bt_controller.yaml`,
`launch/bt_controller.launch.py`) describe an in-package autopilot "brain", but **no node consumes them** —
`setup.py` registers only `bridge` and `joystick`, so `bt_controller` does not exist and
`ros2 launch mumt_ros_bridge bt_controller.launch.py` fails with "executable not found."

These are dead since autonomy moved to the standalone **`py_bt_ros`** repo. The XML uses BehaviorTree.CPP syntax
(custom tags with no registered impl); the YAML sketches an older command-topic controller design (P-control gains,
speed schedule, attitude recovery, formation "parallel" gains) that overlaps the live UE-side `FBVRGymAutopilot`
PID. Treat both as historical reference only.

---

## 5. The closed loop (current reality)

```
Manned (joystick):
  joy_node ─► mumt_joystick ─PUB /mumt/aircraft_commands─► bridge ─5005─► UE   (M_F16)

Autonomous (py_bt_ros behavior tree):
  UE ─5006─► bridge ─PUB /mumt/aircraft_states─► [py_bt_ros BT] ─PUB /aircraft/setpoint─► bridge ─5010─► UE
                                                   (per-UAV setpoint; UE runs the autopilot/autothrottle)   (F16_UAV)
```

The loop is now closed by the BT in `py_bt_ros`, which publishes `AircraftSetpoint` (heading/altitude/target speed)
rather than raw stick commands — UE's per-UAV autopilot turns those setpoints into control surface deflections.
Both paths share the same global bridge topics and are kept apart by `aircraft_name`.

So the bridge works in both directions, but the autonomy that was supposed to sit on top of it is absent.

---

## 6. Build & run (what works today)

```bash
conda deactivate                                  # ★ conda(lerobot) active breaks colcon/ros2
cd ~/dev/mumt_ros_ws                              # the separate ROS workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_msgs mumt_ros_bridge --symlink-install   # custom_msgs first
source install/setup.bash

# Preferred: bring up the whole manned stack (joy_node + mumt_joystick + bridge):
ros2 launch mumt_ros_bridge manned_joystick.launch.py
#   ↳ DO NOT also run the bridge separately — it would bind 0.0.0.0:5006 twice (conflict).
#   ↳ pass start_bridge:=false if the bridge is already running elsewhere.
#   bridge logs: state UDP <- 0.0.0.0:5006 | command UDP -> 127.0.0.1:5005 | setpoint UDP -> 127.0.0.1:5010

# (Bridge alone, only if not using the launch file above):
ros2 run mumt_ros_bridge bridge

# Send a command through ROS (works):
ros2 topic pub --once /mumt/aircraft_commands std_msgs/String \
  '{data: "{\"commands\":[{\"aircraft_name\":\"F16_UAV\",\"roll\":0,\"pitch\":-0.1,\"yaw\":0,\"throttle\":1.0}]}"}'

# Send a setpoint (works — note aircraft_name + target_speed_mps; custom_msgs must be sourced):
ros2 topic pub --once /aircraft/setpoint custom_msgs/AircraftSetpoint \
  '{aircraft_name: "F16_UAV", heading_deg: 90.0, altitude_m: 3000.0, throttle_norm: 0.8, target_speed_mps: 220.0, launch_missile: false}'

# Watch state coming back:
ros2 topic echo /mumt/aircraft_states

# Autonomy: the behavior tree runs from the separate py_bt_ros repo (publishes /aircraft/setpoint).
# The in-package bt_controller.launch.py is stale and not used.
```

---

## 7. Remaining gaps

1. **Clean up the dead artifacts** — the in-package `mumt_autopilot.xml` / `bt_controller.yaml` /
   `bt_controller.launch.py` no longer reflect any runnable path (autonomy is in `py_bt_ros`); remove or archive them.
2. **Namespace toward the target** — both directions still use shared global topics; addressing is in-band via
   `aircraft_name`. Moving to per-vehicle `/{ns}/...` topics is a P0 in [ARCHITECTURE.md](ARCHITECTURE.md).
3. **Add `reset` / `seq` to `AircraftSetpoint.msg`** if ROS-side reset/sequencing is ever needed (currently absent).
4. **One controller per aircraft** — UE's `FBVRGymAutopilot` is the live inner loop; the bridge must never relay
   both a BT setpoint and stick commands to the same UAV at once.

---

## 8. Relationship to the target architecture

The bridge is the central "ROS-Unreal" block's transport, working in both directions. The "Mission Autonomy"
blocks (friendly/enemy BT) in the target diagram are realized by the **`py_bt_ros`** behavior tree, which closes the
loop over these topics (`/mumt/aircraft_states` in, `/aircraft/setpoint` out). The remaining step toward the target
is namespacing the topics per vehicle; the in-package BT artifacts here are vestigial.

---

## 9. Key file paths

| Item | Path |
|---|---|
| Bridge node (implemented) | `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/bridge_node.py` |
| Joystick node (implemented) | `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/joystick_node.py` |
| Manned launch (joy + joystick + bridge) | `mumt_ros_ws/src/mumt_ros_bridge/launch/manned_joystick.launch.py` |
| Package manifest / entry point | `mumt_ros_ws/src/mumt_ros_bridge/package.xml`, `setup.py` |
| Stale BT artifacts (unused) | `behavior_trees/mumt_autopilot.xml`, `config/bt_controller.yaml`, `launch/bt_controller.launch.py` |
| Messages package | `mumt_ros_ws/src/custom_msgs/msg/AircraftSetpoint.msg`, `CMakeLists.txt`, `package.xml` |
| Unreal UDP counterpart | `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` |
