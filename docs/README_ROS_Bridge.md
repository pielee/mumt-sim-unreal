# ROS 2 Bridge & Package (`ros2/`)

> Scope: the ROS 2 side of MUMT_Sim — what is actually implemented, what is only scaffolding, and how
> it connects to Unreal over UDP. Confirmed from source: `bridge_node.py`, both `package.xml` /
> `setup.py` / `CMakeLists.txt`, `mumt_autopilot.xml`, `bt_controller.yaml`, `bt_controller.launch.py`,
> `AircraftSetpoint.msg`.
>
> - Date: 2026-06-23 (git `7632f63`)
> - Related: [README_UDP_Comms.md](README_UDP_Comms.md) (the UDP link), [ARCHITECTURE.md](ARCHITECTURE.md)

---

## 0. TL;DR — implemented vs scaffolding

| Component | Status | Notes |
|---|---|---|
| **`bridge_node.py`** (UDP↔ROS adapter) | ✅ **Implemented & runnable** | The only working executable. Relays commands→UE and state→ROS |
| **`custom_msgs/AircraftSetpoint.msg`** | ✅ **Implemented** | Proper `ament_cmake` interface package |
| **BT controller node** (`mumt_bt_controller`) | ❌ **Not implemented** | Only `mumt_autopilot.xml` + `bt_controller.yaml` + `bt_controller.launch.py` exist — **no node code** |
| **`bt_controller.launch.py`** | ⚠️ **Broken** | Launches `executable="bt_controller"`, but `setup.py` registers only `bridge` → "executable not found" |
| **Framework choice** | ⚠️ **Contradictory** | `package.xml` depends on `py_trees`, but the XML is **BehaviorTree.CPP** syntax (incompatible) |

**Bottom line:** today the ROS side is a thin UDP↔ROS relay. The "autopilot brain" (the behavior tree) is fully
designed on paper (a detailed XML tree + a rich parameter file) but has **zero executable code**, so it cannot run.

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
│           ├── behavior_trees/mumt_autopilot.xml   ← BT design (no impl)
│           ├── config/{joystick.yaml, bt_controller.yaml}
│           ├── launch/{manned_joystick.launch.py, bt_controller.launch.py}
│           ├── setup.py    entry_points: bridge, joystick
│           └── package.xml  depends: rclpy, std_msgs, sensor_msgs, custom_msgs, joy, py_trees
└── py_bt_ros/                   ← behavior tree (Python/py_trees, OWN git repo, not colcon)
                                    connects via ROS topics (modules/ros_bridge.py)
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
| `/aircraft/setpoint` | `custom_msgs/AircraftSetpoint` | SUB | `json.dumps({aircraft_name, heading_deg, altitude_m, throttle_norm(clamped 0..1), launch_missile})` → `sendto(..., 5010)` |
| `/mumt/aircraft_states` | `std_msgs/String` (JSON) | PUB | 50 Hz timer (`create_timer(0.02)`) → `recvfrom(65535)` on 5006 → validate JSON → publish |

### Behavior details
- **Three UDP sockets**: `_cmd_sock` (→5005), `_sp_sock` (→5010), `_recv_sock` (bound `0.0.0.0:5006`, `SO_REUSEADDR`, non-blocking).
- **`_recv_state`** (50 Hz): drains the recv socket in a loop (`while True … except BlockingIOError: break`), validates each datagram as JSON, republishes as a `String`. Invalid JSON → warn + skip.
- **`_on_command`**: validates the incoming String is JSON, then forwards the **raw bytes** to UE (no transform — the ROS String already carries the JSON the UE side expects).
- **`_on_setpoint`**: serializes the typed message to a JSON object (`aircraft_name`, `heading_deg`, `altitude_m`, `throttle_norm` clamped to [0,1], `launch_missile`) and sends it to 5010. The name carries through so UE routes the setpoint to the addressed UAV.

### What it does NOT do
- No namespacing of the bridge **topics** — `/aircraft/setpoint` is one shared global topic. Multi-UAV addressing is done **in the message** via `aircraft_name`, not via per-UAV topics. (Each BT runs under its own `--ns` and tags its own name.)
- No QoS tuning (depth-10 default), no rate limiting on commands, no command echo/ack.
- `launch_missile` is forwarded as a JSON bool (UE reads it into `FUavSetpoint`).

---

## 3. `custom_msgs` (IMPLEMENTED)

A standard `ament_cmake` + `rosidl` interface package generating one message:

```
# AircraftSetpoint.msg
float32 heading_deg
float32 altitude_m
float32 throttle_norm
bool    launch_missile
```

`CMakeLists.txt` → `rosidl_generate_interfaces(custom_msgs "msg/AircraftSetpoint.msg")`.
**Note:** the message has **no `reset` / `seq`** fields even though the UDP wire format carries them — the bridge
fills those in itself. To control reset from ROS, these fields would need adding here.

---

## 4. The BT controller (DESIGNED, NOT IMPLEMENTED)

Three artifacts describe an autopilot "brain" that is meant to read state, decide commands, and publish them — but
**the node that would consume them does not exist**.

### 4.1 `behavior_trees/mumt_autopilot.xml` — the tree
**BehaviorTree.CPP** syntax (`<root main_tree_to_execute>`, `<BehaviorTree ID>`):
```
MUMT_Autopilot (Sequence)
├── HasAircraftState          guard: has state been received?
├── SelectControlledAircraft  filter aircraft by name pattern ("UAV"), up to max
├── Selector (FlightModeSelector)
│   ├── RecoverIfBadAttitude  if roll/pitch out of bounds → recovery; SUCCESS short-circuits
│   └── ParallelFlightControl normal / formation flight controller
└── PublishCommands           serialize commands → command_topic
```
None of these custom node tags (`HasAircraftState`, `SelectControlledAircraft`, `RecoverIfBadAttitude`,
`ParallelFlightControl`, `PublishCommands`) have a registered implementation in C++ or Python anywhere in the repo.
There are **no blackboard ports** declared, and no generic `MoveTo`-style navigation nodes.

### 4.2 `config/bt_controller.yaml` — parameters for node `mumt_bt_controller`
A rich, fully-specified controller config (read by nothing today). It defines the intended closed loop and gains:
- **I/O**: `state_topic=/mumt/aircraft_states`, `command_topic=/mumt/aircraft_commands`, `controlled_name_pattern=UAV`, `max_controlled_aircraft=3`, `tick_rate_hz=20`, `invert_pitch_cmd=true`.
- **Speed-scheduled throttle/pitch**: `low_speed_mps=140`, `high_speed_mps=185`; accel/cruise/slow throttle `1.0/0.75/0.0`; accel/cruise/slow target pitch `12/2/10°`.
- **Attitude P-control**: `pitch_gain=0.10`, `roll_gain=0.03`, `pitch_limit=0.8`, `roll_limit=0.5`, `pitch_deadband_deg=1.5`.
- **Recovery**: trigger at `bad_roll_deg=70`, `bad_pitch_down_deg=-20`; `recovery_target_pitch_deg=8`, `recovery_throttle=0.8`.
- **Formation ("parallel") flight**: leader name, reference speed 165 m/s, speed/heading/pitch gains + damping, per-tick slew limits, deadbands.
- **Timing hysteresis**: `pitch_hold_ticks=6`, `pitch_cooldown_ticks=8`.

> This YAML is the most concrete spec of the intended autonomy. Note it overlaps in purpose with the UE-side
> `FBVRGymAutopilot` PID — there are effectively two autopilot designs (one live in UE C++, one paper-only in ROS).

### 4.3 `launch/bt_controller.launch.py` — broken launch
Declares `params_file` (→ `bt_controller.yaml`) and `xml_file` (→ `mumt_autopilot.xml`) launch args and starts:
```python
Node(package="mumt_ros_bridge", executable="bt_controller", name="mumt_bt_controller", ...)
```
But `setup.py` only registers `bridge` as a console_script. **`bt_controller` does not exist**, so
`ros2 launch mumt_ros_bridge bt_controller.launch.py` fails with "executable not found." The bridge itself is in no
launch file.

---

## 5. Intended closed loop vs current reality

```
INTENDED (when the BT node exists):
  UE ──5006──► bridge ──PUB /mumt/aircraft_states ──► [BT controller] ──PUB /mumt/aircraft_commands ──► bridge ──5005──► UE
                                                         (reads yaml gains, runs the XML tree)

TODAY (BT node missing):
  UE ──5006──► bridge ──PUB /mumt/aircraft_states ──► (no subscriber that closes the loop)
  commands must come from elsewhere:
     • control_sender.py  (direct UDP 5005, bypasses ROS entirely), or
     • a manual `ros2 topic pub /mumt/aircraft_commands std_msgs/String '{...json...}'`
        ──► bridge ──5005──► UE
```

So the bridge works in both directions, but the autonomy that was supposed to sit on top of it is absent.

---

## 6. Build & run (what works today)

```bash
conda deactivate                                  # ★ conda(lerobot) active breaks colcon/ros2
cd ~/dev/mumt_ros_ws                              # the separate ROS workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_msgs mumt_ros_bridge --symlink-install
source install/setup.bash

# Run the bridge (works):
ros2 run mumt_ros_bridge bridge
#   bridge logs: state UDP <- 0.0.0.0:5006 | command UDP -> 127.0.0.1:5005 | setpoint UDP -> 127.0.0.1:5010

# Send a command through ROS (works):
ros2 topic pub --once /mumt/aircraft_commands std_msgs/String \
  '{data: "{\"commands\":[{\"aircraft_name\":\"F16_UAV_0\",\"roll\":0,\"pitch\":-0.1,\"yaw\":0,\"throttle\":1.0}]}"}'

# Send a setpoint (works):
ros2 topic pub --once /aircraft/setpoint custom_msgs/AircraftSetpoint \
  '{heading_deg: 90.0, altitude_m: 3000.0, throttle_norm: 0.8, launch_missile: false}'

# Watch state coming back:
ros2 topic echo /mumt/aircraft_states

# Launch the BT controller (FAILS — executable 'bt_controller' not registered):
# ros2 launch mumt_ros_bridge bt_controller.launch.py
```

---

## 7. Gaps & what it takes to make the BT real

1. **Pick one framework.**
   - (a) Keep **BehaviorTree.CPP**: write a C++ ament package, register node classes (`HasAircraftState`,
     `SelectControlledAircraft`, `RecoverIfBadAttitude`, `ParallelFlightControl`, `PublishCommands`), and add a
     `bt_controller` executable. OR
   - (b) Rewrite the tree in **py_trees** (which `package.xml` already depends on) and load it from Python.
   - Today the two are mixed (py_trees dep + BehaviorTree.CPP XML) and neither is wired.
2. **Register the executable** — add `bt_controller = mumt_ros_bridge.<module>:main` to `setup.py` `console_scripts`
   so the launch file resolves.
3. **Implement the loop**: subscribe `state_topic`, run the tree at `tick_rate_hz`, publish a JSON command batch on
   `command_topic` matching the UDP schema (`{commands:[{aircraft_name,roll,pitch,yaw,throttle}]}`).
4. **Decide where autonomy lives** — reconcile the ROS BT design with the existing UE `FBVRGymAutopilot` PID
   (don't run two fighting controllers on one aircraft).
5. **Namespace toward the target** — move from global topics to per-vehicle `/{ns}/cmd` / `/{ns}/state`
   (P0 in [ARCHITECTURE.md](ARCHITECTURE.md)); add `reset`/`seq` to `AircraftSetpoint.msg` if ROS-side reset is needed.

---

## 8. Relationship to the target architecture

The bridge is the central "ROS-Unreal" block's transport, already working for both directions. The two "Mission
Autonomy" blocks (friendly/enemy BT) in the target diagram correspond to the **unimplemented BT controller** here —
the `bt_controller.yaml` even sketches a friendly-style formation follower (`parallel_*`) and an attitude-recovery
safety layer. Making the BT runnable and namespacing the topics are the two steps that turn this scaffolding into
the target's Mission-Autonomy ↔ ROS-Unreal loop.

---

## 9. Key file paths

| Item | Path |
|---|---|
| Bridge node (implemented) | `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/bridge_node.py` |
| Package manifest / entry point | `mumt_ros_ws/src/mumt_ros_bridge/package.xml`, `setup.py` |
| Behavior tree (design only) | `mumt_ros_ws/src/mumt_ros_bridge/behavior_trees/mumt_autopilot.xml` |
| BT params (unused) | `mumt_ros_ws/src/mumt_ros_bridge/config/bt_controller.yaml` |
| Launch (broken) | `mumt_ros_ws/src/mumt_ros_bridge/launch/bt_controller.launch.py` |
| Messages package | `mumt_ros_ws/src/custom_msgs/msg/AircraftSetpoint.msg`, `CMakeLists.txt`, `package.xml` |
| Unreal UDP counterpart | `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` |
