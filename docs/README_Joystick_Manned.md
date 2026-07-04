# Manned Aircraft Gamepad/Joystick Control (Gamepad → ROS → UDP → Unreal)

> Goal: fly the manned aircraft (`M_F16`) with a gamepad/joystick instead of the keyboard, routing the
> stick through ROS → the existing bridge → UDP into Unreal.
> Design choices (confirmed with the user): reuse the existing global `/mumt/aircraft_commands` topic;
> axis mapping fully parameterized; the keyboard is fully replaced.
>
> Hardware: a **Logitech F710** gamepad (the same controller used for the F1TENTH car) — runs in its
> **Xbox-compatible** (XInput) mode. This runs on a
> normal **desktop PC** (not a Jetson), so the manual xpad install from
> https://github.com/woawo1213/jetpack6-joy is **not needed** — that was only for the Jetson AGX Orin / Jetpack 6
> F1TENTH box. On a desktop the pad is recognized by the stock Linux gamepad driver out of the box; you only
> need the ROS `joy` package for `joy_node`.
>
> - Date: 2026-06-28
> - Related: [README_UDP_Comms.md](README_UDP_Comms.md), [README_ROS_Bridge.md](README_ROS_Bridge.md), [README_F16_Actor.md](README_F16_Actor.md)

---

## 1. End-to-end chain

```
Logitech F710 (Xbox mode)       /dev/input/js0   (stock desktop Linux driver — plug & play)
  → joy_node (pkg: joy)           sensor_msgs/Joy on /joy
  → mumt_joystick (this pkg)      maps axes → {roll,pitch,yaw,throttle}, publishes JSON @ 50 Hz
  → /mumt/aircraft_commands       std_msgs/String  {"commands":[{"aircraft_name":"M_F16",...}]}
  → bridge (this pkg)             forwards JSON → UDP 5005   (unchanged)
  → AUDPControlReceiver           sets UDP_Roll/Pitch/Yaw/Throttle on M_F16 (reflection)
  → M_F16 Blueprint               UDP_* → JSBSimMovementComponent.Commands / EngineCommands
  → JSBSim FDM                    flies
```

This is a **raw / low-level** control path (direct stick → control surfaces) — the correct level for manual
piloting. It deliberately does **not** go through the BVRGym autopilot (that is for high-level
`{heading, alt, throttle}` UAV control; see [README_Autopilot.md](README_Autopilot.md)).

---

## 2. Hardware / driver setup (desktop PC)

The gamepad, ROS, and Unreal all run on the **same desktop computer**, so commands stay on `127.0.0.1`
(localhost) and no Jetson/xpad setup is required.

1. Plug in the gamepad. Desktop Linux recognizes Xbox-compatible pads via the stock driver.
2. Install the ROS `joy` package if you don't have it (`ros-<distro>-joy`). Optionally `joystick`/`jstest` for
   verification.
3. Verify the OS sees it:
   ```bash
   ls /dev/input/js0
   jstest /dev/input/js0       # move sticks/triggers, watch the axis values (optional)
   # or, once joy_node runs:
   ros2 topic echo /joy
   ```
   Note which axis index moves for each control, and the **resting vs pressed** value of the triggers
   (commonly +1 released → −1 fully pressed).

> Everything on one PC → `unreal_ip` stays `127.0.0.1` (the default). The `unreal_ip` launch arg in §4 is only
> needed if you later split ROS and Unreal across machines.

---

## 3. What changed in this repo (already done)

| File | Change |
|---|---|
| `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/joystick_node.py` | **new** — `mumt_joystick`: `sensor_msgs/Joy` → JSON command; gamepad axis mapping + **incremental throttle** |
| `mumt_ros_ws/src/mumt_ros_bridge/config/joystick.yaml` | **new** — Xbox gamepad axis/sign/deadzone defaults, throttle mode + trigger ranges |
| `mumt_ros_ws/src/mumt_ros_bridge/launch/manned_joystick.launch.py` | **new** — starts `joy_node` + `mumt_joystick` (+ `bridge`), with `unreal_ip` arg |
| `mumt_ros_ws/src/mumt_ros_bridge/setup.py` | console_script `joystick = mumt_ros_bridge.joystick_node:main` |
| `mumt_ros_ws/src/mumt_ros_bridge/package.xml` | deps `sensor_msgs`, `joy` |
| `Source/MUMT_Sim/Public/UDPControlReceiver.h` | `ControlledPawnNamePatterns` += `"M_F16"`; `MaxControlledUavs` 2 → 4 |

> Why the C++ change: the 5005 JSON path only drove `F16_UAV`/`UAV` by default — `M_F16` was *observed* but
> not *controlled*. Adding `"M_F16"` puts the manned pawn in the controlled set; the cap bump prevents it
> being truncated when UAVs are also present. (Both are `EditAnywhere` — can also be set on the
> `AUDPControlReceiver` actor in-level without recompiling.)

---

## 4. Default gamepad mapping & build/run

Standard Linux `joy` Xbox layout (defaults in `joystick.yaml`):

| Control | Joy axis | Aircraft |
|---|---|---|
| Right stick X | `axes[3]` | **roll** |
| Right stick Y | `axes[4]` | **pitch** |
| Left stick X | `axes[0]` | **yaw** (rudder) |
| RT (right trigger) | `axes[5]` | **throttle up** (incremental) |
| LT (left trigger) | `axes[2]` | **throttle down** (incremental) |

Throttle is **incremental** by default (a gamepad has no detented throttle): hold **RT** to spool up, **LT**
to spool down, and the value latches. Switch to `throttle_mode: "axis"` in the yaml if you use a real
throttle quadrant/slider.

```bash
# 1) Build (sensor_msgs + joy must be present in your ROS distro)
conda deactivate                      # ★ conda active breaks colcon/ros2
cd ~/dev/mumt_ros_ws                  # separate ROS workspace (sibling of MUMT_Sim)
source /opt/ros/humble/setup.bash
colcon build --packages-select mumt_ros_bridge --symlink-install
source install/setup.bash

# 2) Start Unreal (PIE on RL_30) so UDP 5005/5006 are open.

# 3) Launch the manned-control stack (joy_node + mumt_joystick + bridge):
ros2 launch mumt_ros_bridge manned_joystick.launch.py
#   (all on one PC → default unreal_ip=127.0.0.1; nothing extra needed)
#   only if ROS and Unreal are on separate machines:
ros2 launch mumt_ros_bridge manned_joystick.launch.py unreal_ip:=192.168.x.y
#   already running the bridge separately?  start_bridge:=false
```

Verify:
```bash
ros2 topic echo /joy                      # OS sees the pad
ros2 topic echo /mumt/aircraft_commands   # M_F16 {roll,pitch,yaw,throttle} updating
```

---

## 5. Tuning the mapping (per pad)

1. `ros2 topic echo /joy`, move each control, note `axes[i]` indices and trigger rest/press values.
2. Edit `config/joystick.yaml`:
   - `roll_axis / pitch_axis / yaw_axis` — indices; flip `*_sign` if a control is reversed
   - `throttle_up_axis / throttle_down_axis` — your RT/LT indices
   - `trigger_rest / trigger_full` — **set to the raw values you saw** (e.g. some pads rest at 0, not +1)
   - `throttle_rate` — how fast throttle spools; `deadzone`, `*_scale` — feel
3. Relaunch (with `--symlink-install`, yaml edits need no rebuild).

> Signs are tuned **here**, not in the Blueprint — if pitch/roll is inverted in-game, just flip `pitch_sign`/`roll_sign`.

---

## 6. What you must still do in the Unreal Editor (Blueprint — can't be scripted here)

`M_F16` already has `UDP_Roll/Pitch/Yaw/Throttle` variables. Two edits remain:

### 6.1 Wire UDP_* → flight commands
Open **`Content/Blueprints/M_F16`** → Event Graph (on Event Tick), set:
- `JSBSimMovementComponent → Commands.Aileron`  = `UDP_Roll`
- `JSBSimMovementComponent → Commands.Elevator` = `UDP_Pitch`
- `JSBSimMovementComponent → Commands.Rudder`   = `UDP_Yaw`
- `JSBSimMovementComponent → EngineCommands[0].Throttle` = `UDP_Throttle`

> Tip: **`Content/Blueprints/F16_UAV`** already does exactly this — copy that graph fragment into `M_F16`.

### 6.2 Remove the keyboard flight inputs (full replace)
Delete/disconnect the keyboard **InputAxis/InputAction** nodes in `M_F16` that write into `Commands`
(pitch/roll/rudder/throttle). After this the aircraft is driven only by the gamepad via `UDP_*`.

---

## 7. Notes / limits

- **Manned + UAV run together cleanly.** The UE side now applies commands by **name match only** (the pawn's
  name must contain the command's `aircraft_name`). The gamepad sends `M_F16` → only the manned pawn; UAV
  controllers send the UAVs' exact names → only those. No cross-apply. (Requires the C++ rebuild that added
  name-only matching; top-level/unnamed commands no longer drive any pawn.)
- **Incremental throttle latches** — it stays where you left it; it does **not** spring to idle. Pull LT to
  reduce. Set `throttle_init` if you want a non-zero start.
- **Single desktop PC** is the assumed setup → `unreal_ip = 127.0.0.1` (default), no firewall/network config.
  Only if you later split ROS and Unreal across machines: set `unreal_ip` (cmd → UE) and open UDP 5005 on the
  UE machine's firewall.
- **Toward the target architecture:** later, switch from the global topic to a namespaced
  `/{manned_vehicle}/cmd` and make `bridge_node` vehicle-id aware (P0 in [ARCHITECTURE.md](ARCHITECTURE.md)).

---

## 8. Key files

| Item | Path |
|---|---|
| Joystick mapping node | `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/joystick_node.py` |
| Axis/param config | `mumt_ros_ws/src/mumt_ros_bridge/config/joystick.yaml` |
| Launch (joy + map + bridge) | `mumt_ros_ws/src/mumt_ros_bridge/launch/manned_joystick.launch.py` |
| UE command receiver | `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` (+ `Public/UDPControlReceiver.h`) |
| Manned pawn (edit in editor) | `Content/Blueprints/M_F16.uasset` |
| UAV pawn (reference graph) | `Content/Blueprints/F16_UAV.uasset` |
| Gamepad driver (Jetson) | https://github.com/woawo1213/jetpack6-joy (xpad install) |
