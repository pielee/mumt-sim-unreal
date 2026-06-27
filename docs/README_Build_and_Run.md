# Build & Run — Getting Started

> How to build and run the full MUMT_Sim stack: the Unreal Engine 5.4 project, the ROS 2 workspace, and the
> Python control scripts. Grounded in the actual project config (`.uproject`, `*.Build.cs`, `*.Target.cs`,
> `.gitignore`, `DefaultEngine.ini`).
>
> - Date: 2026-06-23 (git `7632f63`) · Platform validated: Linux (Vulkan SM6)
> - See also: [ARCHITECTURE.md](ARCHITECTURE.md), [README_UDP_Comms.md](README_UDP_Comms.md), [README_ROS_Bridge.md](README_ROS_Bridge.md)

---

## 0. What you're running

```
┌────────────────────────┐     UDP 5005 / 5010 (cmd in)      ┌──────────────────────────┐
│  ROS 2 (bridge_node)   │ ───────────────────────────────► │  Unreal Editor (PIE)      │
│  or Scripts/*.py       │ ◄─────────────────────────────── │  RL_30 (Cesium world)     │
└────────────────────────┘        UDP 5006 (state out)       │   AUDPControlReceiver     │
                                                             │   F16_UAV / M_F16 + JSBSim│
                                                             └──────────────────────────┘
```

Three things get built/run independently:
1. **The UE project** (`MUMT_Sim.uproject`) — the simulator.
2. **The ROS 2 workspace** (`ros2/`) — optional bridge for ROS↔UDP.
3. **Python scripts** (`Scripts/control_sender.py`) — optional standalone controller (no ROS).

---

## 1. Prerequisites

| Need | Version / note |
|---|---|
| **Unreal Engine** | **5.4** (matches `EngineAssociation` in `.uproject`). Linux: the Epic Linux build or a source build. |
| **Cesium for Unreal** | **Must be installed separately** — `Plugins/CesiumForUnreal/` is **gitignored**, so a fresh clone does NOT contain it. Install the release matching UE 5.4. |
| **GeoReferencing plugin** | Epic **built-in** engine plugin — no install, just enabled (the JSBSim plugin depends on it). |
| **JSBSim plugin** | Ships in the repo (`Plugins/JSBSimFlightDynamicsModel/`, source + prebuilt `libJSBSim.a`). No separate install. |
| **C++ toolchain** | Linux: the UE cross-toolchain / clang. Windows: VS 2022 with the workloads in `.vsconfig`. |
| **ROS 2** | A recent distro (Humble/Iron/Jazzy). Needed only for the ROS bridge. `rclpy`, `std_msgs`; `py_trees` is declared but the BT isn't implemented. |
| **Python 3** | For `Scripts/control_sender.py` (stdlib only: `socket`, `json`). |
| **Cesium ion token** | One is committed inside `Content/CesiumSettings/CesiumIonServers/CesiumIonSaaS.uasset`. Replace with your own for your own tilesets (and to rotate the exposed secret). |

### What's gitignored (regenerated on build — don't expect them in a clone)
`Binaries/`, `Intermediate/`, `Saved/`, `DerivedDataCache/`, `*.sln`, **`Plugins/CesiumForUnreal/`**,
`Plugins/JSBSimFlightDynamicsModel/{Binaries,Intermediate}/`, and `ros2/*/{build,install,log}/`.

---

## 2. Build the Unreal project

### 2.1 Get the plugins in place
1. Clone the repo. Confirm `Plugins/JSBSimFlightDynamicsModel/` is present (it is, in git).
2. **Install Cesium for Unreal** into `Plugins/CesiumForUnreal/` (Epic Marketplace / Cesium release for UE 5.4),
   or install it engine-wide. Without it the project will fail to open (RL_30 uses Cesium).
3. GeoReferencing is a stock engine plugin — it's enabled automatically as a JSBSim plugin dependency.

### 2.2 Build (Linux)
The game module (`MUMT_Sim`) links: `Core, CoreUObject, Engine, InputCore, EnhancedInput,
JSBSimFlightDynamicsModel, Json, JsonUtilities, Sockets, Networking` (see `MUMT_Sim.Build.cs`). Targets are
`MUMT_SimEditor` (Editor) and `MUMT_Sim` (Game), BuildSettings V5 / include order Unreal 5.4.

```bash
# Generate project files (adjust <UE_ROOT> to your engine install)
<UE_ROOT>/Engine/Build/BatchFiles/Linux/GenerateProjectFiles.sh \
    -project="$PWD/MUMT_Sim.uproject" -game -engine

# Build the editor target (compiles MUMT_Sim + the JSBSim plugin against libJSBSim.a)
<UE_ROOT>/Engine/Build/BatchFiles/Linux/Build.sh \
    MUMT_SimEditor Linux Development -project="$PWD/MUMT_Sim.uproject"
```
Then open it:
```bash
<UE_ROOT>/Engine/Binaries/Linux/UnrealEditor "$PWD/MUMT_Sim.uproject"
```
(Or just open the `.uproject` with the Editor and accept the "compile modules" prompt.) First open will compile
shaders for the Cesium world — give it time.

> Windows: open `MUMT_Sim.uproject` (right-click → Generate VS project files), build the **MUMT_SimEditor / Development**
> config in VS 2022, then launch.

---

## 3. Level & actor setup (do this once)

Open the main map **`Content/RL_30.umap`** (the Cesium world). For the simulation + external control to work the level
must contain these actors:

| Actor | Status in RL_30 | Purpose |
|---|---|---|
| **Cesium georeference + Cesium3DTileset (World Terrain)** | present | the Earth tiles; needs a valid ion token |
| **Epic `AGeoReferencingSystem`** | **present (confirmed)** | **required by JSBSim** — converts ECEF↔UE; without it the aircraft logs an error and won't fly |
| **`AUDPControlReceiver`** | **verify / add if missing** | the UDP entry point (ports 5005/5010 in, 5006 out). If absent, drop one into the level |
| **F16_UAV / M_F16 pawn** | present (F16_UAV) | the aircraft (see [README_F16_Actor.md](README_F16_Actor.md)) |

Important config note (`DefaultEngine.ini` is stale):
- `GameDefaultMap=/Engine/Maps/Templates/OpenWorld` and `GlobalDefaultGameMode=/Script/AirSim.AirSimGameMode`
  are **wrong** (AirSim is disabled in `.uproject`). Use **World Settings → GameMode = `BP_JSBSimGameMode`** on RL_30,
  and set RL_30 as the Editor Startup / Game Default Map. (Fixing `DefaultEngine.ini` is recommended — see
  [ARCHITECTURE.md](ARCHITECTURE.md) §4 debt list.)

The `AUDPControlReceiver`'s ports/targeting are editable in its Details panel (defaults: `ListenPort=5005`,
`PythonStatePort=5006`, `SetpointListenPort=5010`, `ControlledPawnNamePatterns={F16_UAV,UAV}`).

---

## 4. Build the ROS 2 workspace (optional)

The ROS 2 side is a **separate sibling workspace** `~/dev/mumt_ros_ws/` (NOT inside the UE project), with two
packages under `src/`: `custom_msgs` and `mumt_ros_bridge`. Build **`custom_msgs` first** (it generates
`AircraftSetpoint`), then `mumt_ros_bridge`.

```bash
conda deactivate                       # ★ conda(lerobot) active breaks colcon/ros2
cd ~/dev/mumt_ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_msgs mumt_ros_bridge --symlink-install
source install/setup.bash
```
Run the bridge (the only working executable):
```bash
ros2 run mumt_ros_bridge bridge
# logs: state UDP <- 0.0.0.0:5006 | command UDP -> 127.0.0.1:5005 | setpoint UDP -> 127.0.0.1:5010
```
> ⚠️ `ros2 launch mumt_ros_bridge bt_controller.launch.py` **fails** — it references an executable `bt_controller`
> that doesn't exist (the BT controller is unimplemented). See [README_ROS_Bridge.md](README_ROS_Bridge.md).

---

## 5. Run the whole stack

**Order:** start Unreal first (it binds the receive ports), then a command source.

### Step A — start the simulator
Press **Play (PIE)** in the Editor on RL_30. The `AUDPControlReceiver` binds 5005/5010 and starts streaming state to
`127.0.0.1:5006` at 20 Hz.

### Step B — pick ONE command source
> ⚠️ Do **not** run the ROS bridge and `control_sender.py` at the same time — both bind `0.0.0.0:5006` and will fight
> over the state stream.

**Option 1 — manual flight (no external process):** possess `M_F16` and fly with the keyboard (see
`Config/DefaultInput.ini`: pitch PgUp/PgDn, roll Q/E, rudder A/D, throttle W/S, spawn UAV = P).

**Option 2 — standalone Python autopilot:**
```bash
python3 Scripts/control_sender.py
# receives state on 5006, sends {"commands":[...]} for UAV pawns to 5005
```

**Option 3 — ROS path:**
```bash
ros2 run mumt_ros_bridge bridge          # terminal 1

# low-level control:
ros2 topic pub --once /mumt/aircraft_commands std_msgs/String \
  '{data: "{\"commands\":[{\"aircraft_name\":\"F16_UAV_0\",\"roll\":0,\"pitch\":-0.1,\"yaw\":0,\"throttle\":1.0}]}"}'

# OR high-level autopilot setpoint (drives the in-engine BVRGym PID):
ros2 topic pub --once /aircraft/setpoint custom_msgs/AircraftSetpoint \
  '{heading_deg: 90.0, altitude_m: 3000.0, throttle_norm: 0.8, launch_missile: false}'

ros2 topic echo /mumt/aircraft_states     # watch state come back
```

### Step C — verify it's working
- The UAV reacts to commands / holds the setpoint heading & altitude (autopilot logs `[AP-STATE]`/`[AP-MODE]` to the UE Output Log).
- `control_sender.py` prints `received N aircraft` and `sent: {...}` once per second.
- `ros2 topic echo /mumt/aircraft_states` shows the `aircraft_state_batch` JSON.

---

## 6. Control paths cheat-sheet

| Goal | Channel | Reaches the FDM via |
|---|---|---|
| Manual keyboard flight | UE Enhanced Input | pawn BP → `JSBSim.Commands` |
| Remote low-level (roll/pitch/yaw/throttle) | UDP 5005 JSON | reflection → pawn `UDP_*` vars → BP → `JSBSim.Commands` |
| Remote high-level (heading/alt/throttle) | UDP 5010 binary | `FBVRGymAutopilot` PID → `JSBSim.Commands` directly ([README_Autopilot.md](README_Autopilot.md)) |
| State out | UDP 5006 JSON | `AUDPControlReceiver::SendStateToPython` (20 Hz) |

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Project won't open / missing Cesium classes | `Plugins/CesiumForUnreal/` not installed (it's gitignored) — install the UE 5.4 Cesium release |
| Log: *"Impossible to use a UJSBSimMovementComponent without a GeoReferencingSystem"* | No Epic `AGeoReferencingSystem` in the level — add one (RL_30 already has it; check custom maps) |
| Aircraft loads but doesn't fly / `Trim Failed` | Aircraft model has 0 gear units, or bad initial conditions — check the pawn's `AircraftModel` and the JSBSim data under the plugin's `Resources/JSBSim` |
| No response to external commands | `AUDPControlReceiver` not in the level, wrong ports, or pawn name doesn't match `ControlledPawnNamePatterns` (needs "F16_UAV"/"UAV") |
| Cesium terrain not loading | ion token invalid/expired in `CesiumIonSaaS` — set your own token |
| `ros2 launch ... bt_controller.launch.py` errors "executable not found" | Expected — `bt_controller` is unimplemented; use `ros2 run mumt_ros_bridge bridge` |
| State stream flickers / commands ignored | Both `bridge` and `control_sender.py` bound to 5006 — run only one |
| Wrong default map/game mode on launch | Stale `DefaultEngine.ini` — set RL_30 + `BP_JSBSimGameMode` in World Settings |

---

## 8. Port reference

| Port | Dir | Format | Bound by |
|---|---|---|---|
| 5005 | in → UE | JSON command | UE `ListenSocket` |
| 5010 | in → UE | binary 17B setpoint | UE `SetpointSocket` |
| 5006 | out ← UE | JSON state batch | UE sends here; ROS bridge / `control_sender.py` bind `0.0.0.0:5006` |

Full message formats: [README_UDP_Comms.md](README_UDP_Comms.md).

---

## 9. Document map

| Topic | Doc |
|---|---|
| Whole-project architecture & gap analysis | [ARCHITECTURE.md](ARCHITECTURE.md) |
| JSBSim flight model plugin | [README_JSBSim.md](README_JSBSim.md) |
| The F-16 actor (pawn) | [README_F16_Actor.md](README_F16_Actor.md) |
| UDP communication | [README_UDP_Comms.md](README_UDP_Comms.md) |
| ROS 2 bridge & package | [README_ROS_Bridge.md](README_ROS_Bridge.md) |
| BVRGym autopilot (control law) | [README_Autopilot.md](README_Autopilot.md) |
| **Build & run (this doc)** | README_Build_and_Run.md |
