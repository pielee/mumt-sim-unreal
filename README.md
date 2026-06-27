# MUMT_Sim — Manned–Unmanned Teaming BVR Simulator (Unreal Engine 5.4)

UE5.4 + JSBSim + Cesium based Beyond-Visual-Range air-combat simulator.
Talks to the ROS 2 side over UDP (ports 5005 / 5006 / 5010).

## ⚠️ Before you run — install Cesium & set YOUR OWN ion token
This repo **does not ship the Cesium for Unreal plugin or any Cesium ion token**
(excluded on purpose). To run:

1. Install **Unreal Engine 5.4**.
2. Install the **Cesium for Unreal** plugin (UE 5.4 release) into `Plugins/CesiumForUnreal/`.
3. Open `MUMT_Sim.uproject` (compiles the C++ modules + bundled JSBSim plugin).
4. **Set your own Cesium ion token**: Editor → **Cesium** panel → **Token** → sign in / paste
   your free token. This recreates the ion-server config that was removed from this repo.
5. Open the **RL_30** map → **Play**.

Full build/run guide: [`docs/README_Build_and_Run.md`](docs/README_Build_and_Run.md).

## 3-part system (companion repos)
- **this repo** — Unreal simulator
- **mumt-ros-bridge** — ROS 2 ↔ UDP bridge + joystick teleop
- **mumt-bt** — behavior-tree autonomy (py_trees)

Architecture & internals: see [`docs/`](docs/).
