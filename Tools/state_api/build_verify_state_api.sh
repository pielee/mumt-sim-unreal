#!/usr/bin/env bash
# Host build + run of the read-only State API conversion unit tests.
# Compiles only the dependency-free MumtState layer (no Unreal, no JSBSim).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${OUTDIR:-/tmp/state_api}"
mkdir -p "$OUT"
run() { echo "+ $*"; "$@"; }

echo "############ State API host unit tests ############"
run g++ -std=c++17 -O2 -I"$ROOT/Source/MUMT_Sim/Public" \
	"$ROOT/Tools/state_api/verify_state_api.cpp" -o "$OUT/verify_state_api"
run "$OUT/verify_state_api"

# Optional: compile-verify the provider's JSBSim getter usage against the pinned JSBSim
# headers (no Unreal). Runs only where the plugin's JSBSim Include path is present.
JSB_INC="$ROOT/Plugins/JSBSimFlightDynamicsModel/Source/ThirdParty/JSBSim/Include"
if [ -d "$JSB_INC" ]; then
	echo "---- JSBSim getter + adapter chain probe (-fsyntax-only) ----"
	run g++ -std=c++17 -fsyntax-only -I"$JSB_INC" -I"$ROOT/Source/MUMT_Sim/Public" \
		"$ROOT/Tools/state_api/probe_jsbsim_getters.cpp"
	echo "JSBSim getter + adapter probe: OK"
else
	echo "JSBSim Include path not found, skipping getter probe"
fi
