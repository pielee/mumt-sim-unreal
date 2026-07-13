#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; OUT="${TMPDIR:-/tmp}/verify_npfg_caller_contract_v2"
# -Wno-unused-parameter matches the committed build_verify_guidance_integration_v2.sh: the vendored
# PX4 v1.17.0 NPFG/TECS sources carry unused parameters in their upstream signatures, and the pinned
# reference is not modified to silence them.
g++ -std=c++20 -O2 -Wall -Wextra -Wno-unused-parameter -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControl/Px4NpfgAdapter.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControl/Px4TecsAdapter.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationGuidanceCoordinatorV2.cpp" \
 "$ROOT/Tools/planner_v2/verify_npfg_caller_contract_v2.cpp" -o "$OUT"
"$OUT"
