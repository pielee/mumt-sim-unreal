#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; OUT="${TMPDIR:-/tmp}/verify_planner_v2_nearfield"
g++ -std=c++20 -O2 -Wall -Wextra -Wno-unused-parameter -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/DubinsPath.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/MovingSlotPredictor.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/CaptureSpeedPlanner.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/SlotLocalPathPrimitiveV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationPlannerV2.cpp" \
 "$ROOT/Tools/planner_v2/verify_planner_v2_nearfield.cpp" -o "$OUT"
"$OUT"
