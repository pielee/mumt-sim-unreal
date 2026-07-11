#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; OUT="${TMPDIR:-/tmp}/verify_planner_v2_random"
g++ -std=c++17 -O2 -Wall -Wextra -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/DubinsPath.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/MovingSlotPredictor.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/CaptureSpeedPlanner.cpp" \
 "$ROOT/Tools/planner_v2/verify_planner_v2_random.cpp" -o "$OUT"
"$OUT"
