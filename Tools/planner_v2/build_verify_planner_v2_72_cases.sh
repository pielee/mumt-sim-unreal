#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; OUT="${TMPDIR:-/tmp}/verify_planner_v2_72"
g++ -std=c++17 -O2 -Wall -Wextra -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/DubinsPath.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/MovingSlotPredictor.cpp" \
 "$ROOT/Tools/planner_v2/verify_planner_v2_72_cases.cpp" -o "$OUT"
"$OUT"
