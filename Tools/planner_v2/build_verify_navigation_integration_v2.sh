#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; OUT="${TMPDIR:-/tmp}/verify_navigation_integration_v2"
g++ -std=c++17 -O2 -Wall -Wextra -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/MissionNavigationFrameV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/CanonicalNavigationAdapterV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationSlotGeneratorV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/DubinsPath.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/MovingSlotPredictor.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/CaptureSpeedPlanner.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/SlotLocalPathPrimitiveV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationPlannerV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/PlannerV2Adapters.cpp" \
 "$ROOT/Tools/planner_v2/verify_navigation_integration_v2.cpp" -o "$OUT"
"$OUT"
