#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; OUT="${TMPDIR:-/tmp}/verify_guidance_integration_v2"
g++ -std=c++20 -O2 -Wall -Wextra -Wno-unused-parameter -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControl/Px4NpfgAdapter.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControl/Px4TecsAdapter.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/MissionNavigationFrameV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/CanonicalNavigationAdapterV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationSlotGeneratorV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/DubinsPath.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/MovingSlotPredictor.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/CaptureSpeedPlanner.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationPlannerV2.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/PlannerV2Adapters.cpp" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationGuidanceCoordinatorV2.cpp" \
 "$ROOT/Tools/planner_v2/verify_guidance_integration_v2.cpp" -o "$OUT"
"$OUT"
