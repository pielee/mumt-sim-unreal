#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/verify_formation_slot_v2"
g++ -std=c++17 -O2 -Wall -Wextra -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
  "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationSlotGeneratorV2.cpp" \
  "$ROOT/Tools/planner_v2/verify_formation_slot_v2.cpp" -o "$OUT"
"$OUT"
