#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)";OUT="${TMPDIR:-/tmp}/verify_slot_local_path_v2"
g++ -std=c++17 -O2 -Wall -Wextra -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/SlotLocalPathPrimitiveV2.cpp" \
 "$ROOT/Tools/planner_v2/verify_slot_local_path_v2.cpp" -o "$OUT"
"$OUT"
