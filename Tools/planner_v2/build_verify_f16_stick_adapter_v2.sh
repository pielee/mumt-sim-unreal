#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; OUT="${TMPDIR:-/tmp}/verify_f16_stick_adapter_v2"
g++ -std=c++20 -O2 -Wall -Wextra -Wno-unused-parameter -pedantic -I"$ROOT/Source/MUMT_Sim/Public" \
 "$ROOT/Source/MUMT_Sim/Private/FormationControlV2/F16StickAdapterV2.cpp" \
 "$ROOT/Tools/planner_v2/verify_f16_stick_adapter_v2.cpp" -o "$OUT"
"$OUT"
