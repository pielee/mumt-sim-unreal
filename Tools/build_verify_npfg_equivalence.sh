#!/usr/bin/env bash
# DEPRECATED / NON-AUTHORITATIVE (Phase 2A-R audit): builds a single combined binary (PX4 reference
# + MUMT port linked together) -> COMDAT symbol-folding risk. Authoritative independent harness:
# Tools/equiv/build_verify_npfg.sh. Kept only for historical comparison.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4_REF="${PX4_REF:-/home/ad13/dev/reference/PX4-Autopilot-v1.17.0}"
OUT="${TMPDIR:-/tmp}/verify_npfg_equivalence"
g++ -std=c++17 -O2 -I"$ROOT/Source/MUMT_Sim/Public" -c "$ROOT/Tools/verify_npfg_equivalence.cpp" -o "${OUT}.main.o"
g++ -std=c++17 -O2 -I"$ROOT/Source/MUMT_Sim/Public" -c "$ROOT/Source/MUMT_Sim/Private/FormationControl/Px4NpfgAdapter.cpp" -o "${OUT}.port.o"
g++ -std=c++17 -O2 -I"$ROOT/Tools/px4_shims" -I"$PX4_REF/src" -I"$PX4_REF/src/lib" -I"$PX4_REF/src/lib/matrix" -I"$PX4_REF/platforms/common/include" \
 "${OUT}.main.o" "${OUT}.port.o" "$ROOT/Tools/verify_npfg_reference.cpp" \
 "$PX4_REF/src/lib/npfg/DirectionalGuidance.cpp" \
 "$PX4_REF/src/lib/npfg/AirspeedDirectionController.cpp" \
 "$PX4_REF/src/lib/npfg/CourseToAirspeedRefMapper.cpp" -o "$OUT"
"$OUT"
