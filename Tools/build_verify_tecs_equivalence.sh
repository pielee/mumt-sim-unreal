#!/usr/bin/env bash
# DEPRECATED / NON-AUTHORITATIVE (Phase 2A-R audit): builds a single combined binary (PX4 reference
# + MUMT port linked together) -> COMDAT symbol-folding risk. Authoritative independent harness:
# Tools/equiv/build_verify_tecs.sh. Kept only for historical comparison.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4_REF="${PX4_REF:-/home/ad13/dev/reference/PX4-Autopilot-v1.17.0}"
OUT="${TMPDIR:-/tmp}/verify_tecs_equivalence"
g++ -std=c++20 -O2 -I"$ROOT/Source/MUMT_Sim/Public" -c "$ROOT/Tools/verify_tecs_equivalence.cpp" -o "${OUT}.main.o"
g++ -std=c++20 -O2 -I"$ROOT/Source/MUMT_Sim/Public" -c "$ROOT/Source/MUMT_Sim/Private/FormationControl/Px4TecsAdapter.cpp" -o "${OUT}.port.o"
g++ -std=c++20 -O2 -include "$ROOT/Tools/px4_shims/host_logging.h" -I"$ROOT/Tools/px4_shims" -I"$PX4_REF/src" -I"$PX4_REF/src/lib" -I"$PX4_REF/src/lib/matrix" -I"$PX4_REF/platforms/common/include" \
 "${OUT}.main.o" "${OUT}.port.o" "$ROOT/Tools/verify_tecs_reference.cpp" "$PX4_REF/src/lib/tecs/TECS.cpp" \
 "$PX4_REF/src/lib/motion_planning/VelocitySmoothing.cpp" "$PX4_REF/src/lib/motion_planning/ManualVelocitySmoothingZ.cpp" -o "$OUT"
"$OUT"
