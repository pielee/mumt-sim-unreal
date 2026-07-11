#!/usr/bin/env bash
# TECS independent equivalence: builds THREE separate binaries (original / port /
# comparator), runs the two controllers as separate processes writing CSVs, then
# compares only the CSVs. Prints every compile command + SHA-256 for the audit.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PX4_REF="${PX4_REF:-/home/ad13/dev/reference/PX4-Autopilot-v1.17.0}"
EQ="$ROOT/Tools/equiv"
OUT="${OUTDIR:-/tmp/equiv_tecs}"
mkdir -p "$OUT"
run() { echo "+ $*"; "$@"; }

echo "############ TECS independent equivalence ############"
echo "PX4_REF=$PX4_REF  tag=$(git -C "$PX4_REF" describe --tags 2>/dev/null)  commit=$(git -C "$PX4_REF" rev-parse HEAD 2>/dev/null)"

echo "---- [gen] shared input trace generator (data only, no control law) ----"
run g++ -std=c++17 -O2 -I"$EQ" "$EQ/gen_tecs_traces.cpp" -o "$OUT/gen_tecs"
run "$OUT/gen_tecs" "$OUT/tecs_traces.csv"

echo "---- [original] compile PX4 TECS + motion_planning ONLY (no MUMT include path) ----"
run g++ -std=c++20 -O2 -include "$ROOT/Tools/px4_shims/host_logging.h" -I"$EQ" -I"$ROOT/Tools/px4_shims" \
	-I"$PX4_REF/src" -I"$PX4_REF/src/lib" -I"$PX4_REF/src/lib/matrix" -I"$PX4_REF/platforms/common/include" \
	"$EQ/tecs_original_runner.cpp" \
	"$PX4_REF/src/lib/tecs/TECS.cpp" \
	"$PX4_REF/src/lib/motion_planning/VelocitySmoothing.cpp" \
	"$PX4_REF/src/lib/motion_planning/ManualVelocitySmoothingZ.cpp" \
	-o "$OUT/tecs_original"

echo "---- [port] compile MUMT adapter ONLY (no PX4 include path) ----"
run g++ -std=c++20 -O2 -I"$EQ" -I"$ROOT/Source/MUMT_Sim/Public" \
	"$EQ/tecs_port_runner.cpp" \
	"$ROOT/Source/MUMT_Sim/Private/FormationControl/Px4TecsAdapter.cpp" \
	-o "$OUT/tecs_port"

echo "---- [comparator] compile (links no controller) ----"
run g++ -std=c++17 -O2 -I"$EQ" "$EQ/compare_tecs.cpp" -o "$OUT/compare_tecs"

echo "---- binary identity (sha256) ----"
sha256sum "$OUT/tecs_original" "$OUT/tecs_port" "$OUT/compare_tecs"

echo "---- run each controller as a separate process, then compare CSVs ----"
run "$OUT/tecs_original" "$OUT/tecs_traces.csv" "$OUT/tecs_original_out.csv"
run "$OUT/tecs_port"     "$OUT/tecs_traces.csv" "$OUT/tecs_port_out.csv"
run "$OUT/compare_tecs"  "$OUT/tecs_traces.csv" "$OUT/tecs_original_out.csv" "$OUT/tecs_port_out.csv"
