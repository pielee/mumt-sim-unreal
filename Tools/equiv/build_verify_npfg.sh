#!/usr/bin/env bash
# NPFG independent equivalence: builds THREE separate binaries (original / port /
# comparator), runs the two controllers as separate processes writing CSVs, then
# compares only the CSVs. Prints every compile command + SHA-256 for the audit.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PX4_REF="${PX4_REF:-/home/ad13/dev/reference/PX4-Autopilot-v1.17.0}"
EQ="$ROOT/Tools/equiv"
OUT="${OUTDIR:-/tmp/equiv_npfg}"
mkdir -p "$OUT"
run() { echo "+ $*"; "$@"; }

echo "############ NPFG independent equivalence ############"
echo "PX4_REF=$PX4_REF  tag=$(git -C "$PX4_REF" describe --tags 2>/dev/null)  commit=$(git -C "$PX4_REF" rev-parse HEAD 2>/dev/null)"

echo "---- [gen] shared input trace generator (data only, no control law) ----"
run g++ -std=c++17 -O2 -I"$EQ" "$EQ/gen_npfg_traces.cpp" -o "$OUT/gen_npfg"
run "$OUT/gen_npfg" "$OUT/npfg_traces.csv"

echo "---- [original] compile PX4 npfg sources ONLY (no MUMT include path) ----"
run g++ -std=c++17 -O2 -I"$EQ" -I"$ROOT/Tools/px4_shims" \
	-I"$PX4_REF/src" -I"$PX4_REF/src/lib" -I"$PX4_REF/src/lib/matrix" -I"$PX4_REF/platforms/common/include" \
	"$EQ/npfg_original_runner.cpp" \
	"$PX4_REF/src/lib/npfg/DirectionalGuidance.cpp" \
	"$PX4_REF/src/lib/npfg/AirspeedDirectionController.cpp" \
	"$PX4_REF/src/lib/npfg/CourseToAirspeedRefMapper.cpp" \
	-o "$OUT/npfg_original"

echo "---- [port] compile MUMT adapter ONLY (no PX4 include path) ----"
run g++ -std=c++17 -O2 -I"$EQ" -I"$ROOT/Source/MUMT_Sim/Public" \
	"$EQ/npfg_port_runner.cpp" \
	"$ROOT/Source/MUMT_Sim/Private/FormationControl/Px4NpfgAdapter.cpp" \
	-o "$OUT/npfg_port"

echo "---- [comparator] compile (links no controller) ----"
run g++ -std=c++17 -O2 -I"$EQ" "$EQ/compare_npfg.cpp" -o "$OUT/compare_npfg"

echo "---- binary identity (sha256) ----"
sha256sum "$OUT/npfg_original" "$OUT/npfg_port" "$OUT/compare_npfg"

echo "---- run each controller as a separate process, then compare CSVs ----"
run "$OUT/npfg_original" "$OUT/npfg_traces.csv" "$OUT/npfg_original_out.csv"
run "$OUT/npfg_port"     "$OUT/npfg_traces.csv" "$OUT/npfg_port_out.csv"
run "$OUT/compare_npfg"  "$OUT/npfg_traces.csv" "$OUT/npfg_original_out.csv" "$OUT/npfg_port_out.csv"
