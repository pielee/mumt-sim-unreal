#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
TC="$HOME/unreal/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v22_clang-16.0.6-centos7/x86_64-unknown-linux-gnu"
LC="$HOME/unreal/Engine/Source/ThirdParty/Unix/LibCxx"
PLUGIN="$REPO_ROOT/Plugins/JSBSimFlightDynamicsModel"
JSBSIM_ROOT="$PLUGIN/Resources/JSBSim"
OUT=/tmp/mumt_f16_lateral_dynamics_isolation_v2
BINARY="$OUT/verify_f16_lateral_dynamics_isolation_v2"

mkdir -p "$OUT"
rm -f "$OUT"/run_*.raw.csv "$OUT"/run_*.quantized.csv "$OUT"/run_*.summary \
      "$OUT"/run_*.stdout "$OUT"/run_*.exit "$OUT"/run_*.sha256

# The production guidance chain is compiled from source and linked against the actual JSBSim static
# library: the coordinator (which owns the real PX4 NPFG and TECS adapters) and the F-16 stick
# adapter are the same translation units the game builds.
#
# -Wno-unused-parameter matches the committed build_verify_guidance_stick_integration_v2.sh: the
# vendored PX4 v1.17.0 NPFG/TECS sources carry unused parameters in their upstream signatures, and
# the pinned reference is not modified to silence them. Every other warning stays an error.
"$TC/bin/clang++" \
  -std=c++20 -O2 -Wall -Wextra -Werror -pedantic -Wno-unused-parameter \
  -nostdinc++ -I"$LC/include" -I"$LC/include/c++/v1" \
  -isystem "$PLUGIN/Source/ThirdParty/JSBSim/Include" \
  -I"$REPO_ROOT/Source/MUMT_Sim/Public" \
  "$SCRIPT_DIR/verify_f16_lateral_dynamics_isolation_v2.cpp" \
  "$REPO_ROOT/Source/MUMT_Sim/Private/FormationControl/Px4TecsAdapter.cpp" \
  "$REPO_ROOT/Source/MUMT_Sim/Private/FormationControl/Px4NpfgAdapter.cpp" \
  "$REPO_ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationGuidanceCoordinatorV2.cpp" \
  "$REPO_ROOT/Source/MUMT_Sim/Private/FormationControlV2/F16StickAdapterV2.cpp" \
  "$PLUGIN/Source/ThirdParty/JSBSim/Lib/Linux/libJSBSim.a" \
  "$LC/lib/Unix/x86_64-unknown-linux-gnu/libc++.a" \
  "$LC/lib/Unix/x86_64-unknown-linux-gnu/libc++abi.a" \
  -fuse-ld=lld -lm -lpthread -ldl -o "$BINARY"

for run in 1 2 3; do
  set +e
  "$BINARY" "$JSBSIM_ROOT" "$OUT/run_${run}.raw.csv" "$OUT/run_${run}.quantized.csv" \
    "$OUT/run_${run}.summary" >"$OUT/run_${run}.stdout" 2>&1
  rc=$?
  set -e
  printf '%d\n' "$rc" >"$OUT/run_${run}.exit"
  if [[ $rc -ne 0 ]]; then
    cat "$OUT/run_${run}.stdout"
    exit "$rc"
  fi
  sha256sum "$OUT/run_${run}.raw.csv" >"$OUT/run_${run}.sha256"
  sha256sum "$OUT/run_${run}.quantized.csv" >>"$OUT/run_${run}.sha256"
  printf 'raw_sha256=%s\n' "$(sha256sum "$OUT/run_${run}.raw.csv" | awk '{print $1}')" \
    >>"$OUT/run_${run}.summary"
  printf 'quantized_sha256=%s\n' "$(sha256sum "$OUT/run_${run}.quantized.csv" | awk '{print $1}')" \
    >>"$OUT/run_${run}.summary"
done

# three independent processes must produce byte-identical artifacts
for run in 2 3; do
  cmp --silent "$OUT/run_1.raw.csv" "$OUT/run_${run}.raw.csv"
  cmp --silent "$OUT/run_1.quantized.csv" "$OUT/run_${run}.quantized.csv"
  cmp --silent "$OUT/run_1.summary" "$OUT/run_${run}.summary"
done

cat "$OUT/run_1.summary"
printf 'determinism_runs=3 process_exits=%s/%s/%s\n' \
  "$(cat "$OUT/run_1.exit")" "$(cat "$OUT/run_2.exit")" "$(cat "$OUT/run_3.exit")"
printf 'F16_LATERAL_DYNAMICS_ISOLATION_V2_RESULT=PASS\n'
