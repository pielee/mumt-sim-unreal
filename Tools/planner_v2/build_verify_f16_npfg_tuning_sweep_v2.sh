#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
TC="$HOME/unreal/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v22_clang-16.0.6-centos7/x86_64-unknown-linux-gnu"
LC="$HOME/unreal/Engine/Source/ThirdParty/Unix/LibCxx"
PLUGIN="$REPO_ROOT/Plugins/JSBSimFlightDynamicsModel"
JSBSIM_ROOT="$PLUGIN/Resources/JSBSim"
OUT=/tmp/mumt_f16_npfg_tuning_sweep_v2
BINARY="$OUT/verify_f16_npfg_tuning_sweep_v2"

mkdir -p "$OUT"
rm -f "$OUT"/sweep.* "$OUT"/confirm_*.* "$OUT"/candidates.txt

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
  "$SCRIPT_DIR/verify_f16_npfg_tuning_sweep_v2.cpp" \
  "$REPO_ROOT/Source/MUMT_Sim/Private/FormationControl/Px4TecsAdapter.cpp" \
  "$REPO_ROOT/Source/MUMT_Sim/Private/FormationControl/Px4NpfgAdapter.cpp" \
  "$REPO_ROOT/Source/MUMT_Sim/Private/FormationControlV2/FormationGuidanceCoordinatorV2.cpp" \
  "$REPO_ROOT/Source/MUMT_Sim/Private/FormationControlV2/F16StickAdapterV2.cpp" \
  "$PLUGIN/Source/ThirdParty/JSBSim/Lib/Linux/libJSBSim.a" \
  "$LC/lib/Unix/x86_64-unknown-linux-gnu/libc++.a" \
  "$LC/lib/Unix/x86_64-unknown-linux-gnu/libc++abi.a" \
  -fuse-ld=lld -lm -lpthread -ldl -o "$BINARY"

# ---- 1. coarse sweep: 96 grid combinations + the current production default -----------------------
set +e
"$BINARY" "$JSBSIM_ROOT" sweep "$OUT/sweep.raw.csv" "$OUT/sweep.quantized.csv" \
  "$OUT/sweep.summary" "$OUT/candidates.txt" >"$OUT/sweep.stdout" 2>&1
rc=$?
set -e
printf '%d\n' "$rc" >"$OUT/sweep.exit"
if [[ $rc -ne 0 ]]; then
  cat "$OUT/sweep.stdout"
  exit "$rc"
fi

# ---- 2. focused confirmation of the leading candidates, three independent processes --------------
if [[ ! -s "$OUT/candidates.txt" ]]; then
  echo "no candidate survived the pre-declared reject gates; nothing to confirm"
else
  for run in 1 2 3; do
    set +e
    "$BINARY" "$JSBSIM_ROOT" confirm "$OUT/confirm_${run}.raw.csv" "$OUT/confirm_${run}.quantized.csv" \
      "$OUT/confirm_${run}.summary" "$OUT/candidates.txt" >"$OUT/confirm_${run}.stdout" 2>&1
    rc=$?
    set -e
    printf '%d\n' "$rc" >"$OUT/confirm_${run}.exit"
    if [[ $rc -ne 0 ]]; then
      cat "$OUT/confirm_${run}.stdout"
      exit "$rc"
    fi
  done
  for run in 2 3; do
    cmp --silent "$OUT/confirm_1.raw.csv" "$OUT/confirm_${run}.raw.csv"
    cmp --silent "$OUT/confirm_1.quantized.csv" "$OUT/confirm_${run}.quantized.csv"
    cmp --silent "$OUT/confirm_1.summary" "$OUT/confirm_${run}.summary"
  done
fi

cat "$OUT/sweep.summary"
printf 'sweep_raw_sha256=%s\n' "$(sha256sum "$OUT/sweep.raw.csv" | awk '{print $1}')"
printf 'sweep_quantized_sha256=%s\n' "$(sha256sum "$OUT/sweep.quantized.csv" | awk '{print $1}')"
if [[ -s "$OUT/candidates.txt" ]]; then
  printf 'confirm_raw_sha256=%s\n' "$(sha256sum "$OUT/confirm_1.raw.csv" | awk '{print $1}')"
  printf 'confirm_quantized_sha256=%s\n' "$(sha256sum "$OUT/confirm_1.quantized.csv" | awk '{print $1}')"
  printf 'confirmation_runs=3 process_exits=%s/%s/%s\n' \
    "$(cat "$OUT/confirm_1.exit")" "$(cat "$OUT/confirm_2.exit")" "$(cat "$OUT/confirm_3.exit")"
fi
printf 'F16_NPFG_TUNING_SWEEP_V2_RESULT=PASS\n'
