#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
TC="$HOME/unreal/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v22_clang-16.0.6-centos7/x86_64-unknown-linux-gnu"
LC="$HOME/unreal/Engine/Source/ThirdParty/Unix/LibCxx"
PLUGIN="$REPO_ROOT/Plugins/JSBSimFlightDynamicsModel"
JSBSIM_ROOT="$PLUGIN/Resources/JSBSim"
OUT=/tmp/mumt_f16_vertical_performance_v2
BINARY="$OUT/verify_f16_vertical_performance_v2"

mkdir -p "$OUT"
rm -f "$OUT"/run_*.raw.csv "$OUT"/run_*.quantized.csv "$OUT"/run_*.summary \
      "$OUT"/run_*.stdout "$OUT"/run_*.exit "$OUT"/run_*.raw.sha256 "$OUT"/run_*.quantized.sha256

"$TC/bin/clang++" \
  -std=c++17 -O2 -Wall -Wextra -Werror -pedantic \
  -nostdinc++ -I"$LC/include" -I"$LC/include/c++/v1" \
  -isystem "$PLUGIN/Source/ThirdParty/JSBSim/Include" \
  "$SCRIPT_DIR/verify_f16_vertical_performance_v2.cpp" \
  "$PLUGIN/Source/ThirdParty/JSBSim/Lib/Linux/libJSBSim.a" \
  "$LC/lib/Unix/x86_64-unknown-linux-gnu/libc++.a" \
  "$LC/lib/Unix/x86_64-unknown-linux-gnu/libc++abi.a" \
  -fuse-ld=lld -lm -lpthread -ldl -o "$BINARY"

for run in 1 2 3; do
  set +e
  "$BINARY" "$JSBSIM_ROOT" "$OUT/run_${run}.raw.csv" "$OUT/run_${run}.quantized.csv" "$OUT/run_${run}.summary" \
    >"$OUT/run_${run}.stdout" 2>&1
  rc=$?
  set -e
  printf '%d\n' "$rc" >"$OUT/run_${run}.exit"
  if [[ $rc -ne 0 ]]; then
    cat "$OUT/run_${run}.stdout"
    exit "$rc"
  fi
  sha256sum "$OUT/run_${run}.raw.csv" >"$OUT/run_${run}.raw.sha256"
  sha256sum "$OUT/run_${run}.quantized.csv" >"$OUT/run_${run}.quantized.sha256"
  printf 'raw_sha256=%s\n' "$(awk '{print $1}' "$OUT/run_${run}.raw.sha256")" >>"$OUT/run_${run}.summary"
  printf 'quantized_sha256=%s\n' "$(awk '{print $1}' "$OUT/run_${run}.quantized.sha256")" >>"$OUT/run_${run}.summary"
done

cmp --silent "$OUT/run_1.raw.csv" "$OUT/run_2.raw.csv"
cmp --silent "$OUT/run_1.raw.csv" "$OUT/run_3.raw.csv"
cmp --silent "$OUT/run_1.quantized.csv" "$OUT/run_2.quantized.csv"
cmp --silent "$OUT/run_1.quantized.csv" "$OUT/run_3.quantized.csv"
cmp --silent "$OUT/run_1.summary" "$OUT/run_2.summary"
cmp --silent "$OUT/run_1.summary" "$OUT/run_3.summary"

cat "$OUT/run_1.summary"
printf 'determinism_runs=3 process_exits=%s/%s/%s\n' \
  "$(cat "$OUT/run_1.exit")" "$(cat "$OUT/run_2.exit")" "$(cat "$OUT/run_3.exit")"
printf 'F16_VERTICAL_PERFORMANCE_V2_RESULT=PASS\n'
