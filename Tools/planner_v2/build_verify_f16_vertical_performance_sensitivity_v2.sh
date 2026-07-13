#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../.." && pwd)
TC="$HOME/unreal/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v22_clang-16.0.6-centos7/x86_64-unknown-linux-gnu"
LC="$HOME/unreal/Engine/Source/ThirdParty/Unix/LibCxx"
PLUGIN="$REPO_ROOT/Plugins/JSBSimFlightDynamicsModel"
JSBSIM_ROOT="$PLUGIN/Resources/JSBSim"
OUT=/tmp/mumt_f16_vertical_performance_sensitivity_v2
BINARY="$OUT/verify_f16_vertical_performance_sensitivity_v2"

# Canonical hashes of the committed V1 baseline harness (bb9af6a, 10000 ft / 3000 lb, 28 points).
# The sensitivity matrix must reproduce that condition byte-for-byte in V1's schema.
V1_RAW_SHA256=2a138a7de311b579ad735a10c60d99874048180ba58d124e415661d402f0374a
V1_QUANTIZED_SHA256=6ae72eda2fc31125940b01800e2f80051b2c8001c0fa1bd994d47cbbd4f0c244

mkdir -p "$OUT"
rm -f "$OUT"/run_*.matrix.raw.csv "$OUT"/run_*.matrix.quantized.csv \
      "$OUT"/run_*.baseline_reproduction.raw.csv "$OUT"/run_*.baseline_reproduction.quantized.csv \
      "$OUT"/run_*.summary "$OUT"/run_*.stdout "$OUT"/run_*.exit "$OUT"/run_*.sha256

"$TC/bin/clang++" \
  -std=c++17 -O2 -Wall -Wextra -Werror -pedantic \
  -nostdinc++ -I"$LC/include" -I"$LC/include/c++/v1" \
  -isystem "$PLUGIN/Source/ThirdParty/JSBSim/Include" \
  "$SCRIPT_DIR/verify_f16_vertical_performance_sensitivity_v2.cpp" \
  "$PLUGIN/Source/ThirdParty/JSBSim/Lib/Linux/libJSBSim.a" \
  "$LC/lib/Unix/x86_64-unknown-linux-gnu/libc++.a" \
  "$LC/lib/Unix/x86_64-unknown-linux-gnu/libc++abi.a" \
  -fuse-ld=lld -lm -lpthread -ldl -o "$BINARY"

for run in 1 2 3; do
  set +e
  "$BINARY" "$JSBSIM_ROOT" \
    "$OUT/run_${run}.matrix.raw.csv" "$OUT/run_${run}.matrix.quantized.csv" \
    "$OUT/run_${run}.baseline_reproduction.raw.csv" "$OUT/run_${run}.baseline_reproduction.quantized.csv" \
    "$OUT/run_${run}.summary" >"$OUT/run_${run}.stdout" 2>&1
  rc=$?
  set -e
  printf '%d\n' "$rc" >"$OUT/run_${run}.exit"
  if [[ $rc -ne 0 ]]; then
    cat "$OUT/run_${run}.stdout"
    exit "$rc"
  fi
  for artifact in matrix.raw matrix.quantized baseline_reproduction.raw baseline_reproduction.quantized; do
    sha256sum "$OUT/run_${run}.${artifact}.csv" >>"$OUT/run_${run}.sha256"
    printf '%s_sha256=%s\n' "${artifact//./_}" \
      "$(sha256sum "$OUT/run_${run}.${artifact}.csv" | awk '{print $1}')" >>"$OUT/run_${run}.summary"
  done
done

# determinism: three independent processes must produce byte-identical artifacts
for run in 2 3; do
  cmp --silent "$OUT/run_1.matrix.raw.csv" "$OUT/run_${run}.matrix.raw.csv"
  cmp --silent "$OUT/run_1.matrix.quantized.csv" "$OUT/run_${run}.matrix.quantized.csv"
  cmp --silent "$OUT/run_1.baseline_reproduction.raw.csv" "$OUT/run_${run}.baseline_reproduction.raw.csv"
  cmp --silent "$OUT/run_1.baseline_reproduction.quantized.csv" "$OUT/run_${run}.baseline_reproduction.quantized.csv"
  cmp --silent "$OUT/run_1.summary" "$OUT/run_${run}.summary"
done

# baseline reproduction gate: byte comparison against the committed V1 harness output
BASE_RAW=$(sha256sum "$OUT/run_1.baseline_reproduction.raw.csv" | awk '{print $1}')
BASE_QUANT=$(sha256sum "$OUT/run_1.baseline_reproduction.quantized.csv" | awk '{print $1}')
baseline_ok=1
if [[ "$BASE_RAW" != "$V1_RAW_SHA256" ]]; then
  baseline_ok=0
  printf 'BASELINE_REPRODUCTION_FAIL raw expected=%s actual=%s\n' "$V1_RAW_SHA256" "$BASE_RAW"
fi
if [[ "$BASE_QUANT" != "$V1_QUANTIZED_SHA256" ]]; then
  baseline_ok=0
  printf 'BASELINE_REPRODUCTION_FAIL quantized expected=%s actual=%s\n' "$V1_QUANTIZED_SHA256" "$BASE_QUANT"
fi

cat "$OUT/run_1.summary"
printf 'baseline_reproduction_expected_raw_sha256=%s\n' "$V1_RAW_SHA256"
printf 'baseline_reproduction_expected_quantized_sha256=%s\n' "$V1_QUANTIZED_SHA256"
printf 'baseline_reproduction_actual_raw_sha256=%s\n' "$BASE_RAW"
printf 'baseline_reproduction_actual_quantized_sha256=%s\n' "$BASE_QUANT"
printf 'determinism_runs=3 process_exits=%s/%s/%s\n' \
  "$(cat "$OUT/run_1.exit")" "$(cat "$OUT/run_2.exit")" "$(cat "$OUT/run_3.exit")"

if [[ "$baseline_ok" -ne 1 ]]; then
  printf 'F16_VERTICAL_PERFORMANCE_SENSITIVITY_V2_RESULT=FAIL\n'
  exit 1
fi
printf 'baseline_reproduction=PASS\n'
printf 'F16_VERTICAL_PERFORMANCE_SENSITIVITY_V2_RESULT=PASS\n'
