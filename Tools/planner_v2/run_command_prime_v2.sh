#!/usr/bin/env bash
# run_command_prime_v2.sh — the Phase B gate: the bumpless Legacy -> Formation handoff.
#
# What this gate proves is narrow, and deliberately so:
#
#     the FIRST LegacyOrManual -> FormationControlV2 handoff has ZERO command step.
#
# Not "small". Zero -- on all five controlled fields, exactly. The baseline it is continuous with is the
# FINAL RESOLVED BLOCK the FDM last consumed, not the mutable legacy writer block (which is only an
# input and can be a shifting field-wise mixture of several writers).
#
# It does NOT claim the reverse direction is bumpless. Formation -> Legacy is an immediate safety
# fallback: a controller that has gone stale or non-finite is abandoned instantly, not blended out of.
#
# One test per editor process (the first-PIE expected-warning problem is documented in
# docs/CONTROL_V2_AIRBORNE_AUTOMATION.md). Run EXCLUSIVELY: the other gates bind UDP 5005 and a second
# editor can steal a datagram, producing a failure that has nothing to do with the code.
set -uo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
PROJECT="$REPO_ROOT/MUMT_Sim.uproject"
UE_ROOT="${UE_ROOT:-$HOME/unreal}"
EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd"
OUT_ROOT="${MUMT_PRIME_OUT:-/tmp/mumt_command_prime_v2}"
TIMEOUT_S="${MUMT_PRIME_TIMEOUT_S:-1800}"

SCENARIOS=(
  "no_snapshot|MUMT.ControlV2.PrimeNoResolvedSnapshot"
  "snapshot_exact|MUMT.ControlV2.PrimeSnapshotExact"
  "generation|MUMT.ControlV2.PrimeGeneration"
  "exact_handoff|MUMT.ControlV2.PrimeExactHandoff"
  "unprimed_rejected|MUMT.ControlV2.PrimeUnprimedRejected"
  "wrong_generation|MUMT.ControlV2.PrimeWrongGeneration"
  "stale|MUMT.ControlV2.PrimeStale"
  "non_finite|MUMT.ControlV2.PrimeNonFinite"
  "falling_preemption|MUMT.ControlV2.PrimeFallingPreemption"
  "per_aircraft_isolation|MUMT.ControlV2.PrimePerAircraftIsolation"
  "world_cleanup|MUMT.ControlV2.PrimeWorldCleanup"
  "reset_session_safety|MUMT.ControlV2.PrimeResetSessionSafety"
  "resolver_ownership_lost|MUMT.ControlV2.PrimeResolverOwnershipLost"
  "intervening_consume|MUMT.ControlV2.PrimeInterveningConsumeRejected"
  "stick_exact_first_compute|MUMT.ControlV2.PrimeStickExactFirstCompute"
  "handoff_delta_negative|MUMT.ControlV2.PrimeHandoffDeltaNegativeControl"
  "stick_wrong_identity|MUMT.ControlV2.PrimeStickWrongIdentityDoesNotConsume"
  "stick_latched_no_advance|MUMT.ControlV2.PrimeStickLatchedFrameNoStateAdvance"
  "stick_invalid_prime|MUMT.ControlV2.PrimeStickInvalidPrimeRejected"
  "boundary_stale|MUMT.ControlV2.PrimeActivationBoundaryStaleRejected"
)

fail() { echo "FAIL: $*" >&2; echo "COMMAND_PRIME_V2_RESULT=FAIL"; exit 1; }

[[ -f "$PROJECT" ]] || fail "project not found: $PROJECT"
[[ -x "$EDITOR"  ]] || fail "UnrealEditor-Cmd not found: $EDITOR (set UE_ROOT)"

# Match the EXECUTABLE, not any command line that merely mentions it -- a plain `pgrep -f UnrealEditor-Cmd`
# also matches the shell running this script and would report an editor that does not exist.
EDITOR_PROC_RE='[/]UnrealEditor-Cmd([[:space:]]|$)'
if pgrep -f "$EDITOR_PROC_RE" >/dev/null 2>&1; then
  fail "another UnrealEditor-Cmd is already running. Run this gate exclusively."
fi

# ---- registration ----------------------------------------------------------------------------------
# A filter that matches nothing does not fail -- it runs zero tests and exits clean. That is the most
# dangerous failure mode a gate has: green, having proved nothing.
DISCOVER="$OUT_ROOT/discover"; mkdir -p "$DISCOVER"
timeout "$TIMEOUT_S" "$EDITOR" "$PROJECT" -ExecCmds="Automation List; Quit" \
  -unattended -nopause -nosplash -nullrhi -abslog="$DISCOVER/discover.log" >/dev/null 2>&1
drc=$?
[[ $drc -eq 124 ]] && fail "discovery timed out"
[[ $drc -eq 0 ]] || fail "discovery process exited $drc"
grep -qa "TEST COMPLETE" "$DISCOVER/discover.log" || fail "discovery produced no TEST COMPLETE marker"
for s in "${SCENARIOS[@]}"; do
  name=$(cut -d'|' -f2 <<<"$s")
  grep -qa -- "$name" "$DISCOVER/discover.log" || fail "NOT REGISTERED: $name"
done
echo "registration: ${#SCENARIOS[@]}/${#SCENARIOS[@]} prime tests found in the live engine"

# ---- one test, one editor process ------------------------------------------------------------------
run_one() {
  local slug="$1" name="$2" dir="$OUT_ROOT/$1"
  rm -rf "$dir"; mkdir -p "$dir/report"
  local log="$dir/automation.log"
  local -a cmd=("$EDITOR" "$PROJECT"
    -ExecCmds="Automation RunTests ${name}; Quit"
    -ReportExportPath="$dir/report"
    -FormationTest                       # the aircraft must actually be flying to have a baseline
    -unattended -nopause -nosplash -nullrhi -abslog="$log")

  {
    printf 'slug=%s\ntest=%s\n' "$slug" "$name"
    printf 'engine_root=%s\nproject=%s\n' "$UE_ROOT" "$PROJECT"
    printf 'started_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'command=%s\n' "$(printf '%q ' "${cmd[@]}")"
  } >"$dir/invocation.txt"

  timeout "$TIMEOUT_S" "${cmd[@]}" >/dev/null 2>&1
  local rc=$?
  printf 'ended_utc=%s\nprocess_exit=%d\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$rc" >>"$dir/invocation.txt"
  [[ $rc -eq 124 ]] && { echo "TIMEOUT: $name" >&2; return 1; }

  local crash=0
  for m in "Fatal error" "Assertion failed" "Ensure condition failed" "=== Critical error ===" "SIGSEGV"; do
    grep -qa -- "$m" "$log" && { echo "CRASH/FATAL ($name): $m" >&2; crash=1; }
  done

  local code
  code=$(grep -aoE '\*\*\*\* TEST COMPLETE\. EXIT CODE: -?[0-9]+ \*\*\*\*' "$log" | tail -1 \
         | sed -nE 's/.*EXIT CODE: (-?[0-9]+).*/\1/p')

  grep -a "\[PRIME\]"   "$log" | sed 's/^.*\[PRIME\]/[PRIME]/'     >"$dir/prime.txt"   || true
  grep -a "\[ARBITER\]" "$log" | sed 's/^.*\[ARBITER\]/[ARBITER]/' >"$dir/arbiter.txt" || true

  python3 - "$dir/report/index.json" "$dir/summary.txt" "$dir/durations.txt" "$name" <<'PY'
import json, sys
report, summary_path, durations_path, expected = sys.argv[1:5]
d = json.load(open(report, encoding="utf-8-sig"))   # the engine writes a UTF-8 BOM
tests = d.get("tests", [])
# a passing test that logs a warning lands in succeededWithWarnings, and this scenario always trips the
# pre-existing F16_UAV Blueprint warning -- requiring succeeded==1 would reject a healthy run
succ = (d.get("succeeded") or 0) + (d.get("succeededWithWarnings") or 0)
state = tests[0].get("state") if len(tests) == 1 else None
lines = [
    "requested=%s" % expected,
    "executed_count=%d" % len(tests),
    "executed=%s" % ",".join(t.get("fullTestPath", "") for t in tests),
    "state=%s" % (state or "N/A"),
    "passed_count=%d failed=%s notRun=%s inProcess=%s" % (succ, d.get("failed"), d.get("notRun"), d.get("inProcess")),
]
open(summary_path, "w").write("\n".join(lines) + "\n")
sys.stdout.write("\n".join(lines) + "\n")
with open(durations_path, "w") as f:
    for t in tests:
        dur = t.get("duration")
        f.write("test %s duration_s=%s\n" % (t.get("fullTestPath"),
                ("%.1f" % dur) if isinstance(dur, (int, float)) else "NA"))
bad = 0
if len(tests) != 1 or (tests and tests[0].get("fullTestPath") != expected):
    bad = 1; print("wrong/missing test", file=sys.stderr)
if state != "Success": bad = 1; print("state=%r" % state, file=sys.stderr)
if succ != 1: bad = 1; print("passed_count=%d" % succ, file=sys.stderr)
if d.get("failed") or d.get("notRun") or d.get("inProcess"):
    bad = 1; print("nonzero failed/notRun/inProcess", file=sys.stderr)
sys.exit(bad)
PY
  local verdict=$?

  printf 'engine_exit_code=%s\nprocess_exit=%d\ncrash=%d\n' "${code:-ABSENT}" "$rc" "$crash" >>"$dir/summary.txt"
  echo "  ${slug}: exit=$rc engine_exit=${code:-ABSENT} crash=$crash verdict=$verdict"
  [[ $verdict -eq 0 && $crash -eq 0 && $rc -eq 0 && "$code" == "0" ]] || return 1
  return 0
}

for s in "${SCENARIOS[@]}"; do
  slug=$(cut -d'|' -f1 <<<"$s"); name=$(cut -d'|' -f2 <<<"$s")
  run_one "$slug" "$name" || fail "$slug"
done

# ---- the handoff claim, restated from the run itself ------------------------------------------------
echo
echo "================ PRIME MEASUREMENTS ================"
for s in "${SCENARIOS[@]}"; do
  slug=$(cut -d'|' -f1 <<<"$s")
  echo "--- $slug ---"
  grep -E "TOTALS|HANDOFF|AIRCRAFT|D_RESULT|I_RESULT|K_RESULT|N_RESULT|O_RESULT|P_RESULT|Q_RESULT|R_RESULT|S_RESULT|T_RESULT" "$OUT_ROOT/$slug/prime.txt" 2>/dev/null \
    | grep -v "\[log\]" | head -6 || echo "  (no [PRIME] lines)"
done

# The load-bearing assertion, taken from the artifact rather than trusted to the test alone.
d_line=$(grep -m1 "D_RESULT" "$OUT_ROOT/exact_handoff/prime.txt" 2>/dev/null || true)
[[ -n "$d_line" ]] || fail "exact_handoff produced no D_RESULT line"
grep -qE "dAil=0\.000000000 dElv=0\.000000000 dRud=0\.000000000 dThr=0\.000000000 dSpb=0\.000000000" <<<"$d_line" \
  || fail "the first Legacy->Formation handoff STEPPED: $d_line"
echo "handoff: the first Legacy->Formation command step is exactly zero on all five controlled fields"

# NEGATIVE CONTROL. Without this, the zero above could just be a value compared with itself.
p_line=$(grep -m1 "P_RESULT" "$OUT_ROOT/handoff_delta_negative/prime.txt" 2>/dev/null || true)
[[ -n "$p_line" ]] || fail "handoff_delta_negative produced no P_RESULT line"
# The offset is chosen by the test at runtime (it must stay inside [-1,1] relative to the live baseline),
# so the gate compares the MEASURED delta against the EXPECTED one the test printed -- hardcoding +0.1
# here would break the moment the baseline sits above 0.8 and the test correctly flips to -0.1.
p_meas=$(sed -nE 's/.*dAil=(-?[0-9.]+).*/\1/p' <<<"$p_line")
p_exp=$(sed -nE 's/.*\(expected (-?[0-9.]+)\).*/\1/p' <<<"$p_line")
[[ -n "$p_meas" && -n "$p_exp" ]] || fail "could not read the measured/expected aileron delta: $p_line"
[[ "$p_meas" == "$p_exp" ]] || fail "the negative control did NOT see the deliberate step (measured=$p_meas expected=$p_exp) -- the delta measurement is a tautology"
grep -qE "expected (0\.100000000|-0\.100000000)\)" <<<"$p_line" || fail "the offset is not the declared +/-0.1: $p_line"
grep -qE "dElv=0\.000000000 dRud=0\.000000000 dThr=0\.000000000 dSpb=0\.000000000 non_zero=1" <<<"$p_line" \
  || fail "the negative control moved something other than aileron, or failed to count the step: $p_line"
echo "negative control: the deliberate ${p_exp} aileron offset IS measured as ${p_meas} (the delta is measured, not assumed)"

# A candidate that was fresh when it was SUBMITTED must still be refused if it has rotted by the time the
# handoff actually lands. And it must be refused for THAT reason -- a stale candidate and a moved-on
# baseline are different failures, and a gate that accepted either would not know which one it proved.
t_line=$(grep -m1 "T_RESULT" "$OUT_ROOT/boundary_stale/prime.txt" 2>/dev/null || true)
[[ -n "$t_line" ]] || fail "boundary_stale produced no T_RESULT line"
grep -qE "stale_at_boundary=1 intervening=0 formation=0 handoffs=0 activations_granted=0 mode=LegacyOrManual" <<<"$t_line" \
  || fail "the stale candidate was NOT refused at the consume boundary, or was refused for the wrong reason: $t_line"
echo "boundary: a candidate that went stale between submission and the handoff is refused AS STALE, and Formation never resolves"

# The stick must reproduce the baseline on its FIRST compute, through its own Update().
o_line=$(grep -m1 "O_RESULT" "$OUT_ROOT/stick_exact_first_compute/prime.txt" 2>/dev/null || true)
[[ -n "$o_line" ]] || fail "stick_exact_first_compute produced no O_RESULT line"
echo "stick: the first compute after a prime reproduces the baseline exactly, then the normal path resumes"

echo "COMMAND_PRIME_V2_RESULT=PASS"
