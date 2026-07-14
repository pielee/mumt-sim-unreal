#!/usr/bin/env bash
# run_airborne_control_v2_automation.sh — the ControlV2 Airborne shadow RUNTIME gate.
#
# Every other ControlV2 verification in this repo is a HOST harness: it links the production
# controllers against JSBSim directly and never loads Unreal. That leaves one thing unproven --
# that the same chain still behaves inside a real Unreal world, driven by the real command writer,
# on a real map. These three Automation tests are the only coverage of that, and until this runner
# existed nothing in the repo invoked them, so in practice they never ran.
#
# ONE TEST PER EDITOR PROCESS. This is not a stylistic choice, it is forced:
# the tests each call AddExpectedErrorPlain("bUseAttachParentBound") with an occurrence count of 0,
# which in UE means "must occur AT LEAST ONCE". That Blueprint construction-script warning is emitted
# only on the FIRST PIE session of an editor process -- the asset is compiled once and stays compiled.
# So in a shared process the first test consumes the warning and every later test fails with
# "Expected suppressed ... did not occur", purely as a function of execution order. Giving each test
# its own process makes each one the first PIE, and the expectation is deterministic again.
#
# Pass/fail comes from the Automation report, NOT from the process exit code alone: docs/STATE_API.md
# records a prior run of this project where the verdict was Success while the editor still exited 1.
# All signals are captured and each must independently agree.
#
# It writes nothing into the repository and it never calls `git clean`.
set -uo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
PROJECT="$REPO_ROOT/MUMT_Sim.uproject"
UE_ROOT="${UE_ROOT:-$HOME/unreal}"
EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd"
OUT_ROOT="${MUMT_AUTOMATION_OUT:-/tmp/mumt_control_v2_airborne_automation}"

# Exact registered names, from IMPLEMENT_SIMPLE_AUTOMATION_TEST in
# Source/MUMT_Sim/Private/State/MumtAirborneShadowTest.cpp.
# Flags: EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter.
# Fixed order; each entry is "<slug> <exact test name>", single-space separated (slug=first field,
# name=last field -- the names contain no spaces, so no alignment padding is allowed here).
TESTS=(
  "planner MUMT.ControlV2.AirborneNearFieldPlannerShadow"
  "envelope MUMT.ControlV2.AirborneNearFieldEnvelopeRejectionShadow"
  "full_control MUMT.ControlV2.AirborneFullControlShadow"
)

# The tests load /Game/RL_2 and drive PIE. The aircraft are flown by the EXISTING writer
# (AUDPControlReceiver -> InnerLoopAutopilot), whose scripted formation profile is armed ONLY by the
# -FormationTest switch (UDPControlReceiver.cpp). Without it the aircraft never fly the profile and
# the tests fail their airborne-sample assertions.
#
# -FormationTestExit is deliberately NOT passed: it makes the writer exit the process when the profile
# finishes, which would kill the editor mid-Automation and destroy the result report.
#
# Each test self-terminates (kMaxSimSeconds=300, hard wall cap kMaxWallSeconds=420). One test per
# process, so the timeout only has to cover editor startup + map load + one test.
TIMEOUT_S="${MUMT_AUTOMATION_TIMEOUT_S:-1800}"

fail() { echo "FAIL: $*" >&2; echo "AIRBORNE_CONTROL_V2_AUTOMATION_RESULT=FAIL"; exit 1; }

[[ -f "$PROJECT" ]] || fail "project not found: $PROJECT"
[[ -x "$EDITOR"  ]] || fail "UnrealEditor-Cmd not found or not executable: $EDITOR (set UE_ROOT)"

# ---- 1. discovery: the tests must EXIST before we claim to have run them --------------------------
# A filter that matches nothing does not make Automation fail -- it runs zero tests and exits cleanly.
# That is the single most dangerous failure mode for a gate: green, having proved nothing. So the names
# are confirmed against the live engine's own test list first, once, in its own process.
DISCOVER_DIR="$OUT_ROOT/discover"
mkdir -p "$DISCOVER_DIR"
DISCOVER_LOG="$DISCOVER_DIR/discover.log"
timeout "$TIMEOUT_S" "$EDITOR" "$PROJECT" \
  -ExecCmds="Automation List; Quit" \
  -unattended -nopause -nosplash -nullrhi \
  -abslog="$DISCOVER_LOG" >/dev/null 2>&1
[[ $? -eq 124 ]] && fail "test discovery timed out after ${TIMEOUT_S}s"

for entry in "${TESTS[@]}"; do
  name=${entry##* }
  grep -qa -- "$name" "$DISCOVER_LOG" || fail "NOT REGISTERED in this build: $name"
done
echo "registration: ${#TESTS[@]}/${#TESTS[@]} Airborne shadow tests found in the live engine test list"

# ---- 2. one test, one editor process ---------------------------------------------------------------
run_test() {
  local rep="$1" slug="$2" name="$3"
  local dir="$OUT_ROOT/repetition_${rep}/${slug}"
  rm -rf "$dir"; mkdir -p "$dir/report"
  local log="$dir/automation.log"
  local started ended
  started=$(date -u +%Y-%m-%dT%H:%M:%SZ)

  # `; Quit` is what gives the process a MEANINGFUL exit status: the Automation command handler sets
  # GIsCriticalError when any report has errors and only then exits non-zero (AutomationCommandline.cpp).
  # Without a queued Quit the editor lingers and the exit code says nothing.
  local -a cmd=(
    "$EDITOR" "$PROJECT"
    -ExecCmds="Automation RunTests ${name}; Quit"
    -ReportExportPath="$dir/report"
    -FormationTest
    -unattended -nopause -nosplash -nullrhi
    -abslog="$log"
  )

  timeout "$TIMEOUT_S" "${cmd[@]}" >/dev/null 2>&1
  local rc=$?
  ended=$(date -u +%Y-%m-%dT%H:%M:%SZ)

  {
    printf 'repetition=%s\ntest=%s\n' "$rep" "$name"
    printf 'engine_root=%s\neditor=%s\nproject=%s\n' "$UE_ROOT" "$EDITOR" "$PROJECT"
    printf 'started_utc=%s\nended_utc=%s\ntimeout_s=%s\n' "$started" "$ended" "$TIMEOUT_S"
    printf 'command=%s\n' "$(printf '%q ' "${cmd[@]}")"
    printf 'process_exit=%d\n' "$rc"
  } >"$dir/invocation.txt"

  [[ $rc -eq 124 ]] && { echo "TIMEOUT after ${TIMEOUT_S}s: $name" >&2; return 1; }

  # Narrow, specific markers. A bare "Error" grep would trip on the pre-existing F16_UAV Blueprint
  # construction-script error, which the test itself declares expected via AddExpectedErrorPlain.
  local crash=0
  for m in "Fatal error" "Assertion failed" "Ensure condition failed" "=== Critical error ===" "SIGSEGV" "Signal 11"; do
    grep -qa -- "$m" "$log" && { echo "CRASH/FATAL marker ($name): $m" >&2; crash=1; }
  done

  local report="$dir/report/index.json"
  [[ -s "$report" ]] || { echo "no Automation report written: $report" >&2; return 1; }

  python3 - "$report" "$dir/summary.txt" "$dir/durations.txt" "$name" <<'PY'
import json, sys
report, summary_path, durations_path, expected = sys.argv[1:5]
# The engine writes index.json with a UTF-8 BOM; a plain utf-8 load raises on it.
d = json.load(open(report, encoding="utf-8-sig"))
tests = d.get("tests", [])

succeeded = d.get("succeeded", 0) or 0
with_warnings = d.get("succeededWithWarnings", 0) or 0
failed = d.get("failed", 0) or 0
not_run = d.get("notRun", 0) or 0
in_process = d.get("inProcess", 0) or 0

# A test that PASSES but logs a warning is counted under succeededWithWarnings, not succeeded. This
# scenario always trips the pre-existing F16_UAV Blueprint warning, so requiring succeeded==1 would
# reject a genuinely passing run. The warning is neither suppressed nor deleted -- it is just counted
# where the engine actually puts it.
passed_count = succeeded + with_warnings

state = tests[0].get("state") if len(tests) == 1 else None
executed = [t.get("fullTestPath", "") for t in tests]

lines = [
    "requested=%s" % expected,
    "executed_count=%d" % len(tests),
    "executed=%s" % ",".join(executed),
    "state=%s" % (state if state is not None else "N/A"),
    "succeeded=%d succeededWithWarnings=%d passed_count=%d" % (succeeded, with_warnings, passed_count),
    "failed=%d notRun=%d inProcess=%d" % (failed, not_run, in_process),
]
open(summary_path, "w").write("\n".join(lines) + "\n")
sys.stdout.write("\n".join(lines) + "\n")

with open(durations_path, "w") as f:
    for t in tests:
        dur = t.get("duration")
        f.write("test %s duration_s=%s\n" % (
            t.get("fullTestPath"), ("%.1f" % dur) if isinstance(dur, (int, float)) else "NA"))

bad = 0
def bad_if(cond, msg):
    global bad
    if cond:
        print("VERDICT: %s" % msg, file=sys.stderr)
        bad = 1

bad_if(len(tests) != 1, "expected exactly 1 test in the report, got %d" % len(tests))
bad_if(executed[:1] != [expected], "report ran %r, requested %r" % (executed, expected))
bad_if(state != "Success", "state=%r (expected Success)" % state)
bad_if(passed_count != 1, "passed_count=%d (succeeded=%d + succeededWithWarnings=%d), expected 1"
                          % (passed_count, succeeded, with_warnings))
bad_if(failed != 0, "failed=%d" % failed)
bad_if(not_run != 0, "notRun=%d" % not_run)
bad_if(in_process != 0, "inProcess=%d" % in_process)
sys.exit(bad)
PY
  # Captured, NOT short-circuited: a failing verdict must still leave the exit marker, the crash count
  # and the shadow evidence on disk, or the failure cannot be diagnosed afterwards.
  local verdict=$?

  # The engine prints its own verdict for tools to parse. An ABSENT marker means the editor never
  # reached the queued Quit -- it hung or died -- which must never read as a pass.
  local marker code marker_ok=0
  marker=$(grep -aoE '\*\*\*\* TEST COMPLETE\. EXIT CODE: -?[0-9]+ \*\*\*\*' "$log" | tail -1)
  code=$(sed -nE 's/.*EXIT CODE: (-?[0-9]+).*/\1/p' <<<"${marker:-}")
  [[ "$code" == "0" ]] || { marker_ok=1; echo "engine exit marker not 0 ($name): ${marker:-ABSENT}" >&2; }

  {
    printf 'engine_exit_code=%s\n' "${code:-ABSENT}"
    printf 'process_exit=%d\n' "$rc"
    printf 'crash_fatal_ensure_assert=%d\n' "$crash"
  } >>"$dir/summary.txt"

  # The test's own runtime counters: the shadow-chain recovery evidence.
  grep -a "\[AIRSHADOW\]" "$log" | sed 's/^.*\[AIRSHADOW\]/[AIRSHADOW]/' >"$dir/airshadow.txt" || true

  echo "  rep${rep}/${slug}: exit=$rc engine_exit=${code:-ABSENT} crash=$crash verdict=$verdict"
  [[ $verdict -eq 0 && $crash -eq 0 && $rc -eq 0 && $marker_ok -eq 0 ]] || return 1
  return 0
}

# Fail fast, at every level. Once the gate is red, spending another 20-30 minutes of editor time to
# confirm it only delays the report.
run_repetition() {
  local rep="$1"
  echo "repetition $rep: three tests, one editor process each"
  for entry in "${TESTS[@]}"; do
    local slug=${entry%% *}
    local name=${entry##* }
    run_test "$rep" "$slug" "$name" || return 1
  done
  return 0
}

run_repetition 1 || fail "repetition 1"
run_repetition 2 || fail "repetition 2"

# ---- 3. the two repetitions must agree on the normalized summary ------------------------------------
# Compared: requested/executed names, per-test state, the report counters, process exit, engine exit
# code, crash count. NOT compared: durations, timestamps, absolute paths, PIE instance numbers, UE log
# bytes -- they vary between processes for reasons that say nothing about the control chain.
for entry in "${TESTS[@]}"; do
  slug=${entry%% *}
  if ! diff -q "$OUT_ROOT/repetition_1/$slug/summary.txt" "$OUT_ROOT/repetition_2/$slug/summary.txt" >/dev/null; then
    echo "FAIL: normalized summary differs between repetitions for $slug" >&2
    diff "$OUT_ROOT/repetition_1/$slug/summary.txt" "$OUT_ROOT/repetition_2/$slug/summary.txt" >&2
    fail "normalized summary mismatch"
  fi
done
echo "normalized summaries identical across both repetitions (6 editor processes, 3 tests x 2)"

echo "AIRBORNE_CONTROL_V2_AUTOMATION_RESULT=PASS"
