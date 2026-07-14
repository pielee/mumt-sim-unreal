#!/usr/bin/env bash
# run_command_ownership_telemetry.sh — MEASURE who owns the JSBSim command block.
#
# The read-only audit found three C++ writers in three different execution contexts (Component Tick,
# Actor Tick, 60 Hz timer) with NO TickGroup and NO tick prerequisite anywhere, and a `Commands` block
# that is BlueprintReadWrite. From source alone it was impossible to say what order they run in, whether
# two of them can own one aircraft between two FDM steps, or whether anything unregistered moves the
# block. This runs the real thing and records the real event sequence.
#
# One test per editor process (the shared-process expected-warning problem is documented in
# docs/CONTROL_V2_AIRBORNE_AUTOMATION.md and applies here too).
#
# Scenario B deliberately runs WITHOUT -FormationTest: no setpoints exist, so the autopilot writer
# cannot run and the manual/UDP writer is isolated. Every other scenario needs the scripted profile.
#
# Finally it proves the instrumentation is inert: the SAME airborne scenario is run with telemetry off
# and on, and the aircraft must fly identically (identical [AIRSHADOW] populations). That is a stronger
# claim than "the code only reads" -- it is measured on the actual flight.
#
# RUN THIS EXCLUSIVELY. AUDPControlReceiver binds UDP port 5005, which is a machine-wide resource: if a
# second editor is running (another gate, a PIE session, anything that loads the map) it binds 5005 too,
# and the manual-scenario datagram this runner sends to 127.0.0.1:5005 can be delivered to THAT process
# instead. The manual writer then never fires and scenario B fails for a reason that has nothing to do
# with the code. Do not run it in parallel with the Airborne gate.
set -uo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
PROJECT="$REPO_ROOT/MUMT_Sim.uproject"
UE_ROOT="${UE_ROOT:-$HOME/unreal}"
EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd"
OUT_ROOT="${MUMT_OWNERSHIP_OUT:-/tmp/mumt_command_ownership}"
TIMEOUT_S="${MUMT_OWNERSHIP_TIMEOUT_S:-1800}"

# slug | exact test name | extra editor args
SCENARIOS=(
  "autopilot_only|MUMT.ControlV2.CommandOwnershipAutopilotOnly|-FormationTest"
  "manual_only|MUMT.ControlV2.CommandOwnershipManualOnly|"
  "overlap|MUMT.ControlV2.CommandOwnershipOverlap|-FormationTest"
  "falling|MUMT.ControlV2.CommandOwnershipFalling|-FormationTest"
  "blueprint_audit|MUMT.ControlV2.CommandOwnershipBlueprintAudit|"
)

fail() { echo "FAIL: $*" >&2; echo "COMMAND_OWNERSHIP_RESULT=FAIL"; exit 1; }

[[ -f "$PROJECT" ]] || fail "project not found: $PROJECT"
[[ -x "$EDITOR"  ]] || fail "UnrealEditor-Cmd not found: $EDITOR (set UE_ROOT)"

# Refuse to run alongside another editor rather than produce a mystery failure. Scenario B sends a real
# datagram to 127.0.0.1:5005; a second editor that has also bound 5005 can swallow it, and the run fails
# with "the manual/UDP writer wrote no commands" for a reason that is not in the code at all.
#
# The pattern must match the EXECUTABLE, not any command line that merely mentions it: a plain
# `pgrep -f UnrealEditor-Cmd` also matches the shell running this very script (and any wrapper whose
# argv contains the name), so it would report an editor that does not exist. Anchor on the path
# separator and require a word boundary.
EDITOR_PROC_RE='[/]UnrealEditor-Cmd([[:space:]]|$)'
if pgrep -f "$EDITOR_PROC_RE" >/dev/null 2>&1; then
  fail "another UnrealEditor-Cmd is already running -- it binds UDP 5005 and will steal the manual-scenario datagram. Run this gate exclusively."
fi

# ---- registration: a filter that matches nothing runs zero tests and exits clean --------------------
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
echo "registration: ${#SCENARIOS[@]}/${#SCENARIOS[@]} command-ownership tests found in the live engine"

# ---- run one Automation test in its own editor process ---------------------------------------------
# $1 slug  $2 exact test name  $3 extra args (may be empty)  $4 output dir
run_one() {
  local slug="$1" name="$2" extra="$3" dir="$4"
  rm -rf "$dir"; mkdir -p "$dir/report"
  local log="$dir/automation.log"
  local -a cmd=("$EDITOR" "$PROJECT"
    -ExecCmds="Automation RunTests ${name}; Quit"
    -ReportExportPath="$dir/report")
  # shellcheck disable=SC2206
  [[ -n "$extra" ]] && cmd+=($extra)
  cmd+=(-unattended -nopause -nosplash -nullrhi -abslog="$log")

  {
    printf 'slug=%s\ntest=%s\nextra_args=%s\n' "$slug" "$name" "${extra:-<none>}"
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

  # The command-ownership evidence, straight out of the run.
  grep -a "\[CMDOWN\]" "$log" | sed 's/^.*\[CMDOWN\]/[CMDOWN]/' >"$dir/cmdown.txt" || true
  grep -a "\[AIRSHADOW\]" "$log" | sed 's/^.*\[AIRSHADOW\]/[AIRSHADOW]/' >"$dir/airshadow.txt" || true

  python3 - "$dir/report/index.json" "$dir/summary.txt" "$dir/durations.txt" "$name" <<'PY'
import json, sys
report, summary_path, durations_path, expected = sys.argv[1:5]
d = json.load(open(report, encoding="utf-8-sig"))   # the engine writes a UTF-8 BOM
tests = d.get("tests", [])
succ = (d.get("succeeded") or 0) + (d.get("succeededWithWarnings") or 0)  # a warning is still a pass
state = tests[0].get("state") if len(tests) == 1 else None
lines = [
    "requested=%s" % expected,
    "executed_count=%d" % len(tests),
    "executed=%s" % ",".join(t.get("fullTestPath", "") for t in tests),
    "state=%s" % (state or "N/A"),
    "passed_count=%d failed=%s notRun=%s inProcess=%s" % (
        succ, d.get("failed"), d.get("notRun"), d.get("inProcess")),
]
open(summary_path, "w").write("\n".join(lines) + "\n")
sys.stdout.write("\n".join(lines) + "\n")
with open(durations_path, "w") as f:
    for t in tests:
        dur = t.get("duration")
        f.write("test %s duration_s=%s\n" % (t.get("fullTestPath"),
                ("%.1f" % dur) if isinstance(dur, (int, float)) else "NA"))
bad = 0
if len(tests) != 1 or (tests and tests[0].get("fullTestPath") != expected): bad = 1; print("wrong//missing test", file=sys.stderr)
if state != "Success": bad = 1; print("state=%r" % state, file=sys.stderr)
if succ != 1: bad = 1; print("passed_count=%d" % succ, file=sys.stderr)
if d.get("failed") or d.get("notRun") or d.get("inProcess"): bad = 1; print("nonzero failed/notRun/inProcess", file=sys.stderr)
sys.exit(bad)
PY
  local verdict=$?

  printf 'engine_exit_code=%s\nprocess_exit=%d\ncrash=%d\n' "${code:-ABSENT}" "$rc" "$crash" >>"$dir/summary.txt"
  echo "  ${slug}: exit=$rc engine_exit=${code:-ABSENT} crash=$crash verdict=$verdict"
  [[ $verdict -eq 0 && $crash -eq 0 && $rc -eq 0 && "$code" == "0" ]] || return 1
  return 0
}

# ---- 1. the four scenarios + the Blueprint graph audit ----------------------------------------------
for s in "${SCENARIOS[@]}"; do
  slug=$(cut -d'|' -f1 <<<"$s"); name=$(cut -d'|' -f2 <<<"$s"); extra=$(cut -d'|' -f3 <<<"$s")
  run_one "$slug" "$name" "$extra" "$OUT_ROOT/$slug" || fail "$slug"
done

# ---- 2. telemetry OFF vs ON must fly identically ----------------------------------------------------
# Same airborne scenario, same switches, the ONLY difference is -CommandOwnershipTelemetry. The
# [AIRSHADOW] populations are computed from the actual flight, so if the instrumentation perturbed a
# single command these would diverge.
AIR="MUMT.ControlV2.AirborneFullControlShadow"
run_one "equiv_off" "$AIR" "-FormationTest" "$OUT_ROOT/equiv_off" || fail "equivalence run (telemetry off)"
run_one "equiv_on"  "$AIR" "-FormationTest -CommandOwnershipTelemetry" "$OUT_ROOT/equiv_on" || fail "equivalence run (telemetry on)"

# Compare the flight populations, not the log bytes (timestamps and the telemetry's own lines differ).
norm() { grep -oE "samples=[0-9]+|airborneSamples\(WOW=false\)=[0-9]+|guidance valid=[0-9]+|stick valid=[0-9]+ leaderStickValid=[0-9]+|planner valid=[0-9]+" "$1" | sort; }
norm "$OUT_ROOT/equiv_off/airshadow.txt" >"$OUT_ROOT/equiv_off.norm"
norm "$OUT_ROOT/equiv_on/airshadow.txt"  >"$OUT_ROOT/equiv_on.norm"
if ! diff -q "$OUT_ROOT/equiv_off.norm" "$OUT_ROOT/equiv_on.norm" >/dev/null; then
  echo "FAIL: the flight differs with telemetry on -- the instrumentation is NOT inert" >&2
  diff "$OUT_ROOT/equiv_off.norm" "$OUT_ROOT/equiv_on.norm" >&2
  fail "telemetry off/on equivalence"
fi
[[ -s "$OUT_ROOT/equiv_off.norm" ]] || fail "equivalence comparison found no [AIRSHADOW] populations to compare"
echo "telemetry off/on: identical flight populations (the instrumentation does not touch the command stream)"

# ---- 3. surface the measurements --------------------------------------------------------------------
echo
echo "================ COMMAND OWNERSHIP MEASUREMENTS ================"
for s in "${SCENARIOS[@]}"; do
  slug=$(cut -d'|' -f1 <<<"$s")
  echo "--- $slug ---"
  grep -E "SCENARIO=|TOTALS|WRITERS|OBSERVATIONS|AIRCRAFT|BP_AUDIT" "$OUT_ROOT/$slug/cmdown.txt" 2>/dev/null || echo "  (no [CMDOWN] lines)"
done

echo "COMMAND_OWNERSHIP_RESULT=PASS"
