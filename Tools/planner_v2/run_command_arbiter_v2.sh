#!/usr/bin/env bash
# run_command_arbiter_v2.sh — the Phase A gate for the consume-boundary command arbiter.
#
# The arbiter now stands between EVERY writer (including the aircraft Blueprints) and the FCS. The whole
# risk is a silent behaviour change: drop or alter one field and the aircraft flies differently, and
# nothing else in the project would notice. So the central claim this gate has to establish is not "the
# arbiter works" but "by default it changed NOTHING" -- resolved == legacy in all 15 flight-control
# fields and all 14 per-engine fields, compared one by one rather than hashed.
#
# One test per editor process (the first-PIE expected-warning problem is documented in
# docs/CONTROL_V2_AIRBORNE_AUTOMATION.md and applies here too).
#
# Scenario A and C send a real datagram to 127.0.0.1:5005, so this gate must run EXCLUSIVELY: a second
# editor binds the same port and can swallow it. The guard below refuses rather than fail mysteriously.
set -uo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
PROJECT="$REPO_ROOT/MUMT_Sim.uproject"
UE_ROOT="${UE_ROOT:-$HOME/unreal}"
EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd"
OUT_ROOT="${MUMT_ARBITER_OUT:-/tmp/mumt_command_arbiter_v2}"
TIMEOUT_S="${MUMT_ARBITER_TIMEOUT_S:-1800}"

# slug | exact test name | extra editor args
# B/C/D need the scripted flight profile. A isolates the manual writer, so it must NOT have setpoints.
# E-H only need a live world, so they run short and need no profile.
SCENARIOS=(
  "production_default|MUMT.ControlV2.ArbiterProductionDefault|"
  "lifecycle|MUMT.ControlV2.ArbiterLifecycle|"
  "world_isolation|MUMT.ControlV2.ArbiterWorldIsolationCleanup|"
  "resolver_ownership|MUMT.ControlV2.ArbiterResolverOwnershipLifecycle|"
  "legacy_manual|MUMT.ControlV2.ArbiterLegacyManual|"
  "legacy_autopilot|MUMT.ControlV2.ArbiterLegacyAutopilot|-FormationTest"
  "legacy_overlap|MUMT.ControlV2.ArbiterLegacyOverlap|-FormationTest"
  "falling_over_candidate|MUMT.ControlV2.ArbiterFallingOverCandidate|-FormationTest"
  "candidate_valid|MUMT.ControlV2.ArbiterCandidateValid|"
  "candidate_stale|MUMT.ControlV2.ArbiterCandidateStale|"
  "candidate_invalid|MUMT.ControlV2.ArbiterCandidateInvalid|"
  "isolation|MUMT.ControlV2.ArbiterIsolation|"
)

fail() { echo "FAIL: $*" >&2; echo "COMMAND_ARBITER_V2_RESULT=FAIL"; exit 1; }

[[ -f "$PROJECT" ]] || fail "project not found: $PROJECT"
[[ -x "$EDITOR"  ]] || fail "UnrealEditor-Cmd not found: $EDITOR (set UE_ROOT)"

# Match the EXECUTABLE, not any command line that merely mentions it -- a plain `pgrep -f UnrealEditor-Cmd`
# also matches the shell running this script and would report an editor that does not exist.
EDITOR_PROC_RE='[/]UnrealEditor-Cmd([[:space:]]|$)'
if pgrep -f "$EDITOR_PROC_RE" >/dev/null 2>&1; then
  fail "another UnrealEditor-Cmd is already running -- it binds UDP 5005 and will steal the manual-scenario datagram. Run this gate exclusively."
fi

# ---- registration ----------------------------------------------------------------------------------
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
echo "registration: ${#SCENARIOS[@]}/${#SCENARIOS[@]} arbiter tests found in the live engine"

# ---- one test, one editor process ------------------------------------------------------------------
run_one() {
  local slug="$1" name="$2" extra="$3" dir="$OUT_ROOT/$1"
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

  grep -a "\[ARBITER\]" "$log" | sed 's/^.*\[ARBITER\]/[ARBITER]/' >"$dir/arbiter.txt" || true
  grep -a "\[CMDOWN\]"  "$log" | sed 's/^.*\[CMDOWN\]/[CMDOWN]/'   >"$dir/cmdown.txt"  || true

  python3 - "$dir/report/index.json" "$dir/summary.txt" "$dir/durations.txt" "$name" <<'PY'
import json, sys
report, summary_path, durations_path, expected = sys.argv[1:5]
d = json.load(open(report, encoding="utf-8-sig"))   # the engine writes a UTF-8 BOM
tests = d.get("tests", [])
# a passing test that logs a warning lands in succeededWithWarnings, and this scenario always trips the
# pre-existing F16_UAV Blueprint warning -- so requiring succeeded==1 would reject a healthy run
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
  slug=$(cut -d'|' -f1 <<<"$s"); name=$(cut -d'|' -f2 <<<"$s"); extra=$(cut -d'|' -f3 <<<"$s")
  run_one "$slug" "$name" "$extra" || fail "$slug"
done

# ---- the transparency claim, restated from the runs themselves --------------------------------------
echo
echo "================ ARBITER MEASUREMENTS ================"
for s in "${SCENARIOS[@]}"; do
  slug=$(cut -d'|' -f1 <<<"$s")
  echo "--- $slug ---"
  grep -E "TOTALS|TRANSPARENCY|FALLBACKS|AIRCRAFT|D_RESULT|PROD_DEFAULT|LIFE_RESULT|LIFE_POST_PIE|WORLDISO_RESULT|OWNERSHIP_RESULT" "$OUT_ROOT/$slug/arbiter.txt" 2>/dev/null \
    | grep -v "\[log\]" | head -8 || echo "  (no [ARBITER] lines)"
done

# Legacy transparency is the load-bearing invariant; assert it from the artifacts, not just the tests.
for slug in legacy_manual legacy_autopilot legacy_overlap; do
  line=$(grep -m1 "TRANSPARENCY" "$OUT_ROOT/$slug/arbiter.txt" 2>/dev/null || true)
  [[ -n "$line" ]] || fail "$slug produced no TRANSPARENCY line"
  grep -q "legacy_changed_field_count=0 legacy_block_mutation_count=0" <<<"$line" \
    || fail "$slug is NOT transparent: $line"
done
echo "legacy transparency confirmed in all three legacy scenarios (0 changed fields, 0 legacy mutations)"

# Without this, the whole gate would only prove that a TEST-enabled resolver behaves -- while production
# ran with no resolver at all. It is the load-bearing claim of Phase A.
prod=$(grep -m1 "PROD_DEFAULT" "$OUT_ROOT/production_default/arbiter.txt" 2>/dev/null || true)
[[ -n "$prod" ]] || fail "production_default produced no PROD_DEFAULT line"
grep -q "resolver_enabled=1" <<<"$prod" || fail "the resolver is NOT bound in production: $prod"
grep -q "changed_fields=0 formation=0 resolved_differs=0" <<<"$prod" \
  || fail "the production-default resolver is not transparent: $prod"
echo "production resolver: bound at module startup, transparent, FormationControlV2 inactive"

echo "COMMAND_ARBITER_V2_RESULT=PASS"
