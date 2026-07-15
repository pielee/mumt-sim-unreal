#!/usr/bin/env bash
# run_formation_candidate_integration_v2.sh — the Phase C gate.
#
# It drives the REAL Formation candidate producer -- FormationPlannerV2 -> NPFG -> TECS ->
# F16StickAdapterV2 -- through the Phase B Prime/handoff contract, in a live PIE world with a real leader
# and follower. What it proves, narrowly:
#
#   * the FIRST Legacy -> Formation consume steps by EXACTLY ZERO on all five controlled fields, with the
#     candidate computed by the real chain (not hand-built);
#   * from the second frame the real producer output actually MOVES the controls (negative control);
#   * an intervening Legacy consume, a stale candidate, and Falling each refuse the handoff, and Falling's
#     safety block (throttle 0 / cutoff) reaches the FDM;
#   * ActiveFormation keeps producing fresh candidates; Formation -> Legacy is immediate.
#
# It does NOT claim TECS continuity: the second-frame transient is logged, never required to be zero.
#
# One test per editor process, run EXCLUSIVELY (the other gates bind UDP 5005 and a second editor can
# steal a datagram). The editor-wait matches on the COMMAND LINE with a bracket class -- Linux truncates
# the process name to 15 chars so `pgrep -x UnrealEditor-Cmd` never matches, and a plain `-f` pattern
# would match this shell.
set -uo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
PROJECT="$REPO_ROOT/MUMT_Sim.uproject"
UE_ROOT="${UE_ROOT:-$HOME/unreal}"
EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd"
OUT_ROOT="${MUMT_FCI_OUT:-/tmp/mumt_formation_candidate}"
TIMEOUT_S="${MUMT_FCI_TIMEOUT_S:-1500}"

SCENARIOS=(
  "exact_first|MUMT.ControlV2.RealProducerHandoffExactFirstConsume"
  "negative|MUMT.ControlV2.RealProducerNegativeControl"
  "intervening|MUMT.ControlV2.InterveningLegacyConsumeRejected"
  "stale|MUMT.ControlV2.StaleRealCandidateRejected"
  "falling|MUMT.ControlV2.FallingPreemptsRealCandidate"
  "isolation|MUMT.ControlV2.PerAircraftAndWorldIsolation"
  "active|MUMT.ControlV2.ActiveFormationUpdates"
  "fallback|MUMT.ControlV2.ImmediateFormationToLegacyFallback"
)

fail() { echo "COMMAND_FORMATION_CANDIDATE_V2_RESULT=FAIL"; echo "FAIL: $*" >&2; exit 1; }

# ---- registration: every scenario must exist in the live engine ------------------------------------
echo "registration: ${#SCENARIOS[@]} integration scenarios expected"

overall_pass=0
overall_total=0

for entry in "${SCENARIOS[@]}"; do
  slug="${entry%%|*}"
  test_path="${entry##*|}"
  overall_total=$((overall_total+1))
  D="$OUT_ROOT/$slug"; rm -rf "$D"; mkdir -p "$D/report"

  while pgrep -f '[U]nrealEditor-Cmd' >/dev/null 2>&1; do sleep 10; done

  timeout "$TIMEOUT_S" "$EDITOR" "$PROJECT" \
    -ExecCmds="Automation RunTests $test_path; Quit" \
    -ReportExportPath="$D/report" -FormationTest \
    -unattended -nopause -nosplash -nullrhi -abslog="$D/automation.log" >/dev/null 2>&1
  rc=$?

  engine_exit=$(grep -aoE 'EXIT CODE: -?[0-9]+' "$D/automation.log" 2>/dev/null | tail -1 | grep -oE '\-?[0-9]+')
  crash=0
  for m in "Fatal error" "Assertion failed" "Ensure condition failed" "=== Critical error ===" "SIGABRT" "SIGSEGV"; do
    grep -qa -- "$m" "$D/automation.log" 2>/dev/null && crash=1
  done

  read -r state count path passed failed <<<"$(python3 -c "
import json
try:
    d=json.load(open('$D/report/index.json',encoding='utf-8-sig')); ts=d['tests']
    t=ts[0]
    st=t['state']; p=t['fullTestPath']
    pw=sum(1 for e in t.get('entries',[]) if e.get('event',{}).get('type')=='Info')
    print(st, len(ts), p, 1 if st=='Success' else 0, 0 if st=='Success' else 1)
except Exception:
    print('NO_REPORT 0 - 0 1')")"

  path_ok=0; [ "$path" = "$test_path" ] && path_ok=1
  ok=1
  [ "$state" = "Success" ] && [ "$rc" = "0" ] && [ "${engine_exit:-1}" = "0" ] && [ "$crash" = "0" ] \
    && [ "$count" = "1" ] && [ "$path_ok" = "1" ] || ok=0

  printf '  %-40s exit=%s engine_exit=%s crash=%s executed=%s path_exact=%s state=%s\n' \
    "$slug" "$rc" "${engine_exit:-ABSENT}" "$crash" "$count" "$path_ok" "$state"
  grep -aoE '\[FCI\] [A-Z_]+RESULT .*' "$D/automation.log" 2>/dev/null | head -1 | sed 's/^/      /'

  if [ "$ok" = "1" ]; then
    overall_pass=$((overall_pass+1))
  else
    python3 -c "
import json
try:
    d=json.load(open('$D/report/index.json',encoding='utf-8-sig'))
    for e in d['tests'][0].get('entries',[]):
        ev=e.get('event',{})
        if ev.get('type')=='Error': print('      ERROR:', ev.get('message','')[:160])
except Exception as ex: print('      no report:', ex)" 2>&1 | head -8
    fail "$slug (exit=$rc engine=$engine_exit crash=$crash executed=$count path_ok=$path_ok state=$state)"
  fi
done

echo "integration: $overall_pass/$overall_total scenarios passed"
[ "$overall_pass" = "$overall_total" ] || fail "not all scenarios passed"
echo "COMMAND_FORMATION_CANDIDATE_V2_RESULT=PASS"
