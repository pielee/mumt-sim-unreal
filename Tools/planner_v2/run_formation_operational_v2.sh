#!/usr/bin/env bash
# run_formation_operational_v2.sh — the Phase D gate.
#
# It drives Formation through the REAL operational command path: a UDP datagram on port 5010 is parsed by
# the production AUDPControlReceiver, routed to the per-aircraft UFormationRuntimeOwnerV2, which drives the
# real NPFG/TECS/stick producer through the Prime/handoff contract into the command arbiter. The only
# trigger is an external "control_mode: formation" command -- nothing auto-activates.
#
# What it proves: an old packet stays legacy; an operational enable gives an exact-zero first consume with
# a REAL candidate; the producer keeps moving the controls; repeated enables are idempotent; a slot update
# does not re-prime; a leader change forces a fresh handoff; disable is an immediate Legacy fallback;
# stale/replayed/invalid commands are refused; Falling preempts; per-aircraft isolation holds; and world
# teardown leaves no runtime state.
#
# One test per editor process, run EXCLUSIVELY (the other gates bind UDP 5005/5010 and a second editor can
# steal a datagram). The editor-wait matches the command line with a bracket class -- Linux truncates the
# process name to 15 chars so `pgrep -x UnrealEditor-Cmd` never matches, and a plain `-f` pattern would
# match this shell.
set -uo pipefail

REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
PROJECT="$REPO_ROOT/MUMT_Sim.uproject"
UE_ROOT="${UE_ROOT:-$HOME/unreal}"
EDITOR="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd"
OUT_ROOT="${MUMT_FOP_OUT:-/tmp/mumt_formation_operational}"
TIMEOUT_S="${MUMT_FOP_TIMEOUT_S:-1500}"

SCENARIOS=(
  "no_command|MUMT.ControlV2.NoCommandRemainsLegacy"
  "enable|MUMT.ControlV2.OperationalEnableExactFirstConsume"
  "active|MUMT.ControlV2.OperationalActiveProducerUpdates"
  "idempotent|MUMT.ControlV2.RepeatedEnableIsIdempotent"
  "slot|MUMT.ControlV2.SlotUpdateWhileActive"
  "leader|MUMT.ControlV2.LeaderChangeRequiresNewHandoff"
  "disable|MUMT.ControlV2.OperationalDisableImmediateFallback"
  "invalid|MUMT.ControlV2.InvalidOrStaleOperationalCommandRejected"
  "falling|MUMT.ControlV2.FallingPreemptsOperationalFormation"
  "isolation|MUMT.ControlV2.PerAircraftOperationalIsolation"
  "world|MUMT.ControlV2.WorldCleanupOperational"
)

fail() { echo "COMMAND_FORMATION_OPERATIONAL_V2_RESULT=FAIL"; echo "FAIL: $*" >&2; exit 1; }

echo "registration: ${#SCENARIOS[@]} operational scenarios expected"
pass=0; total=0

for entry in "${SCENARIOS[@]}"; do
  slug="${entry%%|*}"; test_path="${entry##*|}"
  total=$((total+1))
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

  read -r state count path <<<"$(python3 -c "
import json
try:
    d=json.load(open('$D/report/index.json',encoding='utf-8-sig')); ts=d['tests']
    print(ts[0]['state'], len(ts), ts[0]['fullTestPath'])
except Exception: print('NO_REPORT 0 -')")"

  path_ok=0; [ "$path" = "$test_path" ] && path_ok=1
  ok=1
  [ "$state" = "Success" ] && [ "$rc" = "0" ] && [ "${engine_exit:-1}" = "0" ] && [ "$crash" = "0" ] \
    && [ "$count" = "1" ] && [ "$path_ok" = "1" ] || ok=0

  printf '  %-42s exit=%s engine_exit=%s crash=%s executed=%s path_exact=%s state=%s\n' \
    "$slug" "$rc" "${engine_exit:-ABSENT}" "$crash" "$count" "$path_ok" "$state"
  grep -aoE '\[FOP\] [A-Z_]+RESULT .*' "$D/automation.log" 2>/dev/null | head -1 | sed 's/^/      /'

  if [ "$ok" = "1" ]; then
    pass=$((pass+1))
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

echo "operational: $pass/$total scenarios passed"
[ "$pass" = "$total" ] || fail "not all scenarios passed"
echo "COMMAND_FORMATION_OPERATIONAL_V2_RESULT=PASS"
