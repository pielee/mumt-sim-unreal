#!/bin/bash
# 인엔진 편대 회귀 시험 (헤드리스, ROS/조이스틱 불필요).
# 리더를 스크립트로 이륙→직선→3°/s→4°/s+400m상승→롤아웃 시키고, 팔로워의 슬롯오차를
# 실제 경로(플러그인 상태 → pawn 매칭 → 유도 → 내루프 → JSBSim)로 측정해 게이트 판정.
# 결과: Saved/FormationTest.csv + 로그의 [FormTest] 라인.
set -e
PROJ="$(cd "$(dirname "$0")/.." && pwd)/MUMT_Sim.uproject"
LOG=${1:-/tmp/formation_test.log}
rm -f "$(dirname "$PROJ")/Saved/FormationTest.csv"
timeout 600 "$HOME/unreal/Engine/Binaries/Linux/UnrealEditor" "$PROJ" /Game/RL_2 \
  -game -nullrhi -unattended -nosound -nosplash -stdout \
  -FormationTest -FormationTestExit > "$LOG" 2>&1 || true
grep "\[FormTest\]" "$LOG" | sed 's/.*\[FormTest\]/[FormTest]/'
grep -q "전체 PASS" "$LOG"
