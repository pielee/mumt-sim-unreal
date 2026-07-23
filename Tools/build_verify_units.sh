#!/bin/bash
# 3계층 제어기 단위 테스트 [§11] 빌드+실행. 헤더가 std 전용이라 시스템 g++로 충분
# (JSBSim/UE 툴체인 불필요 — 폐루프 검증은 build_verify_formation.sh).
set -e
SRC=$(dirname "$0")/../Source/MUMT_Sim/Public
g++ -std=c++17 -O2 -Wall -I"$SRC" "$(dirname "$0")/verify_controller_units.cpp" -o /tmp/verify_units
/tmp/verify_units
