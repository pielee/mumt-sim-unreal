#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I"$ROOT/Source/MUMT_Sim/Public" \
  "$ROOT/Source/MUMT_Sim/Private/FormationControl/FormationSlotGenerator.cpp" \
  "$ROOT/Source/MUMT_Sim/Private/FormationControl/FormationCapturePlanner.cpp" \
  "$ROOT/Tools/verify_formation_phase2b.cpp" \
  -o /tmp/verify_formation_phase2b
/tmp/verify_formation_phase2b
