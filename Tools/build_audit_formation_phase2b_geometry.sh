#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I"$ROOT/Source/MUMT_Sim/Public" \
  "$ROOT/Source/MUMT_Sim/Private/FormationControl/FormationCapturePlanner.cpp" \
  "$ROOT/Tools/audit_formation_phase2b_geometry.cpp" \
  -o /tmp/audit_formation_phase2b_geometry
/tmp/audit_formation_phase2b_geometry
