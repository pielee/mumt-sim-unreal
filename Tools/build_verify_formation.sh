#!/bin/bash
# FFormationGuidance + FInnerLoopAutopilot 폐루프 검증 하네스 빌드+실행.
# UE 번들 clang/libc++ 필수 (libJSBSim.a가 libc++ ABI — 시스템 g++ 불가).
set -e
TC=$HOME/unreal/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v22_clang-16.0.6-centos7/x86_64-unknown-linux-gnu
LC=$HOME/unreal/Engine/Source/ThirdParty/Unix/LibCxx
P=$(dirname "$0")/../Plugins/JSBSimFlightDynamicsModel
SRC=$(dirname "$0")/../Source/MUMT_Sim/Public
$TC/bin/clang++ -std=c++17 -O2 -nostdinc++ -I$LC/include -I$LC/include/c++/v1 \
  -I$P/Source/ThirdParty/JSBSim/Include -I$SRC \
  "$(dirname "$0")/verify_formation.cpp" \
  $P/Source/ThirdParty/JSBSim/Lib/Linux/libJSBSim.a \
  $LC/lib/Unix/x86_64-unknown-linux-gnu/libc++.a \
  $LC/lib/Unix/x86_64-unknown-linux-gnu/libc++abi.a \
  -fuse-ld=lld -lm -lpthread -ldl -o /tmp/verify_formation
/tmp/verify_formation "$P/Resources/JSBSim"
