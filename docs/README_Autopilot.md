# UAV Flight Control — 3계층 스택 (FormationGuidance → FixedWingGuidance → F16CommandController)

> Scope: 무인기(F16_UAV*)를 조종하는 C++ 제어 스택. BT의 고수준 명령(모드·슬롯·한계값)을
> JSBSim FCS 명령(fcs/*-cmd-norm)으로 변환한다.
> 2026-07-11 전면 교체: 구 BVRGym `FAircraftAutopilot` → GetStick(`Controller_CY`) →
> `InnerLoopAutopilot`+일체형 `FFormationGuidance` 계보를 제거하고 아래 3계층으로 재편.
> 검증: `Tools/build_verify_formation.sh`(JSBSim 스탠드얼론 폐루프) + `Tools/run_formation_test.sh`
> (헤드리스 인엔진 회귀) — 구 스택 기준선 수치를 그대로 재현(전 게이트 PASS).
>
> Related: [README_UDP_Comms.md](README_UDP_Comms.md), [README_JSBSim_IO.md](README_JSBSim_IO.md),
> [README_Joystick_Manned.md](README_Joystick_Manned.md)

---

## 0. 전체 데이터 흐름

```
교수님 BT XML (py_bt_ros)
  ↓ bt_nodes.py Custom Node (SetLeader / SetFormationSlot / EnableFormationFollow / Check*)
ROS /aircraft/setpoint (custom_msgs/AircraftSetpoint — 고수준 명령: 모드·슬롯·톨러런스·한계)
  ↓ mumt_ros_bridge bridge_node.py (JSON 직렬화 + seq/timestamp 자동 스탬프 + NaN 검증)
UDP 5010 → AUDPControlReceiver::ReceiveSetpointData (NaN/낡은 seq 폐기, 이름 라우팅)
  ↓ 60 Hz AutopilotTick → ApplyAutopilotToPawn (모드 분기)
[1] FFormationGuidance   (FormationGuidance.h)   — 리더 직독 → 슬롯 위치/속도/가속도,
  ↓ FFormationTarget       along/cross/vertical/상대속도 오차, 캡처·유지 판정, 안전거리
[2] FFixedWingGuidance   (FixedWingGuidance.h)   — 벡터필드 횡 + REJOIN, 고도→FPA/pitch,
  ↓ FFlightReference       속도법칙(+폐속도·안전거리 제한), course→roll(슬루·뱅크 제한)
[3] FF16CommandController (F16CommandController.h) — roll/pitch/airspeed reference → PID →
  ↓ FF16ControlCommand     aileron/elevator/rudder/throttle/speedbrake cmd-norm
UJSBSimMovementComponent Commands/EngineCommands  (Commands.Aileron = AileronCmdNorm, ...)
  ↓ CopyToJSBSim: FCS->SetDaCmd/SetDeCmd/SetDrCmd(-)/SetThrottleCmd/SetDsbCmd
f16.xml <flight_control> FCS (조종면 위치·레이트 루프·G/AOA 리미터·요-SAS는 여기)
  ↓
JSBSim FDM (120 Hz) → AircraftState → Unreal Actor 갱신 + UDP 5006 상태 배치
  └─ 상태 피드백: 5006 배치의 per-aircraft "guidance" 오브젝트(e_along/e_cross/e_vert/
     captured/maintained/separation/rejoin/refs) → BT CheckFormationCaptured/Maintained
```

- 무인기의 **JSBSim 명령 기록 지점은 `ApplyAutopilotToPawn` 한 곳**뿐이다 (유인기 조이스틱은
  별도 경로 `ApplyControlCommandToPawn` — 5005 raw command, 보존).
- Direct 모드(heading/alt/speed 그대로)와 Attack 모드(FPursuitGuidance → StepDirect)도
  같은 [2]→[3] 체인을 지난다 — 구 시나리오 완전 호환.
- 세 헤더 모두 의존성 없음(std 수학만): JSBSim 스탠드얼론 하네스(`Tools/verify_formation.cpp`)에서
  UE 없이 단독 폐루프 검증 가능.

## 1. 계층별 입·출력

| 계층 | 입력 | 출력 |
|---|---|---|
| [1] FFormationGuidance | 리더 위치/속도(참벡터)/상승률, 자기 위치/속도/고도, 슬롯 오프셋(front/right/up), dt | FFormationTarget: 슬롯 pos/vel(ω×r)/acc(ω×(ω×r)), track·ω, e_along/e_cross/e_vert, 상대속도 오차, 폐속도, 3D 슬롯거리, 리더 3D 거리, captured/maintained/sep_breach |
| [2] FFixedWingGuidance | FFormationTarget(또는 FDirectCmd), 자기 상태(φ/θ/ψ/alt/TAS/climb/pos/vel), min/max 속도, min 고도, max 폐속도, dt | FFlightReference: course/roll(+ff)/alt/FPA/pitch/airspeed reference (+throttle ff 자리) |
| [3] FF16CommandController | FFlightReference의 roll/pitch/airspeed ref + φ/θ/p/q/r/TAS, 개루프 스로틀 폴백, dt | FF16ControlCommand: aileron/elevator/rudder/throttle/speedbrake **cmd-norm** |

제한값 적용 위치([2]): 뱅크 ±60°(편대/추격은 62°), roll ref 슬루 40°/s, θ +25/−20°,
상승 80·강하 60 m/s, 속도 [min,max], REJOIN 폐속도 테이퍼(거리/8s), maximum_closing_speed,
minimum_separation 침범 시 부족량 비례 감속(2 m/s per m).

## 2. 제어 법칙 계보 (검증 이력 보존 — 수치 변경 금지)

전부 JSBSim F-16 폐루프(V1 하네스)와 PIE로 검증된 값을 구 스택에서 그대로 이관:
- 횡: 벡터필드 χc = χ_slot + 70°·(2/π)·atan(0.004·(e_cross + 2.5·ė_cross)), REJOIN(>800m 진입/<400m 복귀)
- 종 속도: V = |v_slot| + clamp(0.25·e_along + 0.80·ė_along, ±150)
- roll_ff = atan(ωV/g)·1.2(요-SAS 보정)·cross게이팅(바닥 0.25), ±54°
- ω 추정: track 미분 + LPF τ=0.25s + 데드밴드 0.3°/s, 리더 <50m/s는 홀드
- 수직: slot_alt + 리더상승률×8s 선행보상; course→roll Kp=2.5
- [3] PID: Roll(0.04,0,Kd 0.012) / Pitch(0.06,0.008,Kd 0.03)+뱅크보상 / Speed PI(0.08,0.020)
- 러더: 0 (bank-to-turn + f16.xml 요-SAS 내장 — 외부 중복 댐핑 금지)

### 2026-07-11 견고성 수정 (게인 불변 — 구조·안전 수정만)

- **Course/Yaw 분리**: course 피드백은 이제 ground course(atan2(Ve,Vn)) 기준.
  yaw(기수 방위)는 지상속도 < 30 m/s(설정 `CourseMinGroundSpeedMps`)일 때 폴백으로만 사용.
  목표−현재 모두 ground course — "Target Course − Yaw" 구조 제거.
- **p/q/r의 실제 의미**: 플러그인 `AircraftState.EulerRates` = JSBSim
  `FGAuxiliary::GetEulerRates` = **오일러각 변화율(φ̇,θ̇,ψ̇), rad/s** — body rate p,q,r가
  아니며, 수신부가 deg/s로 변환해 전달한다. (수정 전에는 rad/s를 deg/s로 오독 →
  레이트 항이 사실상 0이었으나 전 게이트 PASS — D항이 지배 감쇠였음.)
- **감쇠 방식 분리(§5)**: `EDampingMode::DerivOnMeas`(A, 기본 — PID D항) vs
  `BodyRate`(B — Kd=0 + 명시적 레이트 항). 동시 활성화 금지. A/B 하네스 비교(전 게이트
  PASS 동일, 오버슛 A 8.3°/B 6.3°, 포화 0%): 기본 = **A** — UE 경로가 body rate를
  제공하지 않고, UE 인엔진 실검증 이력과 동일 동작이므로.
- **Throttle = Trim FF + PI 보정**: `ThrottleTrimNorm`(기본 0.26 — V1 실측: 220 m/s
  0.289 / 150 m/s 0.225) + Speed PI 보정(±`MaxThrottleCorr` 1.0, 실효 [0,1] 경계로
  anti-windup 정합). `FFlightReference.ThrottleRefNorm >= 0`이면 그 값이 FF (TECS류 확장
  자리). 오차 0에서 스로틀 ≈ 트림 — 구 구조의 "속도 떨어진 뒤 적분기로 회복" 제거.
- **dt/NaN 유효성(§2)**: 세 계층 모두 dt∉유효범위 또는 비유한 입력 시 상태 미갱신;
  Guidance는 마지막 정상 target/reference를, 컨트롤러는 마지막 정상 명령을 유지한다.
  컨트롤러는 `InvalidInputTimeoutS`(1s) 지속 시 Failsafe(중립 조종면+트림
  스로틀 — 상위 수신부의 소실 홀드가 1차 정책, 이것은 최후 단계). 유도 계층은 dt를
  NominalDtS(1/60)로 위생 처리.
- **Speedbrake 설정화**: `SpeedbrakeStartOverspeedMps`(5)–`Full`(20) 선형 —
  기존 코드 동작 유지(주석의 "8 m/s"가 오기였음), 역전 설정은 계단형으로 안전 강등.
- **최소 안전거리 = '감속 보조'(Minimum-Separation Speed-Reduction Assist)**:
  airspeed reference만 낮추는 보조 기능 — **완전한 충돌 회피가 아니다** (측면·정면 회피
  Roll/Path 법칙 없음 → 별도 Collision Avoidance 모듈 필요). 속도가 하한인데 접근이
  지속되면 `separation_warning`(5006 guidance 오브젝트)으로 BT에 경고.
- **상태 초기화(§9)**: 모드 전환 시 Controller.Reset()+Guidance.Reset()(roll_ref 슬루가
  현재 φ에서 범프리스 재시작, 스로틀은 트림에서 시작), 리더 변경 시 Formation.Reset().
  같은 모드·리더 유지 중에는 리셋하지 않음.
- 단위 테스트: `Tools/build_verify_units.sh` (JSBSim/UE 불필요, g++ 단독).

## 3. 프로토콜 (AircraftSetpoint v2)

신규 필드: `capture_tolerance_m`(기본 30) / `maintain_tolerance_m`(기본 50) /
`minimum_separation_m`(0=off) / `maximum_closing_speed_mps`(0=off) /
`sequence_id`(bridge 자동 증가, UE는 낡은 seq 폐기) / `timestamp` / `protocol_version`(=2).
삭제: `use_waypoint`, `target_x/y/z` (GetStick 유물).
UE 수신 검증: NaN/Inf 필드 → 패킷 폐기, 빈 aircraft_name → 무시, seq 역행 → 폐기.

## 4. 삭제된 구형 제어기 (2026-07-11)

| 파일 | 정체 |
|---|---|
| `BVRGymAutopilot.h/.cpp` | BVRGym 캐스케이드 PID(FAircraftAutopilot→FPID 잔존) — 전 계보 제거 |
| `Controller_CY.h/.cpp` | Geometry StickController(GetStick 조준 제어기) — 휴면 상태였음 |
| `Public/BT_Geometry/` | Controller_CY 전용 수학 라이브러리 (원본은 ~/dev/Geometry에 보존) |
| `InnerLoopAutopilot.h` | 07-09 내루프 — 법칙은 [2]/[3]으로 분할 이관 |
| `Scripts/control_sender.py` | 레거시 Python 무인기 제어 루프 (5005 raw) |
| `Config/JSBSim/control.json` | GetStick 시대 정적 명령 기본값 (참조 0) |
