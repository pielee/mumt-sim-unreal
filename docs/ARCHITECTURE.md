# MUMT 스택 아키텍처 (3-repo 통합 개요)

> **개정: 2026-07-22.** 2026-06-28 스냅샷(+ 07-11 구식 경고 헤더)을 전면 개정했다.
> 구 제어 스택(BVRGymAutopilot/FAircraftAutopilot, GetStick, InnerLoopAutopilot) 서술과
> "무장·팀·체력 없음" 서술은 삭제된 코드 기준이므로 제거했고, 현행 3계층 제어 스택·무장/교전
> 시스템·dogfight BT를 반영했다. 구 스냅샷(측정 데이터·갭 분석 포함)은 git 이력에서 볼 수 있다.
>
> 이 문서는 **3개 repo 전체의 현행 구조 개요이자 `docs/` 문서 세트의 허브**다. 각 장은 상세
> README로 링크된다. 처음 읽는 사람은 이 문서를 순서대로 읽지 말고 **[CURRICULUM.md](CURRICULUM.md)**
> 의 단계 순서를 따르는 것을 권장한다.

---

## 0. 세 개의 repo, 한 개의 시스템

| Repo | 역할 | 언어/런타임 |
|---|---|---|
| `~/dev/MUMT_Sim` | UE5.4 시뮬레이터 — JSBSim 물리, 항공기 액터, 저수준 제어 스택, 무장/교전, UDP 서버 | C++ / Blueprint / UE5 |
| `~/dev/mumt_ros_ws` | ROS2 브리지 — ROS 토픽 ↔ UDP 변환, 조이스틱 매핑 | Python (rclpy) |
| `~/dev/py_bt_ros` | BT 두뇌 — 무인기 임무 자율성(이륙/편대/교전)을 행동 트리로 실행 | Python (자체 BT 프레임워크) |

세 repo는 **ROS 토픽이 아니라 raw UDP 3개 포트로 연결**된다 (브리지가 중간 변환).
UE는 인바운드 두 경로 모두 **aircraft_name 기반 이름 라우팅**으로 폰을 찾는다.

### 전체 데이터 흐름

```
        py_bt_ros (BT 두뇌)                          조이스틱 (Thrustmaster T.16000M)
  main.py → BTRunner (~30Hz 틱)                     joy_node → joystick_node.py
  scenarios/*/bt_nodes.py                                │ /mumt/aircraft_commands
        │ /aircraft/setpoint                             │ (std_msgs/String JSON, 50Hz)
        │ (custom_msgs/AircraftSetpoint v2)              │
        ▼                                                ▼
  ┌────────────────── mumt_ros_ws: bridge_node.py (노드 "mumt_bridge") ──────────────────┐
  │  setpoint → JSON 직렬화(+sequence/timestamp 자동 스탬프, NaN/Inf 검증) → UDP 5010     │
  │  command  → JSON 패스스루                                            → UDP 5005     │
  │  UDP 5006 수신(60Hz 드레인 타이머) → /mumt/aircraft_states (String JSON) 발행         │
  └──────────────────────────────────────────────────────────────────────────────────────┘
        │ 5010 셋포인트(고수준)         │ 5005 raw 명령(조종면)         ▲ 5006 상태 배치
        ▼                              ▼                              │
  ╔══════════════════ MUMT_Sim (UE5): AUDPControlReceiver ════════════╧═════════════════╗
  ║  이름 라우팅: 각 패킷의 aircraft_name → 이름이 일치/포함되는 폰에 적용                  ║
  ║                                                                                     ║
  ║  5010 → 60Hz AutopilotTick → ApplyAutopilotToPawn (무인기 제어의 유일한 주입점)       ║
  ║     → [1] FFormationGuidance → [2] FFixedWingGuidance → [3] FF16CommandController    ║
  ║  5005 → ApplyControlCommandToPawn (유인기 raw 조종 — 조이스틱/키보드 대체 경로)        ║
  ║        ▼                                                                            ║
  ║  UJSBSimMovementComponent.Commands/EngineCommands                                   ║
  ║        ▼ CopyToJSBSim → JSBSim FDM 실행(120Hz 고정 서브스텝) → CopyFromJSBSim         ║
  ║  ECEF 좌표 → (Cesium/GeoReferencing) → UE 월드 좌표 → SetActorLocationAndRotation    ║
  ║        ▼                                                                            ║
  ║  상태 송신 타이머 → {"message_type":"aircraft_state_batch", aircraft:[...position,    ║
  ║   attitude, speed, team, weapons:{bullet_ammo,rocket_ammo}, guidance:{e_along,...}]} ║
  ║   → UDP 5006                                                                        ║
  ╚═════════════════════════════════════════════════════════════════════════════════════╝
```

- 포트/스키마 상세: [README_UDP_Comms.md](README_UDP_Comms.md), [STATE_API.md](STATE_API.md)
- 제어 스택 상세: [README_Autopilot.md](README_Autopilot.md)
- 브리지/조이스틱 상세: [README_ROS_Bridge.md](README_ROS_Bridge.md), [README_Joystick_Manned.md](README_Joystick_Manned.md)
- 빌드/실행: [README_Build_and_Run.md](README_Build_and_Run.md)

---

## 1. 항공기: 물리와 화면이 연결되는 원리

이 시스템에서 "비행기"는 서로 독립적인 **두 출처**에서 온 것을 컴포넌트 하나가 연결한 구조다.
UE 자체 물리(Chaos)는 전혀 쓰지 않는다.

### 1.1 물리 — JSBSim 플러그인 (나는 비행기)

- `Plugins/JSBSimFlightDynamicsModel/` 안에 오픈소스 비행역학 라이브러리 **JSBSim이 통째로
  내장**되어 있다 (`Source/ThirdParty/JSBSim/` — FGFDMExec, FGAircraft 등).
- F-16의 비행 특성(날개면적·관성모멘트·공력계수·엔진·FCS 제어법칙)은 C++ 코드가 아니라
  **XML 데이터**다: `Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim/aircraft/f16/f16.xml`
  — JSBSim 커뮤니티의 F-16A Block-32 모델(작성자 Erik Hofman, NASA 공개자료 기반)을 그대로
  가져온 것. 폰의 `AircraftModel = "f16"` 문자열이 이 폴더를 지정한다.
- 조종면 위치·레이트 루프·G/AOA 리미터·요-SAS는 f16.xml의 `<flight_control>`(FCS)에 있다 —
  C++ 제어 스택은 FCS **명령**(cmd-norm)까지만 만든다.
- 플러그인은 순수 인프로세스 FDM이다. 소켓/ROS 등 외부 I/O는 전혀 없다 (전부
  `Source/MUMT_Sim` 모듈의 일).
- 상세: [README_JSBSim.md](README_JSBSim.md), [README_JSBSim_IO.md](README_JSBSim_IO.md)

### 1.2 시각 — 마켓플레이스 아트 재사용 (보이는 비행기)

- 화면의 F-16은 `Content/F16Control/meshes/sk_Jet.uasset` 스켈레탈 메시.
  `F16Control/` 폴더는 구 언리얼 마켓플레이스 비행기 샘플에서 온 것으로, **원래의 아케이드
  비행 로직은 폐기하고 3D 모델·스켈레톤·애니메이션만 재사용**했다. JSBSim과 무관한 "껍데기"다.
- 조종면·랜딩기어 움직임: AnimBP `Content/F16Control/animation/AB_Jet.uasset`이
  `AircraftState`의 舵面 각도(ElevatorPosition, Left/RightAileronPosition, RudderPosition,
  FlapPosition, SpeedBrakePosition)를 읽어 본(bone)을 회전시킨다.
- 대체 기체 예시: `Content/CommercialPlane/` (SK_CommercialPlane 메시 + ABP — Boeing314
  등 다른 JSBSim XML과 짝지을 수 있는 구조 예시).

### 1.3 연결 — UJSBSimMovementComponent 틱 파이프라인

`Plugins/JSBSimFlightDynamicsModel/Source/JSBSimFlightDynamicsModel/Private/JSBSimMovementComponent.cpp`
의 `TickComponent()`가 매 프레임:

1. **입력 주입** `CopyToJSBSim()` — `Commands`(Aileron/Elevator/Rudder ∈[-1,1] 등)와
   `EngineCommands`(엔진별 Throttle)를 JSBSim FCS로 (`SetDaCmd/SetDeCmd/SetDrCmd(-)/SetThrottleCmd`,
   러더는 부호 반전 주의).
2. **물리 실행** `Exec->Run()` — 자체 고정스텝 어큐뮬레이터로 **120Hz** 서브스텝.
3. **상태 추출** `CopyFromJSBSim()` (`:744` 부근) — 위치는 **ECEF**(지구중심좌표, m),
   자세는 오일러각, 그 외 속도/고도/舵面/실속경고를 `AircraftState`(FAircraftState)로.
4. **좌표 변환** — `AGeoReferencingSystem::ECEFToEngine()`으로 ECEF → UE 월드(cm).
   탄젠트 프레임은 East-South-Up, 헤딩 보정 `Yaw -= 90`(JSBSim 0=북, UE 0=동).
5. **액터 갱신** — `SetActorLocationAndRotation()` (`:404` 부근)으로 폰(=메시)을 그 위치·
   자세로 이동. 즉 **메시는 매 틱 JSBSim이 계산한 곳으로 옮겨지는 인형**이다.
6. AGL(지면고도)은 `UEGroundCallback`이 UE 콜리전 라인트레이스로 JSBSim에 공급.

엔진 틱은 `Config/DefaultEngine.ini`에서 **120FPS 고정**(`bUseFixedFrameRate=True`) —
JSBSim 120Hz 서브스텝과 정합되어 결정성을 확보한다.

### 수정하려면 어디를 건드리나

| 하고 싶은 것 | 건드릴 곳 |
|---|---|
| 다른 기종 추가 (물리) | `Resources/JSBSim/aircraft/`에 폴더 추가(Boeing314 등 기존 다수) + 폰의 `AircraftModel` 문자열 |
| 다른 기종 추가 (시각) | 새 SkeletalMesh + AnimBP를 폰 BP의 Jet 컴포넌트에 지정 (CommercialPlane 폴더가 예시) |
| 비행 특성 튜닝 | `f16.xml`의 공력계수/FCS (C++ 아님) |
| 물리→화면 반영 방식 | `JSBSimMovementComponent.cpp` TickComponent / CopyFromJSBSim |
| 조종면 애니메이션 | `AB_Jet` AnimBP (UE 에디터 필요) |

---

## 2. 항공기 액터 — M_F16 / F16_UAV

폰은 **순수 Blueprint**다 (C++ 베이스 클래스 없음). C++는 리플렉션으로 BP 변수를 읽고 쓴다.

| 자산 | 정체 |
|---|---|
| `Content/Blueprints/M_F16.uasset` | 유인기. JSBSimMovementComponent + sk_Jet 메시 + PFD + WeaponComponent/HealthComponent + Team/BulletAmmo/RocketAmmo 변수 |
| `Content/Blueprints/F16_UAV.uasset` | 무인기. 더 가벼운 변형 — UDP/오토파일럿 구동. 런타임 스폰(키 P) |
| `F_16.uasset`, `BP_Airliner.uasset` | **ObjectRedirector → M_F16** (별도 기체 아님, 이름 잔재) |

공통 컴포넌트: SkeletalMesh "Jet"(sk_Jet), UJSBSimMovementComponent(AircraftModel="f16"),
SpringArm+Camera, AudioComponent, WidgetComponent(PFD `UMG_BasicPrimaryFlightDisplay`).
BP 변수 `UDP_Roll/Pitch/Yaw/Throttle`은 C++(UDPControlReceiver)가 쓰고 BP가 읽어 전달하는
5005 경로의 다리다. 상세: [README_F16_Actor.md](README_F16_Actor.md)

게임 모드: `BP_JSBSimGameMode`(DefaultPawn=M_F16), 메인 맵 `Content/RL_30.umap`(Cesium).

### 수정하려면 어디를 건드리나

| 하고 싶은 것 | 건드릴 곳 |
|---|---|
| 폰 구성(컴포넌트/변수/입력 이벤트) | M_F16/F16_UAV BP — **UE 에디터 필요** (바이너리 자산) |
| 키보드 조작 바인딩 | `Config/DefaultInput.ini` (Q/E 에일러론, PgUp/PgDn 엘리베이터, W/S 스로틀, LMB/RMB 발사, P 스폰 등) |
| UAV 스폰 방식 | 현재 키 P 단발 스폰 — BP 레벨/폰 그래프 |
| PFD 표시 항목 | `Content/Blueprints/PFD/UMG_BasicPrimaryFlightDisplay.uasset` (에디터) |

---

## 3. UDP 통신 허브 — AUDPControlReceiver

`Source/MUMT_Sim/Private/UDPControlReceiver.cpp`(.h) — 이 시스템의 네트워크 오케스트레이터.
레벨에 1개 존재하며 세 개의 소켓과 이름 라우팅, 무인기 오토파일럿 틱을 소유한다.

| 포트 | 방향 | 내용 |
|---|---|---|
| **5005** (ListenPort) | RX | raw 명령 `{commands:[{aircraft_name, roll, pitch, yaw, throttle}]}` — 이름 매칭된 폰에만 적용(브로드캐스트 없음). 유인기 조이스틱 경로 |
| **5010** (SetpointListenPort) | RX | AircraftSetpoint v2 JSON — 무인기 고수준 명령(모드/헤딩/고도/속도/편대 슬롯/톨러런스/무장). NaN·Inf·빈 이름·역행 sequence 폐기 |
| **5006** (PythonStatePort) | TX | `aircraft_state_batch` JSON — 전 기체 위치/자세/속도/팀/탄약 + per-aircraft `guidance` 오브젝트(e_along/e_cross/e_vert/captured/maintained/separation 등) |

- **5010 → 60Hz `AutopilotTick` → `ApplyAutopilotToPawn`**: 무인기의 JSBSim 명령 기록
  지점은 이 함수 **한 곳**뿐이다 (아래 4장 스택 호출).
- **5005 → `ApplyControlCommandToPawn`**: 유인기 raw 조종(보존 경로). 두 경로는 이름
  라우팅으로 공존한다 (조이스틱→M_F16, BT→F16_UAV 동시 운용).
- 스키마·검증 규칙·실측 레이트: [README_UDP_Comms.md](README_UDP_Comms.md), [STATE_API.md](STATE_API.md)

### 수정하려면 어디를 건드리나

| 하고 싶은 것 | 건드릴 곳 |
|---|---|
| 상태 배치에 필드 추가 | `UDPControlReceiver.cpp`의 상태 빌드(BuildPawnState 계열) + [STATE_API.md](STATE_API.md) 갱신 + 소비자(bridge/BT) |
| 셋포인트 필드 추가 | `custom_msgs/AircraftSetpoint.msg` + bridge 직렬화 + UE 파싱 + [README_UDP_Comms.md](README_UDP_Comms.md) — **4곳 동기화 필수** |
| 포트/전송률 변경 | `UDPControlReceiver.h` 기본값 (5005/5006/5010, StateSendInterval) |
| 이름 라우팅 규칙 | ReceiveSetpointData/명령 적용부의 이름 매칭 로직 |

---

## 4. 무인기 제어 스택 — 3계층 (2026-07-11 전면 교체)

구 BVRGym `FAircraftAutopilot` → GetStick(`Controller_CY`) → `InnerLoopAutopilot` 계보는
**삭제**되었고, 현행 스택은 헤더 3개로 재편되었다 (전부 `Source/MUMT_Sim/Public/`,
의존성 없는 순수 수학 — UE 없이 스탠드얼론 검증 가능):

```
[1] FFormationGuidance    (FormationGuidance.h)    리더 상태 → 슬롯 위치/속도/가속도, 오차, 캡처 판정
[2] FFixedWingGuidance    (FixedWingGuidance.h)    벡터필드 횡유도 + REJOIN, 고도→FPA/pitch, 속도법칙
[3] FF16CommandController (F16CommandController.h) roll/pitch/airspeed ref → PID → 조종면 cmd-norm
```

- Direct 모드(heading/alt/speed)와 Attack 모드(FPursuitGuidance)도 같은 [2]→[3] 체인을 지난다.
- 편대 다중기 지원: `Public/FormationControl/`의 슬롯 생성기/캡처 플래너 + PX4에서 이식한
  어댑터(TECS/NPFG 등 — [PX4_PORT_MANIFEST.md](PX4_PORT_MANIFEST.md)).
- 제어 게인·법칙은 JSBSim 폐루프 하네스로 검증된 값 — **수치 임의 변경 금지**, 변경 시
  `Tools/build_verify_formation.sh`(스탠드얼론) + `Tools/run_formation_test.sh`(인엔진 회귀)로
  기준선 재검증. 상세(계보·게인·2026-07-11 견고성 수정): [README_Autopilot.md](README_Autopilot.md)

### 수정하려면 어디를 건드리나

| 하고 싶은 것 | 건드릴 곳 |
|---|---|
| 편대 형상/슬롯 정의 | BT 셋포인트(front/right/up 오프셋) 또는 `FormationControl/FormationSlotGenerator.h` |
| 유도 법칙(횡/종/속도) | `FixedWingGuidance.h` — 게인 변경 시 회귀 스크립트 필수 |
| 내루프 PID | `F16CommandController.h` — 동일 |
| 새 비행 모드 추가 | `ApplyAutopilotToPawn`의 모드 분기 + AircraftSetpoint 프로토콜 확장 |

---

## 5. 무장·교전 시스템

Phase 1~3 구현 완료 상태 (Health/Weapon 컴포넌트 → 무장 프로토콜 → dogfight BT).
파일 단위 상세는 [combat_system_files.md](combat_system_files.md) 참조.

- **`WeaponComponent`** (`Source/MUMT_Sim/Public/WeaponComponent.h` + `Private/.cpp`):
  기총(히트스캔 콘 탐색, 사거리/탄수 설정) + 미사일(`FireMissile()` — 콘 내 최근접 적 조준,
  `GetLastMissileTarget()`, `ReportMissileHit()`로 BP 미사일이 명중 보고 → 데미지 적용).
- **`HealthComponent`** (`Public/HealthComponent.h`): 체력/데미지, AGL 임계 크래시 판정,
  OnHealthChanged 이벤트.
- **투사체는 JSBSim이 아니다**: `Content/F16Control/blueprint/BP_rocket.uasset` /
  `BP_Bullet.uasset` — 단순 기구학 + 유도(호밍)로 움직이는 BP 액터.
- 발사 경로: 유인기 키보드/조이스틱(LMB/RMB) 또는 BT 셋포인트의 무장 필드(5010 경유).
- BT 쪽 교전 판단(WEZ)은 py_bt_ros dogfight 시나리오에 있다 (8장).

### 수정하려면 어디를 건드리나

| 하고 싶은 것 | 건드릴 곳 |
|---|---|
| 사거리/콘 각/데미지/탄수 | `WeaponComponent.h` 기본값 — **주의: BT XML의 WEZ 파라미터와 이중 하드코딩** (아래 10장 함정) |
| 미사일 비행/유도 연출 | `BP_rocket` BP 그래프 (에디터) |
| 피격/격추 처리 | `HealthComponent.cpp` + 폰 BP의 이벤트 바인딩 |
| 유인기 기총 연출 | M_F16 BP + C++ ProcessEvent 폴백 `HandleGunFiringChanged` (10장 함정 참조) |

---

## 6. ROS 브리지 — mumt_ros_ws

패키지 2개뿐인 얇은 워크스페이스. UE도 BT도 모르는 **순수 변환 계층**이다.

- **`custom_msgs`**: `msg/AircraftSetpoint.msg` — 셋포인트 프로토콜 정의(한국어 주석).
  내루프 명령 + 유도 모드(direct/formation/attack) + 편대 프로토콜(sequence/timestamp/톨러런스)
  + ControlV2 필드. **프로토콜의 단일 진실 원천**.
- **`mumt_ros_bridge`**:
  - `mumt_ros_bridge/bridge_node.py` — `MumtBridgeNode`. SUB `/aircraft/setpoint` → JSON →
    UDP 5010 (sequence 자동 증가, CLOCK_MONOTONIC timestamp 스탬프, NaN/Inf 드롭),
    SUB `/mumt/aircraft_commands` → UDP 5005, RX UDP 5006 → PUB `/mumt/aircraft_states`(60Hz 드레인).
  - `mumt_ros_bridge/joystick_node.py` — `/joy`(Thrustmaster T.16000M) → `/mumt/aircraft_commands`.
    축/버튼 매핑은 `config/joystick.yaml`.
  - 런치: `launch/manned_joystick.launch.py` (joy_node + joystick + bridge 일괄 —
    **bridge를 따로 또 띄우면 5006 충돌**).
  - ControlV2 하트비트/sequence 계약: `CONTROLV2_SENDER.md` (repo 내).
- 상세: [README_ROS_Bridge.md](README_ROS_Bridge.md), [README_Joystick_Manned.md](README_Joystick_Manned.md)

### 수정하려면 어디를 건드리나

| 하고 싶은 것 | 건드릴 곳 |
|---|---|
| 셋포인트 프로토콜 변경 | `AircraftSetpoint.msg` 먼저 → bridge 직렬화 → UE 파싱 (3장 표와 동일한 4곳 동기화) |
| 조이스틱 축/버튼 매핑 | `config/joystick.yaml` (jstest로 인덱스 확인 후) |
| 원격 UE 접속 | 런치 인자 `unreal_ip:=<ip>` — 단 **CLOCK_MONOTONIC timestamp는 동일 호스트 전제** (10장 함정) |

---

## 7. BT 두뇌 — py_bt_ros

**py_trees가 아니라 자체 BT 프레임워크**다 (구 문서의 py_trees 서술은 구식).

- **프레임워크** (`modules/`):
  `base_bt_nodes.py`(Node/Sequence/Fallback/ReactiveSequence/ReactiveFallback/Parallel + Status),
  `base_bt_nodes_ros.py`(ROS 토픽/서비스/액션 래퍼), `bt_constructor.py`(XML → 트리 파서),
  `agent.py`(에이전트 컨테이너 + 블랙보드), `ros_bridge.py`(rclpy 싱글톤),
  `bt_runner.py`+`bt_visualiser.py`(pygame 시각화, ~30Hz 틱). 진입점 `main.py`.
- **시나리오** (`scenarios/`): 각 폴더 = bt_nodes.py(노드 구현) + *_bt.xml(트리) + configs/.
  - `mumt/` — 공용 프리미티브: Takeoff, HoldSetpoint, MaintainFormation, OrbitLeader 등.
    leader/wingman/uav1/uav2 트리 XML.
  - `mumt_dogfight_1v1/` — 교전: GatherCombatState(상태 파싱→블랙보드) →
    ConditionEnemyAlive → EngageTarget(추격 셋포인트 + 무장 발사 필드 발행). uav1(적)/uav2(아군) XML.
  - `mumt_formation/`, `mumt_intercept/`, `mumt_weapons_test/`, `mumt_waypoint/`,
    `plugins/mrta/`(GRAPE/CBBA/Hungarian 임무 할당).
- **ROS 인터페이스**: SUB `/mumt/aircraft_states`(String JSON) / PUB `/aircraft/setpoint`
  (AircraftSetpoint) — 6장 브리지와 정확히 맞물린다.
- **블랙보드**: GatherCombatState가 own_state/all_states/own_team/enemies/init_alt를 쓰고
  하위 노드가 읽는다 — **스키마 검증 없음** (10장 함정).

### 수정하려면 어디를 건드리나

| 하고 싶은 것 | 건드릴 곳 |
|---|---|
| 임무 시나리오 변경 | 해당 `scenarios/*/`의 `*_bt.xml` (트리 구조) — 코드 수정 없이 가능 |
| 새 행동/조건 노드 | 해당 시나리오 `bt_nodes.py`에 클래스 추가 (base_bt_nodes_ros 래퍼 상속) |
| 교전 판단(WEZ/추격) | `mumt_dogfight_1v1/bt_nodes.py` EngageTarget + XML의 WEZ 파라미터 |
| 새 시나리오 | `scenarios/` 폴더 복제 패턴 (bt_nodes.py + XML + configs) |

---

## 8. 지형·좌표계 — Cesium과 프레임 규약

- 메인 맵 `Content/RL_30.umap`: Cesium World Terrain + CesiumGeoreference. 항공기는 실제
  위경도 위를 난다. (RL_2.umap은 비-Cesium 대안 맵.)
- 클론 시 Cesium 플러그인 + .uplugin 필요, ion 토큰 이슈는 10장 참조.

**프레임 규약 (UE·BT 공통 — 승계, 여전히 유효):**
- UE 월드: x=East, y=South, z=Up (cm).
- 컴퍼스 헤딩(0=북, 90=동) = `atan2(ΔEast, ΔNorth) = atan2(Δx, −Δy)` —
  수학 프레임 `atan2(Δy,Δx)`로 쓰면 90° 틀어진다.
- 고도: 상태 배치의 `z/100` = **UE-Z(m)** — JSBSim ASL이 아니다. 지면이 음수일 수 있어
  BT는 스폰 기준 상대 상승량으로 판단한다.
- JSBSim 위치는 ECEF, 헤딩 0=북 — UE 변환 시 `Yaw -= 90` (1.3장).

---

## 9. 유인기 조종 경로 (참고)

- **조이스틱**: Thrustmaster T.16000M → `joy_node` → `joystick_node.py` →
  `/mumt/aircraft_commands` → bridge → UDP 5005 → 이름 매칭 → M_F16.
  UE Enhanced Input에 HOTAS 바인딩은 없다 (UDP 경유가 유일한 조이스틱 경로).
- **키보드/마우스**: `Config/DefaultInput.ini` — UE 내 직접 바인딩 (LMB 기총, RMB 로켓 포함).
- 상세: [README_Joystick_Manned.md](README_Joystick_Manned.md)

---

## 10. 알려진 부채·함정 (2026-07-22 갱신)

| # | 항목 | 상태 |
|---|---|---|
| 1 | **프로토콜 이중 하드코딩**: 교전 WEZ 파라미터(미사일/기총 사거리·콘 각)가 UE `WeaponComponent`와 py_bt_ros dogfight XML **양쪽에 존재** — 한쪽만 바꾸면 교전이 조용히 어긋난다 | 활성 함정 |
| 2 | **CLOCK_MONOTONIC 동일 호스트 전제**: bridge가 찍는 `command_timestamp`는 머신별 epoch — UE를 원격으로 옮기면 staleness 판정이 깨진다 | 활성 제약 |
| 3 | **블랙보드 무검증**: GatherCombatState 실패/JSON 이상 시 하위 BT 노드 KeyError 가능 | 활성 |
| 4 | **DefaultEngine.ini 스테일 항목**: `GameDefaultMap=OpenWorld`, `GlobalDefaultGameMode=AirSim.AirSimGameMode` — 실제 사용(RL_30 + BP_JSBSimGameMode)과 불일치 | 활성 (동작엔 지장 없으나 혼란 유발) |
| 5 | **Cesium ion 토큰이 uasset 바이너리에 커밋**됨 (`Content/CesiumSettings/CesiumIonServers/CesiumIonSaaS.uasset`) | 활성 (외부화 필요) |
| 6 | **ObjectRedirector 잔재**: `F_16`/`BP_Airliner`는 M_F16의 별칭 — 새 기체로 착각 금지 | 활성 |
| 7 | **유인기 기총 연출은 C++ ProcessEvent 폴백**(`HandleGunFiringChanged`) — 델리게이트 방식이 아님. `[SGF]` 임시 로그 제거 TODO 잔존 | 활성 |
| 8 | **launch 중복 실행 함정**: manned_joystick.launch와 별도 bridge 동시 실행 시 5006 바인딩 충돌 → 상태 수신 깜빡임 | 활성 함정 |
| 9 | NaN/Inf 셋포인트는 **조용히 드롭**(warn 로그만) — "명령이 안 먹는" 증상의 단골 원인 | 활성 |
| 10 | 최소 안전거리 기능은 **감속 보조일 뿐 충돌 회피가 아님** ([README_Autopilot.md](README_Autopilot.md) §견고성) | 설계 제약 |
| 11 | 미사일/기총 투사체는 단순 기구학 — 6-DoF 아님 (의도된 설계) | 설계 선택 |
| 12 | 구식 문서 주의: git 이력의 2026-06-28 ARCHITECTURE 스냅샷과 일부 README 교차 서술은 07-11 제어 교체 이전 기준일 수 있음 — 충돌 시 [README_Autopilot.md](README_Autopilot.md)와 이 문서가 우선 | 문서 위생 |

---

## 부록 A. 핵심 파일 경로 지도

```
MUMT_Sim/
  Source/MUMT_Sim/
    Public/  F16CommandController.h · FixedWingGuidance.h · FormationGuidance.h
             UDPControlReceiver.h · WeaponComponent.h · HealthComponent.h
             FormationControl/ (PX4 어댑터·슬롯 생성기)
    Private/ UDPControlReceiver.cpp · WeaponComponent.cpp · HealthComponent.cpp
  Plugins/JSBSimFlightDynamicsModel/
    Source/JSBSimFlightDynamicsModel/Private/JSBSimMovementComponent.cpp
    Source/JSBSimFlightDynamicsModel/Public/FDMTypes.h
    Resources/JSBSim/aircraft/f16/f16.xml · systems/ · engines/
  Content/
    Blueprints/M_F16 · F16_UAV · BP_JSBSimGameMode · PFD/UMG_BasicPrimaryFlightDisplay
    F16Control/meshes/sk_Jet · animation/AB_Jet · blueprint/BP_rocket · BP_Bullet
    RL_30.umap
  Config/DefaultEngine.ini · DefaultInput.ini
  Tools/build_verify_formation.sh · run_formation_test.sh · build_verify_units.sh
  docs/ (이 문서 세트)

mumt_ros_ws/src/
  custom_msgs/msg/AircraftSetpoint.msg
  mumt_ros_bridge/mumt_ros_bridge/bridge_node.py · joystick_node.py
  mumt_ros_bridge/launch/manned_joystick.launch.py · config/joystick.yaml
  mumt_ros_bridge/CONTROLV2_SENDER.md

py_bt_ros/
  main.py
  modules/base_bt_nodes.py · base_bt_nodes_ros.py · bt_constructor.py ·
          agent.py · ros_bridge.py · bt_runner.py · bt_visualiser.py
  scenarios/mumt/ · mumt_dogfight_1v1/ · mumt_formation/ · mumt_intercept/ ·
            mumt_weapons_test/ · mumt_waypoint/ · plugins/mrta/
```

## 부록 B. 문서 지도 (무엇을 어디서 읽나)

| 문서 | 내용 |
|---|---|
| [CURRICULUM.md](CURRICULUM.md) | **이 스택을 처음 공부하는 순서** (이 문서의 학습용 목차) |
| [README_Build_and_Run.md](README_Build_and_Run.md) | 빌드·실행 절차 |
| [README_JSBSim.md](README_JSBSim.md) / [README_JSBSim_IO.md](README_JSBSim_IO.md) | JSBSim 플러그인 내부와 입출력 |
| [README_F16_Actor.md](README_F16_Actor.md) | 항공기 폰 구성 |
| [README_UDP_Comms.md](README_UDP_Comms.md) / [STATE_API.md](STATE_API.md) | UDP 프로토콜·상태 스키마 |
| [README_Autopilot.md](README_Autopilot.md) | 3계층 제어 스택 (게인·법칙·검증 이력) |
| [PX4_PORT_MANIFEST.md](PX4_PORT_MANIFEST.md) | PX4 이식 부품 목록 |
| [README_ROS_Bridge.md](README_ROS_Bridge.md) / [README_Joystick_Manned.md](README_Joystick_Manned.md) | 브리지·조이스틱 |
| [combat_system_files.md](combat_system_files.md) | 무장/교전 시스템 파일 단위 상세 |
