# MUMT 스택 학습 커리큘럼 (읽기 중심)

> **대상**: 이 프로젝트를 인수받아 앞으로 주도적으로 발전시킬 유지보수자.
> **목표 수준**: 모든 서브시스템에 대해 "이걸 수정하려면 어디를 건드리나"를 답하고
> 실제로 고칠 수 있는 수준.
> **방식**: 읽기 중심 — 각 단계의 자료를 읽고, 이해 확인 질문에 스스로 답해본다.
> 답이 막히면 해당 자료를 다시 읽거나 Claude에게 질문한다. 실습은 강제가 아니며,
> 원할 때 [부록 실습](#부록-선택-실습)에서 골라 한다.
> **페이스**: 한 세션에 한 단계 권장 (0단계 제외 각 단계 1~2시간 분량).
> **순서의 근거**: [ARCHITECTURE.md](ARCHITECTURE.md)의 장 순서 = 데이터가 흐르는
> 의존성 순서다. 안쪽(물리)에서 바깥쪽(두뇌)으로 나간다.

---

## 0단계. 오리엔테이션 — 시스템의 큰 그림

**목표**: 3개 repo가 각각 무엇이고 어떻게 연결되는지 그림 한 장으로 설명할 수 있다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **0장** (데이터 흐름도 포함) — 정독
2. [README_Build_and_Run.md](README_Build_and_Run.md) — 훑어보기 (실행은 안 해도 됨)

**이해 확인 질문**
- 세 repo의 역할을 각각 한 문장으로 말할 수 있는가?
- UDP 포트 5005 / 5006 / 5010은 각각 무엇을, 어느 방향으로 나르는가?
- "이름 라우팅"이란 무엇이고, 왜 조이스틱→유인기와 BT→무인기가 동시에 돌아갈 수 있는가?
- ROS 토픽 `/aircraft/setpoint`와 `/mumt/aircraft_states`는 각각 어느 repo가 발행하고 어느 repo가 소비하는가?

---

## 1단계. 항공기가 나는 원리 — 물리와 화면의 연결

**목표**: "비행기를 어디서 가져왔나"에 완전히 답한다 — 물리 모델의 출처, 시각 모델의
출처, 둘을 잇는 틱 파이프라인을 코드 수준에서 설명할 수 있다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **1장** — 정독
2. [README_JSBSim.md](README_JSBSim.md) — 플러그인 구조
3. [README_JSBSim_IO.md](README_JSBSim_IO.md) — Commands/AircraftState 입출력 상세
4. 코드: `Plugins/JSBSimFlightDynamicsModel/Source/JSBSimFlightDynamicsModel/Private/JSBSimMovementComponent.cpp`
   — `TickComponent` / `CopyToJSBSim` / `CopyFromJSBSim`만 따라 읽기
5. 데이터: `Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim/aircraft/f16/f16.xml`
   — `<metrics>`, `<flight_control>` 섹션 훑어보기

**이해 확인 질문**
- F-16의 비행 특성(공력계수, 관성모멘트)은 C++ 어디에 있는가? (함정 질문)
- 매 프레임 물리 상태가 화면의 메시에 반영되는 5단계 경로를 순서대로 말할 수 있는가?
- ECEF 좌표는 무엇이고, 누가 UE 월드 좌표로 바꿔주는가?
- 조종면(에일러론)이 움직이는 애니메이션은 어떤 자산이 어떤 데이터를 읽어 만드는가?
- **수정 질문**: 비행기가 더 잘 돌게(선회 성능) 만들려면 어디를 고치는가? 새 기종(예: Boeing314)을 추가하려면 물리 쪽과 시각 쪽 각각 무엇이 필요한가?

---

## 2단계. 항공기 액터 — M_F16 / F16_UAV 폰

**목표**: 폰 BP의 컴포넌트 구성과 변수, 그리고 "왜 C++ 베이스 클래스가 없는데 C++가
폰을 제어할 수 있는지"를 설명할 수 있다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **2장**
2. [README_F16_Actor.md](README_F16_Actor.md) — 정독
3. 설정: `Config/DefaultInput.ini` — 키 바인딩 훑어보기

**이해 확인 질문**
- M_F16과 F16_UAV의 차이는 무엇인가? `F_16`과 `BP_Airliner`는 왜 "가짜"인가?
- `UDP_Roll` 같은 BP 변수는 누가 쓰고 누가 읽는가? (C++↔BP 경계)
- F16_UAV는 어떻게 스폰되는가?
- **수정 질문**: 폰에 새 센서(예: 연료량 표시)를 추가해 상태 배치로 내보내려면 어디어디를 건드리는가? (힌트: BP 변수 + 3단계의 상태 빌드)

---

## 3단계. UDP 통신 허브 — AUDPControlReceiver

**목표**: UE 쪽 네트워크 오케스트레이터의 세 소켓과 두 인바운드 경로(raw 명령 vs
셋포인트)를 구분하고, 상태 배치 JSON의 구조를 안다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **3장**
2. [README_UDP_Comms.md](README_UDP_Comms.md) — 정독 (스키마·검증 규칙)
3. [STATE_API.md](STATE_API.md) — 상태 배치 필드 레퍼런스
4. 코드: `Source/MUMT_Sim/Private/UDPControlReceiver.cpp` — `ReceiveSetpointData`,
   `AutopilotTick`, 상태 송신부만 골라 읽기 (전체 70KB를 다 읽으려 하지 말 것)

**이해 확인 질문**
- 5005 명령과 5010 셋포인트는 추상화 수준이 어떻게 다른가? 각각 최종적으로 어느 함수에서 폰에 적용되는가?
- NaN이 섞인 셋포인트를 보내면 무슨 일이 일어나는가? (함정 #9)
- 무인기의 JSBSim 명령 기록 지점이 한 곳뿐이라는 말의 의미와 그 함수 이름은?
- **수정 질문**: 상태 배치에 새 필드 하나를 추가하면 함께 고쳐야 할 곳이 총 몇 군데인가? (UE, 문서, 소비자)

---

## 4단계. ROS 브리지와 조이스틱 — mumt_ros_ws

**목표**: 브리지가 "얇은 변환 계층"임을 이해하고, AircraftSetpoint.msg가 프로토콜의
단일 진실 원천임을 안다. 조이스틱 입력이 유인기까지 가는 전 경로를 말할 수 있다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **6장, 9장**
2. [README_ROS_Bridge.md](README_ROS_Bridge.md) — 정독
3. [README_Joystick_Manned.md](README_Joystick_Manned.md) — 정독
4. 코드: `mumt_ros_ws/src/custom_msgs/msg/AircraftSetpoint.msg` (주석까지 전부),
   `mumt_ros_ws/src/mumt_ros_bridge/mumt_ros_bridge/bridge_node.py` (187줄, 전부 읽을 만함)

**이해 확인 질문**
- 조이스틱을 움직이면 유인기가 반응하기까지 거치는 노드/토픽/포트를 순서대로 나열할 수 있는가?
- sequence_id와 timestamp는 누가 찍고 UE는 언제 패킷을 버리는가?
- 왜 UE를 다른 컴퓨터로 옮기면 staleness 판정이 깨지는가? (함정 #2)
- manned_joystick.launch를 띄운 상태에서 bridge를 하나 더 실행하면? (함정 #8)
- **수정 질문**: 셋포인트에 새 필드를 추가하는 4곳 동기화 절차를 순서대로 말할 수 있는가?

---

## 5단계. 무인기 제어 스택 — 3계층

**목표**: 셋포인트가 조종면 명령이 되기까지 [1]편대유도 → [2]고정익유도 → [3]내루프
PID의 입출력을 계층별로 설명하고, "게인을 함부로 바꾸면 안 되는 이유"를 안다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **4장**
2. [README_Autopilot.md](README_Autopilot.md) — **이 스택에서 가장 중요한 문서. 정독 2회 권장**
3. [PX4_PORT_MANIFEST.md](PX4_PORT_MANIFEST.md) — 이식 부품 목록
4. 코드(헤더만): `Source/MUMT_Sim/Public/FormationGuidance.h` → `FixedWingGuidance.h`
   → `F16CommandController.h` — 각 struct의 입출력 타입 위주로

**이해 확인 질문**
- 세 계층 각각의 입력 타입과 출력 타입은 무엇인가? (FFormationTarget → FFlightReference → FF16ControlCommand)
- Direct/Formation/Attack 모드는 어느 지점에서 갈라지고 어디서 다시 합쳐지는가?
- 요-SAS(러더 감쇠)는 왜 C++ 스택에 없는가? 어디에 있는가?
- "최소 안전거리" 기능이 충돌 회피가 아닌 이유는? (함정 #10)
- **수정 질문**: 편대 슬롯 간격을 바꾸고 싶다 — 어디를 고치는가? 유도 게인을 바꿨다면 커밋 전에 무엇을 돌려야 하는가?

---

## 6단계. BT 두뇌 — py_bt_ros

**목표**: 자체 BT 프레임워크의 틱 모델(Node/Status, Reactive 계열)을 이해하고, XML
트리와 bt_nodes.py 노드 구현의 관계를 안다. 새 노드를 추가할 수 있는 상태가 된다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **7장**
2. 코드: `py_bt_ros/modules/base_bt_nodes.py` — Node.run()과 Status 상태기계 먼저
3. 코드: `py_bt_ros/modules/bt_constructor.py` — XML이 트리가 되는 과정
4. 코드: `py_bt_ros/scenarios/mumt/bt_nodes.py` + `default_bt.xml` — 공용 프리미티브
5. 흐름 추적: `main.py` → `bt_runner.py` → `agent.py` → tree.run() 한 틱 따라가기

**이해 확인 질문**
- Sequence와 ReactiveSequence의 차이는? 이 프로젝트가 py_trees를 안 쓰는 이유로 추정되는 것은?
- 블랙보드에는 누가 무엇을 쓰고, 스키마 검증이 없으면 어떤 사고가 나는가? (함정 #3)
- BT는 어떤 토픽을 읽고 어떤 토픽에 쓰는가? 그 토픽은 최종적으로 어느 UDP 포트로 이어지는가?
- 트리 XML만 고쳐서 할 수 있는 일과 bt_nodes.py를 고쳐야 하는 일의 경계는?
- **수정 질문**: "적기가 접근하면 회피 기동" 행동을 추가한다면 — 어느 파일에 무슨 클래스를 만들고 XML 어디에 꽂는가?

---

## 7단계. 무장·교전 시스템

**목표**: 기총/미사일의 발사 경로(유인기·BT 양쪽), 데미지/격추 처리, 그리고 UE와 BT에
이중 하드코딩된 WEZ 파라미터의 위험을 안다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **5장**
2. [combat_system_files.md](combat_system_files.md) — 파일 단위 상세 (분량 많음 — 레퍼런스로 활용)
3. 코드: `Source/MUMT_Sim/Public/WeaponComponent.h` → `HealthComponent.h` (헤더 먼저)
4. 코드: `py_bt_ros/scenarios/mumt_dogfight_1v1/bt_nodes.py` + `uav1_bt.xml` — 교전 판단 쪽

**이해 확인 질문**
- 미사일 발사부터 명중 데미지까지: FireMissile → BP_rocket → ReportMissileHit 경로를 설명할 수 있는가?
- 투사체가 JSBSim을 안 쓰는 것은 버그인가 설계인가?
- 기총 사거리를 UE에서만 바꾸면 교전에서 무슨 일이 벌어지는가? (함정 #1)
- 유인기 기총 연출이 "폴백"으로 동작한다는 것의 의미는? (함정 #7)
- **수정 질문**: 미사일 데미지를 40→60으로 바꾸려면 어디인가? WEZ를 바꾸려면 몇 군데인가?

---

## 8단계. 졸업 — 전체 추적과 부채 목록

**목표**: 임의의 증상("무인기가 명령을 안 듣는다")을 듣고 어느 계층부터 의심할지 진단
순서를 세울 수 있다. 알려진 함정 12개를 자기 말로 설명할 수 있다.

**읽기**
1. [ARCHITECTURE.md](ARCHITECTURE.md) **10장** (부채·함정) — 정독
2. 복습: 0장 데이터 흐름도를 보지 않고 직접 그려본 뒤 원본과 대조

**이해 확인 질문 (종합)**
- BT가 발행한 셋포인트 하나가 조종면을 움직이기까지의 전 경로를 repo 경계 표시와 함께 그릴 수 있는가?
- "무인기가 갑자기 명령을 안 듣는다" — 함정 목록에서 후보 3개를 대고 각각 확인 방법을 말할 수 있는가? (NaN 드롭, seq 역행, 5006 충돌…)
- 다음에 하고 싶은 기능(예: 교전 폴리싱, 새 시나리오)을 하나 고르고, 건드릴 파일 목록을 스스로 작성해보라 — 이것이 이 커리큘럼의 최종 시험이다.

---

## 부록. 선택 실습

읽기만으로 부족하다고 느낄 때 골라서 한다. 순서 무관, 전부 선택.

| # | 실습 | 관련 단계 | 방법 |
|---|---|---|---|
| A1 | 시뮬 구동 + 조이스틱 비행 | 0, 4 | [README_Build_and_Run.md](README_Build_and_Run.md) 절차 + `ros2 launch mumt_ros_bridge manned_joystick.launch.py` |
| A2 | 상태 패킷 실물 보기 | 3, 4 | `ros2 topic echo /mumt/aircraft_states` 또는 `tcpdump -i lo udp port 5006 -A` |
| A3 | UE 에디터에서 BP 내부 열람 | 1, 2, 7 | M_F16 컴포넌트 트리, AB_Jet AnimGraph, BP_rocket 그래프 열어보기 (BP는 바이너리라 에디터가 유일한 열람 수단) |
| A4 | 새 항공기 조합 실험 | 1 | 폰 복제 → `AircraftModel="Boeing314"` + CommercialPlane 메시 지정 |
| A5 | BT 트리 XML만 수정 | 6 | `default_bt.xml`에서 노드 순서 바꾸고 pygame 시각화로 틱 관찰 |
| A6 | 제어 회귀 스크립트 실행 | 5 | `Tools/build_verify_formation.sh` (UE 없이 폐루프 검증 체험) |
| A7 | 패킷 수동 주입 | 3 | Python one-liner로 UDP 5010에 direct 모드 셋포인트 send → 무인기 반응 관찰 |

---

*이 커리큘럼은 2026-07-22 [ARCHITECTURE.md](ARCHITECTURE.md) 개정판 기준이다. 문서와
코드가 충돌하면 코드가 진실이고, 그 발견은 문서에 반영한다 (부채 #12).*
