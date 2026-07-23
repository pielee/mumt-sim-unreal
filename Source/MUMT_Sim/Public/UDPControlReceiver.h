#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "GameFramework/Actor.h"
#include "IPAddress.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "UDPControlReceiver.generated.h"

class APawn;
class UJSBSimMovementComponent;
// Per-UAV 3계층 제어 스택 상태 (FormationGuidance/FixedWingGuidance/F16CommandController)
// — .cpp 정의 (헤더 의존 최소화).
struct FUavControl;
// 마지막 유도 계산 결과 (5006 상태 피드백용) — .cpp 정의.
struct FUavGuidanceStatus;
// 인엔진 편대 시험 상태 (스크립트 리더 + 슬롯오차 기록) — .cpp 정의.
struct FFormationTest;

struct FRemoteControlCommand
{
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 0.0;
    double Throttle = 0.0;
    bool bValid = false;
    // Weapon triggers (Phase 3) — optional fields, keep defaults for old senders.
    bool  bGunFiring = false;
    int64 MissileFireId = 0;   // 0 = never fired (msg default); fire on change of id > 0
};

// 유도 모드: BT는 모드·대상만 지정하고 연속 수치 계산은 UE 3계층 스택이 60Hz로 수행.
// Direct = 기존 방식(setpoint의 heading/alt/speed 그대로) — 구 시나리오 완전 호환.
enum class EGuidanceMode : uint8
{
    Direct,      // ""/"direct" — heading/alt/speed → FixedWingGuidance::StepDirect
    Formation,   // leader_name pawn 직독 → FormationGuidance → FixedWingGuidance
    Attack,      // target_name pawn 직독 → FPursuitGuidance → StepDirect
};

// High-level autopilot setpoint for one UAV (heading/altitude/speed-or-throttle).
struct FUavSetpoint
{
    float HeadingDeg     = 0.f;   // 목표 헤딩(0=북) → course reference
    float AltitudeM      = 0.f;   // 목표 고도(UE-Z, m) → altitude reference
    float RollFfDeg      = 0.f;   // 뱅크 피드포워드(deg) — direct 모드 외부 보상용
    float Throttle       = 0.8f;  // used only when TargetSpeedMps <= 0 (open-loop)
    float TargetSpeedMps = 0.f;   // >0 → airspeed reference (속도 PI가 유지)
    // ── 유도 모드 ──
    EGuidanceMode Mode = EGuidanceMode::Direct;
    FString LeaderName;           // formation: 리더 pawn 이름 (setpoint 키와 동일 매칭 규칙)
    FString TargetName;           // attack: 표적 pawn 이름
    float SlotFrontM  = -80.f;    // 슬롯 오프셋 — 리더 트랙 프레임 (+앞/+우/+상)
    float SlotRightM  = 100.f;
    float SlotUpM     = 0.f;
    float MinSpeedMps = 70.f;     // 유도 속도 한계
    float MaxSpeedMps = 335.f;
    float MinAltM     = 0.f;      // 유도 고도 하한(UE-Z m). 0 = 가드 없음
    // ── 편대 판정·안전 한계 (신규 프로토콜 v2) ──
    float CaptureTolM    = 0.f;   // 슬롯 캡처 톨러런스(3D m). 0 → 기본 30
    float MaintainTolM   = 0.f;   // 편대 유지 톨러런스(3D m). 0 → 기본 50
    float MinSeparationM = 0.f;   // 리더와의 3D 최소 안전거리. 0 = 검사 없음
    float MaxClosingMps  = 0.f;   // 슬롯 접근 폐속도 상한. 0 = 제한 없음
    // ── 패킷 검증 (신규 프로토콜 v2) ──
    uint32 SequenceId      = 0;   // 0 = 미사용. >0 이면 기체별 단조증가 — 낡은 패킷 폐기
    double TimestampS      = 0.0; // 송신 unix 시각(초). 진단용
    uint8  ProtocolVersion = 0;   // 0/1 = 구 발신자, 2 = 편대 프로토콜
    bool  LaunchMissile  = false; // deprecated — use MissileFireId
    // Weapon triggers (Phase 3) — optional fields, keep defaults for old senders.
    bool  bGunFiring     = false; // level-triggered: fire while true
    int64 MissileFireId  = 0;     // edge-triggered: one shot per new id (0 = never fired)
};

UCLASS()
class MUMT_SIM_API AUDPControlReceiver : public AActor
{
    GENERATED_BODY()

public:
    AUDPControlReceiver();

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    bool StartUDPReceiver();
    void StopUDPReceiver();
    void ReceiveUDPData();
    void ParseCommand(const FString& Message);
    bool ParseJsonCommand(const FString& Message);
    void ParseLegacyCsvCommand(const FString& Message);

    // ── Autopilot internals ──────────────────────────────────────────────
    bool StartSetpointReceiver();
    void StopSetpointReceiver();
    void ReceiveSetpointData();   // called every Tick, drains SetpointSocket
    void AutopilotTick();         // called at 60 Hz via FTimerHandle
    void ApplyAutopilotToPawn(APawn* Pawn, const FString& Key, const FUavSetpoint& Setpoint,
                              const TArray<AActor*>& Pawns);   // Pawns: 리더/표적 직독용 현재 pawn 목록
    void UpdateFormationTest(const TArray<AActor*>& Pawns);    // 인엔진 편대 시험 (스크립트 리더)

    bool StartUDPSender();
    void StopUDPSender();
    void SendStateToPython();

    APawn* FindTargetPawn();
    TArray<APawn*> FindTargetPawns(const TArray<FString>& NamePatterns, int32 MaxCount = INDEX_NONE) const;
    bool DoesPawnMatchPatterns(const APawn* Pawn, const TArray<FString>& NamePatterns) const;
    bool IsUavPawn(const APawn* Pawn) const;
    bool SetBlueprintNumber(APawn* Pawn, const FName VarName, double Value);
    bool ApplyControlCommandToPawn(APawn* Pawn, const FRemoteControlCommand& Command);

    TSharedPtr<FJsonObject> BuildPawnState(APawn* Pawn);
    bool TryGetBlueprintBool(APawn* Pawn, const FString& VarName, bool& OutValue) const;
    bool TryGetBlueprintInt(APawn* Pawn, const FString& VarName, int32& OutValue) const;
    bool TryGetBlueprintNumber(APawn* Pawn, const FString& VarName, double& OutValue) const;
    bool TryGetBlueprintString(APawn* Pawn, const FString& VarName, FString& OutValue) const;
    void AddOptionalBoolField(const TSharedPtr<FJsonObject>& JsonObject, const FString& JsonKey, APawn* Pawn, const FString& VarName) const;
    void AddOptionalIntField(const TSharedPtr<FJsonObject>& JsonObject, const FString& JsonKey, APawn* Pawn, const FString& VarName) const;
    void AddOptionalNumberField(const TSharedPtr<FJsonObject>& JsonObject, const FString& JsonKey, APawn* Pawn, const FString& VarName) const;
    void AddOptionalStringField(const TSharedPtr<FJsonObject>& JsonObject, const FString& JsonKey, APawn* Pawn, const FString& VarName) const;
    UJSBSimMovementComponent* FindJSBSimMovementComponent(APawn* Pawn) const;

private:
    FSocket* ListenSocket    = nullptr;
    FSocket* SendSocket      = nullptr;
    FSocket* SetpointSocket  = nullptr;   // binary autopilot setpoint receiver
    TSharedPtr<FInternetAddr> PythonAddr;

    APawn* CachedTargetPawn = nullptr;
    float StateSendAccumulator = 0.0f;

    FRemoteControlCommand BroadcastCommand;
    TMap<FString, FRemoteControlCommand> NamedControlCommands;
    TArray<FRemoteControlCommand> IndexedControlCommands;

    // Autopilot state (game-thread only) — PER-UAV, keyed by aircraft name.
    // Multiple UAVs (each driven by its own BT) get their own setpoint slot and
    // their own control-stack instance (so their control state never mixes).
    TMap<FString, FUavSetpoint>            Setpoints;    // aircraft name -> latest setpoint
    TMap<FString, TSharedPtr<FUavControl>> UavControls;  // aircraft name -> 3계층 제어 스택
    TMap<FString, uint32>                  LastSetpointSeq;      // aircraft name -> 마지막 sequence_id
    TMap<FString, TSharedPtr<FUavGuidanceStatus>> GuidanceStatusByPawn; // pawn 이름 -> 5006 피드백
    TSharedPtr<FFormationTest>             FormationTest;
    FTimerHandle AutopilotTimerHandle;
    FTimerHandle StateSendTimerHandle;

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Receiver")
    int32 ListenPort = 5005;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Sender")
    FString PythonIP = TEXT("127.0.0.1");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Sender")
    int32 PythonStatePort = 5006;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Sender")
    float StateSendInterval = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    FString TargetPawnName = TEXT("F16_UAV");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    TArray<FString> ObservedPawnNamePatterns = { TEXT("F16"), TEXT("UAV") };

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    TArray<FString> ControlledPawnNamePatterns = { TEXT("F16_UAV"), TEXT("UAV"), TEXT("M_F16") };

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    FString UavNamePattern = TEXT("UAV");

    // ── Autopilot (3계층 스택) ────────────────────────────────────────────

    // Port the bridge sends JSON setpoint packets to
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|UDP")
    int32 SetpointListenPort = 5010;

    // Enable to fly the cached target straight ahead (direct 모드 setpoint 주입) —
    // ROS 없이 PIE에서 reference 추종/게인 튜닝용.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug")
    bool bUseDebugSetpoint = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug",
              meta = (EditCondition = "bUseDebugSetpoint"))
    float DebugUpM = 1000.f;           // climb this far above the spawn altitude

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug",
              meta = (EditCondition = "bUseDebugSetpoint"))
    float DebugTargetSpeedMps = 220.f; // airspeed reference for the debug flight

    // ── 유인기 조종 형성 레이어 (기본값 = 전부 OFF) ────────────────────────────
    // 기본값에서 조이스틱 명령은 가공 없이 그대로 JSBSim에 들어간다
    // (Aileron=Roll, Elevator=Pitch, Rudder=Yaw, Throttle=Throttle).
    // 아래 셋을 켜면 편대비행용 형성 레이어가 다시 얹힌다:
    //   MannedSpeedLimitMps 300 / MannedControlAuthority 0.4 / bMannedRollAssist true

    // Manned top-speed governor (m/s). Full power below the limit; throttle
    // authority tapers above it (soft FBW-style speed protection). 300 leaves the
    // formation UAV (cap 335 = measured f16 Vmax) ~35 m/s closure margin so it can
    // always catch the leader. 0 = off (raw throttle passthrough).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    float MannedSpeedLimitMps = 0.f;

    // Manned control authority: scales the joystick PITCH/YAW commands before the
    // flight model (roll too, but only when bMannedRollAssist is off). Throttle is
    // NOT scaled (governor handles top speed). Manned/joystick path only.
    // 1.0 = no scaling (raw passthrough).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target",
              meta = (ClampMin = "0.05", ClampMax = "1.0"))
    float MannedControlAuthority = 1.0f;

    // ── 유인기 Roll Assist (FBWA/PX4-Stabilized류 스틱 해석층) ─────────────────
    // f16 FCS는 rate-command FBW라 스틱 유지 = 무한정 롤(뱅크 상한 없음) → 리더 선회율이
    // 요동쳐 무인기 roll_ff 가정이 깨진다. 어시스트가 스틱을 '뱅크각 명령'으로 재해석:
    //   목표뱅크 = 스틱 × BankLimit → 필요 롤레이트 = Kp×(목표−현재), ±RateLimit
    //   → /180 정규화(f16 FCS 규격: 1.0 = 180°/s) → aileron-cmd.
    // 효과: 스틱 유지 = 그 뱅크 유지(선회 유지), 놓으면 수평 복귀, 리더 ω 유계·평활.
    // 기본 false = 스틱이 롤레이트 명령으로 그대로 통과(f16 FCS 원래 계약).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    bool bMannedRollAssist = false;

    // 풀스틱 뱅크각. 무인기 뱅크캡이 70°이므로 65~70이 '편대 추종 보장' 범위 —
    // 그 이상은 도그파이트용(무인기가 순간적으로 밀릴 수 있음).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target",
              meta = (EditCondition = "bMannedRollAssist", ClampMin = "10.0", ClampMax = "85.0"))
    float MannedBankLimitDeg = 70.f;

    // 뱅크오차 1° → 롤레이트 명령(°/s). 2.0이면 시정수 ~0.5s의 부드러운 캡처.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target",
              meta = (EditCondition = "bMannedRollAssist"))
    float MannedRollKp = 2.0f;

    // 롤레이트 상한(°/s) — 리더 롤 기동의 민첩성 한계 (풀 F-16은 180+).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target",
              meta = (EditCondition = "bMannedRollAssist"))
    float MannedRollRateLimitDps = 120.f;

    // F16CommandController 조종면 부호 미세조정 (PIE에서 리컴파일 없이 뒤집기).
    // 이름은 구 튜닝값(배치된 레벨 액터에 직렬화)과의 호환을 위해 유지한다.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Stick")
    float StickAileronScale  = 1.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Stick")
    float StickElevatorScale = 1.f;

    // ── 인엔진 편대 시험 ──────────────────────────────────────────────────
    // 리더를 스크립트로 조종(이륙→직선→3°/s→4°/s+400m상승→롤아웃)하고, 팔로워의
    // 슬롯오차/뱅크/모드를 Saved/FormationTest.csv 에 기록한 뒤 게이트 판정을 로그로 남긴다.
    // JSBSim 단독 하네스와 달리 플러그인 상태·pawn 매칭·모드 전환·포화를 전부 지나간다.
    // 커맨드라인 -FormationTest 로도 켤 수 있다 (헤드리스 회귀용).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Test")
    bool bRunFormationTest = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Test")
    FString TestLeaderName = TEXT("M_F16");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Test")
    FString TestFollowerName = TEXT("F16_UAV1");

    // true = 시험이 팔로워 setpoint도 주입(ROS/BT 없이 단독 실행). false = BT가 몰게 둠.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Test")
    bool bTestDriveFollower = true;

    // 시험 종료 시 프로세스 종료 (헤드리스 배치 실행용). -FormationTestExit 로도 설정.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Test")
    bool bTestExitOnFinish = false;

    // Read-only state display
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|State")
    float AutopilotAileron  = 0.f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Autopilot|State")
    float AutopilotElevator = 0.f;

    // Cap on how many name-matched pawns the 5005 JSON path drives at once.
    // Must be >= (number of UAVs + the manned M_F16) or the sorted list gets truncated.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    int32 MaxControlledUavs = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString TeamVarName = TEXT("Team");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString LockTargetVarName = TEXT("LockTarget");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString IsLockedVarName = TEXT("IsLocked");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString IsDeadVarName = TEXT("isDead");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString IsFiringBulletVarName = TEXT("isFiringBullet");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString RocketSpawnedIdVarName = TEXT("RocketSpawnedID");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString ShootingSpeedVarName = TEXT("shootingSpeed");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString BulletAmmoVarName = TEXT("BulletAmmo");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|State Mapping")
    FString RocketAmmoVarName = TEXT("RocketAmmo");

    UPROPERTY(BlueprintReadOnly, Category = "UDP|Control")
    float Roll = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "UDP|Control")
    float Pitch = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "UDP|Control")
    float Yaw = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "UDP|Control")
    float Throttle = 0.0f;
};
