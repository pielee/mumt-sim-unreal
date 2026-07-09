#pragma once

#include "CoreMinimal.h"
#include "BVRGymAutopilot.h"
#include "Dom/JsonObject.h"
#include "GameFramework/Actor.h"
#include "IPAddress.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "UDPControlReceiver.generated.h"

class APawn;
class UJSBSimMovementComponent;
// Per-UAV stick-controller + autothrottle state; defined in the .cpp so
// Controller_CY.h (which redefines PI via BT_Geometry) stays out of this header.
struct FUavControl;

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

// 유도 모드: BT는 모드·대상만 지정하고 heading/alt/speed 숫자 계산은 UE가 60Hz로 수행.
// Direct = 기존 방식(setpoint의 heading/alt/speed 그대로) — 구 시나리오 완전 호환.
enum class EGuidanceMode : uint8
{
    Direct,      // ""/"direct"
    Formation,   // leader_name pawn 직독 → 슬롯+ω×r+벡터필드+roll_ff (FFormationGuidance)
    Attack,      // target_name pawn 직독 → pursuit heading/alt/speed (FPursuitGuidance)
};

// High-level autopilot setpoint for one UAV (heading/altitude/speed-or-throttle).
struct FUavSetpoint
{
    float HeadingDeg     = 0.f;   // 목표 헤딩(0=북) → 내루프 roll_ref
    float AltitudeM      = 0.f;   // 목표 고도(UE-Z, m) → 내루프 theta_ref
    float RollFfDeg      = 0.f;   // 뱅크 피드포워드(deg) — 편대 유도의 선회 지연 보상
    float Throttle       = 0.8f;  // used only when TargetSpeedMps <= 0 (open-loop)
    float TargetSpeedMps = 0.f;   // >0 → autothrottle holds this airspeed
    // ── 유도 모드 (Phase 4: 인엔진 60Hz 유도) ──
    EGuidanceMode Mode = EGuidanceMode::Direct;
    FString LeaderName;           // formation: 리더 pawn 이름 (setpoint 키와 동일 매칭 규칙)
    FString TargetName;           // attack: 표적 pawn 이름
    float SlotFrontM  = -80.f;    // 슬롯 오프셋 — 리더 트랙 프레임 (+앞/+우/+상)
    float SlotRightM  = 100.f;
    float SlotUpM     = 0.f;
    float MinSpeedMps = 70.f;     // 유도 속도 한계
    float MaxSpeedMps = 335.f;
    float MinAltM     = 0.f;      // 유도 고도 하한(UE-Z m). 0 = 가드 없음
    bool  LaunchMissile  = false; // deprecated — use MissileFireId
    // Weapon triggers (Phase 3) — optional fields, keep defaults for old senders.
    bool  bGunFiring     = false; // level-triggered: fire while true
    int64 MissileFireId  = 0;     // edge-triggered: one shot per new id (0 = never fired)
    // Waypoint control (StickController). Target point in UE world coords (cm),
    // same frame as BuildPawnState x/y/z. UE converts to N/E/Up meters for GetStick.
    bool  bUseWaypoint   = false; // true → fly to Target* via stick controller
    float TargetX        = 0.f;   // UE world X (East, cm)
    float TargetY        = 0.f;   // UE world Y (South, cm)
    float TargetZ        = 0.f;   // UE world Z (Up, cm)
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
    // their own PID controller instance (so their control state never mixes).
    TMap<FString, FUavSetpoint>            Setpoints;    // aircraft name -> latest setpoint
    TMap<FString, TSharedPtr<FUavControl>> UavControls;  // aircraft name -> stick controller + autothrottle
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

    // ── BVRGym Autopilot ─────────────────────────────────────────────────

    // Port the bridge sends binary setpoint packets to (msg_type 0x01)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|UDP")
    int32 SetpointListenPort = 5010;

    // Enable to fly the cached target toward a point ahead of its nose, driving the
    // same waypoint/stick path as a real setpoint (PIE tuning without ROS).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug")
    bool bUseDebugSetpoint = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug",
              meta = (EditCondition = "bUseDebugSetpoint"))
    float DebugForwardM = 3000.f;      // look-at point this far ahead of the nose

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug",
              meta = (EditCondition = "bUseDebugSetpoint"))
    float DebugUpM = 1000.f;           // ...and this far above current altitude

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug",
              meta = (EditCondition = "bUseDebugSetpoint"))
    float DebugTargetSpeedMps = 220.f; // autothrottle target for the debug flight

    // Autothrottle (speed-hold). Output is throttle [0,1]; integrator carries the
    // trim throttle, so Ki*IntegMax should be ≈ 1. Active only when a setpoint
    // provides target_speed_mps > 0. (Surface control is now StickController;
    // BVRGym's surface PIDs/NavParams were removed.)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Gains")
    FPID ThrottlePIDConfig = {0.02f, 0.004f, 0.f, 0.f, 250.f};

    // Manned top-speed governor (m/s). Full power below the limit; throttle
    // authority tapers above it (soft FBW-style speed protection). Default 300
    // leaves the formation UAV (cap 335 = measured f16 Vmax) ~35 m/s closure
    // margin so it can always catch the leader. 0 = off.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    float MannedSpeedLimitMps = 300.f;

    // Manned control authority: scales the joystick surface commands (roll/pitch/
    // yaw, [-1,1]) before they hit the flight model. 0.4 = full stick gives only
    // 40% surface, so even a hard input is gentle enough for the formation UAV to
    // follow. 1.0 = full authority (raw). Throttle is NOT scaled (governor handles
    // top speed). Manned/joystick path only — UAV autopilot is unaffected.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target",
              meta = (ClampMin = "0.05", ClampMax = "1.0"))
    float MannedControlAuthority = 0.4f;

    // StickController output → JSBSim surface sign/scale. Verify in PIE and flip a
    // sign here (no recompile) if a surface is inverted. BVRGym left rudder at 0;
    // StickController drives an active rudder, so its sign is unverified in-sim.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Stick")
    float StickAileronScale  = 1.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Stick")
    float StickElevatorScale = 1.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Stick")
    float StickRudderScale   = 1.f;

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
