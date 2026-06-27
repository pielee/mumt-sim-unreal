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

struct FRemoteControlCommand
{
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 0.0;
    double Throttle = 0.0;
    bool bValid = false;
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
    void ApplyAutopilotToPawn(APawn* Pawn);

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

    // Autopilot state (game-thread only)
    FAircraftAutopilot Autopilot;
    FTimerHandle AutopilotTimerHandle;
    FTimerHandle StateSendTimerHandle;
    float ActiveHeadingDeg  = 0.f;
    float ActiveAltitudeM   = 3000.f;
    float ActiveThrottle    = 0.8f;
    bool  bSetpointReceived = false;  // autopilot inactive until first setpoint arrives

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

    // Enable to override UDP with the debug values below (PIE tuning)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug")
    bool bUseDebugSetpoint = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug",
              meta = (EditCondition = "bUseDebugSetpoint"))
    float DebugTargetHeadingDeg = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug",
              meta = (EditCondition = "bUseDebugSetpoint"))
    float DebugTargetAltitudeM = 3000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Debug",
              meta = (EditCondition = "bUseDebugSetpoint"))
    float DebugTargetThrottle = 0.8f;

    // PID gains — edit live in PIE via Details panel
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Gains")
    FPID RollPIDConfig    = {0.01f, 0.f, 0.9f, -0.2f,  0.2f};

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Gains")
    FPID RollSecPIDConfig = {0.2f,  0.f, 0.2f, -1.0f,  1.0f};

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Gains")
    FPID PitchPIDConfig   = {0.3f,  0.f, 1.0f, -1.0f,  1.0f};

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Autopilot|Nav")
    FAutopilotNavParams NavParams;

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
