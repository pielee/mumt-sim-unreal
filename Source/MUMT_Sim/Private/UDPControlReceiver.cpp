#include "UDPControlReceiver.h"

#include "BVRGymAutopilot.h"
#include "Common/UdpSocketBuilder.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "FDMTypes.h"
#include "GameFramework/Pawn.h"
#include "IPAddress.h"
#include "JSBSimMovementComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "TimerManager.h"
#include "UObject/UnrealType.h"

namespace
{
    constexpr float FeetToMeters = 0.3048f;
    constexpr int32 SetpointPacketSize = 17;

    // Binary setpoint packet layout (little-endian, 17 bytes):
    //   [0]     uint8  msg_type  (0x01)
    //   [1..4]  float  heading_deg
    //   [5..8]  float  altitude_m
    //   [9..12] float  throttle_norm
    //   [13]    uint8  launch_missile
    //   [14]    uint8  reset
    //   [15..16] uint16 seq
    struct FSetpointPacket
    {
        float   HeadingDeg   = 0.f;
        float   AltitudeM    = 0.f;
        float   ThrottleNorm = 0.8f;
        bool    Reset        = false;
        uint16  Seq          = 0;
    };

    bool ParseSetpointPacket(const uint8* Buf, int32 Len, FSetpointPacket& Out)
    {
        if (Len < SetpointPacketSize || Buf[0] != 0x01) return false;

        auto R32 = [&](int O) { float V; FMemory::Memcpy(&V, Buf + O, 4); return V; };
        auto R16 = [&](int O) { uint16 V; FMemory::Memcpy(&V, Buf + O, 2); return V; };

        Out.HeadingDeg   = R32(1);
        Out.AltitudeM    = R32(5);
        Out.ThrottleNorm = FMath::Clamp(R32(9), 0.f, 1.f);
        Out.Reset        = Buf[14] != 0;
        Out.Seq          = R16(15);
        return true;
    }
}

namespace
{
    constexpr double KnotToMetersPerSecond = 0.514444;
}

AUDPControlReceiver::AUDPControlReceiver()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AUDPControlReceiver::BeginPlay()
{
    Super::BeginPlay();

    if (StartUDPReceiver())
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Receiver started on port %d"), ListenPort);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] Failed to start receiver"));
    }

    if (StartUDPSender())
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Sender started -> %s:%d"), *PythonIP, PythonStatePort);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] Failed to start sender"));
    }

    if (StartSetpointReceiver())
    {
        UE_LOG(LogTemp, Warning, TEXT("[Autopilot] Setpoint receiver on port %d"), SetpointListenPort);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[Autopilot] Failed to start setpoint receiver on port %d"), SetpointListenPort);
    }

    // Apply initial gain configs to the autopilot
    Autopilot.RollPID    = RollPIDConfig;
    Autopilot.RollSecPID = RollSecPIDConfig;
    Autopilot.PitchPID   = PitchPIDConfig;
    Autopilot.NavParams  = NavParams;

    // 60 Hz autopilot timer
    GetWorldTimerManager().SetTimer(
        AutopilotTimerHandle,
        this, &AUDPControlReceiver::AutopilotTick,
        1.f / 60.f,
        /*bLoop=*/true);

    // State send timer — independent of frame rate
    GetWorldTimerManager().SetTimer(
        StateSendTimerHandle,
        this, &AUDPControlReceiver::SendStateToPython,
        StateSendInterval,
        /*bLoop=*/true);

    CachedTargetPawn = FindTargetPawn();
    if (CachedTargetPawn)
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Primary controlled pawn: %s"), *CachedTargetPawn->GetName());
}

void AUDPControlReceiver::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    ReceiveUDPData();
    ReceiveSetpointData();   // drain binary setpoint packets

    const TArray<APawn*> ControlledPawns = FindTargetPawns(ControlledPawnNamePatterns, MaxControlledUavs);
    CachedTargetPawn = ControlledPawns.Num() > 0 ? ControlledPawns[0] : FindTargetPawn();

    for (APawn* Pawn : ControlledPawns)
    {
        // Name-matched commands ONLY. Each pawn responds solely to a command whose aircraft_name is
        // contained in the pawn's instance name (e.g. command "M_F16" -> pawn "M_F16_C_0";
        // command "F16_UAV_C_2" -> that exact UAV). This lets independent senders run simultaneously
        // over the shared topic — joystick -> manned, controller -> UAVs — without the old positional
        // / broadcast fallback cross-applying one vehicle's command to another.
        const FString PawnName = Pawn->GetName();
        for (const TPair<FString, FRemoteControlCommand>& Entry : NamedControlCommands)
        {
            if (!Entry.Key.IsEmpty() && Entry.Value.bValid && PawnName.Contains(Entry.Key))
            {
                ApplyControlCommandToPawn(Pawn, Entry.Value);
                break;
            }
        }
    }

}

void AUDPControlReceiver::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    GetWorldTimerManager().ClearTimer(AutopilotTimerHandle);
    GetWorldTimerManager().ClearTimer(StateSendTimerHandle);
    StopUDPReceiver();
    StopUDPSender();
    StopSetpointReceiver();
    Super::EndPlay(EndPlayReason);
}

bool AUDPControlReceiver::StartUDPReceiver()
{
    ListenSocket = FUdpSocketBuilder(TEXT("UDP_Control_Receiver"))
        .AsNonBlocking()
        .AsReusable()
        .BoundToPort(ListenPort)
        .WithReceiveBufferSize(2 * 1024 * 1024);

    return ListenSocket != nullptr;
}

void AUDPControlReceiver::StopUDPReceiver()
{
    if (ListenSocket)
    {
        ListenSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenSocket);
        ListenSocket = nullptr;
    }
}

bool AUDPControlReceiver::StartUDPSender()
{
    SendSocket = FUdpSocketBuilder(TEXT("UDP_State_Sender"))
        .AsReusable()
        .WithSendBufferSize(2 * 1024 * 1024);

    if (!SendSocket)
    {
        return false;
    }

    FIPv4Address ParsedIP;
    if (!FIPv4Address::Parse(PythonIP, ParsedIP))
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] Invalid Python IP: %s"), *PythonIP);
        return false;
    }

    PythonAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    PythonAddr->SetIp(ParsedIP.Value);
    PythonAddr->SetPort(PythonStatePort);

    return true;
}

void AUDPControlReceiver::StopUDPSender()
{
    if (SendSocket)
    {
        SendSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SendSocket);
        SendSocket = nullptr;
    }

    PythonAddr.Reset();
}

// ─── Autopilot: setpoint socket ──────────────────────────────────────────────

bool AUDPControlReceiver::StartSetpointReceiver()
{
    SetpointSocket = FUdpSocketBuilder(TEXT("Autopilot_Setpoint_Receiver"))
        .AsNonBlocking()
        .AsReusable()
        .BoundToPort(SetpointListenPort)
        .WithReceiveBufferSize(64 * 1024);

    return SetpointSocket != nullptr;
}

void AUDPControlReceiver::StopSetpointReceiver()
{
    if (SetpointSocket)
    {
        SetpointSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SetpointSocket);
        SetpointSocket = nullptr;
    }
}

void AUDPControlReceiver::ReceiveSetpointData()
{
    if (!SetpointSocket) return;

    uint32 PendingSize = 0;
    uint16 LatestSeq   = 0;
    bool   bGotPacket  = false;

    while (SetpointSocket->HasPendingData(PendingSize))
    {
        uint8 Buf[SetpointPacketSize + 4] = {};
        int32 BytesRead = 0;
        TSharedRef<FInternetAddr> Sender =
            ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

        if (!SetpointSocket->RecvFrom(Buf, sizeof(Buf), BytesRead, *Sender))
            break;

        FSetpointPacket Pkt;
        if (!ParseSetpointPacket(Buf, BytesRead, Pkt)) continue;

        // Keep only the newest packet (wrapping-safe comparison)
        if (!bGotPacket || (uint16)(Pkt.Seq - LatestSeq) < 32768u)
        {
            if (Pkt.Reset)
            {
                Autopilot = FAircraftAutopilot();
                Autopilot.RollPID    = RollPIDConfig;
                Autopilot.RollSecPID = RollSecPIDConfig;
                Autopilot.PitchPID   = PitchPIDConfig;
                Autopilot.NavParams  = NavParams;
            }
            ActiveHeadingDeg  = Pkt.HeadingDeg;
            ActiveAltitudeM   = Pkt.AltitudeM;
            ActiveThrottle    = Pkt.ThrottleNorm;
            LatestSeq         = Pkt.Seq;
            bGotPacket        = true;
            bSetpointReceived = true;
        }
    }
}

// ─── Autopilot: 60 Hz tick ────────────────────────────────────────────────────

void AUDPControlReceiver::AutopilotTick()
{
    // Resolve setpoint: debug UPROPERTY takes priority over UDP
    if (bUseDebugSetpoint)
    {
        ActiveHeadingDeg = DebugTargetHeadingDeg;
        ActiveAltitudeM  = DebugTargetAltitudeM;
        ActiveThrottle   = DebugTargetThrottle;
    }

    // Sync gains in case they were changed live in PIE
    Autopilot.RollPID    = RollPIDConfig;
    Autopilot.RollSecPID = RollSecPIDConfig;
    Autopilot.PitchPID   = PitchPIDConfig;
    Autopilot.NavParams  = NavParams;

    // Apply to the primary controlled pawn (only after first setpoint received)
    if (!bSetpointReceived && !bUseDebugSetpoint) return;
    if (APawn* Pawn = CachedTargetPawn ? CachedTargetPawn : FindTargetPawn())
        ApplyAutopilotToPawn(Pawn);
}

void AUDPControlReceiver::ApplyAutopilotToPawn(APawn* Pawn)
{
    if (!IsValid(Pawn)) return;

    UJSBSimMovementComponent* JSBSim = FindJSBSimMovementComponent(Pawn);
    if (!JSBSim) return;

    // Read flight state
    const FAircraftState& S    = JSBSim->AircraftState;
    const float PhiDeg         = (float)S.LocalEulerAngles.Roll;
    const float ThetaDeg       = (float)S.LocalEulerAngles.Pitch;
    const float PsiDeg         = (float)S.LocalEulerAngles.Yaw;
    const float AltM           = (float)S.AltitudeASLFt * FeetToMeters;

    // Compute errors
    const float DiffHead = FAircraftAutopilot::DeltaHeading(ActiveHeadingDeg, PsiDeg);
    const float DiffAlt  = ActiveAltitudeM - AltM;

    static int32 LogCounter = 0;
    const bool bLogState = (++LogCounter % 10 == 0);  // 6Hz

    // Run outer-loop PID
    const FAutopilotOutput Out = Autopilot.GetControlInput(
        DiffHead, DiffAlt, PhiDeg, ThetaDeg);

    // Write to JSBSim Commands struct — plugin pushes this to FDM every Tick
    JSBSim->Commands.Aileron  = Out.Aileron;
    JSBSim->Commands.Elevator = Out.Elevator;
    JSBSim->Commands.Rudder   = Out.Rudder;

    if (JSBSim->EngineCommands.Num() > 0)
        JSBSim->EngineCommands[0].Throttle = ActiveThrottle;

    // Cache for HUD / Blueprint read
    AutopilotAileron  = Out.Aileron;
    AutopilotElevator = Out.Elevator;

    if (bLogState)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("[AP-STATE] Psi=%.1f Phi=%.1f Theta=%.1f Alt=%.0f | Target(Hdg=%.1f Alt=%.0f) | DiffHead=%.1f DiffAlt=%.0f | Ail=%.3f Elv=%.3f"),
            PsiDeg, PhiDeg, ThetaDeg, AltM,
            ActiveHeadingDeg, ActiveAltitudeM,
            DiffHead, DiffAlt,
            Out.Aileron, Out.Elevator);
    }
}

// ─── Original JSON receiver ───────────────────────────────────────────────────

void AUDPControlReceiver::ReceiveUDPData()
{
    if (!ListenSocket)
    {
        return;
    }

    uint32 PendingDataSize = 0;

    while (ListenSocket->HasPendingData(PendingDataSize))
    {
        TArray<uint8> ReceivedData;
        ReceivedData.SetNumZeroed(FMath::Min(PendingDataSize, 65507u) + 1);

        int32 BytesRead = 0;
        FInternetAddr& SenderAddr = *ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

        if (ListenSocket->RecvFrom(ReceivedData.GetData(), ReceivedData.Num() - 1, BytesRead, SenderAddr))
        {
            ReceivedData[BytesRead] = '\0';

            FString Message = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(ReceivedData.GetData())));
            Message = Message.Left(BytesRead);

            ParseCommand(Message);
        }
    }
}

void AUDPControlReceiver::ParseCommand(const FString& Message)
{
    NamedControlCommands.Reset();
    IndexedControlCommands.Reset();

    if (!ParseJsonCommand(Message))
    {
        ParseLegacyCsvCommand(Message);
    }
}

bool AUDPControlReceiver::ParseJsonCommand(const FString& Message)
{
    TSharedPtr<FJsonObject> RootObject;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Message);
    if (!FJsonSerializer::Deserialize(Reader, RootObject) || !RootObject.IsValid())
    {
        return false;
    }

    auto FillCommand = [](const TSharedPtr<FJsonObject>& JsonObject, FRemoteControlCommand& OutCommand)
    {
        OutCommand.Roll = JsonObject->GetNumberField(TEXT("roll"));
        OutCommand.Pitch = JsonObject->GetNumberField(TEXT("pitch"));
        OutCommand.Yaw = JsonObject->GetNumberField(TEXT("yaw"));
        OutCommand.Throttle = JsonObject->GetNumberField(TEXT("throttle"));
        OutCommand.bValid = true;
    };

    if (RootObject->HasTypedField<EJson::Array>(TEXT("commands")))
    {
        const TArray<TSharedPtr<FJsonValue>>& Commands = RootObject->GetArrayField(TEXT("commands"));
        for (const TSharedPtr<FJsonValue>& CommandValue : Commands)
        {
            const TSharedPtr<FJsonObject>* CommandObject = nullptr;
            if (!CommandValue.IsValid() || !CommandValue->TryGetObject(CommandObject) || !CommandObject || !CommandObject->IsValid())
            {
                continue;
            }

            FRemoteControlCommand ParsedCommand;
            FillCommand(*CommandObject, ParsedCommand);
            IndexedControlCommands.Add(ParsedCommand);

            FString AircraftName;
            if ((*CommandObject)->TryGetStringField(TEXT("aircraft_name"), AircraftName) && !AircraftName.IsEmpty())
            {
                NamedControlCommands.Add(AircraftName, ParsedCommand);
            }
        }
    }

    if (RootObject->HasField(TEXT("roll")) &&
        RootObject->HasField(TEXT("pitch")) &&
        RootObject->HasField(TEXT("yaw")) &&
        RootObject->HasField(TEXT("throttle")))
    {
        FillCommand(RootObject, BroadcastCommand);
    }
    else if (IndexedControlCommands.Num() > 0)
    {
        BroadcastCommand = IndexedControlCommands[0];
    }

    Roll = static_cast<float>(BroadcastCommand.Roll);
    Pitch = static_cast<float>(BroadcastCommand.Pitch);
    Yaw = static_cast<float>(BroadcastCommand.Yaw);
    Throttle = static_cast<float>(BroadcastCommand.Throttle);
    return true;
}

void AUDPControlReceiver::ParseLegacyCsvCommand(const FString& Message)
{
    TArray<FString> Parts;
    Message.ParseIntoArray(Parts, TEXT(","), true);

    if (Parts.Num() != 4)
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] Invalid message format: %s"), *Message);
        BroadcastCommand = FRemoteControlCommand();
        return;
    }

    BroadcastCommand.Roll = FCString::Atof(*Parts[0]);
    BroadcastCommand.Pitch = FCString::Atof(*Parts[1]);
    BroadcastCommand.Yaw = FCString::Atof(*Parts[2]);
    BroadcastCommand.Throttle = FCString::Atof(*Parts[3]);
    BroadcastCommand.bValid = true;

    Roll = static_cast<float>(BroadcastCommand.Roll);
    Pitch = static_cast<float>(BroadcastCommand.Pitch);
    Yaw = static_cast<float>(BroadcastCommand.Yaw);
    Throttle = static_cast<float>(BroadcastCommand.Throttle);
}

APawn* AUDPControlReceiver::FindTargetPawn()
{
    const TArray<APawn*> MatchingPawns = FindTargetPawns(ControlledPawnNamePatterns, 1);
    if (MatchingPawns.Num() > 0)
    {
        return MatchingPawns[0];
    }

    const TArray<FString> FallbackPatterns = { TargetPawnName };
    const TArray<APawn*> FallbackPawns = FindTargetPawns(FallbackPatterns, 1);
    return FallbackPawns.Num() > 0 ? FallbackPawns[0] : nullptr;
}

TArray<APawn*> AUDPControlReceiver::FindTargetPawns(const TArray<FString>& NamePatterns, int32 MaxCount) const
{
    TArray<APawn*> MatchingPawns;

    UWorld* World = GetWorld();
    if (!World)
    {
        return MatchingPawns;
    }

    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(World, APawn::StaticClass(), FoundActors);

    for (AActor* Actor : FoundActors)
    {
        APawn* Pawn = Cast<APawn>(Actor);
        if (DoesPawnMatchPatterns(Pawn, NamePatterns))
        {
            MatchingPawns.Add(Pawn);
        }
    }

    MatchingPawns.Sort([](const APawn& A, const APawn& B)
    {
        return A.GetName() < B.GetName();
    });

    if (MaxCount != INDEX_NONE && MatchingPawns.Num() > MaxCount)
    {
        MatchingPawns.SetNum(MaxCount);
    }

    return MatchingPawns;
}

bool AUDPControlReceiver::DoesPawnMatchPatterns(const APawn* Pawn, const TArray<FString>& NamePatterns) const
{
    if (!IsValid(Pawn))
    {
        return false;
    }

    if (NamePatterns.Num() == 0)
    {
        return TargetPawnName.IsEmpty() || Pawn->GetName().Contains(TargetPawnName);
    }

    for (const FString& Pattern : NamePatterns)
    {
        if (!Pattern.IsEmpty() && Pawn->GetName().Contains(Pattern))
        {
            return true;
        }
    }

    return false;
}

bool AUDPControlReceiver::IsUavPawn(const APawn* Pawn) const
{
    return IsValid(Pawn) && !UavNamePattern.IsEmpty() && Pawn->GetName().Contains(UavNamePattern);
}

bool AUDPControlReceiver::SetBlueprintNumber(APawn* Pawn, const FName VarName, double Value)
{
    if (!Pawn)
    {
        return false;
    }

    FProperty* Prop = Pawn->GetClass()->FindPropertyByName(VarName);
    if (!Prop)
    {
        return false;
    }

    if (FFloatProperty* FloatProp = CastField<FFloatProperty>(Prop))
    {
        FloatProp->SetPropertyValue_InContainer(Pawn, static_cast<float>(Value));
        return true;
    }

    if (FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(Prop))
    {
        DoubleProp->SetPropertyValue_InContainer(Pawn, Value);
        return true;
    }

    return false;
}

bool AUDPControlReceiver::ApplyControlCommandToPawn(APawn* Pawn, const FRemoteControlCommand& Command)
{
    if (!IsValid(Pawn))
    {
        return false;
    }

    // Keep setting the Blueprint variables (for any BP that reads them / for HUD display).
    const bool bRollOk = SetBlueprintNumber(Pawn, TEXT("UDP_Roll"), Command.Roll);
    const bool bPitchOk = SetBlueprintNumber(Pawn, TEXT("UDP_Pitch"), Command.Pitch);
    const bool bYawOk = SetBlueprintNumber(Pawn, TEXT("UDP_Yaw"), Command.Yaw);
    const bool bThrottleOk = SetBlueprintNumber(Pawn, TEXT("UDP_Throttle"), Command.Throttle);

    // Also apply DIRECTLY to the flight model — same path the autopilot uses — so raw joystick
    // control works even when the pawn's Blueprint doesn't forward UDP_* into Commands.
    if (UJSBSimMovementComponent* JSBSim = FindJSBSimMovementComponent(Pawn))
    {
        JSBSim->Commands.Aileron  = Command.Roll;
        JSBSim->Commands.Elevator = Command.Pitch;
        JSBSim->Commands.Rudder   = Command.Yaw;
        if (JSBSim->EngineCommands.Num() > 0)
        {
            JSBSim->EngineCommands[0].Throttle = Command.Throttle;
        }
    }

    return bRollOk && bPitchOk && bYawOk && bThrottleOk;
}

bool AUDPControlReceiver::TryGetBlueprintBool(APawn* Pawn, const FString& VarName, bool& OutValue) const
{
    if (!Pawn || VarName.IsEmpty())
    {
        return false;
    }

    if (FBoolProperty* BoolProp = FindFProperty<FBoolProperty>(Pawn->GetClass(), FName(*VarName)))
    {
        OutValue = BoolProp->GetPropertyValue_InContainer(Pawn);
        return true;
    }

    return false;
}

bool AUDPControlReceiver::TryGetBlueprintInt(APawn* Pawn, const FString& VarName, int32& OutValue) const
{
    if (!Pawn || VarName.IsEmpty())
    {
        return false;
    }

    if (FIntProperty* IntProp = FindFProperty<FIntProperty>(Pawn->GetClass(), FName(*VarName)))
    {
        OutValue = IntProp->GetPropertyValue_InContainer(Pawn);
        return true;
    }

    return false;
}

bool AUDPControlReceiver::TryGetBlueprintNumber(APawn* Pawn, const FString& VarName, double& OutValue) const
{
    if (!Pawn || VarName.IsEmpty())
    {
        return false;
    }

    FProperty* Prop = Pawn->GetClass()->FindPropertyByName(FName(*VarName));
    if (!Prop)
    {
        return false;
    }

    if (const FFloatProperty* FloatProp = CastField<FFloatProperty>(Prop))
    {
        OutValue = FloatProp->GetPropertyValue_InContainer(Pawn);
        return true;
    }

    if (const FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(Prop))
    {
        OutValue = DoubleProp->GetPropertyValue_InContainer(Pawn);
        return true;
    }

    return false;
}

bool AUDPControlReceiver::TryGetBlueprintString(APawn* Pawn, const FString& VarName, FString& OutValue) const
{
    if (!Pawn || VarName.IsEmpty())
    {
        return false;
    }

    if (FStrProperty* StrProp = FindFProperty<FStrProperty>(Pawn->GetClass(), FName(*VarName)))
    {
        OutValue = StrProp->GetPropertyValue_InContainer(Pawn);
        return true;
    }

    if (FNameProperty* NameProp = FindFProperty<FNameProperty>(Pawn->GetClass(), FName(*VarName)))
    {
        OutValue = NameProp->GetPropertyValue_InContainer(Pawn).ToString();
        return true;
    }

    return false;
}

void AUDPControlReceiver::AddOptionalBoolField(const TSharedPtr<FJsonObject>& JsonObject, const FString& JsonKey, APawn* Pawn, const FString& VarName) const
{
    bool Value = false;
    if (TryGetBlueprintBool(Pawn, VarName, Value))
    {
        JsonObject->SetBoolField(JsonKey, Value);
    }
    else
    {
        JsonObject->SetField(JsonKey, MakeShared<FJsonValueNull>());
    }
}

void AUDPControlReceiver::AddOptionalIntField(const TSharedPtr<FJsonObject>& JsonObject, const FString& JsonKey, APawn* Pawn, const FString& VarName) const
{
    int32 Value = 0;
    if (TryGetBlueprintInt(Pawn, VarName, Value))
    {
        JsonObject->SetNumberField(JsonKey, Value);
    }
    else
    {
        JsonObject->SetField(JsonKey, MakeShared<FJsonValueNull>());
    }
}

void AUDPControlReceiver::AddOptionalNumberField(const TSharedPtr<FJsonObject>& JsonObject, const FString& JsonKey, APawn* Pawn, const FString& VarName) const
{
    double Value = 0.0;
    if (TryGetBlueprintNumber(Pawn, VarName, Value))
    {
        JsonObject->SetNumberField(JsonKey, Value);
    }
    else
    {
        JsonObject->SetField(JsonKey, MakeShared<FJsonValueNull>());
    }
}

void AUDPControlReceiver::AddOptionalStringField(const TSharedPtr<FJsonObject>& JsonObject, const FString& JsonKey, APawn* Pawn, const FString& VarName) const
{
    FString Value;
    if (TryGetBlueprintString(Pawn, VarName, Value))
    {
        JsonObject->SetStringField(JsonKey, Value);
    }
    else
    {
        JsonObject->SetField(JsonKey, MakeShared<FJsonValueNull>());
    }
}

UJSBSimMovementComponent* AUDPControlReceiver::FindJSBSimMovementComponent(APawn* Pawn) const
{
    return IsValid(Pawn) ? Pawn->FindComponentByClass<UJSBSimMovementComponent>() : nullptr;
}

TSharedPtr<FJsonObject> AUDPControlReceiver::BuildPawnState(APawn* Pawn)
{
    if (!IsValid(Pawn))
    {
        return nullptr;
    }

    const FVector Location = Pawn->GetActorLocation();
    const FRotator Rotation = Pawn->GetActorRotation();
    UJSBSimMovementComponent* JSBSimComponent = FindJSBSimMovementComponent(Pawn);

    double SpeedKts = 0.0;
    FRotator Attitude = Rotation;
    double ThrottleCommand = 0.0;

    const TSharedPtr<FJsonObject> PawnJson = MakeShared<FJsonObject>();
    PawnJson->SetStringField(TEXT("aircraft_name"), Pawn->GetName());
    PawnJson->SetNumberField(TEXT("x"), Location.X);
    PawnJson->SetNumberField(TEXT("y"), Location.Y);
    PawnJson->SetNumberField(TEXT("z"), Location.Z);

    if (JSBSimComponent)
    {
        const FAircraftState& AircraftState = JSBSimComponent->AircraftState;
        SpeedKts = AircraftState.TotalVelocityKts;
        Attitude = AircraftState.LocalEulerAngles;
        if (JSBSimComponent->EngineCommands.Num() > 0)
        {
            ThrottleCommand = JSBSimComponent->EngineCommands[0].Throttle;
        }
    }

    const double SpeedMps = SpeedKts * KnotToMetersPerSecond;
    PawnJson->SetNumberField(TEXT("speed_mps"), SpeedMps);
    PawnJson->SetNumberField(TEXT("pitch"), Attitude.Pitch);
    PawnJson->SetNumberField(TEXT("roll"), Attitude.Roll);
    PawnJson->SetNumberField(TEXT("yaw"), Attitude.Yaw);
    PawnJson->SetNumberField(TEXT("throttle"), ThrottleCommand);

    AddOptionalStringField(PawnJson, TEXT("team"), Pawn, TeamVarName);

    const TSharedPtr<FJsonObject> WeaponsJson = MakeShared<FJsonObject>();
    AddOptionalIntField(WeaponsJson, TEXT("bullet_ammo"), Pawn, BulletAmmoVarName);
    AddOptionalIntField(WeaponsJson, TEXT("rocket_ammo"), Pawn, RocketAmmoVarName);
    PawnJson->SetObjectField(TEXT("weapons"), WeaponsJson);

    return PawnJson;
}

void AUDPControlReceiver::SendStateToPython()
{
    if (!SendSocket || !PythonAddr.IsValid())
    {
        return;
    }

    TArray<FString> Patterns = ObservedPawnNamePatterns;
    if (Patterns.Num() == 0 && !TargetPawnName.IsEmpty())
    {
        Patterns.Add(TargetPawnName);
    }

    const TArray<APawn*> ObservedPawns = FindTargetPawns(Patterns);
    if (ObservedPawns.Num() == 0)
    {
        return;
    }

    TArray<TSharedPtr<FJsonValue>> AircraftStates;
    AircraftStates.Reserve(ObservedPawns.Num());

    for (APawn* Pawn : ObservedPawns)
    {
        if (TSharedPtr<FJsonObject> PawnState = BuildPawnState(Pawn))
        {
            AircraftStates.Add(MakeShared<FJsonValueObject>(PawnState));
        }
    }

    const TSharedPtr<FJsonObject> RootJson = MakeShared<FJsonObject>();
    RootJson->SetStringField(TEXT("message_type"), TEXT("aircraft_state_batch"));
    RootJson->SetNumberField(TEXT("count"), AircraftStates.Num());
    RootJson->SetArrayField(TEXT("aircraft"), AircraftStates);

    FString JsonMessage;
    const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonMessage);
    FJsonSerializer::Serialize(RootJson.ToSharedRef(), Writer);

    FTCHARToUTF8 Convert(*JsonMessage);
    int32 BytesSent = 0;
    SendSocket->SendTo(
        reinterpret_cast<const uint8*>(Convert.Get()),
        Convert.Length(),
        BytesSent,
        *PythonAddr
    );
}
