#include "UDPControlReceiver.h"

#include "BVRGymAutopilot.h"
#include "Common/UdpSocketBuilder.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "FDMTypes.h"
#include "GameFramework/Pawn.h"
#include "HealthComponent.h"
#include "IPAddress.h"
#include "JSBSimMovementComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "TimerManager.h"
#include "UObject/UnrealType.h"
#include "WeaponComponent.h"
// LAST include: Controller_CY.h pulls BT_Geometry/Math.h which #undef+redefines PI.
// Keeping it last (and this file out of the unity blob while edited) confines that
// to this translation unit; nothing below uses the UE PI macro.
#include "Controller_CY.h"

// Per-UAV surface + speed state. StickController holds internal filters (MF[]/
// ErrorSum) and the throttle PI holds its trim integrator — both must persist
// per aircraft across the 60 Hz ticks, so one instance is kept per name.
struct FUavControl
{
    StickController Stick;   // aim-at-waypoint surfaces (roll/pitch/rudder)
    FPID            Throttle;// speed-hold autothrottle (was BVRGym's ThrottlePID)
};

namespace
{
    constexpr double KnotToMetersPerSecond = 0.514444;

    // Identity name used for state output AND command/setpoint routing.
    // In editor/PIE this is the World Outliner label (e.g. "F16_UAV1"/"M_F16"),
    // which the user sets, instead of the auto object name ("F16_UAV_C_0").
    // Falls back to GetName() outside the editor or when the label is empty.
    FString PawnIdName(const AActor* P)
    {
#if WITH_EDITOR
        if (P)
        {
            const FString Label = P->GetActorLabel(false);
            if (!Label.IsEmpty())
                return Label;
        }
#endif
        return P ? P->GetName() : FString();
    }

    // ETeam → 5006 broadcast string ("manned" / "friendly_uav" / "enemy").
    FString TeamToString(ETeam Team)
    {
        switch (Team)
        {
        case ETeam::Manned:      return TEXT("manned");
        case ETeam::FriendlyUAV: return TEXT("friendly_uav");
        default:                 return TEXT("enemy");
        }
    }
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

    // Per-UAV autopilots are created lazily as setpoints arrive (see AutopilotTick).

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
    ReceiveSetpointData();   // drain JSON setpoint packets (per-UAV)

    const TArray<APawn*> ControlledPawns = FindTargetPawns(ControlledPawnNamePatterns, MaxControlledUavs);
    CachedTargetPawn = ControlledPawns.Num() > 0 ? ControlledPawns[0] : FindTargetPawn();

    for (APawn* Pawn : ControlledPawns)
    {
        // Name-matched commands ONLY. Each pawn responds solely to a command whose aircraft_name is
        // contained in the pawn's instance name (e.g. command "M_F16" -> pawn "M_F16_C_0";
        // command "F16_UAV_C_2" -> that exact UAV). This lets independent senders run simultaneously
        // over the shared topic — joystick -> manned, controller -> UAVs — without the old positional
        // / broadcast fallback cross-applying one vehicle's command to another.
        const FString PawnName = PawnIdName(Pawn);
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
    while (SetpointSocket->HasPendingData(PendingSize))
    {
        TArray<uint8> Data;
        Data.SetNumZeroed(FMath::Min(PendingSize, 65507u) + 1);
        int32 BytesRead = 0;
        FInternetAddr& Sender = *ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

        if (!SetpointSocket->RecvFrom(Data.GetData(), Data.Num() - 1, BytesRead, Sender))
            break;

        Data[BytesRead] = '\0';
        const FString Msg = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Data.GetData()))).Left(BytesRead);

        TSharedPtr<FJsonObject> Root;
        const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Msg);
        if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
            continue;

        // Store one setpoint object into the per-UAV map (keyed by aircraft_name).
        auto StoreOne = [this](const TSharedPtr<FJsonObject>& O)
        {
            FString Name;
            if (!O->TryGetStringField(TEXT("aircraft_name"), Name) || Name.IsEmpty())
                return;

            bool bReset = false;
            if (O->TryGetBoolField(TEXT("reset"), bReset) && bReset)
                UavControls.Remove(Name);   // next tick rebuilds a fresh controller

            FUavSetpoint SP;
            double V;
            if (O->TryGetNumberField(TEXT("heading_deg"),     V)) SP.HeadingDeg     = (float)V;
            if (O->TryGetNumberField(TEXT("altitude_m"),      V)) SP.AltitudeM      = (float)V;
            if (O->TryGetNumberField(TEXT("throttle_norm"),   V)) SP.Throttle       = FMath::Clamp((float)V, 0.f, 1.f);
            if (O->TryGetNumberField(TEXT("target_speed_mps"),V)) SP.TargetSpeedMps = (float)V;
            O->TryGetBoolField(TEXT("launch_missile"), SP.LaunchMissile);
            O->TryGetBoolField(TEXT("gun_firing"), SP.bGunFiring);
            if (O->TryGetNumberField(TEXT("missile_fire_id"), V)) SP.MissileFireId = (int64)V;
            O->TryGetBoolField(TEXT("use_waypoint"), SP.bUseWaypoint);
            if (O->TryGetNumberField(TEXT("target_x"), V)) SP.TargetX = (float)V;
            if (O->TryGetNumberField(TEXT("target_y"), V)) SP.TargetY = (float)V;
            if (O->TryGetNumberField(TEXT("target_z"), V)) SP.TargetZ = (float)V;

            Setpoints.Add(Name, SP);   // latest-wins per aircraft
        };

        if (Root->HasTypedField<EJson::Array>(TEXT("setpoints")))
        {
            for (const TSharedPtr<FJsonValue>& V : Root->GetArrayField(TEXT("setpoints")))
            {
                const TSharedPtr<FJsonObject>* O = nullptr;
                if (V.IsValid() && V->TryGetObject(O) && O && O->IsValid())
                    StoreOne(*O);
            }
        }
        else
        {
            StoreOne(Root);   // single setpoint object
        }
    }
}

// ─── Autopilot: 60 Hz tick (per-UAV) ──────────────────────────────────────────

void AUDPControlReceiver::AutopilotTick()
{
    // Debug: fly the cached target toward a point ahead of its nose (PIE tuning
    // without ROS) — drives the same waypoint/stick path as a real setpoint.
    if (bUseDebugSetpoint && IsValid(CachedTargetPawn))
    {
        FUavSetpoint& SP  = Setpoints.FindOrAdd(PawnIdName(CachedTargetPawn));
        const FVector Loc = CachedTargetPawn->GetActorLocation();
        const FVector Fwd = CachedTargetPawn->GetActorForwardVector();
        const FVector VP  = Loc + Fwd * (DebugForwardM * 100.f) + FVector(0.f, 0.f, DebugUpM * 100.f);
        SP.bUseWaypoint   = true;
        SP.TargetX        = (float)VP.X;
        SP.TargetY        = (float)VP.Y;
        SP.TargetZ        = (float)VP.Z;
        SP.TargetSpeedMps = DebugTargetSpeedMps;
    }

    if (Setpoints.Num() == 0) return;

    UWorld* World = GetWorld();
    if (!World) return;

    // Build the current pawn list once, then drive each aircraft that has a setpoint.
    TArray<AActor*> Pawns;
    UGameplayStatics::GetAllActorsOfClass(World, APawn::StaticClass(), Pawns);

    for (const TPair<FString, FUavSetpoint>& Entry : Setpoints)
    {
        const FString& Key = Entry.Key;
        if (Key.IsEmpty()) continue;

        // 1) Exact name match — preferred and collision-free. With names like
        //    "F16_UAV" and "F16_UAV2" (one a prefix of the other), this resolves
        //    each to the right pawn before any substring logic runs.
        APawn* Match = nullptr;
        for (AActor* A : Pawns)
            if (APawn* P = Cast<APawn>(A))
                if (PawnIdName(P) == Key) { Match = P; break; }

        // 2) Substring fallback (tolerates spawn suffixes, e.g. "M_F16" -> "M_F16_C_0"),
        //    but ONLY when exactly one pawn contains the key. If several match
        //    (key "F16_UAV" would otherwise grab "F16_UAV2"), the name is ambiguous —
        //    skip and warn rather than drive the wrong aircraft.
        if (!Match)
        {
            APawn* Candidate = nullptr;
            int32 NumMatches = 0;
            for (AActor* A : Pawns)
                if (APawn* P = Cast<APawn>(A))
                    if (PawnIdName(P).Contains(Key)) { Candidate = P; ++NumMatches; }

            if (NumMatches == 1)
            {
                Match = Candidate;
            }
            else if (NumMatches > 1)
            {
                UE_LOG(LogTemp, Warning,
                    TEXT("[AP] setpoint name '%s' is ambiguous (%d pawns contain it) — ignored. "
                         "Use the exact pawn name as the aircraft_name."), *Key, NumMatches);
            }
        }

        if (Match)
            ApplyAutopilotToPawn(Match, Key, Entry.Value);
    }
}

void AUDPControlReceiver::ApplyAutopilotToPawn(APawn* Pawn, const FString& Key, const FUavSetpoint& Setpoint)
{
    if (!IsValid(Pawn)) return;

    UJSBSimMovementComponent* JSBSim = FindJSBSimMovementComponent(Pawn);
    if (!JSBSim) return;

    // Dead/falling aircraft keep the hardover surfaces from UHealthComponent —
    // don't fight the crash. Drop the stale controller so any respawn starts fresh.
    if (const UHealthComponent* Health = Pawn->FindComponentByClass<UHealthComponent>())
    {
        if (!Health->IsAlive())
        {
            UavControls.Remove(Key);
            return;
        }
    }

    // Per-UAV stick controller + autothrottle, created on first use — their internal
    // filter/integrator state must persist across ticks (see FUavControl).
    TSharedPtr<FUavControl>& Ctl = UavControls.FindOrAdd(Key);
    if (!Ctl.IsValid()) Ctl = MakeShared<FUavControl>();

    const FAircraftState& S = JSBSim->AircraftState;
    const FVector Loc = Pawn->GetActorLocation();

    // Position: UE world (cm, X=East, Y=South, Z=Up) → Controller_CY (m, North,East,Up).
    // North = -Y/100, East = X/100, Up = Z/100. VP uses the IDENTICAL transform.
    const Vector3 MyLoc(-Loc.Y / 100.0, Loc.X / 100.0, Loc.Z / 100.0);

    // Attitude: JSBSim aero Euler (LocalEulerAngles) in DEGREES → radians.
    // (Roll=phi, Pitch=theta, Yaw=psi with 0=North; NO UE -90 offset — that offset
    // lives only on the visual actor rotator, not on the published LocalEulerAngles.)
    const Vector3 MyRot(
        (double)S.LocalEulerAngles.Roll  * DEG2RAD,
        (double)S.LocalEulerAngles.Pitch * DEG2RAD,
        (double)S.LocalEulerAngles.Yaw   * DEG2RAD);

    const float AltM     = (float)Loc.Z / 100.0f;
    const float SpeedMps = (float)(S.TotalVelocityKts * KnotToMetersPerSecond);

    // Surfaces: aim the nose at the waypoint (StickController) when in waypoint mode.
    // Otherwise leave the surfaces untouched (no BVRGym fallback anymore).
    float Ail = 0.f, Elv = 0.f, Rud = 0.f;
    if (Setpoint.bUseWaypoint)
    {
        const Vector3 VP(-Setpoint.TargetY / 100.0, Setpoint.TargetX / 100.0, Setpoint.TargetZ / 100.0);
        const StickValue SV = Ctl->Stick.GetStick(MyLoc, MyRot, VP);   // Geometry 원본 제어기
        Ail = SV.RollCMD   * StickAileronScale;
        Elv = SV.PitchCMD  * StickElevatorScale;
        Rud = SV.RudderCMD * StickRudderScale;
    }
    // Always write the surfaces: with no waypoint this NEUTRALIZES them (0) rather
    // than freezing the last bank/pitch — a mode change while banked must not spiral.
    // (Dead/falling aircraft already returned above, so their hardover is preserved.)
    JSBSim->Commands.Aileron  = Ail;
    JSBSim->Commands.Elevator = Elv;
    JSBSim->Commands.Rudder   = Rud;

    // Autothrottle (speed-hold PI) — independent of surfaces. FPID error = 0-x, so
    // Update(V - Vtgt) gives error = (Vtgt - V): too slow → positive → throttle up.
    // The FPID instance persists per UAV so its trim integrator carries the steady
    // state throttle. Gains synced each tick without wiping runtime state.
    float ThrottleOut = Setpoint.Throttle;
    if (Setpoint.TargetSpeedMps > 0.f)
    {
        Ctl->Throttle.SetGains(ThrottlePIDConfig);
        ThrottleOut = FMath::Clamp(Ctl->Throttle.Update(SpeedMps - Setpoint.TargetSpeedMps), 0.f, 1.f);
    }
    if (JSBSim->EngineCommands.Num() > 0)
        JSBSim->EngineCommands[0].Throttle = ThrottleOut;

    // Weapon triggers ride along with the setpoint (Phase 3). Missile edge
    // detection lives in UWeaponComponent; only forward ids > 0 so the msg
    // default (0 = never fired) can't be mistaken for a first shot.
    if (UWeaponComponent* Weapon = Pawn->FindComponentByClass<UWeaponComponent>())
    {
        Weapon->SetGunFiring(Setpoint.bGunFiring);
        if (Setpoint.MissileFireId > 0)
            Weapon->ConsumeMissileFireId(Setpoint.MissileFireId);
    }

    // Last-applied (HUD/debug)
    AutopilotAileron  = Ail;
    AutopilotElevator = Elv;

    static int32 LogCounter = 0;
    if (++LogCounter % 60 == 0)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("[Stick] %s wp=%d VP=(%.0f,%.0f,%.0f) | Psi=%.0f Alt=%.0f V=%.0f/%.0f | Ail=%.2f Elv=%.2f Rud=%.2f Thr=%.2f"),
            *Key, Setpoint.bUseWaypoint ? 1 : 0,
            Setpoint.TargetX, Setpoint.TargetY, Setpoint.TargetZ,
            (float)S.LocalEulerAngles.Yaw, AltM, SpeedMps, Setpoint.TargetSpeedMps,
            Ail, Elv, Rud, ThrottleOut);
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
        // Optional weapon triggers (Phase 3) — old senders simply omit them.
        JsonObject->TryGetBoolField(TEXT("gun_firing"), OutCommand.bGunFiring);
        double FireId = 0.0;
        if (JsonObject->TryGetNumberField(TEXT("missile_fire_id"), FireId))
        {
            OutCommand.MissileFireId = static_cast<int64>(FireId);
        }
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

    // Dead/falling aircraft ignore manned commands too (see ApplyAutopilotToPawn).
    if (const UHealthComponent* Health = Pawn->FindComponentByClass<UHealthComponent>())
    {
        if (!Health->IsAlive())
        {
            UavControls.Remove(PawnIdName(Pawn));
            return false;
        }
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
            // Manned speed governor: below the limit full power is available
            // (takeoff/climb feel intact); above it, throttle authority tapers
            // ~10%/(m/s) so top speed settles at the limit. Keeps the leader
            // slower than the UAV cap (measured f16 Vmax 335) so the wingman
            // always has closure margin. 0 = governor off.
            double ThrottleCmd = Command.Throttle;
            if (MannedSpeedLimitMps > 0.f)
            {
                const double SpeedMps = JSBSim->AircraftState.TotalVelocityKts * KnotToMetersPerSecond;
                const double Over = SpeedMps - (double)MannedSpeedLimitMps;
                if (Over > 0.0)
                    ThrottleCmd = FMath::Min(ThrottleCmd, FMath::Max(0.0, 1.0 - Over * 0.1));
            }
            JSBSim->EngineCommands[0].Throttle = ThrottleCmd;
        }
    }

    // Weapon triggers (Phase 3) — same semantics as the autopilot path.
    if (UWeaponComponent* Weapon = Pawn->FindComponentByClass<UWeaponComponent>())
    {
        Weapon->SetGunFiring(Command.bGunFiring);
        if (Command.MissileFireId > 0)
        {
            Weapon->ConsumeMissileFireId(Command.MissileFireId);
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
    PawnJson->SetStringField(TEXT("aircraft_name"), PawnIdName(Pawn));
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

    // Combat state (Phase 3) — omitted when the pawn has no combat components,
    // matching the optional-field convention above. When UHealthComponent exists
    // its ETeam overrides the legacy Blueprint "Team" string set just above.
    if (const UHealthComponent* Health = Pawn->FindComponentByClass<UHealthComponent>())
    {
        PawnJson->SetNumberField(TEXT("hp"), Health->CurrentHP);
        PawnJson->SetNumberField(TEXT("max_hp"), Health->MaxHP);
        PawnJson->SetStringField(TEXT("team"), TeamToString(Health->Team));
        PawnJson->SetBoolField(TEXT("destroyed"), !Health->IsAlive());
    }
    if (const UWeaponComponent* Weapon = Pawn->FindComponentByClass<UWeaponComponent>())
    {
        PawnJson->SetNumberField(TEXT("missile_count"), Weapon->MissileCount);
    }

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
