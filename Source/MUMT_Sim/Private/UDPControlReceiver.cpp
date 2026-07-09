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
#include "InnerLoopAutopilot.h"   // PID 내루프 (GetStick 조준 제어기 대체)
#include "FormationGuidance.h"    // 인엔진 60Hz 유도 (편대 슬롯 / 추격) — Phase 4

// Per-UAV control state. The inner-loop PIDs hold integrator/derivative state
// that must persist per aircraft across the 60 Hz ticks, so one instance per name.
struct FUavControl
{
    FInnerLoopAutopilot Inner;      // PID 내루프: heading/alt/speed/roll_ff → 조종면+스로틀
    FFormationGuidance  Formation;  // 편대 유도 (리더 ω 필터 상태 보유)
    FPursuitGuidance    Pursuit;    // 추격 유도 (무상태)
    FGuidanceCmd        LastGuidCmd;          // 리더/표적 일시 소실 시 홀드용
    double              LastGuidTime = -1.0;  // World seconds (마지막 유효 유도 계산 시각)
    bool                bHasGuidCmd  = false;
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

    // Pawn 이름 매칭 (setpoint 라우팅과 동일 규칙): 정확 일치 우선, 아니면
    // 부분 문자열이 정확히 1개일 때만 채택. OutNumSubstring은 모호성 경고용.
    APawn* MatchPawnByKey(const TArray<AActor*>& Pawns, const FString& Key,
                          int32* OutNumSubstring = nullptr)
    {
        if (OutNumSubstring) *OutNumSubstring = 0;
        if (Key.IsEmpty()) return nullptr;

        for (AActor* A : Pawns)
            if (APawn* P = Cast<APawn>(A))
                if (PawnIdName(P) == Key)
                    return P;

        APawn* Candidate = nullptr;
        int32 Num = 0;
        for (AActor* A : Pawns)
            if (APawn* P = Cast<APawn>(A))
                if (PawnIdName(P).Contains(Key)) { Candidate = P; ++Num; }

        if (OutNumSubstring) *OutNumSubstring = Num;
        return (Num == 1) ? Candidate : nullptr;
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
            if (O->TryGetNumberField(TEXT("roll_ff_deg"),     V)) SP.RollFfDeg      = (float)V;
            if (O->TryGetNumberField(TEXT("throttle_norm"),   V)) SP.Throttle       = FMath::Clamp((float)V, 0.f, 1.f);
            if (O->TryGetNumberField(TEXT("target_speed_mps"),V)) SP.TargetSpeedMps = (float)V;
            O->TryGetBoolField(TEXT("launch_missile"), SP.LaunchMissile);
            O->TryGetBoolField(TEXT("gun_firing"), SP.bGunFiring);
            if (O->TryGetNumberField(TEXT("missile_fire_id"), V)) SP.MissileFireId = (int64)V;
            O->TryGetBoolField(TEXT("use_waypoint"), SP.bUseWaypoint);
            if (O->TryGetNumberField(TEXT("target_x"), V)) SP.TargetX = (float)V;
            if (O->TryGetNumberField(TEXT("target_y"), V)) SP.TargetY = (float)V;
            if (O->TryGetNumberField(TEXT("target_z"), V)) SP.TargetZ = (float)V;

            // ── 유도 모드 (Phase 4). 미지정/""/direct → Direct (구 발신자 호환) ──
            FString ModeStr;
            if (O->TryGetStringField(TEXT("guidance_mode"), ModeStr))
            {
                if (ModeStr.Equals(TEXT("formation"), ESearchCase::IgnoreCase))
                    SP.Mode = EGuidanceMode::Formation;
                else if (ModeStr.Equals(TEXT("attack"), ESearchCase::IgnoreCase))
                    SP.Mode = EGuidanceMode::Attack;
            }
            O->TryGetStringField(TEXT("leader_name"), SP.LeaderName);
            O->TryGetStringField(TEXT("target_name"), SP.TargetName);
            if (O->TryGetNumberField(TEXT("slot_front_m"),  V)) SP.SlotFrontM = (float)V;
            if (O->TryGetNumberField(TEXT("slot_right_m"),  V)) SP.SlotRightM = (float)V;
            if (O->TryGetNumberField(TEXT("slot_up_m"),     V)) SP.SlotUpM    = (float)V;
            if (O->TryGetNumberField(TEXT("min_speed_mps"), V) && V > 0.0) SP.MinSpeedMps = (float)V;
            if (O->TryGetNumberField(TEXT("max_speed_mps"), V) && V > 0.0) SP.MaxSpeedMps = (float)V;
            if (O->TryGetNumberField(TEXT("min_alt_m"),     V)) SP.MinAltM = (float)V;

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

        // 정확 일치 우선 → 부분 문자열이 정확히 1개일 때만 채택 (MatchPawnByKey).
        // 여러 pawn이 매칭되면 모호 — 잘못된 기체를 조종하느니 건너뛰고 경고.
        int32 NumSubstring = 0;
        APawn* Match = MatchPawnByKey(Pawns, Key, &NumSubstring);
        if (!Match && NumSubstring > 1)
        {
            UE_LOG(LogTemp, Warning,
                TEXT("[AP] setpoint name '%s' is ambiguous (%d pawns contain it) — ignored. "
                     "Use the exact pawn name as the aircraft_name."), *Key, NumSubstring);
        }

        if (Match)
            ApplyAutopilotToPawn(Match, Key, Entry.Value, Pawns);
    }
}

void AUDPControlReceiver::ApplyAutopilotToPawn(APawn* Pawn, const FString& Key, const FUavSetpoint& Setpoint,
                                               const TArray<AActor*>& Pawns)
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

    // 현재 상태: JSBSim aero 오일러(LocalEulerAngles, deg, ψ 0=북), UE-Z 고도.
    const double PhiDeg   = S.LocalEulerAngles.Roll;
    const double ThetaDeg = S.LocalEulerAngles.Pitch;
    const double PsiDeg   = S.LocalEulerAngles.Yaw;
    const float  AltM     = (float)(Pawn->GetActorLocation().Z / 100.0);
    const float  SpeedMps = (float)(S.TotalVelocityKts * KnotToMetersPerSecond);
    const double ClimbMps = S.AltitudeRateFtps * 0.3048;   // +위
    const double PDps     = S.EulerRates.Z;                 // 롤 레이트 ≈ p (deg/s)
    const double QDps     = S.EulerRates.Y;                 // 피치 레이트 ≈ q (deg/s)

    // ── 유도 명령 결정 (Phase 4) ─────────────────────────────────────────────
    // Direct: setpoint의 heading/alt/speed/roll_ff 그대로 (기존 경로, 완전 호환).
    // Formation/Attack: 리더/표적 pawn을 같은 월드에서 직독(지연 0)해 60Hz로 계산.
    double HeadingCmd = Setpoint.HeadingDeg;
    double AltCmd     = Setpoint.AltitudeM;
    double SpeedCmd   = Setpoint.TargetSpeedMps;
    double RollFf     = Setpoint.RollFfDeg;

    if (Setpoint.Mode != EGuidanceMode::Direct)
    {
        // 편대/추격은 리더보다 선회 여유가 있어야 컷인 가능: f16 요-SAS(워시아웃 없음)가
        // 정상선회를 방해해 실효 선회율이 이론보다 낮다(V1 실측 +9° 뱅크 필요) → 캡 70°.
        Ctl->Inner.BankLimitDeg = 70.0;

        // 자기 상태 (NED m / m/s): north=-Y/100, east=X/100. 속도는 참벡터(ft/s→m/s).
        const FVector OwnLoc = Pawn->GetActorLocation();
        const double OwnN  = -OwnLoc.Y * 0.01;
        const double OwnE  =  OwnLoc.X * 0.01;
        const double OwnVn = S.VelocityNEDfps.X * 0.3048;
        const double OwnVe = S.VelocityNEDfps.Y * 0.3048;

        const FString& RefName = (Setpoint.Mode == EGuidanceMode::Formation)
            ? Setpoint.LeaderName : Setpoint.TargetName;
        APawn* RefPawn = MatchPawnByKey(Pawns, RefName);
        UJSBSimMovementComponent* RefJSB = RefPawn ? FindJSBSimMovementComponent(RefPawn) : nullptr;

        bool bRefAlive = true;
        if (RefPawn)
            if (const UHealthComponent* RefHealth = RefPawn->FindComponentByClass<UHealthComponent>())
                bRefAlive = RefHealth->IsAlive();

        const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;

        if (RefJSB && bRefAlive)
        {
            const FAircraftState& R = RefJSB->AircraftState;
            const FVector RefLoc = RefPawn->GetActorLocation();
            const double RefN   = -RefLoc.Y * 0.01;
            const double RefE   =  RefLoc.X * 0.01;
            const double RefAlt =  RefLoc.Z * 0.01;
            const double RefVn    = R.VelocityNEDfps.X * 0.3048;
            const double RefVe    = R.VelocityNEDfps.Y * 0.3048;
            const double RefClimb = R.VelocityNEDfps.Z * 0.3048;  // 플러그인이 -vDown 저장 → +위

            // 1초 이상 유도가 끊겼다 재개되면 ω 미분 상태가 낡음 → 리셋(범프리스)
            if (Setpoint.Mode == EGuidanceMode::Formation && Now - Ctl->LastGuidTime > 1.0)
                Ctl->Formation.Reset();

            FGuidanceCmd Cmd;
            if (Setpoint.Mode == EGuidanceMode::Formation)
            {
                Cmd = Ctl->Formation.Step(
                    RefN, RefE, RefAlt, RefVn, RefVe, RefClimb,
                    OwnN, OwnE, OwnVn, OwnVe,
                    Setpoint.SlotFrontM, Setpoint.SlotRightM, Setpoint.SlotUpM,
                    Setpoint.MinSpeedMps, Setpoint.MaxSpeedMps, Setpoint.MinAltM,
                    /*dt=*/1.0 / 60.0);
            }
            else // Attack
            {
                Cmd = Ctl->Pursuit.Step(
                    RefN, RefE, RefAlt, RefVn, RefVe,
                    OwnN, OwnE,
                    Setpoint.MinSpeedMps, Setpoint.MaxSpeedMps, Setpoint.MinAltM);
            }
            Ctl->LastGuidCmd  = Cmd;
            Ctl->LastGuidTime = Now;
            Ctl->bHasGuidCmd  = true;
            HeadingCmd = Cmd.HeadingDeg; AltCmd = Cmd.AltM;
            SpeedCmd   = Cmd.SpeedMps;   RollFf = Cmd.RollFfDeg;
        }
        else if (Ctl->bHasGuidCmd && Now - Ctl->LastGuidTime < 5.0)
        {
            // 리더/표적 일시 소실 → 마지막 유도 명령 홀드
            HeadingCmd = Ctl->LastGuidCmd.HeadingDeg; AltCmd = Ctl->LastGuidCmd.AltM;
            SpeedCmd   = Ctl->LastGuidCmd.SpeedMps;   RollFf = Ctl->LastGuidCmd.RollFfDeg;
        }
        else
        {
            // 5초 초과 소실 → 현재 헤딩·고도 수평 유지 (순항 200)
            HeadingCmd = PsiDeg;
            AltCmd     = (double)AltM;
            SpeedCmd   = 200.0;
            RollFf     = 0.0;
        }
    }

    // 조종면 부호 미세조정(PIE 튜닝) 반영.
    Ctl->Inner.AilSign = StickAileronScale;
    Ctl->Inner.ElvSign = StickElevatorScale;

    // PID 내루프: (heading, altitude, speed, roll_ff) → 조종면 + 스로틀.
    const FInnerLoopOutput O = Ctl->Inner.Step(
        HeadingCmd, AltCmd, SpeedCmd, RollFf,
        PhiDeg, ThetaDeg, PsiDeg, (double)AltM, (double)SpeedMps, ClimbMps, PDps, QDps,
        (double)Setpoint.Throttle, /*dt=*/1.0 / 60.0);

    const float Ail = O.Aileron, Elv = O.Elevator, Rud = O.Rudder, ThrottleOut = O.Throttle;
    JSBSim->Commands.Aileron    = Ail;
    JSBSim->Commands.Elevator   = Elv;
    JSBSim->Commands.Rudder     = Rud;
    JSBSim->Commands.SpeedBrake = O.SpeedBrake;   // 과속 시 감속 보조 (리더 급감속 추종)
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
        const TCHAR* ModeTag =
            (Setpoint.Mode == EGuidanceMode::Formation) ? TEXT("FORM") :
            (Setpoint.Mode == EGuidanceMode::Attack)    ? TEXT("ATK")  : TEXT("DIR");
        UE_LOG(LogTemp, Warning,
            TEXT("[Inner] %s [%s] -> Hdg=%.0f Alt=%.0f V=%.0f Rff=%.1f | Psi=%.0f Phi=%.0f Alt=%.0f V=%.0f | Ail=%.2f Elv=%.2f Thr=%.2f"),
            *Key, ModeTag, (float)HeadingCmd, (float)AltCmd, (float)SpeedCmd, (float)RollFf,
            (float)PsiDeg, (float)PhiDeg, AltM, SpeedMps,
            Ail, Elv, ThrottleOut);
        if (Setpoint.Mode == EGuidanceMode::Formation)
            UE_LOG(LogTemp, Warning, TEXT("[Guid] %s along=%+.0f cross=%+.0f omega=%+.2fdps"),
                *Key, (float)Ctl->Formation.LastEAlongM, (float)Ctl->Formation.LastECrossM,
                (float)(Ctl->Formation.OmegaFilt * 57.29577951));
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
        // Scale surface authority so full stick stays gentle (formation-followable).
        const double Auth = FMath::Clamp((double)MannedControlAuthority, 0.05, 1.0);
        JSBSim->Commands.Aileron  = Command.Roll  * Auth;
        JSBSim->Commands.Elevator = Command.Pitch * Auth;
        JSBSim->Commands.Rudder   = Command.Yaw   * Auth;
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
