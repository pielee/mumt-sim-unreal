#include "UDPControlReceiver.h"

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
// 3계층 무인기 제어 스택 — 최종 JSBSim 명령을 쓰는 무인기 제어기는 이 체인 하나뿐이다.
#include "FormationGuidance.h"     // [1] 편대 슬롯·오차 (리더 직독 60Hz)
#include "FixedWingGuidance.h"     // [2] 비행 Reference (벡터필드 횡 / 고도·속도 종) + 추격
#include "F16CommandController.h"  // [3] Reference → fcs/*-cmd-norm
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/PlatformMisc.h"

// Per-UAV control state — 3계층 스택 인스턴스. PID 적분기·필터·슬루 상태가 기체별로
// 60Hz 틱을 넘어 유지돼야 하므로 이름당 1개.
struct FUavControl
{
    FFormationGuidance    Formation;   // [1] 슬롯·오차 (리더 ω 필터·캡처 판정 상태 보유)
    FFixedWingGuidance    Guidance;    // [2] Reference (REJOIN·roll_ref 슬루 상태 보유)
    FF16CommandController Controller;  // [3] cmd-norm (PID 적분기 보유)
    FPursuitGuidance      Pursuit;     // 추격 유도 (무상태 — StepDirect로 유입)
    FDirectCmd            LastGuidCmd;           // 리더/표적 일시 소실 시 홀드용
    double                LastGuidTime = -1.0;   // World seconds (마지막 유효 유도 계산 시각)
    bool                  bHasGuidCmd  = false;
    // 모드/리더 변경 감지 [§9] — 변경 '시점에만' 리셋 (같은 모드·리더 유지 중 매 틱 리셋 금지)
    EGuidanceMode         LastMode = EGuidanceMode::Direct;
    FString               LastLeaderName;
    bool                  bHasLastMode = false;
};

// 마지막 유도 계산 결과 — 5006 상태 배치의 "guidance" 오브젝트로 BT에 피드백된다.
// (BT의 CheckFormationCaptured/CheckFormationMaintained가 소비 — BT는 오차를 재계산하지 않는다.)
struct FUavGuidanceStatus
{
    EGuidanceMode Mode = EGuidanceMode::Direct;
    double EAlongM = 0.0, ECrossM = 0.0, EVertM = 0.0;
    double ClosingMps = 0.0, SeparationM = 0.0, SlotDistM = 0.0;
    bool   bCaptured = false, bMaintained = false, bRejoin = false, bSepBreach = false;
    bool   bSepWarning = false;   // 감속 보조가 하한에 걸렸는데 거리 계속 감소 [§7]
    uint32 SeqId = 0;
    double RollRefDeg = 0.0, PitchRefDeg = 0.0, AirspeedRefMps = 0.0;
};

// ─── 인엔진 편대 시험 ────────────────────────────────────────────────────────
// 리더를 스크립트 direct 명령으로 몰고(이 경로도 내루프를 통과), 팔로워는 BT와 동일한
// 게이팅(자기 +150m ∧ 리더 +80m)으로 formation 모드에 진입시킨다. 유도는 리더의
// '실제' 상태(VelocityNEDfps)를 읽으므로 리더가 명령을 덜 따라가도 시험은 유효하다.
struct FFormationTest
{
    enum class EPhase : uint8 { Takeoff = 0, Straight, Turn3, Turn4Climb, Rollout, Breakaway, FarChase, Done };

    static constexpr double kDt        = 1.0 / 60.0;
    static constexpr double kRunwayHdg = 90.0;

    struct FStat
    {
        double MaxE = 0.0, SumE = 0.0, MaxPhi = 0.0;
        int32  N = 0;
        void Add(double E, double Phi)
        {
            MaxE = FMath::Max(MaxE, E); SumE += E; ++N;
            MaxPhi = FMath::Max(MaxPhi, FMath::Abs(Phi));
        }
        double Mean() const { return N ? SumE / N : 0.0; }
    };

    // 구간: 이름, 지속(s), 정착유예(s, 이 시간 전은 과도로 보고 게이트 제외), 슬롯오차 게이트(m).
    // Straight는 초기 합류(캡처)를 포함하므로 유예가 길다 — 캡처 자체는 SettleTime으로 별도 측정.
    struct FPhaseDef { const TCHAR* Name; double Dur; double SettleS; double GateM; };
    static const FPhaseDef& Def(EPhase P)
    {
        static const FPhaseDef Defs[] = {
            { TEXT("Takeoff"),          0.0,  0.0, 1e9 },   // 리더 +200m 까지
            { TEXT("Straight 220"),    55.0, 30.0,  30.0 },
            { TEXT("Turn 3dps@220"),   30.0,  6.0,  60.0 },
            { TEXT("Turn 4dps+400m"),  40.0,  6.0, 120.0 },
            { TEXT("Rollout 170"),     25.0,  6.0,  70.0 },  // 감속 과도 실측 59~62m (실행별 지터) + 여유
            // F3 (2026-07-10): 팔로워를 대각선으로 20s 이탈시켜(공격임무 이탈 모사) km급
            // 변위를 만들고 REJOIN 재합류를 검증. 리더 선회로는 변위가 안 생김 — 팔로워가
            // 5°/s 180° 선회를 44m로 그냥 따라옴(1차 실행 실측). 벡터필드 단독은 km급에서
            // ±4km S-위빙(PIE 실측) — 재발 방지 게이트.
            { TEXT("Detour(이탈주입)"), 20.0,  0.0, 1e9 },   // 팔로워 direct 대각 이탈 (무게이트)
            { TEXT("FarChase rejoin"),100.0, 80.0,  40.0 },  // 재합류 물리 소요 ~70s (선회25+추격30+정착)
            { TEXT("Done"),             0.0,  0.0, 1e9 },
        };
        return Defs[(int32)P];
    }

    EPhase Phase = EPhase::Takeoff;
    double PhaseT = 0.0, TotalT = 0.0;
    double LdrSpawnAlt = -1e9, FolSpawnAlt = -1e9;
    double CruiseAlt = 0.0, HdgCmd = kRunwayHdg;
    bool   bFolFormation = false, bWarned = false;
    double MaxEAfterForm = 0.0;      // 발산 가드 (Rollout까지 — Breakaway는 의도적 이탈)
    double FormEntryT = -1.0, SettleT = -1.0;   // 합류 시각 / 슬롯 30m 이내 정착 시각
    double BreakawayPeakM = 0.0;     // 이탈 최대 슬롯거리 (정보)
    double RecaptureT = -1.0;        // FarChase 시작→슬롯 30m 재진입 소요(s)
    FStat  Stat[8];
    double CsvAccum = 0.0;
    TArray<FString> Csv;

    void Advance() { PhaseT += kDt; TotalT += kDt; }
    void Enter(EPhase P) { Phase = P; PhaseT = 0.0; }
    bool InGate() const { return PhaseT >= Def(Phase).SettleS; }
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

    // 헤드리스 회귀: -FormationTest [-FormationTestExit] 로 에디터 설정 없이 시험 실행
    if (FParse::Param(FCommandLine::Get(), TEXT("FormationTest")))
    {
        bRunFormationTest = true;
        if (FParse::Param(FCommandLine::Get(), TEXT("FormationTestExit")))
            bTestExitOnFinish = true;
        UE_LOG(LogTemp, Warning, TEXT("[FormTest] 커맨드라인으로 활성화 (leader='%s' follower='%s' exit=%d)"),
            *TestLeaderName, *TestFollowerName, bTestExitOnFinish ? 1 : 0);
    }

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
            {
                UavControls.Remove(Name);   // next tick rebuilds a fresh controller
                LastSetpointSeq.Remove(Name);
            }

            // ── 수치 검증: NaN/Inf가 하나라도 있으면 패킷 전체 폐기 ──
            bool bFinite = true;
            auto GetNum = [&O, &bFinite](const TCHAR* Field, double& Out) -> bool
            {
                double V = 0.0;
                if (!O->TryGetNumberField(Field, V)) return false;
                if (!FMath::IsFinite(V)) { bFinite = false; return false; }
                Out = V;
                return true;
            };

            FUavSetpoint SP;
            double V;
            if (GetNum(TEXT("heading_deg"),     V)) SP.HeadingDeg     = (float)V;
            if (GetNum(TEXT("altitude_m"),      V)) SP.AltitudeM      = (float)V;
            if (GetNum(TEXT("roll_ff_deg"),     V)) SP.RollFfDeg      = (float)V;
            if (GetNum(TEXT("throttle_norm"),   V)) SP.Throttle       = FMath::Clamp((float)V, 0.f, 1.f);
            if (GetNum(TEXT("target_speed_mps"),V)) SP.TargetSpeedMps = (float)V;
            O->TryGetBoolField(TEXT("launch_missile"), SP.LaunchMissile);
            O->TryGetBoolField(TEXT("gun_firing"), SP.bGunFiring);
            if (GetNum(TEXT("missile_fire_id"), V)) SP.MissileFireId = (int64)V;

            // ── 유도 모드. 미지정/""/direct → Direct (구 발신자 호환) ──
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
            if (GetNum(TEXT("slot_front_m"),  V)) SP.SlotFrontM = (float)V;
            if (GetNum(TEXT("slot_right_m"),  V)) SP.SlotRightM = (float)V;
            if (GetNum(TEXT("slot_up_m"),     V)) SP.SlotUpM    = (float)V;
            if (GetNum(TEXT("min_speed_mps"), V) && V > 0.0) SP.MinSpeedMps = (float)V;
            if (GetNum(TEXT("max_speed_mps"), V) && V > 0.0) SP.MaxSpeedMps = (float)V;
            if (GetNum(TEXT("min_alt_m"),     V)) SP.MinAltM = (float)V;

            // ── 편대 판정·안전 한계 (프로토콜 v2 — 0/미지정이면 UE 기본값) ──
            if (GetNum(TEXT("capture_tolerance_m"),        V) && V > 0.0) SP.CaptureTolM    = (float)V;
            if (GetNum(TEXT("maintain_tolerance_m"),       V) && V > 0.0) SP.MaintainTolM   = (float)V;
            if (GetNum(TEXT("minimum_separation_m"),       V) && V > 0.0) SP.MinSeparationM = (float)V;
            if (GetNum(TEXT("maximum_closing_speed_mps"),  V) && V > 0.0) SP.MaxClosingMps  = (float)V;
            if (GetNum(TEXT("sequence_id"),      V)) SP.SequenceId      = (uint32)FMath::Max(0.0, V);
            if (GetNum(TEXT("timestamp"),        V)) SP.TimestampS      = V;
            if (GetNum(TEXT("protocol_version"), V)) SP.ProtocolVersion = (uint8)FMath::Clamp(V, 0.0, 255.0);

            if (!bFinite)
            {
                UE_LOG(LogTemp, Warning, TEXT("[AP] '%s' setpoint에 NaN/Inf — 패킷 폐기"), *Name);
                return;
            }

            // ── 낡은 sequence_id 폐기 (seq=0은 구 발신자 — 검사 생략) ──
            if (SP.SequenceId > 0)
            {
                const uint32* Last = LastSetpointSeq.Find(Name);
                if (Last && SP.SequenceId <= *Last)
                    return;   // out-of-order/중복 UDP — 최신 명령 유지
                LastSetpointSeq.Add(Name, SP.SequenceId);
            }

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
    // Debug: direct 모드 setpoint 주입 (ROS 없이 PIE에서 reference 추종 튜닝).
    // 현재 헤딩 유지 + DebugUpM 상승 + DebugTargetSpeedMps 유지.
    if (bUseDebugSetpoint && IsValid(CachedTargetPawn))
    {
        FUavSetpoint& SP = Setpoints.FindOrAdd(PawnIdName(CachedTargetPawn));
        if (UJSBSimMovementComponent* DbgJSB = FindJSBSimMovementComponent(CachedTargetPawn))
            SP.HeadingDeg = (float)DbgJSB->AircraftState.LocalEulerAngles.Yaw;
        SP.AltitudeM      = (float)(CachedTargetPawn->GetActorLocation().Z / 100.0) + DebugUpM;
        SP.TargetSpeedMps = DebugTargetSpeedMps;
        SP.Mode           = EGuidanceMode::Direct;
    }

    UWorld* World = GetWorld();
    if (!World) return;

    // Build the current pawn list once, then drive each aircraft that has a setpoint.
    TArray<AActor*> Pawns;
    UGameplayStatics::GetAllActorsOfClass(World, APawn::StaticClass(), Pawns);

    // 인엔진 편대 시험: 리더를 스크립트로 몰고 팔로워 슬롯오차를 기록한다. 실제 경로
    // (플러그인 상태 → pawn 매칭 → 유도 → 내루프 → JSBSim)를 그대로 지나므로,
    // JSBSim 단독 하네스가 구조적으로 못 보는 배선·전환·포화 결함을 잡는다.
    if (bRunFormationTest) UpdateFormationTest(Pawns);

    if (Setpoints.Num() == 0) return;

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

// ─── 인엔진 편대 시험 (60 Hz, AutopilotTick에서 호출) ─────────────────────────
void AUDPControlReceiver::UpdateFormationTest(const TArray<AActor*>& Pawns)
{
    if (!FormationTest.IsValid()) FormationTest = MakeShared<FFormationTest>();
    FFormationTest& T = *FormationTest;
    if (T.Phase == FFormationTest::EPhase::Done) return;

    APawn* Ldr = MatchPawnByKey(Pawns, TestLeaderName);
    APawn* Fol = MatchPawnByKey(Pawns, TestFollowerName);
    UJSBSimMovementComponent* LJ = Ldr ? FindJSBSimMovementComponent(Ldr) : nullptr;
    UJSBSimMovementComponent* FJ = Fol ? FindJSBSimMovementComponent(Fol) : nullptr;
    if (!LJ || !FJ)
    {
        if (!T.bWarned)
        {
            T.bWarned = true;
            UE_LOG(LogTemp, Error, TEXT("[FormTest] pawn 없음 (leader='%s' %s / follower='%s' %s) — 시험 중단"),
                *TestLeaderName, LJ ? TEXT("OK") : TEXT("MISSING"),
                *TestFollowerName, FJ ? TEXT("OK") : TEXT("MISSING"));
        }
        return;
    }

    const FVector LLoc = Ldr->GetActorLocation(), FLoc = Fol->GetActorLocation();
    const double LdrAlt = LLoc.Z * 0.01, FolAlt = FLoc.Z * 0.01;
    if (T.LdrSpawnAlt < -1e8) { T.LdrSpawnAlt = LdrAlt; T.FolSpawnAlt = FolAlt; }

    const FAircraftState& LS = LJ->AircraftState;
    const double LTas = LS.TotalVelocityKts * KnotToMetersPerSecond;
    const double LdrClimb = LdrAlt - T.LdrSpawnAlt;
    const double FolClimb = FolAlt - T.FolSpawnAlt;

    // ── 리더 스크립트 (direct 명령 — 내루프가 추종) ──
    using EP = FFormationTest::EPhase;
    double LdrAltCmd = T.LdrSpawnAlt + 800.0, LdrSpdCmd = 220.0, LdrRff = 0.0, TurnRate = 0.0;

    switch (T.Phase)
    {
    case EP::Takeoff:
        T.HdgCmd = FFormationTest::kRunwayHdg;
        if (LdrClimb >= 200.0)
        {
            T.CruiseAlt = LdrAlt;
            T.HdgCmd    = LS.LocalEulerAngles.Yaw;
            T.Enter(EP::Straight);
            UE_LOG(LogTemp, Warning, TEXT("[FormTest] 리더 이륙 완료(+%.0fm) → 직선 구간"), LdrClimb);
        }
        break;
    case EP::Straight:
        LdrAltCmd = T.CruiseAlt;
        if (T.PhaseT >= FFormationTest::Def(EP::Straight).Dur) { T.Enter(EP::Turn3); }
        break;
    case EP::Turn3:
        TurnRate = 3.0; LdrAltCmd = T.CruiseAlt;
        if (T.PhaseT >= FFormationTest::Def(EP::Turn3).Dur) { T.Enter(EP::Turn4Climb); }
        break;
    case EP::Turn4Climb:
    {
        const double F = FMath::Clamp(T.PhaseT / FFormationTest::Def(EP::Turn4Climb).Dur, 0.0, 1.0);
        TurnRate  = 4.0;
        LdrAltCmd = T.CruiseAlt + 400.0 * F;
        LdrSpdCmd = 220.0 - 50.0 * F;                      // 220 → 170
        if (T.PhaseT >= FFormationTest::Def(EP::Turn4Climb).Dur) { T.CruiseAlt += 400.0; T.Enter(EP::Rollout); }
        break;
    }
    case EP::Rollout:
        LdrAltCmd = T.CruiseAlt; LdrSpdCmd = 170.0;
        if (T.PhaseT >= FFormationTest::Def(EP::Rollout).Dur) { T.Enter(EP::Breakaway); }
        break;
    case EP::Breakaway:                                   // 리더는 직선 유지 (이탈은 팔로워 쪽)
        LdrAltCmd = T.CruiseAlt; LdrSpdCmd = 200.0;
        if (T.PhaseT >= FFormationTest::Def(EP::Breakaway).Dur) { T.Enter(EP::FarChase); }
        break;
    case EP::FarChase:                                    // 직선 유지 — REJOIN 재합류 검증
        LdrAltCmd = T.CruiseAlt; LdrSpdCmd = 200.0;
        if (T.PhaseT >= FFormationTest::Def(EP::FarChase).Dur) { T.Enter(EP::Done); }
        break;
    default: break;
    }

    if (T.Phase != EP::Takeoff && T.Phase != EP::Done)
    {
        T.HdgCmd = FMath::Fmod(T.HdgCmd + TurnRate * FFormationTest::kDt + 360.0, 360.0);
        LdrRff   = FMath::RadiansToDegrees(FMath::Atan2(FMath::DegreesToRadians(TurnRate) * FMath::Max(LTas, 50.0), 9.80665));
    }

    {
        FUavSetpoint LSP;                       // Direct (기본값)
        LSP.HeadingDeg     = (float)T.HdgCmd;
        LSP.AltitudeM      = (float)LdrAltCmd;
        LSP.TargetSpeedMps = (float)LdrSpdCmd;
        LSP.RollFfDeg      = (float)LdrRff;
        Setpoints.Add(PawnIdName(Ldr), LSP);
    }

    // ── 팔로워: BT와 동일한 게이팅으로 formation 진입 ──
    if (bTestDriveFollower)
    {
        if (!T.bFolFormation && FolClimb >= 150.0 && LdrClimb >= 80.0)
        {
            T.bFolFormation = true;
            T.FormEntryT = T.TotalT;
            UE_LOG(LogTemp, Warning, TEXT("[FormTest] 팔로워 편대 진입 (own+%.0fm, leader+%.0fm)"), FolClimb, LdrClimb);
        }
        FUavSetpoint FSP;
        if (!T.bFolFormation)
        {
            FSP.HeadingDeg     = (float)FFormationTest::kRunwayHdg;
            FSP.AltitudeM      = (float)(T.FolSpawnAlt + 800.0);
            FSP.TargetSpeedMps = 220.f;
        }
        else if (T.Phase == EP::Breakaway)
        {
            // 이탈 주입: 편대 명령을 끊고 대각(+45°)으로 이탈 — 공격임무 후 재합류 상황 모사.
            // 상대속도 ~153m/s × 20s ≈ 3km 변위 → FarChase에서 REJOIN(>800m) 발동 검증.
            FSP.HeadingDeg     = (float)FMath::Fmod(T.HdgCmd + 45.0, 360.0);
            FSP.AltitudeM      = (float)T.CruiseAlt;
            FSP.TargetSpeedMps = 250.f;
        }
        else
        {
            FSP.Mode        = EGuidanceMode::Formation;
            FSP.LeaderName  = PawnIdName(Ldr);
            FSP.SlotFrontM  = -80.f; FSP.SlotRightM = 100.f; FSP.SlotUpM = 0.f;
            FSP.MinSpeedMps = 120.f; FSP.MaxSpeedMps = 335.f;
            FSP.MinAltM     = (float)(T.FolSpawnAlt + 150.0);
        }
        Setpoints.Add(PawnIdName(Fol), FSP);
    }

    // ── 계측: 리더 실제 track 기준 슬롯 오차 ──
    const double LN = -LLoc.Y * 0.01, LE = LLoc.X * 0.01;
    const double FN = -FLoc.Y * 0.01, FE = FLoc.X * 0.01;
    const double LVn = LS.VelocityNEDfps.X * 0.3048, LVe = LS.VelocityNEDfps.Y * 0.3048;
    const double Trk = FMath::Atan2(LVe, LVn);
    const double Cs = FMath::Cos(Trk), Sn = FMath::Sin(Trk);
    const double SlotN = LN + (-80.0) * Cs - (100.0) * Sn;
    const double SlotE = LE + (-80.0) * Sn + (100.0) * Cs;
    const double ESlot = FMath::Sqrt(FMath::Square(SlotN - FN) + FMath::Square(SlotE - FE));
    const double FolPhi = FJ->AircraftState.LocalEulerAngles.Roll;

    if (T.bFolFormation)
    {
        if ((int32)T.Phase <= (int32)EP::Rollout)               // Breakaway부터는 의도적 이탈
            T.MaxEAfterForm = FMath::Max(T.MaxEAfterForm, ESlot);
        if (T.Phase == EP::Breakaway || (T.Phase == EP::FarChase && T.RecaptureT < 0.0))
            T.BreakawayPeakM = FMath::Max(T.BreakawayPeakM, ESlot);
        if (T.Phase == EP::FarChase && T.RecaptureT < 0.0 && ESlot < 30.0)
            T.RecaptureT = T.PhaseT;                            // 재합류 소요
        if (T.SettleT < 0.0 && ESlot < 30.0) T.SettleT = T.TotalT;   // 최초 30m 이내 진입
        if (T.InGate()) T.Stat[(int32)T.Phase].Add(ESlot, FolPhi);
    }

    T.CsvAccum += FFormationTest::kDt;
    if (T.CsvAccum >= 0.1)                       // 10 Hz 기록
    {
        T.CsvAccum = 0.0;
        T.Csv.Add(FString::Printf(TEXT("%.2f,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d"),
            T.TotalT, FFormationTest::Def(T.Phase).Name, ESlot,
            (LdrAlt - FolAlt), FolPhi, LS.LocalEulerAngles.Roll,
            FJ->AircraftState.TotalVelocityKts * KnotToMetersPerSecond, LTas,
            FMath::RadiansToDegrees(Trk), T.bFolFormation ? 1 : 0));
    }

    T.Advance();

    // ── 종료: CSV 저장 + 게이트 판정 ──
    if (T.Phase == EP::Done)
    {
        const FString Path = FPaths::ProjectSavedDir() / TEXT("FormationTest.csv");
        FString Out = TEXT("t,phase,eSlot,dAlt,phiFol,phiLdr,vFol,vLdr,trkLdr,formation\n");
        Out += FString::Join(T.Csv, TEXT("\n"));
        FFileHelper::SaveStringToFile(Out, *Path);

        int32 Fail = 0;
        double MaxPhi = 0.0;
        UE_LOG(LogTemp, Warning, TEXT("[FormTest] ══ 인엔진 편대 시험 결과 ══"));
        for (int32 i = (int32)EP::Straight; i <= (int32)EP::FarChase; ++i)
        {
            const FFormationTest::FStat& S = T.Stat[i];
            const FFormationTest::FPhaseDef& D = FFormationTest::Def((EP)i);
            const bool bOk = S.N > 0 && S.MaxE < D.GateM;
            if (!bOk) ++Fail;
            MaxPhi = FMath::Max(MaxPhi, S.MaxPhi);
            UE_LOG(LogTemp, Warning, TEXT("[FormTest]   %-18s max=%7.1fm mean=%7.1fm (n=%d) gate<%.0fm  %s"),
                D.Name, S.MaxE, S.Mean(), S.N, D.GateM, bOk ? TEXT("PASS") : TEXT("★FAIL"));
        }
        const bool bPhiOk = MaxPhi <= 64.0;
        const bool bDivOk = T.MaxEAfterForm < 600.0;
        if (!bPhiOk) ++Fail;
        if (!bDivOk) ++Fail;
        const double Capture = (T.SettleT > 0.0 && T.FormEntryT > 0.0) ? (T.SettleT - T.FormEntryT) : -1.0;
        const bool bCapOk = Capture > 0.0 && Capture < 45.0;
        if (!bCapOk) ++Fail;
        UE_LOG(LogTemp, Warning, TEXT("[FormTest]   합류→30m 캡처   = %.1fs  gate<45s  %s"),
            Capture, bCapOk ? TEXT("PASS") : TEXT("★FAIL"));
        // 재합류 물리 소요: 135° 선회(~25s) + 3km 추격(~30s) + 감속 정착(~15s) ≈ 70s → 여유 90s
        const bool bRecapOk = T.RecaptureT >= 0.0 && T.RecaptureT < 90.0;
        if (!bRecapOk) ++Fail;
        UE_LOG(LogTemp, Warning, TEXT("[FormTest]   이탈 최대거리    = %.0fm (정보) / 재합류 %.1fs  gate<90s  %s"),
            T.BreakawayPeakM, T.RecaptureT, bRecapOk ? TEXT("PASS") : TEXT("★FAIL"));
        UE_LOG(LogTemp, Warning, TEXT("[FormTest]   |phi|max(팔로워) = %.1f°  gate<=64°  %s"), MaxPhi, bPhiOk ? TEXT("PASS") : TEXT("★FAIL"));
        UE_LOG(LogTemp, Warning, TEXT("[FormTest]   발산가드 maxE    = %.0fm  gate<600m (Rollout까지)  %s"), T.MaxEAfterForm, bDivOk ? TEXT("PASS") : TEXT("★FAIL"));
        UE_LOG(LogTemp, Warning, TEXT("[FormTest] %s (FAIL=%d)  csv=%s"),
            Fail ? TEXT("★★ 게이트 불통과") : TEXT("══ 전체 PASS ══"), Fail, *Path);

        bRunFormationTest = false;
        if (bTestExitOnFinish) FPlatformMisc::RequestExit(false);
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
            GuidanceStatusByPawn.Remove(PawnIdName(Pawn));
            return;
        }
    }

    // Per-UAV 3계층 제어 스택, created on first use — 내부 필터/적분기/슬루 상태가
    // 틱을 넘어 유지돼야 한다 (see FUavControl).
    TSharedPtr<FUavControl>& Ctl = UavControls.FindOrAdd(Key);
    if (!Ctl.IsValid()) Ctl = MakeShared<FUavControl>();

    const FAircraftState& S = JSBSim->AircraftState;

    // 현재 상태: JSBSim aero 오일러(LocalEulerAngles, deg, ψ 0=북=기수방위 yaw), UE-Z 고도.
    const double PhiDeg   = S.LocalEulerAngles.Roll;
    const double ThetaDeg = S.LocalEulerAngles.Pitch;
    const double PsiDeg   = S.LocalEulerAngles.Yaw;        // yaw — course 피드백은 FWG가
                                                           // ground velocity로 계산 [§3]
    const float  AltM     = (float)(Pawn->GetActorLocation().Z / 100.0);
    const float  SpeedMps = (float)(S.TotalVelocityKts * KnotToMetersPerSecond);
    const double ClimbMps = S.AltitudeRateFtps * 0.3048;   // +위
    // EulerRates 출처 [§1-6,7]: 플러그인 784행 = JSBSim FGAuxiliary::GetEulerRates(ePhi/eTht/ePsi)
    // → '오일러각 변화율(φ̇,θ̇,ψ̇)'이며 body angular rate p,q,r가 아니다. 단위는 rad/s
    // (LocalEulerAngles만 RadiansToDegrees 변환됨) → 여기서 deg/s로 변환한다.
    // ※ 수정 전에는 rad/s를 deg/s로 오독 → 레이트 항이 사실상 0이었으나 전 게이트 PASS —
    //   감쇠는 PID D항이 지배(§5 방식 A 기본 선택 근거). 종전 Z(요레이트)→X 축 버그는 2026-07-09 수정.
    const double PhiDotDps   = MumtCtl::RadToDeg(S.EulerRates.X);   // φ̇ (deg/s)
    const double ThetaDotDps = MumtCtl::RadToDeg(S.EulerRates.Y);   // θ̇ (deg/s)
    const double PsiDotDps   = MumtCtl::RadToDeg(S.EulerRates.Z);   // ψ̇ (deg/s)
    // 자기 ground velocity (NED m/s): course 피드백[§3]과 편대 유도가 공용 — 참벡터.
    const double OwnVn = S.VelocityNEDfps.X * 0.3048;
    const double OwnVe = S.VelocityNEDfps.Y * 0.3048;
    constexpr double kDt  = 1.0 / 60.0;   // BeginPlay SetTimer(1/60) 고정 주기 [§1-5]

    // ── [1]+[2] 유도: 모드별로 FFlightReference 생성 ─────────────────────────
    // Direct: setpoint의 heading/alt/speed/roll_ff → StepDirect (기존 경로, 완전 호환).
    // Formation: 리더 직독 → FormationGuidance(슬롯·오차) → StepFormation.
    // Attack: 표적 직독 → FPursuitGuidance → StepDirect.
    FFlightReference Refs;
    bool bHaveRefs = false;
    FUavGuidanceStatus Status;
    Status.Mode  = Setpoint.Mode;
    Status.SeqId = Setpoint.SequenceId;

    // ── 모드/리더 변경 시 상태 초기화 [§9] — 변경 시점에만, 매 틱 아님 ──
    // 모드 변경: 컨트롤러 PID(트림 FF에서 범프리스 재시작) + FWG(roll_ref 슬루가 현재 φ에서
    // 재시작, REJOIN 해제). 리더 변경: 편대 추정기(ω 필터·캡처 래치)까지 리셋.
    if (Ctl->bHasLastMode && Ctl->LastMode != Setpoint.Mode)
    {
        Ctl->Controller.Reset();
        Ctl->Guidance.Reset();
        if (Setpoint.Mode == EGuidanceMode::Formation)
            Ctl->Formation.Reset();
        UE_LOG(LogTemp, Log, TEXT("[AP] %s 모드 전환 %d→%d — 제어 상태 리셋"),
            *Key, (int32)Ctl->LastMode, (int32)Setpoint.Mode);
    }
    else if (Setpoint.Mode == EGuidanceMode::Formation &&
             !Ctl->LastLeaderName.IsEmpty() && Ctl->LastLeaderName != Setpoint.LeaderName)
    {
        Ctl->Formation.Reset();
        Ctl->Guidance.bRejoin = false;
        UE_LOG(LogTemp, Log, TEXT("[AP] %s 리더 변경 '%s'→'%s' — 편대 추정기 리셋"),
            *Key, *Ctl->LastLeaderName, *Setpoint.LeaderName);
    }
    Ctl->LastMode = Setpoint.Mode;
    Ctl->bHasLastMode = true;
    if (Setpoint.Mode == EGuidanceMode::Formation)
        Ctl->LastLeaderName = Setpoint.LeaderName;

    if (Setpoint.Mode != EGuidanceMode::Direct)
    {
        // 편대/추격은 리더보다 선회 여유가 있어야 컷인 가능(요-SAS 페널티 +9°) — 단
        // 65°+ 지속 뱅크는 bank-to-turn 모델 밖(피치 push와 결합 시 요 역행·발산,
        // PIE 2026-07-09 실측: 72° 고착 → km 발산 반복) → 캡 62°.
        Ctl->Guidance.BankLimitDeg = 62.0;

        // 자기 위치 (NED m): north=-Y/100, east=X/100. (속도 OwnVn/OwnVe는 위에서 공용 계산)
        const FVector OwnLoc = Pawn->GetActorLocation();
        const double OwnN  = -OwnLoc.Y * 0.01;
        const double OwnE  =  OwnLoc.X * 0.01;

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

            // 1초 이상 유도가 끊겼다 재개되면 ω 미분·REJOIN 상태가 낡음 → 리셋(범프리스)
            if (Setpoint.Mode == EGuidanceMode::Formation && Now - Ctl->LastGuidTime > 1.0)
            {
                Ctl->Formation.Reset();
                Ctl->Guidance.bRejoin = false;
            }

            // 리더 미비행(지상활주/정지) 가드: 슬롯이 지상에 있고 track이 무의미 →
            // 슬롯 추종 대신 안전 홀드(현재 헤딩·고도 유지, 순항 220). PIE 2026-07-09:
            // 리더 이륙 전 편대 진입 → V=70(실속 직전)·저고도 명령이 발산의 시발점.
            const double RefGs = std::sqrt(RefVn * RefVn + RefVe * RefVe);
            if (Setpoint.Mode == EGuidanceMode::Formation && RefGs < 50.0)
            {
                FDirectCmd Hold;
                Hold.CourseDeg = PsiDeg;
                Hold.AltM      = std::max((double)AltM, (double)Setpoint.MinAltM + 150.0);
                Hold.SpeedMps  = 220.0;
                Hold.RollFfDeg = 0.0;
                Ctl->Formation.Reset();
                Ctl->Guidance.bRejoin = false;
                Ctl->LastGuidCmd = Hold; Ctl->LastGuidTime = Now; Ctl->bHasGuidCmd = true;
                Refs = Ctl->Guidance.StepDirect(Hold, OwnVn, OwnVe, PhiDeg, ThetaDeg, PsiDeg,
                                                (double)AltM, (double)SpeedMps, ClimbMps, kDt);
                bHaveRefs = true;
            }
            else if (Setpoint.Mode == EGuidanceMode::Formation)
            {
                // 판정·안전 파라미터 (setpoint 0 = 기본값 유지)
                if (Setpoint.CaptureTolM    > 0.f) Ctl->Formation.CaptureTolM    = Setpoint.CaptureTolM;
                if (Setpoint.MaintainTolM   > 0.f) Ctl->Formation.MaintainTolM   = Setpoint.MaintainTolM;
                Ctl->Formation.MinSeparationM = Setpoint.MinSeparationM;

                // [1] 슬롯·오차
                const FFormationTarget T = Ctl->Formation.Step(
                    RefN, RefE, RefAlt, RefVn, RefVe, RefClimb,
                    OwnN, OwnE, (double)AltM, OwnVn, OwnVe,
                    Setpoint.SlotFrontM, Setpoint.SlotRightM, Setpoint.SlotUpM, kDt);

                // [2] 비행 Reference
                Refs = Ctl->Guidance.StepFormation(
                    T, OwnN, OwnE, OwnVn, OwnVe,
                    PhiDeg, ThetaDeg, PsiDeg, (double)AltM, (double)SpeedMps, ClimbMps,
                    Setpoint.MinSpeedMps, Setpoint.MaxSpeedMps, Setpoint.MinAltM,
                    Setpoint.MaxClosingMps, kDt);
                bHaveRefs = true;

                // 소실 홀드용 direct 등가 명령 + BT 피드백 상태
                FDirectCmd Hold;
                Hold.CourseDeg = Refs.CourseRefDeg; Hold.AltM = Refs.AltRefM;
                Hold.SpeedMps  = Refs.AirspeedRefMps; Hold.RollFfDeg = 0.0;
                Ctl->LastGuidCmd = Hold; Ctl->LastGuidTime = Now; Ctl->bHasGuidCmd = true;

                Status.EAlongM = T.EAlongM;   Status.ECrossM = T.ECrossM; Status.EVertM = T.EVertM;
                Status.ClosingMps = T.ClosingSpeedMps; Status.SeparationM = T.SeparationM;
                Status.SlotDistM = T.SlotDist3M;
                Status.bCaptured = T.bCaptured; Status.bMaintained = T.bMaintained;
                Status.bRejoin = Ctl->Guidance.bRejoin; Status.bSepBreach = T.bSeparationBreach;
                Status.bSepWarning = Ctl->Guidance.bSepWarning;
                // 감속 보조 발동/한계 경고 로그 [§7-2,3] (1Hz 스로틀)
                if (Ctl->Guidance.bSepWarning)
                {
                    static double LastSepWarnLog = -10.0;
                    if (Now - LastSepWarnLog > 1.0)
                    {
                        LastSepWarnLog = Now;
                        UE_LOG(LogTemp, Warning,
                            TEXT("[Guid] %s ★분리 경고: 속도 하한인데 접근 지속 (sep=%.0fm rate=%.1fm/s) — 감속 보조 한계"),
                            *Key, T.SeparationM, T.SeparationRateMps);
                    }
                }
            }
            else // Attack
            {
                const FDirectCmd Cmd = Ctl->Pursuit.Step(
                    RefN, RefE, RefAlt, RefVn, RefVe,
                    OwnN, OwnE,
                    Setpoint.MinSpeedMps, Setpoint.MaxSpeedMps, Setpoint.MinAltM);
                Ctl->LastGuidCmd  = Cmd;
                Ctl->LastGuidTime = Now;
                Ctl->bHasGuidCmd  = true;
                Refs = Ctl->Guidance.StepDirect(Cmd, OwnVn, OwnVe, PhiDeg, ThetaDeg, PsiDeg,
                                                (double)AltM, (double)SpeedMps, ClimbMps, kDt);
                bHaveRefs = true;
            }
        }
        else if (Ctl->bHasGuidCmd && Now - Ctl->LastGuidTime < 5.0)
        {
            // 리더/표적 일시 소실 → 마지막 유도 명령 홀드
            Refs = Ctl->Guidance.StepDirect(Ctl->LastGuidCmd, OwnVn, OwnVe, PhiDeg, ThetaDeg, PsiDeg,
                                            (double)AltM, (double)SpeedMps, ClimbMps, kDt);
            bHaveRefs = true;
        }
        else
        {
            // 5초 초과 소실 → 현재 헤딩·고도 수평 유지 (순항 200)
            FDirectCmd Hold;
            Hold.CourseDeg = PsiDeg;
            Hold.AltM      = (double)AltM;
            Hold.SpeedMps  = 200.0;
            Hold.RollFfDeg = 0.0;
            Refs = Ctl->Guidance.StepDirect(Hold, OwnVn, OwnVe, PhiDeg, ThetaDeg, PsiDeg,
                                            (double)AltM, (double)SpeedMps, ClimbMps, kDt);
            bHaveRefs = true;
        }
    }

    if (!bHaveRefs)
    {
        // Direct: setpoint 그대로 → Reference (기존 시나리오 완전 호환 경로)
        FDirectCmd Cmd;
        Cmd.CourseDeg = Setpoint.HeadingDeg;
        Cmd.AltM      = Setpoint.AltitudeM;
        Cmd.SpeedMps  = Setpoint.TargetSpeedMps;
        Cmd.RollFfDeg = Setpoint.RollFfDeg;
        Refs = Ctl->Guidance.StepDirect(Cmd, OwnVn, OwnVe, PhiDeg, ThetaDeg, PsiDeg,
                                        (double)AltM, (double)SpeedMps, ClimbMps, kDt);
    }

    // ── [3] F16 Command Controller: Reference → fcs/*-cmd-norm ──────────────
    // 조종면 부호 미세조정(PIE 튜닝) 반영.
    Ctl->Controller.AilSign = StickAileronScale;
    Ctl->Controller.ElvSign = StickElevatorScale;

    const FF16ControlCommand O = Ctl->Controller.Step(
        Refs.RollRefDeg, Refs.PitchRefDeg, Refs.AirspeedRefMps, Refs.ThrottleRefNorm,
        PhiDeg, ThetaDeg, PhiDotDps, ThetaDotDps, PsiDotDps,
        (double)SpeedMps, (double)Setpoint.Throttle, kDt);

    // 최종 JSBSim 명령 기록 — 무인기 제어 경로에서 이 지점 하나뿐이다.
    const float Ail = O.AileronCmdNorm, Elv = O.ElevatorCmdNorm;
    const float Rud = O.RudderCmdNorm,  ThrottleOut = O.ThrottleCmdNorm;
    JSBSim->Commands.Aileron    = Ail;
    JSBSim->Commands.Elevator   = Elv;
    JSBSim->Commands.Rudder     = Rud;
    JSBSim->Commands.SpeedBrake = O.SpeedbrakeCmdNorm;   // 과속 시 감속 보조 (리더 급감속 추종)
    if (JSBSim->EngineCommands.Num() > 0)
        JSBSim->EngineCommands[0].Throttle = ThrottleOut;

    // BT 피드백 상태 (5006 배치의 "guidance" 오브젝트)
    Status.RollRefDeg = Refs.RollRefDeg; Status.PitchRefDeg = Refs.PitchRefDeg;
    Status.AirspeedRefMps = Refs.AirspeedRefMps;
    TSharedPtr<FUavGuidanceStatus>& StatusSlot = GuidanceStatusByPawn.FindOrAdd(PawnIdName(Pawn));
    if (!StatusSlot.IsValid()) StatusSlot = MakeShared<FUavGuidanceStatus>();
    *StatusSlot = Status;

    // Weapon triggers ride along with the setpoint (Phase 3). Missile edge
    // detection lives in UWeaponComponent; only forward ids > 0 so the msg
    // default (0 = never fired) can't be mistaken for a first shot.
    if (UWeaponComponent* Weapon = Pawn->FindComponentByClass<UWeaponComponent>())
    {
        if (Setpoint.bGunFiring != Weapon->bGunFiring)   // [SGF-SP] TEMP: 5010 setpoint 경로 발원지 태그
            UE_LOG(LogTemp, Warning, TEXT("[SGF-SP] %s req=%d"), *Pawn->GetName(), Setpoint.bGunFiring ? 1 : 0);
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
        // [§13] 현재 course와 yaw를 함께 기록 (course 피드백 검증용), 스로틀 FF/보정 분해.
        UE_LOG(LogTemp, Warning,
            TEXT("[Ref] %s [%s] -> CrsRef=%.0f RollRef=%.1f PitchRef=%.1f Vref=%.0f | Crs=%.0f Yaw=%.0f Phi=%.0f Alt=%.0f V=%.0f | Ail=%.2f Elv=%.2f Thr=%.2f(FF %.2f%+.2f)%s%s"),
            *Key, ModeTag, (float)Refs.CourseRefDeg, (float)Refs.RollRefDeg,
            (float)Refs.PitchRefDeg, (float)Refs.AirspeedRefMps,
            (float)Ctl->Guidance.LastCurrentCourseDeg, (float)PsiDeg, (float)PhiDeg, AltM, SpeedMps,
            Ail, Elv, ThrottleOut,
            (float)Ctl->Controller.LastThrottleFF, (float)Ctl->Controller.LastThrottleCorr,
            Ctl->Controller.bAilSaturated ? TEXT(" AIL-SAT") : TEXT(""),
            Ctl->Controller.bElvSaturated ? TEXT(" ELV-SAT") : TEXT(""));
        if (Setpoint.Mode == EGuidanceMode::Formation)
            UE_LOG(LogTemp, Warning,
                TEXT("[Guid] %s along=%+.0f cross=%+.0f vert=%+.0f omega=%+.2fdps sep=%.0f cap=%d mnt=%d%s%s"),
                *Key, (float)Status.EAlongM, (float)Status.ECrossM, (float)Status.EVertM,
                (float)(Ctl->Formation.OmegaFilt * 57.29577951), (float)Status.SeparationM,
                Status.bCaptured ? 1 : 0, Status.bMaintained ? 1 : 0,
                Status.bRejoin ? TEXT(" REJOIN") : TEXT(""),
                Status.bSepWarning ? TEXT(" ★SEP-WARN") : TEXT(""));
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
            GuidanceStatusByPawn.Remove(PawnIdName(Pawn));
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

        // Roll: 어시스트 켜짐 = 스틱을 뱅크각 명령으로 해석(FBWA류). f16 FCS가 레이트
        // 루프를 이미 닫고 있으므로 여기는 '자세(뱅크)'만 담당 — 레이트 루프 이중적재 아님.
        // (무인기 내루프의 heading→roll→FCS 캐스케이드와 같은 구조로 이미 검증된 방식.)
        double RollCmd = Command.Roll * Auth;                  // 구 방식(어시스트 꺼짐)
        if (bMannedRollAssist)
        {
            const double Phi    = JSBSim->AircraftState.LocalEulerAngles.Roll;
            const double PhiRef = FMath::Clamp((double)Command.Roll, -1.0, 1.0) * (double)MannedBankLimitDeg;
            const double Rate   = FMath::Clamp((double)MannedRollKp * (PhiRef - Phi),
                                               -(double)MannedRollRateLimitDps,
                                                (double)MannedRollRateLimitDps);
            RollCmd = Rate / 180.0;                            // f16 FCS 규격: 1.0 = 180°/s
        }
        JSBSim->Commands.Aileron  = RollCmd;
        JSBSim->Commands.Elevator = Command.Pitch * Auth;      // FCS에 G/받음각 제한 내장
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
        if (Command.bGunFiring != Weapon->bGunFiring)   // [SGF-CMD] TEMP: 5005 수동조종 경로 발원지 태그
            UE_LOG(LogTemp, Warning, TEXT("[SGF-CMD] %s req=%d"), *Pawn->GetName(), Command.bGunFiring ? 1 : 0);
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

    // 유도 상태 피드백 — BT의 CheckFormationCaptured/Maintained가 소비한다.
    // (BT가 편대 오차를 재계산하지 않도록 UE가 판정 결과를 내려보낸다.)
    if (const TSharedPtr<FUavGuidanceStatus>* Found = GuidanceStatusByPawn.Find(PawnIdName(Pawn)))
    {
        if (Found->IsValid())
        {
            const FUavGuidanceStatus& G = **Found;
            const TSharedPtr<FJsonObject> GuidJson = MakeShared<FJsonObject>();
            GuidJson->SetStringField(TEXT("mode"),
                G.Mode == EGuidanceMode::Formation ? TEXT("formation") :
                G.Mode == EGuidanceMode::Attack    ? TEXT("attack")    : TEXT("direct"));
            GuidJson->SetNumberField(TEXT("e_along_m"),    G.EAlongM);
            GuidJson->SetNumberField(TEXT("e_cross_m"),    G.ECrossM);
            GuidJson->SetNumberField(TEXT("e_vert_m"),     G.EVertM);
            GuidJson->SetNumberField(TEXT("closing_mps"),  G.ClosingMps);
            GuidJson->SetNumberField(TEXT("separation_m"), G.SeparationM);
            GuidJson->SetNumberField(TEXT("slot_dist_m"),  G.SlotDistM);
            GuidJson->SetBoolField(TEXT("captured"),   G.bCaptured);
            GuidJson->SetBoolField(TEXT("maintained"), G.bMaintained);
            GuidJson->SetBoolField(TEXT("rejoin"),     G.bRejoin);
            GuidJson->SetBoolField(TEXT("sep_breach"), G.bSepBreach);
            GuidJson->SetBoolField(TEXT("separation_warning"), G.bSepWarning);   // [§7-6]
            GuidJson->SetNumberField(TEXT("seq"),        G.SeqId);
            GuidJson->SetNumberField(TEXT("roll_ref"),   G.RollRefDeg);
            GuidJson->SetNumberField(TEXT("pitch_ref"),  G.PitchRefDeg);
            GuidJson->SetNumberField(TEXT("speed_ref"),  G.AirspeedRefMps);
            PawnJson->SetObjectField(TEXT("guidance"), GuidJson);
        }
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
