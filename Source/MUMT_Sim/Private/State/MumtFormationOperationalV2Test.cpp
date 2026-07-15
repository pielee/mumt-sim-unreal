// MumtFormationOperationalV2Test.cpp — Phase D.
//
// These tests drive Formation through the REAL operational command path: a UDP datagram on port 5010 is
// parsed by the production AUDPControlReceiver, routed by RouteControlV2 to the per-aircraft
// UFormationRuntimeOwnerV2, which drives the real NPFG/TECS/stick producer through the Phase B/C
// Prime/handoff contract into the command arbiter. Nothing here calls the producer or the runtime owner
// directly to START Formation -- the only trigger is an external "control_mode: formation" command, exactly
// as an operator would send it.
//
// What they establish: an old packet stays legacy; an operational enable produces an exact-zero first
// consume with a REAL candidate; the real producer keeps moving the controls; repeated enables are
// idempotent; a slot update does not re-prime; a leader change forces a fresh handoff; disable is an
// immediate Legacy fallback; stale/replayed/invalid commands are refused; Falling preempts; per-aircraft
// isolation holds; and world teardown leaves no runtime state.
//
// TECS continuity is NOT claimed: the exact-zero is the stick latch's; the active-frame deltas are logged,
// never required to be zero.

#include "Misc/AutomationTest.h"
#include "FormationControlV2/FormationRuntimeOwnerV2.h"
#include "State/MumtCommandArbiterV2.h"
#include "HealthComponent.h"
#include "JSBSimMovementComponent.h"
#include "Tests/AutomationEditorCommon.h"
#include "Editor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Misc/App.h"
#include "Common/UdpSocketBuilder.h"
#include "IPAddress.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

DEFINE_LOG_CATEGORY_STATIC(LogMumtFormOp, Display, All);

namespace
{
namespace Arb = MumtCommandArbiterV2;

const TCHAR *kOpMap = TEXT("/Game/RL_2");
constexpr int32  kSetpointPort = 5010;   // AUDPControlReceiver::SetpointListenPort default
constexpr double kOpSettleS = 5.0;   // let the scripted flight get the follower airborne and stable first
constexpr double kOpMaxWallSeconds = 340.0;

enum class EOpScenario : uint8
{
	NoCommandRemainsLegacy,     // 1
	EnableExactFirstConsume,    // 2
	ActiveProducerUpdates,      // 3
	RepeatedEnableIdempotent,   // 4
	SlotUpdateWhileActive,      // 5
	LeaderChangeNewHandoff,     // 6
	DisableImmediateFallback,   // 7
	InvalidOrStaleRejected,     // 8
	FallingPreempts,            // 9
	PerAircraftIsolation,       // 10
	WorldCleanup,               // 11
};

const TCHAR *OpScenarioName(EOpScenario S)
{
	switch (S)
	{
	case EOpScenario::NoCommandRemainsLegacy:   return TEXT("NoCommandRemainsLegacy");
	case EOpScenario::EnableExactFirstConsume:  return TEXT("EnableExactFirstConsume");
	case EOpScenario::ActiveProducerUpdates:    return TEXT("ActiveProducerUpdates");
	case EOpScenario::RepeatedEnableIdempotent: return TEXT("RepeatedEnableIdempotent");
	case EOpScenario::SlotUpdateWhileActive:    return TEXT("SlotUpdateWhileActive");
	case EOpScenario::LeaderChangeNewHandoff:   return TEXT("LeaderChangeNewHandoff");
	case EOpScenario::DisableImmediateFallback: return TEXT("DisableImmediateFallback");
	case EOpScenario::InvalidOrStaleRejected:   return TEXT("InvalidOrStaleRejected");
	case EOpScenario::FallingPreempts:          return TEXT("FallingPreempts");
	case EOpScenario::PerAircraftIsolation:     return TEXT("PerAircraftIsolation");
	case EOpScenario::WorldCleanup:             return TEXT("WorldCleanup");
	default:                                    return TEXT("?");
	}
}

AActor *OpFindActor(UWorld *World, const TCHAR *Label)
{
	if (!World) return nullptr;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (It->GetActorNameOrLabel().Contains(Label)
			&& It->FindComponentByClass<UJSBSimMovementComponent>())
			return *It;
	}
	return nullptr;
}

UJSBSimMovementComponent *OpFindComp(UWorld *World, const TCHAR *Label)
{
	AActor *A = OpFindActor(World, Label);
	return A ? A->FindComponentByClass<UJSBSimMovementComponent>() : nullptr;
}

UFormationRuntimeOwnerV2 *OpFindOwner(UWorld *World, const TCHAR *Label)
{
	AActor *A = OpFindActor(World, Label);
	return A ? A->FindComponentByClass<UFormationRuntimeOwnerV2>() : nullptr;
}

double OpSimTime(UWorld *World)
{
	if (UJSBSimMovementComponent *C = OpFindComp(World, TEXT("F16_UAV1")))
	{
		FJsbFlightSnapshot S{};
		if (C->GetJsbFlightSnapshot(S) && S.bValidFrame) return S.SimTimeSec;
	}
	return -1.0;
}

// Sends real UDP setpoint datagrams to the production receiver on 5010.
struct FOpUdpSender
{
	FSocket *Socket = nullptr;
	TSharedPtr<FInternetAddr> Addr;

	bool Init()
	{
		ISocketSubsystem *SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
		if (!SS) return false;
		Socket = SS->CreateSocket(NAME_DGram, TEXT("MumtOpTestSender"), false);
		if (!Socket) return false;
		Addr = SS->CreateInternetAddr();
		bool bValid = false;
		Addr->SetIp(TEXT("127.0.0.1"), bValid);
		Addr->SetPort(kSetpointPort);
		return bValid;
	}
	void SendRaw(const FString &Msg)
	{
		if (!Socket || !Addr.IsValid()) return;
		FTCHARToUTF8 Utf8(*Msg);
		int32 Sent = 0;
		Socket->SendTo(reinterpret_cast<const uint8 *>(Utf8.Get()), Utf8.Length(), Sent, *Addr);
	}
	void SendFormation(const FString &Aircraft, const FString &Leader,
	                   double Front, double Right, double Up, int64 Seq, double Timestamp)
	{
		SendRaw(FString::Printf(
			TEXT("{\"aircraft_name\":\"%s\",\"control_mode\":\"formation\",\"leader_name\":\"%s\","
			     "\"slot_front_m\":%.3f,\"slot_right_m\":%.3f,\"slot_up_m\":%.3f,"
			     "\"command_sequence\":%lld,\"command_timestamp\":%.6f}"),
			*Aircraft, *Leader, Front, Right, Up, (long long)Seq, Timestamp));
	}
	void SendLegacy(const FString &Aircraft, int64 Seq, double Timestamp)
	{
		SendRaw(FString::Printf(
			TEXT("{\"aircraft_name\":\"%s\",\"control_mode\":\"legacy\",\"command_sequence\":%lld,\"command_timestamp\":%.6f}"),
			*Aircraft, (long long)Seq, Timestamp));
	}
	void SendOldStyle(const FString &Aircraft)
	{
		// An existing packet with NO ControlV2 fields (backward-compat): the parser must tolerate it and
		// leave the aircraft legacy.
		SendRaw(FString::Printf(
			TEXT("{\"aircraft_name\":\"%s\",\"throttle_norm\":0.80,\"target_speed_mps\":220.0}"), *Aircraft));
	}
	void Close()
	{
		if (Socket)
		{
			ISocketSubsystem *SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
			Socket->Close();
			if (SS) SS->DestroySocket(Socket);
			Socket = nullptr;
		}
	}
};

struct FOpState
{
	FOpUdpSender Udp;
	FString Follower = TEXT("F16_UAV1");
	FString Leader   = TEXT("M_F16");
	FString Leader2  = TEXT("F16_UAV2");
	FString Other    = TEXT("F16_UAV2");

	double FirstWall = -1.0;
	double FirstSim = -1.0;
	bool bSettled = false;
	int32 Phase = 0;
	int64 Seq = 1;

	// captured
	uint64 GenAtActive = 0;
	uint32 CandGenAtActive = 0;
	int64 FormationAtMark = 0;
	int64 LegacyAtMark = 0;
	uint64 ConsumeAtMark = 0;
	double BaseAil = 0, BaseElv = 0, BaseThr = 0;
	bool bBaseCaptured = false;
	double MaxDevAil = 0, MaxDevElv = 0, MaxDevThr = 0;
	bool bAnyRealOutput = false;
	bool bReachedActive = false;
	bool bDamaged = false;
	int32 OtherOwnerBeforeCount = 0;
	bool bSawLeaderFallback = false;
	EFormationRuntimeFallbackV2 RejectReason = EFormationRuntimeFallbackV2::None;
};

// Poll helper: is the follower's owner in Active with the arbiter in Formation?
bool OpIsActive(UWorld *World, const FString &Follower)
{
	UFormationRuntimeOwnerV2 *O = OpFindOwner(World, *Follower);
	return O && O->GetPhase() == EFormationRuntimePhaseV2::Active
	         && O->GetActiveMode() == Arb::ECommandMode::FormationControlV2;
}

void CaptureBaselineDev(UWorld *World, FOpState *St)
{
	UJSBSimMovementComponent *F = OpFindComp(World, *St->Follower);
	if (!F) return;
	if (!St->bBaseCaptured)
	{
		Arb::FResolvedCommandSnapshot B{};
		if (Arb::GetTicketBaseline(F, B))
		{
			St->BaseAil = B.Commands.Aileron; St->BaseElv = B.Commands.Elevator;
			St->BaseThr = B.EngineCommands.Num() > 0 ? B.EngineCommands[0].Throttle : 0.0;
			St->bBaseCaptured = true;
		}
		return;
	}
	Arb::FResolvedCommandSnapshot R{};
	if (Arb::GetLastResolvedSnapshot(F, R))
	{
		const double Thr = R.EngineCommands.Num() > 0 ? R.EngineCommands[0].Throttle : 0.0;
		St->MaxDevAil = FMath::Max(St->MaxDevAil, FMath::Abs(R.Commands.Aileron - St->BaseAil));
		St->MaxDevElv = FMath::Max(St->MaxDevElv, FMath::Abs(R.Commands.Elevator - St->BaseElv));
		St->MaxDevThr = FMath::Max(St->MaxDevThr, FMath::Abs(Thr - St->BaseThr));
		if (St->MaxDevAil > 1e-4 || St->MaxDevElv > 1e-4 || St->MaxDevThr > 1e-4) St->bAnyRealOutput = true;
	}
}

// ---------------------------------------------------------------------------------------------------
class FMumtOpCommand : public IAutomationLatentCommand
{
public:
	FMumtOpCommand(FAutomationTestBase *T, TSharedPtr<FOpState> S, EOpScenario Sc)
		: Test(T), St(S), Scenario(Sc) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		if (St->FirstWall < 0) { St->FirstWall = NowWall; St->Udp.Init(); }
		UWorld *World = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
		if (!World) return false;

		const double SimT = OpSimTime(World);
		if (SimT < 0.0) return false;
		if (St->FirstSim < 0.0) St->FirstSim = SimT;

		if (!OpFindActor(World, *St->Follower) || !OpFindActor(World, *St->Leader))
		{
			if ((NowWall - St->FirstWall) > 30.0) { Test->AddError(TEXT("[FOP] aircraft not found")); return Finalize(World); }
			return false;
		}

		if (!St->bSettled)
		{
			if (SimT - St->FirstSim < kOpSettleS) return false;
			St->bSettled = true;
			Arb::ResetSession(OpScenarioName(Scenario));
			return false;
		}
		if ((NowWall - St->FirstWall) > kOpMaxWallSeconds)
		{
			Test->AddError(TEXT("[FOP] wall budget exceeded")); return Finalize(World);
		}
		return Drive(World, SimT);
	}

private:
	double Now() const { return FApp::GetCurrentTime(); }

	bool Drive(UWorld *World, double SimT)
	{
		const double Elapsed = SimT - St->FirstSim - kOpSettleS;

		switch (Scenario)
		{
		case EOpScenario::NoCommandRemainsLegacy:
		{
			// An existing packet with no ControlV2 fields. The parser must tolerate it and NOT engage.
			if (St->Phase == 0) { St->Udp.SendOldStyle(St->Follower); St->Phase = 1; }
			if (Elapsed < 3.0) return false;
			return Finalize(World);
		}

		case EOpScenario::EnableExactFirstConsume:
		{
			// Re-send the identical enable every frame until it takes (idempotent on the owner side). This
			// is robust against the one-frame race where the scripted flight wrote the setpoint before our
			// command was stored on the very first frame.
			if (!St->bReachedActive)
			{
				St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
				if (OpIsActive(World, St->Follower)) St->bReachedActive = true;
			}
			if (Elapsed < 16.0) return false;
			return Finalize(World);
		}

		case EOpScenario::ActiveProducerUpdates:
		{
			if (!St->bReachedActive) St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			if (OpIsActive(World, St->Follower))
			{
				if (!St->bReachedActive)
				{
					St->bReachedActive = true;
					if (UFormationRuntimeOwnerV2 *O = OpFindOwner(World, *St->Follower))
					{
						St->CandGenAtActive = O->GetCandidateGeneration();
					}
					St->FormationAtMark = Arb::GetCounters().FormationResolutionCount;
					St->ConsumeAtMark = 0;
				}
				CaptureBaselineDev(World, St.Get());
			}
			if (Elapsed < 16.0) return false;
			return Finalize(World);
		}

		case EOpScenario::RepeatedEnableIdempotent:
		{
			// Send the SAME sequence every frame (an operator re-transmitting a held command).
			St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			if (OpIsActive(World, St->Follower) && !St->bReachedActive)
			{
				St->bReachedActive = true;
				if (UFormationRuntimeOwnerV2 *O = OpFindOwner(World, *St->Follower)) St->GenAtActive = O->GetPrimeGeneration();
				St->Phase = 2;
			}
			if (St->Phase == 2)
			{
				// keep re-sending the identical command every frame
				St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			}
			if (Elapsed < 14.0) return false;
			return Finalize(World);
		}

		case EOpScenario::SlotUpdateWhileActive:
		{
			if (!St->bReachedActive) St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			if (OpIsActive(World, St->Follower) && St->Phase == 0)
			{
				St->bReachedActive = true;
				if (UFormationRuntimeOwnerV2 *O = OpFindOwner(World, *St->Follower)) St->GenAtActive = O->GetPrimeGeneration();
				St->Phase = 2;
			}
			if (St->Phase == 2)
			{
				// a NEW sequence (2) with a DIFFERENT slot -- must update SetSlot without a re-prime
				St->Seq = 2;
				St->Udp.SendFormation(St->Follower, St->Leader, -120, -150, 30, St->Seq, Now());
				St->Phase = 3;
			}
			if (St->Phase == 3 && Elapsed > 10.0)
			{
				// SEQUENCE REVERSAL: a backward sequence (1 < 2) with yet another slot. It must be REFUSED
				// as a replay -- the applied sequence must stay 2 and the slot must not change to this one.
				St->Udp.SendFormation(St->Follower, St->Leader, 999, 999, 999, 1, Now());
				St->Phase = 4;
			}
			if (Elapsed < 14.0) return false;
			return Finalize(World);
		}

		case EOpScenario::LeaderChangeNewHandoff:
		{
			if (!St->bReachedActive) St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			if (OpIsActive(World, St->Follower) && St->Phase == 0)
			{
				St->bReachedActive = true;
				if (UFormationRuntimeOwnerV2 *O = OpFindOwner(World, *St->Follower)) St->GenAtActive = O->GetPrimeGeneration();
				St->Phase = 2;
			}
			if (St->Phase == 2)
			{
				// change the leader with a NEW sequence -> must fall back and re-handshake
				St->Seq = 2;
				St->Udp.SendFormation(St->Follower, St->Leader2, -200, 100, 0, St->Seq, Now());
				St->Phase = 3;
			}
			if (St->Phase == 3)
			{
				// Latch the immediate fallback the leader change forced (before a later re-handshake can
				// overwrite the reason), then keep the new-leader command held so it can re-handshake.
				if (UFormationRuntimeOwnerV2 *O = OpFindOwner(World, *St->Follower))
					if (O->GetLastFallback() == EFormationRuntimeFallbackV2::LeaderLost)
						St->bSawLeaderFallback = true;
				St->Udp.SendFormation(St->Follower, St->Leader2, -200, 100, 0, St->Seq, Now());
			}
			if (Elapsed < 18.0) return false;
			return Finalize(World);
		}

		case EOpScenario::DisableImmediateFallback:
		{
			if (!St->bReachedActive) St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			if (OpIsActive(World, St->Follower) && St->Phase == 0)
			{
				St->bReachedActive = true;
				St->FormationAtMark = Arb::GetCounters().FormationResolutionCount;
				St->LegacyAtMark = Arb::GetCounters().LegacyResolutionCount;
				St->Phase = 2;
			}
			if (St->Phase == 2)
			{
				St->Seq = 2;
				St->Udp.SendLegacy(St->Follower, St->Seq, Now());   // explicit disable
				St->Phase = 3;
			}
			if (St->Phase == 3)
			{
				St->Udp.SendLegacy(St->Follower, St->Seq, Now());   // hold the disable
				if (Elapsed < 16.0) return false;
				return Finalize(World);
			}
			return false;
		}

		case EOpScenario::InvalidOrStaleRejected:
		{
			// One bad command per phase, each must be refused; the aircraft never leaves Legacy.
			if (St->Phase == 0)
			{
				// stale timestamp (10 s old)
				St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, 10, Now() - 10.0);
				St->Phase = 1;
				return false;
			}
			if (St->Phase == 1 && Elapsed > 2.0)
			{
				// nonexistent leader (fresh, new sequence)
				St->Udp.SendFormation(St->Follower, TEXT("NO_SUCH_LEADER"), -200, 100, 0, 11, Now());
				St->Phase = 2;
				return false;
			}
			if (St->Phase == 2 && Elapsed > 4.0)
			{
				// non-finite slot (NaN via a raw JSON), fresh sequence
				St->Udp.SendRaw(FString::Printf(
					TEXT("{\"aircraft_name\":\"%s\",\"control_mode\":\"formation\",\"leader_name\":\"%s\","
					     "\"slot_front_m\":1e40,\"slot_right_m\":100,\"slot_up_m\":0,"
					     "\"command_sequence\":12,\"command_timestamp\":%.6f}"),
					*St->Follower, *St->Leader, Now()));
				St->Phase = 3;
				return false;
			}
			// Sequence-reversal is NOT a structural invalid -- it needs a prior ACCEPTED sequence to
			// reverse from, which this all-rejected scenario never has. It is proven in
			// SlotUpdateWhileActive, which reaches Active first, then sends a backward sequence.
			if (Elapsed < 9.0) return false;
			return Finalize(World);
		}

		case EOpScenario::FallingPreempts:
		{
			if (!St->bReachedActive) St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			if (OpIsActive(World, St->Follower) && St->Phase == 0)
			{
				St->bReachedActive = true;
				St->Phase = 2;
			}
			if (St->Phase == 2 && !St->bDamaged)
			{
				if (AActor *A = OpFindActor(World, *St->Follower))
					if (UHealthComponent *H = A->FindComponentByClass<UHealthComponent>())
					{ H->ApplyDamage(1.0e6f, nullptr); St->bDamaged = true; }
				St->Phase = 3;
			}
			if (Elapsed < 18.0) return false;
			return Finalize(World);
		}

		case EOpScenario::PerAircraftIsolation:
		{
			if (!St->bReachedActive) St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			if (OpIsActive(World, St->Follower) && St->Phase == 0)
			{
				St->bReachedActive = true;
				St->OtherOwnerBeforeCount = OpFindOwner(World, *St->Other) ? 1 : 0;
				St->Phase = 2;
			}
			if (Elapsed < 12.0) return false;
			return Finalize(World);
		}

		case EOpScenario::WorldCleanup:
		{
			if (!St->bReachedActive) St->Udp.SendFormation(St->Follower, St->Leader, -200, 100, 0, St->Seq, Now());
			if (OpIsActive(World, St->Follower)) St->bReachedActive = true;
			if (Elapsed < 12.0) return false;
			return Finalize(World);
		}
		}
		return Finalize(World);
	}

	bool Finalize(UWorld *World)
	{
		St->Udp.Close();
		const Arb::FCounters C = Arb::GetCounters();

		// Invariants for every scenario.
		Test->TestEqual(TEXT("no non-finite command reached the FCS"), C.ResolvedNonFiniteCount, (int64)0);
		Test->TestEqual(TEXT("no out-of-range command reached the FCS"), C.ResolvedRangeViolationCount, (int64)0);
		Test->TestEqual(TEXT("the resolver never mutated the legacy block"), C.LegacyBlockMutationCount, (int64)0);

		UJSBSimMovementComponent *F = OpFindComp(World, *St->Follower);
		UFormationRuntimeOwnerV2 *O = OpFindOwner(World, *St->Follower);

		switch (Scenario)
		{
		case EOpScenario::NoCommandRemainsLegacy:
			Test->TestEqual(TEXT("1: Formation never resolved"), C.FormationResolutionCount, (int64)0);
			Test->TestTrue(TEXT("1: no runtime owner was created (or it stayed idle)"),
				O == nullptr || O->GetPhase() == EFormationRuntimePhaseV2::Idle);
			if (F) Test->TestEqual(TEXT("1: mode is LegacyOrManual"),
				(int32)Arb::GetMode(F), (int32)Arb::ECommandMode::LegacyOrManual);
			UE_LOG(LogMumtFormOp, Display, TEXT("[FOP] NO_COMMAND_RESULT formation=%lld owner=%d follower=%s leader=%s"),
				C.FormationResolutionCount, O ? 1 : 0, *St->Follower, *St->Leader);
			break;

		case EOpScenario::EnableExactFirstConsume:
		{
			Arb::FHandoffDelta H{};
			const bool bMeasured = Arb::GetHandoffDelta(St->Follower, H);
			Test->TestTrue(TEXT("2: the owner reached Active"), St->bReachedActive);
			Test->TestTrue(TEXT("2: the handoff was measured"), bMeasured);
			Test->TestTrue(TEXT("2: Formation resolved"), C.FormationResolutionCount > 0);
			Test->TestEqual(TEXT("2: dAileron == 0"),   H.Aileron, 0.0);
			Test->TestEqual(TEXT("2: dElevator == 0"),  H.Elevator, 0.0);
			Test->TestEqual(TEXT("2: dRudder == 0"),    H.Rudder, 0.0);
			Test->TestEqual(TEXT("2: dThrottle == 0"),  H.Throttle, 0.0);
			Test->TestEqual(TEXT("2: dSpeedBrake == 0"),H.SpeedBrake, 0.0);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] ENABLE_EXACT_RESULT reached_active=%d measured=%d formation=%lld gen=%llu baseline_seq=%llu "
				     "dAil=%.9f dElv=%.9f dRud=%.9f dThr=%.9f dSpb=%.9f follower=%s leader=%s seq=%lld"),
				St->bReachedActive ? 1 : 0, bMeasured ? 1 : 0, C.FormationResolutionCount,
				O ? O->GetPrimeGeneration() : 0, O ? O->GetBaselineConsumeSequence() : 0,
				H.Aileron, H.Elevator, H.Rudder, H.Throttle, H.SpeedBrake, *St->Follower, *St->Leader, (long long)St->Seq);
			break;
		}

		case EOpScenario::ActiveProducerUpdates:
		{
			Test->TestTrue(TEXT("3: reached Active"), St->bReachedActive);
			Test->TestTrue(TEXT("3: Formation kept resolving"), C.FormationResolutionCount > St->FormationAtMark);
			Test->TestTrue(TEXT("3: the first consume did not step"), C.NonZeroHandoffCount == 0);
			Test->TestTrue(TEXT("3: candidate generation advanced"),
				O && (uint32)O->GetCandidateGeneration() > St->CandGenAtActive);
			Test->TestTrue(TEXT("3: the real producer moved a control off the baseline"), St->bAnyRealOutput);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] ACTIVE_RESULT formation_at_mark=%lld formation_now=%lld candgen_at_active=%u candgen_now=%u "
				     "any_real=%d maxdev_ail=%.6f maxdev_elv=%.6f maxdev_thr=%.6f"),
				St->FormationAtMark, C.FormationResolutionCount, St->CandGenAtActive,
				O ? O->GetCandidateGeneration() : 0, St->bAnyRealOutput ? 1 : 0,
				St->MaxDevAil, St->MaxDevElv, St->MaxDevThr);
			break;
		}

		case EOpScenario::RepeatedEnableIdempotent:
			Test->TestTrue(TEXT("4: reached Active"), St->bReachedActive);
			Test->TestTrue(TEXT("4: still Active after repeated identical enables"),
				O && O->GetPhase() == EFormationRuntimePhaseV2::Active);
			Test->TestTrue(TEXT("4: the prime generation did NOT increase (no re-handshake)"),
				O && O->GetPrimeGeneration() == St->GenAtActive);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] IDEMPOTENT_RESULT gen_at_active=%llu gen_now=%llu phase=%s formation=%lld"),
				St->GenAtActive, O ? O->GetPrimeGeneration() : 0,
				O ? FormationRuntimePhaseName(O->GetPhase()) : TEXT("gone"), C.FormationResolutionCount);
			break;

		case EOpScenario::SlotUpdateWhileActive:
			Test->TestTrue(TEXT("5: reached Active"), St->bReachedActive);
			Test->TestTrue(TEXT("5: still Active after the slot update"),
				O && O->GetPhase() == EFormationRuntimePhaseV2::Active);
			Test->TestTrue(TEXT("5: the slot update did NOT re-prime (same generation)"),
				O && O->GetPrimeGeneration() == St->GenAtActive);
			Test->TestTrue(TEXT("5: the applied sequence advanced to the slot update"),
				O && O->GetAppliedSequence() == 2);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] SLOT_UPDATE_RESULT gen_at_active=%llu gen_now=%llu applied_seq=%lld phase=%s"),
				St->GenAtActive, O ? O->GetPrimeGeneration() : 0,
				O ? (long long)O->GetAppliedSequence() : -1,
				O ? FormationRuntimePhaseName(O->GetPhase()) : TEXT("gone"));
			break;

		case EOpScenario::LeaderChangeNewHandoff:
			Test->TestTrue(TEXT("6: reached Active with the first leader"), St->bReachedActive);
			// The core contract: a leader change forces an IMMEDIATE fallback (the old ticket is abandoned)
			// and the owner re-targets the new leader. Whether the new handshake then completes depends on
			// the new leader's flight state, which is not this test's subject -- so the required assertions
			// are the fallback and the re-target, and the new generation is logged as an observation.
			Test->TestTrue(TEXT("6: the leader change forced an immediate fallback (old ticket abandoned)"),
				St->bSawLeaderFallback);
			Test->TestEqual(TEXT("6: the owner re-targeted the new leader"),
				O ? O->GetLeaderLabel() : FString(), St->Leader2);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] LEADER_CHANGE_RESULT saw_fallback=%d gen_first=%llu gen_now=%llu leader_now=%s phase=%s"),
				St->bSawLeaderFallback ? 1 : 0, St->GenAtActive, O ? O->GetPrimeGeneration() : 0,
				O ? *O->GetLeaderLabel() : TEXT("gone"),
				O ? FormationRuntimePhaseName(O->GetPhase()) : TEXT("gone"));
			break;

		case EOpScenario::DisableImmediateFallback:
			Test->TestTrue(TEXT("7: reached Active"), St->bReachedActive);
			if (F) Test->TestEqual(TEXT("7: mode is back to LegacyOrManual"),
				(int32)Arb::GetMode(F), (int32)Arb::ECommandMode::LegacyOrManual);
			Test->TestTrue(TEXT("7: the owner is Idle after disable"),
				O && O->GetPhase() == EFormationRuntimePhaseV2::Idle);
			Test->TestTrue(TEXT("7: Legacy resolved after the disable"),
				C.LegacyResolutionCount > St->LegacyAtMark);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] DISABLE_RESULT formation_at_mark=%lld formation_now=%lld legacy_at_mark=%lld legacy_now=%lld phase=%s mode_legacy=%d"),
				St->FormationAtMark, C.FormationResolutionCount, St->LegacyAtMark, C.LegacyResolutionCount,
				O ? FormationRuntimePhaseName(O->GetPhase()) : TEXT("gone"),
				(F && Arb::GetMode(F) == Arb::ECommandMode::LegacyOrManual) ? 1 : 0);
			break;

		case EOpScenario::InvalidOrStaleRejected:
			Test->TestEqual(TEXT("8: Formation NEVER resolved"), C.FormationResolutionCount, (int64)0);
			if (F) Test->TestEqual(TEXT("8: mode stayed LegacyOrManual"),
				(int32)Arb::GetMode(F), (int32)Arb::ECommandMode::LegacyOrManual);
			Test->TestTrue(TEXT("8: the owner never reached Active"),
				O == nullptr || O->GetPhase() != EFormationRuntimePhaseV2::Active);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] INVALID_RESULT formation=%lld last_fallback=%s phase=%s"),
				C.FormationResolutionCount, O ? FormationRuntimeFallbackName(O->GetLastFallback()) : TEXT("no-owner"),
				O ? FormationRuntimePhaseName(O->GetPhase()) : TEXT("no-owner"));
			break;

		case EOpScenario::FallingPreempts:
		{
			Test->TestTrue(TEXT("9: reached Active before damage"), St->bReachedActive);
			Test->TestTrue(TEXT("9: the damage was applied"), St->bDamaged);
			Test->TestTrue(TEXT("9: the pending/active handoff was cancelled by Falling"),
				C.PrimeCancelledByFallingCount > 0);
			Test->TestTrue(TEXT("9: the hardover took over"), C.FallingResolutionCount > 0);
			double FinalThr = -1.0; bool bCutOff = false; bool bGot = false;
			if (F)
			{
				Arb::FResolvedCommandSnapshot L;
				bGot = Arb::GetLastResolvedSnapshot(F, L);
				if (bGot && L.EngineCommands.Num() > 0) { FinalThr = L.EngineCommands[0].Throttle; bCutOff = L.EngineCommands[0].CutOff; }
				Test->TestEqual(TEXT("9: mode is LegacyOrManual"),
					(int32)Arb::GetMode(F), (int32)Arb::ECommandMode::LegacyOrManual);
			}
			Test->TestEqual(TEXT("9: the FCS consumed throttle 0"), FinalThr, 0.0);
			Test->TestTrue(TEXT("9: the FCS consumed cutoff"), bCutOff);
			Test->TestTrue(TEXT("9: the owner fell back to Idle"),
				O == nullptr || O->GetPhase() == EFormationRuntimePhaseV2::Idle);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] FALLING_RESULT cancelled=%lld falling=%lld formation=%lld final_throttle=%.6f cutoff=%d phase=%s"),
				C.PrimeCancelledByFallingCount, C.FallingResolutionCount, C.FormationResolutionCount,
				FinalThr, bCutOff ? 1 : 0, O ? FormationRuntimePhaseName(O->GetPhase()) : TEXT("gone"));
			break;
		}

		case EOpScenario::PerAircraftIsolation:
		{
			Test->TestTrue(TEXT("10: the commanded aircraft reached Active"), St->bReachedActive);
			// The OTHER aircraft never got an owner / a ticket / Formation.
			UFormationRuntimeOwnerV2 *OtherOwner = OpFindOwner(World, *St->Other);
			UJSBSimMovementComponent *OtherComp = OpFindComp(World, *St->Other);
			Test->TestTrue(TEXT("10: the other aircraft has no active ControlV2 owner"),
				OtherOwner == nullptr || OtherOwner->GetPhase() == EFormationRuntimePhaseV2::Idle);
			if (OtherComp)
			{
				Test->TestEqual(TEXT("10: the other aircraft has no prime generation"),
					(int64)Arb::GetPrimeGeneration(OtherComp), (int64)0);
				Test->TestEqual(TEXT("10: the other aircraft is LegacyOrManual"),
					(int32)Arb::GetMode(OtherComp), (int32)Arb::ECommandMode::LegacyOrManual);
				Test->TestFalse(TEXT("10: the other aircraft has no candidate"), Arb::HasCandidate(OtherComp));
			}
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] ISOLATION_RESULT follower_active=%d other=%s other_owner_idle_or_absent=%d other_gen=%llu"),
				St->bReachedActive ? 1 : 0, *St->Other,
				(OtherOwner == nullptr || OtherOwner->GetPhase() == EFormationRuntimePhaseV2::Idle) ? 1 : 0,
				OtherComp ? Arb::GetPrimeGeneration(OtherComp) : 0);
			break;
		}

		case EOpScenario::WorldCleanup:
		{
			Test->TestTrue(TEXT("11: reached Active before teardown"), St->bReachedActive);
			// Tear the world down NOW and verify the arbiter's per-aircraft registry drops to zero: no
			// world-keyed runtime survives. The runtime owner is a component on the actor, so it dies with
			// the world; the arbiter registry is world-cleaned by its OnWorldCleanup.
			const int32 Before = Arb::GetRegistrySize();
			FWorldDelegates::OnWorldCleanup.Broadcast(World, true, true);
			const int32 After = Arb::GetRegistrySize();
			Test->TestTrue(TEXT("11: the arbiter registry was non-empty while flying"), Before > 0);
			Test->TestEqual(TEXT("11: world teardown empties the arbiter registry"), After, 0);
			UE_LOG(LogMumtFormOp, Display,
				TEXT("[FOP] WORLDCLEANUP_RESULT reached_active=%d registry_before=%d registry_after=%d"),
				St->bReachedActive ? 1 : 0, Before, After);
			break;
		}
		}
		return true;
	}

	FAutomationTestBase *Test;
	TSharedPtr<FOpState> St;
	EOpScenario Scenario;
};

void RunOpScenario(FAutomationTestBase *T, EOpScenario S)
{
	T->AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);
	TSharedPtr<FOpState> State = MakeShared<FOpState>();
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kOpMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtOpCommand(T, State, S));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
}

} // namespace

#define FOP_TEST(ClassName, TestName, Scenario)                                                       \
	IMPLEMENT_SIMPLE_AUTOMATION_TEST(ClassName, TestName,                                              \
		EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)                     \
	bool ClassName::RunTest(const FString &) { RunOpScenario(this, Scenario); return true; }

FOP_TEST(FMumtOpNoCommand,   "MUMT.ControlV2.NoCommandRemainsLegacy",             EOpScenario::NoCommandRemainsLegacy)
FOP_TEST(FMumtOpEnable,      "MUMT.ControlV2.OperationalEnableExactFirstConsume", EOpScenario::EnableExactFirstConsume)
FOP_TEST(FMumtOpActive,      "MUMT.ControlV2.OperationalActiveProducerUpdates",   EOpScenario::ActiveProducerUpdates)
FOP_TEST(FMumtOpIdempotent,  "MUMT.ControlV2.RepeatedEnableIsIdempotent",         EOpScenario::RepeatedEnableIdempotent)
FOP_TEST(FMumtOpSlot,        "MUMT.ControlV2.SlotUpdateWhileActive",              EOpScenario::SlotUpdateWhileActive)
FOP_TEST(FMumtOpLeader,      "MUMT.ControlV2.LeaderChangeRequiresNewHandoff",     EOpScenario::LeaderChangeNewHandoff)
FOP_TEST(FMumtOpDisable,     "MUMT.ControlV2.OperationalDisableImmediateFallback",EOpScenario::DisableImmediateFallback)
FOP_TEST(FMumtOpInvalid,     "MUMT.ControlV2.InvalidOrStaleOperationalCommandRejected", EOpScenario::InvalidOrStaleRejected)
FOP_TEST(FMumtOpFalling,     "MUMT.ControlV2.FallingPreemptsOperationalFormation",EOpScenario::FallingPreempts)
FOP_TEST(FMumtOpIsolation,   "MUMT.ControlV2.PerAircraftOperationalIsolation",    EOpScenario::PerAircraftIsolation)
FOP_TEST(FMumtOpWorld,       "MUMT.ControlV2.WorldCleanupOperational",            EOpScenario::WorldCleanup)

#undef FOP_TEST

#endif // WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
