// MumtFormationCandidateIntegrationV2Test.cpp — Phase C.
//
// These tests drive the REAL Formation candidate producer -- FormationPlannerV2 -> NPFG -> TECS ->
// F16StickAdapterV2 -- through the Phase B Prime/handoff contract, in a live PIE world with a real leader
// (M_F16) and follower (F16_UAV1) flying. They are NOT unit tests over injected snapshots: every candidate
// is computed by the actual chain from the actual aircraft state.
//
// What they establish:
//   * the FIRST Formation consume steps by EXACTLY ZERO on all five controlled fields (the stick's baseline
//     latch, armed by the producer, read on the first compute);
//   * from the SECOND frame on the real producer output actually moves the controls (negative control --
//     proof it is a live connection, not a fake that re-emits the baseline forever);
//   * an intervening Legacy consume, a stale candidate, and Falling each refuse the handoff exactly as the
//     arbiter promised, now with a REAL candidate rather than a hand-built one;
//   * ongoing ActiveFormation keeps producing fresh candidates; and Formation -> Legacy is immediate.
//
// TECS continuity is NOT claimed. The exact-zero first consume is the stick latch's guarantee; the second
// frame onward carries whatever transient the real controllers produce, and that transient is measured and
// logged, never asserted to be zero.

#include "Misc/AutomationTest.h"
#include "FormationControlV2/FormationCandidateProducerV2.h"
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
#include "Misc/ScopeExit.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

DEFINE_LOG_CATEGORY_STATIC(LogMumtFormInteg, Display, All);

namespace
{
namespace Arb = MumtCommandArbiterV2;
using FormationControlV2::FFormationCandidateProducerV2;
using FormationControlV2::FProducerFrameResult;
using FormationControlV2::EProducerResult;

const TCHAR *kMap = TEXT("/Game/RL_2");
constexpr double kDtS = 1.0 / 60.0;
constexpr double kSettleS = 2.0;        // let the aircraft trim
constexpr double kWarmS = 1.5;          // warm the shadow chain so the first real candidate is valid
constexpr double kMaxWallSeconds = 320.0;

enum class EScenario : uint8
{
	ExactFirstConsume,       // 1
	NegativeControl,         // 2
	InterveningLegacy,       // 3
	StaleRealCandidate,      // 4
	FallingPreempts,         // 5
	PerAircraftIsolation,    // 6
	ActiveUpdates,           // 7
	ImmediateFallback,       // 8
};

const TCHAR *ScenarioName(EScenario S)
{
	switch (S)
	{
	case EScenario::ExactFirstConsume:    return TEXT("ExactFirstConsume");
	case EScenario::NegativeControl:      return TEXT("NegativeControl");
	case EScenario::InterveningLegacy:    return TEXT("InterveningLegacy");
	case EScenario::StaleRealCandidate:   return TEXT("StaleRealCandidate");
	case EScenario::FallingPreempts:      return TEXT("FallingPreempts");
	case EScenario::PerAircraftIsolation: return TEXT("PerAircraftIsolation");
	case EScenario::ActiveUpdates:        return TEXT("ActiveUpdates");
	case EScenario::ImmediateFallback:    return TEXT("ImmediateFallback");
	default:                              return TEXT("?");
	}
}

UJSBSimMovementComponent *FindAircraft(UWorld *World, const TCHAR *Label, AActor **OutActor = nullptr)
{
	if (!World) return nullptr;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (!It->GetActorNameOrLabel().Contains(Label)) continue;
		if (UJSBSimMovementComponent *C = It->FindComponentByClass<UJSBSimMovementComponent>())
		{
			if (OutActor) *OutActor = *It;
			return C;
		}
	}
	return nullptr;
}

double GetSimTime(UWorld *World)
{
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (UJSBSimMovementComponent *C = It->FindComponentByClass<UJSBSimMovementComponent>())
		{
			FJsbFlightSnapshot S{};
			if (C->GetJsbFlightSnapshot(S) && S.bValidFrame) return S.SimTimeSec;
		}
	}
	return -1.0;
}

uint64 ConsumeSeqOf(const UJSBSimMovementComponent *C)
{
	Arb::FResolvedCommandSnapshot S{};
	return Arb::GetLastResolvedSnapshot(C, S) ? S.ConsumeSequence : 0;
}

struct FIntegState
{
	TSharedPtr<FFormationCandidateProducerV2> Producer;
	FString FollowerName = TEXT("F16_UAV1");
	FString LeaderName   = TEXT("M_F16");

	double FirstWall = -1.0;
	double FirstSim = -1.0;
	bool bMeasuring = false;
	double MeasureStartSim = -1.0;
	int32 Phase = 0;
	uint64 PhaseSeq = 0;

	// captured
	Arb::FPrimeTicket Ticket;
	Arb::FResolvedCommandSnapshot Baseline;
	bool bBaselineCaptured = false;
	bool bClockOverridden = false;
	bool bDamaged = false;

	// negative-control / active-update accumulation
	int32 UpdatesSubmitted = 0;
	int32 UpdateFramesTried = 0;
	double MaxDevAileron = 0.0, MaxDevElevator = 0.0, MaxDevThrottle = 0.0;
	bool bAnyRealOutput = false;
	uint64 FirstUpdateGen = 0, LastUpdateGen = 0;
	int64 FormationConsumesAtActive = 0;
	int64 LegacyConsumesAtFallback = 0;
	bool bFellBack = false;

	FProducerFrameResult LastResult;
};

// ---------------------------------------------------------------------------------------------------
class FMumtIntegCommand : public IAutomationLatentCommand
{
public:
	FMumtIntegCommand(FAutomationTestBase *T, TSharedPtr<FIntegState> S, EScenario Sc)
		: Test(T), St(S), Scenario(Sc) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		if (St->FirstWall < 0) St->FirstWall = NowWall;
		UWorld *World = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
		if (!World) return false;

		const double SimT = GetSimTime(World);
		if (SimT < 0.0) return false;
		if (St->FirstSim < 0.0) St->FirstSim = SimT;

		UJSBSimMovementComponent *F = FindAircraft(World, *St->FollowerName);
		UJSBSimMovementComponent *L = FindAircraft(World, *St->LeaderName);
		if (!F || !L)
		{
			if ((NowWall - St->FirstWall) > 30.0)
			{
				Test->AddError(TEXT("[FCI] leader/follower not found by label"));
				return Finalize(World);
			}
			return false;
		}

		// --- settle, then warm the shadow chain, then reset the arbiter's measurement, then act ---
		const double SinceStart = SimT - St->FirstSim;
		if (!St->bMeasuring)
		{
			if (SinceStart < kSettleS) return false;
			if (SinceStart < kSettleS + kWarmS)
			{
				St->Producer->WarmChain(F, L, kDtS);   // shadow-only: no prime, no submit
				return false;
			}
			St->bMeasuring = true;
			St->MeasureStartSim = SimT;
			Arb::ResetSession(ScenarioName(Scenario));
			return false;
		}
		const double Elapsed = SimT - St->MeasureStartSim;
		if ((NowWall - St->FirstWall) > kMaxWallSeconds)
		{
			Test->AddError(TEXT("[FCI] wall-clock budget exceeded"));
			return Finalize(World);
		}

		return Drive(World, F, L, SimT, Elapsed);
	}

private:
	// Per-scenario state machine. Returns true (done) only via Finalize.
	bool Drive(UWorld *World, UJSBSimMovementComponent *F, UJSBSimMovementComponent *L,
	           double SimT, double Elapsed)
	{
		using EF = Arb::EPrimeFailure;

		switch (Scenario)
		{
		// ---- 1. exact-zero first Formation consume -----------------------------------------------
		case EScenario::ExactFirstConsume:
		{
			if (St->Phase == 0)
			{
				St->LastResult = St->Producer->BeginHandoff(F, L, kDtS);
				St->Ticket.Generation = St->LastResult.Generation;
				St->Ticket.BaselineConsumeSequence = St->LastResult.BaselineConsumeSequence;
				Test->TestEqual(TEXT("1: BeginHandoff Ok"),
					(int32)St->LastResult.Result, (int32)EProducerResult::Ok);
				St->Phase = 1;
				return false;
			}
			// wait for the handoff to be measured (a couple of consumes), then finalize
			if (Elapsed < 2.0) return false;
			return Finalize(World);
		}

		// ---- 2. negative control: real output moves the controls after the latch frame -----------
		case EScenario::NegativeControl:
		{
			if (St->Phase == 0)
			{
				St->LastResult = St->Producer->BeginHandoff(F, L, kDtS);
				Test->TestEqual(TEXT("2: BeginHandoff Ok"),
					(int32)St->LastResult.Result, (int32)EProducerResult::Ok);
				Arb::GetTicketBaseline(F, St->Baseline);
				St->bBaselineCaptured = true;
				St->Phase = 1;
				return false;
			}
			// Once active, keep producing REAL candidates and track how far they move off the baseline.
			if (Arb::GetPrimeState(F) == Arb::EPrimeState::ActiveFormation)
			{
				FProducerFrameResult U = St->Producer->Update(F, L, kDtS);
				++St->UpdateFramesTried;
				if (U.Result == EProducerResult::Ok && U.bSubmitted)
				{
					++St->UpdatesSubmitted;
					St->MaxDevAileron  = FMath::Max(St->MaxDevAileron,  FMath::Abs(U.ChainAileron  - St->Baseline.Commands.Aileron));
					St->MaxDevElevator = FMath::Max(St->MaxDevElevator, FMath::Abs(U.ChainElevator - St->Baseline.Commands.Elevator));
					const double BaseThr = St->Baseline.EngineCommands.Num() > 0 ? St->Baseline.EngineCommands[0].Throttle : 0.0;
					St->MaxDevThrottle = FMath::Max(St->MaxDevThrottle, FMath::Abs(U.ChainThrottle - BaseThr));
					if (St->MaxDevAileron > 1e-4 || St->MaxDevElevator > 1e-4 || St->MaxDevThrottle > 1e-4)
						St->bAnyRealOutput = true;
				}
			}
			if (Elapsed < 4.0) return false;
			return Finalize(World);
		}

		// ---- 3. an intervening Legacy consume between prepare and activation -> refused -----------
		case EScenario::InterveningLegacy:
		{
			if (St->Phase == 0)
			{
				St->LastResult = St->Producer->PrepareHandoff(F, L, kDtS);   // NO activation yet
				Test->TestEqual(TEXT("3: PrepareHandoff Ok"),
					(int32)St->LastResult.Result, (int32)EProducerResult::Ok);
				St->PhaseSeq = ConsumeSeqOf(F);
				St->Phase = 1;
				return false;
			}
			if (St->Phase == 1)
			{
				// let the FDM consume Legacy blocks so the baseline goes stale under the ticket
				if (ConsumeSeqOf(F) < St->PhaseSeq + 5) return false;
				EF Fail = EF::None;
				const bool bAsked = St->Producer->RequestActivation(F, Fail);
				Test->TestTrue(TEXT("3: the activation request is accepted (it only asks)"), bAsked);
				St->Phase = 2;
				return false;
			}
			if (Elapsed < 3.0) return false;
			return Finalize(World);
		}

		// ---- 4. the real candidate is stale by the activation boundary -> refused as StaleCandidate
		case EScenario::StaleRealCandidate:
		{
			if (St->Phase == 0)
			{
				// prepare + make the candidate stale (clock override) + activate, ALL in one tick, so the
				// boundary refusal is StaleCandidate (the candidate rotted) and NOT InterveningConsume
				// (no extra consume slipped in).
				St->LastResult = St->Producer->PrepareHandoff(F, L, kDtS);
				Test->TestEqual(TEXT("4: PrepareHandoff Ok"),
					(int32)St->LastResult.Result, (int32)EProducerResult::Ok);
				const double NowReal = FApp::GetCurrentTime();
				Arb::SetClockOverrideForTesting(NowReal + Arb::kCandidateMaxAgeS * 1.5);
				St->bClockOverridden = true;
				EF Fail = EF::None;
				St->Producer->RequestActivation(F, Fail);
				St->Phase = 1;
				return false;
			}
			if (Elapsed < 3.0) return false;
			return Finalize(World);
		}

		// ---- 5. Falling cancels a real, ready candidate ------------------------------------------
		case EScenario::FallingPreempts:
		{
			if (St->Phase == 0)
			{
				// The candidate must be READY but NOT activated: safety must cancel a pending handoff
				// BEFORE it ever resolves Formation. Activating first would let one Formation consume land
				// before the damage, which is a different (and weaker) claim.
				St->LastResult = St->Producer->PrepareHandoff(F, L, kDtS);
				Test->TestEqual(TEXT("5: PrepareHandoff Ok"),
					(int32)St->LastResult.Result, (int32)EProducerResult::Ok);
				Test->TestEqual(TEXT("5: the candidate is READY (not yet activated)"),
					(int32)Arb::GetPrimeState(F), (int32)Arb::EPrimeState::PrimedCandidateReady);
				St->Ticket.Generation = St->LastResult.Generation;
				St->Ticket.BaselineConsumeSequence = St->LastResult.BaselineConsumeSequence;
				St->Phase = 1;
				return false;
			}
			if (St->Phase == 1 && !St->bDamaged)
			{
				AActor *Actor = nullptr;
				FindAircraft(World, *St->FollowerName, &Actor);
				if (Actor)
				{
					if (UHealthComponent *H = Actor->FindComponentByClass<UHealthComponent>())
					{
						H->ApplyDamage(1.0e6f, nullptr);
						St->bDamaged = true;
					}
				}
				if (!St->bDamaged) { Test->AddError(TEXT("[FCI] 5: no health to damage")); return Finalize(World); }
				St->Phase = 2;
				return false;
			}
			if (Elapsed < 6.0) return false;   // let the hardover own several consumes
			return Finalize(World);
		}

		// ---- 6. per-aircraft isolation: priming the follower leaves the others untouched ---------
		case EScenario::PerAircraftIsolation:
		{
			if (St->Phase == 0)
			{
				St->LastResult = St->Producer->BeginHandoff(F, L, kDtS);
				Test->TestEqual(TEXT("6: BeginHandoff Ok"),
					(int32)St->LastResult.Result, (int32)EProducerResult::Ok);
				St->Phase = 1;
				return false;
			}
			if (Elapsed < 1.5) return false;
			return Finalize(World);
		}

		// ---- 7. ActiveFormation keeps producing fresh candidates ---------------------------------
		case EScenario::ActiveUpdates:
		{
			if (St->Phase == 0)
			{
				St->LastResult = St->Producer->BeginHandoff(F, L, kDtS);
				Test->TestEqual(TEXT("7: BeginHandoff Ok"),
					(int32)St->LastResult.Result, (int32)EProducerResult::Ok);
				Arb::GetTicketBaseline(F, St->Baseline);
				St->Phase = 1;
				return false;
			}
			if (Arb::GetPrimeState(F) == Arb::EPrimeState::ActiveFormation)
			{
				if (St->FormationConsumesAtActive == 0)
					St->FormationConsumesAtActive = Arb::GetCounters().FormationResolutionCount;
				FProducerFrameResult U = St->Producer->Update(F, L, kDtS);
				++St->UpdateFramesTried;
				if (U.Result == EProducerResult::Ok && U.bSubmitted)
				{
					++St->UpdatesSubmitted;
					if (St->FirstUpdateGen == 0) St->FirstUpdateGen = U.CandidateGeneration;
					St->LastUpdateGen = U.CandidateGeneration;
					const double BaseThr = St->Baseline.EngineCommands.Num() > 0 ? St->Baseline.EngineCommands[0].Throttle : 0.0;
					if (FMath::Abs(U.ChainAileron - St->Baseline.Commands.Aileron) > 1e-4
					 || FMath::Abs(U.ChainElevator - St->Baseline.Commands.Elevator) > 1e-4
					 || FMath::Abs(U.ChainThrottle - BaseThr) > 1e-4)
						St->bAnyRealOutput = true;
				}
			}
			if (Elapsed < 4.0) return false;
			return Finalize(World);
		}

		// ---- 8. immediate Formation -> Legacy fallback -------------------------------------------
		case EScenario::ImmediateFallback:
		{
			if (St->Phase == 0)
			{
				St->LastResult = St->Producer->BeginHandoff(F, L, kDtS);
				Test->TestEqual(TEXT("8: BeginHandoff Ok"),
					(int32)St->LastResult.Result, (int32)EProducerResult::Ok);
				St->Phase = 1;
				return false;
			}
			if (St->Phase == 1)
			{
				// keep it active for a few consumes, submitting fresh candidates
				if (Arb::GetPrimeState(F) == Arb::EPrimeState::ActiveFormation)
				{
					St->Producer->Update(F, L, kDtS);
					if (Arb::GetCounters().FormationResolutionCount >= 3)
					{
						St->FormationConsumesAtActive = Arb::GetCounters().FormationResolutionCount;
						St->LegacyConsumesAtFallback = Arb::GetCounters().LegacyResolutionCount;
						Arb::RequestLegacyFallback(F);   // the immediate safety fallback
						St->bFellBack = true;
						St->PhaseSeq = ConsumeSeqOf(F);
						St->Phase = 2;
					}
				}
				return false;
			}
			if (St->Phase == 2)
			{
				// after fallback, do NOT submit anything more; verify the next consumes are Legacy
				if (ConsumeSeqOf(F) < St->PhaseSeq + 4) return false;
				return Finalize(World);
			}
			return false;
		}
		}
		return Finalize(World);
	}

	bool Finalize(UWorld *World)
	{
		ON_SCOPE_EXIT { if (St->bClockOverridden) { Arb::ClearClockOverrideForTesting(); St->bClockOverridden = false; } };

		const Arb::FCounters C = Arb::GetCounters();
		UJSBSimMovementComponent *F = FindAircraft(World, *St->FollowerName);

		// Invariants that hold for every scenario.
		Test->TestEqual(TEXT("no non-finite command ever reached the FCS"), C.ResolvedNonFiniteCount, (int64)0);
		Test->TestEqual(TEXT("no out-of-range command reached the FCS"), C.ResolvedRangeViolationCount, (int64)0);
		Test->TestEqual(TEXT("the resolver never mutated the legacy block"), C.LegacyBlockMutationCount, (int64)0);
		Test->TestEqual(TEXT("no consume bypassed the resolver"), C.MissingResolutionCount, (int64)0);

		switch (Scenario)
		{
		case EScenario::ExactFirstConsume:
		{
			Arb::FHandoffDelta H{};
			const bool bMeasured = Arb::GetHandoffDelta(St->FollowerName, H);
			Test->TestTrue(TEXT("1: the handoff was measured"), bMeasured);
			Test->TestTrue(TEXT("1: Formation resolved at least once"), C.FormationResolutionCount > 0);
			// THE CLAIM: exactly zero on all five controlled fields.
			Test->TestEqual(TEXT("1: dAileron == 0"),   H.Aileron, 0.0);
			Test->TestEqual(TEXT("1: dElevator == 0"),  H.Elevator, 0.0);
			Test->TestEqual(TEXT("1: dRudder == 0"),    H.Rudder, 0.0);
			Test->TestEqual(TEXT("1: dThrottle == 0"),  H.Throttle, 0.0);
			Test->TestEqual(TEXT("1: dSpeedBrake == 0"),H.SpeedBrake, 0.0);
			UE_LOG(LogMumtFormInteg, Display,
				TEXT("[FCI] EXACT_FIRST_RESULT measured=%d formation_resolutions=%lld dAil=%.9f dElv=%.9f dRud=%.9f dThr=%.9f dSpb=%.9f"),
				bMeasured ? 1 : 0, C.FormationResolutionCount, H.Aileron, H.Elevator, H.Rudder, H.Throttle, H.SpeedBrake);
			break;
		}

		case EScenario::NegativeControl:
		{
			Test->TestTrue(TEXT("2: the first Formation consume still stepped by zero"),
				C.NonZeroHandoffCount == 0);
			Test->TestTrue(TEXT("2: real candidates were submitted after activation"), St->UpdatesSubmitted > 0);
			// THE POINT: the real producer moved the controls off the baseline. A fake connection that
			// re-emitted the baseline forever would leave every deviation at zero.
			Test->TestTrue(TEXT("2: the real producer output moved a control field off the baseline"),
				St->bAnyRealOutput);
			UE_LOG(LogMumtFormInteg, Display,
				TEXT("[FCI] NEG_CONTROL_RESULT submitted=%d frames=%d any_real=%d maxdev_ail=%.6f maxdev_elv=%.6f maxdev_thr=%.6f first_step_nonzero=%lld"),
				St->UpdatesSubmitted, St->UpdateFramesTried, St->bAnyRealOutput ? 1 : 0,
				St->MaxDevAileron, St->MaxDevElevator, St->MaxDevThrottle, C.NonZeroHandoffCount);
			break;
		}

		case EScenario::InterveningLegacy:
		{
			Test->TestTrue(TEXT("3: the stale baseline was caught at the consume boundary"),
				C.InterveningConsumeRejectedCount > 0);
			Test->TestEqual(TEXT("3: Formation NEVER resolved"), C.FormationResolutionCount, (int64)0);
			Test->TestEqual(TEXT("3: no handoff was measured"), C.HandoffsMeasured, (int64)0);
			if (F)
			{
				Test->TestEqual(TEXT("3: the reason was InterveningConsume"),
					(int32)Arb::GetLastBoundaryFailure(F), (int32)Arb::EPrimeFailure::InterveningConsume);
				Test->TestEqual(TEXT("3: the aircraft stayed on Legacy"),
					(int32)Arb::GetMode(F), (int32)Arb::ECommandMode::LegacyOrManual);
				Test->TestEqual(TEXT("3: the dead ticket was discarded"),
					(int32)Arb::GetPrimeState(F), (int32)Arb::EPrimeState::IdleLegacy);
			}
			UE_LOG(LogMumtFormInteg, Display,
				TEXT("[FCI] INTERVENING_RESULT intervening=%lld formation=%lld handoffs=%lld"),
				C.InterveningConsumeRejectedCount, C.FormationResolutionCount, C.HandoffsMeasured);
			break;
		}

		case EScenario::StaleRealCandidate:
		{
			Test->TestEqual(TEXT("4: the stale candidate was caught at the boundary"),
				C.ActivationStaleRejectedCount, (int64)1);
			Test->TestEqual(TEXT("4: and NOT mistaken for an intervening consume"),
				C.InterveningConsumeRejectedCount, (int64)0);
			Test->TestEqual(TEXT("4: Formation NEVER resolved"), C.FormationResolutionCount, (int64)0);
			if (F)
			{
				Test->TestEqual(TEXT("4: the reason was StaleCandidate"),
					(int32)Arb::GetLastBoundaryFailure(F), (int32)Arb::EPrimeFailure::StaleCandidate);
				Test->TestEqual(TEXT("4: the aircraft stayed on Legacy"),
					(int32)Arb::GetMode(F), (int32)Arb::ECommandMode::LegacyOrManual);
			}
			UE_LOG(LogMumtFormInteg, Display,
				TEXT("[FCI] STALE_RESULT stale_at_boundary=%lld intervening=%lld formation=%lld"),
				C.ActivationStaleRejectedCount, C.InterveningConsumeRejectedCount, C.FormationResolutionCount);
			break;
		}

		case EScenario::FallingPreempts:
		{
			Test->TestTrue(TEXT("5: the aircraft was damaged"), St->bDamaged);
			Test->TestTrue(TEXT("5: the pending handoff was cancelled by Falling"),
				C.PrimeCancelledByFallingCount > 0);
			Test->TestEqual(TEXT("5: Formation NEVER resolved -- safety outranks bumpless"),
				C.FormationResolutionCount, (int64)0);
			Test->TestTrue(TEXT("5: the hardover took over"), C.FallingResolutionCount > 0);
			double FinalThr = -1.0; bool bCutOff = false; bool bGot = false;
			if (F)
			{
				Arb::FResolvedCommandSnapshot Last;
				bGot = Arb::GetLastResolvedSnapshot(F, Last);
				if (bGot && Last.EngineCommands.Num() > 0)
				{
					FinalThr = Last.EngineCommands[0].Throttle;
					bCutOff = Last.EngineCommands[0].CutOff;
				}
				Test->TestEqual(TEXT("5: prime state back to IdleLegacy"),
					(int32)Arb::GetPrimeState(F), (int32)Arb::EPrimeState::IdleLegacy);
				Test->TestFalse(TEXT("5: candidate invalidated"), Arb::HasCandidate(F));
				Test->TestEqual(TEXT("5: mode back to LegacyOrManual"),
					(int32)Arb::GetMode(F), (int32)Arb::ECommandMode::LegacyOrManual);
			}
			Test->TestTrue(TEXT("5: the consumed block was readable"), bGot);
			Test->TestEqual(TEXT("5: the FCS consumed throttle 0"), FinalThr, 0.0);
			Test->TestTrue(TEXT("5: the FCS consumed cutoff"), bCutOff);
			UE_LOG(LogMumtFormInteg, Display,
				TEXT("[FCI] FALLING_RESULT cancelled=%lld formation=%lld falling=%lld final_throttle=%.6f cutoff=%d"),
				C.PrimeCancelledByFallingCount, C.FormationResolutionCount, C.FallingResolutionCount,
				FinalThr, bCutOff ? 1 : 0);
			break;
		}

		case EScenario::PerAircraftIsolation:
		{
			Test->TestTrue(TEXT("6: the follower has a prime generation"), Arb::GetPrimeGeneration(F) > 0);
			// its snapshot / ticket baseline belong to IT
			Arb::FResolvedCommandSnapshot Own, Base;
			if (F)
			{
				Test->TestTrue(TEXT("6: follower has its own snapshot"), Arb::GetLastResolvedSnapshot(F, Own));
				Test->TestTrue(TEXT("6: ...owned by itself"), Own.Component.Get() == F);
				Test->TestTrue(TEXT("6: ...in its own world"), Own.World.Get() == F->GetWorld());
				Test->TestTrue(TEXT("6: follower has a ticket baseline"), Arb::GetTicketBaseline(F, Base));
				Test->TestTrue(TEXT("6: ...owned by itself"), Base.Component.Get() == F);
			}
			// the OTHERS: no ticket, no candidate, still Legacy
			int32 OthersChecked = 0;
			for (const TCHAR *Label : { TEXT("M_F16"), TEXT("F16_UAV2") })
			{
				UJSBSimMovementComponent *O = FindAircraft(World, Label);
				if (!O || O == F) continue;
				++OthersChecked;
				Test->TestEqual(TEXT("6: another aircraft has NO prime generation"),
					(int64)Arb::GetPrimeGeneration(O), (int64)0);
				Test->TestEqual(TEXT("6: another aircraft is IdleLegacy"),
					(int32)Arb::GetPrimeState(O), (int32)Arb::EPrimeState::IdleLegacy);
				Test->TestFalse(TEXT("6: another aircraft has NO candidate"), Arb::HasCandidate(O));
				Test->TestEqual(TEXT("6: another aircraft is still LegacyOrManual"),
					(int32)Arb::GetMode(O), (int32)Arb::ECommandMode::LegacyOrManual);
				Arb::FResolvedCommandSnapshot NoBase;
				Test->TestFalse(TEXT("6: another aircraft has NO ticket baseline"),
					Arb::GetTicketBaseline(O, NoBase));
			}
			Test->TestTrue(TEXT("6: at least one other aircraft was checked"), OthersChecked > 0);
			UE_LOG(LogMumtFormInteg, Display,
				TEXT("[FCI] ISOLATION_RESULT follower_gen=%llu others_checked=%d"),
				Arb::GetPrimeGeneration(F), OthersChecked);
			break;
		}

		case EScenario::ActiveUpdates:
		{
			Test->TestTrue(TEXT("7: real candidates were submitted while active"), St->UpdatesSubmitted > 1);
			Test->TestTrue(TEXT("7: the candidate generation advanced (not one frozen candidate)"),
				St->LastUpdateGen > St->FirstUpdateGen);
			Test->TestTrue(TEXT("7: Formation kept resolving as new candidates arrived"),
				C.FormationResolutionCount > St->FormationConsumesAtActive);
			Test->TestTrue(TEXT("7: the producer did NOT just re-emit the baseline forever"),
				St->bAnyRealOutput);
			if (F)
				Test->TestEqual(TEXT("7: still ActiveFormation at the end"),
					(int32)Arb::GetPrimeState(F), (int32)Arb::EPrimeState::ActiveFormation);
			UE_LOG(LogMumtFormInteg, Display,
				TEXT("[FCI] ACTIVE_RESULT submitted=%d first_gen=%llu last_gen=%llu formation_at_active=%lld formation_now=%lld any_real=%d"),
				St->UpdatesSubmitted, St->FirstUpdateGen, St->LastUpdateGen,
				St->FormationConsumesAtActive, C.FormationResolutionCount, St->bAnyRealOutput ? 1 : 0);
			break;
		}

		case EScenario::ImmediateFallback:
		{
			Test->TestTrue(TEXT("8: it reached ActiveFormation and resolved Formation"),
				St->FormationConsumesAtActive > 0);
			Test->TestTrue(TEXT("8: the fallback was requested"), St->bFellBack);
			if (F)
			{
				Test->TestEqual(TEXT("8: mode is back to LegacyOrManual"),
					(int32)Arb::GetMode(F), (int32)Arb::ECommandMode::LegacyOrManual);
				Test->TestEqual(TEXT("8: prime state back to IdleLegacy"),
					(int32)Arb::GetPrimeState(F), (int32)Arb::EPrimeState::IdleLegacy);
				Test->TestFalse(TEXT("8: no candidate held"), Arb::HasCandidate(F));
			}
			// after the fallback, the consumes are Legacy: Formation stopped growing, Legacy grew.
			Test->TestEqual(TEXT("8: Formation stopped resolving after fallback"),
				C.FormationResolutionCount, St->FormationConsumesAtActive);
			Test->TestTrue(TEXT("8: Legacy resolved after the fallback"),
				C.LegacyResolutionCount > St->LegacyConsumesAtFallback);
			UE_LOG(LogMumtFormInteg, Display,
				TEXT("[FCI] FALLBACK_RESULT formation_at_fallback=%lld formation_now=%lld legacy_at_fallback=%lld legacy_now=%lld mode_legacy=%d"),
				St->FormationConsumesAtActive, C.FormationResolutionCount,
				St->LegacyConsumesAtFallback, C.LegacyResolutionCount,
				(F && Arb::GetMode(F) == Arb::ECommandMode::LegacyOrManual) ? 1 : 0);
			break;
		}
		}
		return true;
	}

	FAutomationTestBase *Test;
	TSharedPtr<FIntegState> St;
	EScenario Scenario;
};

void RunScenario(FAutomationTestBase *T, EScenario S)
{
	// Pre-existing, unrelated Blueprint Construction Script warning in the formation aircraft. Matched
	// narrowly so it cannot mask a real command error.
	T->AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);

	TSharedPtr<FIntegState> State = MakeShared<FIntegState>();
	State->Producer = MakeShared<FFormationCandidateProducerV2>();

	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtIntegCommand(T, State, S));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
}

} // namespace

#define FCI_TEST(ClassName, TestName, Scenario)                                                       \
	IMPLEMENT_SIMPLE_AUTOMATION_TEST(ClassName, TestName,                                              \
		EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)                     \
	bool ClassName::RunTest(const FString &) { RunScenario(this, Scenario); return true; }

FCI_TEST(FMumtFciExactFirst,   "MUMT.ControlV2.RealProducerHandoffExactFirstConsume", EScenario::ExactFirstConsume)
FCI_TEST(FMumtFciNegative,     "MUMT.ControlV2.RealProducerNegativeControl",          EScenario::NegativeControl)
FCI_TEST(FMumtFciIntervening,  "MUMT.ControlV2.InterveningLegacyConsumeRejected",     EScenario::InterveningLegacy)
FCI_TEST(FMumtFciStale,        "MUMT.ControlV2.StaleRealCandidateRejected",           EScenario::StaleRealCandidate)
FCI_TEST(FMumtFciFalling,      "MUMT.ControlV2.FallingPreemptsRealCandidate",         EScenario::FallingPreempts)
FCI_TEST(FMumtFciIsolation,    "MUMT.ControlV2.PerAircraftAndWorldIsolation",         EScenario::PerAircraftIsolation)
FCI_TEST(FMumtFciActive,       "MUMT.ControlV2.ActiveFormationUpdates",               EScenario::ActiveUpdates)
FCI_TEST(FMumtFciFallback,     "MUMT.ControlV2.ImmediateFormationToLegacyFallback",   EScenario::ImmediateFallback)

#undef FCI_TEST

#endif // WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
