// MumtCommandPrimeV2Test.cpp — the bumpless Legacy -> Formation handoff (Phase B).
//
// The command a handoff must be continuous with is the FINAL RESOLVED BLOCK the FDM last consumed --
// not the legacy writer block, which is only an input and can be a shifting field-wise mixture of
// several writers. Everything here is anchored to that.
//
// The claim Phase B proves is exactly one thing, and it is stated narrowly on purpose:
//
//     LegacyOrManual -> FormationControlV2, the FIRST handoff, has zero command step.
//
// The reverse direction is NOT claimed to be bumpless. Formation -> Legacy is an immediate safety
// fallback: it snaps back to whatever the legacy writers are producing right now, because a controller
// that has gone stale or non-finite must be abandoned instantly, not blended out of.
#include "Misc/AutomationTest.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

#include "State/MumtCommandArbiterV2.h"
#include "FormationControlV2/F16StickAdapterV2.h"
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

#include <limits>

DEFINE_LOG_CATEGORY_STATIC(LogMumtPrimeTest, Display, All);

namespace
{
namespace Arb = MumtCommandArbiterV2;

const TCHAR *kPrimeMap = TEXT("/Game/RL_2");
constexpr double kPrimeMaxWallSeconds = 300.0;
constexpr double kPrimeSettleS = 1.5;
constexpr double kPrimeDamageAtS = 8.0;

enum class EPrimeScenario : uint8
{
	NoResolvedSnapshot,   // A
	SnapshotExact,        // B
	Generation,           // C
	ExactHandoff,         // D
	UnprimedRejected,     // E
	WrongGeneration,      // F
	Stale,                // G
	NonFinite,            // H
	FallingPreemption,    // I
	PerAircraftIsolation, // J
	WorldCleanup,         // K
	ResetSessionSafety,   // L
	ResolverOwnershipLost,// M
	InterveningConsume,   // N
	StickExactFirstCompute,// O
	HandoffDeltaNegative, // P
	StickWrongIdentity,   // Q
	StickLatchedNoAdvance,// R
	StickInvalidPrime,    // S
	ActivationBoundaryStale,// T
};

const TCHAR *PrimeScenarioName(EPrimeScenario S)
{
	switch (S)
	{
	case EPrimeScenario::NoResolvedSnapshot:    return TEXT("A_NoResolvedSnapshot");
	case EPrimeScenario::SnapshotExact:         return TEXT("B_SnapshotExact");
	case EPrimeScenario::Generation:            return TEXT("C_Generation");
	case EPrimeScenario::ExactHandoff:          return TEXT("D_ExactHandoff");
	case EPrimeScenario::UnprimedRejected:      return TEXT("E_UnprimedRejected");
	case EPrimeScenario::WrongGeneration:       return TEXT("F_WrongGeneration");
	case EPrimeScenario::Stale:                 return TEXT("G_Stale");
	case EPrimeScenario::NonFinite:             return TEXT("H_NonFinite");
	case EPrimeScenario::FallingPreemption:     return TEXT("I_FallingPreemption");
	case EPrimeScenario::PerAircraftIsolation:  return TEXT("J_PerAircraftIsolation");
	case EPrimeScenario::WorldCleanup:          return TEXT("K_WorldCleanup");
	case EPrimeScenario::ResetSessionSafety:    return TEXT("L_ResetSessionSafety");
	case EPrimeScenario::ResolverOwnershipLost: return TEXT("M_ResolverOwnershipLost");
	case EPrimeScenario::InterveningConsume:    return TEXT("N_InterveningConsume");
	case EPrimeScenario::StickExactFirstCompute:return TEXT("O_StickExactFirstCompute");
	case EPrimeScenario::HandoffDeltaNegative:  return TEXT("P_HandoffDeltaNegative");
	case EPrimeScenario::StickWrongIdentity:    return TEXT("Q_StickWrongIdentity");
	case EPrimeScenario::StickLatchedNoAdvance: return TEXT("R_StickLatchedNoAdvance");
	default:                                    return TEXT("S_StickInvalidPrime");
	}
}

UJSBSimMovementComponent *FindPrimeAircraft(UWorld *World, const TCHAR *Label, AActor **OutActor = nullptr)
{
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

double GetPrimeSimTime(UWorld *World)
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

// Builds a candidate that reproduces the baseline exactly. This is what makes the handoff provable:
// if the first Formation command equals what was already being consumed, there is nothing to step.
Arb::FPrimedCandidate CandidateFromBaseline(const Arb::FPrimeTicket &T)
{
	Arb::FPrimedCandidate P;
	P.PrimeGeneration = T.Generation;
	P.BaselineConsumeSequence = T.BaselineConsumeSequence;
	P.Candidate.AileronNorm    = T.Baseline.Commands.Aileron;
	P.Candidate.ElevatorNorm   = T.Baseline.Commands.Elevator;
	P.Candidate.RudderNorm     = T.Baseline.Commands.Rudder;
	P.Candidate.SpeedBrakeNorm = T.Baseline.Commands.SpeedBrake;
	P.Candidate.ThrottleNorm   = T.Baseline.EngineCommands.Num() > 0
		? T.Baseline.EngineCommands[0].Throttle : 0.0;
	P.Candidate.Generation = T.Generation;
	P.Candidate.bValid = true;
	P.Candidate.bCommandReady = true;
	P.Candidate.bFinite = true;
	P.Candidate.TimestampS = FApp::GetCurrentTime();
	return P;
}

// A foreign resolver for scenario M. It deliberately changes nothing.
void PrimeForeignResolver(const UJSBSimMovementComponent *, uint64, const FFlightControlCommands &,
                          const TArray<FEngineCommand> &, FJSBSimResolvedCommandBlock &) {}

struct FPrimeState
{
	double FirstWall = 0.0;
	double FirstSim = -1.0;
	double MeasureStartSim = -1.0;
	bool bMeasuring = false;
	bool bStarted = false;
	bool bActed = false;
	bool bDamaged = false;
	FString TargetName;
	UWorld *OtherWorld = nullptr;
	// N: the activation is deliberately delayed so a Legacy consume can slip in between.
	bool bPrimedForIntervening = false;
	bool bLateActivated = false;
	int32 InterveningTicks = 0;
	Arb::FPrimeTicket StaleTicket;
	double OffsetAileron = 0.0;   // P: the deliberate step the negative control must SEE
	bool bClockOverridden = false;// T: the arbiter's clock is frozen forward; it MUST be released
};

class FMumtPrimeCommand : public IAutomationLatentCommand
{
public:
	FMumtPrimeCommand(FAutomationTestBase *T, TSharedPtr<FPrimeState> S, EPrimeScenario Sc)
		: Test(T), St(S), Scenario(Sc) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		UWorld *World = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
		if (!World) return false;

		if (!St->bStarted)
		{
			St->bStarted = true;
			St->FirstWall = NowWall;
			// The resolver is bound by the MODULE. If a test had to switch it on, none of this would say
			// anything about production.
			Test->TestTrue(TEXT("PRIME: the resolver is bound in production"), Arb::IsEnabled());
		}

		const double SimT = GetPrimeSimTime(World);
		if (SimT >= 0.0 && St->FirstSim < 0.0) St->FirstSim = SimT;
		const double SinceStart = (St->FirstSim >= 0.0 && SimT >= 0.0) ? (SimT - St->FirstSim) : 0.0;

		if (!St->bMeasuring)
		{
			if (SinceStart < kPrimeSettleS) return false;
			St->bMeasuring = true;
			St->MeasureStartSim = SimT;
			Arb::ResetSession(PrimeScenarioName(Scenario));
			return false;
		}
		const double Elapsed = SimT - St->MeasureStartSim;

		if (!St->bActed)
		{
			St->bActed = true;
			if (!Act(World)) return Finalize();
			// Falling scenarios need real flight time after the act; everything else can settle briefly.
			return false;
		}

		// N: let the FDM consume a few Legacy blocks, THEN try to activate on the now-stale ticket.
		if (Scenario == EPrimeScenario::InterveningConsume && St->bPrimedForIntervening && !St->bLateActivated)
		{
			if (++St->InterveningTicks < 20) return false;   // consumes happen between latent ticks
			UJSBSimMovementComponent *T2 = FindPrimeAircraft(World, TEXT("F16_UAV1"));
			if (T2)
			{
				Arb::EPrimeFailure F2 = Arb::EPrimeFailure::None;
				// The activation request itself is accepted -- it only ASKS. The refusal happens at the
				// consume boundary, which is the only place the stale baseline is visible.
				Test->TestTrue(TEXT("N: the activation request is accepted (it only asks)"),
					Arb::ActivateFormationForTesting(T2, F2));
			}
			St->bLateActivated = true;
			return false;
		}

		if (Scenario == EPrimeScenario::FallingPreemption && !St->bDamaged && Elapsed >= kPrimeDamageAtS)
		{
			AActor *Actor = nullptr;
			if (FindPrimeAircraft(World, TEXT("F16_UAV1"), &Actor) && Actor)
			{
				if (UHealthComponent *H = Actor->FindComponentByClass<UHealthComponent>())
				{
					H->ApplyDamage(1.0e6f, nullptr);
					St->bDamaged = true;
					UE_LOG(LogMumtPrimeTest, Display, TEXT("[PRIME] scenario I: damaged %s at t=%.2f"),
						*Actor->GetActorNameOrLabel(), SimT);
				}
			}
			if (!St->bDamaged)
			{
				Test->AddError(TEXT("[PRIME] scenario I: no damageable F16_UAV1"));
				return Finalize();
			}
			return false;
		}

		const double Need = (Scenario == EPrimeScenario::FallingPreemption) ? 16.0 : 3.0;
		if (Elapsed < Need && (NowWall - St->FirstWall) < kPrimeMaxWallSeconds) return false;
		return Finalize();
	}

private:
	// Everything that must happen without a consume slipping in between is done in ONE Update() call.
	// Latent commands run on the editor tick; the PIE world consumes between them. Splitting
	// RequestPrime / SubmitPrimedCandidate / Activate across ticks would let the baseline go stale under
	// the ticket and the handoff would step -- through no fault of the arbiter.
	bool Act(UWorld *World)
	{
		using EF = Arb::EPrimeFailure;
		UJSBSimMovementComponent *Target = FindPrimeAircraft(World, TEXT("F16_UAV1"));
		if (!Target)
		{
			Test->AddError(TEXT("[PRIME] no F16_UAV1"));
			return false;
		}
		St->TargetName = TEXT("F16_UAV1");

		Arb::FPrimeTicket Ticket;
		EF Fail = EF::None;

		switch (Scenario)
		{
		case EPrimeScenario::NoResolvedSnapshot:
		{
			// An aircraft that has never been consumed has no baseline to be continuous with. Built in a
			// second world so it can never be reached by the PIE consume path.
			St->OtherWorld = UWorld::CreateWorld(EWorldType::Game, false);
			AActor *A = St->OtherWorld ? St->OtherWorld->SpawnActor<AActor>() : nullptr;
			UJSBSimMovementComponent *Fresh = A ? NewObject<UJSBSimMovementComponent>(A) : nullptr;
			if (!Fresh) { Test->AddError(TEXT("[PRIME] A: could not build a fresh component")); return false; }

			const bool bOk = Arb::RequestPrime(Fresh, Ticket, Fail);
			Test->TestFalse(TEXT("A: priming an aircraft with no consumed command fails"), bOk);
			Test->TestEqual(TEXT("A: and the reason is NoResolvedSnapshot"),
				static_cast<int32>(Fail), static_cast<int32>(EF::NoResolvedSnapshot));
			Test->TestEqual(TEXT("A: the mode is untouched"),
				static_cast<int32>(Arb::GetMode(Fresh)), static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
			break;
		}

		case EPrimeScenario::SnapshotExact:
		{
			Arb::FResolvedCommandSnapshot Last;
			Test->TestTrue(TEXT("B: a consumed command exists"), Arb::GetLastResolvedSnapshot(Target, Last));
			Test->TestTrue(TEXT("B: the prime is granted"), Arb::RequestPrime(Target, Ticket, Fail));

			// All 15 + 14 fields, field by field. A hash would hide a dropped field behind padding.
			Test->TestEqual(TEXT("B: baseline consume sequence matches"),
				(int64)Ticket.BaselineConsumeSequence, (int64)Last.ConsumeSequence);
			Test->TestEqual(TEXT("B: snapshot == last resolved (changed fields)"),
				CountDiff(Ticket.Baseline, Last), 0);

			// WHOSE command is it? A snapshot without an owner is just five numbers: it would prime happily
			// against a block another aircraft, or another world, actually flew.
			Test->TestTrue(TEXT("B: the snapshot is owned by THIS component"), Last.Component.Get() == Target);
			Test->TestTrue(TEXT("B: ...in THIS world"), Last.World.Get() == Target->GetWorld());
			Test->TestTrue(TEXT("B: the ticket baseline carries the same owner"),
				Ticket.Baseline.Component.Get() == Target);
			Test->TestTrue(TEXT("B: ...and the same world"),
				Ticket.Baseline.World.Get() == Target->GetWorld());

			// The legacy writer block keeps moving underneath. The snapshot must not.
			const double Before = Ticket.Baseline.Commands.Aileron;
			Target->Commands.Aileron = Before + 0.5;   // a writer would do exactly this
			Arb::FPrimeTicket Again;
			Arb::FResolvedCommandSnapshot Now;
			Arb::GetLastResolvedSnapshot(Target, Now);
			Test->TestEqual(TEXT("B: mutating the legacy block does NOT move the taken snapshot"),
				Ticket.Baseline.Commands.Aileron, Before);
			(void)Again; (void)Now;
			break;
		}

		case EPrimeScenario::Generation:
		{
			Test->TestTrue(TEXT("C: first prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			const uint32 G1 = Ticket.Generation;

			Arb::FPrimeTicket T2;
			Test->TestTrue(TEXT("C: second prime granted"), Arb::RequestPrime(Target, T2, Fail));
			Test->TestEqual(TEXT("C: the generation is strictly monotonic"), (int64)T2.Generation, (int64)G1 + 1);

			// The old ticket is dead the instant a new prime is issued.
			Arb::FPrimedCandidate Old = CandidateFromBaseline(Ticket);
			Old.PrimeGeneration = G1;
			Test->TestFalse(TEXT("C: a candidate from the superseded generation is refused"),
				Arb::SubmitPrimedCandidate(Target, Old, Fail));
			Test->TestEqual(TEXT("C: and the reason is WrongGeneration"),
				static_cast<int32>(Fail), static_cast<int32>(EF::WrongGeneration));

			Test->TestTrue(TEXT("C: a candidate from the current generation is accepted"),
				Arb::SubmitPrimedCandidate(Target, CandidateFromBaseline(T2), Fail));
			break;
		}

		case EPrimeScenario::ExactHandoff:
		{
			Test->TestTrue(TEXT("D: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			Test->TestTrue(TEXT("D: the primed candidate is accepted"),
				Arb::SubmitPrimedCandidate(Target, CandidateFromBaseline(Ticket), Fail));
			Test->TestEqual(TEXT("D: the aircraft is PrimedCandidateReady"),
				static_cast<int32>(Arb::GetPrimeState(Target)),
				static_cast<int32>(Arb::EPrimeState::PrimedCandidateReady));
			Test->TestTrue(TEXT("D: activation granted"), Arb::ActivateFormationForTesting(Target, Fail));
			break;
		}

		case EPrimeScenario::UnprimedRejected:
		{
			// A perfectly good candidate, pushed through the RAW seam with no prime behind it.
			Arb::FFormationCandidate K{};
			K.AileronNorm = 0.1; K.ElevatorNorm = -0.05; K.RudderNorm = 0.0;
			K.ThrottleNorm = 0.5; K.SpeedBrakeNorm = 0.0;
			K.bValid = K.bCommandReady = K.bFinite = true;
			K.TimestampS = FApp::GetCurrentTime();
			Arb::SetCandidateForTesting(Target, K);

			Test->TestFalse(TEXT("E: activation without a prime is refused"),
				Arb::ActivateFormationForTesting(Target, Fail));
			Test->TestEqual(TEXT("E: and the reason is NotPrimed"),
				static_cast<int32>(Fail), static_cast<int32>(EF::NotPrimed));
			Test->TestEqual(TEXT("E: the aircraft stays on Legacy"),
				static_cast<int32>(Arb::GetMode(Target)), static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
			break;
		}

		case EPrimeScenario::WrongGeneration:
		{
			Test->TestTrue(TEXT("F: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			Arb::FPrimedCandidate Bad = CandidateFromBaseline(Ticket);
			Bad.PrimeGeneration = Ticket.Generation + 7;   // never issued
			Test->TestFalse(TEXT("F: a candidate with a generation that was never issued is refused"),
				Arb::SubmitPrimedCandidate(Target, Bad, Fail));
			Test->TestEqual(TEXT("F: and the reason is WrongGeneration"),
				static_cast<int32>(Fail), static_cast<int32>(EF::WrongGeneration));
			Test->TestFalse(TEXT("F: activation is still refused"),
				Arb::ActivateFormationForTesting(Target, Fail));
			Test->TestEqual(TEXT("F: the mode never changed"),
				static_cast<int32>(Arb::GetMode(Target)), static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
			break;
		}

		case EPrimeScenario::Stale:
		{
			Test->TestTrue(TEXT("G: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			Arb::FPrimedCandidate Old = CandidateFromBaseline(Ticket);
			Old.Candidate.TimestampS = FApp::GetCurrentTime() - 1.0;   // the producer died a second ago
			Test->TestFalse(TEXT("G: a stale candidate is refused"),
				Arb::SubmitPrimedCandidate(Target, Old, Fail));
			Test->TestEqual(TEXT("G: and the reason is StaleCandidate"),
				static_cast<int32>(Fail), static_cast<int32>(EF::StaleCandidate));
			Test->TestEqual(TEXT("G: the mode never changed"),
				static_cast<int32>(Arb::GetMode(Target)), static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
			break;
		}

		case EPrimeScenario::NonFinite:
		{
			Test->TestTrue(TEXT("H: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			Arb::FPrimedCandidate Bad = CandidateFromBaseline(Ticket);
			// Claims to be finite AND ready, but carries a NaN. The arbiter must VERIFY, not trust.
			Bad.Candidate.AileronNorm = std::numeric_limits<double>::quiet_NaN();
			Test->TestFalse(TEXT("H: a candidate that LIES about being finite is refused"),
				Arb::SubmitPrimedCandidate(Target, Bad, Fail));
			Test->TestEqual(TEXT("H: and the reason is NonFiniteCandidate"),
				static_cast<int32>(Fail), static_cast<int32>(EF::NonFiniteCandidate));
			break;
		}

		case EPrimeScenario::FallingPreemption:
		{
			Test->TestTrue(TEXT("I: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			St->StaleTicket = Ticket;   // kept so the post-mortem can try to replay it
			Test->TestTrue(TEXT("I: the primed candidate is accepted"),
				Arb::SubmitPrimedCandidate(Target, CandidateFromBaseline(Ticket), Fail));
			Test->TestEqual(TEXT("I: the aircraft is PrimedCandidateReady before the damage"),
				static_cast<int32>(Arb::GetPrimeState(Target)),
				static_cast<int32>(Arb::EPrimeState::PrimedCandidateReady));
			// Deliberately NOT activated: the handoff is caught mid-flight by the damage.
			break;
		}

		case EPrimeScenario::PerAircraftIsolation:
		{
			Test->TestTrue(TEXT("J: prime granted for F16_UAV1"), Arb::RequestPrime(Target, Ticket, Fail));
			Test->TestTrue(TEXT("J: its candidate is accepted"),
				Arb::SubmitPrimedCandidate(Target, CandidateFromBaseline(Ticket), Fail));
			Test->TestTrue(TEXT("J: it activates"), Arb::ActivateFormationForTesting(Target, Fail));
			break;
		}

		case EPrimeScenario::WorldCleanup:
		{
			// World B gets its own aircraft, its own prime state and its own generation. Tearing down the
			// PIE world must not touch any of it. Done in one Update() so PIE cannot re-register in
			// between and mask the difference between "wipe this world" and "wipe everything".
			Test->TestTrue(TEXT("K: PIE aircraft primed"), Arb::RequestPrime(Target, Ticket, Fail));

			St->OtherWorld = UWorld::CreateWorld(EWorldType::Game, false);
			AActor *A = St->OtherWorld ? St->OtherWorld->SpawnActor<AActor>() : nullptr;
			UJSBSimMovementComponent *Other = A ? NewObject<UJSBSimMovementComponent>(A) : nullptr;
			if (!Other) { Test->AddError(TEXT("[PRIME] K: could not build world B")); return false; }

			// World B gets a REAL prime: an injected resolved snapshot (its component never ticks, so it
			// can never consume one), a granted ticket with its own generation, and a candidate. Setting
			// only a mode would not notice a wiped ticket or a destroyed snapshot.
			Arb::FResolvedCommandSnapshot SnapB{};
			SnapB.Commands.Aileron = 0.13; SnapB.Commands.Elevator = -0.07; SnapB.Commands.Rudder = 0.01;
			SnapB.Commands.SpeedBrake = 0.20; SnapB.Commands.Flap = 0.30; SnapB.Commands.GearDown = 1.0;
			SnapB.Commands.PitchTrim = 0.05;   // a Blueprint-owned field: it must survive too
			FEngineCommand EB{}; EB.Throttle = 0.44; EB.Running = true;
			SnapB.EngineCommands.Add(EB);
			SnapB.ConsumeSequence = 4242;
			SnapB.TimestampS = FApp::GetCurrentTime();
			SnapB.bValid = true;
			Arb::InjectResolvedSnapshotForTesting(Other, SnapB);

			Arb::FPrimeTicket TicketB;
			Arb::EPrimeFailure FB = Arb::EPrimeFailure::None;

			// A baseline is not enough. A prime is a promise of continuity for an aircraft that is actually
			// flying, so the arbiter demands a PROVABLY alive one -- and this actor has no health component
			// at all, which is not the same as being healthy.
			Test->TestFalse(TEXT("K: an aircraft with no health component cannot be primed"),
				Arb::RequestPrime(Other, TicketB, FB));
			Test->TestEqual(TEXT("K: and the reason says so honestly"),
				static_cast<int32>(FB), static_cast<int32>(EF::UnknownHealth));

			// Give it one. It is alive by construction (LifeState defaults to Alive).
			UHealthComponent *HealthB = NewObject<UHealthComponent>(A);
			if (!HealthB) { Test->AddError(TEXT("[PRIME] K: could not build world B's health")); return false; }
			Test->TestTrue(TEXT("K: world B's aircraft is alive"), HealthB->IsAlive());

			Test->TestTrue(TEXT("K: world B's aircraft can now be primed"),
				Arb::RequestPrime(Other, TicketB, FB));
			Test->TestTrue(TEXT("K: world B has its own generation"), TicketB.Generation > 0);
			Test->TestEqual(TEXT("K: anchored to world B's own consume"),
				(int64)TicketB.BaselineConsumeSequence, (int64)4242);

			// The candidate goes in through the REAL prime path -- not the raw seam. World B must end up
			// in a genuine PrimedCandidateReady state, with the mode still LegacyOrManual: priming is not
			// activating, and a test that forced the mode would be checking a state production can never
			// reach.
			Arb::EPrimeFailure FC = Arb::EPrimeFailure::None;
			Test->TestTrue(TEXT("K: world B's primed candidate is accepted"),
				Arb::SubmitPrimedCandidate(Other, CandidateFromBaseline(TicketB), FC));
			Test->TestEqual(TEXT("K: world B is PrimedCandidateReady"),
				static_cast<int32>(Arb::GetPrimeState(Other)),
				static_cast<int32>(Arb::EPrimeState::PrimedCandidateReady));
			Test->TestEqual(TEXT("K: and its mode is STILL LegacyOrManual"),
				static_cast<int32>(Arb::GetMode(Other)),
				static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
			const bool bBHadCandidate = Arb::HasCandidate(Other);
			Test->TestTrue(TEXT("K: world B has a candidate before the teardown"), bBHadCandidate);

			const uint64 GenA = Arb::GetPrimeGeneration(Target);
			Test->TestTrue(TEXT("K: world A's aircraft has a prime generation"), GenA > 0);

			const int32 Both = Arb::GetRegistrySize();
			Test->TestTrue(TEXT("K: both worlds are registered"), Both >= 2);

			FWorldDelegates::OnWorldCleanup.Broadcast(World, true, true);
			const int32 AfterA = Arb::GetRegistrySize();
			Test->TestEqual(TEXT("K: ONLY world B survives world A's teardown"), AfterA, 1);

			// Everything about world B must be intact -- a blanket wipe would have destroyed all of it.
			Test->TestEqual(TEXT("K: world B's mode is STILL LegacyOrManual"),
				static_cast<int32>(Arb::GetMode(Other)),
				static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
			Test->TestTrue(TEXT("K: world B's candidate survived"), Arb::HasCandidate(Other));
			Test->TestEqual(TEXT("K: world B's generation survived"),
				(int64)Arb::GetPrimeGeneration(Other), (int64)TicketB.Generation);
			Test->TestEqual(TEXT("K: world B is STILL PrimedCandidateReady"),
				static_cast<int32>(Arb::GetPrimeState(Other)),
				static_cast<int32>(Arb::EPrimeState::PrimedCandidateReady));

			// The 29-field ticket baseline must be intact, including the Blueprint-owned fields.
			Arb::FResolvedCommandSnapshot BaseB;
			Test->TestTrue(TEXT("K: world B's ticket baseline survived"),
				Arb::GetTicketBaseline(Other, BaseB));
			Test->TestEqual(TEXT("K: ...aileron"),   BaseB.Commands.Aileron,   0.13);
			Test->TestEqual(TEXT("K: ...speedbrake"),BaseB.Commands.SpeedBrake,0.20);
			Test->TestEqual(TEXT("K: ...flap (Blueprint-owned)"), BaseB.Commands.Flap, 0.30);
			Test->TestEqual(TEXT("K: ...pitch trim (Blueprint-owned)"), BaseB.Commands.PitchTrim, 0.05);
			Test->TestEqual(TEXT("K: ...engine throttle"),
				BaseB.EngineCommands.Num() > 0 ? BaseB.EngineCommands[0].Throttle : -1.0, 0.44);
			Arb::FResolvedCommandSnapshot SnapNow;
			Test->TestTrue(TEXT("K: world B's resolved snapshot survived"),
				Arb::GetLastResolvedSnapshot(Other, SnapNow));

			// FIELD BY FIELD against the block that went in -- all 15 flight-control fields and all 14
			// per-engine fields. Spot-checking aileron and throttle and calling that "29 fields preserved"
			// would be a claim the test never made: a wiped trim, brake or magneto would sail through.
			Test->TestEqual(TEXT("K: the surviving snapshot is IDENTICAL to the injected one (29 fields)"),
				CountDiff(SnapB, SnapNow), 0);
			Test->TestEqual(TEXT("K: the surviving ticket baseline is IDENTICAL to it too (29 fields)"),
				CountDiff(SnapB, BaseB), 0);
			Test->TestEqual(TEXT("K: the engine array kept its length"),
				SnapNow.EngineCommands.Num(), SnapB.EngineCommands.Num());
			Test->TestEqual(TEXT("K: the ticket baseline's engine array too"),
				BaseB.EngineCommands.Num(), SnapB.EngineCommands.Num());
			Test->TestEqual(TEXT("K: ...with its own consume sequence"),
				(int64)SnapNow.ConsumeSequence, (int64)4242);
			Test->TestEqual(TEXT("K: the ticket baseline's sequence too"),
				(int64)BaseB.ConsumeSequence, (int64)4242);
			// Identity survived too -- the snapshot still knows whose command it is, and in which world.
			Test->TestTrue(TEXT("K: ...still owned by world B's component"), SnapNow.Component.Get() == Other);
			Test->TestTrue(TEXT("K: ...still in world B"), SnapNow.World.Get() == St->OtherWorld);
			Test->TestTrue(TEXT("K: the ticket baseline kept its identity"), BaseB.Component.Get() == Other);
			Test->TestTrue(TEXT("K: ...and its world"), BaseB.World.Get() == St->OtherWorld);

			FWorldDelegates::OnWorldCleanup.Broadcast(St->OtherWorld, true, true);
			const int32 AfterB = Arb::GetRegistrySize();
			Test->TestEqual(TEXT("K: tearing down world B empties the registry"), AfterB, 0);
			Test->TestFalse(TEXT("K: and world B's candidate is gone with it"), Arb::HasCandidate(Other));

			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] K_RESULT both=%d after_world_A=%d after_world_B=%d worldA_gen=%llu worldB_had_candidate=%d "
				     "snapshot_diff=%d baseline_diff=%d"),
				Both, AfterA, AfterB, GenA, bBHadCandidate ? 1 : 0,
				CountDiff(SnapB, SnapNow), CountDiff(SnapB, BaseB));
			St->OtherWorld->DestroyWorld(false);
			St->OtherWorld = nullptr;
			return false;   // finalize on the normal path
		}

		case EPrimeScenario::ResetSessionSafety:
		{
			// A handoff genuinely IN FLIGHT -- primed, candidate accepted, waiting only for activation.
			// Resetting on a merely-PrimePending aircraft would have proved much less: the candidate, which
			// is the thing a reset could most easily destroy, would never have existed.
			Test->TestTrue(TEXT("L: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			const uint64 Gen = Ticket.Generation;
			Test->TestTrue(TEXT("L: the primed candidate is accepted"),
				Arb::SubmitPrimedCandidate(Target, CandidateFromBaseline(Ticket), Fail));
			Test->TestEqual(TEXT("L: the aircraft is PrimedCandidateReady"),
				static_cast<int32>(Arb::GetPrimeState(Target)),
				static_cast<int32>(Arb::EPrimeState::PrimedCandidateReady));
			Test->TestEqual(TEXT("L: and still LegacyOrManual (priming is not activating)"),
				static_cast<int32>(Arb::GetMode(Target)),
				static_cast<int32>(Arb::ECommandMode::LegacyOrManual));

			Arb::FResolvedCommandSnapshot BaseBefore;
			Test->TestTrue(TEXT("L: it has a ticket baseline"), Arb::GetTicketBaseline(Target, BaseBefore));

			// A session reset clears the MEASUREMENT. It must not be able to cancel a handoff in flight.
			Arb::ResetSession(TEXT("L_after_reset"));

			Test->TestEqual(TEXT("L: the prime state SURVIVED the session reset"),
				static_cast<int32>(Arb::GetPrimeState(Target)),
				static_cast<int32>(Arb::EPrimeState::PrimedCandidateReady));
			Test->TestEqual(TEXT("L: the generation SURVIVED"), (int64)Arb::GetPrimeGeneration(Target), (int64)Gen);
			Test->TestTrue(TEXT("L: the CANDIDATE survived"), Arb::HasCandidate(Target));
			Test->TestEqual(TEXT("L: the mode is untouched"),
				static_cast<int32>(Arb::GetMode(Target)),
				static_cast<int32>(Arb::ECommandMode::LegacyOrManual));

			Arb::FResolvedCommandSnapshot Snap;
			Test->TestTrue(TEXT("L: the resolved snapshot SURVIVED"),
				Arb::GetLastResolvedSnapshot(Target, Snap));
			Arb::FResolvedCommandSnapshot BaseAfter;
			Test->TestTrue(TEXT("L: the ticket baseline SURVIVED"), Arb::GetTicketBaseline(Target, BaseAfter));
			// All 29 fields of it -- a reset that quietly zeroed one would make the handoff step by exactly
			// that field, and no coarser check would see it.
			Test->TestEqual(TEXT("L: the ticket baseline is unchanged in ALL 29 fields"),
				CountDiff(BaseBefore, BaseAfter), 0);
			Test->TestEqual(TEXT("L: ...same consume sequence"),
				(int64)BaseAfter.ConsumeSequence, (int64)BaseBefore.ConsumeSequence);
			Test->TestTrue(TEXT("L: ...same owner"), BaseAfter.Component.Get() == Target);
			Test->TestTrue(TEXT("L: ...same world"), BaseAfter.World.Get() == Target->GetWorld());

			// A reset must not manufacture an activation either: nothing may be pending that was not
			// pending before.
			Test->TestNotEqual(TEXT("L: the reset did NOT arm an activation"),
				static_cast<int32>(Arb::GetPrimeState(Target)),
				static_cast<int32>(Arb::EPrimeState::ActivationPending));

			const Arb::FCounters C = Arb::GetCounters();
			Test->TestEqual(TEXT("L: but the counters were cleared"), C.PrimeGrantedCount, (int64)0);
			Test->TestEqual(TEXT("L: ...all of them"), C.PrimedCandidateAcceptedCount, (int64)0);
			Test->TestEqual(TEXT("L: ...including the consume count"), C.ConsumeCount, (int64)0);

			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] L_RESULT state_after_reset=%s generation=%llu candidate=%d baseline_diff=%d granted_counter=%lld"),
				Arb::PrimeStateName(Arb::GetPrimeState(Target)), Arb::GetPrimeGeneration(Target),
				Arb::HasCandidate(Target) ? 1 : 0, CountDiff(BaseBefore, BaseAfter), C.PrimeGrantedCount);
			break;
		}

		case EPrimeScenario::InterveningConsume:
		{
			// Prime and submit -- but do NOT activate yet. The activation is deliberately delayed so the
			// FDM consumes a Legacy block in between, making the baseline stale.
			Test->TestTrue(TEXT("N: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			Test->TestTrue(TEXT("N: the primed candidate is accepted"),
				Arb::SubmitPrimedCandidate(Target, CandidateFromBaseline(Ticket), Fail));
			St->StaleTicket = Ticket;
			St->bPrimedForIntervening = true;
			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] N: primed at baseline_seq=%llu; delaying activation so the FDM moves on"),
				Ticket.BaselineConsumeSequence);
			break;
		}

		case EPrimeScenario::StickExactFirstCompute:
		{
			// This one never touches the arbiter. It calls F16StickAdapterV2::Update() directly, because
			// the point is to prove the CONTROLLER reproduces the baseline -- submitting a baseline
			// candidate to the arbiter would prove nothing about the stick at all.
			using namespace FormationControlV2;
			FF16StickConfigV2 Cfg{};
			F16StickAdapterV2 Stick;

			// A non-zero baseline, and a guidance input whose NATURAL target is deliberately different --
			// large attitude errors and a non-zero pitch rate. If the latch were absent, the first command
			// would follow that target (slew-limited), not the baseline.
			const double BaseAil = 0.2371, BaseElv = -0.1834, BaseRud = 0.0917, BaseThr = 0.6428;
			const double BodyQ = 0.0731;   // non-zero body pitch rate

			FF16StickInputV2 In{};
			In.RollReferenceRad = 0.6;  In.PitchReferenceRad = 0.25;  In.ThrottleReferenceNorm = 0.95;
			In.bGuidanceValid = true;
			In.CurrentRollRad = -0.4;   In.CurrentPitchRad = -0.15;   In.bAttitudeValid = true;
			In.BodyRollRateRadps = 0.05; In.BodyPitchRateRadps = BodyQ; In.BodyYawRateRadps = 0.0;
			In.bBodyRatesValid = true;
			In.AlphaRad = 0.0; In.BetaRad = 0.0; In.bAlphaBetaValid = true;
			In.EasMps = 190.0; In.TasMps = 220.0; In.bAirspeedValid = true;
			In.SimulationTimeS = 100.0; In.DtS = 1.0 / 60.0;
			In.ResetGeneration = 3;
			In.PrimeGeneration = 11; In.PrimeConsumeSequence = 222;   // the frame the latch is FOR

			Test->TestTrue(TEXT("O: the prime is accepted"),
				Stick.PrimeFromResolvedCommand(BaseAil, BaseElv, BaseRud, BaseThr, BodyQ, Cfg,
				                               /*primeGeneration=*/11, /*consumeSequence=*/222,
				                               /*resetGeneration=*/3));
			Test->TestTrue(TEXT("O: the prime armed the one-shot latch"), Stick.HasPrimedBaselineLatch());
			Test->TestEqual(TEXT("O: the latch carries its generation"),
				(int64)Stick.PrimedGeneration(), (int64)11);
			Test->TestEqual(TEXT("O: the latch carries its consume sequence"),
				(int64)Stick.PrimedConsumeSequence(), (int64)222);

			const FF16StickCommandV2 First = Stick.Update(In, Cfg);
			Test->TestTrue(TEXT("O: the first compute is a valid command"), First.bValid);
			// THE CLAIM: the FIRST compute reproduces the baseline EXACTLY.
			Test->TestEqual(TEXT("O: first aileron == baseline"),  First.AileronCmdNorm,  BaseAil);
			Test->TestEqual(TEXT("O: first elevator == baseline"), First.ElevatorCmdNorm, BaseElv);
			Test->TestEqual(TEXT("O: first rudder == baseline"),   First.RudderCmdNorm,   BaseRud);
			Test->TestEqual(TEXT("O: first throttle == baseline"), First.ThrottleCmdNorm, BaseThr);
			Test->TestFalse(TEXT("O: the latch was consumed"), Stick.HasPrimedBaselineLatch());

			// The second compute runs the ORDINARY path again -- target, integrator, slew, clamp. With the
			// references above pulling hard away from the baseline, it must move.
			In.SimulationTimeS += In.DtS;
			const FF16StickCommandV2 Second = Stick.Update(In, Cfg);
			Test->TestTrue(TEXT("O: the second compute is valid"), Second.bValid);
			Test->TestTrue(TEXT("O: the second compute is NOT the latch again (the normal path resumed)"),
				Second.AileronCmdNorm != First.AileronCmdNorm || Second.ElevatorCmdNorm != First.ElevatorCmdNorm);
			Test->TestFalse(TEXT("O: the latch was not re-armed by an ordinary update"),
				Stick.HasPrimedBaselineLatch());

			// A NEW prime replaces the latch; Reset() destroys it.
			Test->TestTrue(TEXT("O: a new prime is accepted"),
				Stick.PrimeFromResolvedCommand(0.4, 0.1, -0.2, 0.3, BodyQ, Cfg, 12, 333, 3));
			Test->TestTrue(TEXT("O: a new prime re-arms the latch"), Stick.HasPrimedBaselineLatch());
			Test->TestEqual(TEXT("O: with the NEW generation"), (int64)Stick.PrimedGeneration(), (int64)12);
			Stick.Reset(4);
			Test->TestFalse(TEXT("O: Reset() destroys the latch"), Stick.HasPrimedBaselineLatch());
			Test->TestEqual(TEXT("O: and clears its generation"), (int64)Stick.PrimedGeneration(), (int64)0);

			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] O_RESULT first{ail=%.6f elv=%.6f rud=%.6f thr=%.6f} baseline{ail=%.6f elv=%.6f rud=%.6f thr=%.6f} "
				     "second{ail=%.6f elv=%.6f} latch_after_first=0"),
				First.AileronCmdNorm, First.ElevatorCmdNorm, First.RudderCmdNorm, First.ThrottleCmdNorm,
				BaseAil, BaseElv, BaseRud, BaseThr, Second.AileronCmdNorm, Second.ElevatorCmdNorm);
			// SpeedBrake is deliberately absent: it is NOT a stick output. Carrying it across the handoff is
			// the arbiter's baseline-copy responsibility, proved in D/P, not the stick's.
			break;
		}

		case EPrimeScenario::StickWrongIdentity:
		{
			// A latch is a promise about ONE consume of ONE prime. Spending it on any other frame would
			// emit a command the aircraft was not flying -- a step wearing a handoff's clothes.
			using namespace FormationControlV2;
			FF16StickConfigV2 Cfg{};
			F16StickAdapterV2 Stick;

			const double BaseAil = 0.2371, BaseElv = -0.1834, BaseRud = 0.0917, BaseThr = 0.6428;
			const double BodyQ = 0.0731;

			FF16StickInputV2 In{};
			In.RollReferenceRad = 0.6;  In.PitchReferenceRad = 0.25;  In.ThrottleReferenceNorm = 0.95;
			In.bGuidanceValid = true;
			In.CurrentRollRad = -0.4;   In.CurrentPitchRad = -0.15;   In.bAttitudeValid = true;
			In.BodyRollRateRadps = 0.05; In.BodyPitchRateRadps = BodyQ; In.BodyYawRateRadps = 0.0;
			In.bBodyRatesValid = true;  In.bAlphaBetaValid = true;
			In.EasMps = 190.0; In.TasMps = 220.0; In.bAirspeedValid = true;
			In.SimulationTimeS = 100.0; In.DtS = 1.0 / 60.0; In.ResetGeneration = 3;

			Test->TestTrue(TEXT("Q: the prime is accepted"),
				Stick.PrimeFromResolvedCommand(BaseAil, BaseElv, BaseRud, BaseThr, BodyQ, Cfg, 11, 222, 3));
			const double IntegAfterPrime = Stick.PrimedPitchIntegrator();

			// WRONG GENERATION.
			In.PrimeGeneration = 12; In.PrimeConsumeSequence = 222;
			FF16StickCommandV2 Bad1 = Stick.Update(In, Cfg);
			Test->TestFalse(TEXT("Q: a wrong-generation frame produces NO command"), Bad1.bValid);
			Test->TestEqual(TEXT("Q: and says why"), static_cast<int32>(Bad1.FailureReason),
				static_cast<int32>(EF16StickFailureV2::PrimeIdentityMismatch));
			Test->TestTrue(TEXT("Q: the latch was NOT consumed"), Stick.HasPrimedBaselineLatch());
			Test->TestEqual(TEXT("Q: the integrator was NOT touched"),
				Stick.PrimedPitchIntegrator(), IntegAfterPrime);
			Test->TestEqual(TEXT("Q: Prev aileron was NOT touched"), Stick.PrevCommandAileron(), BaseAil);

			// WRONG CONSUME SEQUENCE.
			In.PrimeGeneration = 11; In.PrimeConsumeSequence = 223;
			FF16StickCommandV2 Bad2 = Stick.Update(In, Cfg);
			Test->TestFalse(TEXT("Q: a wrong-sequence frame produces NO command"), Bad2.bValid);
			Test->TestEqual(TEXT("Q: and says why"), static_cast<int32>(Bad2.FailureReason),
				static_cast<int32>(EF16StickFailureV2::PrimeIdentityMismatch));
			Test->TestTrue(TEXT("Q: the latch is STILL armed"), Stick.HasPrimedBaselineLatch());
			Test->TestEqual(TEXT("Q: the integrator is STILL untouched"),
				Stick.PrimedPitchIntegrator(), IntegAfterPrime);

			// THE RIGHT FRAME. Only now is it spent.
			In.PrimeGeneration = 11; In.PrimeConsumeSequence = 222;
			FF16StickCommandV2 Good = Stick.Update(In, Cfg);
			Test->TestTrue(TEXT("Q: the matching frame produces a command"), Good.bValid);
			Test->TestEqual(TEXT("Q: it is the baseline"), Good.AileronCmdNorm, BaseAil);
			Test->TestEqual(TEXT("Q: elevator too"), Good.ElevatorCmdNorm, BaseElv);
			Test->TestFalse(TEXT("Q: and the latch is now spent"), Stick.HasPrimedBaselineLatch());

			// Exactly once: a repeat of the same identity must NOT re-emit the baseline.
			In.SimulationTimeS += In.DtS;
			FF16StickCommandV2 Again = Stick.Update(In, Cfg);
			Test->TestTrue(TEXT("Q: the next frame is an ordinary command"), Again.bValid);
			Test->TestFalse(TEXT("Q: the latch was consumed exactly once"), Stick.HasPrimedBaselineLatch());

			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] Q_RESULT wrong_gen_valid=%d wrong_seq_valid=%d good_ail=%.6f baseline_ail=%.6f latch_after=0"),
				Bad1.bValid ? 1 : 0, Bad2.bValid ? 1 : 0, Good.AileronCmdNorm, BaseAil);
			break;
		}

		case EPrimeScenario::StickLatchedNoAdvance:
		{
			// The latched frame must ADVANCE NOTHING: no integration, no target, no slew progression.
			// If the ordinary path ran first and the outputs were merely overwritten, the controller would
			// carry one frame of state it never actually flew.
			using namespace FormationControlV2;
			FF16StickConfigV2 Cfg{};
			Cfg.PitchRateDampingGain = 0.05;   // make the integrator/damping path genuinely active
			F16StickAdapterV2 Stick;

			const double BaseAil = 0.31, BaseElv = -0.22, BaseRud = 0.05, BaseThr = 0.71;
			const double BodyQ = 0.09;

			FF16StickInputV2 In{};
			// References pulling HARD away from the baseline, so an ordinary frame would move a lot.
			In.RollReferenceRad = 0.9;  In.PitchReferenceRad = 0.5;  In.ThrottleReferenceNorm = 0.1;
			In.bGuidanceValid = true;
			In.CurrentRollRad = -0.7;   In.CurrentPitchRad = -0.4;   In.bAttitudeValid = true;
			In.BodyRollRateRadps = 0.1; In.BodyPitchRateRadps = BodyQ; In.BodyYawRateRadps = 0.0;
			In.bBodyRatesValid = true;  In.bAlphaBetaValid = true;
			In.EasMps = 190.0; In.TasMps = 220.0; In.bAirspeedValid = true;
			In.SimulationTimeS = 50.0; In.DtS = 1.0 / 60.0; In.ResetGeneration = 2;
			In.PrimeGeneration = 77; In.PrimeConsumeSequence = 999;

			Test->TestTrue(TEXT("R: the prime is accepted"),
				Stick.PrimeFromResolvedCommand(BaseAil, BaseElv, BaseRud, BaseThr, BodyQ, Cfg, 77, 999, 2));
			const double IntegBefore = Stick.PrimedPitchIntegrator();
			const uint64 NormalBefore = Stick.NormalPathUpdates();

			const FF16StickCommandV2 Latched = Stick.Update(In, Cfg);
			Test->TestTrue(TEXT("R: the latched frame is valid"), Latched.bValid);
			Test->TestEqual(TEXT("R: it emits the baseline"), Latched.AileronCmdNorm, BaseAil);
			Test->TestEqual(TEXT("R: elevator too"), Latched.ElevatorCmdNorm, BaseElv);

			// THE POINT: the integrator did not move on the latched frame.
			Test->TestEqual(TEXT("R: the PitchIntegrator did NOT advance on the latched frame"),
				Stick.PrimedPitchIntegrator(), IntegBefore);
			// And the slew anchors are the baseline, not some half-computed target.
			Test->TestEqual(TEXT("R: Prev aileron == baseline"),  Stick.PrevCommandAileron(),  BaseAil);
			Test->TestEqual(TEXT("R: Prev elevator == baseline"), Stick.PrevCommandElevator(), BaseElv);
			Test->TestEqual(TEXT("R: Prev rudder == baseline"),   Stick.PrevCommandRudder(),   BaseRud);
			Test->TestEqual(TEXT("R: Prev throttle == baseline"), Stick.PrevCommandThrottle(), BaseThr);

			// THE DECISIVE CHECK: the ordinary path did not RUN at all on the latched frame.
			//
			// Watching the integrator would be unreliable here -- conditional anti-windup legitimately
			// skips integration under saturation, so a still integrator could mean either "the latch
			// suppressed it" or "the controller correctly refused to wind up". The counter cannot be
			// confused between the two.
			Test->TestEqual(TEXT("R: the ordinary path did NOT run on the latched frame"),
				(int64)Stick.NormalPathUpdates(), (int64)NormalBefore);

			// From the SECOND frame the ordinary path DOES run.
			In.SimulationTimeS += In.DtS;
			const FF16StickCommandV2 Normal = Stick.Update(In, Cfg);
			Test->TestTrue(TEXT("R: the second frame is valid"), Normal.bValid);
			Test->TestEqual(TEXT("R: and the ordinary path ran exactly once"),
				(int64)Stick.NormalPathUpdates(), (int64)NormalBefore + 1);
			Test->TestNotEqual(TEXT("R: the command moves off the baseline"),
				Normal.AileronCmdNorm, BaseAil);

			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] R_RESULT normal_before=%llu normal_after_latched=%llu normal_after_second=%llu "
				     "integ_latched=%.9f latched_ail=%.6f normal_ail=%.6f"),
				NormalBefore, NormalBefore, Stick.NormalPathUpdates(),
				IntegBefore, Latched.AileronCmdNorm, Normal.AileronCmdNorm);
			break;
		}

		case EPrimeScenario::StickInvalidPrime:
		{
			// A prime that cannot be trusted must change NOTHING. A half-applied prime is worse than none:
			// the slew anchors would move to a command nobody validated, and the latch would promise a
			// baseline nobody checked.
			using namespace FormationControlV2;
			FF16StickConfigV2 Cfg{};
			F16StickAdapterV2 Stick;

			// The WHOLE observable state, not one representative field. "PrevAileron is unchanged" says
			// nothing about the integrator, the latch, or the generation the latch is keyed to -- and a
			// partial mutation is exactly the failure being tested for.
			struct FStickSnap
			{
				double Integ, PrevA, PrevE, PrevR, PrevT;
				bool bLatch, bHavePrev;
				uint64 Gen, Seq, Normal;
			};
			auto Capture = [](const F16StickAdapterV2 &K) {
				return FStickSnap{ K.PrimedPitchIntegrator(), K.PrevCommandAileron(), K.PrevCommandElevator(),
					K.PrevCommandRudder(), K.PrevCommandThrottle(), K.HasPrimedBaselineLatch(),
					K.HasPrimedCommands(), K.PrimedGeneration(), K.PrimedConsumeSequence(),
					K.NormalPathUpdates() };
			};

			// Establish a real, valid controller state first.
			Test->TestTrue(TEXT("S: a valid prime is accepted"),
				Stick.PrimeFromResolvedCommand(0.15, -0.10, 0.02, 0.50, 0.01, Cfg, 5, 55, 1));
			Test->TestTrue(TEXT("S: and it armed the latch"), Stick.HasPrimedBaselineLatch());
			const FStickSnap Before = Capture(Stick);

			const double NaNv = std::numeric_limits<double>::quiet_NaN();

			// The config is an INPUT to the seed: the integrator is Clamp(D*q - elevator, +/-Limit) and the
			// baseline's range is read straight off it. A prime that validated the baseline against a config
			// nobody checked would have validated nothing at all.
			FF16StickConfigV2 CfgNaNGain = Cfg;  CfgNaNGain.PitchRateDampingGain = NaNv;
			FF16StickConfigV2 CfgNaNLimit = Cfg; CfgNaNLimit.PitchIntegratorLimit = NaNv;
			FF16StickConfigV2 CfgNegLimit = Cfg; CfgNegLimit.PitchIntegratorLimit = -0.30;
			FF16StickConfigV2 CfgNaNElev = Cfg;  CfgNaNElev.ElevatorMax = NaNv;
			FF16StickConfigV2 CfgInvElev = Cfg;  CfgInvElev.ElevatorMin = 1.0; CfgInvElev.ElevatorMax = -1.0;
			FF16StickConfigV2 CfgNaNThr = Cfg;   CfgNaNThr.ThrottleMin = NaNv;
			FF16StickConfigV2 CfgInvThr = Cfg;   CfgInvThr.ThrottleMin = 1.0; CfgInvThr.ThrottleMax = 0.0;

			struct FBadPrime { const TCHAR *Why; double A, E, R, T, Q; uint64 G, S; const FF16StickConfigV2 *C; };
			const FBadPrime Bad[] = {
				{ TEXT("generation 0"),               0.2, -0.1,  0.0, 0.4, 0.01, 0,  55, &Cfg },
				{ TEXT("consume sequence 0"),         0.2, -0.1,  0.0, 0.4, 0.01, 6,  0,  &Cfg },
				{ TEXT("non-finite aileron"),         NaNv,-0.1,  0.0, 0.4, 0.01, 6,  66, &Cfg },
				{ TEXT("non-finite throttle"),        0.2, -0.1,  0.0, NaNv,0.01, 6,  66, &Cfg },
				{ TEXT("non-finite pitch rate"),      0.2, -0.1,  0.0, 0.4, NaNv, 6,  66, &Cfg },
				{ TEXT("out-of-range aileron"),       1.7, -0.1,  0.0, 0.4, 0.01, 6,  66, &Cfg },
				{ TEXT("out-of-range throttle"),      0.2, -0.1,  0.0, 1.4, 0.01, 6,  66, &Cfg },
				{ TEXT("out-of-range rudder"),        0.2, -0.1, -2.0, 0.4, 0.01, 6,  66, &Cfg },
				{ TEXT("non-finite damping gain"),    0.2, -0.1,  0.0, 0.4, 0.01, 6,  66, &CfgNaNGain },
				{ TEXT("non-finite integrator limit"),0.2, -0.1,  0.0, 0.4, 0.01, 6,  66, &CfgNaNLimit },
				{ TEXT("negative integrator limit"),  0.2, -0.1,  0.0, 0.4, 0.01, 6,  66, &CfgNegLimit },
				{ TEXT("non-finite elevator range"),  0.2, -0.1,  0.0, 0.4, 0.01, 6,  66, &CfgNaNElev },
				{ TEXT("inverted elevator range"),    0.2, -0.1,  0.0, 0.4, 0.01, 6,  66, &CfgInvElev },
				{ TEXT("non-finite throttle range"),  0.2, -0.1,  0.0, 0.4, 0.01, 6,  66, &CfgNaNThr },
				{ TEXT("inverted throttle range"),    0.2, -0.1,  0.0, 0.4, 0.01, 6,  66, &CfgInvThr },
			};

			for (const FBadPrime &B : Bad)
			{
				const bool bOk = Stick.PrimeFromResolvedCommand(B.A, B.E, B.R, B.T, B.Q, *B.C, B.G, B.S, 1);
				Test->TestFalse(FString::Printf(TEXT("S: a prime with %s is refused"), B.Why), bOk);

				// AND IT CHANGED NOTHING -- every field of the state, every time.
				const FStickSnap After = Capture(Stick);
				const FString W(B.Why);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: integrator untouched"), *W), After.Integ, Before.Integ);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: Prev aileron untouched"), *W), After.PrevA, Before.PrevA);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: Prev elevator untouched"), *W), After.PrevE, Before.PrevE);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: Prev rudder untouched"), *W), After.PrevR, Before.PrevR);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: Prev throttle untouched"), *W), After.PrevT, Before.PrevT);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: the latch is untouched"), *W), After.bLatch, Before.bLatch);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: bHavePrevCommands untouched"), *W), After.bHavePrev, Before.bHavePrev);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: the latch generation untouched"), *W),
					(int64)After.Gen, (int64)Before.Gen);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: the latch consume sequence untouched"), *W),
					(int64)After.Seq, (int64)Before.Seq);
				Test->TestEqual(*FString::Printf(TEXT("S[%s]: the normal-path counter untouched"), *W),
					(int64)After.Normal, (int64)Before.Normal);
			}

			// Only a valid prime re-arms.
			Test->TestTrue(TEXT("S: a valid prime is still accepted afterwards"),
				Stick.PrimeFromResolvedCommand(0.3, -0.2, 0.05, 0.6, 0.02, Cfg, 9, 99, 1));
			Test->TestEqual(TEXT("S: with the new generation"), (int64)Stick.PrimedGeneration(), (int64)9);
			Test->TestEqual(TEXT("S: and the new consume sequence"), (int64)Stick.PrimedConsumeSequence(), (int64)99);

			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] S_RESULT bad_primes_refused=%d state_unchanged=1 final_generation=%llu"),
				(int32)UE_ARRAY_COUNT(Bad), Stick.PrimedGeneration());
			break;
		}

		case EPrimeScenario::ActivationBoundaryStale:
		{
			// FRESH AT SUBMISSION IS NOT FRESH AT THE HANDOFF.
			//
			// A candidate is checked for freshness when it is submitted. That check is about the past. The
			// handoff happens later, at a consume boundary, and by then the producer may have died, stalled,
			// or simply been overtaken by the aircraft's own motion. Handing over to a command computed for
			// a state the aircraft has since left is a step -- exactly the one this phase exists to prevent
			// -- so freshness is asked AGAIN, at the boundary, where the answer actually matters.
			Test->TestTrue(TEXT("T: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));

			Arb::FPrimedCandidate K = CandidateFromBaseline(Ticket);
			const double NowReal = FApp::GetCurrentTime();
			K.Candidate.TimestampS = NowReal - Arb::kCandidateMaxAgeS * 0.9;   // just INSIDE the threshold
			Test->TestTrue(TEXT("T: the candidate is FRESH at submission and is accepted"),
				Arb::SubmitPrimedCandidate(Target, K, Fail));
			Test->TestEqual(TEXT("T: the aircraft is PrimedCandidateReady"),
				static_cast<int32>(Arb::GetPrimeState(Target)),
				static_cast<int32>(Arb::EPrimeState::PrimedCandidateReady));

			Test->TestTrue(TEXT("T: the activation request is accepted (it only asks)"),
				Arb::ActivateFormationForTesting(Target, Fail));
			Test->TestEqual(TEXT("T: it is ActivationPending, NOT Formation"),
				static_cast<int32>(Arb::GetPrimeState(Target)),
				static_cast<int32>(Arb::EPrimeState::ActivationPending));
			Test->TestEqual(TEXT("T: and the mode is still LegacyOrManual"),
				static_cast<int32>(Arb::GetMode(Target)),
				static_cast<int32>(Arb::ECommandMode::LegacyOrManual));

			// The candidate rots while the activation is in flight. The clock is made an INPUT rather than
			// sleeping until it happens: a sleep would make the test hostage to the game scheduler, and a
			// test that is sometimes right is not a test. Production clock behaviour is untouched -- this
			// override does not exist outside a test build.
			Arb::SetClockOverrideForTesting(NowReal + Arb::kCandidateMaxAgeS * 0.5);   // age = 1.4x the limit
			St->bClockOverridden = true;
			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] T: candidate submitted fresh (age %.4f s), clock advanced so it is stale (age %.4f s) "
				     "before the boundary; max_age=%.4f s"),
				Arb::kCandidateMaxAgeS * 0.9, Arb::kCandidateMaxAgeS * 1.4, Arb::kCandidateMaxAgeS);
			break;
		}

		case EPrimeScenario::HandoffDeltaNegative:
		{
			// NEGATIVE CONTROL. If the delta were measured against "the previous consume" instead of the
			// immutable ticket baseline, a deliberate offset would still read as zero. It must not.
			Test->TestTrue(TEXT("P: prime granted"), Arb::RequestPrime(Target, Ticket, Fail));
			Arb::FPrimedCandidate Off = CandidateFromBaseline(Ticket);
			// The offset must not push the candidate out of [-1,1], or it would be refused as out of
			// range and the negative control would "pass" for the wrong reason.
			const double BaseA = Ticket.Baseline.Commands.Aileron;
			St->OffsetAileron = (BaseA <= 0.8) ? 0.1 : -0.1;
			Off.Candidate.AileronNorm = BaseA + St->OffsetAileron;   // ONLY aileron
			Test->TestTrue(TEXT("P: the offset candidate is accepted"),
				Arb::SubmitPrimedCandidate(Target, Off, Fail));
			Test->TestTrue(TEXT("P: activation granted"), Arb::ActivateFormationForTesting(Target, Fail));
			break;
		}

		case EPrimeScenario::ResolverOwnershipLost:
		{
			Test->TestTrue(TEXT("M: the arbiter owns the resolver to begin with"), Arb::IsEnabled());

			// A foreign resolver seizes the single-cast delegate.
			UJSBSimMovementComponent::CommandResolver.BindStatic(&PrimeForeignResolver);
			Test->TestFalse(TEXT("M: the arbiter no longer claims to be enabled"), Arb::IsEnabled());

			// We do not own the consume boundary, so we cannot promise anything about what will be
			// consumed -- priming or activating would be writing a cheque we cannot cash.
			Test->TestFalse(TEXT("M: priming is refused while another resolver owns the boundary"),
				Arb::RequestPrime(Target, Ticket, Fail));
			Test->TestEqual(TEXT("M: and the reason is ResolverNotOwned"),
				static_cast<int32>(Fail), static_cast<int32>(EF::ResolverNotOwned));
			Test->TestFalse(TEXT("M: activation is refused too"),
				Arb::ActivateFormationForTesting(Target, Fail));

			// Disabling must not delete somebody else's resolver.
			Arb::SetEnabled(false);
			Test->TestTrue(TEXT("M: the foreign resolver is still bound"),
				UJSBSimMovementComponent::CommandResolver.IsBound());
			Test->TestTrue(TEXT("M: the takeover was counted"), Arb::GetResolverOwnershipLostCount() > 0);

			// And it must not SEIZE it back either. Exactly one resolver may own the consume boundary;
			// silently evicting the current owner is the very failure this whole effort exists to remove.
			// (This is what logs the expected "already bound by something else" error.)
			const int64 ConflictsBeforeSeize = Arb::GetResolverOwnershipConflictCount();
			Arb::SetEnabled(true);
			Test->TestFalse(TEXT("M: the arbiter will NOT seize a resolver it does not own"), Arb::IsEnabled());
			Test->TestEqual(TEXT("M: the refusal was counted as a conflict"),
				Arb::GetResolverOwnershipConflictCount(), ConflictsBeforeSeize + 1);
			Test->TestTrue(TEXT("M: the foreign resolver is STILL bound after the refusal"),
				UJSBSimMovementComponent::CommandResolver.IsBound());

			// The foreign owner leaves of its own accord; only then may the arbiter take the boundary back.
			UJSBSimMovementComponent::CommandResolver.Unbind();
			Arb::SetEnabled(true);
			Test->TestTrue(TEXT("M: the arbiter recovers the resolver once it is free"), Arb::IsEnabled());

			// Priming CANNOT resume immediately, and that is correct, not a bug: disabling the arbiter
			// cleared the registry, so there is no observed command to be continuous with. A prime issued
			// now would be anchored to nothing. The post-recovery prime is therefore asserted in
			// Finalize(), after the FDM has consumed again -- see below.
			Arb::FPrimeTicket TooSoon;
			Test->TestFalse(TEXT("M: priming right after recovery has no baseline yet"),
				Arb::RequestPrime(Target, TooSoon, Fail));
			Test->TestEqual(TEXT("M: and the reason is NoResolvedSnapshot"),
				static_cast<int32>(Fail), static_cast<int32>(EF::NoResolvedSnapshot));
			break;
		}
		}
		return true;
	}

	static int32 CountDiff(const Arb::FResolvedCommandSnapshot &A, const Arb::FResolvedCommandSnapshot &B)
	{
		int32 N = 0;
		auto D = [&N](double X, double Y) { if (X != Y) ++N; };
		const FFlightControlCommands &X = A.Commands;
		const FFlightControlCommands &Y = B.Commands;
		D(X.Aileron, Y.Aileron); D(X.Elevator, Y.Elevator); D(X.Rudder, Y.Rudder);
		D(X.YawTrim, Y.YawTrim); D(X.PitchTrim, Y.PitchTrim); D(X.RollTrim, Y.RollTrim);
		D(X.Steer, Y.Steer); D(X.LeftBrake, Y.LeftBrake); D(X.RightBrake, Y.RightBrake);
		D(X.CenterBrake, Y.CenterBrake); D(X.ParkingBrake, Y.ParkingBrake); D(X.GearDown, Y.GearDown);
		D(X.Flap, Y.Flap); D(X.SpeedBrake, Y.SpeedBrake); D(X.Spoiler, Y.Spoiler);
		if (A.EngineCommands.Num() != B.EngineCommands.Num()) return N + 1;
		for (int32 i = 0; i < A.EngineCommands.Num(); ++i)
		{
			const FEngineCommand &P = A.EngineCommands[i];
			const FEngineCommand &Q = B.EngineCommands[i];
			D(P.Throttle, Q.Throttle); D(P.Mixture, Q.Mixture); D(P.PropellerAdvance, Q.PropellerAdvance);
			if (P.Starter != Q.Starter) ++N;
			if (P.Running != Q.Running) ++N;
			if (P.PropellerFeather != Q.PropellerFeather) ++N;
			if (P.Magnetos != Q.Magnetos) ++N;
			if (P.Augmentation != Q.Augmentation) ++N;
			if (P.Injection != Q.Injection) ++N;
			if (P.Ignition != Q.Ignition) ++N;
			if (P.Reverse != Q.Reverse) ++N;
			if (P.CutOff != Q.CutOff) ++N;
			if (P.GeneratorPower != Q.GeneratorPower) ++N;
			if (P.Condition != Q.Condition) ++N;
		}
		return N;
	}

	bool Finalize()
	{
		for (const FString &L : Arb::BuildReport()) UE_LOG(LogMumtPrimeTest, Display, TEXT("%s"), *L);
		const Arb::FCounters C = Arb::GetCounters();
		// A frozen clock must never outlive the scenario that froze it. (T clears it itself once it has
		// proved the boundary refused; this is the belt-and-braces release for every other exit path.)
		ON_SCOPE_EXIT { if (St->bClockOverridden) { Arb::ClearClockOverrideForTesting(); St->bClockOverridden = false; } };

		// Invariants that must hold no matter what the scenario did.
		Test->TestEqual(TEXT("PRIME: no non-finite command ever reached the FCS"),
			C.ResolvedNonFiniteCount, (int64)0);
		Test->TestEqual(TEXT("PRIME: no duplicate resolution"), C.DuplicateResolutionCount, (int64)0);
		Test->TestEqual(TEXT("PRIME: no consume bypassed the resolver"), C.MissingResolutionCount, (int64)0);
		Test->TestEqual(TEXT("PRIME: the resolver never mutated the legacy block"),
			C.LegacyBlockMutationCount, (int64)0);

		switch (Scenario)
		{
		case EPrimeScenario::ExactHandoff:
		{
			Arb::FHandoffDelta H{};
			Test->TestTrue(TEXT("D: the handoff was measured"), Arb::GetHandoffDelta(St->TargetName, H));
			// THE CLAIM. Not "small". Zero.
			Test->TestEqual(TEXT("D: dAileron == 0"), H.Aileron, 0.0);
			Test->TestEqual(TEXT("D: dElevator == 0"), H.Elevator, 0.0);
			Test->TestEqual(TEXT("D: dRudder == 0"), H.Rudder, 0.0);
			Test->TestEqual(TEXT("D: dThrottle == 0"), H.Throttle, 0.0);
			Test->TestEqual(TEXT("D: dSpeedBrake == 0"), H.SpeedBrake, 0.0);
			Test->TestEqual(TEXT("D: no handoff anywhere stepped"), C.NonZeroHandoffCount, (int64)0);
			Test->TestTrue(TEXT("D: Formation actually took over"), C.FormationResolutionCount > 0);
			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] D_RESULT seq=%llu dAil=%.9f dElv=%.9f dRud=%.9f dThr=%.9f dSpb=%.9f"),
				H.ConsumeSequence, H.Aileron, H.Elevator, H.Rudder, H.Throttle, H.SpeedBrake);
			break;
		}

		case EPrimeScenario::UnprimedRejected:
			Test->TestEqual(TEXT("E: Formation never resolved"), C.FormationResolutionCount, (int64)0);
			Test->TestTrue(TEXT("E: the activation was refused"), C.ActivationRejectedCount > 0);
			break;

		case EPrimeScenario::WrongGeneration:
			Test->TestEqual(TEXT("F: Formation never resolved"), C.FormationResolutionCount, (int64)0);
			Test->TestTrue(TEXT("F: the wrong generation was counted"), C.WrongGenerationRejectedCount > 0);
			break;

		case EPrimeScenario::Stale:
			Test->TestEqual(TEXT("G: Formation never resolved"), C.FormationResolutionCount, (int64)0);
			Test->TestTrue(TEXT("G: the stale ticket was counted"), C.StaleTicketRejectedCount > 0);
			break;

		case EPrimeScenario::NonFinite:
			Test->TestEqual(TEXT("H: Formation never resolved"), C.FormationResolutionCount, (int64)0);
			Test->TestEqual(TEXT("H: nothing non-finite reached the FCS"), C.ResolvedNonFiniteCount, (int64)0);
			break;

		case EPrimeScenario::FallingPreemption:
		{
			Test->TestTrue(TEXT("I: the aircraft was damaged"), St->bDamaged);
			Test->TestTrue(TEXT("I: the pending handoff was cancelled by Falling"),
				C.PrimeCancelledByFallingCount > 0);
			Test->TestEqual(TEXT("I: Formation NEVER resolved -- safety outranks bumpless"),
				C.FormationResolutionCount, (int64)0);
			Test->TestTrue(TEXT("I: the hardover took over"), C.FallingResolutionCount > 0);

			Arb::FAircraftCounters A{};
			Test->TestTrue(TEXT("I: the aircraft is known"), Arb::GetAircraftCounters(St->TargetName, A));
			Test->TestEqual(TEXT("I: it never resolved Formation"), A.FormationResolutions, (int64)0);

			// COUNTERS ARE NOT THE CONTRACT. Read the block the FDM actually consumed.
			UWorld *W = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
			UJSBSimMovementComponent *Dead = W ? FindPrimeAircraft(W, TEXT("F16_UAV1")) : nullptr;
			Test->TestTrue(TEXT("I: the aircraft is still present"), Dead != nullptr);
			double FinalThr = -1.0; bool bFinalCutOff = false; bool bGotBlock = false;
			if (Dead)
			{
				Arb::FResolvedCommandSnapshot Last;
				bGotBlock = Arb::GetLastResolvedSnapshot(Dead, Last);
				if (bGotBlock && Last.EngineCommands.Num() > 0)
				{
					FinalThr = Last.EngineCommands[0].Throttle;
					bFinalCutOff = Last.EngineCommands[0].CutOff;
				}
				// The dead ticket must be gone and unusable.
				Test->TestEqual(TEXT("I: the prime state is back to IdleLegacy"),
					static_cast<int32>(Arb::GetPrimeState(Dead)),
					static_cast<int32>(Arb::EPrimeState::IdleLegacy));
				Test->TestFalse(TEXT("I: the candidate was invalidated"), Arb::HasCandidate(Dead));
				Test->TestEqual(TEXT("I: the mode is back to LegacyOrManual"),
					static_cast<int32>(Arb::GetMode(Dead)),
					static_cast<int32>(Arb::ECommandMode::LegacyOrManual));

				// The generation-N ticket must not be reusable: submitting its candidate must be refused.
				Arb::FPrimedCandidate Replay;
				Replay.PrimeGeneration = St->StaleTicket.Generation;
				Replay.BaselineConsumeSequence = St->StaleTicket.BaselineConsumeSequence;
				Replay.Candidate.bValid = Replay.Candidate.bCommandReady = Replay.Candidate.bFinite = true;
				Replay.Candidate.TimestampS = FApp::GetCurrentTime();
				Arb::EPrimeFailure F3 = Arb::EPrimeFailure::None;
				Test->TestFalse(TEXT("I: the dead ticket cannot be replayed"),
					Arb::SubmitPrimedCandidate(Dead, Replay, F3));
			}
			Test->TestTrue(TEXT("I: the consumed block was readable"), bGotBlock);
			Test->TestEqual(TEXT("I: the FCS consumed engine throttle 0"), FinalThr, 0.0);
			Test->TestTrue(TEXT("I: the FCS consumed engine cutoff"), bFinalCutOff);

			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] I_RESULT cancelled=%lld formation=%lld falling=%lld final_throttle=%.6f final_cutoff=%d"),
				C.PrimeCancelledByFallingCount, C.FormationResolutionCount, C.FallingResolutionCount,
				FinalThr, bFinalCutOff ? 1 : 0);
			break;
		}

		case EPrimeScenario::PerAircraftIsolation:
		{
			UWorld *W = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
			UJSBSimMovementComponent *Uav1 = W ? FindPrimeAircraft(W, TEXT("F16_UAV1")) : nullptr;
			UJSBSimMovementComponent *Manned = W ? FindPrimeAircraft(W, TEXT("M_F16")) : nullptr;
			UJSBSimMovementComponent *Uav2 = W ? FindPrimeAircraft(W, TEXT("F16_UAV2")) : nullptr;
			Test->TestTrue(TEXT("J: all three aircraft found"), Uav1 && Manned && Uav2);
			if (!Uav1 || !Manned || !Uav2) break;

			// The primed one.
			Test->TestTrue(TEXT("J: the primed aircraft has a generation"), Arb::GetPrimeGeneration(Uav1) > 0);
			Test->TestNotEqual(TEXT("J: and its prime state moved off IdleLegacy"),
				static_cast<int32>(Arb::GetPrimeState(Uav1)), static_cast<int32>(Arb::EPrimeState::IdleLegacy));

			// The OTHERS. A ticket, candidate or snapshot leaking across aircraft would show up here --
			// checking only "UAV1 has a generation" would never have caught it.
			for (auto *Other : { Manned, Uav2 })
			{
				Test->TestEqual(TEXT("J: another aircraft has NO prime generation"),
					(int64)Arb::GetPrimeGeneration(Other), (int64)0);
				Test->TestEqual(TEXT("J: another aircraft is IdleLegacy"),
					static_cast<int32>(Arb::GetPrimeState(Other)),
					static_cast<int32>(Arb::EPrimeState::IdleLegacy));
				Test->TestFalse(TEXT("J: another aircraft has NO candidate"), Arb::HasCandidate(Other));
				Test->TestEqual(TEXT("J: another aircraft is still LegacyOrManual"),
					static_cast<int32>(Arb::GetMode(Other)),
					static_cast<int32>(Arb::ECommandMode::LegacyOrManual));

				// A ticket baseline belonging to another aircraft must not be visible here at all.
				Arb::FResolvedCommandSnapshot NoBase;
				Test->TestFalse(TEXT("J: another aircraft has NO ticket baseline"),
					Arb::GetTicketBaseline(Other, NoBase));

				// Its snapshot must be ITS OWN, proven by IDENTITY -- not by a consume sequence, which is
				// no identity at all: two aircraft can sit on the same sequence, so "sequence != 0" would
				// have proved nothing.
				Arb::FResolvedCommandSnapshot Own;
				Test->TestTrue(TEXT("J: another aircraft has its OWN resolved snapshot"),
					Arb::GetLastResolvedSnapshot(Other, Own));
				Test->TestTrue(TEXT("J: ...owned by ITSELF"), Own.Component.Get() == Other);
				Test->TestTrue(TEXT("J: ...in its OWN world"), Own.World.Get() == Other->GetWorld());
			}

			// And the primed aircraft's snapshot belongs to IT.
			{
				Arb::FResolvedCommandSnapshot Own1;
				Test->TestTrue(TEXT("J: the primed aircraft has its own snapshot"),
					Arb::GetLastResolvedSnapshot(Uav1, Own1));
				Test->TestTrue(TEXT("J: ...owned by itself"), Own1.Component.Get() == Uav1);
				Test->TestTrue(TEXT("J: ...in its own world"), Own1.World.Get() == Uav1->GetWorld());
				Test->TestTrue(TEXT("J: it has a candidate"), Arb::HasCandidate(Uav1));
				Arb::FResolvedCommandSnapshot Base1;
				Test->TestTrue(TEXT("J: and a ticket baseline"), Arb::GetTicketBaseline(Uav1, Base1));
				Test->TestTrue(TEXT("J: whose baseline is its own too"), Base1.Component.Get() == Uav1);
			}

			Arb::FAircraftCounters MannedC{};
			Test->TestTrue(TEXT("J: the other aircraft is known"),
				Arb::GetAircraftCounters(TEXT("M_F16"), MannedC));
			Test->TestEqual(TEXT("J: it never resolved Formation"), MannedC.FormationResolutions, (int64)0);
			Test->TestEqual(TEXT("J: its block passed through untouched"), MannedC.LegacyChangedFields, (int64)0);
			break;
		}

		case EPrimeScenario::ActivationBoundaryStale:
		{
			UWorld *W = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
			UJSBSimMovementComponent *T2 = W ? FindPrimeAircraft(W, TEXT("F16_UAV1")) : nullptr;
			Test->TestTrue(TEXT("T: the aircraft is still there"), T2 != nullptr);

			// THE POINT: the boundary refused, and it refused for the RIGHT reason. A stale candidate and a
			// moved-on baseline are different failures with different fixes, and a test that accepted either
			// would not know which one it had proved.
			Test->TestEqual(TEXT("T: the stale candidate was caught AT THE CONSUME BOUNDARY"),
				C.ActivationStaleRejectedCount, (int64)1);
			Test->TestEqual(TEXT("T: and NOT mistaken for an intervening consume"),
				C.InterveningConsumeRejectedCount, (int64)0);
			if (T2)
			{
				Test->TestEqual(TEXT("T: the recorded reason is StaleCandidate"),
					static_cast<int32>(Arb::GetLastBoundaryFailure(T2)),
					static_cast<int32>(Arb::EPrimeFailure::StaleCandidate));
			}

			Test->TestEqual(TEXT("T: Formation NEVER resolved"), C.FormationResolutionCount, (int64)0);
			Test->TestEqual(TEXT("T: no handoff was measured"), C.HandoffsMeasured, (int64)0);
			Test->TestEqual(TEXT("T: activation was never granted"), C.ActivationGrantedCount, (int64)0);

			if (T2)
			{
				Test->TestEqual(TEXT("T: the aircraft stayed on Legacy"),
					static_cast<int32>(Arb::GetMode(T2)),
					static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
				Test->TestEqual(TEXT("T: the dead ticket was discarded"),
					static_cast<int32>(Arb::GetPrimeState(T2)),
					static_cast<int32>(Arb::EPrimeState::IdleLegacy));
				Test->TestFalse(TEXT("T: and so was its candidate"), Arb::HasCandidate(T2));
				Arb::FResolvedCommandSnapshot NoBase;
				Test->TestFalse(TEXT("T: the ticket baseline is gone with it"),
					Arb::GetTicketBaseline(T2, NoBase));

				// Only a NEW prime, against the command the aircraft is actually flying now, may proceed.
				Arb::ClearClockOverrideForTesting();
				St->bClockOverridden = false;
				Arb::FPrimeTicket Fresh;
				Arb::EPrimeFailure F4 = Arb::EPrimeFailure::None;
				Test->TestTrue(TEXT("T: a NEW prime is granted"), Arb::RequestPrime(T2, Fresh, F4));
				Test->TestTrue(TEXT("T: with a generation that was never used before"), Fresh.Generation > 1);

				UE_LOG(LogMumtPrimeTest, Display,
					TEXT("[PRIME] T_RESULT stale_at_boundary=%lld intervening=%lld formation=%lld handoffs=%lld "
					     "activations_granted=%lld mode=%s new_generation=%llu"),
					C.ActivationStaleRejectedCount, C.InterveningConsumeRejectedCount,
					C.FormationResolutionCount, C.HandoffsMeasured, C.ActivationGrantedCount,
					Arb::GetMode(T2) == Arb::ECommandMode::LegacyOrManual ? TEXT("LegacyOrManual") : TEXT("Formation"),
					Fresh.Generation);
			}
			break;
		}

		case EPrimeScenario::InterveningConsume:
		{
			Test->TestTrue(TEXT("N: the activation was attempted"), St->bLateActivated);
			// THE POINT: the baseline went stale between the prime and the activation landing, so the
			// hand-over must have been refused AT THE CONSUME BOUNDARY.
			Test->TestTrue(TEXT("N: the stale baseline was caught at the consume boundary"),
				C.InterveningConsumeRejectedCount > 0);
			Test->TestEqual(TEXT("N: Formation NEVER resolved"), C.FormationResolutionCount, (int64)0);
			Test->TestEqual(TEXT("N: no handoff was measured"), C.HandoffsMeasured, (int64)0);

			UWorld *W = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
			UJSBSimMovementComponent *T2 = W ? FindPrimeAircraft(W, TEXT("F16_UAV1")) : nullptr;
			if (T2)
			{
				Test->TestEqual(TEXT("N: the aircraft stayed on Legacy"),
					static_cast<int32>(Arb::GetMode(T2)),
					static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
				Test->TestEqual(TEXT("N: the dead ticket was discarded"),
					static_cast<int32>(Arb::GetPrimeState(T2)),
					static_cast<int32>(Arb::EPrimeState::IdleLegacy));
				Test->TestFalse(TEXT("N: its candidate was invalidated"), Arb::HasCandidate(T2));

				// Progress requires a NEW prime against what the aircraft is flying NOW.
				Arb::FPrimeTicket Fresh;
				Arb::EPrimeFailure F2 = Arb::EPrimeFailure::None;
				Test->TestTrue(TEXT("N: a NEW prime is the only way forward"),
					Arb::RequestPrime(T2, Fresh, F2));
				Test->TestTrue(TEXT("N: and it is a later generation than the dead one"),
					Fresh.Generation > St->StaleTicket.Generation);
				UE_LOG(LogMumtPrimeTest, Display,
					TEXT("[PRIME] N_RESULT intervening_rejected=%lld formation=%lld dead_gen=%llu new_gen=%llu"),
					C.InterveningConsumeRejectedCount, C.FormationResolutionCount,
					St->StaleTicket.Generation, Fresh.Generation);
			}
			break;
		}

		case EPrimeScenario::HandoffDeltaNegative:
		{
			Arb::FHandoffDelta H{};
			Test->TestTrue(TEXT("P: the handoff was measured"), Arb::GetHandoffDelta(St->TargetName, H));
			// THE NEGATIVE CONTROL. If the delta compared the block with itself, this would be 0 and the
			// zero in D would prove nothing.
			Test->TestEqual(TEXT("P: dAileron == the deliberate offset"), H.Aileron, St->OffsetAileron);
			Test->TestEqual(TEXT("P: dElevator == 0"), H.Elevator, 0.0);
			Test->TestEqual(TEXT("P: dRudder == 0"), H.Rudder, 0.0);
			Test->TestEqual(TEXT("P: dThrottle == 0"), H.Throttle, 0.0);
			Test->TestEqual(TEXT("P: dSpeedBrake == 0"), H.SpeedBrake, 0.0);
			Test->TestEqual(TEXT("P: the step was COUNTED"), C.NonZeroHandoffCount, (int64)1);
			UE_LOG(LogMumtPrimeTest, Display,
				TEXT("[PRIME] P_RESULT dAil=%.9f (expected %.9f) dElv=%.9f dRud=%.9f dThr=%.9f dSpb=%.9f non_zero=%lld"),
				H.Aileron, St->OffsetAileron, H.Elevator, H.Rudder, H.Throttle, H.SpeedBrake,
				C.NonZeroHandoffCount);
			break;
		}

		case EPrimeScenario::StickExactFirstCompute:
		case EPrimeScenario::StickWrongIdentity:
		case EPrimeScenario::StickLatchedNoAdvance:
		case EPrimeScenario::StickInvalidPrime:
			// O/Q/R are controller-level: everything was asserted in Act() against F16StickAdapterV2
			// directly. They touch no arbiter state, so there is nothing to check here.
			break;

		case EPrimeScenario::ResolverOwnershipLost:
		{
			// By now the FDM has consumed again under the recovered resolver, so a baseline exists and a
			// prime must work -- with a FRESH generation. Recovery does not resurrect the old ticket.
			Test->TestTrue(TEXT("M: the arbiter still owns the resolver at the end"), Arb::IsEnabled());
			UWorld *W = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
			UJSBSimMovementComponent *Target = W ? FindPrimeAircraft(W, TEXT("F16_UAV1")) : nullptr;
			Test->TestTrue(TEXT("M: the aircraft is still there"), Target != nullptr);
			if (Target)
			{
				Arb::FPrimeTicket Fresh;
				Arb::EPrimeFailure Fail = Arb::EPrimeFailure::None;
				Test->TestTrue(TEXT("M: priming works again once the FDM has consumed under the recovered resolver"),
					Arb::RequestPrime(Target, Fresh, Fail));
				Test->TestTrue(TEXT("M: and it issued a fresh generation"), Fresh.Generation > 0);
				UE_LOG(LogMumtPrimeTest, Display,
					TEXT("[PRIME] M_RESULT conflicts=%lld lost=%lld recovered=%d new_generation=%u"),
					Arb::GetResolverOwnershipConflictCount(), Arb::GetResolverOwnershipLostCount(),
					Arb::IsEnabled() ? 1 : 0, Fresh.Generation);
			}
			break;
		}

		default:
			break;
		}

		if (St->OtherWorld) { St->OtherWorld->DestroyWorld(false); St->OtherWorld = nullptr; }
		return true;
	}

	FAutomationTestBase *Test;
	TSharedPtr<FPrimeState> St;
	EPrimeScenario Scenario;
};

void RunPrimeScenario(FAutomationTestBase *T, EPrimeScenario S)
{
	T->AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);
	if (S == EPrimeScenario::ResolverOwnershipLost)
	{
		// The arbiter logs an Error when it refuses to evict a foreign resolver. That refusal IS the
		// behaviour under test.
		T->AddExpectedErrorPlain(TEXT("already bound by something else"), EAutomationExpectedErrorFlags::Contains, 0);
	}
	TSharedPtr<FPrimeState> State = MakeShared<FPrimeState>();
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kPrimeMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtPrimeCommand(T, State, S));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
}

} // namespace

#define PRIME_TEST(ClassName, TestName, ScenarioValue)                                                 \
	IMPLEMENT_SIMPLE_AUTOMATION_TEST(ClassName, TestName,                                              \
		EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)                     \
	bool ClassName::RunTest(const FString &)                                                           \
	{ RunPrimeScenario(this, ScenarioValue); return true; }

PRIME_TEST(FMumtPrimeNoSnapshotTest,   "MUMT.ControlV2.PrimeNoResolvedSnapshot",    EPrimeScenario::NoResolvedSnapshot)
PRIME_TEST(FMumtPrimeSnapshotTest,     "MUMT.ControlV2.PrimeSnapshotExact",         EPrimeScenario::SnapshotExact)
PRIME_TEST(FMumtPrimeGenerationTest,   "MUMT.ControlV2.PrimeGeneration",            EPrimeScenario::Generation)
PRIME_TEST(FMumtPrimeHandoffTest,      "MUMT.ControlV2.PrimeExactHandoff",          EPrimeScenario::ExactHandoff)
PRIME_TEST(FMumtPrimeUnprimedTest,     "MUMT.ControlV2.PrimeUnprimedRejected",      EPrimeScenario::UnprimedRejected)
PRIME_TEST(FMumtPrimeWrongGenTest,     "MUMT.ControlV2.PrimeWrongGeneration",       EPrimeScenario::WrongGeneration)
PRIME_TEST(FMumtPrimeStaleTest,        "MUMT.ControlV2.PrimeStale",                 EPrimeScenario::Stale)
PRIME_TEST(FMumtPrimeNonFiniteTest,    "MUMT.ControlV2.PrimeNonFinite",             EPrimeScenario::NonFinite)
PRIME_TEST(FMumtPrimeFallingTest,      "MUMT.ControlV2.PrimeFallingPreemption",     EPrimeScenario::FallingPreemption)
PRIME_TEST(FMumtPrimeIsolationTest,    "MUMT.ControlV2.PrimePerAircraftIsolation",  EPrimeScenario::PerAircraftIsolation)
PRIME_TEST(FMumtPrimeWorldCleanupTest, "MUMT.ControlV2.PrimeWorldCleanup",          EPrimeScenario::WorldCleanup)
PRIME_TEST(FMumtPrimeResetSafetyTest,  "MUMT.ControlV2.PrimeResetSessionSafety",    EPrimeScenario::ResetSessionSafety)
PRIME_TEST(FMumtPrimeBoundaryStaleTest,"MUMT.ControlV2.PrimeActivationBoundaryStaleRejected", EPrimeScenario::ActivationBoundaryStale)
PRIME_TEST(FMumtPrimeOwnershipTest,    "MUMT.ControlV2.PrimeResolverOwnershipLost", EPrimeScenario::ResolverOwnershipLost)
PRIME_TEST(FMumtPrimeInterveningTest,  "MUMT.ControlV2.PrimeInterveningConsumeRejected", EPrimeScenario::InterveningConsume)
PRIME_TEST(FMumtPrimeStickTest,        "MUMT.ControlV2.PrimeStickExactFirstCompute",     EPrimeScenario::StickExactFirstCompute)
PRIME_TEST(FMumtPrimeNegControlTest,   "MUMT.ControlV2.PrimeHandoffDeltaNegativeControl",EPrimeScenario::HandoffDeltaNegative)
PRIME_TEST(FMumtPrimeStickIdentityTest,"MUMT.ControlV2.PrimeStickWrongIdentityDoesNotConsume", EPrimeScenario::StickWrongIdentity)
PRIME_TEST(FMumtPrimeStickNoAdvanceTest,"MUMT.ControlV2.PrimeStickLatchedFrameNoStateAdvance", EPrimeScenario::StickLatchedNoAdvance)
PRIME_TEST(FMumtPrimeStickInvalidTest, "MUMT.ControlV2.PrimeStickInvalidPrimeRejected",         EPrimeScenario::StickInvalidPrime)

#undef PRIME_TEST

#endif // WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
