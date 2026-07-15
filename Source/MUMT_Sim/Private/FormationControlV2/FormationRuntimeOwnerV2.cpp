#include "FormationControlV2/FormationRuntimeOwnerV2.h"

#include "JSBSimMovementComponent.h"
#include "HealthComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "Misc/App.h"

namespace
{
namespace Arb = MumtCommandArbiterV2;

constexpr int32  kMaxWarmTicks   = 180;    // 3 s at 60 Hz -- generous; the chain warms in ~3 frames
constexpr double kMaxCommandAgeS = 1.0;    // an operational enable older than this is stale
constexpr double kMaxDtS         = 0.5;    // a sim-time jump larger than this is a discontinuity
constexpr double kSlotAbsMaxM    = 100000.0;

bool FiniteInRange(double V) { return FMath::IsFinite(V) && FMath::Abs(V) <= kSlotAbsMaxM; }
}

const TCHAR *FormationRuntimePhaseName(EFormationRuntimePhaseV2 P)
{
	switch (P)
	{
	case EFormationRuntimePhaseV2::Warming:            return TEXT("Warming");
	case EFormationRuntimePhaseV2::Priming:            return TEXT("Priming");
	case EFormationRuntimePhaseV2::AwaitingActivation: return TEXT("AwaitingActivation");
	case EFormationRuntimePhaseV2::Active:             return TEXT("Active");
	default:                                           return TEXT("Idle");
	}
}

const TCHAR *FormationRuntimeFallbackName(EFormationRuntimeFallbackV2 R)
{
	switch (R)
	{
	case EFormationRuntimeFallbackV2::ExplicitDisable:      return TEXT("ExplicitDisable");
	case EFormationRuntimeFallbackV2::FollowerLost:         return TEXT("FollowerLost");
	case EFormationRuntimeFallbackV2::LeaderLost:           return TEXT("LeaderLost");
	case EFormationRuntimeFallbackV2::WorldMismatch:        return TEXT("WorldMismatch");
	case EFormationRuntimeFallbackV2::IdentityChanged:      return TEXT("IdentityChanged");
	case EFormationRuntimeFallbackV2::NotAlive:             return TEXT("NotAlive");
	case EFormationRuntimeFallbackV2::UnknownHealth:        return TEXT("UnknownHealth");
	case EFormationRuntimeFallbackV2::SimTimeDiscontinuity: return TEXT("SimTimeDiscontinuity");
	case EFormationRuntimeFallbackV2::StaleCommand:         return TEXT("StaleCommand");
	case EFormationRuntimeFallbackV2::SequenceReplay:       return TEXT("SequenceReplay");
	case EFormationRuntimeFallbackV2::NonFiniteSlot:        return TEXT("NonFiniteSlot");
	case EFormationRuntimeFallbackV2::NoLeaderSpecified:    return TEXT("NoLeaderSpecified");
	case EFormationRuntimeFallbackV2::ProducerRejected:     return TEXT("ProducerRejected");
	case EFormationRuntimeFallbackV2::ProducerInvalid:      return TEXT("ProducerInvalid");
	default:                                                return TEXT("None");
	}
}

UFormationRuntimeOwnerV2::UFormationRuntimeOwnerV2()
{
	PrimaryComponentTick.bCanEverTick = true;
	// The producer must read the JSBSim snapshot AFTER the movement component refreshes it, so this
	// component ticks in a LATER group as a coarse guarantee; the exact ordering is pinned by an explicit
	// per-component prerequisite in ResolveFollower(). TG_PostPhysics is after TG_DuringPhysics (where the
	// movement component ticks).
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
	Producer = MakeUnique<FormationControlV2::FFormationCandidateProducerV2>();
}

void UFormationRuntimeOwnerV2::OnRegister()
{
	Super::OnRegister();
	if (const AActor *Owner = GetOwner())
	{
		FollowerWorld = Owner->GetWorld();
	}
}

void UFormationRuntimeOwnerV2::OnUnregister()
{
	// Belt-and-braces: if we are torn down mid-handoff, do not leave the arbiter thinking Formation is
	// still live for a component that is going away.
	if (UJSBSimMovementComponent *F = FollowerComp.Get())
	{
		if (IsInGameThread() && Phase != EFormationRuntimePhaseV2::Idle)
		{
			Arb::RequestLegacyFallback(F);
		}
	}
	Super::OnUnregister();
}

MumtCommandArbiterV2::ECommandMode UFormationRuntimeOwnerV2::GetActiveMode() const
{
	const UJSBSimMovementComponent *F = FollowerComp.Get();
	return F ? Arb::GetMode(F) : Arb::ECommandMode::LegacyOrManual;
}

UJSBSimMovementComponent *UFormationRuntimeOwnerV2::ResolveFollower()
{
	if (UJSBSimMovementComponent *Cached = FollowerComp.Get()) return Cached;

	AActor *Owner = GetOwner();
	if (!Owner) return nullptr;
	UJSBSimMovementComponent *Mv = Owner->FindComponentByClass<UJSBSimMovementComponent>();
	if (!Mv) return nullptr;
	FollowerComp = Mv;
	FollowerWorld = Owner->GetWorld();

	// Pin the tick ordering EXACTLY: this component ticks after the follower's movement component, so the
	// snapshot it reads is the one CopyFromJSBSim just refreshed, and the candidate it submits is ready for
	// the next CopyToJSBSim consume. This is set from OUR side -- no change to the JSBSim plugin -- and
	// does not depend on component registration order.
	if (!bTickPrerequisiteBound)
	{
		AddTickPrerequisiteComponent(Mv);
		bTickPrerequisiteBound = true;
	}
	return Mv;
}

void UFormationRuntimeOwnerV2::FallBack(EFormationRuntimeFallbackV2 Reason)
{
	check(IsInGameThread());
	if (UJSBSimMovementComponent *F = FollowerComp.Get())
	{
		Arb::RequestLegacyFallback(F);   // immediate: next consume is Legacy, no blend
	}
	Producer->Reset();
	Phase = EFormationRuntimePhaseV2::Idle;
	// Any fallback requires a NEW explicit enable to resume -- recovering from a hardover or a lost leader
	// must never silently resume Formation. A held (same-sequence) command will NOT restart it.
	bFormationRequested = false;
	LastFollowerSimTimeS = -1.0;
	WarmTicks = 0;
	ActiveUpdateCount = 0;
	LastFallback = Reason;
}

bool UFormationRuntimeOwnerV2::SafetyOk(UJSBSimMovementComponent *Follower, UJSBSimMovementComponent *Leader,
                                        EFormationRuntimeFallbackV2 &OutReason) const
{
	if (!Follower)                     { OutReason = EFormationRuntimeFallbackV2::FollowerLost;    return false; }
	if (Follower->GetWorld() != FollowerWorld.Get()) { OutReason = EFormationRuntimeFallbackV2::IdentityChanged; return false; }
	if (!Leader)                       { OutReason = EFormationRuntimeFallbackV2::LeaderLost;      return false; }
	if (Leader->GetWorld() != Follower->GetWorld()) { OutReason = EFormationRuntimeFallbackV2::WorldMismatch; return false; }

	const AActor *Owner = Follower->GetOwner();
	const UHealthComponent *H = Owner ? Owner->FindComponentByClass<UHealthComponent>() : nullptr;
	if (!H)                             { OutReason = EFormationRuntimeFallbackV2::UnknownHealth;   return false; }
	if (!H->IsAlive())                  { OutReason = EFormationRuntimeFallbackV2::NotAlive;        return false; }
	return true;
}

bool UFormationRuntimeOwnerV2::ApplyOperationalRequest(bool bFormation,
                                                       UJSBSimMovementComponent *Leader, const FString &InLeaderLabel,
                                                       double FrontM, double RightM, double UpM,
                                                       int64 Sequence, double CommandTimestampS_,
                                                       EFormationRuntimeFallbackV2 &OutReason)
{
	check(IsInGameThread());
	OutReason = EFormationRuntimeFallbackV2::None;

	// An explicit Legacy/disable request: immediate fallback. Idempotent -- disabling an already-idle
	// aircraft is a no-op.
	if (!bFormation)
	{
		if (bFormationRequested || Phase != EFormationRuntimePhaseV2::Idle)
		{
			FallBack(EFormationRuntimeFallbackV2::ExplicitDisable);
		}
		bFormationRequested = false;
		return true;
	}

	// Formation enable. A held command (same sequence) is a no-op: the receiver re-pushes the persistent
	// setpoint every 60 Hz, and re-priming on every packet would be exactly the churn this guards against.
	if (Sequence == AppliedSequence && bFormationRequested)
	{
		return true;
	}
	// Out-of-order / replayed sequence is refused and does NOT re-execute.
	if (Sequence <= AppliedSequence)
	{
		OutReason = EFormationRuntimeFallbackV2::SequenceReplay;
		LastFallback = OutReason;
		return false;
	}

	// A genuinely new command. Validate it before it can change anything.
	if (!Leader)
	{
		OutReason = EFormationRuntimeFallbackV2::NoLeaderSpecified;
		LastFallback = OutReason;
		return false;
	}
	if (!FiniteInRange(FrontM) || !FiniteInRange(RightM) || !FiniteInRange(UpM))
	{
		OutReason = EFormationRuntimeFallbackV2::NonFiniteSlot;
		LastFallback = OutReason;
		return false;
	}
	// Staleness is checked only on the enabling edge (a genuinely new sequence). A held command must not
	// rot as wall-time advances.
	const double Age = FApp::GetCurrentTime() - CommandTimestampS_;
	if (CommandTimestampS_ >= 0.0 && Age > kMaxCommandAgeS)
	{
		OutReason = EFormationRuntimeFallbackV2::StaleCommand;
		LastFallback = OutReason;
		return false;
	}

	const bool bLeaderChanged = bFormationRequested && (InLeaderLabel != LeaderLabel);

	if (!bFormationRequested)
	{
		// Rising edge: Legacy -> Formation. The tick will Warm -> Prime -> activate.
		bFormationRequested = true;
		LeaderLabel = InLeaderLabel;
		LeaderComp = Leader;
		SlotFrontM = FrontM; SlotRightM = RightM; SlotUpM = UpM;
		Producer->SetSlot({FrontM, RightM, UpM});
		Phase = EFormationRuntimePhaseV2::Idle;   // tick starts Warming
		LastFollowerSimTimeS = -1.0;
		WarmTicks = 0;
	}
	else if (bLeaderChanged)
	{
		// A new leader is a new formation problem: fall back immediately, reset, and re-handshake against
		// the new leader. The old ticket/candidate cannot be reused.
		FallBack(EFormationRuntimeFallbackV2::LeaderLost);
		bFormationRequested = true;
		LeaderLabel = InLeaderLabel;
		LeaderComp = Leader;
		SlotFrontM = FrontM; SlotRightM = RightM; SlotUpM = UpM;
		Producer->SetSlot({FrontM, RightM, UpM});
		Phase = EFormationRuntimePhaseV2::Idle;
	}
	else
	{
		// Same leader, new sequence: a slot update. Apply it to the live producer WITHOUT a re-prime.
		LeaderComp = Leader;   // refresh the weak pointer
		SlotFrontM = FrontM; SlotRightM = RightM; SlotUpM = UpM;
		Producer->SetSlot({FrontM, RightM, UpM});
	}

	AppliedSequence = Sequence;
	CommandTimestampS = CommandTimestampS_;
	return true;
}

void UFormationRuntimeOwnerV2::TickComponent(float DeltaTime, ELevelTick TickType,
                                             FActorComponentTickFunction *ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	check(IsInGameThread());

	UJSBSimMovementComponent *Follower = ResolveFollower();

	// Legacy (nothing requested): the aircraft is transparent legacy. If we were mid-handoff, this only
	// happens after a fallback already set Idle, so there is nothing to do.
	if (!bFormationRequested)
	{
		return;
	}

	UJSBSimMovementComponent *Leader = LeaderComp.Get();

	// Safety outranks everything. Falling / lost leader / identity change / world mismatch / unknown health
	// all drop to Legacy immediately, before any candidate is produced.
	EFormationRuntimeFallbackV2 Reason = EFormationRuntimeFallbackV2::None;
	if (!SafetyOk(Follower, Leader, Reason))
	{
		FallBack(Reason);
		return;
	}

	// Read the freshest snapshot (the movement component refreshed it this frame via CopyFromJSBSim, and we
	// tick after it). A transiently invalid frame is skipped, not a fallback.
	FJsbFlightSnapshot Snap{};
	if (!Follower->GetJsbFlightSnapshot(Snap) || !Snap.bValidFrame)
	{
		return;
	}

	// Cadence: drive the producer ONCE per genuine sim-time advance, with dt from the sim clock -- never a
	// wall-clock dt, never twice on the same SimTimeSec.
	const double SimT = Snap.SimTimeSec;
	if (LastFollowerSimTimeS < 0.0)
	{
		LastFollowerSimTimeS = SimT;   // first sample: need a second to form a dt
		return;
	}
	if (SimT <= LastFollowerSimTimeS)
	{
		return;   // sim time did not advance (paused / same frame): produce nothing
	}
	const double Dt = SimT - LastFollowerSimTimeS;
	if (!FMath::IsFinite(Dt) || Dt <= 0.0 || Dt > kMaxDtS || Snap.bHolding)
	{
		// A reset / hold / discontinuity: once we are actually flying Formation this is a safety fallback;
		// before that it is just a frame to skip.
		if (Phase == EFormationRuntimePhaseV2::Active)
		{
			FallBack(EFormationRuntimeFallbackV2::SimTimeDiscontinuity);
		}
		LastFollowerSimTimeS = SimT;
		return;
	}

	using FormationControlV2::EProducerResult;
	switch (Phase)
	{
	case EFormationRuntimePhaseV2::Idle:
		Phase = EFormationRuntimePhaseV2::Warming;
		WarmTicks = 0;
		// fallthrough into Warming this frame
	case EFormationRuntimePhaseV2::Warming:
	{
		Producer->WarmChain(Follower, Leader, Dt);
		++WarmTicks;
		if (Producer->IsChainReady())
		{
			Phase = EFormationRuntimePhaseV2::Priming;
		}
		else if (WarmTicks > kMaxWarmTicks)
		{
			FallBack(EFormationRuntimeFallbackV2::ProducerInvalid);
		}
		break;
	}
	case EFormationRuntimePhaseV2::Priming:
	{
		const FormationControlV2::FProducerFrameResult R = Producer->BeginHandoff(Follower, Leader, Dt);
		LastProducerResult = R.Result;
		if (R.Result == EProducerResult::Ok)
		{
			PrimeGeneration = R.Generation;
			BaselineConsumeSequence = R.BaselineConsumeSequence;
			Phase = EFormationRuntimePhaseV2::AwaitingActivation;
		}
		else
		{
			FallBack(EFormationRuntimeFallbackV2::ProducerRejected);
		}
		break;
	}
	case EFormationRuntimePhaseV2::AwaitingActivation:
	{
		const Arb::EPrimeState PS = Arb::GetPrimeState(Follower);
		if (PS == Arb::EPrimeState::ActiveFormation)
		{
			Phase = EFormationRuntimePhaseV2::Active;
			ActiveUpdateCount = 0;
		}
		else if (PS == Arb::EPrimeState::IdleLegacy)
		{
			// The boundary refused activation (intervening consume / stale candidate / Falling). Reset and
			// let a new enable try again.
			FallBack(EFormationRuntimeFallbackV2::ProducerRejected);
		}
		// else: still pending -- wait for the consume boundary.
		break;
	}
	case EFormationRuntimePhaseV2::Active:
	{
		// The arbiter may have fallen back on its own (candidate stale, Falling caught at the boundary).
		if (Arb::GetMode(Follower) != Arb::ECommandMode::FormationControlV2)
		{
			FallBack(EFormationRuntimeFallbackV2::ProducerInvalid);
			break;
		}
		const FormationControlV2::FProducerFrameResult R = Producer->Update(Follower, Leader, Dt);
		LastProducerResult = R.Result;
		if (R.Result == EProducerResult::Ok && R.bSubmitted)
		{
			++ActiveUpdateCount;
			CandidateGeneration = R.CandidateGeneration;
		}
		// A transiently not-ready chain frame is not a fallback: if it persists the candidate goes stale and
		// the arbiter falls back on its own, which we detect above next frame.
		break;
	}
	}

	LastFollowerSimTimeS = SimT;
}
