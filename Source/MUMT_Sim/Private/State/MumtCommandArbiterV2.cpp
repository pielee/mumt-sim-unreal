#include "State/MumtCommandArbiterV2.h"

#include "JSBSimMovementComponent.h"
#include "HealthComponent.h"
#include "GameFramework/Actor.h"
#include "Misc/App.h"
#include "Engine/World.h"

DEFINE_LOG_CATEGORY_STATIC(LogMumtArbiter, Display, All);

namespace MumtCommandArbiterV2
{
namespace
{

// EVERY field the FDM actually consumes. Not a hash, not a memcmp: padding and unconsumed members
// would make either of those lie in both directions. If a future field starts being consumed by
// CopyToJSBSim or ApplyEnginesCommands, it must be added here or the transparency proof is hollow.
//
// Flight controls consumed by CopyToJSBSim (15):
//   Aileron RollTrim Elevator PitchTrim Rudder YawTrim Flap SpeedBrake Spoiler
//   LeftBrake RightBrake CenterBrake ParkingBrake GearDown Steer
//   (Steer is carried even though CopyToJSBSim currently steers from Rudder -- if that is ever fixed
//    the comparison must already be watching it.)
int32 CountChangedCommandFields(const FFlightControlCommands &A, const FFlightControlCommands &B)
{
	int32 N = 0;
	auto D = [&N](double X, double Y) { if (X != Y) ++N; };
	D(A.Aileron, B.Aileron);
	D(A.Elevator, B.Elevator);
	D(A.Rudder, B.Rudder);
	D(A.YawTrim, B.YawTrim);
	D(A.PitchTrim, B.PitchTrim);
	D(A.RollTrim, B.RollTrim);
	D(A.Steer, B.Steer);
	D(A.LeftBrake, B.LeftBrake);
	D(A.RightBrake, B.RightBrake);
	D(A.CenterBrake, B.CenterBrake);
	D(A.ParkingBrake, B.ParkingBrake);
	D(A.GearDown, B.GearDown);
	D(A.Flap, B.Flap);
	D(A.SpeedBrake, B.SpeedBrake);
	D(A.Spoiler, B.Spoiler);
	return N;
}

// Every field ApplyEnginesCommands consumes (14), per engine.
int32 CountChangedEngineFields(const TArray<FEngineCommand> &A, const TArray<FEngineCommand> &B)
{
	if (A.Num() != B.Num())
	{
		return FMath::Max(A.Num(), B.Num());   // an engine appeared or vanished: maximally different
	}
	int32 N = 0;
	for (int32 i = 0; i < A.Num(); ++i)
	{
		const FEngineCommand &X = A[i];
		const FEngineCommand &Y = B[i];
		if (X.Throttle != Y.Throttle) ++N;
		if (X.Mixture != Y.Mixture) ++N;
		if (X.Starter != Y.Starter) ++N;
		if (X.Running != Y.Running) ++N;
		if (X.PropellerAdvance != Y.PropellerAdvance) ++N;
		if (X.PropellerFeather != Y.PropellerFeather) ++N;
		if (X.Magnetos != Y.Magnetos) ++N;
		if (X.Augmentation != Y.Augmentation) ++N;
		if (X.Injection != Y.Injection) ++N;
		if (X.Ignition != Y.Ignition) ++N;
		if (X.Reverse != Y.Reverse) ++N;
		if (X.CutOff != Y.CutOff) ++N;
		if (X.GeneratorPower != Y.GeneratorPower) ++N;
		if (X.Condition != Y.Condition) ++N;
	}
	return N;
}

bool IsBlockFinite(const FJSBSimResolvedCommandBlock &R)
{
	const FFlightControlCommands &C = R.Commands;
	const bool bCmds = FMath::IsFinite(C.Aileron) && FMath::IsFinite(C.Elevator) && FMath::IsFinite(C.Rudder)
		&& FMath::IsFinite(C.YawTrim) && FMath::IsFinite(C.PitchTrim) && FMath::IsFinite(C.RollTrim)
		&& FMath::IsFinite(C.Steer) && FMath::IsFinite(C.LeftBrake) && FMath::IsFinite(C.RightBrake)
		&& FMath::IsFinite(C.CenterBrake) && FMath::IsFinite(C.ParkingBrake) && FMath::IsFinite(C.GearDown)
		&& FMath::IsFinite(C.Flap) && FMath::IsFinite(C.SpeedBrake) && FMath::IsFinite(C.Spoiler);
	if (!bCmds) return false;
	for (const FEngineCommand &E : R.EngineCommands)
	{
		if (!FMath::IsFinite(E.Throttle) || !FMath::IsFinite(E.Mixture) || !FMath::IsFinite(E.PropellerAdvance))
		{
			return false;
		}
	}
	return true;
}

// OBSERVED, never enforced. Phase A must not clamp: that would change what the FDM consumes, which is
// precisely what this phase promises not to do.
//
// Every NORMALIZED consumed field is checked, not just the four stick axes -- the declared ranges come
// from FDMTypes.h ([-1..1] for the surfaces and trims, [0..1] for the rest). Checking only a subset and
// calling the counter "range violations" would be a lie by omission: a Blueprint writing an out-of-range
// flap or brake would read as clean.
bool IsBlockOutOfRange(const FJSBSimResolvedCommandBlock &R)
{
	const FFlightControlCommands &C = R.Commands;
	auto BadSigned   = [](double V) { return V < -1.0 || V > 1.0; };   // [-1..1]
	auto BadUnsigned = [](double V) { return V <  0.0 || V > 1.0; };   // [0..1]

	if (BadSigned(C.Aileron) || BadSigned(C.Elevator) || BadSigned(C.Rudder)) return true;
	if (BadSigned(C.YawTrim) || BadSigned(C.PitchTrim) || BadSigned(C.RollTrim)) return true;
	if (BadSigned(C.Steer)) return true;
	if (BadUnsigned(C.LeftBrake) || BadUnsigned(C.RightBrake) || BadUnsigned(C.CenterBrake)) return true;
	if (BadUnsigned(C.ParkingBrake) || BadUnsigned(C.GearDown)) return true;
	if (BadUnsigned(C.Flap) || BadUnsigned(C.SpeedBrake) || BadUnsigned(C.Spoiler)) return true;

	for (const FEngineCommand &E : R.EngineCommands)
	{
		if (BadUnsigned(E.Throttle) || BadUnsigned(E.Mixture) || BadUnsigned(E.PropellerAdvance)) return true;
	}
	return false;
}

bool IsCandidateFinite(const FFormationCandidate &K)
{
	return FMath::IsFinite(K.AileronNorm) && FMath::IsFinite(K.ElevatorNorm) && FMath::IsFinite(K.RudderNorm)
	    && FMath::IsFinite(K.ThrottleNorm) && FMath::IsFinite(K.SpeedBrakeNorm);
}

#if WITH_DEV_AUTOMATION_TESTS
double GClockOverrideS = -1.0;   // < 0 == no override; the ONLY way this is ever set is from a test
#endif

// The arbiter's single notion of "now". In a shipping build this compiles down to FApp::GetCurrentTime()
// and nothing else: the override does not exist there.
double NowS()
{
#if WITH_DEV_AUTOMATION_TESTS
	if (GClockOverrideS >= 0.0) return GClockOverrideS;
#endif
	return FApp::GetCurrentTime();
}

// Freshness is asked TWICE about the same candidate -- once when it is submitted and again at the consume
// boundary where the handoff actually lands -- so it lives in one place. A candidate that was fresh at
// submission is not thereby fresh forever, and a second question with a different answer is the whole point.
bool IsCandidateFresh(const FFormationCandidate &K)
{
	const double Age = NowS() - K.TimestampS;
	return K.TimestampS >= 0.0 && Age >= 0.0 && Age <= kCandidateMaxAgeS;
}

struct FPerAircraft
{
	FString ActorName;
	ECommandMode Mode = ECommandMode::LegacyOrManual;
	FFormationCandidate Candidate;
	bool bHaveCandidate = false;

	// PHASE C: the accepted candidate's full 29-field block, if it carried one. When present the arbiter
	// applies all 29 fields in Formation mode (non-producer fields = baseline); when absent it overlays
	// only the five controlled axes on the live legacy block, exactly as Phase B did.
	bool bCandidateHasFullBlock = false;
	FFlightControlCommands CandidateFullCommands;
	TArray<FEngineCommand> CandidateFullEngineCommands;

	// the decision taken in Resolve, consumed by OnResolved
	uint64 LastResolveSeq = 0;
	uint64 LastObservedSeq = 0;
	EResolutionSource LastSource = EResolutionSource::Legacy;
	EFallbackReason LastFallback = EFallbackReason::None;
	FFlightControlCommands LegacyAtResolve;
	TArray<FEngineCommand> LegacyEngineAtResolve;
	bool bResolvePending = false;

	// ---- Phase B ----
	FResolvedCommandSnapshot LastResolved;   // the 29-field block the FDM actually consumed
	EPrimeState PrimeState = EPrimeState::IdleLegacy;
	FPrimeTicket Ticket;
	uint64 NextPrimeGeneration = 1;          // strictly monotonic per aircraft; 0 means "no ticket"
	bool bAwaitingHandoffMeasure = false;    // the next Formation consume is the handoff
	EPrimeFailure LastBoundaryFailure = EPrimeFailure::None;   // why the boundary last refused to hand over
	FHandoffDelta Handoff;

	FAircraftCounters Counters;
};

bool GEnabled = false;

// OWNERSHIP OF THE SINGLE-CAST RESOLVER, BY IDENTITY -- not by a boolean.
//
// A boolean "we bound it" is a lie waiting to happen: if another binder calls BindStatic on the same
// single-cast delegate, it silently replaces our instance and our flag stays true. We would then happily
// Unbind() on shutdown and delete SOMEBODY ELSE'S resolver. That is precisely the silent-takeover bug
// this project exists to eliminate, so ownership is established the only way that cannot be faked: the
// handle of the delegate instance we bound, compared against the handle currently in the delegate.
FDelegateHandle GResolverHandle;         // the handle of OUR bound resolver instance
int64 GOwnershipConflicts = 0;           // somebody else already owned it -- we never evict them
int64 GOwnershipLost = 0;                // it WAS ours and was taken over behind our back
FDelegateHandle GObserverHandle;         // our entry in the multicast, so we remove only ours
FDelegateHandle GWorldCleanupHandle;

// Is the resolver currently bound to the instance WE bound?
bool WeOwnResolver()
{
	return GResolverHandle.IsValid()
	    && UJSBSimMovementComponent::CommandResolver.GetHandle() == GResolverHandle;
}
FString GScenario;
FCounters GCounters;
TMap<TWeakObjectPtr<const UJSBSimMovementComponent>, FPerAircraft> GAircraft;
TArray<FString> GEventLog;
constexpr int32 kMaxEventLines = 3000;

void LogEvent(const FString &Line)
{
	if (GEventLog.Num() < kMaxEventLines) GEventLog.Add(Line);
}

// Drops entries whose component has been destroyed (actor destruction, PIE shutdown). Without this the
// registry would keep dead keys, and a mode/candidate set on one aircraft could outlive it.
//
// A TWeakObjectPtr going invalid is NOT cleanup on its own -- the entry, its mode and its candidate all
// still sit in the map. This is what actually removes them, and it is called on every consume (the map
// holds a handful of aircraft, so it is free) as well as on world teardown.
void PurgeDead()
{
	check(IsInGameThread());
	for (auto It = GAircraft.CreateIterator(); It; ++It)
	{
		if (!It.Key().IsValid()) It.RemoveCurrent();
	}
}

// PIE stop / level change / world teardown.
//
// WORLD-SCOPED, not a blanket wipe. The registry is global but the aircraft in it are not: an editor
// world and a PIE world can coexist, and clearing everything would silently delete the mode, candidate
// and counters of aircraft belonging to a world that is still alive. That is the same class of bug --
// one actor's teardown quietly destroying another's state -- that this whole effort exists to remove.
//
// Order matters: a component whose weak pointer has already gone invalid cannot be asked for its world,
// so invalidity is checked FIRST and those entries are dropped (they belong to nobody now anyway).
void OnWorldCleanup(UWorld *World, bool /*bSessionEnded*/, bool /*bCleanupResources*/)
{
	check(IsInGameThread());
	for (auto It = GAircraft.CreateIterator(); It; ++It)
	{
		const UJSBSimMovementComponent *Component = It.Key().Get();
		if (Component == nullptr)
		{
			It.RemoveCurrent();          // dead: belongs to no world
			continue;
		}
		if (World != nullptr && Component->GetWorld() == World)
		{
			It.RemoveCurrent();          // belongs to the world being torn down
		}
		// else: a live component in a DIFFERENT world -- its mode, candidate and generation survive.
	}
}

FPerAircraft &Find(const UJSBSimMovementComponent *Component)
{
	FPerAircraft &S = GAircraft.FindOrAdd(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	if (S.ActorName.IsEmpty() && Component && Component->GetOwner())
	{
		S.ActorName = Component->GetOwner()->GetActorNameOrLabel();
	}
	return S;
}

// Health is the top of the priority order. "No health component" is NOT treated as Alive: we cannot
// prove the aircraft is flyable, so we refuse to override and let the legacy block through unchanged --
// which is exactly what such an aircraft does today.
enum class EHealth : uint8 { Alive, NotAlive, Unknown };

EHealth GetHealth(const UJSBSimMovementComponent *Component)
{
	const AActor *Owner = Component ? Component->GetOwner() : nullptr;
	if (!Owner) return EHealth::Unknown;
	const UHealthComponent *H = Owner->FindComponentByClass<UHealthComponent>();
	if (!H) return EHealth::Unknown;
	return H->IsAlive() ? EHealth::Alive : EHealth::NotAlive;
}

// ---- the resolver -----------------------------------------------------------------------------
void Resolve(const UJSBSimMovementComponent *Component, uint64 ConsumeSequence,
             const FFlightControlCommands &LegacyCommands, const TArray<FEngineCommand> &LegacyEngineCommands,
             FJSBSimResolvedCommandBlock &Resolved)
{
	check(IsInGameThread());   // the consume boundary and the registry are game-thread only
	if (!GEnabled || Component == nullptr) return;

	FPerAircraft &S = Find(Component);
	++GCounters.ResolverCallCount;

	if (S.bResolvePending && S.LastResolveSeq == ConsumeSequence)
	{
		++GCounters.DuplicateResolutionCount;
	}
	S.LastResolveSeq = ConsumeSequence;
	S.bResolvePending = true;
	S.LegacyAtResolve = LegacyCommands;
	S.LegacyEngineAtResolve = LegacyEngineCommands;

	// Resolved already IS the legacy block (CopyToJSBSim copied it). Every path below that does not
	// explicitly overwrite a field therefore leaves the legacy value in place -- including the field
	// mixture between writers, which is current behaviour and must survive.
	const EHealth Health = GetHealth(Component);

	// 1. Falling outranks EVERYTHING, including a perfectly good Formation candidate. The hardover is
	//    what puts a dead aircraft into the ground; nothing may fly it out.
	if (Health == EHealth::NotAlive)
	{
		S.LastSource = EResolutionSource::FallingLegacy;
		S.LastFallback = EFallbackReason::Falling;

		// SAFETY PRE-EMPTS BUMPLESS. A handoff in flight is abandoned outright -- no blend, no delay --
		// and the ticket is destroyed so it cannot be replayed if the aircraft is somehow revived. The
		// mode goes back to Legacy too: recovering from a hardover must not silently resume Formation.
		if (S.PrimeState != EPrimeState::IdleLegacy)
		{
			++GCounters.PrimeCancelledByFallingCount;
			LogEvent(FString::Printf(
				TEXT("[PRIME] CANCELLED_BY_FALLING actor=%s state=%s generation=%llu"),
				*S.ActorName, PrimeStateName(S.PrimeState), S.Ticket.Generation));
			S.PrimeState = EPrimeState::IdleLegacy;
			S.Ticket = FPrimeTicket{};
			S.bHaveCandidate = false;
			S.bCandidateHasFullBlock = false;
			S.bAwaitingHandoffMeasure = false;
			S.Mode = ECommandMode::LegacyOrManual;
			S.Counters.Mode = S.Mode;
		}
		return;
	}

	// 1b. THE ACTIVATION BOUNDARY.
	//
	// ActivateFormationForTesting() only ASKS. The switch itself happens here, at the consume boundary,
	// because only here can we check the thing that actually matters: is the block the FDM last consumed
	// still the one the ticket was cut from?
	//
	// If ANY Legacy consume slipped in between the prime and the activation landing, the baseline is
	// stale -- the aircraft is now flying something the candidate was never continuous with -- and
	// switching would step. Flipping the mode inside Activate() could not see this: the intervening
	// consume happens after it returns.
	if (S.PrimeState == EPrimeState::ActivationPending)
	{
		const bool bBaselineStillCurrent =
			S.Ticket.bValid && S.LastResolved.bValid
			&& S.LastResolved.ConsumeSequence == S.Ticket.BaselineConsumeSequence;

		if (!bBaselineStillCurrent)
		{
			++GCounters.InterveningConsumeRejectedCount;
			S.LastBoundaryFailure = EPrimeFailure::InterveningConsume;
			LogEvent(FString::Printf(
				TEXT("[PRIME] INTERVENING_CONSUME actor=%s generation=%llu baseline_seq=%llu last_resolved_seq=%llu "
				     "-- the aircraft moved on; refusing to hand over to a stale candidate"),
				*S.ActorName, S.Ticket.Generation, S.Ticket.BaselineConsumeSequence,
				S.LastResolved.bValid ? S.LastResolved.ConsumeSequence : 0));

			// The ticket and its candidate are dead. Progress requires a NEW prime against the command
			// the aircraft is actually flying now.
			S.PrimeState = EPrimeState::IdleLegacy;
			S.Ticket = FPrimeTicket{};
			S.bHaveCandidate = false;
			S.bCandidateHasFullBlock = false;
			S.Mode = ECommandMode::LegacyOrManual;
			S.Counters.Mode = S.Mode;
			S.LastSource = EResolutionSource::Legacy;
			S.LastFallback = EFallbackReason::None;
			return;
		}

		// The baseline is current -- but is the CANDIDATE? It was fresh when it was submitted; that was
		// then. Freshness is not a property a candidate keeps: the producer may have died, the frame that
		// built it may be long gone, and handing the aircraft to a command computed for a state it has
		// since left is exactly the step this phase exists to prevent. So it is asked again, HERE, at the
		// only moment that matters -- and this refusal is NOT the same as an intervening consume, so it
		// does not get to hide behind that counter.
		if (!S.bHaveCandidate || !IsCandidateFresh(S.Candidate))
		{
			++GCounters.ActivationStaleRejectedCount;
			S.LastBoundaryFailure = EPrimeFailure::StaleCandidate;
			LogEvent(FString::Printf(
				TEXT("[PRIME] STALE_AT_BOUNDARY actor=%s generation=%llu candidate_age=%.6f max_age=%.6f "
				     "-- the candidate rotted between submission and the handoff; staying on Legacy"),
				*S.ActorName, S.Ticket.Generation,
				S.bHaveCandidate ? NowS() - S.Candidate.TimestampS : -1.0, kCandidateMaxAgeS));

			S.PrimeState = EPrimeState::IdleLegacy;
			S.Ticket = FPrimeTicket{};
			S.bHaveCandidate = false;
			S.bCandidateHasFullBlock = false;
			S.Mode = ECommandMode::LegacyOrManual;
			S.Counters.Mode = S.Mode;
			S.LastSource = EResolutionSource::Legacy;
			S.LastFallback = EFallbackReason::Stale;
			return;
		}

		// The baseline is still what the aircraft is flying, and the candidate is still worth flying it
		// with. Hand over on THIS consume.
		S.LastBoundaryFailure = EPrimeFailure::None;
		S.Mode = ECommandMode::FormationControlV2;
		S.Counters.Mode = S.Mode;
		S.PrimeState = EPrimeState::ActiveFormation;
		S.bAwaitingHandoffMeasure = true;
		++GCounters.ActivationGrantedCount;
		++GCounters.ModeTransitionCount;
		LogEvent(FString::Printf(TEXT("[PRIME] ACTIVATED actor=%s generation=%llu at_consume=%llu"),
			*S.ActorName, S.Ticket.Generation, ConsumeSequence));
	}

	// 2. Default mode: pass the legacy block through untouched.
	if (S.Mode != ECommandMode::FormationControlV2)
	{
		S.LastSource = EResolutionSource::Legacy;
		S.LastFallback = EFallbackReason::None;
		return;
	}

	// 3. Formation mode, but we could not establish that the aircraft is alive.
	if (Health == EHealth::Unknown)
	{
		S.LastSource = EResolutionSource::Legacy;
		S.LastFallback = EFallbackReason::UnknownHealth;
		return;
	}

	// 4. Formation mode: the candidate has to earn it. Any doubt -> legacy.
	if (!S.bHaveCandidate)
	{
		S.LastSource = EResolutionSource::Legacy;
		S.LastFallback = EFallbackReason::NoCandidate;
		return;
	}
	const FFormationCandidate &K = S.Candidate;
	if (!K.bValid || !K.bCommandReady)
	{
		S.LastSource = EResolutionSource::Legacy;
		S.LastFallback = EFallbackReason::Invalid;
		return;
	}
	if (!K.bFinite || !IsCandidateFinite(K))
	{
		S.LastSource = EResolutionSource::Legacy;
		S.LastFallback = EFallbackReason::NonFinite;
		return;
	}
	if (!IsCandidateFresh(K))
	{
		S.LastSource = EResolutionSource::Legacy;
		S.LastFallback = EFallbackReason::Stale;
		return;
	}

	// 5. Accepted.
	if (S.bCandidateHasFullBlock)
	{
		// PHASE C: apply the producer's FULL 29-field block. The producer built it by copying the immutable
		// prime baseline and overwriting only the fields it owns, so the non-producer fields already carry
		// the baseline value. Applying the whole block makes every field continuous with what the aircraft
		// was flying at the handoff -- not a mixture of producer axes and one-more-frame of legacy writers.
		//
		// The block is applied wholesale, but only to fields the FDM actually consumes: the engine array is
		// clamped to what the aircraft has, so a candidate can never invent an engine.
		Resolved.Commands = S.CandidateFullCommands;
		const int32 N = FMath::Min(Resolved.EngineCommands.Num(), S.CandidateFullEngineCommands.Num());
		for (int32 i = 0; i < N; ++i)
		{
			Resolved.EngineCommands[i] = S.CandidateFullEngineCommands[i];
		}
	}
	else
	{
		// PHASE B path: overlay ONLY the axes the Formation chain owns; everything else -- trims, flaps,
		// brakes, gear, the Blueprints' fields -- keeps its legacy value. Formation does not get to silently
		// zero a field just because it has no opinion about it.
		Resolved.Commands.Aileron = K.AileronNorm;
		Resolved.Commands.Elevator = K.ElevatorNorm;
		Resolved.Commands.Rudder = K.RudderNorm;
		Resolved.Commands.SpeedBrake = K.SpeedBrakeNorm;
		if (Resolved.EngineCommands.Num() > 0)
		{
			Resolved.EngineCommands[0].Throttle = K.ThrottleNorm;
		}
	}
	S.LastSource = EResolutionSource::Formation;
	S.LastFallback = EFallbackReason::None;
	S.Counters.CandidateGeneration = K.Generation;
}

// ---- the audit of what was actually resolved ---------------------------------------------------
void OnResolved(const UJSBSimMovementComponent *Component, uint64 ConsumeSequence,
                const FFlightControlCommands &LegacyCommands, const TArray<FEngineCommand> &LegacyEngineCommands,
                const FJSBSimResolvedCommandBlock &Resolved)
{
	check(IsInGameThread());
	if (!GEnabled || Component == nullptr) return;

	// Once per consume: a destroyed aircraft's entry (and its mode/candidate) leaves the registry here.
	PurgeDead();

	FPerAircraft &S = Find(Component);
	++GCounters.ConsumeCount;
	++S.Counters.Consumes;

	if (!S.bResolvePending || S.LastResolveSeq != ConsumeSequence)
	{
		++GCounters.MissingResolutionCount;   // a consume the resolver never saw
	}
	S.bResolvePending = false;
	S.LastObservedSeq = ConsumeSequence;

	// The resolver is handed the legacy block as CONST, so it cannot write it. This checks that claim
	// against reality instead of trusting the type system: the legacy block seen here must be the one
	// seen at resolve time.
	if (CountChangedCommandFields(S.LegacyAtResolve, LegacyCommands) != 0
		|| CountChangedEngineFields(S.LegacyEngineAtResolve, LegacyEngineCommands) != 0)
	{
		++GCounters.LegacyBlockMutationCount;
	}

	const int32 Changed = CountChangedCommandFields(LegacyCommands, Resolved.Commands)
	                    + CountChangedEngineFields(LegacyEngineCommands, Resolved.EngineCommands);

	switch (S.LastSource)
	{
	case EResolutionSource::Formation:
		++GCounters.FormationResolutionCount;
		++S.Counters.FormationResolutions;
		break;
	case EResolutionSource::FallingLegacy:
		++GCounters.FallingResolutionCount;
		++S.Counters.FallingResolutions;
		// FALLING AND LEGACY BOTH PASS THE BLOCK THROUGH: any change here is a bug, not a policy.
		GCounters.LegacyChangedFieldCount += Changed;
		S.Counters.LegacyChangedFields += Changed;
		break;
	default:
		++GCounters.LegacyResolutionCount;
		++S.Counters.LegacyResolutions;
		GCounters.LegacyChangedFieldCount += Changed;
		S.Counters.LegacyChangedFields += Changed;
		break;
	}

	switch (S.LastFallback)
	{
	case EFallbackReason::Stale:     ++GCounters.StaleFallbackCount;         ++S.Counters.StaleFallbacks; break;
	case EFallbackReason::Invalid:   ++GCounters.InvalidFallbackCount;       ++S.Counters.InvalidFallbacks; break;
	case EFallbackReason::NonFinite: ++GCounters.NonFiniteFallbackCount;     ++S.Counters.NonFiniteFallbacks; break;
	case EFallbackReason::NoCandidate: ++GCounters.NoCandidateFallbackCount; break;
	case EFallbackReason::UnknownHealth: ++GCounters.UnknownHealthFallbackCount; break;
	default: break;
	}

	if (!IsBlockFinite(Resolved))    ++GCounters.ResolvedNonFiniteCount;
	if (IsBlockOutOfRange(Resolved)) ++GCounters.ResolvedRangeViolationCount;

	// ---- Phase B: the baseline a future handoff must be continuous with -----------------------------
	// This is the command the FDM ACTUALLY consumed -- not the legacy writer block, which is only an
	// input and can be a shifting mixture of several writers. All 29 fields are kept.
	//
	// THE HANDOFF STEP, measured against the IMMUTABLE TICKET BASELINE -- not against "the previous
	// consume".
	//
	// That distinction is the whole validity of the measurement. Comparing this consume with the previous
	// one would, whenever nothing intervened, be comparing the block with a copy of itself: a zero would
	// prove only that the arbiter did not corrupt its own snapshot. The ticket baseline is a frozen copy
	// taken at RequestPrime and never touched again, so a zero here means the FDM really is consuming the
	// command it was already flying. (PrimeHandoffDeltaNegativeControl exists to prove this can be
	// non-zero.)
	if (S.bAwaitingHandoffMeasure && S.Ticket.bValid && S.Ticket.Baseline.bValid
		&& S.LastSource == EResolutionSource::Formation)
	{
		const FFlightControlCommands &P = S.Ticket.Baseline.Commands;
		const FFlightControlCommands &N = Resolved.Commands;
		const double PrevThr = S.Ticket.Baseline.EngineCommands.Num() > 0
			? S.Ticket.Baseline.EngineCommands[0].Throttle : 0.0;
		const double NewThr  = Resolved.EngineCommands.Num() > 0 ? Resolved.EngineCommands[0].Throttle : 0.0;

		S.Handoff.Aileron    = N.Aileron - P.Aileron;
		S.Handoff.Elevator   = N.Elevator - P.Elevator;
		S.Handoff.Rudder     = N.Rudder - P.Rudder;
		S.Handoff.SpeedBrake = N.SpeedBrake - P.SpeedBrake;
		S.Handoff.Throttle   = NewThr - PrevThr;
		S.Handoff.ConsumeSequence = ConsumeSequence;
		S.Handoff.bMeasured = true;
		S.bAwaitingHandoffMeasure = false;

		++GCounters.HandoffsMeasured;
		const bool bStep = S.Handoff.Aileron != 0.0 || S.Handoff.Elevator != 0.0 || S.Handoff.Rudder != 0.0
		                || S.Handoff.Throttle != 0.0 || S.Handoff.SpeedBrake != 0.0;
		if (bStep) ++GCounters.NonZeroHandoffCount;

		LogEvent(FString::Printf(
			TEXT("[PRIME] HANDOFF actor=%s seq=%llu dAil=%.9f dElv=%.9f dRud=%.9f dThr=%.9f dSpb=%.9f step=%d"),
			*S.ActorName, ConsumeSequence, S.Handoff.Aileron, S.Handoff.Elevator, S.Handoff.Rudder,
			S.Handoff.Throttle, S.Handoff.SpeedBrake, bStep ? 1 : 0));
	}

	S.LastResolved.Commands = Resolved.Commands;
	S.LastResolved.EngineCommands = Resolved.EngineCommands;
	S.LastResolved.ConsumeSequence = ConsumeSequence;
	S.LastResolved.TimestampS = NowS();
	// Identity comes from the component the RESOLVER was invoked with -- never from a caller's claim.
	S.LastResolved.Component = Component;
	S.LastResolved.World = Component->GetWorld();
	S.LastResolved.bValid = true;

	++GCounters.ResolutionGeneration;
	S.Counters.LastSource = S.LastSource;
	S.Counters.Mode = S.Mode;

	LogEvent(FString::Printf(
		TEXT("[ARBITER] RESOLVE seq=%llu actor=%s mode=%s source=%s fallback=%s changed_fields=%d "
		     "legacy{ail=%.6f elv=%.6f rud=%.6f spb=%.6f thr=%.6f} resolved{ail=%.6f elv=%.6f rud=%.6f spb=%.6f thr=%.6f}"),
		ConsumeSequence, *S.ActorName,
		S.Mode == ECommandMode::FormationControlV2 ? TEXT("FormationControlV2") : TEXT("LegacyOrManual"),
		SourceName(S.LastSource), FallbackName(S.LastFallback), Changed,
		LegacyCommands.Aileron, LegacyCommands.Elevator, LegacyCommands.Rudder, LegacyCommands.SpeedBrake,
		LegacyEngineCommands.Num() > 0 ? LegacyEngineCommands[0].Throttle : 0.0,
		Resolved.Commands.Aileron, Resolved.Commands.Elevator, Resolved.Commands.Rudder, Resolved.Commands.SpeedBrake,
		Resolved.EngineCommands.Num() > 0 ? Resolved.EngineCommands[0].Throttle : 0.0));
}

} // namespace

const TCHAR *SourceName(EResolutionSource S)
{
	switch (S)
	{
	case EResolutionSource::Formation:     return TEXT("Formation");
	case EResolutionSource::FallingLegacy: return TEXT("FallingLegacy");
	default:                               return TEXT("Legacy");
	}
}

const TCHAR *FallbackName(EFallbackReason R)
{
	switch (R)
	{
	case EFallbackReason::NoCandidate:   return TEXT("NoCandidate");
	case EFallbackReason::Stale:         return TEXT("Stale");
	case EFallbackReason::Invalid:       return TEXT("Invalid");
	case EFallbackReason::NonFinite:     return TEXT("NonFinite");
	case EFallbackReason::Falling:       return TEXT("Falling");
	case EFallbackReason::UnknownHealth: return TEXT("UnknownHealth");
	default:                             return TEXT("None");
	}
}

// "Enabled" means WE are the resolver -- not merely that we once asked to be. If another binder took
// the delegate, the arbiter is not running, and saying otherwise would be a silent recovery: callers
// would believe the consume boundary is arbitrated when it is somebody else's code deciding.
bool IsEnabled() { return GEnabled && WeOwnResolver(); }

int32 GetRegistrySize() { return GAircraft.Num(); }
int64 GetResolverOwnershipConflictCount() { return GOwnershipConflicts; }
int64 GetResolverOwnershipLostCount() { return GOwnershipLost; }

// IDEMPOTENT in both directions, because it is now called from module startup AND from tests, and the
// two must not fight:
//   * enabling twice must not register the observer twice (which would double-count every consume),
//   * disabling twice must not reach into the multicast and remove somebody else's binding -- we only
//     ever remove the handle WE stored,
//   * the resolver is single-cast, so if anything other than us already owns it, silently calling
//     BindStatic would evict that owner without a trace. That is exactly the class of bug this whole
//     effort exists to eliminate, so it is detected and reported rather than papered over.
void SetEnabled(bool bEnable)
{
	check(IsInGameThread());
	if (bEnable)
	{
		// Somebody else owns the consume boundary. Do not evict them, do not "recover" it silently --
		// exactly one resolver may own it, and a takeover has to be visible.
		if (UJSBSimMovementComponent::CommandResolver.IsBound() && !WeOwnResolver())
		{
			++GOwnershipConflicts;
			GEnabled = false;
			UE_LOG(LogMumtArbiter, Error,
				TEXT("[ARBITER] the command resolver is already bound by something else -- refusing to "
				     "evict it. Exactly one resolver may own the consume boundary."));
			return;
		}
		if (!WeOwnResolver())
		{
			UJSBSimMovementComponent::CommandResolver.BindStatic(&Resolve);
			GResolverHandle = UJSBSimMovementComponent::CommandResolver.GetHandle();
		}
		if (!GObserverHandle.IsValid())
		{
			GObserverHandle = UJSBSimMovementComponent::CommandResolvedObserver.AddStatic(&OnResolved);
		}
		if (!GWorldCleanupHandle.IsValid())
		{
			GWorldCleanupHandle = FWorldDelegates::OnWorldCleanup.AddStatic(&OnWorldCleanup);
		}
		GEnabled = true;
	}
	else
	{
		// Unbind ONLY if the delegate still holds the instance we bound. If someone replaced it behind
		// our back, that is theirs now: leave it alone and say so. Removing it would delete a resolver
		// we do not own -- the very failure mode the arbiter was built to prevent.
		if (GResolverHandle.IsValid())
		{
			if (WeOwnResolver())
			{
				UJSBSimMovementComponent::CommandResolver.Unbind();
			}
			else
			{
				++GOwnershipLost;
				UE_LOG(LogMumtArbiter, Warning,
					TEXT("[ARBITER] the command resolver was taken over by another binder -- leaving it "
					     "in place rather than unbinding somebody else's resolver."));
			}
			GResolverHandle.Reset();
		}
		if (GObserverHandle.IsValid())
		{
			// Multicast: remove ONLY the handle we stored, never anyone else's entry.
			UJSBSimMovementComponent::CommandResolvedObserver.Remove(GObserverHandle);
			GObserverHandle.Reset();
		}
		if (GWorldCleanupHandle.IsValid())
		{
			FWorldDelegates::OnWorldCleanup.Remove(GWorldCleanupHandle);
			GWorldCleanupHandle.Reset();
		}
		GAircraft.Reset();
		GEnabled = false;
	}
}

// Resets the MEASUREMENT, not the CONFIGURATION. Counters, the event log and the per-consume bookkeeping
// are cleared; each aircraft's mode and candidate survive.
//
// This distinction is load-bearing. Dropping the registry here silently un-configured every aircraft --
// a caller that reset the session to skip a startup transient would find its Formation aircraft quietly
// back in LegacyOrManual, and the resulting test would "pass" while proving nothing. A session reset
// must never be able to change what the arbiter would decide.
void ResetSession(const FString &ScenarioLabel)
{
	check(IsInGameThread());
	GScenario = ScenarioLabel;
	GCounters = FCounters{};
	GEventLog.Reset();

	PurgeDead();
	for (TPair<TWeakObjectPtr<const UJSBSimMovementComponent>, FPerAircraft> &P : GAircraft)
	{
		FPerAircraft &S = P.Value;
		S.Counters = FAircraftCounters{};
		S.Counters.Mode = S.Mode;   // the counters view mirrors the configured mode, which survives
		S.bResolvePending = false;
		S.LastResolveSeq = 0;
		S.LastObservedSeq = 0;
		S.LastSource = EResolutionSource::Legacy;
		S.LastFallback = EFallbackReason::None;
		// PRESERVED, deliberately: Mode, Candidate, bHaveCandidate, ActorName, and everything Phase B
		// added -- PrimeState, Ticket, NextPrimeGeneration, LastResolved, bAwaitingHandoffMeasure,
		// Handoff. A session reset clears the MEASUREMENT; it must never be able to cancel a handoff in
		// flight or hand back a generation that was already issued.
	}
}

void SetModeForTesting(const UJSBSimMovementComponent *Component, ECommandMode Mode)
{
	check(IsInGameThread());
	if (!Component) return;
	PurgeDead();
	FPerAircraft &S = Find(Component);
	if (S.Mode != Mode)
	{
		++GCounters.ModeTransitionCount;
		S.Mode = Mode;
	}
}

ECommandMode GetMode(const UJSBSimMovementComponent *Component)
{
	if (!Component) return ECommandMode::LegacyOrManual;
	const FPerAircraft *S = GAircraft.Find(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	return S ? S->Mode : ECommandMode::LegacyOrManual;
}

void SetCandidateForTesting(const UJSBSimMovementComponent *Component, const FFormationCandidate &Candidate)
{
	check(IsInGameThread());
	if (!Component) return;
	FPerAircraft &S = Find(Component);
	S.Candidate = Candidate;
	S.bHaveCandidate = true;
}

// ================================================================================================
// PRIME / BUMPLESS HANDOFF (Phase B)
// ================================================================================================
const TCHAR *PrimeStateName(EPrimeState S)
{
	switch (S)
	{
	case EPrimeState::PrimePending:         return TEXT("PrimePending");
	case EPrimeState::PrimedCandidateReady: return TEXT("PrimedCandidateReady");
	case EPrimeState::ActivationPending:    return TEXT("ActivationPending");
	case EPrimeState::ActiveFormation:      return TEXT("ActiveFormation");
	default:                                return TEXT("IdleLegacy");
	}
}

const TCHAR *PrimeFailureName(EPrimeFailure F)
{
	switch (F)
	{
	case EPrimeFailure::NoResolvedSnapshot:  return TEXT("NoResolvedSnapshot");
	case EPrimeFailure::NonFiniteSnapshot:   return TEXT("NonFiniteSnapshot");
	case EPrimeFailure::InvalidComponent:    return TEXT("InvalidComponent");
	case EPrimeFailure::Falling:             return TEXT("Falling");
	case EPrimeFailure::NoTicket:            return TEXT("NoTicket");
	case EPrimeFailure::WrongGeneration:     return TEXT("WrongGeneration");
	case EPrimeFailure::WrongBaseline:       return TEXT("WrongBaseline");
	case EPrimeFailure::StaleCandidate:      return TEXT("StaleCandidate");
	case EPrimeFailure::InvalidCandidate:    return TEXT("InvalidCandidate");
	case EPrimeFailure::NonFiniteCandidate:  return TEXT("NonFiniteCandidate");
	case EPrimeFailure::OutOfRangeCandidate: return TEXT("OutOfRangeCandidate");
	case EPrimeFailure::NotPrimed:           return TEXT("NotPrimed");
	case EPrimeFailure::ResolverNotOwned:    return TEXT("ResolverNotOwned");
	case EPrimeFailure::InterveningConsume:  return TEXT("InterveningConsume");
	case EPrimeFailure::GenerationExhausted: return TEXT("GenerationExhausted");
	case EPrimeFailure::IdentityMismatch:    return TEXT("IdentityMismatch");
	case EPrimeFailure::UnknownHealth:       return TEXT("UnknownHealth");
	default:                                 return TEXT("None");
	}
}

bool GetLastResolvedSnapshot(const UJSBSimMovementComponent *Component, FResolvedCommandSnapshot &Out)
{
	if (!Component) return false;
	const FPerAircraft *S = GAircraft.Find(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	if (!S || !S->LastResolved.bValid) return false;
	Out = S->LastResolved;
	return true;
}

bool RequestPrime(const UJSBSimMovementComponent *Component, FPrimeTicket &OutTicket, EPrimeFailure &OutFailure)
{
	check(IsInGameThread());   // the registry and the consume path are game-thread only
	++GCounters.PrimeRequestCount;
	OutTicket = FPrimeTicket{};
	OutFailure = EPrimeFailure::None;

	auto Reject = [&](EPrimeFailure F) {
		OutFailure = F;
		++GCounters.PrimeRejectedCount;
		return false;
	};

	// If we do not own the consume boundary, we cannot promise anything about what the FDM will consume,
	// so we must not hand out a ticket that claims we can.
	if (!IsEnabled())                   return Reject(EPrimeFailure::ResolverNotOwned);
	if (!Component || !Component->GetWorld()) return Reject(EPrimeFailure::InvalidComponent);

	FPerAircraft &S = Find(Component);
	if (!S.LastResolved.bValid)         return Reject(EPrimeFailure::NoResolvedSnapshot);

	// The baseline must belong to THIS aircraft in THIS world. A consume sequence is not an identity --
	// two aircraft can sit on the same one -- so priming against a snapshot that came from somewhere else
	// would anchor the handoff to a command this aircraft never flew.
	if (S.LastResolved.Component.Get() != Component
		|| S.LastResolved.World.Get() != Component->GetWorld())
	{
		return Reject(EPrimeFailure::IdentityMismatch);
	}

	FJSBSimResolvedCommandBlock AsBlock;
	AsBlock.Commands = S.LastResolved.Commands;
	AsBlock.EngineCommands = S.LastResolved.EngineCommands;
	if (!IsBlockFinite(AsBlock))        return Reject(EPrimeFailure::NonFiniteSnapshot);

	// Safety outranks bumpless: a dead aircraft is not handed over, it is put down. And an aircraft whose
	// health we cannot establish is not assumed healthy -- priming it would be promising continuity for
	// something we cannot prove is even flyable.
	const EHealth H = GetHealth(Component);
	if (H == EHealth::NotAlive) return Reject(EPrimeFailure::Falling);
	if (H != EHealth::Alive)    return Reject(EPrimeFailure::UnknownHealth);

	// A NEW generation immediately invalidates the previous ticket and any candidate riding on it.
	// Monotonic per aircraft and NEVER reused: a wrapped generation would let a replayed candidate from
	// a superseded prime look current again, which is the exact bug the generation exists to prevent.
	// So exhaustion FAILS the request rather than wrapping. (uint64 will not reach this in any session
	// that could physically occur; it is refused rather than hoped about.)
	if (S.NextPrimeGeneration == TNumericLimits<uint64>::Max())
	{
		return Reject(EPrimeFailure::GenerationExhausted);
	}

	S.Ticket = FPrimeTicket{};
	S.Ticket.Generation = S.NextPrimeGeneration++;
	S.Ticket.BaselineConsumeSequence = S.LastResolved.ConsumeSequence;
	S.Ticket.TimestampS = NowS();
	S.Ticket.Baseline = S.LastResolved;
	S.Ticket.bValid = true;

	S.bHaveCandidate = false;               // any earlier candidate is dead the moment a prime is issued
	S.bCandidateHasFullBlock = false;
	S.PrimeState = EPrimeState::PrimePending;
	// Mode deliberately UNCHANGED: priming is not activating.

	OutTicket = S.Ticket;
	++GCounters.PrimeGrantedCount;
	LogEvent(FString::Printf(
		TEXT("[PRIME] GRANTED actor=%s generation=%llu baseline_seq=%llu ail=%.6f elv=%.6f rud=%.6f thr=%.6f spb=%.6f"),
		*S.ActorName, S.Ticket.Generation, S.Ticket.BaselineConsumeSequence,
		S.Ticket.Baseline.Commands.Aileron, S.Ticket.Baseline.Commands.Elevator,
		S.Ticket.Baseline.Commands.Rudder,
		S.Ticket.Baseline.EngineCommands.Num() > 0 ? S.Ticket.Baseline.EngineCommands[0].Throttle : 0.0,
		S.Ticket.Baseline.Commands.SpeedBrake));
	return true;
}

bool SubmitPrimedCandidate(const UJSBSimMovementComponent *Component, const FPrimedCandidate &In,
                           EPrimeFailure &OutFailure)
{
	check(IsInGameThread());
	OutFailure = EPrimeFailure::None;

	auto Reject = [&](EPrimeFailure F) {
		OutFailure = F;
		++GCounters.PrimedCandidateRejectedCount;
		if (F == EPrimeFailure::WrongGeneration) ++GCounters.WrongGenerationRejectedCount;
		if (F == EPrimeFailure::StaleCandidate)  ++GCounters.StaleTicketRejectedCount;
		return false;
	};

	if (!IsEnabled())                   return Reject(EPrimeFailure::ResolverNotOwned);
	if (!Component || !Component->GetWorld()) return Reject(EPrimeFailure::InvalidComponent);

	FPerAircraft &S = Find(Component);
	const EHealth H = GetHealth(Component);
	if (H == EHealth::NotAlive) return Reject(EPrimeFailure::Falling);
	if (H != EHealth::Alive)    return Reject(EPrimeFailure::UnknownHealth);

	// It must descend from the CURRENT prime. Without these two checks a candidate from a superseded
	// prime -- anchored to a command the aircraft has long stopped flying -- would look identical to a
	// fresh one, and the handoff would step.
	if (!S.Ticket.bValid || S.PrimeState == EPrimeState::IdleLegacy) return Reject(EPrimeFailure::NoTicket);
	if (In.PrimeGeneration != S.Ticket.Generation)                   return Reject(EPrimeFailure::WrongGeneration);
	if (In.BaselineConsumeSequence != S.Ticket.BaselineConsumeSequence) return Reject(EPrimeFailure::WrongBaseline);

	const FFormationCandidate &K = In.Candidate;
	if (!K.bValid || !K.bCommandReady)  return Reject(EPrimeFailure::InvalidCandidate);
	if (!K.bFinite || !IsCandidateFinite(K)) return Reject(EPrimeFailure::NonFiniteCandidate);

	if (!IsCandidateFresh(K)) return Reject(EPrimeFailure::StaleCandidate);

	// Observed range, same limits the resolver observes. A candidate outside them is refused rather than
	// clamped -- clamping would silently change what the aircraft flies.
	auto BadSigned = [](double V) { return V < -1.0 || V > 1.0; };
	if (BadSigned(K.AileronNorm) || BadSigned(K.ElevatorNorm) || BadSigned(K.RudderNorm)
		|| K.ThrottleNorm < 0.0 || K.ThrottleNorm > 1.0
		|| K.SpeedBrakeNorm < 0.0 || K.SpeedBrakeNorm > 1.0)
	{
		return Reject(EPrimeFailure::OutOfRangeCandidate);
	}

	// PHASE C: if the candidate carries a full 29-field block, it is held to the SAME finite/range bar the
	// resolver observes on every consume -- a full block is handed wholesale to the FCS, so an un-checked
	// field would reach the aircraft. It is validated here, before acceptance, and refused (not clamped)
	// on any violation.
	if (In.bHasFullBlock)
	{
		FJSBSimResolvedCommandBlock FullBlock;
		FullBlock.Commands = In.FullCommands;
		FullBlock.EngineCommands = In.FullEngineCommands;
		if (!IsBlockFinite(FullBlock))    return Reject(EPrimeFailure::NonFiniteCandidate);
		if (IsBlockOutOfRange(FullBlock)) return Reject(EPrimeFailure::OutOfRangeCandidate);
	}

	S.Candidate = K;
	S.bHaveCandidate = true;
	// Set the full-block state on EVERY accept (including to false) so a newer 5-field candidate can never
	// inherit a stale full block from a previous submission.
	S.bCandidateHasFullBlock = In.bHasFullBlock;
	if (In.bHasFullBlock)
	{
		S.CandidateFullCommands = In.FullCommands;
		S.CandidateFullEngineCommands = In.FullEngineCommands;
	}
	else
	{
		S.CandidateFullCommands = FFlightControlCommands{};
		S.CandidateFullEngineCommands.Reset();
	}
	// PHASE C: an aircraft ALREADY in ActiveFormation stays there when the producer submits its next
	// candidate -- an ongoing update is not a fresh handoff and must not drop back to PrimedCandidateReady
	// (which would make the state machine claim the handover had not happened yet). The Phase B first
	// submit, from PrimePending, still advances to PrimedCandidateReady exactly as before.
	if (S.PrimeState != EPrimeState::ActiveFormation)
	{
		S.PrimeState = EPrimeState::PrimedCandidateReady;
	}
	++GCounters.PrimedCandidateAcceptedCount;
	LogEvent(FString::Printf(TEXT("[PRIME] CANDIDATE_ACCEPTED actor=%s generation=%llu baseline_seq=%llu full_block=%d"),
		*S.ActorName, In.PrimeGeneration, In.BaselineConsumeSequence, In.bHasFullBlock ? 1 : 0));
	return true;
}

// Shared by the test door (ActivateFormationForTesting) and the production door
// (RequestFormationActivation). Both only ASK: the mode is NOT changed here. The switch happens in
// Resolve(), at the consume boundary, because only there can we still check whether the baseline is
// current -- an intervening Legacy consume can only be seen from there. Flipping the mode here would hand
// over to a candidate anchored to a command the aircraft has already stopped flying, and it would step.
bool RequestActivationInternal(const UJSBSimMovementComponent *Component, EPrimeFailure &OutFailure)
{
	check(IsInGameThread());
	OutFailure = EPrimeFailure::None;

	auto Reject = [&](EPrimeFailure F) {
		OutFailure = F;
		++GCounters.ActivationRejectedCount;
		return false;
	};

	if (!IsEnabled())                   return Reject(EPrimeFailure::ResolverNotOwned);
	if (!Component || !Component->GetWorld()) return Reject(EPrimeFailure::InvalidComponent);

	FPerAircraft &S = Find(Component);
	const EHealth H = GetHealth(Component);
	if (H == EHealth::NotAlive) return Reject(EPrimeFailure::Falling);
	if (H != EHealth::Alive)    return Reject(EPrimeFailure::UnknownHealth);

	// The whole point: no prime, no activation. A perfectly valid, fresh candidate is NOT enough -- if it
	// did not come from a prime, nothing anchors it to what the aircraft is currently flying.
	if (S.PrimeState != EPrimeState::PrimedCandidateReady) return Reject(EPrimeFailure::NotPrimed);

	S.PrimeState = EPrimeState::ActivationPending;
	LogEvent(FString::Printf(
		TEXT("[PRIME] ACTIVATION_REQUESTED actor=%s generation=%llu baseline_seq=%llu (switch happens at the next consume)"),
		*S.ActorName, S.Ticket.Generation, S.Ticket.BaselineConsumeSequence));
	return true;
}

bool ActivateFormationForTesting(const UJSBSimMovementComponent *Component, EPrimeFailure &OutFailure)
{
	return RequestActivationInternal(Component, OutFailure);
}

// PHASE C production door. Identical policy to the test door -- it only asks; the boundary re-checks and
// switches. Kept as a separate, production-named verb so the handoff path is not spelled "...ForTesting".
bool RequestFormationActivation(const UJSBSimMovementComponent *Component, EPrimeFailure &OutFailure)
{
	return RequestActivationInternal(Component, OutFailure);
}

// PHASE C production door: the immediate Formation -> Legacy safety fallback. No blend, no delay -- the
// mode drops to LegacyOrManual now, so the very next consume resolves the legacy block, and all prime
// state is discarded so nothing can silently resume Formation on a stale ticket.
void RequestLegacyFallback(const UJSBSimMovementComponent *Component)
{
	check(IsInGameThread());
	if (!Component) return;
	FPerAircraft *S = GAircraft.Find(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	if (!S) return;

	const bool bWasFormation = S->Mode == ECommandMode::FormationControlV2
		|| S->PrimeState != EPrimeState::IdleLegacy;
	S->Mode = ECommandMode::LegacyOrManual;
	S->Counters.Mode = S->Mode;
	S->PrimeState = EPrimeState::IdleLegacy;
	S->Ticket = FPrimeTicket{};
	S->bHaveCandidate = false;
	S->bCandidateHasFullBlock = false;
	S->bAwaitingHandoffMeasure = false;
	if (bWasFormation)
	{
		++GCounters.ModeTransitionCount;
		LogEvent(FString::Printf(TEXT("[PRIME] LEGACY_FALLBACK actor=%s (immediate, no blend)"), *S->ActorName));
	}
}

EPrimeFailure GetLastBoundaryFailure(const UJSBSimMovementComponent *Component)
{
	if (!Component) return EPrimeFailure::None;
	const FPerAircraft *S = GAircraft.Find(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	return S ? S->LastBoundaryFailure : EPrimeFailure::None;
}

EPrimeState GetPrimeState(const UJSBSimMovementComponent *Component)
{
	if (!Component) return EPrimeState::IdleLegacy;
	const FPerAircraft *S = GAircraft.Find(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	return S ? S->PrimeState : EPrimeState::IdleLegacy;
}

uint64 GetPrimeGeneration(const UJSBSimMovementComponent *Component)
{
	if (!Component) return 0;
	const FPerAircraft *S = GAircraft.Find(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	return S ? S->Ticket.Generation : 0;
}

bool HasCandidate(const UJSBSimMovementComponent *Component)
{
	if (!Component) return false;
	const FPerAircraft *S = GAircraft.Find(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	return S && S->bHaveCandidate;
}

#if WITH_DEV_AUTOMATION_TESTS
void SetClockOverrideForTesting(double NowSeconds) { check(IsInGameThread()); GClockOverrideS = NowSeconds; }
void ClearClockOverrideForTesting()                { check(IsInGameThread()); GClockOverrideS = -1.0; }

void InjectResolvedSnapshotForTesting(const UJSBSimMovementComponent *Component,
                                      const FResolvedCommandSnapshot &Snapshot)
{
	check(IsInGameThread());
	if (!Component) return;
	FPerAircraft &S = Find(Component);
	S.LastResolved = Snapshot;
	// The caller does NOT get to declare whose command this is. Identity is forced from the target, so an
	// injection can supply values but never forge an owner.
	S.LastResolved.Component = Component;
	S.LastResolved.World = Component->GetWorld();
	S.LastResolved.bValid = true;
}
#endif

bool GetTicketBaseline(const UJSBSimMovementComponent *Component, FResolvedCommandSnapshot &Out)
{
	if (!Component) return false;
	const FPerAircraft *S = GAircraft.Find(TWeakObjectPtr<const UJSBSimMovementComponent>(Component));
	if (!S || !S->Ticket.bValid || !S->Ticket.Baseline.bValid) return false;
	Out = S->Ticket.Baseline;
	return true;
}

bool GetHandoffDelta(const FString &ActorName, FHandoffDelta &Out)
{
	for (const TPair<TWeakObjectPtr<const UJSBSimMovementComponent>, FPerAircraft> &P : GAircraft)
	{
		if (P.Value.ActorName == ActorName && P.Value.Handoff.bMeasured)
		{
			Out = P.Value.Handoff;
			return true;
		}
	}
	return false;
}

const FCounters &GetCounters() { return GCounters; }

bool GetAircraftCounters(const FString &ActorName, FAircraftCounters &Out)
{
	for (const TPair<TWeakObjectPtr<const UJSBSimMovementComponent>, FPerAircraft> &P : GAircraft)
	{
		if (P.Value.ActorName == ActorName)
		{
			Out = P.Value.Counters;
			return true;
		}
	}
	return false;
}

TArray<FString> BuildReport()
{
	TArray<FString> Out;
	const FCounters &C = GCounters;
	Out.Add(FString::Printf(TEXT("[ARBITER] SCENARIO=%s"), *GScenario));
	Out.Add(FString::Printf(
		TEXT("[ARBITER] TOTALS consume_count=%lld resolver_call_count=%lld legacy=%lld formation=%lld "
		     "falling=%lld duplicate_resolution=%lld missing_resolution=%lld"),
		C.ConsumeCount, C.ResolverCallCount, C.LegacyResolutionCount, C.FormationResolutionCount,
		C.FallingResolutionCount, C.DuplicateResolutionCount, C.MissingResolutionCount));
	Out.Add(FString::Printf(
		TEXT("[ARBITER] TRANSPARENCY legacy_changed_field_count=%lld legacy_block_mutation_count=%lld "
		     "resolved_non_finite=%lld resolved_range_violation=%lld"),
		C.LegacyChangedFieldCount, C.LegacyBlockMutationCount, C.ResolvedNonFiniteCount,
		C.ResolvedRangeViolationCount));
	Out.Add(FString::Printf(
		TEXT("[ARBITER] FALLBACKS stale=%lld invalid=%lld non_finite=%lld no_candidate=%lld "
		     "unknown_health=%lld mode_transitions=%lld"),
		C.StaleFallbackCount, C.InvalidFallbackCount, C.NonFiniteFallbackCount,
		C.NoCandidateFallbackCount, C.UnknownHealthFallbackCount, C.ModeTransitionCount));

	for (const TPair<TWeakObjectPtr<const UJSBSimMovementComponent>, FPerAircraft> &P : GAircraft)
	{
		const FAircraftCounters &A = P.Value.Counters;
		Out.Add(FString::Printf(
			TEXT("[ARBITER] AIRCRAFT actor=%s mode=%s consumes=%lld legacy=%lld formation=%lld falling=%lld "
			     "changed_fields=%lld stale=%lld invalid=%lld non_finite=%lld last_source=%s cand_gen=%u"),
			*P.Value.ActorName,
			A.Mode == ECommandMode::FormationControlV2 ? TEXT("FormationControlV2") : TEXT("LegacyOrManual"),
			A.Consumes, A.LegacyResolutions, A.FormationResolutions, A.FallingResolutions,
			A.LegacyChangedFields, A.StaleFallbacks, A.InvalidFallbacks, A.NonFiniteFallbacks,
			SourceName(A.LastSource), A.CandidateGeneration));
	}

	Out.Add(FString::Printf(
		TEXT("[PRIME] TOTALS requests=%lld granted=%lld rejected=%lld candidates_accepted=%lld "
		     "candidates_rejected=%lld activations_granted=%lld activations_rejected=%lld"),
		C.PrimeRequestCount, C.PrimeGrantedCount, C.PrimeRejectedCount,
		C.PrimedCandidateAcceptedCount, C.PrimedCandidateRejectedCount,
		C.ActivationGrantedCount, C.ActivationRejectedCount));
	Out.Add(FString::Printf(
		TEXT("[PRIME] HANDOFF handoffs_measured=%lld non_zero_handoff_count=%lld "
		     "cancelled_by_falling=%lld stale_ticket_rejected=%lld wrong_generation_rejected=%lld "
		     "intervening_consume_rejected=%lld activation_stale_rejected=%lld"),
		C.HandoffsMeasured, C.NonZeroHandoffCount, C.PrimeCancelledByFallingCount,
		C.StaleTicketRejectedCount, C.WrongGenerationRejectedCount,
		C.InterveningConsumeRejectedCount, C.ActivationStaleRejectedCount));

	for (const TPair<TWeakObjectPtr<const UJSBSimMovementComponent>, FPerAircraft> &P : GAircraft)
	{
		const FPerAircraft &S = P.Value;
		Out.Add(FString::Printf(
			TEXT("[PRIME] AIRCRAFT actor=%s prime_state=%s generation=%llu next_generation=%llu "
			     "baseline_seq=%llu have_snapshot=%d handoff_measured=%d "
			     "dAil=%.9f dElv=%.9f dRud=%.9f dThr=%.9f dSpb=%.9f"),
			*S.ActorName, PrimeStateName(S.PrimeState), S.Ticket.Generation, S.NextPrimeGeneration,
			S.Ticket.BaselineConsumeSequence, S.LastResolved.bValid ? 1 : 0, S.Handoff.bMeasured ? 1 : 0,
			S.Handoff.Aileron, S.Handoff.Elevator, S.Handoff.Rudder, S.Handoff.Throttle, S.Handoff.SpeedBrake));
	}

	Out.Add(FString::Printf(TEXT("[ARBITER] EVENT_LOG lines=%d"), GEventLog.Num()));
	Out.Append(GEventLog);
	return Out;
}

} // namespace MumtCommandArbiterV2
