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

struct FPerAircraft
{
	FString ActorName;
	ECommandMode Mode = ECommandMode::LegacyOrManual;
	FFormationCandidate Candidate;
	bool bHaveCandidate = false;

	// the decision taken in Resolve, consumed by OnResolved
	uint64 LastResolveSeq = 0;
	uint64 LastObservedSeq = 0;
	EResolutionSource LastSource = EResolutionSource::Legacy;
	EFallbackReason LastFallback = EFallbackReason::None;
	FFlightControlCommands LegacyAtResolve;
	TArray<FEngineCommand> LegacyEngineAtResolve;
	bool bResolvePending = false;

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
		return;
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
	const double Age = FApp::GetCurrentTime() - K.TimestampS;
	if (K.TimestampS < 0.0 || Age > kCandidateMaxAgeS || Age < 0.0)
	{
		S.LastSource = EResolutionSource::Legacy;
		S.LastFallback = EFallbackReason::Stale;
		return;
	}

	// 5. Accepted. ONLY the axes the Formation chain owns are overwritten; everything else -- trims,
	//    flaps, brakes, gear, the Blueprints' fields -- keeps its legacy value. Formation does not get
	//    to silently zero a field just because it has no opinion about it.
	Resolved.Commands.Aileron = K.AileronNorm;
	Resolved.Commands.Elevator = K.ElevatorNorm;
	Resolved.Commands.Rudder = K.RudderNorm;
	Resolved.Commands.SpeedBrake = K.SpeedBrakeNorm;
	if (Resolved.EngineCommands.Num() > 0)
	{
		Resolved.EngineCommands[0].Throttle = K.ThrottleNorm;
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
		// S.Mode, S.Candidate, S.bHaveCandidate, S.ActorName deliberately preserved.
	}
}

void SetModeForTesting(const UJSBSimMovementComponent *Component, ECommandMode Mode)
{
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
	if (!Component) return;
	FPerAircraft &S = Find(Component);
	S.Candidate = Candidate;
	S.bHaveCandidate = true;
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

	Out.Add(FString::Printf(TEXT("[ARBITER] EVENT_LOG lines=%d"), GEventLog.Num()));
	Out.Append(GEventLog);
	return Out;
}

} // namespace MumtCommandArbiterV2
