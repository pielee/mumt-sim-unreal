// MumtCommandArbiterV2.h — the single point where the final JSBSim command block is decided.
//
// WHAT THE MEASUREMENTS FORCED (see docs/CONTROL_V2_COMMAND_OWNERSHIP.md):
//   * Four writers touch the command block -- the Falling hardover, the InnerLoop autopilot, the
//     manual/UDP path, and the aircraft BLUEPRINTS (five command-write nodes each, confirmed by graph
//     audit). None of them can simply be removed.
//   * They run in three different contexts with no TickGroup and no tick prerequisite, so the block
//     JSBSim consumes is a field-wise MIXTURE: e.g. aileron/elevator/throttle from the manual writer
//     while SpeedBrake still holds the autopilot's value, because the manual writer never touches it.
//   * Falling ownership is exclusive and correct, and must stay that way.
//
// So arbitration cannot live in any writer. It lives at the CONSUME BOUNDARY -- inside
// UJSBSimMovementComponent::CopyToJSBSim, after every writer (including Blueprint) has had its say and
// before the FCS setters run. That is the only place in the program where "what will actually fly the
// aircraft" is a well-posed question.
//
// PHASE A CONTRACT: the default mode is LegacyOrManual and it is BIT-FOR-BIT TRANSPARENT. The resolved
// block equals the legacy block in every consumed field -- including the mixture above, including the
// trims/flaps/brakes the Blueprints set. Production ControlV2 is NOT activated here. The Formation path
// exists, is exercised by tests through an explicit dev API, and can only be reached by calling that
// API -- never by config, Blueprint default, or a UDP packet.
#pragma once

#include "CoreMinimal.h"
// FResolvedCommandSnapshot stores the command blocks BY VALUE (all 29 consumed fields), so the full
// types are required here -- a forward declaration would only allow pointers.
#include "FDMTypes.h"

class UJSBSimMovementComponent;
class UWorld;
struct FJSBSimResolvedCommandBlock;

namespace MumtCommandArbiterV2
{

enum class ECommandMode : uint8
{
	LegacyOrManual = 0,   // DEFAULT. Resolved == legacy, always.
	FormationControlV2,   // reachable only through SetModeForTesting()
};

// Where the resolved block came from. Recorded per consume.
enum class EResolutionSource : uint8
{
	Legacy = 0,      // legacy block passed through
	FallingLegacy,   // legacy block passed through because the aircraft is not Alive (hardover)
	Formation,       // the Formation candidate was applied
	Count
};

// Why a Formation candidate was refused. Exactly one reason per refusal.
enum class EFallbackReason : uint8
{
	None = 0,
	NoCandidate,
	Stale,
	Invalid,       // !bValid or !bCommandReady
	NonFinite,
	Falling,
	UnknownHealth, // no UHealthComponent -> we cannot prove it is Alive, so we do not override
	Count
};

const TCHAR *SourceName(EResolutionSource S);
const TCHAR *FallbackName(EFallbackReason R);

// The Formation candidate. Deliberately NOT wired to NPFG/TECS/Stick in this phase: this is the shape
// of the input the real chain will eventually produce, so the arbiter can be proven before it is fed.
struct FFormationCandidate
{
	double AileronNorm = 0.0;
	double ElevatorNorm = 0.0;
	double RudderNorm = 0.0;
	double ThrottleNorm = 0.0;
	double SpeedBrakeNorm = 0.0;
	double TimestampS = -1.0;     // FApp::GetCurrentTime() when it was produced
	uint32 Generation = 0;
	bool bFinite = false;
	bool bValid = false;
	bool bCommandReady = false;
};

// A candidate older than this is stale and is refused. 0.1 s is six frames at 60 Hz: long enough to
// survive a hitch, short enough that a dead producer cannot keep flying the aircraft.
inline constexpr double kCandidateMaxAgeS = 0.1;

// ---- lifecycle -------------------------------------------------------------------------------
// Binds/unbinds the resolver on UJSBSimMovementComponent. While disabled the resolver is UNBOUND, so
// CopyToJSBSim feeds the FCS from an untouched copy of the legacy block and the build behaves exactly
// as it did before this file existed.
// PRODUCTION: bound for the module's whole lifetime (FMumtSimModule::StartupModule). Not a test seam.
// Idempotent in both directions; refuses to evict a resolver owned by anything else.
bool IsEnabled();
void SetEnabled(bool bEnable);

// Clears the MEASUREMENT (counters, log, per-consume bookkeeping). Each aircraft's mode and candidate
// SURVIVE -- a session reset must never be able to change what the arbiter would decide.
void ResetSession(const FString &ScenarioLabel);

// Lifecycle, so cleanup can be asserted rather than assumed. A TWeakObjectPtr going invalid is not
// cleanup: the entry, its mode and its candidate still sit in the registry until they are purged.
int32 GetRegistrySize();

// Ownership of the single-cast resolver is established by DELEGATE HANDLE IDENTITY, not by a boolean:
// another binder can replace our instance, and a flag would not notice. IsEnabled() is false whenever
// the delegate no longer holds the instance we bound -- there is no silent takeover and no silent
// recovery.
int64 GetResolverOwnershipConflictCount();   // we tried to bind, somebody else already owned it
int64 GetResolverOwnershipLostCount();       // it WAS ours and was taken over behind our back

// ---- test-only / dev seam --------------------------------------------------------------------
// The ONLY way to leave LegacyOrManual. Nothing in production calls these.
//
// SetModeForTesting is the RAW seam: it flips the mode with no prime, and exists so the resolver's own
// policy (falling priority, staleness, finiteness, per-aircraft isolation) can be exercised directly.
// It is NOT the handoff path -- see the Prime API below, which is.
void SetModeForTesting(const UJSBSimMovementComponent *Component, ECommandMode Mode);
ECommandMode GetMode(const UJSBSimMovementComponent *Component);
void SetCandidateForTesting(const UJSBSimMovementComponent *Component, const FFormationCandidate &Candidate);

// ================================================================================================
// PRIME / BUMPLESS HANDOFF (Phase B)
// ================================================================================================
// The command a handoff must be continuous with is NOT the mutable legacy writer block -- that is an
// input, and it can be a field-wise mixture of several writers that changes under you. It is the FINAL
// RESOLVED BLOCK the FDM actually consumed on the last step. That is the only thing the aircraft is
// genuinely flying, so that is the baseline everything here is anchored to.
//
//   Legacy writers -> legacy block -> resolver -> RESOLVED BLOCK -> [prime snapshot] -> prime the
//   stateful controllers -> first candidate -> validate -> only then activate Formation.

// The full consumed command, all 29 fields. Formation only overwrites five of them today, but the
// snapshot keeps everything: the Blueprints' trims/flaps/brakes and the engine state are part of what
// the aircraft is flying, and a snapshot that dropped them would become a second source of truth the
// moment anything else starts using it.
struct FResolvedCommandSnapshot
{
	FFlightControlCommands Commands;        // 15 consumed flight-control fields
	TArray<FEngineCommand> EngineCommands;  // 14 consumed fields per engine
	uint64 ConsumeSequence = 0;             // which FDM consume produced it
	double TimestampS = -1.0;
	bool bValid = false;

	// WHOSE command this is. Without it, "the last resolved snapshot" is just a bag of numbers: a
	// consume sequence proves nothing about identity (two aircraft can be on the same sequence), and a
	// prime taken against the wrong aircraft's baseline would hand over to a command that was never
	// flown. Set from the component the resolver was invoked with, never from a caller's claim.
	TWeakObjectPtr<const UJSBSimMovementComponent> Component;
	TWeakObjectPtr<const UWorld> World;
};
// NOTE: FuelFreeze is NOT part of this snapshot. It is a global on the movement component
// (Propulsion->SetFuelFreeze) and never travelled through the command block; including it here would
// invent a command path that does not exist.

// The handoff state machine, kept SEPARATE from ECommandMode. Mode answers "who decides the command";
// this answers "how far along is the handover". Fusing them would force every mode check to also reason
// about half-finished transitions.
enum class EPrimeState : uint8
{
	IdleLegacy = 0,        // no prime in flight
	PrimePending,          // a prime ticket exists; mode is STILL LegacyOrManual
	PrimedCandidateReady,  // a candidate matching that ticket has been accepted
	ActivationPending,     // activation ASKED FOR; the switch happens at the next consume boundary
	ActiveFormation,       // the switch actually happened
};
const TCHAR *PrimeStateName(EPrimeState S);

// Why a prime or a primed candidate was refused.
enum class EPrimeFailure : uint8
{
	None = 0,
	NoResolvedSnapshot,   // nothing has been consumed yet -- there is no baseline to be continuous with
	NonFiniteSnapshot,
	InvalidComponent,
	Falling,              // safety outranks bumpless, always
	NoTicket,             // a candidate arrived with no prime behind it
	WrongGeneration,      // a candidate from a superseded prime
	WrongBaseline,        // a candidate anchored to a different consume than the ticket
	StaleCandidate,
	InvalidCandidate,     // !bValid / !bCommandReady
	NonFiniteCandidate,
	OutOfRangeCandidate,
	NotPrimed,            // activation attempted without PrimedCandidateReady
	ResolverNotOwned,     // we do not own the consume boundary; we must not drive it
	InterveningConsume,   // the FDM consumed a Legacy block after the baseline was taken
	GenerationExhausted,
	IdentityMismatch,
	// Appended LAST so existing indices stay stable. A prime demands a PROVABLY alive aircraft: an
	// aircraft with no health component is not "falling", it is unproven -- and an unproven aircraft is
	// not one we may promise a bumpless handoff for.
	UnknownHealth,     // the snapshot does not belong to this component / this world
	Count
};
const TCHAR *PrimeFailureName(EPrimeFailure F);

// A candidate that claims to descend from a specific prime. The two extra fields are what make the
// handoff verifiable: without them, a stale candidate from a superseded prime is indistinguishable
// from a fresh one.
struct FPrimedCandidate
{
	FFormationCandidate Candidate;
	uint64 PrimeGeneration = 0;   // 0 is never issued: it means "no ticket"
	uint64 BaselineConsumeSequence = 0;

	// PHASE C: the FULL 29-field block the real producer wants applied. The producer copies the IMMUTABLE
	// prime baseline and overwrites only the fields it owns (aileron/elevator/rudder/throttle from the
	// stick; speed-brake is not produced and stays at baseline), so every non-producer field -- trims,
	// flaps, brakes, gear, all engine fields but throttle -- carries the BASELINE value, not whatever the
	// legacy writers happen to be doing on the handoff frame. That is what makes the whole block continuous
	// with what the aircraft was actually flying.
	//
	// When absent (bHasFullBlock == false, the Phase B path), the arbiter overlays ONLY the five controlled
	// axes onto the live legacy block, exactly as before -- so no Phase B behaviour changes.
	bool bHasFullBlock = false;
	FFlightControlCommands FullCommands;
	TArray<FEngineCommand> FullEngineCommands;
};

// Generations are uint64, strictly monotonic per aircraft, and NEVER reused. 0 is invalid by
// construction. On the (unreachable) event of exhaustion, RequestPrime FAILS rather than wrapping --
// a wrapped generation would let a replayed candidate from a superseded prime look current again,
// which is the exact class of bug the generation exists to prevent.
struct FPrimeTicket
{
	uint64 Generation = 0;
	uint64 BaselineConsumeSequence = 0;
	double TimestampS = -1.0;
	FResolvedCommandSnapshot Baseline;   // the 29-field command the aircraft was flying, IMMUTABLE
	bool bValid = false;
};

// The last command the FDM actually consumed for this aircraft. Empty until the first consume.
bool GetLastResolvedSnapshot(const UJSBSimMovementComponent *Component, FResolvedCommandSnapshot &Out);

// Takes a baseline. Fails if there is nothing consumed to be continuous with, if it is non-finite, if
// the aircraft is not Alive (safety outranks bumpless), or if we do not own the resolver. On success it
// issues a NEW generation, which immediately invalidates any earlier ticket and candidate, and leaves
// the mode at LegacyOrManual -- priming is not activating.
bool RequestPrime(const UJSBSimMovementComponent *Component, FPrimeTicket &OutTicket, EPrimeFailure &OutFailure);

// Accepts a candidate only if it descends from the CURRENT ticket and passes every check on its own
// merits. Any doubt leaves the aircraft on Legacy, with the reason counted.
bool SubmitPrimedCandidate(const UJSBSimMovementComponent *Component, const FPrimedCandidate &Candidate,
                           EPrimeFailure &OutFailure);

// TEST/DEV ONLY. Refuses unless the aircraft is in PrimedCandidateReady. Nothing in production calls it.
bool ActivateFormationForTesting(const UJSBSimMovementComponent *Component, EPrimeFailure &OutFailure);

// ---- PHASE C: production handoff verbs -------------------------------------------------------------
// The production twin of ActivateFormationForTesting. It ONLY asks: it sets ActivationPending and the
// actual switch still happens at the next FDM consume boundary in Resolve(), where the baseline-current
// and candidate-fresh re-checks live. It is an EXPLICIT request -- nothing here auto-activates Formation,
// and the default mode stays LegacyOrManual until something calls this. Refuses unless PrimedCandidateReady.
bool RequestFormationActivation(const UJSBSimMovementComponent *Component, EPrimeFailure &OutFailure);

// Formation -> Legacy is an IMMEDIATE safety fallback, never a bumpless handoff. This drops the aircraft
// back to LegacyOrManual with no blend: the very next consume resolves the legacy block, and any prime
// state, ticket and candidate are discarded. Holding or blending a command from a producer we have just
// decided to stop trusting would be exactly backwards.
void RequestLegacyFallback(const UJSBSimMovementComponent *Component);

// Why the LAST activation attempt was refused AT THE CONSUME BOUNDARY (not by ActivateFormationForTesting,
// which only asks). The two refusals there are different failures and must never be confused: the baseline
// moving on (InterveningConsume) is not the same as the candidate rotting in place (StaleCandidate).
EPrimeFailure GetLastBoundaryFailure(const UJSBSimMovementComponent *Component);

// TEST/DEV ONLY. Gives an aircraft a resolved snapshot it never actually consumed.
//
// It exists for ONE reason: an aircraft in a second, non-ticking world never reaches CopyToJSBSim, so it
// can never acquire a baseline -- and without a baseline it cannot be primed, which would make it
// impossible to prove that a world teardown preserves ANOTHER world's prime state. This is an observation
// scaffold, not a command path: it writes only the arbiter's own snapshot, never Commands/EngineCommands,
// and nothing in production calls it.
#if WITH_DEV_AUTOMATION_TESTS
// The caller's Component/World fields are IGNORED and overwritten from the target component: an
// injection must not be able to forge an identity.
// Freezes the arbiter's notion of "now" so candidate staleness can be exercised deterministically. Sleeping
// until a candidate rots would make the test hostage to the game scheduler; this makes the clock an input.
// PRODUCTION CLOCK BEHAVIOUR IS UNCHANGED: the override does not exist in a non-test build, and with no
// override set every call still reads FApp::GetCurrentTime().
void SetClockOverrideForTesting(double NowSeconds);
void ClearClockOverrideForTesting();

void InjectResolvedSnapshotForTesting(const UJSBSimMovementComponent *Component,
                                      const FResolvedCommandSnapshot &Snapshot);
#endif

EPrimeState GetPrimeState(const UJSBSimMovementComponent *Component);
uint64 GetPrimeGeneration(const UJSBSimMovementComponent *Component);
bool HasCandidate(const UJSBSimMovementComponent *Component);
// The IMMUTABLE baseline the ticket was cut from. The handoff delta is measured against THIS, never
// against "the previous consume" -- otherwise a delta of zero could just be a value compared with itself.
bool GetTicketBaseline(const UJSBSimMovementComponent *Component, FResolvedCommandSnapshot &Out);

// The command step across the first Formation consume, per controlled field, measured as
//     (what the FDM consumed on the first Formation step) - (the IMMUTABLE ticket baseline)
// NOT as "this consume minus the previous consume": that would compare the block with itself whenever
// nothing intervened, and a zero would prove nothing.
struct FHandoffDelta
{
	double Aileron = 0.0, Elevator = 0.0, Rudder = 0.0, Throttle = 0.0, SpeedBrake = 0.0;
	uint64 ConsumeSequence = 0;
	bool bMeasured = false;
};
bool GetHandoffDelta(const FString &ActorName, FHandoffDelta &Out);

// ---- counters --------------------------------------------------------------------------------
struct FCounters
{
	int64 ConsumeCount = 0;
	int64 ResolverCallCount = 0;
	int64 LegacyResolutionCount = 0;
	int64 FormationResolutionCount = 0;
	int64 FallingResolutionCount = 0;
	int64 StaleFallbackCount = 0;
	int64 InvalidFallbackCount = 0;
	int64 NonFiniteFallbackCount = 0;
	int64 NoCandidateFallbackCount = 0;
	int64 UnknownHealthFallbackCount = 0;
	int64 ModeTransitionCount = 0;
	int64 LegacyChangedFieldCount = 0;      // MUST be 0: fields that differ in LegacyOrManual mode
	int64 ResolvedNonFiniteCount = 0;
	int64 ResolvedRangeViolationCount = 0;
	int64 DuplicateResolutionCount = 0;     // a second resolve for one consume sequence
	int64 MissingResolutionCount = 0;       // a consume sequence that skipped a resolve
	int64 LegacyBlockMutationCount = 0;     // MUST be 0: the resolver wrote the legacy members
	uint32 ResolutionGeneration = 0;

	// ---- Phase B: prime / handoff ----
	int64 PrimeRequestCount = 0;
	int64 PrimeGrantedCount = 0;
	int64 PrimeRejectedCount = 0;
	int64 PrimedCandidateAcceptedCount = 0;
	int64 PrimedCandidateRejectedCount = 0;
	int64 ActivationGrantedCount = 0;
	int64 ActivationRejectedCount = 0;
	int64 PrimeCancelledByFallingCount = 0;   // safety pre-empted a handoff in flight
	int64 StaleTicketRejectedCount = 0;
	int64 WrongGenerationRejectedCount = 0;
	int64 HandoffsMeasured = 0;
	int64 NonZeroHandoffCount = 0;            // a step on the first Formation consume
	int64 InterveningConsumeRejectedCount = 0;// the baseline went stale before activation landed
	int64 ActivationStaleRejectedCount = 0;   // the CANDIDATE went stale before activation landed
};
const FCounters &GetCounters();

// Per-aircraft view, so isolation between aircraft can be asserted rather than assumed.
struct FAircraftCounters
{
	int64 Consumes = 0;
	int64 LegacyResolutions = 0;
	int64 FormationResolutions = 0;
	int64 FallingResolutions = 0;
	int64 LegacyChangedFields = 0;
	int64 StaleFallbacks = 0;
	int64 InvalidFallbacks = 0;
	int64 NonFiniteFallbacks = 0;
	uint32 CandidateGeneration = 0;
	ECommandMode Mode = ECommandMode::LegacyOrManual;
	EResolutionSource LastSource = EResolutionSource::Legacy;
};
bool GetAircraftCounters(const FString &ActorName, FAircraftCounters &Out);

TArray<FString> BuildReport();

} // namespace MumtCommandArbiterV2
