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

class UJSBSimMovementComponent;
struct FFlightControlCommands;
struct FEngineCommand;
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
void SetModeForTesting(const UJSBSimMovementComponent *Component, ECommandMode Mode);
ECommandMode GetMode(const UJSBSimMovementComponent *Component);
void SetCandidateForTesting(const UJSBSimMovementComponent *Component, const FFormationCandidate &Candidate);

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
