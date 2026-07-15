#pragma once

// FormationRuntimeOwnerV2 — Phase D.
//
// The per-aircraft production runtime that drives the Phase C candidate producer from an OPERATIONAL
// command. Phase C proved the producer works when a test calls BeginHandoff(); Phase D connects the real
// UDP/BT command path to it, on the game thread, with the correct tick ordering, and NEVER auto-activates:
// Formation begins only when an explicit external "control_mode: formation" request arrives.
//
// WHY A PER-AIRCRAFT UActorComponent (and not a world manager or a map in the receiver):
//   * Tick ordering is a hard requirement -- the producer must read the JSBSim snapshot AFTER the movement
//     component has refreshed it (CopyFromJSBSim) and submit a candidate BEFORE the next consume
//     (CopyToJSBSim). A component can express that exactly with AddTickPrerequisiteComponent(movement),
//     which needs no change to the JSBSim plugin and does not depend on accidental registration order. A
//     60 Hz world timer (the receiver's AutopilotTick) cannot guarantee it.
//   * Lifetime/cleanup is automatic: the component is owned by the follower actor, so world teardown, actor
//     destruction, aircraft removal and PIE end all destroy it and its producer with no bookkeeping. On
//     disable / Falling / leader loss it stays but goes idle after an immediate Legacy fallback.
//   * The receiver stays thin: it parses and routes an operational request; it does NOT hold the
//     planner/NPFG/TECS/stick state. All of that lives in the producer inside this component.
//
// It changes NOTHING about the arbiter's guarantees -- it is a caller of the Phase B/C contract. The
// default is Legacy: with no external Formation request the aircraft is bit-for-bit transparent legacy.

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FormationControlV2/FormationCandidateProducerV2.h"
#include "State/MumtCommandArbiterV2.h"
#include "FormationRuntimeOwnerV2.generated.h"

class UJSBSimMovementComponent;

UENUM()
enum class EFormationRuntimePhaseV2 : uint8
{
	Idle,                 // Legacy: nothing requested, or fell back
	Warming,              // running the chain in shadow so the first real candidate is valid
	Priming,              // about to prime / just primed
	AwaitingActivation,   // prime submitted + activation requested; waiting for the consume boundary
	Active                // ActiveFormation: producing fresh candidates each advancing frame
};

UENUM()
enum class EFormationRuntimeFallbackV2 : uint8
{
	None,
	ExplicitDisable,      // an external Legacy request
	FollowerLost,         // the follower movement component/world went invalid
	LeaderLost,           // the leader movement component went invalid
	WorldMismatch,        // follower and leader are in different worlds
	IdentityChanged,      // the follower component identity changed under us
	NotAlive,             // Falling / Crashed
	UnknownHealth,        // no health component -> cannot prove flyable
	SimTimeDiscontinuity, // reset / hold / non-monotonic sim time
	StaleCommand,         // the operational command timestamp is too old
	SequenceReplay,       // a non-advancing / out-of-order request sequence
	NonFiniteSlot,        // slot offset not finite / out of range
	NoLeaderSpecified,    // Formation requested with no leader
	ProducerRejected,     // BeginHandoff / activation refused by the arbiter
	ProducerInvalid       // the producer produced a non-finite / not-ready result
};

const TCHAR *FormationRuntimePhaseName(EFormationRuntimePhaseV2 P);
const TCHAR *FormationRuntimeFallbackName(EFormationRuntimeFallbackV2 R);

UCLASS(ClassGroup = (MUMT), meta = (BlueprintSpawnableComponent))
class MUMT_SIM_API UFormationRuntimeOwnerV2 : public UActorComponent
{
	GENERATED_BODY()

public:
	UFormationRuntimeOwnerV2();

	// Called by the receiver on the game thread with a parsed operational request for THIS aircraft.
	// Idempotent: a repeated same-sequence enable does not re-prime. A slot-only change updates the active
	// producer without a re-prime. A leader change falls back and re-handshakes. Returns false (and sets
	// OutReason) if the request was rejected (stale, replayed, non-finite, no leader) -- the aircraft stays
	// on whatever it was doing (Legacy for a first request).
	//
	// bFormation == false is an explicit Legacy/disable request: immediate fallback, producer reset.
	bool ApplyOperationalRequest(bool bFormation,
	                             UJSBSimMovementComponent *Leader, const FString &LeaderLabel,
	                             double FrontM, double RightM, double UpM,
	                             int64 Sequence, double CommandTimestampS,
	                             EFormationRuntimeFallbackV2 &OutReason);

	// Observation, for tests/telemetry. All game-thread.
	EFormationRuntimePhaseV2 GetPhase() const { return Phase; }
	bool IsFormationRequested() const { return bFormationRequested; }
	MumtCommandArbiterV2::ECommandMode GetActiveMode() const;
	uint64 GetPrimeGeneration() const { return PrimeGeneration; }
	uint64 GetBaselineConsumeSequence() const { return BaselineConsumeSequence; }
	uint32 GetCandidateGeneration() const { return CandidateGeneration; }
	int64 GetAppliedSequence() const { return AppliedSequence; }
	EFormationRuntimeFallbackV2 GetLastFallback() const { return LastFallback; }
	FormationControlV2::EProducerResult GetLastProducerResult() const { return LastProducerResult; }
	FString GetLeaderLabel() const { return LeaderLabel; }
	int32 GetActiveUpdateCount() const { return ActiveUpdateCount; }

protected:
	virtual void OnRegister() override;
	virtual void OnUnregister() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction *ThisTickFunction) override;

private:
	UJSBSimMovementComponent *ResolveFollower();      // from GetOwner(); binds the tick prerequisite once
	void FallBack(EFormationRuntimeFallbackV2 Reason); // immediate Legacy fallback + producer reset
	bool SafetyOk(UJSBSimMovementComponent *Follower, UJSBSimMovementComponent *Leader,
	              EFormationRuntimeFallbackV2 &OutReason) const;

	TUniquePtr<FormationControlV2::FFormationCandidateProducerV2> Producer;

	TWeakObjectPtr<UJSBSimMovementComponent> FollowerComp;
	TWeakObjectPtr<const UWorld> FollowerWorld;
	TWeakObjectPtr<UJSBSimMovementComponent> LeaderComp;
	bool bTickPrerequisiteBound = false;

	// requested / applied external command state
	bool bFormationRequested = false;
	FString LeaderLabel;
	double SlotFrontM = -200.0, SlotRightM = 100.0, SlotUpM = 0.0;
	int64 AppliedSequence = -1;
	double CommandTimestampS = -1.0;

	// runtime
	EFormationRuntimePhaseV2 Phase = EFormationRuntimePhaseV2::Idle;
	double LastFollowerSimTimeS = -1.0;
	int32 WarmTicks = 0;
	int32 ActiveUpdateCount = 0;
	uint64 PrimeGeneration = 0;
	uint64 BaselineConsumeSequence = 0;
	uint32 CandidateGeneration = 0;
	EFormationRuntimeFallbackV2 LastFallback = EFormationRuntimeFallbackV2::None;
	FormationControlV2::EProducerResult LastProducerResult = FormationControlV2::EProducerResult::Ok;
};
