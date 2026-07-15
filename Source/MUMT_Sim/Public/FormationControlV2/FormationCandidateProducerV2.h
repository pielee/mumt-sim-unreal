#pragma once

// FormationCandidateProducerV2 — Phase C.
//
// This is the piece that was deliberately absent in Phase A and B: the thing that runs the REAL Formation
// control chain (FormationPlannerV2 -> NPFG -> TECS -> F16StickAdapterV2) and feeds its output through the
// Phase B Prime/handoff contract into the command arbiter.
//
// It changes NOTHING about what the arbiter guarantees. It is a CALLER of the existing contract:
//   * the first Formation consume still steps by exactly zero -- guaranteed by the stick's baseline latch,
//     which this producer arms and then reads on the first frame;
//   * the baseline is still the immutable resolved snapshot the FDM last consumed;
//   * generation / consume-sequence binding, intervening-consume rejection, staleness, Falling
//     cancellation, UnknownHealth and per-aircraft/world identity are all still the arbiter's, unchanged.
//
// It is production code, on the game thread, but it does NOT auto-activate anything: Formation begins only
// when BeginHandoff() is called explicitly. There is no UDP/BT/Blueprint path to it in this phase.
//
// Candidate shape (Phase C contract): every candidate is a FULL 29-field block. The producer copies the
// immutable prime baseline and overwrites only the fields the chain owns -- aileron, elevator, rudder and
// throttle from the stick. Speed-brake is NOT produced by the chain and stays at the baseline; every other
// field (trims, flaps, brakes, gear, engine state) also stays at the baseline. Nothing is left
// uninitialised.

#include "CoreMinimal.h"
#include "State/MumtCommandArbiterV2.h"
#include "State/MumtControlState.h"
#include "FormationControlV2/FormationGuidanceCoordinatorV2.h"
#include "FormationControlV2/F16StickAdapterV2.h"
#include "FormationControlV2/FormationPlannerV2.h"
#include "FormationControlV2/CanonicalNavigationAdapterV2.h"
#include "FormationControlV2/FormationSlotGeneratorV2.h"
#include "FormationControlV2/MissionNavigationFrameV2.h"

class UJSBSimMovementComponent;
class UWorld;

namespace FormationControlV2
{

// Why a real candidate could not be produced or a handoff could not begin. Distinct from the arbiter's
// EPrimeFailure: this covers the PRODUCER's own preconditions (no snapshot, chain not ready) before the
// arbiter is even asked.
enum class EProducerResult : uint8
{
	Ok = 0,
	NoFollowerComponent,
	NoLeaderComponent,
	NoFollowerSnapshot,      // GetJsbFlightSnapshot failed / invalid frame
	NoLeaderSnapshot,
	FrameNotEstablished,     // the mission NED origin could not be set
	ChainNotReady,           // the guidance/stick chain did not produce a command this frame
	NonFiniteChainOutput,
	PrimeRejected,           // the arbiter refused RequestPrime (see LastPrimeFailure)
	SnapshotUnavailable,     // no last-resolved snapshot to be continuous with
	CandidateRejected,       // the arbiter refused SubmitPrimedCandidate
	ActivationRejected,      // the arbiter refused RequestFormationActivation
	NotActive,               // an ongoing update was asked for but the aircraft is not in ActiveFormation
	Count
};

const TCHAR *ProducerResultName(EProducerResult R);

// The slot the follower is trying to hold, relative to the leader. Body-frame offsets, same convention as
// the shadow test's FFormationSlotCommandV2 (front/right/up, metres).
struct FProducerSlotSpec
{
	double FrontM = -200.0;
	double RightM = 100.0;
	double UpM = 0.0;
};

// Everything the producer computed on the last frame, for observation/telemetry. The chain output is
// separated from the applied candidate so a test can tell the exact-zero latch frame (chain == baseline
// by construction) from a real-output frame.
struct FProducerFrameResult
{
	EProducerResult Result = EProducerResult::Ok;
	MumtCommandArbiterV2::EPrimeFailure LastPrimeFailure = MumtCommandArbiterV2::EPrimeFailure::None;

	// The raw chain command (stick output) this frame.
	double ChainAileron = 0.0, ChainElevator = 0.0, ChainRudder = 0.0, ChainThrottle = 0.0;
	bool bChainCommandReady = false;
	bool bLatchFrame = false;        // this frame the stick emitted the primed baseline (first compute)

	// The candidate actually submitted (full block's five controlled fields, for logging).
	double CandAileron = 0.0, CandElevator = 0.0, CandRudder = 0.0, CandThrottle = 0.0, CandSpeedBrake = 0.0;
	bool bSubmitted = false;

	uint64 Generation = 0;
	uint64 BaselineConsumeSequence = 0;
	uint32 CandidateGeneration = 0;   // monotonic per aircraft, so a test can see candidates advancing
};

// One per follower aircraft. Holds the stateful chain (planner held-path/progress, TECS integrators, stick
// integrator + slew anchors + prime latch) plus the mission frame and per-field state trackers. Game
// thread only. Not a UObject: it is owned by whatever drives it (today, the integration test; tomorrow, a
// narrow production trigger).
class MUMT_SIM_API FFormationCandidateProducerV2
{
public:
	FFormationCandidateProducerV2();

	void SetSlot(const FProducerSlotSpec &Slot) { SlotSpec = Slot; }
	void SetStickConfig(const FF16StickConfigV2 &Cfg) { StickConfig = Cfg; }
	void SetGuidanceConfig(const FGuidanceConfigV2 &Cfg) { GuidanceConfig = Cfg; }

	// Reset the chain (new episode). Clears planner/guidance/stick state and the mission frame.
	void Reset();

	// Run the chain in SHADOW for one frame WITHOUT touching the arbiter: no prime, no candidate, no
	// submit. This is how the producer is meant to reach a handoff -- the canonical nav adapter needs
	// consecutive samples before it can report a ground course, and the coordinator skips its very first
	// frame to build its controllers, so a chain asked to produce a real command cold would return
	// nothing for a few frames. Running it in shadow first (exactly what the airborne shadow test does)
	// means that when the handoff happens, the very next frame already produces a valid real command --
	// so the baseline candidate does not go stale waiting for the chain to warm up.
	void WarmChain(const UJSBSimMovementComponent *Follower, const UJSBSimMovementComponent *Leader, double DtS);

	// Whether the last WarmChain/compute produced a valid chain command -- the caller can wait for this
	// before beginning the handoff, so the ongoing (post-latch) frames are immediately real.
	bool IsChainReady() const { return bChainReady; }

	// STEP 1-7 of the Phase C prime order, in one game-thread call so no Legacy consume can slip between
	// them (splitting them across ticks would let the baseline go stale under the ticket):
	//   1. read the last resolved snapshot the FDM consumed (the baseline)
	//   2. RequestPrime
	//   3. keep generation + baseline consume sequence + immutable baseline
	//   4. prime the stick from that baseline
	//   5. compute the FIRST real candidate for the SAME aircraft/world/state sample (the primed stick
	//      returns the baseline exactly on this first compute -- that is the zero-step guarantee)
	//   6. SubmitPrimedCandidate as a full 29-field block
	//   7. request Formation activation
	// The actual switch is confirmed later, at the next FDM consume boundary, by the arbiter.
	FProducerFrameResult BeginHandoff(const UJSBSimMovementComponent *Follower,
	                                  const UJSBSimMovementComponent *Leader, double DtS);

	// Steps 1-6 ONLY (prime, prime stick, compute first candidate, submit) -- it does NOT request
	// activation. Exposed so a caller (the integration test) can control the timing between preparation
	// and activation, which is exactly what proves the arbiter rejects an activation whose baseline went
	// stale in between. BeginHandoff() is PrepareHandoff() immediately followed by RequestActivation().
	FProducerFrameResult PrepareHandoff(const UJSBSimMovementComponent *Follower,
	                                    const UJSBSimMovementComponent *Leader, double DtS);

	// Step 7 ONLY. Asks the arbiter to activate; the switch is confirmed at the next consume boundary.
	bool RequestActivation(const UJSBSimMovementComponent *Follower,
	                       MumtCommandArbiterV2::EPrimeFailure &OutFailure);

	// Ongoing production once the aircraft is in ActiveFormation: compute a fresh real candidate and submit
	// it (same ticket, new timestamp). If the aircraft is not ActiveFormation this returns NotActive and
	// submits nothing -- an update is not a handoff. If the producer simply stops calling this, the last
	// candidate goes stale and the arbiter falls back to Legacy on its own (the Phase B safety contract).
	FProducerFrameResult Update(const UJSBSimMovementComponent *Follower,
	                            const UJSBSimMovementComponent *Leader, double DtS);

private:
	// Runs planner -> NPFG/TECS coordinator -> stick for the follower against the leader, mirroring the
	// airborne shadow chain. Fills Out with the stick command. primeGeneration/primeConsumeSequence are
	// threaded into the stick input so its one-shot baseline latch fires on the correct frame.
	bool ComputeChain(const UJSBSimMovementComponent *Follower, const UJSBSimMovementComponent *Leader,
	                  double DtS, uint64 PrimeGeneration, uint64 PrimeConsumeSequence,
	                  FProducerFrameResult &Out);

	// Build the full-block candidate = baseline copy with the four chain-owned fields overwritten.
	MumtCommandArbiterV2::FPrimedCandidate BuildCandidate(
		const MumtCommandArbiterV2::FPrimeTicket &Ticket, const FProducerFrameResult &Chain) const;

	FProducerSlotSpec SlotSpec{};
	FF16StickConfigV2 StickConfig{};
	FGuidanceConfigV2 GuidanceConfig{};

	// Per-aircraft chain state (follower). The leader only contributes a canonical nav state, so it needs
	// its own nav tracker but not a planner/guidance/stick.
	FCanonicalNavigationTrackerV2 FollowerNavTracker;
	FCanonicalNavigationTrackerV2 LeaderNavTracker;
	FormationPlannerV2 Planner;
	FormationGuidanceCoordinatorV2 Guidance;
	F16StickAdapterV2 Stick;
	MissionNavigationFrameV2 Frame;
	bool bFrameSet = false;

	MumtState::FMumtStateTracker FollowerStateTracker;

	uint32 CandidateGenerationCounter = 0;   // increments per submitted candidate, for observability
	uint32 ResetGeneration = 1;
	bool bChainReady = false;                // the last compute produced a valid chain command
};

} // namespace FormationControlV2
