#include "FormationControlV2/FormationCandidateProducerV2.h"

#include "JSBSimMovementComponent.h"
#include "FormationControlV2/PlannerV2Adapters.h"
#include "Engine/World.h"

namespace FormationControlV2
{

namespace
{
constexpr double kFtToM = 0.3048;

bool PcFin(double V) { return FMath::IsFinite(V); }

// Same raw mapping the airborne shadow chain uses (MumtAirborneShadowTest.cpp ToRaw). Kept local so the
// producer depends only on the public snapshot getter, not on the test.
FNavigationRawSnapshotV2 ToRaw(const FJsbFlightSnapshot &S)
{
	FNavigationRawSnapshotV2 R{};
	R.VehicleCgEcefFt = {S.VehicleCgEcefXFt, S.VehicleCgEcefYFt, S.VehicleCgEcefZFt};
	R.EcefVelocityFps = {S.EcefVelocityXFps, S.EcefVelocityYFps, S.EcefVelocityZFps};
	R.GeodeticLatitudeRad = S.GeodeticLatitudeRad; R.LongitudeRad = S.LongitudeRad;
	R.GeodeticAltitudeFt = S.GeodeticAltitudeFt;
	R.EquivalentAirspeedKts = S.VequivalentKTS; R.TrueAirspeedFps = S.VtFps;
	R.WindNEDFps = {S.WindNorthFps, S.WindEastFps};
	R.AltitudeAslFt = S.AltAslFt; R.ClimbRateFps = S.HdotFps;
	R.SimulationTimeS = S.SimTimeSec; R.bHolding = S.bHolding; R.bValidFrame = S.bValidFrame;
	return R;
}

bool GetSnapshot(const UJSBSimMovementComponent *Comp, FJsbFlightSnapshot &Out)
{
	return Comp && Comp->GetJsbFlightSnapshot(Out) && Out.bValidFrame;
}
} // namespace

const TCHAR *ProducerResultName(EProducerResult R)
{
	switch (R)
	{
	case EProducerResult::NoFollowerComponent: return TEXT("NoFollowerComponent");
	case EProducerResult::NoLeaderComponent:   return TEXT("NoLeaderComponent");
	case EProducerResult::NoFollowerSnapshot:  return TEXT("NoFollowerSnapshot");
	case EProducerResult::NoLeaderSnapshot:    return TEXT("NoLeaderSnapshot");
	case EProducerResult::FrameNotEstablished: return TEXT("FrameNotEstablished");
	case EProducerResult::ChainNotReady:       return TEXT("ChainNotReady");
	case EProducerResult::NonFiniteChainOutput:return TEXT("NonFiniteChainOutput");
	case EProducerResult::PrimeRejected:       return TEXT("PrimeRejected");
	case EProducerResult::SnapshotUnavailable: return TEXT("SnapshotUnavailable");
	case EProducerResult::CandidateRejected:   return TEXT("CandidateRejected");
	case EProducerResult::ActivationRejected:  return TEXT("ActivationRejected");
	case EProducerResult::NotActive:           return TEXT("NotActive");
	default:                                   return TEXT("Ok");
	}
}

FFormationCandidateProducerV2::FFormationCandidateProducerV2() = default;

void FFormationCandidateProducerV2::Reset()
{
	check(IsInGameThread());
	++ResetGeneration;
	Planner.Reset(ResetGeneration);
	Guidance.Reset(ResetGeneration);
	Stick.Reset(ResetGeneration);
	FollowerNavTracker = FCanonicalNavigationTrackerV2{};
	LeaderNavTracker = FCanonicalNavigationTrackerV2{};
	FollowerStateTracker = MumtState::FMumtStateTracker{};
	Frame = MissionNavigationFrameV2{};
	bFrameSet = false;
}

bool FFormationCandidateProducerV2::ComputeChain(const UJSBSimMovementComponent *Follower,
                                                 const UJSBSimMovementComponent *Leader, double DtS,
                                                 uint64 PrimeGeneration, uint64 PrimeConsumeSequence,
                                                 FProducerFrameResult &Out)
{
	FJsbFlightSnapshot FS{}, LS{};
	if (!GetSnapshot(Follower, FS)) { Out.Result = EProducerResult::NoFollowerSnapshot; return false; }
	if (!GetSnapshot(Leader, LS))   { Out.Result = EProducerResult::NoLeaderSnapshot;   return false; }

	// Establish the mission NED frame from the leader on the first frame, exactly as the shadow chain does.
	if (!bFrameSet)
	{
		const bool bOk = Frame.SetOrigin(
			{LS.GeodeticLatitudeRad, LS.LongitudeRad, LS.GeodeticAltitudeFt * kFtToM, 1u, true});
		if (!bOk || !Frame.IsValid()) { Out.Result = EProducerResult::FrameNotEstablished; return false; }
		bFrameSet = true;
	}

	const auto FSt = MumtState::ConvertJsbToControlState(FS, FollowerStateTracker);

	const auto LNav = CanonicalNavigationAdapterV2::Convert(ToRaw(LS), Frame, ResetGeneration, LeaderNavTracker);
	const auto FNav = CanonicalNavigationAdapterV2::Convert(ToRaw(FS), Frame, ResetGeneration, FollowerNavTracker);

	FFormationSlotCommandV2 Cmd{};
	Cmd.FrontM = SlotSpec.FrontM; Cmd.RightM = SlotSpec.RightM; Cmd.UpM = SlotSpec.UpM;
	Cmd.CommandReceivedSimulationTimeS = FS.SimTimeSec; Cmd.SourceSequence = 1; Cmd.bValid = true;
	const auto Slot = FormationSlotGeneratorV2::Calculate(LNav, Cmd, FS.SimTimeSec);

	const auto PIn = PlannerV2InputAdapter::Build({FNav, Slot, FS.SimTimeSec, DtS});
	FormationPlannerV2Diagnostics Diag{};
	FormationPlannerV2Output PO{};
	if (PIn.bValid) PO = Planner.Update(PIn.Input, Diag);
	const auto Dto = PlannerV2OutputAdapter::Build(PO, Slot, FNav);

	FGuidanceCoordinatorInputV2 GI{};
	GI.Follower = FNav; GI.Slot = Slot; GI.PlannerDto = Dto;
	GI.CurrentPitchRad = FSt.Pitch_rad; GI.bCurrentPitchValid = FSt.bAttitudeValid;
	GI.CurrentRollRad = FSt.Roll_rad;   GI.bCurrentRollValid = FSt.bAttitudeValid;
	GI.SimulationTimeS = FS.SimTimeSec; GI.DtS = DtS;
	GI.ResetGeneration = ResetGeneration; GI.OriginGeneration = FNav.OriginGeneration;
	const auto GO = Guidance.Update(GI, GuidanceConfig);

	FF16StickInputV2 SI{};
	SI.RollReferenceRad = GO.RollReferenceRad; SI.PitchReferenceRad = GO.PitchReferenceRad;
	SI.ThrottleReferenceNorm = GO.ThrottleReferenceNorm; SI.bGuidanceValid = GO.bCommandReady;
	SI.CurrentRollRad = FSt.Roll_rad; SI.CurrentPitchRad = FSt.Pitch_rad; SI.bAttitudeValid = FSt.bAttitudeValid;
	SI.BodyRollRateRadps = FS.BodyRollRatePRadps; SI.BodyPitchRateRadps = FS.BodyPitchRateQRadps;
	SI.BodyYawRateRadps = FS.BodyYawRateRRadps;
	SI.bBodyRatesValid = PcFin(FS.BodyRollRatePRadps) && PcFin(FS.BodyPitchRateQRadps) && PcFin(FS.BodyYawRateRRadps);
	SI.AlphaRad = FS.AlphaRad; SI.BetaRad = FS.BetaRad; SI.bAlphaBetaValid = PcFin(FS.AlphaRad) && PcFin(FS.BetaRad);
	SI.EasMps = FSt.EquivalentAirspeed_mps; SI.TasMps = FSt.TrueAirspeed_mps;
	SI.bAirspeedValid = FSt.bEasValid && FSt.bTasValid;
	SI.SimulationTimeS = FS.SimTimeSec; SI.DtS = DtS; SI.bPaused = FSt.bPaused;
	SI.ResetGeneration = GO.ResetGeneration;
	// Thread the prime identity so the stick's one-shot baseline latch fires on the correct frame (and only
	// then). After the latch is spent, these are ignored and the normal path runs.
	SI.PrimeGeneration = PrimeGeneration; SI.PrimeConsumeSequence = PrimeConsumeSequence;

	const auto SO = Stick.Update(SI, StickConfig);

	Out.ChainAileron = SO.AileronCmdNorm; Out.ChainElevator = SO.ElevatorCmdNorm;
	Out.ChainRudder = SO.RudderCmdNorm;   Out.ChainThrottle = SO.ThrottleCmdNorm;
	Out.bChainCommandReady = SO.bValid;
	Out.bLatchFrame = (SO.FailureReason == EF16StickFailureV2::None) && !Stick.HasPrimedBaselineLatch()
	                  && PrimeGeneration != 0 && PrimeConsumeSequence != 0;

	if (!SO.bValid)
	{
		bChainReady = false;
		Out.Result = EProducerResult::ChainNotReady;
		return false;
	}
	if (!PcFin(SO.AileronCmdNorm) || !PcFin(SO.ElevatorCmdNorm) || !PcFin(SO.RudderCmdNorm) || !PcFin(SO.ThrottleCmdNorm))
	{
		bChainReady = false;
		Out.Result = EProducerResult::NonFiniteChainOutput;
		return false;
	}
	bChainReady = true;
	return true;
}

void FFormationCandidateProducerV2::WarmChain(const UJSBSimMovementComponent *Follower,
                                             const UJSBSimMovementComponent *Leader, double DtS)
{
	check(IsInGameThread());
	FProducerFrameResult Scratch{};
	// prime identity 0/0: no latch is expected, so the stick runs its normal path. Nothing is submitted.
	const bool bOk = ComputeChain(Follower, Leader, DtS, 0, 0, Scratch);
	bChainReady = bOk && Scratch.bChainCommandReady;
}

MumtCommandArbiterV2::FPrimedCandidate FFormationCandidateProducerV2::BuildCandidate(
	const MumtCommandArbiterV2::FPrimeTicket &Ticket, const FProducerFrameResult &Chain) const
{
	using namespace MumtCommandArbiterV2;
	FPrimedCandidate P;
	P.PrimeGeneration = Ticket.Generation;
	P.BaselineConsumeSequence = Ticket.BaselineConsumeSequence;

	// The five-field summary the arbiter's Phase B validity checks read.
	P.Candidate.AileronNorm    = Chain.ChainAileron;
	P.Candidate.ElevatorNorm   = Chain.ChainElevator;
	P.Candidate.RudderNorm     = Chain.ChainRudder;
	P.Candidate.ThrottleNorm   = Chain.ChainThrottle;
	// Speed-brake is NOT produced by the chain: it stays at the baseline. (Its baseline value is copied
	// into the full block below; the five-field summary carries the same value.)
	P.Candidate.SpeedBrakeNorm = Ticket.Baseline.Commands.SpeedBrake;
	P.Candidate.Generation = Chain.CandidateGeneration;
	P.Candidate.TimestampS = FApp::GetCurrentTime();
	P.Candidate.bValid = true;
	P.Candidate.bCommandReady = true;
	P.Candidate.bFinite = true;

	// THE FULL 29-FIELD BLOCK (Phase C): copy the immutable baseline, overwrite ONLY the fields the chain
	// owns. Every non-producer field -- trims, flaps, brakes, gear, and every engine field but throttle --
	// carries the baseline value, so the whole block is continuous with what the aircraft was flying.
	P.bHasFullBlock = true;
	P.FullCommands = Ticket.Baseline.Commands;
	P.FullCommands.Aileron  = Chain.ChainAileron;
	P.FullCommands.Elevator = Chain.ChainElevator;
	P.FullCommands.Rudder   = Chain.ChainRudder;
	// SpeedBrake left at baseline (already copied). Everything else left at baseline.
	P.FullEngineCommands = Ticket.Baseline.EngineCommands;
	if (P.FullEngineCommands.Num() > 0)
	{
		P.FullEngineCommands[0].Throttle = Chain.ChainThrottle;
	}
	return P;
}

FProducerFrameResult FFormationCandidateProducerV2::PrepareHandoff(const UJSBSimMovementComponent *Follower,
                                                                   const UJSBSimMovementComponent *Leader,
                                                                   double DtS)
{
	check(IsInGameThread());
	using namespace MumtCommandArbiterV2;
	FProducerFrameResult R{};

	if (!Follower) { R.Result = EProducerResult::NoFollowerComponent; return R; }
	if (!Leader)   { R.Result = EProducerResult::NoLeaderComponent;   return R; }

	// 1. The baseline: the last command the FDM actually consumed for this aircraft.
	FResolvedCommandSnapshot Snap{};
	if (!GetLastResolvedSnapshot(Follower, Snap)) { R.Result = EProducerResult::SnapshotUnavailable; return R; }

	// Body pitch rate for the integrator seed, from the follower's current state.
	FJsbFlightSnapshot FS{};
	if (!GetSnapshot(Follower, FS)) { R.Result = EProducerResult::NoFollowerSnapshot; return R; }

	// 2. RequestPrime.
	FPrimeTicket Ticket{};
	EPrimeFailure Fail = EPrimeFailure::None;
	if (!RequestPrime(Follower, Ticket, Fail))
	{
		R.Result = EProducerResult::PrimeRejected; R.LastPrimeFailure = Fail; return R;
	}
	// 3. Keep the ticket identity.
	R.Generation = Ticket.Generation;
	R.BaselineConsumeSequence = Ticket.BaselineConsumeSequence;

	// 4. Prime the stick from the baseline (arms the one-shot baseline latch, keyed to this generation +
	//    consume sequence). If it refuses, the baseline was not usable -- do NOT proceed with a half-armed
	//    handoff; drop back to Legacy cleanly.
	const bool bStickPrimed = Stick.PrimeFromResolvedCommand(
		Snap.Commands.Aileron, Snap.Commands.Elevator, Snap.Commands.Rudder,
		Snap.EngineCommands.Num() > 0 ? Snap.EngineCommands[0].Throttle : 0.0,
		FS.BodyPitchRateQRadps, StickConfig, Ticket.Generation, Ticket.BaselineConsumeSequence, ResetGeneration);
	if (!bStickPrimed)
	{
		RequestLegacyFallback(Follower);
		R.Result = EProducerResult::ChainNotReady; return R;
	}

	// 5. Compute the FIRST candidate. The primed stick returns the baseline EXACTLY on this first compute
	//    (the latch), so the candidate reproduces the baseline -- the zero-step guarantee.
	if (!ComputeChain(Follower, Leader, DtS, Ticket.Generation, Ticket.BaselineConsumeSequence, R))
	{
		RequestLegacyFallback(Follower);
		return R;   // R.Result already set by ComputeChain
	}

	// 6. Submit as a full 29-field block.
	R.CandidateGeneration = ++CandidateGenerationCounter;
	const FPrimedCandidate Cand = BuildCandidate(Ticket, R);
	R.CandAileron = Cand.Candidate.AileronNorm; R.CandElevator = Cand.Candidate.ElevatorNorm;
	R.CandRudder = Cand.Candidate.RudderNorm;   R.CandThrottle = Cand.Candidate.ThrottleNorm;
	R.CandSpeedBrake = Cand.Candidate.SpeedBrakeNorm;
	if (!SubmitPrimedCandidate(Follower, Cand, Fail))
	{
		RequestLegacyFallback(Follower);
		R.Result = EProducerResult::CandidateRejected; R.LastPrimeFailure = Fail; return R;
	}
	R.bSubmitted = true;
	R.Result = EProducerResult::Ok;
	return R;
}

bool FFormationCandidateProducerV2::RequestActivation(const UJSBSimMovementComponent *Follower,
                                                      MumtCommandArbiterV2::EPrimeFailure &OutFailure)
{
	check(IsInGameThread());
	return MumtCommandArbiterV2::RequestFormationActivation(Follower, OutFailure);
}

FProducerFrameResult FFormationCandidateProducerV2::BeginHandoff(const UJSBSimMovementComponent *Follower,
                                                                 const UJSBSimMovementComponent *Leader,
                                                                 double DtS)
{
	using namespace MumtCommandArbiterV2;
	// Steps 1-6 atomically, then step 7 -- all in one game-thread call so no Legacy consume can slip
	// between the prime and the activation (which would make the baseline stale and step the handoff).
	FProducerFrameResult R = PrepareHandoff(Follower, Leader, DtS);
	if (R.Result != EProducerResult::Ok) return R;

	EPrimeFailure Fail = EPrimeFailure::None;
	if (!RequestActivation(Follower, Fail))
	{
		RequestLegacyFallback(Follower);
		R.Result = EProducerResult::ActivationRejected; R.LastPrimeFailure = Fail;
	}
	return R;
}

FProducerFrameResult FFormationCandidateProducerV2::Update(const UJSBSimMovementComponent *Follower,
                                                           const UJSBSimMovementComponent *Leader,
                                                           double DtS)
{
	check(IsInGameThread());
	using namespace MumtCommandArbiterV2;
	FProducerFrameResult R{};

	if (!Follower) { R.Result = EProducerResult::NoFollowerComponent; return R; }
	if (!Leader)   { R.Result = EProducerResult::NoLeaderComponent;   return R; }

	// An ongoing update only makes sense once the handover has actually landed. Before that, the arbiter is
	// still holding the first (baseline) candidate; resubmitting would be racing the activation boundary.
	if (GetPrimeState(Follower) != EPrimeState::ActiveFormation)
	{
		R.Result = EProducerResult::NotActive; return R;
	}

	// The ticket the aircraft is currently flying on. Its baseline is immutable; only the candidate moves.
	FResolvedCommandSnapshot Baseline{};
	if (!GetTicketBaseline(Follower, Baseline)) { R.Result = EProducerResult::SnapshotUnavailable; return R; }
	FPrimeTicket Ticket{};
	Ticket.Generation = GetPrimeGeneration(Follower);
	Ticket.BaselineConsumeSequence = Baseline.ConsumeSequence;
	Ticket.Baseline = Baseline;
	Ticket.bValid = true;

	// The latch is spent, so the stick now produces REAL output -- pass 0/0 as prime identity so no latch
	// is expected. (It is already consumed; this is belt-and-braces.)
	if (!ComputeChain(Follower, Leader, DtS, 0, 0, R))
	{
		// A momentary invalid chain frame is NOT a fallback trigger here: the arbiter will fall back on its
		// own if we stop submitting fresh candidates (the last one goes stale). We simply skip this frame.
		return R;   // R.Result set by ComputeChain
	}

	R.CandidateGeneration = ++CandidateGenerationCounter;
	EPrimeFailure Fail = EPrimeFailure::None;
	const FPrimedCandidate Cand = BuildCandidate(Ticket, R);
	R.CandAileron = Cand.Candidate.AileronNorm; R.CandElevator = Cand.Candidate.ElevatorNorm;
	R.CandRudder = Cand.Candidate.RudderNorm;   R.CandThrottle = Cand.Candidate.ThrottleNorm;
	R.CandSpeedBrake = Cand.Candidate.SpeedBrakeNorm;
	R.Generation = Ticket.Generation;
	R.BaselineConsumeSequence = Ticket.BaselineConsumeSequence;
	if (!SubmitPrimedCandidate(Follower, Cand, Fail))
	{
		R.Result = EProducerResult::CandidateRejected; R.LastPrimeFailure = Fail; return R;
	}
	R.bSubmitted = true;
	R.Result = EProducerResult::Ok;
	return R;
}

} // namespace FormationControlV2
