// MumtAirborneShadowTest.cpp — AIRBORNE live shadow validation of the V2 control chain.
//
// The aircraft are flown by the EXISTING command writer (AUDPControlReceiver -> InnerLoopAutopilot
// -> UJSBSimMovementComponent::Commands / EngineCommands), driven by its in-engine scripted
// formation profile which is enabled headlessly with the existing `-FormationTest` command-line
// switch (Takeoff -> Straight 220 -> Turn 3 deg/s -> Turn 4 deg/s + 400 m climb + 220->170 decel
// -> Rollout 170 -> Breakaway/Detour -> FarChase rejoin). No scenario/Blueprint/Content/Config is
// created or modified.
//
// This test only OBSERVES. Per tick it reads the follower's atomic JSBSim snapshot and shadow-runs:
//   Snapshot -> MissionNavigation -> CanonicalNavigation -> SlotGenerator(V2, real moving leader)
//            -> FormationPlannerV2 -> PlannerV2OutputAdapter -> Guidance (real PX4 NPFG/TECS)
//            -> F16StickAdapterV2 -> command DTO
// The command DTO is recorded and asserted; it is NEVER written to JSBSim Commands, EngineCommands
// or fcs/*-cmd-norm. Separation from the existing writer is structural: this file contains no write
// path at all, and it uses its own adapter instances (one set per observed aircraft).
#include "Misc/AutomationTest.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

#include "JSBSimMovementComponent.h"
#include "State/MumtControlState.h"
#include "FormationControlV2/CanonicalNavigationAdapterV2.h"
#include "FormationControlV2/FormationSlotGeneratorV2.h"
#include "FormationControlV2/FormationPlannerV2.h"
#include "FormationControlV2/FormationGuidanceCoordinatorV2.h"
#include "FormationControlV2/F16StickAdapterV2.h"
#include "FormationControlV2/PlannerV2Adapters.h"
#include "FormationControlV2/SlotLocalPathPrimitiveV2.h"
#include "Tests/AutomationEditorCommon.h"
#include "Editor.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformTime.h"
#include <cmath>

DEFINE_LOG_CATEGORY_STATIC(LogMumtAir, Display, All);

namespace
{
// Unity builds concatenate this TU with MumtLiveSnapshotTest.cpp, which has its own anonymous
// namespace; identically-named helpers would collide there. Keep these names TU-distinct.
constexpr double kAirFtToM = 0.3048;
constexpr double kDtS = 1.0 / 60.0;
const TCHAR *kAirMap = TEXT("/Game/RL_2");
constexpr double kMaxWallSeconds = 420.0;   // hard cap for the whole observation
constexpr double kMaxSimSeconds  = 300.0;   // covers takeoff + all scripted phases
// Single reset generation for the whole observation: the canonical conversion, the slot, the
// guidance coordinator and the stick adapter must all agree, or the coordinator rejects the frame.
constexpr uint32 kResetGen = 1u;
static bool AirIsFin(double x) { return std::isfinite(x); }

// Frame populations. Every observed sample lands in exactly one of these — see the accounting
// assertion (Unclassified must be 0).
enum class EFramePop : uint8 { Initial, Feasible, Infeasible, Boundary, InvalidInput, Unclassified };

// Mutually exclusive open-loop observation phases. Ground/airborne comes from JSBSim's existing
// weight-on-wheels state, never from an airspeed or ASL heuristic. PlannerRejected and Invalid take
// precedence because those samples have no command-ready V2 output to grade for excitation.
enum class EOpenLoopPhase : uint8 { Initialization, TakeoffOrGround, Airborne, PlannerRejected, Invalid, Count };
constexpr int32 kOpenLoopPhaseCount = static_cast<int32>(EOpenLoopPhase::Count);

// Which acceptance contract this run grades. All three observe the SAME frames and log the SAME
// populations; they differ only in which population they grade and which assertions they raise.
enum class EAcceptance : uint8 { PlannerFeasible, EnvelopeRejection, FullControl };

// The settling window used to grade convergence: the last 20% of a feasible run.
constexpr double kSettlingWindowFraction = 0.20;
// Recovery must happen within this many frames after feasibility returns.
constexpr int32 kMaxRecoveryFrames = 90;   // 1.5 s at 60 Hz

// One shadow chain (adapters are per-aircraft, proving instance independence).
struct FShadowChain
{
	FormationControlV2::FCanonicalNavigationTrackerV2 NavTracker;
	FormationControlV2::FormationPlannerV2            Planner;
	FormationControlV2::FormationGuidanceCoordinatorV2 Guidance;
	FormationControlV2::F16StickAdapterV2             Stick;
};

struct FAirState
{
	TWeakObjectPtr<UJSBSimMovementComponent> Leader, Follower;
	FString LeaderName, FollowerName, AllAircraft;
	int32 TotalComponents = 0, ActorsWithComp = 0, MaxCompsPerActor = 0;
	bool bDiscovered = false;

	FormationControlV2::MissionNavigationFrameV2 Frame;
	bool bFrameSet = false;
	FShadowChain FollowerChain, LeaderChain;   // independent instances per aircraft
	FormationControlV2::FF16StickConfigV2 StickConfig{};

	double FirstWall = -1.0;
	double FirstSimTime = -1.0, LastSimTime = -1.0, MaxSimTime = 0.0;
	int32 Samples = 0;

	// ---- pipeline observation ----
	int32 PlannerValid = 0, GuidanceValid = 0, StickValid = 0, LeaderStickValid = 0;
	int32 PlannerInvalid = 0, StickNotReadyOnPlannerInvalid = 0;
	int32 AirborneSamples = 0;                 // authoritative JSBSim WOW == false phase

	// ---- state ranges ----
	double AltMin = 1e9, AltMax = -1e9, EasMin = 1e9, EasMax = -1e9, TasMin = 1e9, TasMax = -1e9;
	double RollMin = 1e9, RollMax = -1e9, PitchMin = 1e9, PitchMax = -1e9;
	double PMin = 1e9, PMax = -1e9, QMin = 1e9, QMax = -1e9, RMin = 1e9, RMax = -1e9;
	double AlphaMin = 1e9, AlphaMax = -1e9, BetaMin = 1e9, BetaMax = -1e9;
	double GsMin = 1e9, GsMax = -1e9, ClimbMin = 1e9, ClimbMax = -1e9;
	double MaxAbsWind = 0.0;

	// ---- planner ----
	double AlongMin = 1e9, AlongMax = -1e9, CrossMin = 1e9, CrossMax = -1e9;
	double CurvMin = 1e9, CurvMax = -1e9, TgtEasMin = 1e9, TgtEasMax = -1e9, TgtAltMin = 1e9, TgtAltMax = -1e9;
	int32 ModeTransitions = 0, ReplanCount = 0, HeldPathFrames = 0, ZeroTangentFrames = 0;
	uint8 LastMode = 255; uint32 ReplanReasonMask = 0;
	int32 TypeCount[7] = {0,0,0,0,0,0,0};
	// ---- near-field / candidate-storm observation ----
	int32 ModeFrames[5] = {0,0,0,0,0};          // Rejoin, NearFieldSlotTrack, CaptureEntry, ClosureTaper, SlotHold
	int32 CandidateEvaluations = 0, MaxConsecutiveCandidateFailures = 0, SlotTrackFrames = 0;
	uint32 CandidateRejectMask = 0;
	double NfTargetEasMin = 1e9, NfTargetEasMax = -1e9;
	double InitialAbsAlong = -1.0, InitialAbsCross = -1.0, BestAbsAlong = 1e9, BestAbsCross = 1e9;
	// Histogram is sized off the enum sentinel, so a newly added PlannerFailure can never fall
	// outside the reporting range again.
	int32 PlannerFailCount[FormationControlV2::kPlannerFailureCount] = {0};
	int32 GuidanceFailCount[16] = {0};  // indexed by EGuidanceFailureV2
	static_assert(static_cast<int32>(FormationControlV2::EGuidanceFailureV2::InvalidConfig) < 16,
		"GuidanceFailCount must include InvalidConfig");

	// ---- mutually exclusive open-loop phase accounting ----
	int32 OpenPhaseCount[kOpenLoopPhaseCount] = {0,0,0,0,0};
	double OpenPhaseStartS[kOpenLoopPhaseCount] = {1e9,1e9,1e9,1e9,1e9};
	double OpenPhaseEndS[kOpenLoopPhaseCount] = {-1,-1,-1,-1,-1};
	uint8 LastOpenPhase = 255;
	int32 OpenPhaseTransitions = 0, WowUnavailableFrames = 0;

	// ---- frame populations (authoritative shared turn-bound contract) ----
	// total == Initial + Feasible + Infeasible + Boundary + InvalidInput, and Unclassified == 0.
	int32 PopInitial = 0, PopFeasible = 0, PopInfeasible = 0, PopBoundary = 0, PopInvalidInput = 0, PopUnclassified = 0;

	// feasible-population acceptance
	int32 FeasValid = 0, FeasInvalid = 0, FeasCandidateFail = 0, FeasCurvInfeasible = 0;
	int32 FeasStale = 0, FeasNonFinite = 0, FeasGenMismatch = 0, FeasExactPoseDubins = 0;
	int32 FeasModeFrames[5] = {0,0,0,0,0};
	double FeasFirstAbsCross = -1.0, FeasFirstAbsAlong = -1.0;
	TArray<double> FeasAbsCross, FeasAbsAlong;   // per valid feasible frame, in order (settling window)

	// infeasible-population rejection acceptance
	int32 InfCurvInfeasible = 0, InfOtherFailure = 0, InfFalseAccept = 0, InfCandidateFail = 0;
	int32 InfPathValid = 0, InfTargetEasValid = 0, InfStale = 0, InfGuidanceReady = 0, InfStickReady = 0;
	int32 InfRun = 0, InfLongestRun = 0;
	int32 RecoveryFrames = -1, MaxRecoveryFrames = 0;   // frames from last infeasible frame to next valid
	int32 InfByLeaderRate[4] = {0,0,0,0};               // <1, 1-2.5, 2.5-3.5, >3.5 deg/s
	int32 FalseReject = 0;                              // feasible frame rejected as SlotCurvatureInfeasible
	int32 CurvClampViolations = 0;                      // |output curvature| > 1/Rmin on ANY valid frame

	// boundary-population
	int32 BndRun = 0, BndLongestRun = 0, BndStale = 0, BndNonFinite = 0, BndPartial = 0, BndContradiction = 0;
	int32 BndModeChanges = 0; uint8 BndLastMode = 255;

	// ---- guidance ----
	double RollRefMin = 1e9, RollRefMax = -1e9, PitchRefMin = 1e9, PitchRefMax = -1e9, ThrRefMin = 1e9, ThrRefMax = -1e9;
	double FfMin = 1e9, FfMax = -1e9, FbMin = 1e9, FbMax = -1e9, LatMin = 1e9, LatMax = -1e9;
	double FeasMin = 1e9, FeasMax = -1e9;
	int32 UnderspeedFrames = 0, FastDescendFrames = 0;

	// ---- stick ----
	double AilMin = 1e9, AilMax = -1e9, ElevMin = 1e9, ElevMax = -1e9, RudMin = 1e9, RudMax = -1e9, ThrCmdMin = 1e9, ThrCmdMax = -1e9;
	int32 SatFrames = 0, SlewFrames = 0, SatRun = 0, MaxSatRun = 0;
	int32 RollSignChecks = 0, RollSignViolations = 0, PitchSignChecks = 0, PitchSignViolations = 0;
	int32 PosRollErrPosAil = 0, NegRollErrNegAil = 0, PosPitchErrNegElev = 0, NegPitchErrPosElev = 0;
	bool bPrevStick = false; FormationControlV2::FF16StickCommandV2 PrevStick{};

	// ---- phase-specific open-loop output coverage (diagnostic unless asserted explicitly) ----
	double GroundPitchRefMin = 1e9, GroundPitchRefMax = -1e9;
	double AirPitchRefMin = 1e9, AirPitchRefMax = -1e9;
	double AirElevMin = 1e9, AirElevMax = -1e9, AirThrottleMin = 1e9, AirThrottleMax = -1e9;
	double AirRollRefMin = 1e9, AirRollRefMax = -1e9;
	int32 GroundValidOutputs = 0, AirValidOutputs = 0;
	int32 AirPitchRefPositive = 0, AirPitchRefNegative = 0;
	int32 AirElevPositive = 0, AirElevNegative = 0, AirRollPositive = 0, AirRollNegative = 0;
	int32 PartialGuidanceStickFrames = 0;

	// ---- invariants ----
	bool bAllFinite = true, bAllInRange = true, bSlewOk = true, bNoStale = true;
	bool bResetGenOk = true, bGameThread = true, bSimMonotonic = true;
	int32 InvalidRun = 0, MaxInvalidRun = 0;

	TArray<FString> Csv;
};

static UWorld *GetAirPIEWorld()
{
	if (!GEditor) return nullptr;
	for (const FWorldContext &C : GEditor->GetWorldContexts())
		if (C.WorldType == EWorldType::PIE && C.World()) return C.World();
	return nullptr;
}

static FormationControlV2::FNavigationRawSnapshotV2 ToRaw(const FJsbFlightSnapshot &S)
{
	FormationControlV2::FNavigationRawSnapshotV2 R{};
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
} // namespace

class FMumtAirborneSampleCommand : public IAutomationLatentCommand
{
public:
	FMumtAirborneSampleCommand(FAutomationTestBase *T, TSharedPtr<FAirState> S, EAcceptance InMode)
		: Test(T), St(S), Mode(InMode) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		if (St->FirstWall < 0) St->FirstWall = NowWall;
		if (!IsInGameThread()) St->bGameThread = false;

		UWorld *World = GetAirPIEWorld();
		if (!World) { if (NowWall - St->FirstWall > 60.0) { Test->AddError(TEXT("[AIR] no PIE world")); return Finalize(); } return false; }

		if (!St->bDiscovered)
		{
			// Identify aircraft the SAME way the existing writer (AUDPControlReceiver::PawnIdName /
			// MatchPawnByKey) does: by the World Outliner actor LABEL ("M_F16" / "F16_UAV1"), not the
			// auto object name ("F16_UAV_C_2"). Otherwise a parked sibling UAV would be observed.
			auto IdName = [](const AActor *A) -> FString {
				const FString Label = A->GetActorLabel(false);
				return Label.IsEmpty() ? A->GetName() : Label;
			};
			St->ActorsWithComp = St->TotalComponents = St->MaxCompsPerActor = 0;
			for (TActorIterator<AActor> It(World); It; ++It)
			{
				TArray<UJSBSimMovementComponent *> Comps; It->GetComponents(Comps);
				if (Comps.Num() == 0) continue;
				St->ActorsWithComp++; St->TotalComponents += Comps.Num();
				St->MaxCompsPerActor = FMath::Max(St->MaxCompsPerActor, Comps.Num());
				const FString Id = IdName(*It);
				St->AllAircraft += FString::Printf(TEXT("%s "), *Id);
				if (Id == TEXT("F16_UAV1")) { St->Follower = Comps[0]; St->FollowerName = Id; }
				else if (Id == TEXT("M_F16")) { St->Leader = Comps[0]; St->LeaderName = Id; }
			}
			if (!St->Leader.IsValid() || !St->Follower.IsValid())
			{
				if (NowWall - St->FirstWall > 60.0)
				{
					Test->AddError(FString::Printf(TEXT("[AIR] formation-test aircraft not found by label (leader='M_F16' %s, follower='F16_UAV1' %s); labels seen: %s"),
						St->Leader.IsValid() ? TEXT("OK") : TEXT("MISSING"), St->Follower.IsValid() ? TEXT("OK") : TEXT("MISSING"), *St->AllAircraft));
					return Finalize();
				}
				return false;
			}
			St->bDiscovered = true;
		}

		UJSBSimMovementComponent *L = St->Leader.Get(), *F = St->Follower.Get();
		if (!L || !F) { Test->AddError(TEXT("[AIR] component invalidated")); return Finalize(); }

		// Atomic per-aircraft snapshots (read-only).
		FJsbFlightSnapshot LS{}, FS{};
		const bool lok = L->GetJsbFlightSnapshot(LS), fok = F->GetJsbFlightSnapshot(FS);
		if (!lok || !fok || !LS.bValidFrame || !FS.bValidFrame) return Continue(NowWall);

		if (!St->bFrameSet)
		{
			St->Frame.SetOrigin({LS.GeodeticLatitudeRad, LS.LongitudeRad, LS.GeodeticAltitudeFt * kAirFtToM, 1u, true});
			St->bFrameSet = true; St->FirstSimTime = FS.SimTimeSec;
		}
		// sample only when the follower's sim time advanced (JSBSim stepped)
		if (St->LastSimTime >= 0.0 && FS.SimTimeSec <= St->LastSimTime + 1e-9) return Continue(NowWall);
		if (St->LastSimTime >= 0.0 && FS.SimTimeSec < St->LastSimTime) St->bSimMonotonic = false;

		MumtState::FMumtStateTracker Dummy{}; // per-call SI conversion for logging only
		const auto FSt = MumtState::ConvertJsbToControlState(FS, Dummy);

		// ---- SHADOW CHAIN (follower) ----
		const auto LNav = FormationControlV2::CanonicalNavigationAdapterV2::Convert(ToRaw(LS), St->Frame, kResetGen, St->LeaderChain.NavTracker);
		const auto FNav = FormationControlV2::CanonicalNavigationAdapterV2::Convert(ToRaw(FS), St->Frame, kResetGen, St->FollowerChain.NavTracker);

		FormationControlV2::FFormationSlotCommandV2 Cmd{};
		Cmd.FrontM = -200.0; Cmd.RightM = 100.0; Cmd.UpM = 0.0;
		Cmd.CommandReceivedSimulationTimeS = FS.SimTimeSec; Cmd.SourceSequence = 1; Cmd.bValid = true;
		const auto Slot = FormationControlV2::FormationSlotGeneratorV2::Calculate(LNav, Cmd, FS.SimTimeSec);

		const auto PIn = FormationControlV2::PlannerV2InputAdapter::Build({FNav, Slot, FS.SimTimeSec, kDtS});
		FormationControlV2::FormationPlannerV2Diagnostics D{};
		FormationControlV2::FormationPlannerV2Output PO{};
		if (PIn.bValid) PO = St->FollowerChain.Planner.Update(PIn.Input, D);
		const auto Dto = FormationControlV2::PlannerV2OutputAdapter::Build(PO, Slot, FNav);

		FormationControlV2::FGuidanceCoordinatorInputV2 GI{};
		GI.Follower = FNav; GI.Slot = Slot; GI.PlannerDto = Dto;
		GI.CurrentPitchRad = FSt.Pitch_rad; GI.bCurrentPitchValid = FSt.bAttitudeValid;
		GI.SimulationTimeS = FS.SimTimeSec; GI.DtS = kDtS;
		// Must be the SAME generation used for the canonical conversion above (kResetGen), otherwise the
		// coordinator correctly rejects the frame with ResetMismatch. (The per-call SI tracker below is
		// only used for logging, so its own generation counter is not the pipeline's.)
		GI.ResetGeneration = kResetGen; GI.OriginGeneration = FNav.OriginGeneration;
		const auto GO = St->FollowerChain.Guidance.Update(GI);

		FormationControlV2::FF16StickInputV2 SI{};
		SI.RollReferenceRad = GO.RollReferenceRad; SI.PitchReferenceRad = GO.PitchReferenceRad;
		SI.ThrottleReferenceNorm = GO.ThrottleReferenceNorm; SI.bGuidanceValid = GO.bCommandReady;
		SI.CurrentRollRad = FSt.Roll_rad; SI.CurrentPitchRad = FSt.Pitch_rad; SI.bAttitudeValid = FSt.bAttitudeValid;
		SI.BodyRollRateRadps = FS.BodyRollRatePRadps; SI.BodyPitchRateRadps = FS.BodyPitchRateQRadps; SI.BodyYawRateRadps = FS.BodyYawRateRRadps;
		SI.bBodyRatesValid = AirIsFin(FS.BodyRollRatePRadps) && AirIsFin(FS.BodyPitchRateQRadps) && AirIsFin(FS.BodyYawRateRRadps);
		SI.AlphaRad = FS.AlphaRad; SI.BetaRad = FS.BetaRad; SI.bAlphaBetaValid = AirIsFin(FS.AlphaRad) && AirIsFin(FS.BetaRad);
		SI.EasMps = FSt.EquivalentAirspeed_mps; SI.TasMps = FSt.TrueAirspeed_mps; SI.bAirspeedValid = FSt.bEasValid && FSt.bTasValid;
		SI.SimulationTimeS = FS.SimTimeSec; SI.DtS = kDtS; SI.bPaused = FSt.bPaused; SI.ResetGeneration = GO.ResetGeneration;
		const auto SO = St->FollowerChain.Stick.Update(SI, St->StickConfig);

		// Leader chain runs too (independence: separate instances, separate state).
		{
			const auto LSlot = FormationControlV2::FormationSlotGeneratorV2::Calculate(LNav, Cmd, LS.SimTimeSec);
			const auto LPIn = FormationControlV2::PlannerV2InputAdapter::Build({LNav, LSlot, LS.SimTimeSec, kDtS});
			FormationControlV2::FormationPlannerV2Diagnostics LD{};
			FormationControlV2::FormationPlannerV2Output LPO{};
			if (LPIn.bValid) LPO = St->LeaderChain.Planner.Update(LPIn.Input, LD);
			const auto LDto = FormationControlV2::PlannerV2OutputAdapter::Build(LPO, LSlot, LNav);
			MumtState::FMumtStateTracker LDummy{}; const auto LSt = MumtState::ConvertJsbToControlState(LS, LDummy);
			FormationControlV2::FGuidanceCoordinatorInputV2 LGI{};
			LGI.Follower = LNav; LGI.Slot = LSlot; LGI.PlannerDto = LDto;
			LGI.CurrentPitchRad = LSt.Pitch_rad; LGI.bCurrentPitchValid = LSt.bAttitudeValid;
			LGI.SimulationTimeS = LS.SimTimeSec; LGI.DtS = kDtS;
			LGI.ResetGeneration = kResetGen; LGI.OriginGeneration = LNav.OriginGeneration;
			const auto LGO = St->LeaderChain.Guidance.Update(LGI);
			FormationControlV2::FF16StickInputV2 LSI{};
			LSI.RollReferenceRad = LGO.RollReferenceRad; LSI.PitchReferenceRad = LGO.PitchReferenceRad;
			LSI.ThrottleReferenceNorm = LGO.ThrottleReferenceNorm; LSI.bGuidanceValid = LGO.bCommandReady;
			LSI.CurrentRollRad = LSt.Roll_rad; LSI.CurrentPitchRad = LSt.Pitch_rad; LSI.bAttitudeValid = LSt.bAttitudeValid;
			LSI.bBodyRatesValid = true; LSI.bAlphaBetaValid = true; LSI.bAirspeedValid = true;
			LSI.SimulationTimeS = LS.SimTimeSec; LSI.DtS = kDtS; LSI.ResetGeneration = LGO.ResetGeneration;
			const auto LSO = St->LeaderChain.Stick.Update(LSI, St->StickConfig);
			if (LSO.bValid) St->LeaderStickValid++;
		}

		// ---------------- record ----------------
		St->Samples++; St->LastSimTime = FS.SimTimeSec; St->MaxSimTime = FS.SimTimeSec;
		const double gs = FNav.GroundVelocityNE_mps.Norm();
		St->AltMin = FMath::Min(St->AltMin, FSt.AltitudeAsl_m); St->AltMax = FMath::Max(St->AltMax, FSt.AltitudeAsl_m);
		St->EasMin = FMath::Min(St->EasMin, FSt.EquivalentAirspeed_mps); St->EasMax = FMath::Max(St->EasMax, FSt.EquivalentAirspeed_mps);
		St->TasMin = FMath::Min(St->TasMin, FSt.TrueAirspeed_mps); St->TasMax = FMath::Max(St->TasMax, FSt.TrueAirspeed_mps);
		St->RollMin = FMath::Min(St->RollMin, FSt.Roll_rad); St->RollMax = FMath::Max(St->RollMax, FSt.Roll_rad);
		St->PitchMin = FMath::Min(St->PitchMin, FSt.Pitch_rad); St->PitchMax = FMath::Max(St->PitchMax, FSt.Pitch_rad);
		St->PMin = FMath::Min(St->PMin, FS.BodyRollRatePRadps); St->PMax = FMath::Max(St->PMax, FS.BodyRollRatePRadps);
		St->QMin = FMath::Min(St->QMin, FS.BodyPitchRateQRadps); St->QMax = FMath::Max(St->QMax, FS.BodyPitchRateQRadps);
		St->RMin = FMath::Min(St->RMin, FS.BodyYawRateRRadps); St->RMax = FMath::Max(St->RMax, FS.BodyYawRateRRadps);
		St->AlphaMin = FMath::Min(St->AlphaMin, FS.AlphaRad); St->AlphaMax = FMath::Max(St->AlphaMax, FS.AlphaRad);
		St->BetaMin = FMath::Min(St->BetaMin, FS.BetaRad); St->BetaMax = FMath::Max(St->BetaMax, FS.BetaRad);
		St->GsMin = FMath::Min(St->GsMin, gs); St->GsMax = FMath::Max(St->GsMax, gs);
		St->ClimbMin = FMath::Min(St->ClimbMin, FSt.ClimbRate_mps); St->ClimbMax = FMath::Max(St->ClimbMax, FSt.ClimbRate_mps);
		St->MaxAbsWind = FMath::Max(St->MaxAbsWind, FNav.WindNE_mps.Norm());

		// Authoritative open-loop phase: JSBSim updates FGear::HasWeightOnWheel from the
		// ground-reaction model. No EAS/ASL threshold is used. The private FormationTest phase cannot
		// be read here without changing Production code, so WOW is the existing read-only boundary.
		const bool bWowAvailable = F->Gears.Num() > 0;
		bool bWeightOnWheels = false;
		for (const FGear &Gear : F->Gears) bWeightOnWheels |= Gear.HasWeightOnWheel;
		if (!bWowAvailable) St->WowUnavailableFrames++;
		if (GO.bCommandReady != SO.bValid) St->PartialGuidanceStickFrames++;

		using FPF = FormationControlV2::PlannerFailure;
		EOpenLoopPhase OpenPhase = EOpenLoopPhase::Invalid;
		if (St->Samples <= 1 || GO.FailureReason == FormationControlV2::EGuidanceFailureV2::ResetFrame ||
		    SO.FailureReason == FormationControlV2::EF16StickFailureV2::ResetFrame) {
			OpenPhase = EOpenLoopPhase::Initialization;
		} else if (!PIn.bValid || !bWowAvailable || PO.Failure == FPF::Paused ||
		           PO.Failure == FPF::AbnormalDt || PO.Failure == FPF::CriticalInputInvalid ||
		           PO.Failure == FPF::TurnBoundInvalid || PO.Failure == FPF::SlotCurvatureUnavailable ||
		           (PO.bValid && (!GO.bCommandReady || !SO.bValid))) {
			OpenPhase = EOpenLoopPhase::Invalid;
		} else if (!PO.bValid) {
			OpenPhase = EOpenLoopPhase::PlannerRejected;
		} else if (bWeightOnWheels) {
			OpenPhase = EOpenLoopPhase::TakeoffOrGround;
		} else {
			OpenPhase = EOpenLoopPhase::Airborne;
		}
		const int32 OpenPhaseIndex = static_cast<int32>(OpenPhase);
		St->OpenPhaseCount[OpenPhaseIndex]++;
		St->OpenPhaseStartS[OpenPhaseIndex] = FMath::Min(St->OpenPhaseStartS[OpenPhaseIndex], FS.SimTimeSec);
		St->OpenPhaseEndS[OpenPhaseIndex] = FMath::Max(St->OpenPhaseEndS[OpenPhaseIndex], FS.SimTimeSec);
		if (St->LastOpenPhase != 255 && St->LastOpenPhase != static_cast<uint8>(OpenPhase)) St->OpenPhaseTransitions++;
		St->LastOpenPhase = static_cast<uint8>(OpenPhase);
		if (OpenPhase == EOpenLoopPhase::Airborne) St->AirborneSamples++;

		// candidate-evaluation observation runs on EVERY frame, valid or not: a replan storm shows up
		// precisely on the frames where the planner failed to select a candidate.
		St->CandidateEvaluations = FMath::Max(St->CandidateEvaluations, D.CandidateEvaluationCount);
		St->MaxConsecutiveCandidateFailures = FMath::Max(St->MaxConsecutiveCandidateFailures, D.ConsecutiveCandidateFailures);
		St->CandidateRejectMask |= D.LastCandidateRejectMask;
		if (D.bSlotLocalPath) St->SlotTrackFrames++;
		{
			const int mi = (int)PO.Mode;
			if (mi >= 0 && mi < 5) St->ModeFrames[mi]++;
		}

		// ---------------- frame population (AUTHORITATIVE shared turn-bound contract) ----------------
		// The classification uses FormationControlV2::ClassifySlotCurvature — the SAME function the
		// production primitive rejects with. This test never re-derives Rmin, the bank limit or the
		// safety factor, and never applies a threshold of its own to let the planner through.
		using FCC = FormationControlV2::SlotCurvatureClassV2;
		EFramePop Pop = EFramePop::Unclassified;
		if (OpenPhase == EOpenLoopPhase::Initialization || OpenPhase == EOpenLoopPhase::TakeoffOrGround) {
			Pop = EFramePop::Initial;
		} else if (OpenPhase == EOpenLoopPhase::Invalid || !D.bTurnBoundValid) {
			Pop = EFramePop::InvalidInput;                             // input-contract failures
		} else {
			switch ((FCC)D.SlotCurvatureClass) {
			case FCC::Feasible:             Pop = EFramePop::Feasible;   break;
			case FCC::Infeasible:           Pop = EFramePop::Infeasible; break;
			case FCC::Boundary:             Pop = EFramePop::Boundary;   break;
			// No usable slot curvature but the straight assumption is on => effective kappa 0,
			// which is trivially inside any positive turn bound.
			case FCC::CurvatureUnavailable: Pop = EFramePop::Feasible;   break;
			}
		}
		switch (Pop) {
		case EFramePop::Initial:      St->PopInitial++;      break;
		case EFramePop::Feasible:     St->PopFeasible++;     break;
		case EFramePop::Infeasible:   St->PopInfeasible++;   break;
		case EFramePop::Boundary:     St->PopBoundary++;     break;
		case EFramePop::InvalidInput: St->PopInvalidInput++; break;
		default:                      St->PopUnclassified++; break;
		}

		// curvature clamp check applies to EVERY valid frame, in every population: the planner must
		// never emit a path tighter than the bound it itself computed.
		if (PO.bValid && PO.bPathValid && D.bTurnBoundValid && D.RminM > 0) {
			if (FMath::Abs(PO.Path.SignedCurvaturePerM) > 1.0 / D.RminM + 1e-9) St->CurvClampViolations++;
		}
		// A "stale" output is any invalid frame still carrying path or speed content.
		const bool bStaleNow = !PO.bValid && (PO.bPathValid || PO.bTargetEasValid || PO.TargetEasMps != 0.0);

		if (Pop == EFramePop::Feasible) {
			if (PO.bValid) St->FeasValid++; else St->FeasInvalid++;
			if (PO.Failure == FPF::CandidateSelectionFailed) St->FeasCandidateFail++;
			if (PO.Failure == FPF::SlotCurvatureInfeasible) { St->FeasCurvInfeasible++; St->FalseReject++; }
			if (bStaleNow) St->FeasStale++;
			if (D.bNearFieldExactPoseDubinsCalled) St->FeasExactPoseDubins++;
			const int fmi = (int)PO.Mode; if (fmi >= 0 && fmi < 5) St->FeasModeFrames[fmi]++;
			if (PO.bValid) {
				if (!AirIsFin(PO.Path.Position.N) || !AirIsFin(PO.Path.Position.E) ||
				    !AirIsFin(PO.Path.UnitTangent.N) || !AirIsFin(PO.Path.SignedCurvaturePerM) ||
				    !AirIsFin(PO.TargetEasMps)) St->FeasNonFinite++;
				if (!PO.bPathValid || !PO.bTargetEasValid) St->FeasGenMismatch++;   // path/TargetEAS must agree
				if (St->FeasFirstAbsCross < 0) { St->FeasFirstAbsCross = FMath::Abs(D.CrossErrorM); St->FeasFirstAbsAlong = FMath::Abs(D.AlongErrorM); }
				St->FeasAbsCross.Add(FMath::Abs(D.CrossErrorM));
				St->FeasAbsAlong.Add(FMath::Abs(D.AlongErrorM));
			}
		} else if (Pop == EFramePop::Infeasible) {
			if (PO.bValid) St->InfFalseAccept++;
			if (PO.Failure == FPF::SlotCurvatureInfeasible) St->InfCurvInfeasible++;
			else if (!PO.bValid) St->InfOtherFailure++;
			if (PO.Failure == FPF::CandidateSelectionFailed) St->InfCandidateFail++;
			if (PO.bPathValid) St->InfPathValid++;
			if (PO.bTargetEasValid) St->InfTargetEasValid++;
			if (bStaleNow) St->InfStale++;
			if (GO.bCommandReady) St->InfGuidanceReady++;
			if (SO.bValid) St->InfStickReady++;
			// phase correspondence: leader course rate implied by the slot curvature it is flying
			const double leaderRateDps = FMath::RadiansToDegrees(FMath::Abs(D.SlotCurvaturePerM) * D.SlotGroundSpeedMps);
			const int band = leaderRateDps < 1.0 ? 0 : (leaderRateDps < 2.5 ? 1 : (leaderRateDps < 3.5 ? 2 : 3));
			St->InfByLeaderRate[band]++;
			St->InfRun++; St->InfLongestRun = FMath::Max(St->InfLongestRun, St->InfRun);
			St->RecoveryFrames = 0;   // start counting recovery on the next feasible/valid frame
		} else if (Pop == EFramePop::Boundary) {
			St->BndRun++; St->BndLongestRun = FMath::Max(St->BndLongestRun, St->BndRun);
			if (bStaleNow) St->BndStale++;
			if (PO.bValid && (!PO.bPathValid || !PO.bTargetEasValid)) St->BndContradiction++;
			if (!PO.bValid && (PO.bPathValid || PO.bTargetEasValid)) St->BndPartial++;
			if (PO.bValid && (!AirIsFin(PO.TargetEasMps) || !AirIsFin(PO.Path.SignedCurvaturePerM))) St->BndNonFinite++;
			const uint8 bm = (uint8)PO.Mode;
			if (St->BndLastMode != 255 && bm != St->BndLastMode) St->BndModeChanges++;
			St->BndLastMode = bm;
		}
		if (Pop != EFramePop::Infeasible) St->InfRun = 0;
		if (Pop != EFramePop::Boundary) St->BndRun = 0;
		// recovery latency: once feasibility returns, how long until the planner is valid again
		if (St->RecoveryFrames >= 0 && Pop != EFramePop::Infeasible) {
			if (PO.bValid) { St->MaxRecoveryFrames = FMath::Max(St->MaxRecoveryFrames, St->RecoveryFrames); St->RecoveryFrames = -1; }
			else St->RecoveryFrames++;
		}

		if (PO.bValid) {
			St->PlannerValid++;
			if (PO.Mode == FormationControlV2::PlannerMode::NearFieldSlotTrack) {
				St->NfTargetEasMin = FMath::Min(St->NfTargetEasMin, PO.TargetEasMps);
				St->NfTargetEasMax = FMath::Max(St->NfTargetEasMax, PO.TargetEasMps);
			}
			const double tn = PO.Path.UnitTangent.Norm();
			if (!AirIsFin(tn) || tn < 1e-6) St->ZeroTangentFrames++;
			St->AlongMin = FMath::Min(St->AlongMin, D.AlongErrorM); St->AlongMax = FMath::Max(St->AlongMax, D.AlongErrorM);
			St->CrossMin = FMath::Min(St->CrossMin, D.CrossErrorM); St->CrossMax = FMath::Max(St->CrossMax, D.CrossErrorM);
			if (St->InitialAbsAlong < 0) { St->InitialAbsAlong=FMath::Abs(D.AlongErrorM); St->InitialAbsCross=FMath::Abs(D.CrossErrorM); }
			St->BestAbsAlong=FMath::Min(St->BestAbsAlong,FMath::Abs(D.AlongErrorM));St->BestAbsCross=FMath::Min(St->BestAbsCross,FMath::Abs(D.CrossErrorM));
			St->CurvMin = FMath::Min(St->CurvMin, PO.Path.SignedCurvaturePerM); St->CurvMax = FMath::Max(St->CurvMax, PO.Path.SignedCurvaturePerM);
			St->TgtEasMin = FMath::Min(St->TgtEasMin, PO.TargetEasMps); St->TgtEasMax = FMath::Max(St->TgtEasMax, PO.TargetEasMps);
			if (Dto.Tecs.bTargetAltitudeValid) { St->TgtAltMin = FMath::Min(St->TgtAltMin, Dto.Tecs.TargetAltitudeAslM); St->TgtAltMax = FMath::Max(St->TgtAltMax, Dto.Tecs.TargetAltitudeAslM); }
			const uint8 m = (uint8)PO.Mode;
			if (St->LastMode != 255 && m != St->LastMode) St->ModeTransitions++;
			St->LastMode = m;
			St->ReplanCount = D.ReplanCount; St->ReplanReasonMask |= D.ReplanReasons;
			if (D.bUsingHeldPath) St->HeldPathFrames++;
			const int ti = (int)D.SelectedType; if (ti >= 0 && ti < 7) St->TypeCount[ti]++;
		} else {
			St->PlannerInvalid++;
			const int fi = (int)PO.Failure;
			if (fi >= 0 && fi < (int)FormationControlV2::kPlannerFailureCount) St->PlannerFailCount[fi]++;
			if (!SO.bValid) St->StickNotReadyOnPlannerInvalid++;
		}
		if (!GO.bCommandReady) { const int gi2 = (int)GO.FailureReason; if (gi2 >= 0 && gi2 < 16) St->GuidanceFailCount[gi2]++; }

		if (GO.bCommandReady) {
			St->GuidanceValid++;
			St->RollRefMin = FMath::Min(St->RollRefMin, GO.RollReferenceRad); St->RollRefMax = FMath::Max(St->RollRefMax, GO.RollReferenceRad);
			St->PitchRefMin = FMath::Min(St->PitchRefMin, GO.PitchReferenceRad); St->PitchRefMax = FMath::Max(St->PitchRefMax, GO.PitchReferenceRad);
			St->ThrRefMin = FMath::Min(St->ThrRefMin, GO.ThrottleReferenceNorm); St->ThrRefMax = FMath::Max(St->ThrRefMax, GO.ThrottleReferenceNorm);
			St->FfMin = FMath::Min(St->FfMin, GO.LateralAccelerationFeedforwardMps2); St->FfMax = FMath::Max(St->FfMax, GO.LateralAccelerationFeedforwardMps2);
			St->FbMin = FMath::Min(St->FbMin, GO.LateralAccelerationFeedbackMps2); St->FbMax = FMath::Max(St->FbMax, GO.LateralAccelerationFeedbackMps2);
			St->LatMin = FMath::Min(St->LatMin, GO.LateralAccelerationTotalMps2); St->LatMax = FMath::Max(St->LatMax, GO.LateralAccelerationTotalMps2);
			St->FeasMin = FMath::Min(St->FeasMin, GO.WindFeasibility); St->FeasMax = FMath::Max(St->FeasMax, GO.WindFeasibility);
			if (GO.UnderspeedRatio > 0.0) St->UnderspeedFrames++;
			if (GO.FastDescendRatio > 0.0) St->FastDescendFrames++;
			St->bAllFinite &= AirIsFin(GO.RollReferenceRad) && AirIsFin(GO.PitchReferenceRad) && AirIsFin(GO.ThrottleReferenceNorm) &&
			                  AirIsFin(GO.LateralAccelerationTotalMps2) && AirIsFin(GO.WindFeasibility);
		}

		if (SO.bValid) {
			St->StickValid++; St->InvalidRun = 0;
			if (SO.ResetGeneration != GO.ResetGeneration) St->bResetGenOk = false;
			St->bAllFinite &= AirIsFin(SO.AileronCmdNorm) && AirIsFin(SO.ElevatorCmdNorm) && AirIsFin(SO.RudderCmdNorm) && AirIsFin(SO.ThrottleCmdNorm);
			St->bAllInRange &= SO.AileronCmdNorm >= -1 && SO.AileronCmdNorm <= 1 && SO.ElevatorCmdNorm >= -1 && SO.ElevatorCmdNorm <= 1 &&
			                   SO.RudderCmdNorm >= -1 && SO.RudderCmdNorm <= 1 && SO.ThrottleCmdNorm >= 0 && SO.ThrottleCmdNorm <= 1;
			St->AilMin = FMath::Min(St->AilMin, SO.AileronCmdNorm); St->AilMax = FMath::Max(St->AilMax, SO.AileronCmdNorm);
			St->ElevMin = FMath::Min(St->ElevMin, SO.ElevatorCmdNorm); St->ElevMax = FMath::Max(St->ElevMax, SO.ElevatorCmdNorm);
			St->RudMin = FMath::Min(St->RudMin, SO.RudderCmdNorm); St->RudMax = FMath::Max(St->RudMax, SO.RudderCmdNorm);
			St->ThrCmdMin = FMath::Min(St->ThrCmdMin, SO.ThrottleCmdNorm); St->ThrCmdMax = FMath::Max(St->ThrCmdMax, SO.ThrottleCmdNorm);

			// derived saturation / slew flags (observation only)
			const bool sat = FMath::Abs(SO.AileronCmdNorm) >= 1.0 - 1e-9 || FMath::Abs(SO.ElevatorCmdNorm) >= 1.0 - 1e-9 ||
			                 SO.ThrottleCmdNorm <= St->StickConfig.ThrottleMin + 1e-9 || SO.ThrottleCmdNorm >= St->StickConfig.ThrottleMax - 1e-9;
			bool slewLimited = false;
			if (St->bPrevStick) {
				const double dA = FMath::Abs(SO.AileronCmdNorm - St->PrevStick.AileronCmdNorm);
				const double dE = FMath::Abs(SO.ElevatorCmdNorm - St->PrevStick.ElevatorCmdNorm);
				const double dT = FMath::Abs(SO.ThrottleCmdNorm - St->PrevStick.ThrottleCmdNorm);
				if (dA > St->StickConfig.AileronSlewPerS * kDtS + 1e-9 ||
				    dE > St->StickConfig.ElevatorSlewPerS * kDtS + 1e-9 ||
				    dT > St->StickConfig.ThrottleSlewPerS * kDtS + 1e-9) St->bSlewOk = false;
				slewLimited = dA >= St->StickConfig.AileronSlewPerS * kDtS - 1e-9 ||
				              dE >= St->StickConfig.ElevatorSlewPerS * kDtS - 1e-9 ||
				              dT >= St->StickConfig.ThrottleSlewPerS * kDtS - 1e-9;
			}
			if (sat) { St->SatFrames++; St->SatRun++; St->MaxSatRun = FMath::Max(St->MaxSatRun, St->SatRun); } else St->SatRun = 0;
			if (slewLimited) St->SlewFrames++;

			// sign contract (skip slew-limited / saturated frames where the command is still ramping)
			if (GO.bCommandReady && !slewLimited) {
				const double rollErr = FormationControlV2::WrapPi(GO.RollReferenceRad - FSt.Roll_rad);
				if (FMath::Abs(rollErr) > 0.02 && FMath::Abs(SO.AileronCmdNorm) < 1.0 - 1e-9) {
					St->RollSignChecks++;
					if (rollErr > 0) { if (SO.AileronCmdNorm > 0) St->PosRollErrPosAil++; else St->RollSignViolations++; }
					else { if (SO.AileronCmdNorm < 0) St->NegRollErrNegAil++; else St->RollSignViolations++; }
				}
				const double pitchErr = FormationControlV2::WrapPi(GO.PitchReferenceRad - FSt.Pitch_rad);
				if (FMath::Abs(pitchErr) > 0.02 && FMath::Abs(SO.ElevatorCmdNorm) < 1.0 - 1e-9) {
					St->PitchSignChecks++;
					if (pitchErr > 0) { if (SO.ElevatorCmdNorm < 0) St->PosPitchErrNegElev++; else St->PitchSignViolations++; }
					else { if (SO.ElevatorCmdNorm > 0) St->NegPitchErrPosElev++; else St->PitchSignViolations++; }
				}
			}
			St->PrevStick = SO; St->bPrevStick = true;
		} else {
			St->InvalidRun++; St->MaxInvalidRun = FMath::Max(St->MaxInvalidRun, St->InvalidRun);
			// invalid frame must be a fresh zero-neutral command (no stale)
			if (SO.AileronCmdNorm != 0.0 || SO.ElevatorCmdNorm != 0.0 || SO.RudderCmdNorm != 0.0 || SO.ThrottleCmdNorm != 0.0)
				St->bNoStale = false;
			St->bPrevStick = false; // next valid frame must re-slew from neutral
		}

		if (GO.bCommandReady && SO.bValid && OpenPhase == EOpenLoopPhase::TakeoffOrGround) {
			St->GroundValidOutputs++;
			St->GroundPitchRefMin = FMath::Min(St->GroundPitchRefMin, GO.PitchReferenceRad);
			St->GroundPitchRefMax = FMath::Max(St->GroundPitchRefMax, GO.PitchReferenceRad);
		} else if (GO.bCommandReady && SO.bValid && OpenPhase == EOpenLoopPhase::Airborne) {
			St->AirValidOutputs++;
			St->AirPitchRefMin = FMath::Min(St->AirPitchRefMin, GO.PitchReferenceRad);
			St->AirPitchRefMax = FMath::Max(St->AirPitchRefMax, GO.PitchReferenceRad);
			St->AirElevMin = FMath::Min(St->AirElevMin, SO.ElevatorCmdNorm);
			St->AirElevMax = FMath::Max(St->AirElevMax, SO.ElevatorCmdNorm);
			St->AirThrottleMin = FMath::Min(St->AirThrottleMin, SO.ThrottleCmdNorm);
			St->AirThrottleMax = FMath::Max(St->AirThrottleMax, SO.ThrottleCmdNorm);
			St->AirRollRefMin = FMath::Min(St->AirRollRefMin, GO.RollReferenceRad);
			St->AirRollRefMax = FMath::Max(St->AirRollRefMax, GO.RollReferenceRad);
			if (GO.PitchReferenceRad > 0.0) St->AirPitchRefPositive++;
			else if (GO.PitchReferenceRad < 0.0) St->AirPitchRefNegative++;
			if (SO.ElevatorCmdNorm > 0.0) St->AirElevPositive++;
			else if (SO.ElevatorCmdNorm < 0.0) St->AirElevNegative++;
			if (GO.RollReferenceRad > 0.0) St->AirRollPositive++;
			else if (GO.RollReferenceRad < 0.0) St->AirRollNegative++;
		}

		if (St->Csv.Num() < 20000) {
			// Existing columns 1..34 remain byte-order compatible; openLoopPhase is appended as column 35.
			// 35: Initialization=0, TakeoffOrGround=1, Airborne=2, PlannerRejected=3, Invalid=4.
			St->Csv.Add(FString::Printf(TEXT("%.4f,%.2f,%.3f,%.3f,%.2f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%.2f,%.2f,%.8f,%.2f,%.2f,%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%d,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d"),
				FS.SimTimeSec, FSt.AltitudeAsl_m, FSt.EquivalentAirspeed_mps, FSt.TrueAirspeed_mps, gs,
				FSt.Roll_rad, FSt.Pitch_rad, FS.BodyRollRatePRadps, FS.BodyPitchRateQRadps, FS.BodyYawRateRRadps,
				FS.AlphaRad, FS.BetaRad,
				(int)PO.Mode, D.AlongErrorM, D.CrossErrorM, PO.Path.SignedCurvaturePerM, PO.TargetEasMps,
				Dto.Tecs.bTargetAltitudeValid ? Dto.Tecs.TargetAltitudeAslM : 0.0, (int)D.SelectedType,
				GO.RollReferenceRad, GO.PitchReferenceRad, GO.ThrottleReferenceNorm,
				GO.LateralAccelerationFeedforwardMps2, GO.LateralAccelerationFeedbackMps2, GO.LateralAccelerationTotalMps2,
				GO.WindFeasibility, GO.UnderspeedRatio > 0 ? 1 : 0,
				SO.AileronCmdNorm, SO.ElevatorCmdNorm, SO.RudderCmdNorm, SO.ThrottleCmdNorm,
				SO.bValid ? 1 : 0, (int)SO.FailureReason, PO.bValid ? 1 : 0, OpenPhaseIndex));
		}

		const bool done = (FS.SimTimeSec - St->FirstSimTime) >= kMaxSimSeconds;
		if (done || (NowWall - St->FirstWall) >= kMaxWallSeconds) return Finalize();
		return false;
	}

private:
	bool Continue(double NowWall)
	{
		if ((NowWall - St->FirstWall) >= kMaxWallSeconds) return Finalize();
		return false;
	}

	bool Finalize()
	{
		FFileHelper::SaveStringArrayToFile(St->Csv, TEXT("/tmp/mumt_airborne_shadow.csv"));
		FFileHelper::SaveStringArrayToFile(St->Csv, *(FPaths::ProjectSavedDir() / TEXT("Diagnostics/MumtAirborneShadow.csv")));

		const double span = FMath::Max(1e-6, St->MaxSimTime - St->FirstSimTime);
		const double satRatio = St->StickValid > 0 ? (double)St->SatFrames / (double)St->StickValid : 0.0;
		int32 OpenPhaseTotal = 0;
		for (int32 k = 0; k < kOpenLoopPhaseCount; ++k) OpenPhaseTotal += St->OpenPhaseCount[k];

		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] map=%s leader=%s follower=%s aircraftLabels=[%s] actorsWithComp=%d totalComponents=%d maxCompsPerActor=%d"),
			kAirMap, *St->LeaderName, *St->FollowerName, *St->AllAircraft, St->ActorsWithComp, St->TotalComponents, St->MaxCompsPerActor);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] samples=%d simTime=[%.2f..%.2f] airborneSamples(WOW=false)=%d gameThread=%d monotonic=%d"),
			St->Samples, St->FirstSimTime, St->MaxSimTime, St->AirborneSamples, St->bGameThread, St->bSimMonotonic);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] OPEN_LOOP_PHASE populations total=%d initialization=%d takeoffOrGround=%d airborne=%d plannerRejected=%d invalid=%d unclassified=%d transitions=%d wowUnavailable=%d"),
			St->Samples, St->OpenPhaseCount[0], St->OpenPhaseCount[1], St->OpenPhaseCount[2],
			St->OpenPhaseCount[3], St->OpenPhaseCount[4], St->Samples - OpenPhaseTotal,
			St->OpenPhaseTransitions, St->WowUnavailableFrames);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] OPEN_LOOP_PHASE ranges initialization=[%.2f..%.2f] takeoffOrGround=[%.2f..%.2f] airborne=[%.2f..%.2f] plannerRejected=[%.2f..%.2f] invalid=[%.2f..%.2f] source=JSBSim_FGear_HasWeightOnWheel"),
			St->OpenPhaseStartS[0], St->OpenPhaseEndS[0], St->OpenPhaseStartS[1], St->OpenPhaseEndS[1],
			St->OpenPhaseStartS[2], St->OpenPhaseEndS[2], St->OpenPhaseStartS[3], St->OpenPhaseEndS[3],
			St->OpenPhaseStartS[4], St->OpenPhaseEndS[4]);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] state alt=[%.1f..%.1f] eas=[%.1f..%.1f] tas=[%.1f..%.1f] gs=[%.1f..%.1f] climb=[%.2f..%.2f] roll=[%.3f..%.3f] pitch=[%.3f..%.3f]"),
			St->AltMin, St->AltMax, St->EasMin, St->EasMax, St->TasMin, St->TasMax, St->GsMin, St->GsMax, St->ClimbMin, St->ClimbMax, St->RollMin, St->RollMax, St->PitchMin, St->PitchMax);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] rates p=[%.3f..%.3f] q=[%.3f..%.3f] r=[%.3f..%.3f] alpha=[%.4f..%.4f] beta=[%.4f..%.4f] maxWind=%.3f"),
			St->PMin, St->PMax, St->QMin, St->QMax, St->RMin, St->RMax, St->AlphaMin, St->AlphaMax, St->BetaMin, St->BetaMax, St->MaxAbsWind);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] planner valid=%d invalid=%d along=[%.1f..%.1f] cross=[%.1f..%.1f] curv=[%.6f..%.6f] tgtEas=[%.1f..%.1f] tgtAlt=[%.1f..%.1f] modeTransitions=%d replans=%d replanMask=0x%x heldPathFrames=%d zeroTangent=%d types(LSL/RSR/LSR/RSL/RLR/LRL/INV)=%d/%d/%d/%d/%d/%d/%d"),
			St->PlannerValid, St->PlannerInvalid, St->AlongMin, St->AlongMax, St->CrossMin, St->CrossMax, St->CurvMin, St->CurvMax,
			St->TgtEasMin, St->TgtEasMax, St->TgtAltMin, St->TgtAltMax, St->ModeTransitions, St->ReplanCount, St->ReplanReasonMask, St->HeldPathFrames, St->ZeroTangentFrames,
			St->TypeCount[0], St->TypeCount[1], St->TypeCount[2], St->TypeCount[3], St->TypeCount[4], St->TypeCount[5], St->TypeCount[6]);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] nearfield modeFrames(Rejoin/NearFieldSlotTrack/CaptureEntry/ClosureTaper/SlotHold)=%d/%d/%d/%d/%d candidateEvaluations=%d maxConsecutiveCandidateFailures=%d lastRejectMask=0x%02x slotTrackFrames=%d nfTargetEas=[%.1f..%.1f] evalPerValidUpdateHz=%.2f"),
			St->ModeFrames[0], St->ModeFrames[1], St->ModeFrames[2], St->ModeFrames[3], St->ModeFrames[4],
			St->CandidateEvaluations, St->MaxConsecutiveCandidateFailures, St->CandidateRejectMask, St->SlotTrackFrames,
			St->NfTargetEasMin, St->NfTargetEasMax, (double)St->CandidateEvaluations / span);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] guidance valid=%d roll=[%.5f..%.5f] pitch=[%.5f..%.5f] throttle=[%.4f..%.4f] ff=[%.3f..%.3f] fb=[%.3f..%.3f] total=[%.3f..%.3f] feasibility=[%.4f..%.4f] underspeedFrames=%d fastDescendFrames=%d"),
			St->GuidanceValid, St->RollRefMin, St->RollRefMax, St->PitchRefMin, St->PitchRefMax, St->ThrRefMin, St->ThrRefMax,
			St->FfMin, St->FfMax, St->FbMin, St->FbMax, St->LatMin, St->LatMax, St->FeasMin, St->FeasMax, St->UnderspeedFrames, St->FastDescendFrames);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] stick valid=%d leaderStickValid=%d aileron=[%.6f..%.6f] elevator=[%.6f..%.6f] rudder=[%.6f..%.6f] throttle=[%.6f..%.6f] satFrames=%d satRatio=%.4f maxSatRunS=%.2f slewFrames=%d"),
			St->StickValid, St->LeaderStickValid, St->AilMin, St->AilMax, St->ElevMin, St->ElevMax, St->RudMin, St->RudMax, St->ThrCmdMin, St->ThrCmdMax,
			St->SatFrames, satRatio, St->MaxSatRun * kDtS, St->SlewFrames);
		// Printed straight off the enum sentinel: adding a PlannerFailure can never again leave a
		// failure out of the report (which is exactly how SlotCurvatureInfeasible stayed invisible).
		static_assert(FormationControlV2::kPlannerFailureCount == 18,
			"PlannerFailure changed: update kPlannerFailureNames below to match the enum.");
		static const TCHAR *kPlannerFailureNames[FormationControlV2::kPlannerFailureCount] = {
			TEXT("None"), TEXT("Paused"), TEXT("AbnormalDt"), TEXT("CriticalInput"), TEXT("TurnBound"),
			TEXT("PredRefresh"), TEXT("HeldExpired"), TEXT("Projection"), TEXT("Candidate"), TEXT("RejoinTO"),
			TEXT("CaptureTO"), TEXT("TaperTO"), TEXT("Wind"), TEXT("Ratio"), TEXT("Decel"),
			TEXT("SlotPathInvalid"), TEXT("SlotCurvUnavail"), TEXT("SlotCurvInfeasible")
		};
		{
			FString FailLine;
			for (int32 k = 0; k < (int32)FormationControlV2::kPlannerFailureCount; ++k)
				FailLine += FString::Printf(TEXT("%s=%d "), kPlannerFailureNames[k], St->PlannerFailCount[k]);
			UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] plannerFailures %s"), *FailLine);
		}
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] guidanceFailures None/Disabled/Shadow/Paused/ResetFrame/InvFollower/InvDto/InvWind/InvTime/OriginMis/ResetMis/ZeroTan/NonFinIn/NonFinOut/InvalidConfig=%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d"),
			St->GuidanceFailCount[0], St->GuidanceFailCount[1], St->GuidanceFailCount[2], St->GuidanceFailCount[3], St->GuidanceFailCount[4],
			St->GuidanceFailCount[5], St->GuidanceFailCount[6], St->GuidanceFailCount[7], St->GuidanceFailCount[8], St->GuidanceFailCount[9],
			St->GuidanceFailCount[10], St->GuidanceFailCount[11], St->GuidanceFailCount[12], St->GuidanceFailCount[13], St->GuidanceFailCount[14]);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] OPEN_LOOP_DIAGNOSTIC instantaneous_error_vs_final_pi_command rollChecks=%d rollViolations=%d (+err/+ail=%d, -err/-ail=%d) pitchChecks=%d pitchMismatches=%d (+err/-elev=%d, -err/+elev=%d)"),
			St->RollSignChecks, St->RollSignViolations, St->PosRollErrPosAil, St->NegRollErrNegAil,
			St->PitchSignChecks, St->PitchSignViolations, St->PosPitchErrNegElev, St->NegPitchErrPosElev);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] OPEN_LOOP_COVERAGE ground pitchRef=[%.6f..%.6f] valid=%d | airborne valid=%d pitchRef=[%.6f..%.6f] pos/neg=%d/%d elevator=[%.6f..%.6f] pos/neg=%d/%d throttle=[%.6f..%.6f] rollRef=[%.6f..%.6f] pos/neg=%d/%d"),
			St->GroundPitchRefMin, St->GroundPitchRefMax, St->GroundValidOutputs,
			St->AirValidOutputs, St->AirPitchRefMin, St->AirPitchRefMax, St->AirPitchRefPositive, St->AirPitchRefNegative,
			St->AirElevMin, St->AirElevMax, St->AirElevPositive, St->AirElevNegative,
			St->AirThrottleMin, St->AirThrottleMax, St->AirRollRefMin, St->AirRollRefMax,
			St->AirRollPositive, St->AirRollNegative);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] OPEN_LOOP_DIAGNOSTIC saturation frames=%d ratio=%.4f maxRunS=%.2f; recovery belongs to plant-in-loop"),
			St->SatFrames, satRatio, St->MaxSatRun * kDtS);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] HARD_INVARIANTS finite=%d inRange=%d slewOk=%d noStale=%d resetGenOk=%d plannerInvalid->stickNotReady=%d/%d partialGuidanceStick=%d longestInvalidS=%.2f validUpdateHz=%.1f commandWrites=0 fcsCmdNormWrites=0 activeConnections=0 newComponents=0 newFdm=0"),
			St->bAllFinite, St->bAllInRange, St->bSlewOk, St->bNoStale, St->bResetGenOk,
			St->StickNotReadyOnPlannerInvalid, St->PlannerInvalid, St->PartialGuidanceStickFrames,
			St->MaxInvalidRun * kDtS, (double)St->StickValid / span);

		// ---- frame-population accounting (shared authoritative contract) ----
		const int32 PopTotal = St->PopInitial + St->PopFeasible + St->PopInfeasible + St->PopBoundary + St->PopInvalidInput;
		// settling window: mean |cross| / |along| over the last kSettlingWindowFraction of feasible frames
		auto TailMean = [](const TArray<double> &V) -> double {
			if (V.Num() == 0) return -1.0;
			const int32 N = FMath::Max(1, (int32)(V.Num() * kSettlingWindowFraction));
			double s = 0.0; for (int32 k = V.Num() - N; k < V.Num(); ++k) s += V[k];
			return s / N;
		};
		auto HeadMean = [](const TArray<double> &V) -> double {
			if (V.Num() == 0) return -1.0;
			const int32 N = FMath::Max(1, (int32)(V.Num() * kSettlingWindowFraction));
			double s = 0.0; for (int32 k = 0; k < N; ++k) s += V[k];
			return s / N;
		};
		const double CrossHead = HeadMean(St->FeasAbsCross), CrossTail = TailMean(St->FeasAbsCross);
		const double AlongHead = HeadMean(St->FeasAbsAlong), AlongTail = TailMean(St->FeasAbsAlong);
		const double FeasAvail = St->PopFeasible > 0 ? (double)St->FeasValid / (double)St->PopFeasible : 0.0;

		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] populations total=%d = initial=%d + feasible=%d + infeasible=%d + boundary=%d + invalidInput=%d | unclassified=%d accountingOk=%d"),
			St->Samples, St->PopInitial, St->PopFeasible, St->PopInfeasible, St->PopBoundary, St->PopInvalidInput,
			St->PopUnclassified, PopTotal == St->Samples);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] feasible valid=%d invalid=%d availability=%.5f candidateFail=%d curvInfeasible(falseReject)=%d stale=%d nonFinite=%d genMismatch=%d exactPoseDubins=%d modeFrames=%d/%d/%d/%d/%d cross=%.1f->%.1f along=%.1f->%.1f"),
			St->FeasValid, St->FeasInvalid, FeasAvail, St->FeasCandidateFail, St->FeasCurvInfeasible,
			St->FeasStale, St->FeasNonFinite, St->FeasGenMismatch, St->FeasExactPoseDubins,
			St->FeasModeFrames[0], St->FeasModeFrames[1], St->FeasModeFrames[2], St->FeasModeFrames[3], St->FeasModeFrames[4],
			CrossHead, CrossTail, AlongHead, AlongTail);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] infeasible frames=%d slotCurvInfeasible=%d otherFailure(mismatch)=%d falseAccept=%d candidateFail=%d pathValid=%d tgtEasValid=%d stale=%d guidanceReady=%d stickReady=%d longestRejectionS=%.2f maxRecoveryS=%.2f leaderRateBands(<1/1-2.5/2.5-3.5/>3.5 degps)=%d/%d/%d/%d"),
			St->PopInfeasible, St->InfCurvInfeasible, St->InfOtherFailure, St->InfFalseAccept, St->InfCandidateFail,
			St->InfPathValid, St->InfTargetEasValid, St->InfStale, St->InfGuidanceReady, St->InfStickReady,
			St->InfLongestRun * kDtS, St->MaxRecoveryFrames * kDtS,
			St->InfByLeaderRate[0], St->InfByLeaderRate[1], St->InfByLeaderRate[2], St->InfByLeaderRate[3]);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] boundary frames=%d longestRunS=%.2f stale=%d nonFinite=%d partial=%d contradiction=%d modeChanges=%d | curvatureClampViolations=%d"),
			St->PopBoundary, St->BndLongestRun * kDtS, St->BndStale, St->BndNonFinite, St->BndPartial,
			St->BndContradiction, St->BndModeChanges, St->CurvClampViolations);

		// ---- assertions ----
		Test->TestTrue(TEXT("leader and follower JSBSim components found (1 per actor)"), St->bDiscovered && St->MaxCompsPerActor == 1);
		// Accounting is a hard contract for every mode: no frame may be silently dropped.
		Test->TestTrue(TEXT("frame accounting closes (no unclassified frames)"),
			St->PopUnclassified == 0 && PopTotal == St->Samples);
		Test->TestTrue(TEXT("open-loop phase accounting closes"), OpenPhaseTotal == St->Samples);
		Test->TestTrue(TEXT("JSBSim WOW phase source available on every sample"), St->WowUnavailableFrames == 0);
		Test->TestTrue(TEXT("authoritative JSBSim WOW observes airborne phase"), St->AirborneSamples > 100);
		Test->TestTrue(TEXT("all valid shadow outputs finite"), St->bAllFinite);
		Test->TestTrue(TEXT("stick commands within [-1,1] / throttle [0,1]"), St->bAllInRange);
		Test->TestTrue(TEXT("no configured slew-limit violation"), St->bSlewOk);
		Test->TestTrue(TEXT("no stale command on invalid frame"), St->bNoStale);
		Test->TestTrue(TEXT("reset generation consistent guidance->stick"), St->bResetGenOk);
		Test->TestTrue(TEXT("no zero path tangent"), St->ZeroTangentFrames == 0);
		Test->TestTrue(TEXT("planner invalid => stick not command-ready"), St->StickNotReadyOnPlannerInvalid == St->PlannerInvalid);
		Test->TestTrue(TEXT("guidance/stick command-ready contract has no partial output"), St->PartialGuidanceStickFrames == 0);
		Test->TestTrue(TEXT("all shadow calls on game thread"), St->bGameThread);
		Test->TestTrue(TEXT("simulation time monotonic"), St->bSimMonotonic);
		Test->TestTrue(TEXT("roll sign contract holds (+err -> +aileron)"), St->RollSignChecks > 0 && St->RollSignViolations == 0);
		Test->TestTrue(TEXT("guidance and stick produced valid airborne updates"), St->GuidanceValid > 100 && St->StickValid > 100);
		Test->TestTrue(TEXT("per-aircraft shadow instances independent (leader chain ran)"), St->LeaderStickValid > 0);
		using PM = FormationControlV2::PlannerMode;
		using PF = FormationControlV2::PlannerFailure;

		if (Mode == EAcceptance::PlannerFeasible) {
			// Graded ONLY on the feasible population. Infeasible frames are still counted and logged
			// above (and are graded by the EnvelopeRejection acceptance) — never deleted or hidden.
			Test->TestTrue(TEXT("feasible sample count sufficient"), St->PopFeasible >= 1000);
			Test->TestTrue(TEXT("feasible planner availability >= 99.9%"), FeasAvail >= 0.999);
			Test->TestTrue(TEXT("feasible CandidateSelectionFailed absent"), St->FeasCandidateFail == 0);
			Test->TestTrue(TEXT("feasible SlotCurvatureInfeasible absent (no false reject)"), St->FeasCurvInfeasible == 0);
			Test->TestTrue(TEXT("no exact-pose Dubins call in near field"), St->FeasExactPoseDubins == 0);
			Test->TestTrue(TEXT("feasible outputs finite"), St->FeasNonFinite == 0);
			Test->TestTrue(TEXT("feasible stale output absent"), St->FeasStale == 0);
			Test->TestTrue(TEXT("feasible path/TargetEAS validity agree"), St->FeasGenMismatch == 0);
			Test->TestTrue(TEXT("curvature never exceeds the planner's own turn bound"), St->CurvClampViolations == 0);
			Test->TestTrue(TEXT("NearFieldSlotTrack observed"), St->FeasModeFrames[(int)PM::NearFieldSlotTrack] > 0);
			Test->TestTrue(TEXT("ClosureTaper or SlotHold observed"),
				St->FeasModeFrames[(int)PM::ClosureTaper] + St->FeasModeFrames[(int)PM::SlotHold] > 0);
			Test->TestTrue(TEXT("cross error improves over the settling window"),
				St->FeasAbsCross.Num() > 0 && CrossTail <= CrossHead);
			Test->TestTrue(TEXT("along error improves or tapers"),
				(St->FeasAbsAlong.Num() > 0 && AlongTail <= AlongHead) || St->FeasModeFrames[(int)PM::ClosureTaper] > 0);
		} else if (Mode == EAcceptance::EnvelopeRejection) {
			// Graded ONLY on the infeasible population: the planner must refuse a slot curvature it
			// cannot fly, and must refuse it CLEANLY — no stale output, no partial path, nothing
			// downstream ever becomes command-ready, and it must not latch.
			Test->TestTrue(TEXT("infeasible sample count sufficient"), St->PopInfeasible >= 500);
			Test->TestTrue(TEXT("every infeasible frame reports SlotCurvatureInfeasible"),
				St->InfCurvInfeasible == St->PopInfeasible);
			Test->TestTrue(TEXT("no other PlannerFailure on infeasible frames"), St->InfOtherFailure == 0);
			Test->TestTrue(TEXT("no false accept (infeasible frame produced valid output)"), St->InfFalseAccept == 0);
			Test->TestTrue(TEXT("CandidateSelectionFailed absent on infeasible frames"), St->InfCandidateFail == 0);
			Test->TestTrue(TEXT("rejected frames carry no path"), St->InfPathValid == 0);
			Test->TestTrue(TEXT("rejected frames carry no TargetEAS"), St->InfTargetEasValid == 0);
			Test->TestTrue(TEXT("rejected frames emit no stale output"), St->InfStale == 0);
			Test->TestTrue(TEXT("guidance never command-ready on a rejected frame"), St->InfGuidanceReady == 0);
			Test->TestTrue(TEXT("stick never command-ready on a rejected frame"), St->InfStickReady == 0);
			Test->TestTrue(TEXT("no curvature clamp anywhere"), St->CurvClampViolations == 0);
			Test->TestTrue(TEXT("no false reject in the feasible population"), St->FalseReject == 0);
			// Phase correspondence: rejection must coincide with a genuinely aggressive leader turn,
			// never with straight flight. (Turn3 = 3 deg/s, Turn4Climb = 4 deg/s.)
			Test->TestTrue(TEXT("rejection never occurs in straight leader flight"), St->InfByLeaderRate[0] == 0);
			Test->TestTrue(TEXT("rejection concentrated in the scripted 3/4 deg/s turns"),
				St->InfByLeaderRate[2] + St->InfByLeaderRate[3] > 0);
			// No permanent latch: once feasibility returns, the planner must be valid again quickly.
			Test->TestTrue(TEXT("planner recovers within bounded dwell (no latch)"),
				St->MaxRecoveryFrames <= kMaxRecoveryFrames);
			Test->TestTrue(TEXT("feasible frames still exist after rejection (not latched off)"), St->FeasValid > 0);
		} else {
			// Open-loop acceptance: V2 commands are not applied to this aircraft. Grade only observable
			// DTO/invariant and excitation contracts; PI-memory sign and saturation recovery are logged
			// above and belong to deterministic adapter / plant-in-loop tests respectively.
			Test->TestTrue(TEXT("airborne command-ready output population sufficient"), St->AirValidOutputs > 100);
			Test->TestTrue(TEXT("airborne PitchReference non-constant"),
				St->AirPitchRefMax - St->AirPitchRefMin > 1e-4);
			Test->TestTrue(TEXT("airborne throttle finite and in configured range"),
				AirIsFin(St->AirThrottleMin) && AirIsFin(St->AirThrottleMax) &&
				St->AirThrottleMin >= St->StickConfig.ThrottleMin && St->AirThrottleMax <= St->StickConfig.ThrottleMax);
		}
		return true;
	}

	FAutomationTestBase *Test;
	TSharedPtr<FAirState> St;
	EAcceptance Mode{EAcceptance::FullControl};
};

// All three acceptances observe the identical shadow run and log the identical populations. They
// differ only in which population they grade — the frames are never deleted or hidden.
static void RunAirborneShadow(FAutomationTestBase *T, EAcceptance InMode)
{
	// Pre-existing, unrelated Blueprint Construction Script error in F16_UAV / M_F16
	// ("Set bUseAttachParentBound" -> array-get on None). Content is out of scope and not fixed
	// here; matched narrowly so it cannot hide any V2 guidance/stick error.
	T->AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);

	TSharedPtr<FAirState> State = MakeShared<FAirState>();
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kAirMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtAirborneSampleCommand(T, State, InMode));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtAirborneNearFieldPlannerShadowTest, "MUMT.ControlV2.AirborneNearFieldPlannerShadow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtAirborneNearFieldPlannerShadowTest::RunTest(const FString &)
{ RunAirborneShadow(this, EAcceptance::PlannerFeasible); return true; }

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtAirborneNearFieldEnvelopeRejectionShadowTest, "MUMT.ControlV2.AirborneNearFieldEnvelopeRejectionShadow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtAirborneNearFieldEnvelopeRejectionShadowTest::RunTest(const FString &)
{ RunAirborneShadow(this, EAcceptance::EnvelopeRejection); return true; }

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtAirborneFullControlShadowTest, "MUMT.ControlV2.AirborneFullControlShadow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtAirborneFullControlShadowTest::RunTest(const FString &)
{ RunAirborneShadow(this, EAcceptance::FullControl); return true; }

#endif // WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
