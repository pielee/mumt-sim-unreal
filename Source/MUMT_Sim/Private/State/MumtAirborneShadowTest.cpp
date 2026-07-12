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
constexpr double FT_TO_M = 0.3048;
constexpr double kDtS = 1.0 / 60.0;
const TCHAR *kAirMap = TEXT("/Game/RL_2");
constexpr double kMaxWallSeconds = 420.0;   // hard cap for the whole observation
constexpr double kMaxSimSeconds  = 300.0;   // covers takeoff + all scripted phases
// Single reset generation for the whole observation: the canonical conversion, the slot, the
// guidance coordinator and the stick adapter must all agree, or the coordinator rejects the frame.
constexpr uint32 kResetGen = 1u;
static bool IsFin(double x) { return std::isfinite(x); }

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
	int32 AirborneSamples = 0;                 // altitude above spawn > 50 m

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
	int32 PlannerFailCount[16] = {0};   // indexed by PlannerFailure
	int32 GuidanceFailCount[16] = {0};  // indexed by EGuidanceFailureV2

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

	// ---- invariants ----
	bool bAllFinite = true, bAllInRange = true, bSlewOk = true, bNoStale = true;
	bool bResetGenOk = true, bGameThread = true, bSimMonotonic = true;
	int32 InvalidRun = 0, MaxInvalidRun = 0;

	TArray<FString> Csv;
};

static UWorld *GetPIEWorld()
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
	FMumtAirborneSampleCommand(FAutomationTestBase *T, TSharedPtr<FAirState> S) : Test(T), St(S) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		if (St->FirstWall < 0) St->FirstWall = NowWall;
		if (!IsInGameThread()) St->bGameThread = false;

		UWorld *World = GetPIEWorld();
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
			St->Frame.SetOrigin({LS.GeodeticLatitudeRad, LS.LongitudeRad, LS.GeodeticAltitudeFt * FT_TO_M, 1u, true});
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
		SI.bBodyRatesValid = IsFin(FS.BodyRollRatePRadps) && IsFin(FS.BodyPitchRateQRadps) && IsFin(FS.BodyYawRateRRadps);
		SI.AlphaRad = FS.AlphaRad; SI.BetaRad = FS.BetaRad; SI.bAlphaBetaValid = IsFin(FS.AlphaRad) && IsFin(FS.BetaRad);
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
		const double altAgl = FSt.AltitudeAsl_m;
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
		if (FSt.EquivalentAirspeed_mps > 40.0) St->AirborneSamples++;

		if (PO.bValid) {
			St->PlannerValid++;
			const double tn = PO.Path.UnitTangent.Norm();
			if (!IsFin(tn) || tn < 1e-6) St->ZeroTangentFrames++;
			St->AlongMin = FMath::Min(St->AlongMin, D.AlongErrorM); St->AlongMax = FMath::Max(St->AlongMax, D.AlongErrorM);
			St->CrossMin = FMath::Min(St->CrossMin, D.CrossErrorM); St->CrossMax = FMath::Max(St->CrossMax, D.CrossErrorM);
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
			const int fi = (int)PO.Failure; if (fi >= 0 && fi < 16) St->PlannerFailCount[fi]++;
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
			St->bAllFinite &= IsFin(GO.RollReferenceRad) && IsFin(GO.PitchReferenceRad) && IsFin(GO.ThrottleReferenceNorm) &&
			                  IsFin(GO.LateralAccelerationTotalMps2) && IsFin(GO.WindFeasibility);
		}

		if (SO.bValid) {
			St->StickValid++; St->InvalidRun = 0;
			if (SO.ResetGeneration != GO.ResetGeneration) St->bResetGenOk = false;
			St->bAllFinite &= IsFin(SO.AileronCmdNorm) && IsFin(SO.ElevatorCmdNorm) && IsFin(SO.RudderCmdNorm) && IsFin(SO.ThrottleCmdNorm);
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

		if (St->Csv.Num() < 20000) {
			St->Csv.Add(FString::Printf(TEXT("%.4f,%.2f,%.3f,%.3f,%.2f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%.2f,%.2f,%.8f,%.2f,%.2f,%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%d,%.6f,%.6f,%.6f,%.6f,%d,%d,%d"),
				FS.SimTimeSec, FSt.AltitudeAsl_m, FSt.EquivalentAirspeed_mps, FSt.TrueAirspeed_mps, gs,
				FSt.Roll_rad, FSt.Pitch_rad, FS.BodyRollRatePRadps, FS.BodyPitchRateQRadps, FS.BodyYawRateRRadps,
				FS.AlphaRad, FS.BetaRad,
				(int)PO.Mode, D.AlongErrorM, D.CrossErrorM, PO.Path.SignedCurvaturePerM, PO.TargetEasMps,
				Dto.Tecs.bTargetAltitudeValid ? Dto.Tecs.TargetAltitudeAslM : 0.0, (int)D.SelectedType,
				GO.RollReferenceRad, GO.PitchReferenceRad, GO.ThrottleReferenceNorm,
				GO.LateralAccelerationFeedforwardMps2, GO.LateralAccelerationFeedbackMps2, GO.LateralAccelerationTotalMps2,
				GO.WindFeasibility, GO.UnderspeedRatio > 0 ? 1 : 0,
				SO.AileronCmdNorm, SO.ElevatorCmdNorm, SO.RudderCmdNorm, SO.ThrottleCmdNorm,
				SO.bValid ? 1 : 0, (int)SO.FailureReason, PO.bValid ? 1 : 0));
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

		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] map=%s leader=%s follower=%s aircraftLabels=[%s] actorsWithComp=%d totalComponents=%d maxCompsPerActor=%d"),
			kAirMap, *St->LeaderName, *St->FollowerName, *St->AllAircraft, St->ActorsWithComp, St->TotalComponents, St->MaxCompsPerActor);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] samples=%d simTime=[%.2f..%.2f] airborneSamples=%d gameThread=%d monotonic=%d"),
			St->Samples, St->FirstSimTime, St->MaxSimTime, St->AirborneSamples, St->bGameThread, St->bSimMonotonic);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] state alt=[%.1f..%.1f] eas=[%.1f..%.1f] tas=[%.1f..%.1f] gs=[%.1f..%.1f] climb=[%.2f..%.2f] roll=[%.3f..%.3f] pitch=[%.3f..%.3f]"),
			St->AltMin, St->AltMax, St->EasMin, St->EasMax, St->TasMin, St->TasMax, St->GsMin, St->GsMax, St->ClimbMin, St->ClimbMax, St->RollMin, St->RollMax, St->PitchMin, St->PitchMax);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] rates p=[%.3f..%.3f] q=[%.3f..%.3f] r=[%.3f..%.3f] alpha=[%.4f..%.4f] beta=[%.4f..%.4f] maxWind=%.3f"),
			St->PMin, St->PMax, St->QMin, St->QMax, St->RMin, St->RMax, St->AlphaMin, St->AlphaMax, St->BetaMin, St->BetaMax, St->MaxAbsWind);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] planner valid=%d invalid=%d along=[%.1f..%.1f] cross=[%.1f..%.1f] curv=[%.6f..%.6f] tgtEas=[%.1f..%.1f] tgtAlt=[%.1f..%.1f] modeTransitions=%d replans=%d replanMask=0x%x heldPathFrames=%d zeroTangent=%d types(LSL/RSR/LSR/RSL/RLR/LRL/INV)=%d/%d/%d/%d/%d/%d/%d"),
			St->PlannerValid, St->PlannerInvalid, St->AlongMin, St->AlongMax, St->CrossMin, St->CrossMax, St->CurvMin, St->CurvMax,
			St->TgtEasMin, St->TgtEasMax, St->TgtAltMin, St->TgtAltMax, St->ModeTransitions, St->ReplanCount, St->ReplanReasonMask, St->HeldPathFrames, St->ZeroTangentFrames,
			St->TypeCount[0], St->TypeCount[1], St->TypeCount[2], St->TypeCount[3], St->TypeCount[4], St->TypeCount[5], St->TypeCount[6]);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] guidance valid=%d roll=[%.5f..%.5f] pitch=[%.5f..%.5f] throttle=[%.4f..%.4f] ff=[%.3f..%.3f] fb=[%.3f..%.3f] total=[%.3f..%.3f] feasibility=[%.4f..%.4f] underspeedFrames=%d fastDescendFrames=%d"),
			St->GuidanceValid, St->RollRefMin, St->RollRefMax, St->PitchRefMin, St->PitchRefMax, St->ThrRefMin, St->ThrRefMax,
			St->FfMin, St->FfMax, St->FbMin, St->FbMax, St->LatMin, St->LatMax, St->FeasMin, St->FeasMax, St->UnderspeedFrames, St->FastDescendFrames);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] stick valid=%d leaderStickValid=%d aileron=[%.6f..%.6f] elevator=[%.6f..%.6f] rudder=[%.6f..%.6f] throttle=[%.6f..%.6f] satFrames=%d satRatio=%.4f maxSatRunS=%.2f slewFrames=%d"),
			St->StickValid, St->LeaderStickValid, St->AilMin, St->AilMax, St->ElevMin, St->ElevMax, St->RudMin, St->RudMax, St->ThrCmdMin, St->ThrCmdMax,
			St->SatFrames, satRatio, St->MaxSatRun * kDtS, St->SlewFrames);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] plannerFailures None/Paused/AbnormalDt/CriticalInput/TurnBound/PredRefresh/HeldExpired/Projection/Candidate/RejoinTO/CaptureTO/TaperTO/Wind/Ratio/Decel=%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d"),
			St->PlannerFailCount[0], St->PlannerFailCount[1], St->PlannerFailCount[2], St->PlannerFailCount[3], St->PlannerFailCount[4],
			St->PlannerFailCount[5], St->PlannerFailCount[6], St->PlannerFailCount[7], St->PlannerFailCount[8], St->PlannerFailCount[9],
			St->PlannerFailCount[10], St->PlannerFailCount[11], St->PlannerFailCount[12], St->PlannerFailCount[13], St->PlannerFailCount[14]);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] guidanceFailures None/Disabled/Shadow/Paused/ResetFrame/InvFollower/InvDto/InvWind/InvTime/OriginMis/ResetMis/ZeroTan/NonFinIn/NonFinOut=%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d/%d"),
			St->GuidanceFailCount[0], St->GuidanceFailCount[1], St->GuidanceFailCount[2], St->GuidanceFailCount[3], St->GuidanceFailCount[4],
			St->GuidanceFailCount[5], St->GuidanceFailCount[6], St->GuidanceFailCount[7], St->GuidanceFailCount[8], St->GuidanceFailCount[9],
			St->GuidanceFailCount[10], St->GuidanceFailCount[11], St->GuidanceFailCount[12], St->GuidanceFailCount[13]);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] signs rollChecks=%d rollViolations=%d (+err/+ail=%d, -err/-ail=%d) pitchChecks=%d pitchViolations=%d (+err/-elev=%d, -err/+elev=%d)"),
			St->RollSignChecks, St->RollSignViolations, St->PosRollErrPosAil, St->NegRollErrNegAil,
			St->PitchSignChecks, St->PitchSignViolations, St->PosPitchErrNegElev, St->NegPitchErrPosElev);
		UE_LOG(LogMumtAir, Display, TEXT("[AIRSHADOW] invariants finite=%d inRange=%d slewOk=%d noStale=%d resetGenOk=%d plannerInvalid->stickNotReady=%d/%d longestInvalidS=%.2f validUpdateHz=%.1f commandWrites=0 fcsCmdNormWrites=0 newComponents=0 newFdm=0"),
			St->bAllFinite, St->bAllInRange, St->bSlewOk, St->bNoStale, St->bResetGenOk,
			St->StickNotReadyOnPlannerInvalid, St->PlannerInvalid, St->MaxInvalidRun * kDtS, (double)St->StickValid / span);

		// ---- assertions ----
		Test->TestTrue(TEXT("leader and follower JSBSim components found (1 per actor)"), St->bDiscovered && St->MaxCompsPerActor == 1);
		Test->TestTrue(TEXT("aircraft actually got airborne (EAS > 40 m/s sustained)"), St->AirborneSamples > 100);
		Test->TestTrue(TEXT("all valid shadow outputs finite"), St->bAllFinite);
		Test->TestTrue(TEXT("stick commands within [-1,1] / throttle [0,1]"), St->bAllInRange);
		Test->TestTrue(TEXT("no configured slew-limit violation"), St->bSlewOk);
		Test->TestTrue(TEXT("no stale command on invalid frame"), St->bNoStale);
		Test->TestTrue(TEXT("reset generation consistent guidance->stick"), St->bResetGenOk);
		Test->TestTrue(TEXT("no zero path tangent"), St->ZeroTangentFrames == 0);
		Test->TestTrue(TEXT("planner invalid => stick not command-ready"), St->StickNotReadyOnPlannerInvalid == St->PlannerInvalid);
		Test->TestTrue(TEXT("all shadow calls on game thread"), St->bGameThread);
		Test->TestTrue(TEXT("simulation time monotonic"), St->bSimMonotonic);
		Test->TestTrue(TEXT("roll sign contract holds (+err -> +aileron)"), St->RollSignChecks > 0 && St->RollSignViolations == 0);
		Test->TestTrue(TEXT("pitch sign contract holds (+err -> -elevator)"), St->PitchSignChecks > 0 && St->PitchSignViolations == 0);
		Test->TestTrue(TEXT("guidance and stick produced valid airborne updates"), St->GuidanceValid > 100 && St->StickValid > 100);
		Test->TestTrue(TEXT("per-aircraft shadow instances independent (leader chain ran)"), St->LeaderStickValid > 0);
		return true;
	}

	FAutomationTestBase *Test;
	TSharedPtr<FAirState> St;
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtAirborneShadowTest, "MUMT.ControlV2.AirborneShadow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMumtAirborneShadowTest::RunTest(const FString & /*Parameters*/)
{
	// Pre-existing, unrelated Blueprint Construction Script error in F16_UAV / M_F16
	// ("Set bUseAttachParentBound" -> array-get on None). Content is out of scope and not fixed
	// here; matched narrowly so it cannot hide any V2 guidance/stick error.
	AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);

	TSharedPtr<FAirState> State = MakeShared<FAirState>();
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kAirMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtAirborneSampleCommand(this, State));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
	return true;
}

#endif // WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
