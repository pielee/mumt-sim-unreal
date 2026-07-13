// verify_tecs_f16_longitudinal_closed_loop_v2.cpp
//
// Longitudinal closed loop: the PRODUCTION guidance chain driving the ACTUAL JSBSim F-16 plant.
//
//   FormationGuidanceCoordinatorV2  (production, owns the real FPx4TecsAdapter + FPx4NpfgAdapter)
//     -> PitchReferenceRad / ThrottleReferenceNorm
//   F16StickAdapterV2               (production)
//     -> normalized aileron/elevator/rudder/throttle commands
//   actual f16.xml FCS  ->  actual surfaces  ->  actual aerodynamics  ->  actual JSBSim FDM
//     -> altitude / climb rate / EAS / TAS / pitch / body rates
//   -> back into FormationGuidanceCoordinatorV2
//
// This harness verifies the production TECS caller and F16StickAdapterV2 against the actual JSBSim
// F-16 plant in isolated longitudinal cases. It does not select or tune production TECS performance
// parameters.
//
// Any EAS or throttle values overridden by this host fixture are test-local initialization values
// and are not production recommendations.
//
// No TECS or stick control law is reimplemented here: every pitch/throttle reference comes out of
// the production coordinator, and every normalized command comes out of the production stick
// adapter. The harness only owns the FGFDMExec, the navigation frame, the setpoint schedule and the
// measurement.
//
// Lateral channel: the real NPFG inside the coordinator still runs every frame. It is held
// quiescent (straight path, follower on the path, tangent aligned with the ground course, zero
// curvature, zero wind) so the graded gates can be longitudinal only. Lateral closed-loop
// verification is a separate commit.
#include "FormationControlV2/F16StickAdapterV2.h"
#include "FormationControlV2/FormationGuidanceCoordinatorV2.h"

#include "FGFDMExec.h"
#include "initialization/FGTrim.h"
#include "models/FGAuxiliary.h"
#include "models/FGFCS.h"
#include "models/FGGroundReactions.h"
#include "models/FGMassBalance.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"
#include "models/propulsion/FGTank.h"
#include "simgear/misc/sg_path.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using FormationControlV2::EF16StickFailureV2;
using FormationControlV2::EGuidanceFailureV2;
using FormationControlV2::F16StickAdapterV2;
using FormationControlV2::FCanonicalNavigationStateV2;
using FormationControlV2::FF16StickCommandV2;
using FormationControlV2::FF16StickConfigV2;
using FormationControlV2::FF16StickInputV2;
using FormationControlV2::FFormationSlotStateV2;
using FormationControlV2::FGuidanceConfigV2;
using FormationControlV2::FGuidanceCoordinatorInputV2;
using FormationControlV2::FGuidanceCoordinatorOutputV2;
using FormationControlV2::FormationGuidanceCoordinatorV2;
using FormationControlV2::FPlannerV2OutputAdapterResult;
using FormationControlV2::Vec2;

// ---- fixed-step simulation, no wall clock, no sleep, no Unreal tick --------------------------
constexpr double kFdmDtS = 1.0 / 120.0;
constexpr double kControllerDtS = 1.0 / 60.0;
constexpr int kFdmStepsPerControlFrame = 2;

constexpr double kFtToM = 0.3048;
constexpr double kKnotToMps = 0.5144444444444444;
constexpr double kEarthRadiusM = 6378137.0;   // WGS-84 semi-major; harness-owned flat-earth frame
constexpr double kFiniteLimit = 1.0e12;

// ---- airborne initial condition, identical to the committed C1 stick plant harness -------------
constexpr double kInitialLatitudeDeg = 47.0;
constexpr double kInitialLongitudeDeg = -122.0;
constexpr double kInitialAltitudeFt = 10000.0;   // 3,048 m
constexpr double kInitialTasMps = 220.0;
constexpr double kInitialHeadingDeg = 90.0;      // due east
constexpr double kTank0Lb = 1500.0, kTank1Lb = 1500.0, kTank2Lb = 0.0, kTank3Lb = 0.0;  // 3,000 lb
constexpr double kExpectedFuelLb = 3000.0;

// ---- case schedule ----------------------------------------------------------------------------
// The settle window is long on purpose. Engaging the loop at the trim point excites a lightly
// damped energy oscillation (the production TECS keeps the PX4 default integrator and damping gains,
// which are zero), and its amplitude roughly halves every 10 s. The baseline window exists to
// measure numerical noise, natural trim drift and the controller's engagement bias -- NOT that
// transient. Measuring inside the transient would inflate every derived threshold to the point of
// swallowing the real step response, so the loop is allowed to settle first and the noise window is
// taken from the settled hold.
constexpr double kPrimeS = 0.5;        // hold the exact trim command while the adapters spin up
constexpr double kSettleEndS = 60.0;   // closed loop engaged; engagement transient decays here
constexpr double kStepTimeS = 70.0;    // measurement window is [kSettleEndS, kStepTimeS)
constexpr double kCaseDurationS = 130.0;
constexpr double kReversalTimeS = 100.0;
constexpr double kReversalDurationS = 160.0;
constexpr double kAltitudeStepM = 50.0;
constexpr double kEasStepMps = 10.0;

// ---- test-local TECS configuration -------------------------------------------------------------
//
// The TECS performance values used by this host fixture are derived from the committed
// single-condition solver characterization (verify_f16_vertical_performance_v2, 10,000 ft /
// 3,000 lb, wind 0, clean, speedbrake retracted). They are test-local controller-excitation values,
// not production recommendations or operational limits. Each one is the equilibrium the PX4 setter
// actually asks for, taken at the airspeed PX4's own definition names:
//
//   TecsMaxClimbRateMps  "climb rate produced by max allowed throttle"
//       -> V1 Military profile (throttle-cmd 0.5, maximum dry power) at the trim EAS 189.070713.
//          SteadyClimbFeasible, source-valid, above_military_mach_table = 0,
//          commandable_under_current_policy = 1.
//   TecsMinSinkRateMps   "minimum sink rate at min throttle and TRIM speed"
//       -> V1 Idle profile (throttle-cmd 0.0) at the configured trim EAS 189.070713.
//          SteadySinkFeasible, source-valid, policy-compatible, speedbrake retracted.
//   TecsMaxSinkRateMps   "maximum sink rate at min throttle and MAX speed"
//       -> V1 Idle profile (throttle-cmd 0.0) at the configured EasMax 220.0.
//          SteadySinkFeasible, source-valid, policy-compatible, speedbrake retracted.
//
// Deliberately NOT used: the 12.281952778325049 m/s sink at EAS 90 (smallest sink magnitude in the
// whole scan, but not at trim speed, so it is not what min_sink_rate means); the
// 215.288731617419700 m/s sink at EAS 280 (outside the configured EasMax and outside the current
// pitch policy); any Augmented/afterburner, source-invalid or policy-exceeded equilibrium.
constexpr double kTestLocalEasMinMps = 170.0;
constexpr double kTestLocalEasMaxMps = 220.0;
constexpr double kTestLocalMaxClimbRateMps = 100.539354597920735;   // V1 Military @ EAS 189.070713
constexpr double kTestLocalMinSinkRateMps = 53.569103350590687;     // V1 Idle @ EAS 189.070713 (trim)
constexpr double kTestLocalMaxSinkRateMps = 83.108839138474863;     // V1 Idle @ EAS 220.0 (EasMax)

// Known-good trim anchor (committed C1 stick plant harness). The fixture asserts the actual solver
// result against it rather than assuming it.
constexpr double kAnchorTrimEasMps = 189.070713;
constexpr double kAnchorTrimTasMps = 220.0;
constexpr double kAnchorTrimThrottleCmd = 0.296895857;
constexpr double kAnchorTrimThrottlePos = 0.593791714;
constexpr double kAnchorTrimPitchRad = 0.012764009;
constexpr double kAnchorTrimTolerance = 1.0e-6;   // the C1 harness reproduces these to 1e-9

// ---- numerical floors: a threshold never falls below these ------------------------------------
constexpr double kAngleFloorRad = 1.0e-7;
constexpr double kRateFloorRadps = 1.0e-7;
constexpr double kCommandFloorNorm = 1.0e-6;
constexpr double kAltitudeFloorM = 1.0e-3;
constexpr double kClimbFloorMps = 1.0e-3;
constexpr double kEasFloorMps = 1.0e-3;
constexpr double kDriftRateFloorMps = 1.0e-3;
constexpr double kNoiseMultiplier = 5.0;

// ---- physical sanity bounds (NOT performance gates) --------------------------------------------
constexpr double kAltitudeErrorSanityM = 1000.0;
constexpr double kEasSanityMinMps = 50.0;
constexpr double kEasSanityMaxMps = 400.0;
constexpr double kDivergenceWindowS = 10.0;   // trailing window for the non-divergence slope test

bool Finite(double v) { return std::isfinite(v) && std::abs(v) < kFiniteLimit; }
constexpr double kNa = std::numeric_limits<double>::quiet_NaN();

// ================================================================================================
// The owned plant. Nothing outside this object ever writes to the FDM.
// ================================================================================================
struct FPlantState {
    double TimeS{};
    double NorthM{}, EastM{};
    double VelNorthMps{}, VelEastMps{};
    double GroundCourseRad{}, GroundSpeedMps{};
    double AltitudeAslM{}, AltitudeAglM{}, ClimbRateMps{};
    double EasMps{}, TasMps{}, EasToTasRatio{};
    double RollRad{}, PitchRad{}, YawRad{};
    double PRadps{}, QRadps{}, RRadps{};
    double AlphaRad{}, BetaRad{}, LoadFactor{};
    double ElevatorPosRad{}, AileronLeftPosRad{}, RudderPosRad{}, ThrottlePos{};
    double GearPos{}, FlapPosNorm{}, SpeedbrakePos{};
    bool bWow{}, bEngineRunning{};
    bool bRatioValid{};

    bool IsFinite() const
    {
        const std::array<double, 25> v{
            TimeS, NorthM, EastM, VelNorthMps, VelEastMps, GroundCourseRad, GroundSpeedMps,
            AltitudeAslM, AltitudeAglM, ClimbRateMps, EasMps, TasMps, RollRad, PitchRad, YawRad,
            PRadps, QRadps, RRadps, AlphaRad, BetaRad, LoadFactor,
            ElevatorPosRad, AileronLeftPosRad, RudderPosRad, ThrottlePos};
        return std::all_of(v.begin(), v.end(), Finite);
    }
};

struct FTrimResult {
    bool bAttempted{}, bSuccess{};
    double PitchRad{}, RollRad{}, EasMps{}, TasMps{}, AltitudeM{};
    double ThrottleCmd{}, ThrottlePos{}, ElevatorCmd{}, ElevatorPosRad{}, AileronCmd{};
    double MassSlugs{}, FuelLb{};
    std::array<double, 4> TankLb{{kNa, kNa, kNa, kNa}};
};

struct FWriteCounters {
    std::uint64_t CoordinatorUpdates{}, StickUpdates{}, FdmRuns{}, CommandFrames{};
    std::uint64_t AileronWrites{}, ElevatorWrites{}, RudderWrites{}, ThrottleWrites{};
    std::uint64_t ExpectedResetFrames{}, UnexpectedInvalidCoordinatorFrames{};
    std::uint64_t UnexpectedInvalidStickFrames{};
    std::uint64_t WritesOutsideOwnedFdm{}, ProductionWriterInvocations{};
    std::uint64_t FdmRunFailures{};
};

class FOwnedF16Plant {
public:
    bool Initialize(const std::string &root, std::string &failure)
    {
        Exec = std::make_unique<JSBSim::FGFDMExec>();
        OwnedIdentity = Exec.get();
        Exec->SetDebugLevel(0);
        Exec->SetRootDir(SGPath(root));
        Exec->SetAircraftPath(SGPath("aircraft"));
        Exec->SetEnginePath(SGPath("engine"));
        Exec->SetSystemsPath(SGPath("systems"));
        if (!Exec->LoadModel("f16")) { failure = "LoadModel(f16) failed"; return false; }
        Exec->Setdt(kFdmDtS);

        Fcs = Exec->GetFCS(); Propagate = Exec->GetPropagate(); Auxiliary = Exec->GetAuxiliary();
        Propulsion = Exec->GetPropulsion(); Ground = Exec->GetGroundReactions();
        MassBalance = Exec->GetMassBalance();
        if (!Fcs || !Propagate || !Auxiliary || !Propulsion || !Ground || !MassBalance) {
            failure = "required JSBSim model pointer is null"; return false;
        }
        if (Propulsion->GetNumTanks() != 4) {
            failure = "f16 model does not declare the expected 4 fuel tanks"; return false;
        }
        const std::array<double, 4> want{kTank0Lb, kTank1Lb, kTank2Lb, kTank3Lb};
        for (unsigned i = 0; i < 4; ++i) {
            auto tank = Propulsion->GetTank(i);
            if (!tank) { failure = "null fuel tank"; return false; }
            if (want[i] < 0.0 || want[i] > tank->GetCapacity() + 1e-9) {
                failure = "requested tank fuel exceeds the declared model capacity"; return false;
            }
            tank->SetContents(want[i]);
        }

        // Airborne IC: TAS 220 m/s at 10,000 ft, wings level, gamma 0, no wind. This is the C1 test
        // condition, not a TECS min/trim/max airspeed recommendation.
        Exec->SetPropertyValue("ic/lat-geod-deg", kInitialLatitudeDeg);
        Exec->SetPropertyValue("ic/long-gc-deg", kInitialLongitudeDeg);
        Exec->SetPropertyValue("ic/h-sl-ft", kInitialAltitudeFt);
        Exec->SetPropertyValue("ic/psi-true-deg", kInitialHeadingDeg);
        Exec->SetPropertyValue("ic/vt-kts", kInitialTasMps / kKnotToMps);
        Exec->SetPropertyValue("ic/gamma-deg", 0.0);
        Exec->SetPropertyValue("ic/phi-deg", 0.0);
        Exec->SetPropertyValue("ic/theta-deg", 0.0);
        Exec->SetPropertyValue("ic/p-rad_sec", 0.0);
        Exec->SetPropertyValue("ic/q-rad_sec", 0.0);
        Exec->SetPropertyValue("ic/r-rad_sec", 0.0);
        Exec->SetPropertyValue("atmosphere/wind-north-fps", 0.0);
        Exec->SetPropertyValue("atmosphere/wind-east-fps", 0.0);
        Exec->SetPropertyValue("atmosphere/wind-down-fps", 0.0);
        if (!Exec->RunIC()) { failure = "RunIC failed"; return false; }

        Propulsion->InitRunning(-1);
        Propulsion->SetFuelFreeze(true);
        Fcs->SetGearCmd(0.0); Fcs->SetGearPos(0.0);
        Fcs->SetDfCmd(0.0); Fcs->SetDfPos(JSBSim::ofNorm, 0.0);
        Fcs->SetDsbCmd(0.0);
        Fcs->SetThrottleCmd(0, 0.6);

        Trim.bAttempted = true;
        JSBSim::FGTrim trim(Exec.get(), JSBSim::tFull);
        Trim.bSuccess = trim.DoTrim();
        if (!Trim.bSuccess) {
            trim.Report();
            failure = "FGTrim(tFull).DoTrim() failed; a silent trim fallback is forbidden";
            return false;
        }

        Trim.PitchRad = Propagate->GetEuler(JSBSim::FGJSBBase::eTht);
        Trim.RollRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePhi);
        Trim.EasMps = Auxiliary->GetVequivalentKTS() * kKnotToMps;
        Trim.TasMps = Auxiliary->GetVt() * kFtToM;
        Trim.AltitudeM = Propagate->GetAltitudeASL() * kFtToM;
        Trim.ThrottleCmd = Fcs->GetThrottleCmd(0);
        Trim.ThrottlePos = Fcs->GetThrottlePos(0);
        Trim.ElevatorCmd = Fcs->GetDeCmd();
        Trim.ElevatorPosRad = Fcs->GetDePos(JSBSim::ofRad);
        Trim.AileronCmd = Fcs->GetDaCmd();
        Trim.MassSlugs = MassBalance->GetMass();
        Trim.FuelLb = Exec->GetPropertyValue("propulsion/total-fuel-lbs");
        for (unsigned i = 0; i < 4; ++i) {
            auto tank = Propulsion->GetTank(i);
            Trim.TankLb[i] = tank ? tank->GetContents() : kNa;
        }
        if (!Finite(Trim.EasMps) || Trim.EasMps <= 0.0 || !Finite(Trim.TasMps) || Trim.TasMps <= 0.0) {
            failure = "non-finite or non-positive airspeed after trim"; return false;
        }
        if (std::abs(Trim.FuelLb - kExpectedFuelLb) > 1e-6) {
            failure = "fuel load is not the expected 3,000 lb"; return false;
        }

        OriginLatRad = Propagate->GetLatitude();
        OriginLonRad = Propagate->GetLongitude();
        if (!Finite(OriginLatRad) || !Finite(OriginLonRad)) {
            failure = "non-finite geodetic origin after trim"; return false;
        }
        if (!Read().IsFinite()) { failure = "non-finite plant state after successful trim"; return false; }
        return true;
    }

    FPlantState Read() const
    {
        FPlantState s{};
        s.TimeS = Exec->GetSimTime();
        // Harness-owned local NE frame: a flat-earth transform of the ACTUAL FDM geodetic state.
        // Nothing is estimated or scripted; this only expresses the plant position in the frame the
        // production navigation types expect.
        const double lat = Propagate->GetLatitude();
        const double lon = Propagate->GetLongitude();
        s.NorthM = (lat - OriginLatRad) * kEarthRadiusM;
        s.EastM = (lon - OriginLonRad) * kEarthRadiusM * std::cos(OriginLatRad);
        s.VelNorthMps = Propagate->GetVel(JSBSim::FGJSBBase::eNorth) * kFtToM;
        s.VelEastMps = Propagate->GetVel(JSBSim::FGJSBBase::eEast) * kFtToM;
        s.GroundSpeedMps = Auxiliary->GetVground() * kFtToM;
        s.GroundCourseRad = std::atan2(s.VelEastMps, s.VelNorthMps);
        s.AltitudeAslM = Propagate->GetAltitudeASL() * kFtToM;
        s.AltitudeAglM = Propagate->GetDistanceAGL() * kFtToM;
        s.ClimbRateMps = Propagate->Gethdot() * kFtToM;
        s.EasMps = Auxiliary->GetVequivalentKTS() * kKnotToMps;
        s.TasMps = Auxiliary->GetVt() * kFtToM;
        // TECS eas_to_tas converts EAS into TAS: tas = eas * eas_to_tas. A zero or non-finite EAS is
        // rejected outright rather than divided through.
        s.bRatioValid = Finite(s.EasMps) && s.EasMps > 1.0e-6 && Finite(s.TasMps);
        s.EasToTasRatio = s.bRatioValid ? (s.TasMps / s.EasMps) : kNa;
        s.RollRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePhi);
        s.PitchRad = Propagate->GetEuler(JSBSim::FGJSBBase::eTht);
        s.YawRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePsi);
        s.PRadps = Propagate->GetPQR(1);
        s.QRadps = Propagate->GetPQR(2);
        s.RRadps = Propagate->GetPQR(3);
        s.AlphaRad = Auxiliary->Getalpha();
        s.BetaRad = Auxiliary->Getbeta();
        s.LoadFactor = Auxiliary->GetNlf();
        s.ElevatorPosRad = Fcs->GetDePos(JSBSim::ofRad);
        s.AileronLeftPosRad = Fcs->GetDaLPos(JSBSim::ofRad);
        s.RudderPosRad = Fcs->GetDrPos(JSBSim::ofRad);
        s.ThrottlePos = Fcs->GetThrottlePos(0);
        s.GearPos = Fcs->GetGearPos();
        s.FlapPosNorm = Fcs->GetDfPos(JSBSim::ofNorm);
        s.SpeedbrakePos = Fcs->GetDsbPos(JSBSim::ofNorm);
        s.bWow = Ground->GetWOW();
        auto engine = Propulsion->GetEngine(0);
        s.bEngineRunning = engine ? engine->GetRunning() : false;
        return s;
    }

    // Normalized FCS commands only. No surface position, no aerodynamic coefficient, ever.
    bool WriteOwned(const FF16StickCommandV2 &cmd, FWriteCounters &counters)
    {
        if (Exec.get() != OwnedIdentity) { ++counters.WritesOutsideOwnedFdm; return false; }
        Fcs->SetDaCmd(cmd.AileronCmdNorm); ++counters.AileronWrites;
        Fcs->SetDeCmd(cmd.ElevatorCmdNorm); ++counters.ElevatorWrites;
        // The plugin's CopyToJSBSim passes elevator straight through and flips only rudder.
        Fcs->SetDrCmd(-cmd.RudderCmdNorm); ++counters.RudderWrites;
        Fcs->SetThrottleCmd(0, cmd.ThrottleCmdNorm); ++counters.ThrottleWrites;
        ++counters.CommandFrames;
        return true;
    }

    FF16StickCommandV2 TrimCommand() const
    {
        FF16StickCommandV2 c{};
        c.AileronCmdNorm = Trim.AileronCmd;
        c.ElevatorCmdNorm = Trim.ElevatorCmd;
        c.RudderCmdNorm = 0.0;
        c.ThrottleCmdNorm = std::clamp(Trim.ThrottleCmd, 0.0, 1.0);
        c.bValid = true;
        return c;
    }

    bool Run(FWriteCounters &counters)
    {
        ++counters.FdmRuns;
        const bool ok = Exec->Run();
        if (!ok) ++counters.FdmRunFailures;
        return ok;
    }

    const FTrimResult &TrimInfo() const { return Trim; }

private:
    std::unique_ptr<JSBSim::FGFDMExec> Exec;
    JSBSim::FGFDMExec *OwnedIdentity{};
    std::shared_ptr<JSBSim::FGFCS> Fcs;
    std::shared_ptr<JSBSim::FGPropagate> Propagate;
    std::shared_ptr<JSBSim::FGAuxiliary> Auxiliary;
    std::shared_ptr<JSBSim::FGPropulsion> Propulsion;
    std::shared_ptr<JSBSim::FGGroundReactions> Ground;
    std::shared_ptr<JSBSim::FGMassBalance> MassBalance;
    FTrimResult Trim{};
    double OriginLatRad{}, OriginLonRad{};
};

// ================================================================================================
// Cases
// ================================================================================================
struct FCaseDef {
    std::string Name;
    std::uint32_t ResetGeneration{};
    double DurationS{kCaseDurationS};
    double AltitudeStepM{};      // applied at kStepTimeS
    double EasStepMps{};         // applied at kStepTimeS
    bool bReversal{};            // altitude command returns to baseline at kReversalTimeS
};

std::vector<FCaseDef> BuildCases()
{
    std::vector<FCaseDef> cases;
    cases.push_back({"baseline_hold_a", 1, kCaseDurationS, 0.0, 0.0, false});
    cases.push_back({"baseline_hold_b", 2, kCaseDurationS, 0.0, 0.0, false});
    cases.push_back({"baseline_hold_c", 3, kCaseDurationS, 0.0, 0.0, false});
    cases.push_back({"altitude_step_up", 4, kCaseDurationS, +kAltitudeStepM, 0.0, false});
    cases.push_back({"altitude_step_down", 5, kCaseDurationS, -kAltitudeStepM, 0.0, false});
    cases.push_back({"eas_step_up", 6, kCaseDurationS, 0.0, +kEasStepMps, false});
    cases.push_back({"eas_step_down", 7, kCaseDurationS, 0.0, -kEasStepMps, false});
    cases.push_back({"altitude_reversal", 8, kReversalDurationS, +kAltitudeStepM, 0.0, true});
    return cases;
}

struct FFrame {
    int CaseIndex{};
    double TimeS{};
    std::uint32_t ResetGeneration{};
    bool bCommandReady{};
    int GuidanceFailure{}, StickFailure{};
    bool bStickValid{};
    double AltitudeSetpointM{}, AltitudeErrorM{}, EasSetpointMps{}, EasErrorMps{};
    double RollReferenceRad{}, PitchReferenceRad{}, ThrottleReferenceNorm{};
    double UnderspeedRatio{}, FastDescendRatio{};
    double TotalEnergyRateSp{}, TotalEnergyRateEst{}, EnergyBalanceRateSp{}, EnergyBalanceRateEst{};
    double TecsPitchIntegrator{}, TecsThrottleIntegrator{};
    double AileronCmd{}, ElevatorCmd{}, RudderCmd{}, ThrottleCmd{};
    bool bCommandApplied{};
    FPlantState State{};
};

// Baseline noise/drift, measured only over the quiet window before any step.
struct FNoise {
    double PitchRefP2p{}, ThrottleRefP2p{}, ElevatorCmdP2p{}, ThrottleCmdP2p{}, RollRefP2p{};
    double QP2p{}, ClimbP2p{}, AltitudeP2p{}, EasP2p{};
    double AltitudeDriftM{}, EasDriftMps{}, PitchDriftRad{}, AltitudeDriftRateMps{};
};

struct FThresholds {
    double PitchRef{}, Elevator{}, Q{}, Climb{}, Altitude{}, ThrottleRef{}, ThrottleCmd{}, Eas{};
    double AltitudeDriftRateMps{};
};

double PeakToPeak(const std::vector<double> &v)
{
    if (v.empty()) return 0.0;
    const auto mm = std::minmax_element(v.begin(), v.end());
    return *mm.second - *mm.first;
}
double Mean(const std::vector<double> &v)
{
    if (v.empty()) return kNa;
    double s = 0.0;
    for (double x : v) s += x;
    return s / static_cast<double>(v.size());
}

double AltitudeSetpoint(const FCaseDef &c, double trimAltitudeM, double timeS)
{
    double sp = trimAltitudeM;
    if (timeS >= kStepTimeS) sp += c.AltitudeStepM;
    if (c.bReversal && timeS >= kReversalTimeS) sp = trimAltitudeM;
    return sp;
}
double EasSetpoint(const FCaseDef &c, double trimEasMps, double timeS)
{
    return trimEasMps + (timeS >= kStepTimeS ? c.EasStepMps : 0.0);
}

const char *GuidanceFailureName(EGuidanceFailureV2 f)
{
    switch (f) {
    case EGuidanceFailureV2::None: return "None";
    case EGuidanceFailureV2::Disabled: return "Disabled";
    case EGuidanceFailureV2::ShadowDisabled: return "ShadowDisabled";
    case EGuidanceFailureV2::Paused: return "Paused";
    case EGuidanceFailureV2::ResetFrame: return "ResetFrame";
    case EGuidanceFailureV2::InvalidFollower: return "InvalidFollower";
    case EGuidanceFailureV2::InvalidPlannerDto: return "InvalidPlannerDto";
    case EGuidanceFailureV2::InvalidWind: return "InvalidWind";
    case EGuidanceFailureV2::InvalidTime: return "InvalidTime";
    case EGuidanceFailureV2::OriginMismatch: return "OriginMismatch";
    case EGuidanceFailureV2::ResetMismatch: return "ResetMismatch";
    case EGuidanceFailureV2::ZeroTangent: return "ZeroTangent";
    case EGuidanceFailureV2::NonFiniteInput: return "NonFiniteInput";
    case EGuidanceFailureV2::NonFiniteOutput: return "NonFiniteOutput";
    case EGuidanceFailureV2::InvalidConfig: return "InvalidConfig";
    }
    return "Unknown";
}
const char *StickFailureName(EF16StickFailureV2 f)
{
    switch (f) {
    case EF16StickFailureV2::None: return "None";
    case EF16StickFailureV2::ResetFrame: return "ResetFrame";
    case EF16StickFailureV2::Paused: return "Paused";
    case EF16StickFailureV2::InvalidGuidance: return "InvalidGuidance";
    case EF16StickFailureV2::InvalidAttitude: return "InvalidAttitude";
    case EF16StickFailureV2::InvalidBodyRates: return "InvalidBodyRates";
    case EF16StickFailureV2::InvalidAlphaBeta: return "InvalidAlphaBeta";
    case EF16StickFailureV2::InvalidTime: return "InvalidTime";
    case EF16StickFailureV2::AbnormalDt: return "AbnormalDt";
    case EF16StickFailureV2::NonFiniteInput: return "NonFiniteInput";
    case EF16StickFailureV2::NonFiniteOutput: return "NonFiniteOutput";
    }
    return "Unknown";
}

struct FAudit {
    std::uint64_t Checks{}, Failures{};
    std::vector<std::string> FailureMessages;
    void Check(bool ok, const std::string &what)
    {
        ++Checks;
        if (!ok) {
            ++Failures;
            if (FailureMessages.size() < 40) FailureMessages.push_back(what);
        }
    }
};

struct FCaseResult {
    FCaseDef Def{};
    bool bTrimSuccess{};
    FTrimResult Trim{};
    std::uint64_t ControlFrames{}, FdmSteps{};
    FWriteCounters Counters{};
    FNoise Noise{};
    // pre-step references (mean over the quiet window)
    double RefPitchRef{kNa}, RefThrottleRef{kNa}, RefElevatorCmd{kNa}, RefThrottleCmd{kNa};
    double RefQ{kNa}, RefClimb{kNa}, RefAltitude{kNa}, RefEas{kNa};
    // step response
    double InitialAltitudeErrorM{kNa}, FinalAltitudeErrorM{kNa};
    double InitialEasErrorMps{kNa}, FinalEasErrorMps{kNa};
    double ReversalAltitudeErrorM{kNa};
    double DetectPitchRefS{-1.0}, DetectElevatorS{-1.0}, DetectQS{-1.0}, DetectClimbS{-1.0};
    double DetectAltitudeS{-1.0}, DetectThrottleRefS{-1.0}, DetectThrottleCmdS{-1.0}, DetectEasS{-1.0};
    double ReversalDetectPitchRefS{-1.0}, ReversalDetectElevatorS{-1.0}, ReversalDetectQS{-1.0};
    double ReversalDetectClimbS{-1.0};
    double PeakClimbMps{kNa}, PeakSinkMps{kNa};
    double PitchRefMin{kNa}, PitchRefMax{kNa};
    double ThrottleRefMin{kNa}, ThrottleRefMax{kNa};
    double ElevatorCmdMin{kNa}, ElevatorCmdMax{kNa};
    double ThrottleCmdMin{kNa}, ThrottleCmdMax{kNa};
    double RollRefAbsMax{kNa};
    double MaxAbsAltitudeErrorM{kNa};
    double AltitudeErrorSlopeTailMps{kNa};
    // settled tail (last kDivergenceWindowS) means, for the step-response deltas
    double TailThrottleRef{kNa}, TailThrottleCmd{kNa}, TailEas{kNa}, TailPitchRef{kNa};
    double ThrottleRefDelta{kNa}, ThrottleCmdDelta{kNa}, EasResponseDelta{kNa};
    double SteRateSpAtStep{kNa}, SteRateEstAtStep{kNa}, SteRateSpFinal{kNa}, SteRateEstFinal{kNa};
    double UnderspeedMax{0.0}, FastDescendMax{0.0};
    std::uint64_t ThrottleSaturatedFrames{}, PitchSaturatedFrames{}, ElevatorSaturatedFrames{};
    std::uint64_t UnderspeedFrames{}, FastDescendFrames{};
    std::uint64_t NonFiniteStates{}, UnexpectedWow{}, ConfigViolations{};
    std::uint64_t CommandRangeViolations{}, CommandSlewViolations{}, TimestampRegressions{};
    std::uint64_t ResetGenerationMismatches{};
    std::vector<FFrame> Trace;
};

// ================================================================================================
// One isolated case: fresh FDM, fresh trim, fresh coordinator, fresh stick, fresh counters.
// ================================================================================================
bool RunCase(const std::string &root, int caseIndex, const FCaseDef &def,
             const FGuidanceConfigV2 &guidanceConfigTemplate, const FF16StickConfigV2 &stickConfig,
             FCaseResult &result, FAudit &audit, std::string &failure)
{
    result.Def = def;

    FOwnedF16Plant plant;
    if (!plant.Initialize(root, failure)) return false;
    result.bTrimSuccess = true;
    const FTrimResult &trim = plant.TrimInfo();
    result.Trim = trim;

    // The trim airspeed and trim throttle come from the ACTUAL solver result, never from a literal.
    // The production defaults (EAS trim 15 m/s, throttle trim 0.5) describe no airframe.
    FGuidanceConfigV2 guidanceConfig = guidanceConfigTemplate;
    guidanceConfig.TecsEquivalentAirspeedTrimMps = trim.EasMps;
    guidanceConfig.ThrottleTrim = std::clamp(trim.ThrottleCmd, guidanceConfig.ThrottleMin, guidanceConfig.ThrottleMax);

    // The known-good anchor is asserted, not assumed.
    audit.Check(std::abs(trim.EasMps - kAnchorTrimEasMps) <= kAnchorTrimTolerance, def.Name + ": trim EAS is not the known-good anchor");
    audit.Check(std::abs(trim.TasMps - kAnchorTrimTasMps) <= kAnchorTrimTolerance, def.Name + ": trim TAS is not the known-good anchor");
    audit.Check(std::abs(trim.ThrottleCmd - kAnchorTrimThrottleCmd) <= kAnchorTrimTolerance, def.Name + ": trim throttle command is not the known-good anchor");
    audit.Check(std::abs(trim.ThrottlePos - kAnchorTrimThrottlePos) <= kAnchorTrimTolerance, def.Name + ": trim throttle position is not the known-good anchor");
    audit.Check(std::abs(trim.PitchRad - kAnchorTrimPitchRad) <= kAnchorTrimTolerance, def.Name + ": trim pitch is not the known-good anchor");

    // The production contract validates the completed config. A rejection here is a hard stop, not
    // something to work around inside the fixture.
    if (!FormationControlV2::IsGuidanceConfigValid(guidanceConfig)) {
        failure = def.Name + ": the completed test-local config is rejected by IsGuidanceConfigValid";
        return false;
    }

    FormationGuidanceCoordinatorV2 coordinator;
    coordinator.Reset(def.ResetGeneration);
    F16StickAdapterV2 stick;
    stick.Reset(def.ResetGeneration);

    const double trimAltitudeM = trim.AltitudeM;
    const double trimEasMps = trim.EasMps;

    // Straight path: through the trim position, tangent along the trim ground course (due east).
    const FPlantState initial = plant.Read();
    const Vec2 pathOrigin{initial.NorthM, initial.EastM};
    const Vec2 pathTangent{std::cos(initial.GroundCourseRad), std::sin(initial.GroundCourseRad)};

    FF16StickCommandV2 heldCommand = plant.TrimCommand();
    FF16StickCommandV2 previousApplied = heldCommand;
    bool havePreviousApplied = true;
    FF16StickCommandV2 previousValid{};
    bool havePreviousValid = false;

    double previousTimeS = -1.0;
    const int controlFrames = static_cast<int>(std::llround(def.DurationS / kControllerDtS));

    for (int frameIndex = 0; frameIndex < controlFrames; ++frameIndex) {
        const FPlantState state = plant.Read();
        if (!state.IsFinite()) ++result.NonFiniteStates;
        audit.Check(state.IsFinite(), def.Name + ": non-finite plant state");
        if (state.bWow) ++result.UnexpectedWow;
        audit.Check(!state.bWow, def.Name + ": unexpected weight on wheels");
        const bool configOk = std::abs(state.GearPos) <= 1e-9 && std::abs(state.FlapPosNorm) <= 1e-9 &&
                              std::abs(state.SpeedbrakePos) <= 1e-9 && state.bEngineRunning;
        if (!configOk) ++result.ConfigViolations;
        audit.Check(configOk, def.Name + ": gear/flap/speedbrake/engine configuration violation");
        if (state.TimeS < previousTimeS - 1e-12) ++result.TimestampRegressions;
        audit.Check(state.TimeS >= previousTimeS - 1e-12, def.Name + ": simulation timestamp regression");
        previousTimeS = state.TimeS;

        const double altitudeSetpointM = AltitudeSetpoint(def, trimAltitudeM, state.TimeS);
        const double easSetpointMps = EasSetpoint(def, trimEasMps, state.TimeS);

        // ---- production coordinator input, built from the ACTUAL plant state ------------------
        FGuidanceCoordinatorInputV2 in{};
        FCanonicalNavigationStateV2 &f = in.Follower;
        f.PositionNE_m = Vec2{state.NorthM, state.EastM};
        f.GroundVelocityNE_mps = Vec2{state.VelNorthMps, state.VelEastMps};
        f.GroundCourse_rad = state.GroundCourseRad;
        f.CourseRate_radps = 0.0;
        f.Curvature_per_m = 0.0;
        f.AltitudeAsl_m = state.AltitudeAslM;
        f.ClimbRate_mps = state.ClimbRateMps;
        f.SimulationTimeS = state.TimeS;
        f.ResetGeneration = def.ResetGeneration;
        f.OriginGeneration = def.ResetGeneration;
        f.EquivalentAirspeed_mps = state.EasMps;
        f.TrueAirspeed_mps = state.TasMps;
        f.WindNE_mps = Vec2{0.0, 0.0};
        f.EasToTasRatio = state.EasToTasRatio;
        f.bPositionValid = f.bGroundVelocityValid = f.bGroundCourseValid = true;
        f.bCourseRateValid = f.bCurvatureValid = f.bAltitudeValid = f.bClimbRateValid = true;
        f.bSimulationTimeValid = f.bEasValid = f.bTasValid = f.bWindValid = f.bOriginValid = true;
        f.bRatioValid = state.bRatioValid;   // a zero/non-finite EAS is refused, never divided through

        FFormationSlotStateV2 &slot = in.Slot;
        slot.ResetGeneration = def.ResetGeneration;
        slot.OriginGeneration = def.ResetGeneration;
        slot.bValid = true;

        // Quiescent lateral channel: the real NPFG still runs, it is simply given a straight path
        // that the follower is already on, aligned with its ground course, with zero curvature.
        FPlannerV2OutputAdapterResult &dto = in.PlannerDto;
        const Vec2 offset = f.PositionNE_m - pathOrigin;
        const double along = offset.Dot(pathTangent);
        dto.Npfg.PathPositionNE_m = pathOrigin + pathTangent * along;
        dto.Npfg.PathUnitTangentNE = pathTangent;
        dto.Npfg.PathCurvature_per_m = 0.0;
        dto.Npfg.bValid = true;
        dto.Tecs.TargetEasMps = easSetpointMps;
        dto.Tecs.TargetAltitudeAslM = altitudeSetpointM;
        dto.Tecs.TargetClimbRateMps = 0.0;
        dto.Tecs.bTargetEasValid = true;
        dto.Tecs.bTargetAltitudeValid = true;
        dto.Tecs.bTargetClimbRateValid = false;   // altitude-demand mode, not height-rate mode
        dto.Tecs.bCommandReady = true;

        in.CurrentPitchRad = state.PitchRad;
        in.bCurrentPitchValid = true;
        in.SimulationTimeS = state.TimeS;
        in.DtS = kControllerDtS;
        in.ResetGeneration = def.ResetGeneration;
        in.OriginGeneration = def.ResetGeneration;
        in.bControllerEnabled = true;
        in.bShadowEnabled = true;
        in.bPaused = false;

        ++result.Counters.CoordinatorUpdates;
        const FGuidanceCoordinatorOutputV2 guidance = coordinator.Update(in, guidanceConfig);

        const bool expectedResetFrame = (frameIndex == 0);
        if (expectedResetFrame) {
            ++result.Counters.ExpectedResetFrames;
            audit.Check(!guidance.bCommandReady &&
                            guidance.FailureReason == EGuidanceFailureV2::ResetFrame,
                        def.Name + ": first coordinator frame is not the expected ResetFrame");
        } else if (!guidance.bCommandReady) {
            ++result.Counters.UnexpectedInvalidCoordinatorFrames;
            audit.Check(false, def.Name + ": unexpected invalid coordinator frame (" +
                                   GuidanceFailureName(guidance.FailureReason) + ")");
        }
        if (guidance.bCommandReady) {
            const bool genOk = guidance.ResetGeneration == def.ResetGeneration;
            if (!genOk) ++result.ResetGenerationMismatches;
            audit.Check(genOk, def.Name + ": coordinator reset-generation mismatch");
        }

        // ---- production stick input ------------------------------------------------------------
        FF16StickInputV2 si{};
        si.RollReferenceRad = guidance.RollReferenceRad;
        si.PitchReferenceRad = guidance.PitchReferenceRad;
        si.ThrottleReferenceNorm = guidance.ThrottleReferenceNorm;
        si.bGuidanceValid = guidance.bCommandReady;
        si.CurrentRollRad = state.RollRad;
        si.CurrentPitchRad = state.PitchRad;
        si.bAttitudeValid = true;
        si.BodyRollRateRadps = state.PRadps;
        si.BodyPitchRateRadps = state.QRadps;
        si.BodyYawRateRadps = state.RRadps;
        si.bBodyRatesValid = true;
        si.AlphaRad = state.AlphaRad;
        si.BetaRad = state.BetaRad;
        si.bAlphaBetaValid = true;
        si.EasMps = state.EasMps;
        si.TasMps = state.TasMps;
        si.bAirspeedValid = true;
        si.SimulationTimeS = state.TimeS;
        si.DtS = kControllerDtS;
        si.bPaused = false;
        si.ResetGeneration = def.ResetGeneration;

        ++result.Counters.StickUpdates;
        const FF16StickCommandV2 command = stick.Update(si, stickConfig);

        if (expectedResetFrame) {
            audit.Check(!command.bValid && command.FailureReason == EF16StickFailureV2::ResetFrame,
                        def.Name + ": first stick frame is not the expected ResetFrame");
        } else if (!command.bValid) {
            ++result.Counters.UnexpectedInvalidStickFrames;
            audit.Check(false, def.Name + ": unexpected invalid stick output (" +
                                   StickFailureName(command.FailureReason) + ")");
        }

        if (command.bValid) {
            const bool rangeOk =
                command.AileronCmdNorm >= -1.0 - 1e-12 && command.AileronCmdNorm <= 1.0 + 1e-12 &&
                command.ElevatorCmdNorm >= stickConfig.ElevatorMin - 1e-12 &&
                command.ElevatorCmdNorm <= stickConfig.ElevatorMax + 1e-12 &&
                command.RudderCmdNorm >= -1.0 - 1e-12 && command.RudderCmdNorm <= 1.0 + 1e-12 &&
                command.ThrottleCmdNorm >= stickConfig.ThrottleMin - 1e-12 &&
                command.ThrottleCmdNorm <= stickConfig.ThrottleMax + 1e-12;
            if (!rangeOk) ++result.CommandRangeViolations;
            audit.Check(rangeOk, def.Name + ": command range violation");

            if (havePreviousValid) {
                const bool slewOk =
                    std::abs(command.AileronCmdNorm - previousValid.AileronCmdNorm) <= stickConfig.AileronSlewPerS * kControllerDtS + 1e-12 &&
                    std::abs(command.ElevatorCmdNorm - previousValid.ElevatorCmdNorm) <= stickConfig.ElevatorSlewPerS * kControllerDtS + 1e-12 &&
                    std::abs(command.RudderCmdNorm - previousValid.RudderCmdNorm) <= stickConfig.RudderSlewPerS * kControllerDtS + 1e-12 &&
                    std::abs(command.ThrottleCmdNorm - previousValid.ThrottleCmdNorm) <= stickConfig.ThrottleSlewPerS * kControllerDtS + 1e-12;
                if (!slewOk) ++result.CommandSlewViolations;
                audit.Check(slewOk, def.Name + ": stick command slew violation");
            }
            previousValid = command;
            havePreviousValid = true;

            // Hand the FCS over only after the prime window, so the stick's throttle slew anchor has
            // reached the trim command and the plant never sees a step at engagement.
            if (state.TimeS >= kPrimeS) {
                const bool appliedSlewOk =
                    !havePreviousApplied ||
                    (std::abs(command.AileronCmdNorm - previousApplied.AileronCmdNorm) <= stickConfig.AileronSlewPerS * kControllerDtS + 1e-12 &&
                     std::abs(command.ElevatorCmdNorm - previousApplied.ElevatorCmdNorm) <= stickConfig.ElevatorSlewPerS * kControllerDtS + 1e-12 &&
                     std::abs(command.RudderCmdNorm - previousApplied.RudderCmdNorm) <= stickConfig.RudderSlewPerS * kControllerDtS + 1e-12 &&
                     std::abs(command.ThrottleCmdNorm - previousApplied.ThrottleCmdNorm) <= stickConfig.ThrottleSlewPerS * kControllerDtS + 1e-12);
                if (!appliedSlewOk) ++result.CommandSlewViolations;
                audit.Check(appliedSlewOk, def.Name + ": applied FCS command slew violation");
                heldCommand = command;
                previousApplied = command;
                havePreviousApplied = true;
            }
        }

        const bool applied = plant.WriteOwned(heldCommand, result.Counters);
        audit.Check(applied, def.Name + ": command write did not target the owned FGFDMExec");

        FFrame frame{};
        frame.CaseIndex = caseIndex;
        frame.TimeS = state.TimeS;
        frame.ResetGeneration = def.ResetGeneration;
        frame.bCommandReady = guidance.bCommandReady;
        frame.GuidanceFailure = static_cast<int>(guidance.FailureReason);
        frame.StickFailure = static_cast<int>(command.FailureReason);
        frame.bStickValid = command.bValid;
        frame.AltitudeSetpointM = altitudeSetpointM;
        frame.AltitudeErrorM = altitudeSetpointM - state.AltitudeAslM;
        frame.EasSetpointMps = easSetpointMps;
        frame.EasErrorMps = easSetpointMps - state.EasMps;
        frame.RollReferenceRad = guidance.RollReferenceRad;
        frame.PitchReferenceRad = guidance.PitchReferenceRad;
        frame.ThrottleReferenceNorm = guidance.ThrottleReferenceNorm;
        frame.UnderspeedRatio = guidance.UnderspeedRatio;
        frame.FastDescendRatio = guidance.FastDescendRatio;
        frame.TotalEnergyRateSp = guidance.TotalEnergyRateSetpoint;
        frame.TotalEnergyRateEst = guidance.TotalEnergyRateEstimate;
        frame.EnergyBalanceRateSp = guidance.EnergyBalanceRateSetpoint;
        frame.EnergyBalanceRateEst = guidance.EnergyBalanceRateEstimate;
        frame.TecsPitchIntegrator = guidance.PitchIntegrator;
        frame.TecsThrottleIntegrator = guidance.ThrottleIntegrator;
        frame.AileronCmd = heldCommand.AileronCmdNorm;
        frame.ElevatorCmd = heldCommand.ElevatorCmdNorm;
        frame.RudderCmd = heldCommand.RudderCmdNorm;
        frame.ThrottleCmd = heldCommand.ThrottleCmdNorm;
        frame.bCommandApplied = applied;
        frame.State = state;
        result.Trace.push_back(frame);
        ++result.ControlFrames;

        for (int sub = 0; sub < kFdmStepsPerControlFrame; ++sub) {
            const bool ok = plant.Run(result.Counters);
            audit.Check(ok, def.Name + ": FGFDMExec::Run() failed");
            ++result.FdmSteps;
            if (!ok) { failure = def.Name + ": FGFDMExec::Run() failed"; return false; }
        }
    }
    return true;
}

// ================================================================================================
// Post-processing
// ================================================================================================
void MeasureNoise(FCaseResult &r)
{
    std::vector<double> pitchRef, throttleRef, elevCmd, throttleCmd, rollRef, q, climb, alt, eas;
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS < kSettleEndS || fr.TimeS >= kStepTimeS) continue;
        pitchRef.push_back(fr.PitchReferenceRad);
        throttleRef.push_back(fr.ThrottleReferenceNorm);
        elevCmd.push_back(fr.ElevatorCmd);
        throttleCmd.push_back(fr.ThrottleCmd);
        rollRef.push_back(fr.RollReferenceRad);
        q.push_back(fr.State.QRadps);
        climb.push_back(fr.State.ClimbRateMps);
        alt.push_back(fr.State.AltitudeAslM);
        eas.push_back(fr.State.EasMps);
    }
    r.Noise.PitchRefP2p = PeakToPeak(pitchRef);
    r.Noise.ThrottleRefP2p = PeakToPeak(throttleRef);
    r.Noise.ElevatorCmdP2p = PeakToPeak(elevCmd);
    r.Noise.ThrottleCmdP2p = PeakToPeak(throttleCmd);
    r.Noise.RollRefP2p = PeakToPeak(rollRef);
    r.Noise.QP2p = PeakToPeak(q);
    r.Noise.ClimbP2p = PeakToPeak(climb);
    r.Noise.AltitudeP2p = PeakToPeak(alt);
    r.Noise.EasP2p = PeakToPeak(eas);

    r.RefPitchRef = Mean(pitchRef);
    r.RefThrottleRef = Mean(throttleRef);
    r.RefElevatorCmd = Mean(elevCmd);
    r.RefThrottleCmd = Mean(throttleCmd);
    r.RefQ = Mean(q);
    r.RefClimb = Mean(climb);
    r.RefAltitude = Mean(alt);
    r.RefEas = Mean(eas);

    // drift across the whole engaged, unstepped part of the run (baseline cases only see this)
    const FFrame *settle = nullptr;
    const FFrame *last = r.Trace.empty() ? nullptr : &r.Trace.back();
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS >= kSettleEndS) { settle = &fr; break; }
    }
    if (settle && last && last->TimeS > settle->TimeS) {
        r.Noise.AltitudeDriftM = last->State.AltitudeAslM - settle->State.AltitudeAslM;
        r.Noise.EasDriftMps = last->State.EasMps - settle->State.EasMps;
        r.Noise.PitchDriftRad = last->State.PitchRad - settle->State.PitchRad;
        r.Noise.AltitudeDriftRateMps = std::abs(r.Noise.AltitudeDriftM) / (last->TimeS - settle->TimeS);
    }
}

FThresholds DeriveThresholds(const std::vector<FCaseResult> &baselines)
{
    FNoise worst{};
    for (const FCaseResult &b : baselines) {
        worst.PitchRefP2p = std::max(worst.PitchRefP2p, b.Noise.PitchRefP2p);
        worst.ThrottleRefP2p = std::max(worst.ThrottleRefP2p, b.Noise.ThrottleRefP2p);
        worst.ElevatorCmdP2p = std::max(worst.ElevatorCmdP2p, b.Noise.ElevatorCmdP2p);
        worst.ThrottleCmdP2p = std::max(worst.ThrottleCmdP2p, b.Noise.ThrottleCmdP2p);
        worst.QP2p = std::max(worst.QP2p, b.Noise.QP2p);
        worst.ClimbP2p = std::max(worst.ClimbP2p, b.Noise.ClimbP2p);
        worst.AltitudeP2p = std::max(worst.AltitudeP2p, b.Noise.AltitudeP2p);
        worst.EasP2p = std::max(worst.EasP2p, b.Noise.EasP2p);
        worst.AltitudeDriftRateMps = std::max(worst.AltitudeDriftRateMps, b.Noise.AltitudeDriftRateMps);
    }
    FThresholds t{};
    t.PitchRef = std::max(kNoiseMultiplier * worst.PitchRefP2p, kAngleFloorRad);
    t.Elevator = std::max(kNoiseMultiplier * worst.ElevatorCmdP2p, kCommandFloorNorm);
    t.Q = std::max(kNoiseMultiplier * worst.QP2p, kRateFloorRadps);
    t.Climb = std::max(kNoiseMultiplier * worst.ClimbP2p, kClimbFloorMps);
    t.Altitude = std::max(kNoiseMultiplier * worst.AltitudeP2p, kAltitudeFloorM);
    t.ThrottleRef = std::max(kNoiseMultiplier * worst.ThrottleRefP2p, kCommandFloorNorm);
    t.ThrottleCmd = std::max(kNoiseMultiplier * worst.ThrottleCmdP2p, kCommandFloorNorm);
    t.Eas = std::max(kNoiseMultiplier * worst.EasP2p, kEasFloorMps);
    t.AltitudeDriftRateMps = std::max(kNoiseMultiplier * worst.AltitudeDriftRateMps, kDriftRateFloorMps);
    return t;
}

// First time at or after `fromS` where (value - reference) * sign exceeds the threshold.
double DetectDirection(const std::vector<FFrame> &trace, double fromS, double reference,
                       double threshold, int sign, double FFrame::*member)
{
    for (const FFrame &fr : trace) {
        if (fr.TimeS < fromS) continue;
        const double delta = (fr.*member - reference) * static_cast<double>(sign);
        if (delta > threshold) return fr.TimeS;
    }
    return -1.0;
}
double DetectStateDirection(const std::vector<FFrame> &trace, double fromS, double reference,
                            double threshold, int sign, double FPlantState::*member)
{
    for (const FFrame &fr : trace) {
        if (fr.TimeS < fromS) continue;
        const double delta = (fr.State.*member - reference) * static_cast<double>(sign);
        if (delta > threshold) return fr.TimeS;
    }
    return -1.0;
}

// Least-squares slope of |altitude error| over the trailing window. A positive slope larger than the
// baseline-derived drift rate is persistent divergence.
double TailErrorSlope(const std::vector<FFrame> &trace, double endS)
{
    const double startS = endS - kDivergenceWindowS;
    double n = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (const FFrame &fr : trace) {
        if (fr.TimeS < startS) continue;
        const double x = fr.TimeS - startS;
        const double y = std::abs(fr.AltitudeErrorM);
        n += 1.0; sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const double den = n * sxx - sx * sx;
    if (n < 2.0 || std::abs(den) < 1e-12) return kNa;
    return (n * sxy - sx * sy) / den;
}

void Summarize(FCaseResult &r)
{
    double pitchRefMin = 1e18, pitchRefMax = -1e18, throttleRefMin = 1e18, throttleRefMax = -1e18;
    double elevMin = 1e18, elevMax = -1e18, throttleCmdMin = 1e18, throttleCmdMax = -1e18;
    double rollAbsMax = 0.0, peakClimb = -1e18, peakSink = 1e18, maxAbsAltErr = 0.0;
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS < kPrimeS) continue;
        pitchRefMin = std::min(pitchRefMin, fr.PitchReferenceRad);
        pitchRefMax = std::max(pitchRefMax, fr.PitchReferenceRad);
        throttleRefMin = std::min(throttleRefMin, fr.ThrottleReferenceNorm);
        throttleRefMax = std::max(throttleRefMax, fr.ThrottleReferenceNorm);
        elevMin = std::min(elevMin, fr.ElevatorCmd);
        elevMax = std::max(elevMax, fr.ElevatorCmd);
        throttleCmdMin = std::min(throttleCmdMin, fr.ThrottleCmd);
        throttleCmdMax = std::max(throttleCmdMax, fr.ThrottleCmd);
        rollAbsMax = std::max(rollAbsMax, std::abs(fr.RollReferenceRad));
        peakClimb = std::max(peakClimb, fr.State.ClimbRateMps);
        peakSink = std::min(peakSink, fr.State.ClimbRateMps);
        maxAbsAltErr = std::max(maxAbsAltErr, std::abs(fr.AltitudeErrorM));
        if (fr.ThrottleReferenceNorm >= 1.0 - 1e-9 || fr.ThrottleReferenceNorm <= 0.0 + 1e-9)
            ++r.ThrottleSaturatedFrames;
        if (std::abs(fr.PitchReferenceRad) >= 0.5 - 1e-9) ++r.PitchSaturatedFrames;
        if (std::abs(fr.ElevatorCmd) >= 1.0 - 1e-9) ++r.ElevatorSaturatedFrames;
        if (fr.UnderspeedRatio > 1e-9) ++r.UnderspeedFrames;
        if (fr.FastDescendRatio > 1e-9) ++r.FastDescendFrames;
    }
    r.PitchRefMin = pitchRefMin; r.PitchRefMax = pitchRefMax;
    r.ThrottleRefMin = throttleRefMin; r.ThrottleRefMax = throttleRefMax;
    r.ElevatorCmdMin = elevMin; r.ElevatorCmdMax = elevMax;
    r.ThrottleCmdMin = throttleCmdMin; r.ThrottleCmdMax = throttleCmdMax;
    r.RollRefAbsMax = rollAbsMax;
    r.PeakClimbMps = peakClimb; r.PeakSinkMps = peakSink;
    r.MaxAbsAltitudeErrorM = maxAbsAltErr;

    const FFrame *atStep = nullptr;
    const FFrame *atReversal = nullptr;
    for (const FFrame &fr : r.Trace) {
        if (!atStep && fr.TimeS >= kStepTimeS) atStep = &fr;
        if (r.Def.bReversal && !atReversal && fr.TimeS >= kReversalTimeS) atReversal = &fr;
    }
    if (atStep) {
        r.InitialAltitudeErrorM = atStep->AltitudeErrorM;
        r.InitialEasErrorMps = atStep->EasErrorMps;
    }
    if (atReversal) r.ReversalAltitudeErrorM = atReversal->AltitudeErrorM;
    if (atStep) {
        r.SteRateSpAtStep = atStep->TotalEnergyRateSp;
        r.SteRateEstAtStep = atStep->TotalEnergyRateEst;
    }
    if (!r.Trace.empty()) {
        r.FinalAltitudeErrorM = r.Trace.back().AltitudeErrorM;
        r.FinalEasErrorMps = r.Trace.back().EasErrorMps;
        r.AltitudeErrorSlopeTailMps = TailErrorSlope(r.Trace, r.Trace.back().TimeS);
        r.SteRateSpFinal = r.Trace.back().TotalEnergyRateSp;
        r.SteRateEstFinal = r.Trace.back().TotalEnergyRateEst;

        // Settled tail means: the step response is judged against the settled pre-step window mean,
        // never against the trim instant.
        const double tailStartS = r.Trace.back().TimeS - kDivergenceWindowS;
        std::vector<double> tThrRef, tThrCmd, tEas, tPitchRef;
        for (const FFrame &fr : r.Trace) {
            if (fr.TimeS < tailStartS) continue;
            tThrRef.push_back(fr.ThrottleReferenceNorm);
            tThrCmd.push_back(fr.ThrottleCmd);
            tEas.push_back(fr.State.EasMps);
            tPitchRef.push_back(fr.PitchReferenceRad);
        }
        r.TailThrottleRef = Mean(tThrRef);
        r.TailThrottleCmd = Mean(tThrCmd);
        r.TailEas = Mean(tEas);
        r.TailPitchRef = Mean(tPitchRef);
        r.ThrottleRefDelta = r.TailThrottleRef - r.RefThrottleRef;
        r.ThrottleCmdDelta = r.TailThrottleCmd - r.RefThrottleCmd;
        r.EasResponseDelta = r.TailEas - r.RefEas;
    }
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS < kPrimeS) continue;
        r.UnderspeedMax = std::max(r.UnderspeedMax, fr.UnderspeedRatio);
        r.FastDescendMax = std::max(r.FastDescendMax, fr.FastDescendRatio);
    }
}

void GradeCase(FCaseResult &r, const FThresholds &t, FAudit &audit)
{
    const std::string &n = r.Def.Name;

    // Hard invariants (every case)
    audit.Check(r.NonFiniteStates == 0, n + ": non-finite plant state count is not zero");
    audit.Check(r.UnexpectedWow == 0, n + ": unexpected WOW count is not zero");
    audit.Check(r.ConfigViolations == 0, n + ": gear/flap/speedbrake/engine violation count is not zero");
    audit.Check(r.CommandRangeViolations == 0, n + ": command range violation count is not zero");
    audit.Check(r.CommandSlewViolations == 0, n + ": command slew violation count is not zero");
    audit.Check(r.TimestampRegressions == 0, n + ": timestamp regression count is not zero");
    audit.Check(r.ResetGenerationMismatches == 0, n + ": reset-generation mismatch count is not zero");
    audit.Check(r.Counters.FdmRunFailures == 0, n + ": FDM run failure count is not zero");
    audit.Check(r.Counters.UnexpectedInvalidCoordinatorFrames == 0,
                n + ": unexpected invalid coordinator frames");
    audit.Check(r.Counters.UnexpectedInvalidStickFrames == 0, n + ": unexpected invalid stick frames");
    audit.Check(r.Counters.ExpectedResetFrames == 1, n + ": expected exactly one reset frame");
    audit.Check(r.Counters.WritesOutsideOwnedFdm == 0, n + ": writes outside the owned FGFDMExec");
    audit.Check(r.Counters.ProductionWriterInvocations == 0, n + ": production writer invocations");

    // Physical sanity (NOT a performance gate): the state must stay in a believable envelope.
    audit.Check(Finite(r.MaxAbsAltitudeErrorM) && r.MaxAbsAltitudeErrorM < kAltitudeErrorSanityM,
                n + ": altitude error left the physical sanity bound");
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS < kPrimeS) continue;
        if (!(fr.State.EasMps > kEasSanityMinMps && fr.State.EasMps < kEasSanityMaxMps)) {
            audit.Check(false, n + ": EAS left the physical sanity bound");
            break;
        }
    }

    const bool isAltitudeStep = std::abs(r.Def.AltitudeStepM) > 0.0;
    const bool isEasStep = std::abs(r.Def.EasStepMps) > 0.0;

    if (isAltitudeStep) {
        const int s = r.Def.AltitudeStepM > 0.0 ? +1 : -1;
        r.DetectPitchRefS = DetectDirection(r.Trace, kStepTimeS, r.RefPitchRef, t.PitchRef, s,
                                            &FFrame::PitchReferenceRad);
        // elevator-cmd-norm is NEGATIVE for nose-up (f16.xml G-limiter convention)
        r.DetectElevatorS = DetectDirection(r.Trace, kStepTimeS, r.RefElevatorCmd, t.Elevator, -s,
                                            &FFrame::ElevatorCmd);
        r.DetectQS = DetectStateDirection(r.Trace, kStepTimeS, r.RefQ, t.Q, s, &FPlantState::QRadps);
        r.DetectClimbS = DetectStateDirection(r.Trace, kStepTimeS, r.RefClimb, t.Climb, s,
                                              &FPlantState::ClimbRateMps);
        r.DetectAltitudeS = DetectStateDirection(r.Trace, kStepTimeS, r.RefAltitude, t.Altitude, s,
                                                 &FPlantState::AltitudeAslM);
        r.DetectThrottleRefS = DetectDirection(r.Trace, kStepTimeS, r.RefThrottleRef, t.ThrottleRef,
                                               s, &FFrame::ThrottleReferenceNorm);

        audit.Check(r.DetectPitchRefS >= 0.0, n + ": no pitch-reference response in the commanded direction");
        audit.Check(r.DetectElevatorS >= 0.0, n + ": no elevator-command response in the commanded direction");
        audit.Check(r.DetectQS >= 0.0, n + ": no body pitch-rate response in the commanded direction");
        audit.Check(r.DetectClimbS >= 0.0, n + ": no climb-rate response in the commanded direction");
        audit.Check(r.DetectAltitudeS >= 0.0, n + ": altitude did not move toward the setpoint");
        audit.Check(Finite(r.InitialAltitudeErrorM) && Finite(r.FinalAltitudeErrorM) &&
                        std::abs(r.FinalAltitudeErrorM) < std::abs(r.InitialAltitudeErrorM),
                    n + ": final altitude error is not smaller than the initial altitude error");
    }

    if (isEasStep) {
        const int s = r.Def.EasStepMps > 0.0 ? +1 : -1;
        r.DetectThrottleRefS = DetectDirection(r.Trace, kStepTimeS, r.RefThrottleRef, t.ThrottleRef,
                                               s, &FFrame::ThrottleReferenceNorm);
        r.DetectThrottleCmdS = DetectDirection(r.Trace, kStepTimeS, r.RefThrottleCmd, t.ThrottleCmd,
                                               s, &FFrame::ThrottleCmd);
        r.DetectEasS = DetectStateDirection(r.Trace, kStepTimeS, r.RefEas, t.Eas, s, &FPlantState::EasMps);

        audit.Check(r.DetectThrottleRefS >= 0.0, n + ": no throttle-reference response in the commanded direction");
        audit.Check(r.DetectThrottleCmdS >= 0.0, n + ": no throttle-command response in the commanded direction");
        audit.Check(r.DetectEasS >= 0.0, n + ": EAS did not move toward the setpoint");
        audit.Check(Finite(r.InitialEasErrorMps) && Finite(r.FinalEasErrorMps) &&
                        std::abs(r.FinalEasErrorMps) < std::abs(r.InitialEasErrorMps),
                    n + ": final EAS error is not smaller than the initial EAS error");
        // The uncommanded channel must not run away: the trailing |altitude error| slope may not
        // exceed the baseline-derived drift rate.
        audit.Check(Finite(r.AltitudeErrorSlopeTailMps) &&
                        r.AltitudeErrorSlopeTailMps <= t.AltitudeDriftRateMps,
                    n + ": altitude error is diverging at the end of the run");
    }

    if (r.Def.bReversal) {
        // Direction reversal after the altitude command returns to the baseline: every longitudinal
        // signal must reverse, and the error must keep shrinking (no integrator stuck on the old
        // direction).
        r.ReversalDetectPitchRefS = DetectDirection(r.Trace, kReversalTimeS, r.RefPitchRef, t.PitchRef,
                                                    -1, &FFrame::PitchReferenceRad);
        r.ReversalDetectElevatorS = DetectDirection(r.Trace, kReversalTimeS, r.RefElevatorCmd,
                                                    t.Elevator, +1, &FFrame::ElevatorCmd);
        r.ReversalDetectQS = DetectStateDirection(r.Trace, kReversalTimeS, r.RefQ, t.Q, -1,
                                                  &FPlantState::QRadps);
        r.ReversalDetectClimbS = DetectStateDirection(r.Trace, kReversalTimeS, r.RefClimb, t.Climb, -1,
                                                      &FPlantState::ClimbRateMps);
        audit.Check(r.ReversalDetectPitchRefS >= 0.0, n + ": pitch reference did not reverse");
        audit.Check(r.ReversalDetectElevatorS >= 0.0, n + ": elevator command did not reverse");
        audit.Check(r.ReversalDetectQS >= 0.0 || r.ReversalDetectClimbS >= 0.0,
                    n + ": neither body pitch rate nor climb rate reversed");
        audit.Check(Finite(r.ReversalAltitudeErrorM) && Finite(r.FinalAltitudeErrorM) &&
                        std::abs(r.FinalAltitudeErrorM) < std::abs(r.ReversalAltitudeErrorM),
                    n + ": final altitude error is not smaller than the error at the reversal");
    }

    if (!isAltitudeStep && !isEasStep) {
        // baseline hold: no command, so the only requirement is that it does not run away
        audit.Check(Finite(r.AltitudeErrorSlopeTailMps), n + ": baseline altitude-error slope is not finite");
    }
}

// ================================================================================================
// Output
// ================================================================================================
void WriteNumber(std::ostream &out, double v, int precision)
{
    if (Finite(v)) out << std::fixed << std::setprecision(precision) << v;
    else out << "NA";
}
std::string Num(double v)
{
    if (!Finite(v)) return "NA";
    std::ostringstream s;
    s << std::fixed << std::setprecision(12) << v;
    return s.str();
}
std::string OptTime(double v) { return v < 0.0 ? std::string("not_detected") : Num(v); }

void WriteCsv(const std::string &path, const std::vector<FCaseResult> &results, int precision)
{
    std::ofstream out(path);
    out << "case,case_index,simulation_time_s,reset_generation,command_ready,guidance_failure,"
           "stick_valid,stick_failure,"
           "altitude_setpoint_m,actual_altitude_m,altitude_error_m,"
           "target_climb_rate_valid,target_climb_rate_mps,actual_climb_rate_mps,"
           "eas_setpoint_mps,actual_eas_mps,actual_tas_mps,eas_to_tas_ratio,eas_error_mps,"
           "pitch_rad,roll_rad,yaw_rad,p_radps,q_radps,r_radps,"
           "roll_reference_rad,pitch_reference_rad,throttle_reference_norm,"
           "underspeed_ratio,fast_descend_ratio,total_energy_rate_sp,total_energy_rate_estimate,"
           "energy_balance_rate_sp,energy_balance_rate_estimate,tecs_pitch_integrator,"
           "tecs_throttle_integrator,"
           "aileron_cmd_norm,elevator_cmd_norm,rudder_cmd_norm,throttle_cmd_norm,command_applied,"
           "elevator_position_rad,aileron_position_rad,rudder_position_rad,throttle_position,"
           "alpha_rad,beta_rad,load_factor,wow,engine_running,gear_pos,flap_pos_norm,speedbrake_pos,"
           "north_m,east_m,ground_course_rad,ground_speed_mps\n";
    for (const FCaseResult &r : results) {
        for (const FFrame &f : r.Trace) {
            out << r.Def.Name << ',' << f.CaseIndex << ',';
            WriteNumber(out, f.TimeS, precision); out << ',' << f.ResetGeneration << ','
                << (f.bCommandReady ? 1 : 0) << ','
                << GuidanceFailureName(static_cast<EGuidanceFailureV2>(f.GuidanceFailure)) << ','
                << (f.bStickValid ? 1 : 0) << ','
                << StickFailureName(static_cast<EF16StickFailureV2>(f.StickFailure)) << ',';
            WriteNumber(out, f.AltitudeSetpointM, precision); out << ',';
            WriteNumber(out, f.State.AltitudeAslM, precision); out << ',';
            WriteNumber(out, f.AltitudeErrorM, precision); out << ',';
            out << "0,NA,";   // target_climb_rate_valid = 0 -> the rate value is NA, never a fake 0
            WriteNumber(out, f.State.ClimbRateMps, precision); out << ',';
            WriteNumber(out, f.EasSetpointMps, precision); out << ',';
            WriteNumber(out, f.State.EasMps, precision); out << ',';
            WriteNumber(out, f.State.TasMps, precision); out << ',';
            WriteNumber(out, f.State.EasToTasRatio, precision); out << ',';
            WriteNumber(out, f.EasErrorMps, precision); out << ',';
            const std::array<double, 6> att{f.State.PitchRad, f.State.RollRad, f.State.YawRad,
                                            f.State.PRadps, f.State.QRadps, f.State.RRadps};
            for (double v : att) { WriteNumber(out, v, precision); out << ','; }
            const std::array<double, 10> tecs{
                f.RollReferenceRad, f.PitchReferenceRad, f.ThrottleReferenceNorm, f.UnderspeedRatio,
                f.FastDescendRatio, f.TotalEnergyRateSp, f.TotalEnergyRateEst, f.EnergyBalanceRateSp,
                f.EnergyBalanceRateEst, f.TecsPitchIntegrator};
            for (double v : tecs) { WriteNumber(out, v, precision); out << ','; }
            WriteNumber(out, f.TecsThrottleIntegrator, precision); out << ',';
            const std::array<double, 4> cmd{f.AileronCmd, f.ElevatorCmd, f.RudderCmd, f.ThrottleCmd};
            for (double v : cmd) { WriteNumber(out, v, precision); out << ','; }
            out << (f.bCommandApplied ? 1 : 0) << ',';
            const std::array<double, 7> plant{f.State.ElevatorPosRad, f.State.AileronLeftPosRad,
                                              f.State.RudderPosRad, f.State.ThrottlePos,
                                              f.State.AlphaRad, f.State.BetaRad, f.State.LoadFactor};
            for (double v : plant) { WriteNumber(out, v, precision); out << ','; }
            out << (f.State.bWow ? 1 : 0) << ',' << (f.State.bEngineRunning ? 1 : 0) << ',';
            const std::array<double, 3> cfg{f.State.GearPos, f.State.FlapPosNorm, f.State.SpeedbrakePos};
            for (double v : cfg) { WriteNumber(out, v, precision); out << ','; }
            WriteNumber(out, f.State.NorthM, precision); out << ',';
            WriteNumber(out, f.State.EastM, precision); out << ',';
            WriteNumber(out, f.State.GroundCourseRad, precision); out << ',';
            WriteNumber(out, f.State.GroundSpeedMps, precision); out << '\n';
        }
    }
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s <JSBSim-root> <raw.csv> <quantized.csv> <summary.txt>\n", argv[0]);
        return 2;
    }
    const std::string root = argv[1];

    // ---- test-local fixture config ---------------------------------------------------------------
    // The production FGuidanceConfigV2 defaults describe no airframe: its EAS window (10..80 m/s) does
    // not contain this aircraft's level-trim EAS (~189 m/s), and its climb/sink rates are the TECS
    // class placeholders. The production defaults are NOT changed. This host fixture supplies the
    // airframe-scale values the controller needs in order to be built at the known-good F-16 trim
    // point, and nothing else.
    const FGuidanceConfigV2 productionDefaults{};
    FGuidanceConfigV2 guidanceConfig{};              // start from the committed production defaults
    guidanceConfig.EasMinMps = kTestLocalEasMinMps;
    guidanceConfig.EasMaxMps = kTestLocalEasMaxMps;
    guidanceConfig.TecsMaxClimbRateMps = kTestLocalMaxClimbRateMps;
    guidanceConfig.TecsMinSinkRateMps = kTestLocalMinSinkRateMps;
    guidanceConfig.TecsMaxSinkRateMps = kTestLocalMaxSinkRateMps;
    // TecsEquivalentAirspeedTrimMps and ThrottleTrim are set per case from the ACTUAL FGTrim(tFull)
    // result -- never from a literal -- so the config cannot be validated until the plant is trimmed.
    // Every generic TECS gain, damping, filter and time constant stays at the committed production
    // default (see kGenericTuningFields below), as do the pitch/throttle bounds and the roll limit.

    const FF16StickConfigV2 stickConfig{};           // production defaults, no gain changes

    FAudit audit;
    const std::vector<FCaseDef> cases = BuildCases();
    std::vector<FCaseResult> results(cases.size());

    // Pass 1: the three baseline holds, which define every directional threshold.
    for (std::size_t i = 0; i < cases.size(); ++i) {
        if (std::abs(cases[i].AltitudeStepM) > 0.0 || std::abs(cases[i].EasStepMps) > 0.0) continue;
        std::string failure;
        if (!RunCase(root, static_cast<int>(i), cases[i], guidanceConfig, stickConfig, results[i],
                     audit, failure)) {
            std::fprintf(stderr, "BLOCKED: %s\n", failure.c_str());
            return 1;
        }
        MeasureNoise(results[i]);
        Summarize(results[i]);
    }

    std::vector<FCaseResult> baselines;
    for (std::size_t i = 0; i < cases.size(); ++i)
        if (std::abs(cases[i].AltitudeStepM) == 0.0 && std::abs(cases[i].EasStepMps) == 0.0)
            baselines.push_back(results[i]);
    const FThresholds thresholds = DeriveThresholds(baselines);

    // Pass 2: the commanded cases. Thresholds are already fixed and were never seen by these runs.
    for (std::size_t i = 0; i < cases.size(); ++i) {
        if (std::abs(cases[i].AltitudeStepM) == 0.0 && std::abs(cases[i].EasStepMps) == 0.0) continue;
        std::string failure;
        if (!RunCase(root, static_cast<int>(i), cases[i], guidanceConfig, stickConfig, results[i],
                     audit, failure)) {
            std::fprintf(stderr, "BLOCKED: %s\n", failure.c_str());
            return 1;
        }
        MeasureNoise(results[i]);
        Summarize(results[i]);
    }

    for (FCaseResult &r : results) GradeCase(r, thresholds, audit);

    // The three baselines are the same experiment run three times: they must agree exactly.
    for (std::size_t i = 1; i < baselines.size(); ++i) {
        audit.Check(results[0].Trace.size() == results[i].Trace.size(),
                    "baseline repeat frame-count mismatch");
        bool identical = results[0].Trace.size() == results[i].Trace.size();
        if (identical) {
            for (std::size_t k = 0; k < results[0].Trace.size(); ++k) {
                if (results[0].Trace[k].State.AltitudeAslM != results[i].Trace[k].State.AltitudeAslM ||
                    results[0].Trace[k].PitchReferenceRad != results[i].Trace[k].PitchReferenceRad ||
                    results[0].Trace[k].ThrottleCmd != results[i].Trace[k].ThrottleCmd) {
                    identical = false;
                    break;
                }
            }
        }
        audit.Check(identical, "baseline repeats are not bit-identical (reset isolation failure)");
    }

    WriteCsv(argv[2], results, 15);
    WriteCsv(argv[3], results, 9);

    FWriteCounters total{};
    std::uint64_t totalFrames = 0, totalFdmSteps = 0;
    for (const FCaseResult &r : results) {
        total.CoordinatorUpdates += r.Counters.CoordinatorUpdates;
        total.StickUpdates += r.Counters.StickUpdates;
        total.FdmRuns += r.Counters.FdmRuns;
        total.CommandFrames += r.Counters.CommandFrames;
        total.AileronWrites += r.Counters.AileronWrites;
        total.ElevatorWrites += r.Counters.ElevatorWrites;
        total.RudderWrites += r.Counters.RudderWrites;
        total.ThrottleWrites += r.Counters.ThrottleWrites;
        total.ExpectedResetFrames += r.Counters.ExpectedResetFrames;
        total.UnexpectedInvalidCoordinatorFrames += r.Counters.UnexpectedInvalidCoordinatorFrames;
        total.UnexpectedInvalidStickFrames += r.Counters.UnexpectedInvalidStickFrames;
        total.WritesOutsideOwnedFdm += r.Counters.WritesOutsideOwnedFdm;
        total.ProductionWriterInvocations += r.Counters.ProductionWriterInvocations;
        total.FdmRunFailures += r.Counters.FdmRunFailures;
        totalFrames += r.ControlFrames;
        totalFdmSteps += r.FdmSteps;
    }

    const FTrimResult refTrim{};
    (void)refTrim;

    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "TECS_F16_LONGITUDINAL_CLOSED_LOOP_V2\n";
    out << "This harness verifies the production TECS caller and F16StickAdapterV2 against the "
           "actual JSBSim F-16 plant in isolated longitudinal cases. It does not select or tune "
           "production TECS performance parameters.\n";
    out << "Any EAS or throttle values overridden by this host fixture are test-local initialization "
           "values and are not production recommendations.\n";
    out << "chain=FormationGuidanceCoordinatorV2(production_FPx4TecsAdapter+FPx4NpfgAdapter)"
           "->PitchReferenceRad/ThrottleReferenceNorm->F16StickAdapterV2(production)"
           "->normalized_elevator/throttle/aileron/rudder->f16.xml_FCS->actual_surfaces"
           "->actual_aerodynamics->JSBSim_FDM->altitude/climb/EAS/pitch->FormationGuidanceCoordinatorV2\n";
    out << "model=f16 engine=F100-PW-229 data=Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim "
           "fdm_dt_s=" << kFdmDtS << " controller_dt_s=" << kControllerDtS
        << " fdm_steps_per_control_frame=" << kFdmStepsPerControlFrame
        << " clock=fixed_step_simulation_time_only\n";
    out << "initialization_mode=FGTrim_tFull initial_altitude_ft=" << kInitialAltitudeFt
        << " initial_tas_mps=" << kInitialTasMps << " heading_deg=" << kInitialHeadingDeg
        << " wind_mps=0 gear=up flap=clean speedbrake=retracted fuel=frozen\n";

    const FTrimResult &t0 = results[0].Trim;
    out << "trim solver=FGTrim(tFull) attempted=" << (t0.bAttempted ? 1 : 0)
        << " success=" << (t0.bSuccess ? 1 : 0)
        << " altitude_m=" << Num(t0.AltitudeM)
        << " eas_mps=" << Num(t0.EasMps)
        << " tas_mps=" << Num(t0.TasMps)
        << " pitch_rad=" << Num(t0.PitchRad)
        << " roll_rad=" << Num(t0.RollRad)
        << " throttle_cmd=" << Num(t0.ThrottleCmd)
        << " throttle_pos=" << Num(t0.ThrottlePos)
        << " elevator_cmd=" << Num(t0.ElevatorCmd)
        << " elevator_pos_rad=" << Num(t0.ElevatorPosRad)
        << " mass_slugs=" << Num(t0.MassSlugs)
        << " fuel_lb=" << Num(t0.FuelLb)
        << " tanks_lb=" << Num(t0.TankLb[0]) << '|' << Num(t0.TankLb[1]) << '|'
        << Num(t0.TankLb[2]) << '|' << Num(t0.TankLb[3])
        << " fresh_trim_per_case=1\n";

    out << "The TECS performance values used by this host fixture are derived from the committed "
           "single-condition solver characterization. They are test-local controller-excitation "
           "values, not production recommendations or operational limits.\n";
    out << "Generic TECS control gains, damping, filters and time constants remain at the committed "
           "production defaults.\n";
    out << "test_local_config"
        << " EasMinMps=" << Num(guidanceConfig.EasMinMps) << "(production=" << Num(productionDefaults.EasMinMps) << ")"
        << " EasMaxMps=" << Num(guidanceConfig.EasMaxMps) << "(production=" << Num(productionDefaults.EasMaxMps) << ")"
        << " TecsEquivalentAirspeedTrimMps=actual_FGTrim_tFull_EAS_per_case=" << Num(t0.EasMps)
        << "(production=" << Num(productionDefaults.TecsEquivalentAirspeedTrimMps) << ")"
        << " ThrottleTrim=actual_FGTrim_tFull_throttle_cmd_per_case=" << Num(t0.ThrottleCmd)
        << "(production=" << Num(productionDefaults.ThrottleTrim) << ")"
        << " TecsMaxClimbRateMps=" << Num(guidanceConfig.TecsMaxClimbRateMps)
        << "(production=" << Num(productionDefaults.TecsMaxClimbRateMps) << ")"
        << " TecsMinSinkRateMps=" << Num(guidanceConfig.TecsMinSinkRateMps)
        << "(production=" << Num(productionDefaults.TecsMinSinkRateMps) << ")"
        << " TecsMaxSinkRateMps=" << Num(guidanceConfig.TecsMaxSinkRateMps)
        << "(production=" << Num(productionDefaults.TecsMaxSinkRateMps) << ")"
        << " stick_config=production_defaults_unchanged\n";
    out << "performance_value_provenance source_harness=verify_f16_vertical_performance_v2 "
           "condition=10000ft/3000lb/wind0/clean/speedbrake_retracted"
           " max_climb=Military(throttle_cmd_0.5)@EAS_189.070713:SteadyClimbFeasible,source_valid,"
           "above_military_mach_table=0,commandable_under_current_policy=1"
           " min_sink=Idle(throttle_cmd_0.0)@configured_trim_EAS_189.070713:SteadySinkFeasible,"
           "source_valid,policy_compatible,speedbrake=0"
           " max_sink=Idle(throttle_cmd_0.0)@configured_EasMax_220.0:SteadySinkFeasible,source_valid,"
           "policy_compatible,speedbrake=0"
           " excluded=EAS_90_sink_12.281952778325049(not_at_trim_speed),"
           "EAS_280_sink_215.288731617419700(outside_configured_EasMax_and_outside_pitch_policy),"
           "Augmented_afterburner,source_invalid,policy_exceeded\n";
    out << "generic_tuning_unchanged_at_production_defaults"
        << " TecsVerticalAccelLimitMps2=" << Num(guidanceConfig.TecsVerticalAccelLimitMps2)
        << " TecsAltitudeErrorTimeConstantS=" << Num(guidanceConfig.TecsAltitudeErrorTimeConstantS)
        << " TecsAirspeedErrorTimeConstantS=" << Num(guidanceConfig.TecsAirspeedErrorTimeConstantS)
        << " TecsPitchIntegratorGain=" << Num(guidanceConfig.TecsPitchIntegratorGain)
        << " TecsPitchDamping=" << Num(guidanceConfig.TecsPitchDamping)
        << " TecsThrottleIntegratorGain=" << Num(guidanceConfig.TecsThrottleIntegratorGain)
        << " TecsThrottleDamping=" << Num(guidanceConfig.TecsThrottleDamping)
        << " TecsThrottleSlewRatePerS=" << Num(guidanceConfig.TecsThrottleSlewRatePerS)
        << " TecsSteRateTimeConstantS=" << Num(guidanceConfig.TecsSteRateTimeConstantS)
        << " TecsSebRateFeedForwardGain=" << Num(guidanceConfig.TecsSebRateFeedForwardGain)
        << " TecsAltitudeRateFeedForward=" << Num(guidanceConfig.TecsAltitudeRateFeedForward)
        << " TecsSpeedWeight=" << Num(guidanceConfig.TecsSpeedWeight)
        << " TecsRollToThrottleCompensation=" << Num(guidanceConfig.TecsRollToThrottleCompensation)
        << " TecsAirspeedMeasurementStdDevMps=" << Num(guidanceConfig.TecsAirspeedMeasurementStdDevMps)
        << " TecsAirspeedRateMeasurementStdDevMps2=" << Num(guidanceConfig.TecsAirspeedRateMeasurementStdDevMps2)
        << " TecsAirspeedFilterProcessStdDevMps2=" << Num(guidanceConfig.TecsAirspeedFilterProcessStdDevMps2)
        << " PitchMinRad=" << Num(guidanceConfig.PitchMinRad)
        << " PitchMaxRad=" << Num(guidanceConfig.PitchMaxRad)
        << " ThrottleMin=" << Num(guidanceConfig.ThrottleMin)
        << " ThrottleMax=" << Num(guidanceConfig.ThrottleMax)
        << " TargetClimbRateMps=" << Num(guidanceConfig.TargetClimbRateMps)
        << " TargetSinkRateMps=" << Num(guidanceConfig.TargetSinkRateMps)
        << " RollLimitRad=" << Num(guidanceConfig.RollLimitRad) << '\n';
    out << "test_local_override_scope=host_fixture_only production_config_modified=0 "
           "tecs_gains_modified=0 stick_gains_modified=0 aircraft_xml_modified=0\n";
    out << "tecs_debug_availability tas_rate_control=NA_not_exposed_by_FGuidanceCoordinatorOutputV2 "
           "total_energy_rate=exposed energy_balance_rate=exposed underspeed_ratio=exposed "
           "fast_descend_ratio=exposed pitch_integrator=exposed throttle_integrator=exposed\n";

    out << "cases=" << results.size() << " total_control_frames=" << totalFrames
        << " total_fdm_steps=" << totalFdmSteps << '\n';
    out << "derived_thresholds source=baseline_peak_to_peak_x" << kNoiseMultiplier << "_or_numerical_floor"
        << " pitch_reference_rad=" << Num(thresholds.PitchRef)
        << " elevator_cmd_norm=" << Num(thresholds.Elevator)
        << " q_radps=" << Num(thresholds.Q)
        << " climb_rate_mps=" << Num(thresholds.Climb)
        << " altitude_m=" << Num(thresholds.Altitude)
        << " throttle_reference_norm=" << Num(thresholds.ThrottleRef)
        << " throttle_cmd_norm=" << Num(thresholds.ThrottleCmd)
        << " eas_mps=" << Num(thresholds.Eas)
        << " altitude_drift_rate_mps=" << Num(thresholds.AltitudeDriftRateMps) << '\n';

    for (const FCaseResult &r : results) {
        out << "case name=" << r.Def.Name << " reset_generation=" << r.Def.ResetGeneration
            << " duration_s=" << Num(r.Def.DurationS) << " control_frames=" << r.ControlFrames
            << " fdm_steps=" << r.FdmSteps
            << " altitude_step_m=" << Num(r.Def.AltitudeStepM)
            << " eas_step_mps=" << Num(r.Def.EasStepMps)
            << " reversal=" << (r.Def.bReversal ? 1 : 0) << '\n';
        out << "  baseline_window_noise pitch_ref_p2p=" << Num(r.Noise.PitchRefP2p)
            << " throttle_ref_p2p=" << Num(r.Noise.ThrottleRefP2p)
            << " elevator_cmd_p2p=" << Num(r.Noise.ElevatorCmdP2p)
            << " throttle_cmd_p2p=" << Num(r.Noise.ThrottleCmdP2p)
            << " roll_ref_p2p=" << Num(r.Noise.RollRefP2p)
            << " q_p2p=" << Num(r.Noise.QP2p)
            << " climb_p2p=" << Num(r.Noise.ClimbP2p)
            << " altitude_p2p=" << Num(r.Noise.AltitudeP2p)
            << " eas_p2p=" << Num(r.Noise.EasP2p) << '\n';
        out << "  drift altitude_m=" << Num(r.Noise.AltitudeDriftM)
            << " eas_mps=" << Num(r.Noise.EasDriftMps)
            << " pitch_rad=" << Num(r.Noise.PitchDriftRad)
            << " altitude_drift_rate_mps=" << Num(r.Noise.AltitudeDriftRateMps)
            << " tail_altitude_error_slope_mps=" << Num(r.AltitudeErrorSlopeTailMps) << '\n';
        out << "  errors initial_altitude_error_m=" << Num(r.InitialAltitudeErrorM)
            << " final_altitude_error_m=" << Num(r.FinalAltitudeErrorM)
            << " reversal_altitude_error_m=" << Num(r.ReversalAltitudeErrorM)
            << " initial_eas_error_mps=" << Num(r.InitialEasErrorMps)
            << " final_eas_error_mps=" << Num(r.FinalEasErrorMps)
            << " max_abs_altitude_error_m=" << Num(r.MaxAbsAltitudeErrorM) << '\n';
        out << "  direction_detected_s pitch_reference=" << OptTime(r.DetectPitchRefS)
            << " elevator_cmd=" << OptTime(r.DetectElevatorS)
            << " q=" << OptTime(r.DetectQS)
            << " climb_rate=" << OptTime(r.DetectClimbS)
            << " altitude=" << OptTime(r.DetectAltitudeS)
            << " throttle_reference=" << OptTime(r.DetectThrottleRefS)
            << " throttle_cmd=" << OptTime(r.DetectThrottleCmdS)
            << " eas=" << OptTime(r.DetectEasS) << '\n';
        if (r.Def.bReversal) {
            out << "  reversal_detected_s pitch_reference=" << OptTime(r.ReversalDetectPitchRefS)
                << " elevator_cmd=" << OptTime(r.ReversalDetectElevatorS)
                << " q=" << OptTime(r.ReversalDetectQS)
                << " climb_rate=" << OptTime(r.ReversalDetectClimbS) << '\n';
        }
        out << "  ranges pitch_reference_rad=[" << Num(r.PitchRefMin) << ',' << Num(r.PitchRefMax) << ']'
            << " throttle_reference_norm=[" << Num(r.ThrottleRefMin) << ',' << Num(r.ThrottleRefMax) << ']'
            << " elevator_cmd_norm=[" << Num(r.ElevatorCmdMin) << ',' << Num(r.ElevatorCmdMax) << ']'
            << " throttle_cmd_norm=[" << Num(r.ThrottleCmdMin) << ',' << Num(r.ThrottleCmdMax) << ']'
            << " roll_reference_abs_max_rad=" << Num(r.RollRefAbsMax)
            << " peak_climb_mps=" << Num(r.PeakClimbMps)
            << " peak_sink_mps=" << Num(r.PeakSinkMps) << '\n';
        out << "  settled_pre_step_window=[" << Num(kSettleEndS) << ',' << Num(kStepTimeS) << ')'
            << " pre_step_eas_mean_mps=" << Num(r.RefEas)
            << " pre_step_throttle_reference_mean=" << Num(r.RefThrottleRef)
            << " pre_step_throttle_cmd_mean=" << Num(r.RefThrottleCmd)
            << " pre_step_pitch_reference_mean_rad=" << Num(r.RefPitchRef)
            << " pre_step_climb_mean_mps=" << Num(r.RefClimb)
            << " requested_eas_after_step_mps=" << Num(r.Trace.empty() ? kNa : r.Trace.back().EasSetpointMps)
            << " tail_eas_mean_mps=" << Num(r.TailEas)
            << " throttle_reference_delta=" << Num(r.ThrottleRefDelta)
            << " throttle_cmd_delta=" << Num(r.ThrottleCmdDelta)
            << " eas_response_delta_mps=" << Num(r.EasResponseDelta) << '\n';
        out << "  energy_rates ste_rate_sp_at_step=" << Num(r.SteRateSpAtStep)
            << " ste_rate_estimate_at_step=" << Num(r.SteRateEstAtStep)
            << " ste_rate_sp_final=" << Num(r.SteRateSpFinal)
            << " ste_rate_estimate_final=" << Num(r.SteRateEstFinal)
            << " tas_rate_control=NA_not_exposed"
            << " underspeed_ratio_max=" << Num(r.UnderspeedMax)
            << " fast_descend_ratio_max=" << Num(r.FastDescendMax) << '\n';
        out << "  diagnostics throttle_saturated_frames=" << r.ThrottleSaturatedFrames
            << " pitch_reference_saturated_frames=" << r.PitchSaturatedFrames
            << " elevator_saturated_frames=" << r.ElevatorSaturatedFrames
            << " underspeed_frames=" << r.UnderspeedFrames
            << " fast_descend_frames=" << r.FastDescendFrames << '\n';
        out << "  quality non_finite_states=" << r.NonFiniteStates
            << " unexpected_wow=" << r.UnexpectedWow
            << " configuration_violations=" << r.ConfigViolations
            << " command_range_violations=" << r.CommandRangeViolations
            << " command_slew_violations=" << r.CommandSlewViolations
            << " timestamp_regressions=" << r.TimestampRegressions
            << " reset_generation_mismatches=" << r.ResetGenerationMismatches
            << " fdm_run_failures=" << r.Counters.FdmRunFailures
            << " expected_reset_frames=" << r.Counters.ExpectedResetFrames
            << " unexpected_invalid_coordinator_frames=" << r.Counters.UnexpectedInvalidCoordinatorFrames
            << " unexpected_invalid_stick_frames=" << r.Counters.UnexpectedInvalidStickFrames << '\n';
    }

    out << "write_accounting coordinator_updates=" << total.CoordinatorUpdates
        << " stick_updates=" << total.StickUpdates
        << " fdm_runs=" << total.FdmRuns
        << " command_frames=" << total.CommandFrames
        << " aileron_writes=" << total.AileronWrites
        << " elevator_writes=" << total.ElevatorWrites
        << " rudder_writes=" << total.RudderWrites
        << " throttle_writes=" << total.ThrottleWrites
        << " expected_reset_frames=" << total.ExpectedResetFrames
        << " unexpected_invalid_frames=" << (total.UnexpectedInvalidCoordinatorFrames +
                                             total.UnexpectedInvalidStickFrames)
        << " writes_outside_owned_fdm=" << total.WritesOutsideOwnedFdm
        << " production_writer_invocations=" << total.ProductionWriterInvocations
        << " fdm_run_failures=" << total.FdmRunFailures << '\n';
    out << "isolation ue_world_loaded=0 game_pawns_searched=0 active_connection=0 udp_bt_bridge_access=0 "
           "legacy_writer_access=0 surface_position_direct_writes=0 aerodynamic_property_direct_writes=0 "
           "test_owned_fgfdmexec_only=1\n";
    out << "checks=" << audit.Checks << " failures=" << audit.Failures << '\n';
    for (const std::string &m : audit.FailureMessages) out << "  failure: " << m << '\n';

    const std::string summary = out.str();
    std::ofstream sf(argv[4]);
    sf << summary;
    std::fputs(summary.c_str(), stdout);
    std::printf("TECS_F16_LONGITUDINAL_CLOSED_LOOP_V2_RESULT=%s\n", audit.Failures == 0 ? "PASS" : "FAIL");
    return audit.Failures == 0 ? 0 : 1;
}
