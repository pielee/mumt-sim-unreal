// verify_f16_vertical_performance_sensitivity_v2.cpp
//
// F-16 vertical-performance sensitivity matrix: the V1 fixed-throttle equilibrium method extended
// across the committed altitude x fuel grid.
//
// This matrix characterizes fixed-throttle, constant-EAS solver equilibria across selected altitude
// and fuel conditions. It is not an operational vertical-performance envelope and is not used
// directly as a TECS parameter set.
//
// Solver feasibility, absolute model-source validity, military-table coverage, and compatibility
// with current static TECS/Stick bounds are reported separately.
//
// Static policy-bound compatibility does not prove dynamic closed-loop achievability.
//
// PX4 fw_performance_model source is absent from the pinned reference. No PX4 density-correction
// law is inferred or applied.
//
// METHOD (identical to the committed V1 harness, bb9af6a):
//   FGTrim trim(exec, tCustom); trim.SetTolerance(1.0e-3); trim.ClearStates();
//     AddState(tUdot, tGamma)      <-- longitudinal accel zeroed by FLIGHT PATH ANGLE, not thrust
//     AddState(tWdot, tAlpha)  AddState(tQdot, tElevator)
//     AddState(tVdot, tPhi)    AddState(tPdot, tAileron)
//     AddState(tRdot, tRudder) AddState(tHmgt, tBeta)
//   The airspeed is pinned by the IC, the throttle command is pinned by the caller, and gamma is the
//   solver variable. tFull cannot be used for the vertical profiles: with throttle as the solver
//   variable it reports failure at exactly the endpoints being characterized (throttle pinned at 0
//   or 1). tFull is used ONLY for the gamma = 0 LevelReference points.
//
// GRID (read out of the committed trim sensitivity matrix, 3824ea2, not assumed):
//   altitude 5000/10000/20000/30000/40000 ft (outer loop), fuel 1000/3000/6000 lb (inner loop),
//   fuel split evenly over internal tanks 0 and 1, external tanks 2 and 3 empty. Fuel is set as tank
//   contents; the total mass is never forced.
//
// THROTTLE MAPPING (read out of the shipped model):
//   f16.xml FCS pure_gain: fcs/throttle-pos-norm = 2 * fcs/throttle-cmd-norm
//   F100-PW-229.xml: <augmented>1</augmented> <augmethod>2</augmethod>
//   FGTurbine.h: "2 = throttle range is expanded in the FCS, and values above 1.0 are afterburner"
//   => cmd 0.0 = idle/windmilling, cmd 0.5 = maximum dry/military (pos 1.0),
//      cmd 1.0 = maximum augmented/full afterburner (pos 2.0)
//
// Every point owns a fresh process AND a fresh FGFDMExec. No FDM/FCS/engine/fuel/command/timestamp
// state is shared. No UE World, Pawn, or production writer is touched, and no surface position or
// aerodynamic coefficient is ever written.
//
// PROCESS ISOLATION: the pinned JSBSim static library is built with its assertions enabled, and
// FGTrim::checkLimits can drive an aero table lookup to a non-finite argument at extreme
// fixed-throttle points (observed at 10000 ft / 6000 lb, full afterburner, 110 m/s EAS), which calls
// abort() from inside the library. abort() cannot be caught, so one such point would kill the whole
// 420-point scan. Each point therefore runs in its own child process and streams its record back to
// the parent. A point whose solver aborts its child is reported as TrimFailed with
// process_survived = 0 and an explicit diagnostic -- never as an equilibrium, and never with a
// substitute trim. Tolerances, classification rules, and the trim method are unchanged.
#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "initialization/FGTrim.h"
#include "models/FGAccelerations.h"
#include "models/FGAtmosphere.h"
#include "models/FGAuxiliary.h"
#include "models/FGFCS.h"
#include "models/FGGroundReactions.h"
#include "models/FGMassBalance.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"
#include "models/propulsion/FGTank.h"
#include "models/propulsion/FGTurbine.h"
#include "simgear/misc/sg_path.hxx"

#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

constexpr double kFdmDtS = 1.0 / 120.0;
constexpr double kFtToM = 0.3048;
constexpr double kKnotToMps = 0.5144444444444444;
constexpr double kSlugToKg = 14.593902937206363;

constexpr double kLatitudeDeg = 47.0;
constexpr double kLongitudeDeg = -122.0;
constexpr double kHeadingDeg = 90.0;
constexpr double kInternalTankCapacityLb = 3486.0;

// ---- tolerances, identical to the committed V1 harness (never relaxed to fit results) ----------
constexpr double kTrimTolerance = 1.0e-3;
constexpr double kLinearResidualTol = kTrimTolerance;          // ft/s^2
constexpr double kAngularResidualTol = kTrimTolerance / 10.0;  // rad/s^2
constexpr double kClimbConsistencyTolMps = 1.0e-9;
constexpr double kEasMismatchTolMps = 1.0e-6;
constexpr double kLevelClimbThresholdMps = 1.0e-3;
constexpr double kFiniteLimit = 1.0e12;

// ---- model source-table bounds, read out of the shipped files ----------------------------------
constexpr double kAeroAlphaMinRad = -0.5236;
constexpr double kAeroAlphaMaxRad = 0.7850;
constexpr double kEngineMachMaxAug = 2.6;
constexpr double kEngineMachMaxMil = 1.4;
constexpr double kEngineDensityAltMinFt = -10000.0;
constexpr double kEngineDensityAltMaxFt = 60000.0;
constexpr double kSpeedbrakeAutoAlphaDeg = 53.0;
constexpr double kSpeedbrakeAutoVFps = 18.0;

// ---- current production policy, READ-ONLY (FGuidanceConfigV2 / FF16StickConfigV2 defaults) ------
constexpr double kPolicyPitchMinRad = -0.5;
constexpr double kPolicyPitchMaxRad = 0.5;
constexpr double kPolicyThrottleMin = 0.0;
constexpr double kPolicyThrottleMax = 1.0;   // admits full afterburner; recorded, not decided
constexpr double kPolicySurfaceCmdLimit = 1.0;

// ---- baseline condition, reproducing the committed V1 harness (bb9af6a) -------------------------
constexpr double kBaselineAltitudeFt = 10000.0;
constexpr double kBaselineFuelLb = 3000.0;
constexpr std::size_t kPointsPerCondition = 28;
constexpr std::size_t kExpectedConditions = 15;

const char *kInterpretation =
    "This matrix characterizes fixed-throttle, constant-EAS solver equilibria across selected "
    "altitude and fuel conditions. It is not an operational vertical-performance envelope and is "
    "not used directly as a TECS parameter set.";
const char *kSeparation =
    "Solver feasibility, absolute model-source validity, military-table coverage, and compatibility "
    "with current static TECS/Stick bounds are reported separately.";
const char *kPolicyCaveat =
    "Static policy-bound compatibility does not prove dynamic closed-loop achievability.";
const char *kDensityCaveat =
    "PX4 fw_performance_model source is absent from the pinned reference. No PX4 density-correction "
    "law is inferred or applied.";

bool Finite(double v) { return std::isfinite(v) && std::abs(v) < kFiniteLimit; }
constexpr double kNa = std::numeric_limits<double>::quiet_NaN();

enum class EProfile : std::uint8_t { LevelReference, Idle, Military, Augmented };
const char *ProfileName(EProfile p)
{
    switch (p) {
    case EProfile::LevelReference: return "LevelReference";
    case EProfile::Idle: return "Idle";
    case EProfile::Military: return "Military";
    case EProfile::Augmented: return "Augmented";
    }
    return "LevelReference";
}

enum class EClass : std::uint8_t {
    LevelFeasible, SteadyClimbFeasible, SteadySinkFeasible,
    TrimFailed, SteadyStateNotReached, AirspeedNotHeld, ThrottleConditionMismatch,
    ControlSaturated, ModelRangeExceeded, NonFinite, UnexpectedWOW, FdmRunFailed, InvalidInput
};
const char *ClassName(EClass c)
{
    switch (c) {
    case EClass::LevelFeasible: return "LevelFeasible";
    case EClass::SteadyClimbFeasible: return "SteadyClimbFeasible";
    case EClass::SteadySinkFeasible: return "SteadySinkFeasible";
    case EClass::TrimFailed: return "TrimFailed";
    case EClass::SteadyStateNotReached: return "SteadyStateNotReached";
    case EClass::AirspeedNotHeld: return "AirspeedNotHeld";
    case EClass::ThrottleConditionMismatch: return "ThrottleConditionMismatch";
    case EClass::ControlSaturated: return "ControlSaturated";
    case EClass::ModelRangeExceeded: return "ModelRangeExceeded";
    case EClass::NonFinite: return "NonFinite";
    case EClass::UnexpectedWOW: return "UnexpectedWOW";
    case EClass::FdmRunFailed: return "FdmRunFailed";
    case EClass::InvalidInput: return "InvalidInput";
    }
    return "InvalidInput";
}
constexpr std::array<EClass, 13> kAllClasses{
    EClass::LevelFeasible, EClass::SteadyClimbFeasible, EClass::SteadySinkFeasible,
    EClass::TrimFailed, EClass::SteadyStateNotReached, EClass::AirspeedNotHeld,
    EClass::ThrottleConditionMismatch, EClass::ControlSaturated, EClass::ModelRangeExceeded,
    EClass::NonFinite, EClass::UnexpectedWOW, EClass::FdmRunFailed, EClass::InvalidInput};

bool Equilibrium(EClass c)
{
    return c == EClass::LevelFeasible || c == EClass::SteadyClimbFeasible || c == EClass::SteadySinkFeasible;
}

// ---- condition grid: read out of the committed trim sensitivity matrix (3824ea2) ----------------
struct FCondition {
    int Id{}, AltIndex{}, FuelIndex{};
    double AltitudeFt{}, RequestedFuelLb{};
    double Tank0Lb{}, Tank1Lb{}, Tank2Lb{}, Tank3Lb{};
    bool IsBaseline() const
    {
        return std::abs(AltitudeFt - kBaselineAltitudeFt) < 1e-9 &&
               std::abs(RequestedFuelLb - kBaselineFuelLb) < 1e-9;
    }
};

std::vector<FCondition> BuildConditions()
{
    const std::array<double, 5> altitudesFt{5000.0, 10000.0, 20000.0, 30000.0, 40000.0};
    const std::array<double, 3> fuelLb{1000.0, 3000.0, 6000.0};
    std::vector<FCondition> conditions;
    int id = 0;
    for (std::size_t a = 0; a < altitudesFt.size(); ++a) {
        for (std::size_t f = 0; f < fuelLb.size(); ++f) {
            FCondition c{};
            c.Id = id++;
            c.AltIndex = static_cast<int>(a);
            c.FuelIndex = static_cast<int>(f);
            c.AltitudeFt = altitudesFt[a];
            c.RequestedFuelLb = fuelLb[f];
            c.Tank0Lb = fuelLb[f] * 0.5;
            c.Tank1Lb = fuelLb[f] * 0.5;
            c.Tank2Lb = 0.0;
            c.Tank3Lb = 0.0;
            conditions.push_back(c);
        }
    }
    return conditions;
}

struct FPoint {
    // identity
    int GlobalId{};
    int ConditionId{}, AltIndex{}, FuelIndex{};
    int IdInCondition{};                       // 0..27, matches the V1 point_id ordering exactly
    double AltitudeFt{}, RequestedFuelLb{};

    EProfile Profile{EProfile::LevelReference};
    double RequestedThrottleCmd{};
    double RequestedEasMps{};
    EClass Classification{EClass::InvalidInput};
    char Diagnostic[192]{};   // fixed-size: the record is shipped over a pipe from an isolated child

    bool bAllAddStatesOk{}, bTrimAttempted{}, bTrimSuccess{}, bRunIcOk{};

    double ActualEasMps{kNa}, TasMps{kNa}, Mach{kNa}, AltitudeM{kNa}, DensityAltitudeFt{kNa};
    double DensitySlugFt3{kNa}, DensityRatio{kNa}, SoundSpeedMps{kNa};
    double GammaRad{kNa}, ClimbRateMps{kNa}, ReconstructedClimbMps{kNa}, ClimbMismatchMps{kNa};
    double PitchRad{kNa}, RollRad{kNa}, AlphaRad{kNa}, BetaRad{kNa};
    double P{kNa}, Q{kNa}, R{kNa}, LoadFactor{kNa};
    double ElevatorCmd{kNa}, StabilatorPosRad{kNa};
    double AileronCmd{kNa}, AileronPosRad{kNa};
    double RudderCmd{kNa}, RudderPosRad{kNa};
    double EasMismatchMps{kNa};

    double MassSlug{kNa}, MassKg{kNa}, WeightLb{kNa}, FuelLb{kNa};
    std::array<double, 4> TankLb{{kNa, kNa, kNa, kNa}};
    double CgXIn{kNa}, CgYIn{kNa}, CgZIn{kNa};
    double GearPos{kNa}, FlapPosNorm{kNa}, SpeedbrakePos{kNa}, WindMps{0.0};

    double ThrottleCmdNorm{kNa}, ThrottlePosNorm{kNa}, ThrustLb{kNa};
    bool bEngineRunning{};
    int EnginePhase{-1};
    bool bTurbineAvailable{}, bAugmentation{};
    double N1{kNa}, N2{kNa};
    double LevelTrimThrottleAtSameEas{kNa};
    bool bLevelTrimThrottleAvailable{};

    double Udot{kNa}, Vdot{kNa}, Wdot{kNa}, Pdot{kNa}, Qdot{kNa}, Rdot{kNa};
    double MaxLinearResidual{kNa}, MaxAngularResidual{kNa};

    bool bPitchPolicyExceeded{}, bThrottlePolicyExceeded{};
    bool bElevatorLimitExceeded{}, bAileronLimitExceeded{}, bRudderLimitExceeded{};
    bool bCommandableUnderCurrentPolicy{};

    bool bAlphaTableExceeded{}, bEngineMachTableExceeded{}, bDensityAltTableExceeded{};
    bool bAboveMilitaryMachTable{};
    bool bAbsoluteSourceValid{}, bSourceRangeValidForProfile{};
    bool bMilitarySourceRangeApplicable{}, bMilitarySourceRangeValid{};

    bool bWow{}, bSpeedbrakeUnexpected{};
    std::uint64_t ExternalWriteAttempts{};
    bool bProcessSurvived{true};
};
static_assert(std::is_trivially_copyable<FPoint>::value, "FPoint is shipped over a pipe as raw bytes");

void SetDiag(FPoint &p, const std::string &text)
{
    std::snprintf(p.Diagnostic, sizeof(p.Diagnostic), "%s", text.c_str());
}

class FVerticalPlant {
public:
    bool Load(const std::string &root, const FCondition &cond, FPoint &p)
    {
        Exec = std::make_unique<JSBSim::FGFDMExec>();
        Owned = Exec.get();
        Exec->SetDebugLevel(0);
        Exec->SetRootDir(SGPath(root));
        Exec->SetAircraftPath(SGPath("aircraft"));
        Exec->SetEnginePath(SGPath("engine"));
        Exec->SetSystemsPath(SGPath("systems"));
        if (!Exec->LoadModel("f16")) { SetDiag(p, "LoadModel(f16) failed"); return false; }
        Exec->Setdt(kFdmDtS);
        IC = Exec->GetIC(); Fcs = Exec->GetFCS(); Prop = Exec->GetPropagate();
        Aux = Exec->GetAuxiliary(); Atm = Exec->GetAtmosphere(); Pls = Exec->GetPropulsion();
        Grd = Exec->GetGroundReactions(); Mass = Exec->GetMassBalance(); Acc = Exec->GetAccelerations();
        if (!IC || !Fcs || !Prop || !Aux || !Atm || !Pls || !Grd || !Mass || !Acc) {
            SetDiag(p, "required JSBSim model pointer is null"); return false;
        }
        if (Pls->GetNumTanks() != 4) { SetDiag(p, "f16 model does not declare the expected 4 fuel tanks"); return false; }
        const std::array<double, 4> want{cond.Tank0Lb, cond.Tank1Lb, cond.Tank2Lb, cond.Tank3Lb};
        for (unsigned i = 0; i < 4; ++i) {
            auto tank = Pls->GetTank(i);
            if (!tank) { SetDiag(p, "null fuel tank"); return false; }
            if (want[i] < 0.0 || want[i] > tank->GetCapacity() + 1e-9) {
                SetDiag(p, "requested tank fuel exceeds the declared model capacity"); return false;
            }
            tank->SetContents(want[i]);
        }
        return true;
    }

    bool RunIc(const FCondition &cond, double easMps, FPoint &p)
    {
        IC->SetGeodLatitudeDegIC(kLatitudeDeg);
        IC->SetLongitudeDegIC(kLongitudeDeg);
        IC->SetAltitudeASLFtIC(cond.AltitudeFt);
        IC->SetPsiDegIC(kHeadingDeg);
        IC->SetPhiDegIC(0.0);
        IC->SetThetaDegIC(0.0);
        IC->SetWindDirDegIC(0.0);
        IC->SetWindMagKtsIC(0.0);
        IC->SetWindDownKtsIC(0.0);
        Exec->SetPropertyValue("ic/p-rad_sec", 0.0);
        Exec->SetPropertyValue("ic/q-rad_sec", 0.0);
        Exec->SetPropertyValue("ic/r-rad_sec", 0.0);
        IC->SetVequivalentKtsIC(easMps / kKnotToMps);
        IC->SetFlightPathAngleDegIC(0.0);          // initial guess only; gamma is a solver variable
        p.bRunIcOk = Exec->RunIC();
        if (!p.bRunIcOk) { SetDiag(p, "RunIC failed"); return false; }
        Pls->InitRunning(-1);
        Pls->SetFuelFreeze(true);
        Fcs->SetGearCmd(0.0); Fcs->SetGearPos(0.0);
        Fcs->SetDfCmd(0.0); Fcs->SetDfPos(JSBSim::ofNorm, 0.0);
        Fcs->SetDsbCmd(0.0);                       // speedbrake retracted, clean configuration
        return true;
    }

    // gamma = 0 horizontal-equilibrium reference. Never reused as a fixed-throttle vertical profile
    // and never recommended as a production throttle_trim.
    bool BeginLevelTrim(FPoint &p)
    {
        Fcs->SetThrottleCmd(0, 0.6);
        p.bAllAddStatesOk = true;
        p.bTrimAttempted = true;
        Trim = std::make_unique<JSBSim::FGTrim>(Exec.get(), JSBSim::tFull);
        Trim->SetTolerance(kTrimTolerance);
        return true;
    }

    bool BeginFixedThrottleTrim(double throttleCmd, FPoint &p)
    {
        Fcs->SetThrottleCmd(0, throttleCmd);
        p.bTrimAttempted = true;
        Trim = std::make_unique<JSBSim::FGTrim>(Exec.get(), JSBSim::tCustom);
        Trim->SetTolerance(kTrimTolerance);
        Trim->ClearStates();
        const bool a1 = Trim->AddState(JSBSim::tUdot, JSBSim::tGamma);
        const bool a2 = Trim->AddState(JSBSim::tWdot, JSBSim::tAlpha);
        const bool a3 = Trim->AddState(JSBSim::tQdot, JSBSim::tElevator);
        const bool a4 = Trim->AddState(JSBSim::tVdot, JSBSim::tPhi);
        const bool a5 = Trim->AddState(JSBSim::tPdot, JSBSim::tAileron);
        const bool a6 = Trim->AddState(JSBSim::tRdot, JSBSim::tRudder);
        const bool a7 = Trim->AddState(JSBSim::tHmgt, JSBSim::tBeta);
        p.bAllAddStatesOk = a1 && a2 && a3 && a4 && a5 && a6 && a7;
        if (!p.bAllAddStatesOk) { SetDiag(p, "FGTrim::AddState failed"); return false; }
        return true;
    }

    bool ExecuteTrim(FPoint &p)
    {
        p.bTrimSuccess = Trim->DoTrim();
        return p.bTrimSuccess;
    }

    // Condition-level state only: what the point IS, independent of whether the solver converges.
    // Captured before DoTrim so an isolated child that the third-party solver aborts still reports
    // its mass, fuel and atmosphere instead of dropping the row.
    void ReadCondition(FPoint &p) const
    {
        p.AltitudeM = Prop->GetAltitudeASL() * kFtToM;
        p.DensityAltitudeFt = Exec->GetPropertyValue("atmosphere/density-altitude");
        p.DensitySlugFt3 = Atm->GetDensity();
        p.DensityRatio = Atm->GetDensityRatio();
        p.SoundSpeedMps = Atm->GetSoundSpeed() * kFtToM;
        p.MassSlug = Mass->GetMass();
        p.MassKg = p.MassSlug * kSlugToKg;
        p.WeightLb = Mass->GetWeight();
        p.FuelLb = Exec->GetPropertyValue("propulsion/total-fuel-lbs");
        for (unsigned i = 0; i < 4; ++i) {
            auto tank = Pls->GetTank(i);
            p.TankLb[i] = tank ? tank->GetContents() : kNa;
        }
        p.CgXIn = Mass->GetXYZcg(1); p.CgYIn = Mass->GetXYZcg(2); p.CgZIn = Mass->GetXYZcg(3);
        p.GearPos = Fcs->GetGearPos();
        p.FlapPosNorm = Fcs->GetDfPos(JSBSim::ofNorm);
        p.SpeedbrakePos = Fcs->GetDsbPos(JSBSim::ofNorm);
        p.WindMps = 0.0;
    }

    void Read(FPoint &p) const
    {
        p.ActualEasMps = Aux->GetVequivalentKTS() * kKnotToMps;
        p.TasMps = Aux->GetVt() * kFtToM;
        p.Mach = Aux->GetMach();
        p.AltitudeM = Prop->GetAltitudeASL() * kFtToM;
        p.DensityAltitudeFt = Exec->GetPropertyValue("atmosphere/density-altitude");
        p.DensitySlugFt3 = Atm->GetDensity();
        p.DensityRatio = Atm->GetDensityRatio();
        p.SoundSpeedMps = Atm->GetSoundSpeed() * kFtToM;
        p.GammaRad = Exec->GetPropertyValue("flight-path/gamma-rad");
        p.ClimbRateMps = Prop->Gethdot() * kFtToM;
        p.ReconstructedClimbMps = p.TasMps * std::sin(p.GammaRad);
        p.ClimbMismatchMps = std::abs(p.ClimbRateMps - p.ReconstructedClimbMps);
        p.PitchRad = Prop->GetEuler(JSBSim::FGJSBBase::eTht);
        p.RollRad = Prop->GetEuler(JSBSim::FGJSBBase::ePhi);
        p.AlphaRad = Aux->Getalpha();
        p.BetaRad = Aux->Getbeta();
        p.P = Prop->GetPQR(1); p.Q = Prop->GetPQR(2); p.R = Prop->GetPQR(3);
        p.LoadFactor = Aux->GetNlf();
        p.ElevatorCmd = Fcs->GetDeCmd(); p.StabilatorPosRad = Fcs->GetDePos(JSBSim::ofRad);
        p.AileronCmd = Fcs->GetDaCmd(); p.AileronPosRad = Fcs->GetDaLPos(JSBSim::ofRad);
        p.RudderCmd = Fcs->GetDrCmd(); p.RudderPosRad = Fcs->GetDrPos(JSBSim::ofRad);
        p.EasMismatchMps = std::abs(p.ActualEasMps - p.RequestedEasMps);

        p.MassSlug = Mass->GetMass();
        p.MassKg = p.MassSlug * kSlugToKg;
        p.WeightLb = Mass->GetWeight();
        p.FuelLb = Exec->GetPropertyValue("propulsion/total-fuel-lbs");
        for (unsigned i = 0; i < 4; ++i) {
            auto tank = Pls->GetTank(i);
            p.TankLb[i] = tank ? tank->GetContents() : kNa;
        }
        p.CgXIn = Mass->GetXYZcg(1); p.CgYIn = Mass->GetXYZcg(2); p.CgZIn = Mass->GetXYZcg(3);
        p.GearPos = Fcs->GetGearPos();
        p.FlapPosNorm = Fcs->GetDfPos(JSBSim::ofNorm);
        p.SpeedbrakePos = Fcs->GetDsbPos(JSBSim::ofNorm);
        p.WindMps = 0.0;

        p.ThrottleCmdNorm = Fcs->GetThrottleCmd(0);
        p.ThrottlePosNorm = Fcs->GetThrottlePos(0);
        auto engine = Pls->GetEngine(0);
        p.bEngineRunning = engine ? engine->GetRunning() : false;
        p.ThrustLb = engine ? engine->GetThrust() : kNa;
        auto turbine = std::dynamic_pointer_cast<JSBSim::FGTurbine>(engine);
        p.bTurbineAvailable = static_cast<bool>(turbine);
        if (turbine) {
            p.EnginePhase = static_cast<int>(turbine->GetPhase());
            p.bAugmentation = turbine->GetAugmentation();
            p.N1 = turbine->GetN1();
            p.N2 = turbine->GetN2();
        }

        p.Udot = Acc->GetUVWdot(1); p.Vdot = Acc->GetUVWdot(2); p.Wdot = Acc->GetUVWdot(3);
        p.Pdot = Acc->GetPQRdot(1); p.Qdot = Acc->GetPQRdot(2); p.Rdot = Acc->GetPQRdot(3);
        p.MaxLinearResidual = std::max({std::abs(p.Udot), std::abs(p.Vdot), std::abs(p.Wdot)});
        p.MaxAngularResidual = std::max({std::abs(p.Pdot), std::abs(p.Qdot), std::abs(p.Rdot)});

        p.bWow = Grd->GetWOW();
        const double alphaDeg = p.AlphaRad * 180.0 / M_PI;
        const double vFps = Aux->GetVt();
        const bool deepStall = alphaDeg >= kSpeedbrakeAutoAlphaDeg && vFps <= kSpeedbrakeAutoVFps;
        p.bSpeedbrakeUnexpected = Finite(p.SpeedbrakePos) && p.SpeedbrakePos > 1e-9 && !deepStall;
    }

    bool StateFinite(const FPoint &p) const
    {
        const std::array<double, 24> v{
            p.ActualEasMps, p.TasMps, p.Mach, p.AltitudeM, p.DensityAltitudeFt, p.DensitySlugFt3,
            p.SoundSpeedMps, p.GammaRad, p.ClimbRateMps, p.PitchRad, p.RollRad, p.AlphaRad, p.BetaRad,
            p.P, p.Q, p.R, p.LoadFactor, p.ElevatorCmd, p.StabilatorPosRad, p.AileronCmd,
            p.RudderCmd, p.ThrottleCmdNorm, p.ThrottlePosNorm, p.ThrustLb};
        return std::all_of(v.begin(), v.end(), Finite);
    }

    bool OwnedIdentityOk() const { return Exec.get() == Owned; }

private:
    std::unique_ptr<JSBSim::FGFDMExec> Exec;
    std::unique_ptr<JSBSim::FGTrim> Trim;
    JSBSim::FGFDMExec *Owned{};
    std::shared_ptr<JSBSim::FGInitialCondition> IC;
    std::shared_ptr<JSBSim::FGFCS> Fcs;
    std::shared_ptr<JSBSim::FGPropagate> Prop;
    std::shared_ptr<JSBSim::FGAuxiliary> Aux;
    std::shared_ptr<JSBSim::FGAtmosphere> Atm;
    std::shared_ptr<JSBSim::FGPropulsion> Pls;
    std::shared_ptr<JSBSim::FGGroundReactions> Grd;
    std::shared_ptr<JSBSim::FGMassBalance> Mass;
    std::shared_ptr<JSBSim::FGAccelerations> Acc;
};

void ApplyFlags(FPoint &p)
{
    // Layer 2: compatibility with the CURRENT static production bounds. Read-only. This checks only
    // static reference and command bounds; it does not prove closed-loop dynamic achievability.
    p.bPitchPolicyExceeded = Finite(p.PitchRad) &&
                             (p.PitchRad < kPolicyPitchMinRad || p.PitchRad > kPolicyPitchMaxRad);
    p.bThrottlePolicyExceeded = Finite(p.ThrottleCmdNorm) &&
                                (p.ThrottleCmdNorm < kPolicyThrottleMin - 1e-12 ||
                                 p.ThrottleCmdNorm > kPolicyThrottleMax + 1e-12);
    p.bElevatorLimitExceeded = Finite(p.ElevatorCmd) && std::abs(p.ElevatorCmd) > kPolicySurfaceCmdLimit + 1e-12;
    p.bAileronLimitExceeded = Finite(p.AileronCmd) && std::abs(p.AileronCmd) > kPolicySurfaceCmdLimit + 1e-12;
    p.bRudderLimitExceeded = Finite(p.RudderCmd) && std::abs(p.RudderCmd) > kPolicySurfaceCmdLimit + 1e-12;
    p.bCommandableUnderCurrentPolicy = !p.bPitchPolicyExceeded && !p.bThrottlePolicyExceeded &&
                                       !p.bElevatorLimitExceeded && !p.bAileronLimitExceeded &&
                                       !p.bRudderLimitExceeded;

    // (A) absolute model source range -- exceeding this is ModelRangeExceeded.
    p.bAlphaTableExceeded = Finite(p.AlphaRad) &&
                            (p.AlphaRad < kAeroAlphaMinRad || p.AlphaRad > kAeroAlphaMaxRad);
    p.bEngineMachTableExceeded = Finite(p.Mach) && (p.Mach < 0.0 || p.Mach > kEngineMachMaxAug);
    p.bDensityAltTableExceeded = Finite(p.DensityAltitudeFt) &&
                                 (p.DensityAltitudeFt < kEngineDensityAltMinFt ||
                                  p.DensityAltitudeFt > kEngineDensityAltMaxFt);
    p.bAbsoluteSourceValid = !p.bAlphaTableExceeded && !p.bEngineMachTableExceeded && !p.bDensityAltTableExceeded;

    // (B) Above the MILITARY thrust table's Mach extent but still inside the augmented table's
    // absolute range. The equilibrium classification is kept; the point is simply excluded from any
    // military-power inference. This never fails the test.
    p.bAboveMilitaryMachTable = Finite(p.Mach) && p.Mach > kEngineMachMaxMil;
    p.bMilitarySourceRangeApplicable = (p.Profile == EProfile::Military);
    p.bMilitarySourceRangeValid = p.bMilitarySourceRangeApplicable
                                      ? (p.bAbsoluteSourceValid && !p.bAboveMilitaryMachTable)
                                      : false;
    p.bSourceRangeValidForProfile =
        p.bAbsoluteSourceValid && (p.Profile != EProfile::Military || !p.bAboveMilitaryMachTable);
}

FPoint MakeRequest(const FCondition &cond, int globalId, int idInCondition, EProfile profile,
                   double throttleCmd, double easMps, double levelTrimThrottle,
                   bool levelTrimThrottleAvailable)
{
    FPoint p{};
    p.GlobalId = globalId;
    p.ConditionId = cond.Id;
    p.AltIndex = cond.AltIndex;
    p.FuelIndex = cond.FuelIndex;
    p.IdInCondition = idInCondition;
    p.AltitudeFt = cond.AltitudeFt;
    p.RequestedFuelLb = cond.RequestedFuelLb;
    p.Profile = profile;
    p.RequestedThrottleCmd = throttleCmd;
    p.RequestedEasMps = easMps;
    p.LevelTrimThrottleAtSameEas = levelTrimThrottle;
    p.bLevelTrimThrottleAvailable = levelTrimThrottleAvailable;
    return p;
}

// Runs one point to completion inside the isolated child. `emit` ships the record to the parent; it
// is called once with the pre-solver record (condition state captured, solver outcome still pending)
// and once with the final record. The parent keeps the last record it received, so a point whose
// solver kills the child is still reported with its true mass, fuel and atmosphere.
void RunPointInChild(const std::string &root, const FCondition &cond, FPoint &p,
                     const std::function<void(const FPoint &)> &emit)
{
    const EProfile profile = p.Profile;
    const double throttleCmd = p.RequestedThrottleCmd;
    const double easMps = p.RequestedEasMps;

    if (!Finite(easMps) || easMps <= 0.0 || !Finite(throttleCmd) || throttleCmd < 0.0 || throttleCmd > 1.0) {
        p.Classification = EClass::InvalidInput;
        SetDiag(p, "requested EAS or throttle command is invalid");
        emit(p);
        return;
    }

    try {
        FVerticalPlant plant;
        if (!plant.Load(root, cond, p)) { p.Classification = EClass::FdmRunFailed; emit(p); return; }
        if (!plant.RunIc(cond, easMps, p)) { p.Classification = EClass::FdmRunFailed; emit(p); return; }
        if (!plant.OwnedIdentityOk()) {
            ++p.ExternalWriteAttempts;
            p.Classification = EClass::FdmRunFailed;
            SetDiag(p, "write target was not the owned FGFDMExec");
            emit(p);
            return;
        }

        const bool begun = (profile == EProfile::LevelReference) ? plant.BeginLevelTrim(p)
                                                                 : plant.BeginFixedThrottleTrim(throttleCmd, p);
        if (!begun || !p.bAllAddStatesOk) { p.Classification = EClass::InvalidInput; emit(p); return; }

        // Pre-solver record. If the third-party solver aborts this child, THIS is what the parent
        // reports, with process_survived = 0 and the diagnostic below. Nothing is substituted for
        // the trim: the point is reported as a trim failure, never as an equilibrium.
        plant.ReadCondition(p);
        p.Classification = EClass::TrimFailed;
        SetDiag(p, "FGTrim::DoTrim() aborted the isolated process (JSBSim library assertion)");
        emit(p);

        const bool trimmed = plant.ExecuteTrim(p);
        plant.Read(p);
        ApplyFlags(p);
        if (!trimmed) {
            p.Classification = EClass::TrimFailed;
            SetDiag(p, "FGTrim::DoTrim() returned false");
            emit(p);
            return;
        }
        if (!plant.StateFinite(p)) {
            p.Classification = EClass::NonFinite;
            SetDiag(p, "non-finite state after trim");
            emit(p);
            return;
        }
        if (p.bWow) {
            p.Classification = EClass::UnexpectedWOW;
            SetDiag(p, "WOW after airborne trim");
            emit(p);
            return;
        }
        if (profile != EProfile::LevelReference) {
            const bool cmdHeld = std::abs(p.ThrottleCmdNorm - throttleCmd) <= 1e-12;
            const bool posOk = std::abs(p.ThrottlePosNorm - 2.0 * throttleCmd) <= 1e-9;
            if (!cmdHeld || !posOk) {
                p.Classification = EClass::ThrottleConditionMismatch;
                SetDiag(p, cmdHeld ? "throttle position is not 2x the command"
                                   : "solver moved the throttle command");
                emit(p);
                return;
            }
        }
        if (!Finite(p.EasMismatchMps) || p.EasMismatchMps > kEasMismatchTolMps) {
            p.Classification = EClass::AirspeedNotHeld;
            SetDiag(p, "post-trim EAS does not match the requested EAS");
            emit(p);
            return;
        }
        if (!Finite(p.MaxLinearResidual) || p.MaxLinearResidual > kLinearResidualTol ||
            !Finite(p.MaxAngularResidual) || p.MaxAngularResidual > kAngularResidualTol) {
            p.Classification = EClass::SteadyStateNotReached;
            SetDiag(p, "residual acceleration exceeds the declared trim tolerance");
            emit(p);
            return;
        }
        if (!p.bAbsoluteSourceValid) {
            p.Classification = EClass::ModelRangeExceeded;
            SetDiag(p, "trim state is outside an absolute model source-table bound");
            emit(p);
            return;
        }
        if (p.bElevatorLimitExceeded || p.bAileronLimitExceeded || p.bRudderLimitExceeded ||
            std::abs(p.ElevatorCmd) >= kPolicySurfaceCmdLimit - 1e-12) {
            p.Classification = EClass::ControlSaturated;
            SetDiag(p, "a surface command is at or beyond its limit");
            emit(p);
            return;
        }
        if (!Finite(p.ClimbMismatchMps) || p.ClimbMismatchMps > kClimbConsistencyTolMps) {
            p.Classification = EClass::SteadyStateNotReached;
            SetDiag(p, "climb rate is inconsistent with TAS * sin(gamma)");
            emit(p);
            return;
        }
        if (p.ClimbRateMps > kLevelClimbThresholdMps) p.Classification = EClass::SteadyClimbFeasible;
        else if (p.ClimbRateMps < -kLevelClimbThresholdMps) p.Classification = EClass::SteadySinkFeasible;
        else p.Classification = EClass::LevelFeasible;
        SetDiag(p, "");
        emit(p);
    } catch (const std::exception &e) {
        p.Classification = EClass::TrimFailed;
        SetDiag(p, std::string("exception: ") + e.what());
        emit(p);
    } catch (...) {
        p.Classification = EClass::TrimFailed;
        SetDiag(p, "unknown exception");
        emit(p);
    }
}

bool WriteExact(int fd, const void *data, std::size_t size)
{
    const char *cursor = static_cast<const char *>(data);
    while (size > 0) {
        const ssize_t n = ::write(fd, cursor, size);
        if (n <= 0) return false;
        cursor += n;
        size -= static_cast<std::size_t>(n);
    }
    return true;
}

bool ReadExact(int fd, void *data, std::size_t size)
{
    char *cursor = static_cast<char *>(data);
    while (size > 0) {
        const ssize_t n = ::read(fd, cursor, size);
        if (n <= 0) return false;
        cursor += n;
        size -= static_cast<std::size_t>(n);
    }
    return true;
}

// Every point is solved in its own process. The pinned JSBSim static library ships with its
// assertions enabled, and FGTrim::checkLimits can drive an aero table lookup to a non-finite
// argument at extreme fixed-throttle points, which calls abort() from inside the library. abort()
// cannot be caught, so a single point would otherwise kill the whole 420-point scan. Process
// isolation strengthens the per-point isolation already required (fresh FGFDMExec, fresh model load,
// fresh IC, fresh tank/engine/FCS state, fresh trim object) and lets the harness report such a point
// truthfully instead of losing the run. It changes no tolerance, no classification rule, and no part
// of the trim method.
FPoint RunPointIsolated(const std::string &root, const FCondition &cond, int globalId, int idInCondition,
                        EProfile profile, double throttleCmd, double easMps,
                        double levelTrimThrottle, bool levelTrimThrottleAvailable)
{
    FPoint request = MakeRequest(cond, globalId, idInCondition, profile, throttleCmd, easMps,
                                 levelTrimThrottle, levelTrimThrottleAvailable);

    int fds[2] = {-1, -1};
    if (::pipe(fds) != 0) {
        request.Classification = EClass::FdmRunFailed;
        request.bProcessSurvived = false;
        SetDiag(request, "pipe() failed; the point could not be isolated");
        return request;
    }

    std::fflush(nullptr);   // never duplicate a buffered byte into the child
    const pid_t pid = ::fork();
    if (pid < 0) {
        ::close(fds[0]);
        ::close(fds[1]);
        request.Classification = EClass::FdmRunFailed;
        request.bProcessSurvived = false;
        SetDiag(request, "fork() failed; the point could not be isolated");
        return request;
    }

    if (pid == 0) {
        ::close(fds[0]);
        const rlimit noCore{0, 0};
        ::setrlimit(RLIMIT_CORE, &noCore);   // a library abort must not litter the tree with cores
        const int out = fds[1];
        FPoint p = request;
        RunPointInChild(root, cond, p, [out](const FPoint &record) {
            WriteExact(out, &record, sizeof(record));
        });
        ::close(out);
        std::fflush(nullptr);
        ::_exit(0);
    }

    ::close(fds[1]);
    FPoint received{};
    FPoint last = request;
    bool got = false;
    while (ReadExact(fds[0], &received, sizeof(received))) {
        last = received;
        got = true;
    }
    ::close(fds[0]);

    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    const bool exitedCleanly = WIFEXITED(status) && WEXITSTATUS(status) == 0;

    if (exitedCleanly && got) {
        last.bProcessSurvived = true;
        return last;
    }

    last.bProcessSurvived = false;
    if (!got) {
        // The child died before it could even report the condition it was solving.
        last = request;
        last.Classification = EClass::FdmRunFailed;
        last.bProcessSurvived = false;
        SetDiag(last, "isolated point process died before reporting any record");
        return last;
    }
    // The child reported its condition and then died inside the solver. The record already carries
    // TrimFailed and the abort diagnostic; process_survived = 0 records how it failed.
    return last;
}

void WriteNumber(std::ostream &out, double v, int precision)
{
    if (Finite(v)) out << std::fixed << std::setprecision(precision) << v;
    else out << "NA";
}

// Summary scalars: an unavailable value is reported as NA, never as 0 and never as "nan".
std::string Num(double v)
{
    if (!Finite(v)) return "NA";
    std::ostringstream s;
    s << std::fixed << std::setprecision(12) << v;
    return s.str();
}
const char *Flag(bool available, bool value) { return available ? (value ? "1" : "0") : "NA"; }

// The V1 record, byte-for-byte: same columns, same order, same precision, same point ordering.
// Used for the baseline-condition reproduction file, which is compared against the committed V1
// harness output.
void WriteV1Record(std::ostream &out, const FPoint &p, int precision)
{
    out << p.IdInCondition << ',' << ProfileName(p.Profile) << ',';
    WriteNumber(out, p.RequestedThrottleCmd, precision); out << ',';
    WriteNumber(out, p.RequestedEasMps, precision); out << ',';
    WriteNumber(out, p.ActualEasMps, precision); out << ',';
    WriteNumber(out, p.TasMps, precision); out << ',';
    WriteNumber(out, p.Mach, precision); out << ',';
    WriteNumber(out, p.AltitudeFt, precision); out << ',';
    WriteNumber(out, p.AltitudeFt * kFtToM, precision); out << ',';
    WriteNumber(out, p.DensityAltitudeFt, precision); out << ',';
    WriteNumber(out, p.DensitySlugFt3, precision); out << ',';
    WriteNumber(out, p.SoundSpeedMps, precision); out << ',';
    WriteNumber(out, p.FuelLb, precision); out << ',';
    for (std::size_t i = 0; i < 4; ++i) { WriteNumber(out, p.TankLb[i], precision); if (i + 1 < 4) out << '|'; }
    out << ',';
    const std::array<double, 7> m{p.MassSlug, p.MassKg, p.WeightLb, p.CgXIn, p.CgYIn, p.CgZIn, p.WindMps};
    for (double v : m) { WriteNumber(out, v, precision); out << ','; }
    const std::array<double, 3> cfg{p.GearPos, p.FlapPosNorm, p.SpeedbrakePos};
    for (double v : cfg) { WriteNumber(out, v, precision); out << ','; }
    out << ClassName(p.Classification) << ',' << (p.bTrimAttempted ? 1 : 0) << ','
        << (p.bTrimSuccess ? 1 : 0) << ',' << (p.bAllAddStatesOk ? 1 : 0) << ','
        << (p.bRunIcOk ? 1 : 0) << ',';
    const std::array<double, 15> tr{
        p.GammaRad, p.ClimbRateMps, p.ReconstructedClimbMps, p.ClimbMismatchMps,
        p.PitchRad, p.RollRad, p.AlphaRad, p.BetaRad, p.P, p.Q, p.R, p.LoadFactor,
        p.ElevatorCmd, p.StabilatorPosRad, p.AileronCmd};
    for (double v : tr) { WriteNumber(out, v, precision); out << ','; }
    WriteNumber(out, p.AileronPosRad, precision); out << ',';
    WriteNumber(out, p.RudderCmd, precision); out << ',';
    WriteNumber(out, p.RudderPosRad, precision); out << ',';
    WriteNumber(out, p.ThrottleCmdNorm, precision); out << ',';
    WriteNumber(out, p.ThrottlePosNorm, precision); out << ',';
    out << (p.bEngineRunning ? 1 : 0) << ',' << p.EnginePhase << ',' << (p.bTurbineAvailable ? 1 : 0)
        << ',' << (p.bTurbineAvailable ? (p.bAugmentation ? "1" : "0") : "NA") << ',';
    WriteNumber(out, p.N1, precision); out << ',';
    WriteNumber(out, p.N2, precision); out << ',';
    WriteNumber(out, p.ThrustLb, precision); out << ",1,";
    WriteNumber(out, p.LevelTrimThrottleAtSameEas, precision); out << ',';
    out << (p.bLevelTrimThrottleAvailable ? 1 : 0) << ',';
    const std::array<double, 10> res{p.Udot, p.Vdot, p.Wdot, p.Pdot, p.Qdot, p.Rdot,
                                     p.MaxLinearResidual, p.MaxAngularResidual,
                                     kLinearResidualTol, kAngularResidualTol};
    for (double v : res) { WriteNumber(out, v, precision); out << ','; }
    WriteNumber(out, kPolicyPitchMinRad, precision); out << ',';
    WriteNumber(out, kPolicyPitchMaxRad, precision); out << ',';
    out << (p.bPitchPolicyExceeded ? 1 : 0) << ',' << (p.bThrottlePolicyExceeded ? 1 : 0) << ','
        << (p.bElevatorLimitExceeded ? 1 : 0) << ',' << (p.bAileronLimitExceeded ? 1 : 0) << ','
        << (p.bRudderLimitExceeded ? 1 : 0) << ',' << (p.bCommandableUnderCurrentPolicy ? 1 : 0) << ',';
    WriteNumber(out, p.EasMismatchMps, precision); out << ',';
    out << (p.bAlphaTableExceeded ? 1 : 0) << ',' << (p.bEngineMachTableExceeded ? 1 : 0) << ','
        << (p.bDensityAltTableExceeded ? 1 : 0) << ',' << (p.bAboveMilitaryMachTable ? 1 : 0) << ','
        << (p.bWow ? 1 : 0) << ',' << (p.bSpeedbrakeUnexpected ? 1 : 0) << ','
        << (p.Classification == EClass::NonFinite ? 1 : 0) << ',' << p.ExternalWriteAttempts << ','
        << (p.bProcessSurvived ? 1 : 0) << ',' << p.Diagnostic << '\n';
}

const char *kV1Header =
    "point_id,throttle_profile,throttle_cmd_norm_requested,requested_eas_mps,actual_eas_mps,tas_mps,mach,"
    "altitude_ft,altitude_m,density_altitude_ft,density_slug_ft3,speed_of_sound_mps,fuel_lb,"
    "tank_fuel_distribution_lb,mass_slug,mass_kg,weight_lb,cg_x_in,cg_y_in,cg_z_in,wind_mps,"
    "gear_pos,flap_pos_norm,speedbrake_pos,"
    "classification,trim_attempted,trim_success,all_add_states_success,run_ic_success,"
    "gamma_rad,climb_rate_mps,reconstructed_climb_rate_mps,climb_rate_mismatch_mps,"
    "pitch_rad,roll_rad,alpha_rad,beta_rad,p_radps,q_radps,r_radps,load_factor,"
    "elevator_cmd_norm,stabilator_position_rad,aileron_cmd_norm,aileron_position_rad,"
    "rudder_cmd_norm,rudder_position_rad,"
    "throttle_cmd_norm,throttle_position_norm,engine_running,engine_phase,turbine_available,"
    "augmentation,n1,n2,thrust_lb,fuel_freeze,level_trim_throttle_at_same_eas,level_trim_throttle_available,"
    "udot_ftps2,vdot_ftps2,wdot_ftps2,pdot_radps2,qdot_radps2,rdot_radps2,"
    "max_linear_residual_ftps2,max_angular_residual_radps2,linear_residual_tol,angular_residual_tol,"
    "pitch_policy_min_rad,pitch_policy_max_rad,pitch_policy_exceeded,throttle_policy_exceeded,"
    "elevator_limit_exceeded,aileron_limit_exceeded,rudder_limit_exceeded,commandable_under_current_policy,"
    "eas_mismatch_mps,alpha_table_exceeded,engine_mach_table_exceeded,density_altitude_table_exceeded,"
    "above_military_mach_table,wow,speedbrake_deployed_unexpectedly,nonfinite,external_write_attempts,"
    "process_survived,diagnostic\n";

void WriteBaselineCsv(const std::string &path, const std::vector<FPoint> &points, int baselineId, int precision)
{
    std::ofstream out(path);
    out << kV1Header;
    for (const FPoint &p : points)
        if (p.ConditionId == baselineId) WriteV1Record(out, p, precision);
}

// The full matrix: condition columns first, then the complete V1 record.
void WriteMatrixCsv(const std::string &path, const std::vector<FPoint> &points, int precision)
{
    std::ofstream out(path);
    out << "global_point_id,condition_id,altitude_grid_index,fuel_grid_index,altitude_ft,altitude_m,"
           "fuel_total_lb,tank0_lb,tank1_lb,tank2_lb,tank3_lb,mass_slug,mass_kg,weight_lb,"
           "cg_x_in,cg_y_in,cg_z_in,density_slug_ft3,density_ratio,density_altitude_ft,"
           "absolute_source_valid,source_range_valid_for_profile,military_source_range_applicable,"
           "military_source_range_valid," << kV1Header;
    for (const FPoint &p : points) {
        out << p.GlobalId << ',' << p.ConditionId << ',' << p.AltIndex << ',' << p.FuelIndex << ',';
        WriteNumber(out, p.AltitudeFt, precision); out << ',';
        WriteNumber(out, p.AltitudeFt * kFtToM, precision); out << ',';
        WriteNumber(out, p.FuelLb, precision); out << ',';
        for (std::size_t i = 0; i < 4; ++i) { WriteNumber(out, p.TankLb[i], precision); out << ','; }
        const std::array<double, 9> cond{p.MassSlug, p.MassKg, p.WeightLb, p.CgXIn, p.CgYIn, p.CgZIn,
                                         p.DensitySlugFt3, p.DensityRatio, p.DensityAltitudeFt};
        for (double v : cond) { WriteNumber(out, v, precision); out << ','; }
        out << (p.bAbsoluteSourceValid ? 1 : 0) << ',' << (p.bSourceRangeValidForProfile ? 1 : 0) << ','
            << (p.bMilitarySourceRangeApplicable ? 1 : 0) << ','
            << (p.bMilitarySourceRangeApplicable ? (p.bMilitarySourceRangeValid ? "1" : "0") : "NA") << ',';
        WriteV1Record(out, p, precision);
    }
}

// ---- point set, identical to the committed V1 harness -------------------------------------------
const std::vector<double> kIdleSweep{70.0, 90.0, 110.0, 130.0, 150.0, 170.0,
                                     189.070713, 220.0, 235.0, 250.0, 280.0};
const std::vector<double> kPoweredEas{110.0, 189.070713, 235.0};
constexpr double kAnchorEasMps = 189.070713;

struct FKey {
    EProfile Profile;
    double Eas;
    bool operator<(const FKey &o) const
    {
        if (Profile != o.Profile) return Profile < o.Profile;
        return Eas < o.Eas;
    }
};

} // namespace

int main(int argc, char **argv)
{
    if (argc != 7) {
        std::fprintf(stderr,
                     "usage: %s <JSBSim-root> <matrix.raw.csv> <matrix.quantized.csv> "
                     "<baseline.raw.csv> <baseline.quantized.csv> <summary.txt>\n", argv[0]);
        return 2;
    }
    const std::string root = argv[1];
    const std::vector<FCondition> conditions = BuildConditions();

    std::vector<FPoint> points;
    points.reserve(conditions.size() * kPointsPerCondition);
    int globalId = 0;

    for (const FCondition &cond : conditions) {
        int idInCondition = 0;
        std::map<double, double> levelThrottle;
        std::map<double, bool> levelOk;

        // pass 1 -- LevelReference (tFull, gamma = 0) at every EAS. Horizontal-equilibrium reference
        // only; never reused as a fixed-throttle vertical profile.
        for (double eas : kIdleSweep) {
            FPoint p = RunPointIsolated(root, cond, globalId++, idInCondition++, EProfile::LevelReference,
                                        0.0, eas, kNa, false);
            levelOk[eas] = Equilibrium(p.Classification);
            levelThrottle[eas] = Equilibrium(p.Classification) ? p.ThrottleCmdNorm : kNa;
            points.push_back(p);
        }
        auto lvl = [&](double eas) {
            const auto it = levelThrottle.find(eas);
            return it == levelThrottle.end() ? kNa : it->second;
        };
        auto lvlOk = [&](double eas) {
            const auto it = levelOk.find(eas);
            return it != levelOk.end() && it->second;
        };

        // pass 2 -- fixed-throttle equilibria
        for (double eas : kIdleSweep)
            points.push_back(RunPointIsolated(root, cond, globalId++, idInCondition++, EProfile::Idle,
                                              0.0, eas, lvl(eas), lvlOk(eas)));
        for (double eas : kPoweredEas)
            points.push_back(RunPointIsolated(root, cond, globalId++, idInCondition++, EProfile::Military,
                                              0.5, eas, lvl(eas), lvlOk(eas)));
        for (double eas : kPoweredEas)
            points.push_back(RunPointIsolated(root, cond, globalId++, idInCondition++, EProfile::Augmented,
                                              1.0, eas, lvl(eas), lvlOk(eas)));
    }

    // ---- gates -----------------------------------------------------------------------------------
    std::uint64_t failures = 0;
    auto require = [&failures](bool c) { if (!c) ++failures; };
    require(conditions.size() == kExpectedConditions);
    require(points.size() == kExpectedConditions * kPointsPerCondition);

    std::map<EClass, std::size_t> totals;
    std::uint64_t wow = 0, speedbrake = 0, nonFinite = 0, external = 0, throttleMismatch = 0, fdmFailed = 0;
    std::vector<const FPoint *> aborted;
    std::size_t equilibria = 0, policyCompatible = 0, absoluteValid = 0, aboveMil = 0, profileValid = 0;
    double worstLin = 0.0, worstAng = 0.0, worstEas = 0.0, worstClimb = 0.0;
    for (const FPoint &p : points) {
        ++totals[p.Classification];
        if (p.bWow) ++wow;
        if (p.bSpeedbrakeUnexpected) ++speedbrake;
        if (p.Classification == EClass::NonFinite) ++nonFinite;
        if (p.Classification == EClass::ThrottleConditionMismatch) ++throttleMismatch;
        if (p.Classification == EClass::FdmRunFailed) ++fdmFailed;
        if (p.bAboveMilitaryMachTable) ++aboveMil;
        if (!p.bProcessSurvived) aborted.push_back(&p);
        external += p.ExternalWriteAttempts;
        require(p.ExternalWriteAttempts == 0);
        require(p.bAllAddStatesOk);
        require(!(p.bProcessSurvived && p.Classification == EClass::TrimFailed &&
                  std::strstr(p.Diagnostic, "aborted the isolated process") != nullptr));
        if (Equilibrium(p.Classification)) {
            require(p.bProcessSurvived);
            ++equilibria;
            if (p.bCommandableUnderCurrentPolicy) ++policyCompatible;
            if (p.bAbsoluteSourceValid) ++absoluteValid;
            if (p.bSourceRangeValidForProfile) ++profileValid;
            require(p.EasMismatchMps <= kEasMismatchTolMps);
            require(p.MaxLinearResidual <= kLinearResidualTol);
            require(p.MaxAngularResidual <= kAngularResidualTol);
            require(p.ClimbMismatchMps <= kClimbConsistencyTolMps);
            require(!p.bWow && !p.bSpeedbrakeUnexpected);
            require(std::abs(p.SpeedbrakePos) <= 1e-9);
            require(std::abs(p.GearPos) <= 1e-9 && std::abs(p.FlapPosNorm) <= 1e-9);
            require(p.bEngineRunning && Finite(p.ThrustLb));
            require(Finite(p.TankLb[0]) && p.TankLb[0] <= kInternalTankCapacityLb + 1e-9);
            require(Finite(p.TankLb[1]) && p.TankLb[1] <= kInternalTankCapacityLb + 1e-9);
            worstLin = std::max(worstLin, p.MaxLinearResidual);
            worstAng = std::max(worstAng, p.MaxAngularResidual);
            worstEas = std::max(worstEas, p.EasMismatchMps);
            worstClimb = std::max(worstClimb, p.ClimbMismatchMps);
        }
        if (p.Profile != EProfile::LevelReference && p.bTrimSuccess) {
            require(std::abs(p.ThrottleCmdNorm - p.RequestedThrottleCmd) <= 1e-12);
            require(std::abs(p.ThrottlePosNorm - 2.0 * p.RequestedThrottleCmd) <= 1e-9);
        }
    }
    std::size_t classified = 0;
    for (EClass c : kAllClasses) classified += totals[c];
    require(classified == points.size());
    // hard failures
    require(wow == 0 && speedbrake == 0 && nonFinite == 0 && external == 0 &&
            throttleMismatch == 0 && fdmFailed == 0);

    // fuel must move the mass (tank contents, never a forced total)
    {
        std::map<std::pair<int, int>, double> massBy;
        for (const FPoint &p : points)
            if (Finite(p.MassSlug))
                massBy[{static_cast<int>(std::llround(p.AltitudeFt)),
                        static_cast<int>(std::llround(p.RequestedFuelLb))}] = p.MassSlug;
        for (int alt : {5000, 10000, 20000, 30000, 40000}) {
            const auto lo = massBy.find({alt, 1000}), mid = massBy.find({alt, 3000}), hi = massBy.find({alt, 6000});
            require(lo != massBy.end() && mid != massBy.end() && hi != massBy.end());
            if (lo != massBy.end() && mid != massBy.end() && hi != massBy.end())
                require(lo->second < mid->second && mid->second < hi->second);
        }
    }

    int baselineId = -1;
    for (const FCondition &c : conditions) if (c.IsBaseline()) baselineId = c.Id;
    require(baselineId >= 0);
    {
        std::size_t baselinePoints = 0;
        for (const FPoint &p : points) if (p.ConditionId == baselineId) ++baselinePoints;
        require(baselinePoints == kPointsPerCondition);
    }

    // ---- output ----------------------------------------------------------------------------------
    WriteMatrixCsv(argv[2], points, 15);
    WriteMatrixCsv(argv[3], points, 9);
    WriteBaselineCsv(argv[4], points, baselineId, 15);   // compared byte-for-byte against V1
    WriteBaselineCsv(argv[5], points, baselineId, 9);

    auto inCondition = [&](int id) {
        std::vector<const FPoint *> v;
        for (const FPoint &p : points) if (p.ConditionId == id) v.push_back(&p);
        return v;
    };
    auto atKey = [&](EProfile prof, double eas) {
        std::vector<const FPoint *> v;
        for (const FPoint &p : points)
            if (p.Profile == prof && std::abs(p.RequestedEasMps - eas) < 1e-9) v.push_back(&p);
        std::sort(v.begin(), v.end(), [](const FPoint *a, const FPoint *b) { return a->ConditionId < b->ConditionId; });
        return v;
    };

    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "F16_VERTICAL_PERFORMANCE_SENSITIVITY_V2\n";
    out << kInterpretation << '\n';
    out << kSeparation << '\n';
    out << kPolicyCaveat << '\n';
    out << kDensityCaveat << '\n';
    out << "model=f16 engine=F100-PW-229 data=Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim "
           "solver=FGTrim(tCustom){tUdot<-tGamma,tWdot<-tAlpha,tQdot<-tElevator,tVdot<-tPhi,"
           "tPdot<-tAileron,tRdot<-tRudder,tHmgt<-tBeta} level_reference_solver=FGTrim(tFull) "
           "trim_tolerance=" << kTrimTolerance << " fdm_dt_s=" << kFdmDtS << '\n';
    out << "altitude_grid_ft=5000,10000,20000,30000,40000 fuel_grid_lb=1000,3000,6000 "
           "fuel_distribution=internal_tanks_0_and_1_split_evenly external_tanks=empty "
           "wind_mps=0 gear=up flap=clean speedbrake=retracted fuel=frozen heading_deg="
        << kHeadingDeg << '\n';
    out << "throttle_profiles idle_cmd=0.0 military_dry_cmd=0.5 augmented_cmd=1.0 "
           "mapping=f16xml_fcs_gain_2_and_F100_augmethod_2 position=2x_command "
           "production_ThrottleMax=1.0_admits_full_afterburner_recorded_not_decided\n";
    out << "eas_samples_mps idle_and_level=70,90,110,130,150,170,189.070713,220,235,250,280 "
           "powered=110,189.070713,235 anchor=" << kAnchorEasMps
        << " note=characterization_samples_only_not_EasMin_EasTrim_EasMax_stall_VNE_max_climb_or_min_sink_speed\n";
    out << "tolerances linear_residual_ftps2=" << kLinearResidualTol
        << " angular_residual_radps2=" << kAngularResidualTol
        << " eas_mismatch_mps=" << kEasMismatchTolMps
        << " climb_consistency_mps=" << kClimbConsistencyTolMps << '\n';
    out << "current_policy pitch_min_rad=" << kPolicyPitchMinRad << " pitch_max_rad=" << kPolicyPitchMaxRad
        << " throttle_min=" << kPolicyThrottleMin << " throttle_max=" << kPolicyThrottleMax
        << " surface_cmd_limit=" << kPolicySurfaceCmdLimit
        << " source=FGuidanceConfigV2_FF16StickConfigV2_defaults\n";
    out << "total_conditions=" << conditions.size() << " total_points=" << points.size()
        << " points_per_condition=" << kPointsPerCondition << '\n';
    out << "classifications";
    for (EClass c : kAllClasses) out << ' ' << ClassName(c) << '=' << totals[c];
    out << '\n';
    out << "classified=" << classified << " unclassified=" << (points.size() - classified)
        << " solver_equilibria=" << equilibria
        << " policy_bound_compatible=" << policyCompatible
        << " absolute_source_valid=" << absoluteValid
        << " source_range_valid_for_profile=" << profileValid
        << " above_military_mach_table=" << aboveMil
        << " failures=" << failures << '\n';
    out << "baseline_reproduction condition_id=" << baselineId
        << " altitude_ft=" << kBaselineAltitudeFt << " fuel_lb=" << kBaselineFuelLb
        << " points=" << kPointsPerCondition
        << " compared_against=committed_V1_harness_bb9af6a schema=identical\n";
    out << "worst_over_equilibria linear_residual_ftps2=" << worstLin
        << " angular_residual_radps2=" << worstAng
        << " eas_mismatch_mps=" << worstEas
        << " climb_consistency_mps=" << worstClimb << '\n';
    out << "quality unexpected_wow=" << wow << " unexpected_speedbrake_deployment=" << speedbrake
        << " non_finite=" << nonFinite << " fdm_run_failed=" << fdmFailed
        << " throttle_condition_mismatch=" << throttleMismatch
        << " external_write_attempts=" << external << '\n';
    out << "point_isolation=one_process_per_point processes_spawned=" << points.size()
        << " solver_process_aborted=" << aborted.size()
        << " note=the_pinned_JSBSim_static_library_asserts_and_calls_abort_inside_FGTrim_checkLimits_at_"
           "some_extreme_fixed_throttle_points_such_a_point_is_reported_as_TrimFailed_with_"
           "process_survived_0_never_as_an_equilibrium\n";
    for (const FPoint *p : aborted)
        out << "  solver_process_abort condition_id=" << p->ConditionId
            << " altitude_ft=" << p->AltitudeFt << " fuel_lb=" << p->RequestedFuelLb
            << " point_id=" << p->IdInCondition << " profile=" << ProfileName(p->Profile)
            << " requested_eas_mps=" << p->RequestedEasMps
            << " classification=" << ClassName(p->Classification)
            << " process_survived=0 diagnostic=" << p->Diagnostic << '\n';
    out << "production_writer_invocations=0 ue_world_loaded=0 game_pawns_searched=0 active_connection=0 "
           "tecs_parameters_modified=0 aircraft_xml_modified=0 surface_position_direct_writes=0 "
           "aerodynamic_property_direct_writes=0\n";

    // ---- per-condition summaries -----------------------------------------------------------------
    for (const FCondition &cond : conditions) {
        const std::vector<const FPoint *> cp = inCondition(cond.Id);
        std::map<EClass, std::size_t> counts;
        for (const FPoint *p : cp) ++counts[p->Classification];
        std::size_t cClassified = 0;
        for (EClass c : kAllClasses) cClassified += counts[c];
        std::size_t cEq = 0, cPolicy = 0, cAbs = 0, cProfileValid = 0, cAboveMil = 0;
        std::size_t idleSink = 0, milClimb = 0, augClimb = 0;
        double cLin = 0.0, cAng = 0.0, cEas = 0.0, cKin = 0.0;
        const FPoint *anchor = nullptr;
        for (const FPoint *p : cp) {
            if (p->Profile == EProfile::LevelReference && std::abs(p->RequestedEasMps - kAnchorEasMps) < 1e-9)
                anchor = p;
            if (p->bAboveMilitaryMachTable) ++cAboveMil;
            if (!Equilibrium(p->Classification)) continue;
            ++cEq;
            if (p->bCommandableUnderCurrentPolicy) ++cPolicy;
            if (p->bAbsoluteSourceValid) ++cAbs;
            if (p->bSourceRangeValidForProfile) ++cProfileValid;
            if (p->Profile == EProfile::Idle && p->Classification == EClass::SteadySinkFeasible) ++idleSink;
            if (p->Profile == EProfile::Military && p->Classification == EClass::SteadyClimbFeasible) ++milClimb;
            if (p->Profile == EProfile::Augmented && p->Classification == EClass::SteadyClimbFeasible) ++augClimb;
            cLin = std::max(cLin, p->MaxLinearResidual);
            cAng = std::max(cAng, p->MaxAngularResidual);
            cEas = std::max(cEas, p->EasMismatchMps);
            cKin = std::max(cKin, p->ClimbMismatchMps);
        }
        out << "condition id=" << cond.Id
            << " altitude_ft=" << cond.AltitudeFt << " altitude_m=" << cond.AltitudeFt * kFtToM
            << " fuel_lb=" << cond.RequestedFuelLb
            << " tanks_lb=" << cond.Tank0Lb << '|' << cond.Tank1Lb << '|' << cond.Tank2Lb << '|' << cond.Tank3Lb
            << " baseline=" << (cond.IsBaseline() ? 1 : 0) << '\n';
        out << "  mass_slug=" << Num(anchor ? anchor->MassSlug : kNa)
            << " mass_kg=" << Num(anchor ? anchor->MassKg : kNa)
            << " weight_lb=" << Num(anchor ? anchor->WeightLb : kNa)
            << " cg_in=" << Num(anchor ? anchor->CgXIn : kNa) << ',' << Num(anchor ? anchor->CgYIn : kNa)
            << ',' << Num(anchor ? anchor->CgZIn : kNa)
            << " density_slug_ft3=" << Num(anchor ? anchor->DensitySlugFt3 : kNa)
            << " density_ratio=" << Num(anchor ? anchor->DensityRatio : kNa)
            << " density_altitude_ft=" << Num(anchor ? anchor->DensityAltitudeFt : kNa) << '\n';
        out << "  total_points=" << cp.size() << " classified=" << cClassified
            << " unclassified=" << (cp.size() - cClassified)
            << " solver_equilibria=" << cEq << " policy_bound_compatible=" << cPolicy
            << " absolute_source_valid=" << cAbs << " source_range_valid_for_profile=" << cProfileValid
            << " above_military_mach_table=" << cAboveMil
            << " idle_sink_equilibria=" << idleSink << " military_climb_equilibria=" << milClimb
            << " augmented_climb_equilibria=" << augClimb
            << " trim_failed=" << counts[EClass::TrimFailed] << '\n';
        out << "  classifications";
        for (EClass c : kAllClasses) out << ' ' << ClassName(c) << '=' << counts[c];
        out << '\n';
        out << "  maxima linear_residual=" << cLin << " angular_residual=" << cAng
            << " eas_mismatch=" << cEas << " climb_consistency=" << cKin << '\n';

        // idle sink scan within this condition
        {
            const FPoint *lo = nullptr, *hi = nullptr; int signChanges = 0;
            const FPoint *prev = nullptr; int prevSign = 0;
            std::vector<const FPoint *> idle;
            for (const FPoint *p : cp)
                if (p->Profile == EProfile::Idle && p->Classification == EClass::SteadySinkFeasible) idle.push_back(p);
            std::sort(idle.begin(), idle.end(), [](const FPoint *a, const FPoint *b) {
                return a->RequestedEasMps < b->RequestedEasMps; });
            for (const FPoint *p : idle) {
                const double mag = std::abs(p->ClimbRateMps);
                if (!lo || mag < std::abs(lo->ClimbRateMps)) lo = p;
                if (!hi || mag > std::abs(hi->ClimbRateMps)) hi = p;
                if (prev) {
                    const double d = mag - std::abs(prev->ClimbRateMps);
                    const int s = d > 0 ? 1 : (d < 0 ? -1 : 0);
                    if (prevSign != 0 && s != 0 && s != prevSign) ++signChanges;
                    if (s != 0) prevSign = s;
                }
                prev = p;
            }
            out << "  idle_sink_scan lowest_magnitude_sample_mps=" << Num(lo ? std::abs(lo->ClimbRateMps) : kNa)
                << " at_eas_mps=" << Num(lo ? lo->RequestedEasMps : kNa)
                << " highest_magnitude_sample_mps=" << Num(hi ? std::abs(hi->ClimbRateMps) : kNa)
                << " at_eas_mps=" << Num(hi ? hi->RequestedEasMps : kNa)
                << " magnitude_slope_sign_changes=" << signChanges
                << " local_u_shape_within_scan=" << (signChanges >= 1 ? 1 : 0)
                << " note=within_tested_samples_only\n";
        }

        // military / augmented climb within this condition
        for (EProfile prof : {EProfile::Military, EProfile::Augmented}) {
            const FPoint *best = nullptr, *bestPolicy = nullptr;
            for (const FPoint *p : cp) {
                if (p->Profile != prof || p->Classification != EClass::SteadyClimbFeasible) continue;
                if (!best || p->ClimbRateMps > best->ClimbRateMps) best = p;
                if (p->bCommandableUnderCurrentPolicy &&
                    (!bestPolicy || p->ClimbRateMps > bestPolicy->ClimbRateMps)) bestPolicy = p;
            }
            out << "  " << (prof == EProfile::Military ? "military_climb" : "augmented_climb")
                << " largest_solver_feasible_sample_mps=" << Num(best ? best->ClimbRateMps : kNa)
                << " at_eas_mps=" << Num(best ? best->RequestedEasMps : kNa)
                << " gamma_rad=" << Num(best ? best->GammaRad : kNa)
                << " pitch_rad=" << Num(best ? best->PitchRad : kNa)
                << " thrust_lb=" << Num(best ? best->ThrustLb : kNa)
                << " source_range_valid=" << Flag(best != nullptr, best && best->bSourceRangeValidForProfile)
                << " largest_policy_bound_compatible_sample_mps=" << Num(bestPolicy ? bestPolicy->ClimbRateMps : kNa)
                << " at_eas_mps=" << Num(bestPolicy ? bestPolicy->RequestedEasMps : kNa)
                << " note=within_tested_samples_only\n";
        }
    }

    // ---- cross-condition sensitivity (diagnostic; non-monotonicity is NOT an error) ---------------
    std::vector<FKey> keys;
    for (double e : kIdleSweep) keys.push_back({EProfile::LevelReference, e});
    for (double e : kIdleSweep) keys.push_back({EProfile::Idle, e});
    for (double e : kPoweredEas) keys.push_back({EProfile::Military, e});
    for (double e : kPoweredEas) keys.push_back({EProfile::Augmented, e});

    for (const FKey &k : keys) {
        const std::vector<const FPoint *> v = atKey(k.Profile, k.Eas);
        std::size_t ok = 0, srcOk = 0, polOk = 0;
        double climbMin = 1e18, climbMax = -1e18, gMin = 1e18, gMax = -1e18, pMin = 1e18, pMax = -1e18;
        for (const FPoint *p : v) {
            if (!Equilibrium(p->Classification)) continue;
            ++ok;
            if (p->bSourceRangeValidForProfile) ++srcOk;
            if (p->bCommandableUnderCurrentPolicy) ++polOk;
            climbMin = std::min(climbMin, p->ClimbRateMps); climbMax = std::max(climbMax, p->ClimbRateMps);
            gMin = std::min(gMin, p->GammaRad); gMax = std::max(gMax, p->GammaRad);
            pMin = std::min(pMin, p->PitchRad); pMax = std::max(pMax, p->PitchRad);
        }
        out << "sensitivity profile=" << ProfileName(k.Profile) << " eas_mps=" << k.Eas
            << " available_conditions=" << v.size() << " solver_success=" << ok
            << " source_range_valid=" << srcOk << " policy_bound_compatible=" << polOk
            << " climb_mps=[" << Num(ok ? climbMin : kNa) << ',' << Num(ok ? climbMax : kNa) << ']'
            << " gamma_rad=[" << Num(ok ? gMin : kNa) << ',' << Num(ok ? gMax : kNa) << ']'
            << " pitch_rad=[" << Num(ok ? pMin : kNa) << ',' << Num(ok ? pMax : kNa) << ']';

        // altitude trend at the baseline fuel; fuel trend at the baseline altitude
        auto trend = [&](bool byAltitude) {
            std::vector<const FPoint *> s;
            for (const FPoint *p : v) {
                const bool sel = byAltitude ? std::abs(p->RequestedFuelLb - kBaselineFuelLb) < 1e-9
                                            : std::abs(p->AltitudeFt - kBaselineAltitudeFt) < 1e-9;
                if (sel && Equilibrium(p->Classification)) s.push_back(p);
            }
            std::sort(s.begin(), s.end(), [byAltitude](const FPoint *a, const FPoint *b) {
                return byAltitude ? a->AltitudeFt < b->AltitudeFt : a->RequestedFuelLb < b->RequestedFuelLb; });
            std::ostringstream t;
            t << std::fixed << std::setprecision(6);
            int changes = 0, prevSign = 0;
            for (std::size_t i = 0; i < s.size(); ++i) {
                if (i) t << '|';
                t << (byAltitude ? s[i]->AltitudeFt : s[i]->RequestedFuelLb) << ':' << s[i]->ClimbRateMps;
                if (i) {
                    const double d = s[i]->ClimbRateMps - s[i - 1]->ClimbRateMps;
                    const int sg = d > 0 ? 1 : (d < 0 ? -1 : 0);
                    if (prevSign != 0 && sg != 0 && sg != prevSign) ++changes;
                    if (sg != 0) prevSign = sg;
                }
            }
            return std::make_pair(t.str(), changes);
        };
        const auto alt = trend(true);
        const auto fuel = trend(false);
        out << " climb_vs_altitude_at_3000lb=" << (alt.first.empty() ? "none" : alt.first)
            << " altitude_direction_changes=" << alt.second
            << " climb_vs_fuel_at_10000ft=" << (fuel.first.empty() ? "none" : fuel.first)
            << " fuel_direction_changes=" << fuel.second
            << " note=monotonicity_is_diagnostic_only\n";
    }

    // ---- common comparison sets ------------------------------------------------------------------
    {
        std::vector<std::string> allSolver, allAbsolute, allMilTable, allPolicy;
        std::vector<std::string> excluded;
        for (const FKey &k : keys) {
            const std::vector<const FPoint *> v = atKey(k.Profile, k.Eas);
            bool solverAll = !v.empty(), absAll = !v.empty(), milAll = (k.Profile == EProfile::Military), polAll = !v.empty();
            std::string why;
            for (const FPoint *p : v) {
                if (!Equilibrium(p->Classification)) {
                    solverAll = false;
                    why += std::string(" cond") + std::to_string(p->ConditionId) + "=" + ClassName(p->Classification);
                }
                if (!p->bAbsoluteSourceValid) absAll = false;
                if (k.Profile == EProfile::Military && !p->bMilitarySourceRangeValid) milAll = false;
                if (!p->bCommandableUnderCurrentPolicy) {
                    polAll = false;
                    if (p->bPitchPolicyExceeded)
                        why += std::string(" cond") + std::to_string(p->ConditionId) + "=pitch_policy";
                }
            }
            std::ostringstream key;
            key << ProfileName(k.Profile) << '@' << std::fixed << std::setprecision(6) << k.Eas;
            if (solverAll) allSolver.push_back(key.str());
            if (absAll) allAbsolute.push_back(key.str());
            if (k.Profile == EProfile::Military && milAll) allMilTable.push_back(key.str());
            if (polAll) allPolicy.push_back(key.str());
            if (!why.empty()) excluded.push_back(key.str() + ":" + why);
        }
        auto join = [](const std::vector<std::string> &v) {
            if (v.empty()) return std::string("none");
            std::string s;
            for (std::size_t i = 0; i < v.size(); ++i) { if (i) s += ','; s += v[i]; }
            return s;
        };
        out << "common_set solver_success_in_all_15=" << allSolver.size() << " [" << join(allSolver) << "]\n";
        out << "common_set absolute_source_valid_in_all_15=" << allAbsolute.size() << " [" << join(allAbsolute) << "]\n";
        out << "common_set military_table_valid_in_all_15=" << allMilTable.size() << " [" << join(allMilTable) << "]\n";
        out << "common_set policy_bound_compatible_in_all_15=" << allPolicy.size() << " [" << join(allPolicy) << "]\n";
        out << "common_set_exclusions" ;
        if (excluded.empty()) out << " none";
        for (const std::string &e : excluded) out << ' ' << e;
        out << '\n';
    }

    const std::string summary = out.str();
    std::ofstream sf(argv[6]); sf << summary;
    std::fputs(summary.c_str(), stdout);
    std::printf("F16_VERTICAL_PERFORMANCE_SENSITIVITY_V2_RESULT=%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
