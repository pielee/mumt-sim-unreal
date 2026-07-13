// verify_f16_vertical_performance_v2.cpp
//
// F-16 sustained vertical performance, V1 baseline characterization.
//
// This test characterizes fixed-throttle, constant-airspeed solver equilibria for the JSBSim F-16
// at one altitude and mass condition. It does not define operational climb or sink limits and is
// not used directly as a TECS parameter set.
//
// Solver-feasible equilibrium and compatibility with the current TECS/Stick pitch and throttle
// policy are reported separately.
//
// METHOD -- fixed-throttle equilibrium search (audit verdict B).
//   The airspeed is pinned by the initial condition, the throttle command is pinned by the caller,
//   and the FLIGHT PATH ANGLE is the solver variable:
//
//     FGTrim trim(exec, tCustom);
//     trim.AddState(tUdot, tGamma);      <-- longitudinal accel zeroed by gamma, NOT by throttle
//     trim.AddState(tWdot, tAlpha);
//     trim.AddState(tQdot, tElevator);
//     trim.AddState(tVdot, tPhi);
//     trim.AddState(tPdot, tAileron);
//     trim.AddState(tRdot, tRudder);
//     trim.AddState(tHmgt, tBeta);
//
//   This is the full six-axis equilibrium of tFull with one substitution: udot is zeroed with gamma
//   instead of thrust. tFull cannot be used for this: with throttle as the solver variable it
//   reports FAILURE at exactly the endpoints we must characterize (throttle pinned at 0 or 1), which
//   was measured during the audit.
//
//   The PX4 contract this matches (pinned v1.17.0, TECS.cpp::_calcThrottleControlOutput comment):
//     "Specific total energy rate = _STE_rate_max is achieved when throttle is set to throttle_max
//      Specific total energy rate = 0 at cruise throttle
//      Specific total energy rate = _STE_rate_min is achieved when throttle is set to throttle_min"
//   and _calculateTotalEnergyRateLimit: STE_rate_max = max_climb_rate * g. Since STE_rate = g*hdot +
//   V*Vdot and the trim has Vdot == 0 by construction, the trimmed climb rate IS the quantity TECS
//   means. NOTE: the pinned PX4 reference does NOT contain src/lib/fw_performance_model, so the
//   density correction PX4 applies to those parameters cannot be audited and is NOT applied here.
//
// THROTTLE MAPPING -- read out of the shipped model, not assumed:
//   f16.xml FCS:  fcs/throttle-pos-norm = 2 * fcs/throttle-cmd-norm   (pure_gain, gain 2)
//   F100-PW-229.xml: <augmented>1</augmented>, <augmethod>2</augmethod>
//   FGTurbine.h: "2 = throttle range is expanded in the FCS, and values above 1.0 are afterburner"
//   =>  cmd 0.0 = idle / windmilling
//       cmd 0.5 = maximum dry / military   (position 1.0)
//       cmd 1.0 = maximum augmented / full afterburner (position 2.0)
//   The current production FGuidanceConfigV2::ThrottleMax is 1.0, i.e. it admits full afterburner.
//   That is recorded, not decided, here.
//
// Every point owns a fresh FGFDMExec. No FDM/FCS/engine/fuel/command/timestamp state is shared. No
// UE World, Pawn, or production writer is touched, and no surface position or aerodynamic
// coefficient is ever written.
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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kFdmDtS = 1.0 / 120.0;
constexpr double kFtToM = 0.3048;
constexpr double kKnotToMps = 0.5144444444444444;
constexpr double kSlugToKg = 14.593902937206363;

// ---- fixed V1 condition ----------------------------------------------------------------------
constexpr double kAltitudeFt = 10000.0;
constexpr double kLatitudeDeg = 47.0;
constexpr double kLongitudeDeg = -122.0;
constexpr double kHeadingDeg = 90.0;
constexpr double kTank0Lb = 1500.0, kTank1Lb = 1500.0, kTank2Lb = 0.0, kTank3Lb = 0.0;   // 3000 lb total
constexpr double kInternalTankCapacityLb = 3486.0;

// ---- tolerances, fixed from measurement (see the header of the audit; NOT fitted to results) ----
// SetTolerance(t) is called explicitly: JSBSim holds the recti-linear accelerations to t and the
// angular accelerations to t/10. Measured worst residuals over the whole V1 point set were
// 8.60e-04 ft/s^2 linear and 8.21e-05 rad/s^2 angular, i.e. inside this contract.
constexpr double kTrimTolerance = 1.0e-3;
constexpr double kLinearResidualTol = kTrimTolerance;          // ft/s^2
constexpr double kAngularResidualTol = kTrimTolerance / 10.0;  // rad/s^2
// Climb-rate kinematic reconstruction (TAS * sin(gamma)). Worst measured mismatch was 2.56e-12 m/s.
constexpr double kClimbConsistencyTolMps = 1.0e-9;
// Same airspeed contract as the committed level trim map.
constexpr double kEasMismatchTolMps = 1.0e-6;
// Sign threshold separating a level equilibrium from a climb/sink one.
constexpr double kLevelClimbThresholdMps = 1.0e-3;
constexpr double kFiniteLimit = 1.0e12;

// ---- model source-table bounds, read out of the shipped files -----------------------------------
constexpr double kAeroAlphaMinRad = -0.5236;         // f16.xml aerodynamics
constexpr double kAeroAlphaMaxRad = 0.7850;          // f16.xml aerodynamics
constexpr double kEngineMachMaxAug = 2.6;            // F100-PW-229.xml AugThrust (widest)
constexpr double kEngineMachMaxMil = 1.4;            // F100-PW-229.xml MilThrust
constexpr double kEngineDensityAltMinFt = -10000.0;  // F100-PW-229.xml thrust tables
constexpr double kEngineDensityAltMaxFt = 60000.0;   // F100-PW-229.xml thrust tables
// f16.xml speedbrake channel deploys automatically only in a deep stall.
constexpr double kSpeedbrakeAutoAlphaDeg = 53.0;
constexpr double kSpeedbrakeAutoVFps = 18.0;

// ---- current production policy, READ-ONLY (FGuidanceConfigV2 / FF16StickConfigV2 defaults) ------
// Duplicated as constants because this host test does not link the production module. They are
// reported as "current policy", never changed, and never used as a performance value.
constexpr double kPolicyPitchMinRad = -0.5;   // FGuidanceConfigV2::PitchMinRad
constexpr double kPolicyPitchMaxRad = 0.5;    // FGuidanceConfigV2::PitchMaxRad
constexpr double kPolicyThrottleMin = 0.0;    // FGuidanceConfigV2::ThrottleMin
constexpr double kPolicyThrottleMax = 1.0;    // FGuidanceConfigV2::ThrottleMax  (admits full AB)
constexpr double kPolicySurfaceCmdLimit = 1.0;  // FF16StickConfigV2 elevator/aileron/rudder [-1,1]

const char *kInterpretation =
    "This test characterizes fixed-throttle, constant-airspeed solver equilibria for the JSBSim "
    "F-16 at one altitude and mass condition. It does not define operational climb or sink limits "
    "and is not used directly as a TECS parameter set.";
const char *kSeparation =
    "Solver-feasible equilibrium and compatibility with the current TECS/Stick pitch and throttle "
    "policy are reported separately.";

bool Finite(double v) { return std::isfinite(v) && std::abs(v) < kFiniteLimit; }

// ---- throttle profiles -------------------------------------------------------------------------
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

// ---- primary classification (exactly one per point) --------------------------------------------
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

constexpr double kNa = std::numeric_limits<double>::quiet_NaN();

struct FPoint {
    int Id{};
    EProfile Profile{EProfile::LevelReference};
    double RequestedThrottleCmd{};
    double RequestedEasMps{};
    EClass Classification{EClass::InvalidInput};
    std::string Diagnostic;

    bool bAllAddStatesOk{}, bTrimAttempted{}, bTrimSuccess{}, bRunIcOk{};

    // state
    double ActualEasMps{kNa}, TasMps{kNa}, Mach{kNa}, AltitudeM{kNa}, DensityAltitudeFt{kNa};
    double DensitySlugFt3{kNa}, SoundSpeedMps{kNa};
    double GammaRad{kNa}, ClimbRateMps{kNa}, ReconstructedClimbMps{kNa}, ClimbMismatchMps{kNa};
    double PitchRad{kNa}, RollRad{kNa}, AlphaRad{kNa}, BetaRad{kNa};
    double P{kNa}, Q{kNa}, R{kNa}, LoadFactor{kNa};
    double ElevatorCmd{kNa}, StabilatorPosRad{kNa};
    double AileronCmd{kNa}, AileronPosRad{kNa};
    double RudderCmd{kNa}, RudderPosRad{kNa};
    double EasMismatchMps{kNa};

    // mass / condition
    double MassSlug{kNa}, MassKg{kNa}, WeightLb{kNa}, FuelLb{kNa};
    std::array<double, 4> TankLb{{kNa, kNa, kNa, kNa}};
    double CgXIn{kNa}, CgYIn{kNa}, CgZIn{kNa};
    double GearPos{kNa}, FlapPosNorm{kNa}, SpeedbrakePos{kNa}, WindMps{0.0};

    // engine
    double ThrottleCmdNorm{kNa}, ThrottlePosNorm{kNa}, ThrustLb{kNa};
    bool bEngineRunning{};
    int EnginePhase{-1};
    bool bTurbineAvailable{};
    bool bAugmentation{};
    double N1{kNa}, N2{kNa};
    double LevelTrimThrottleAtSameEas{kNa};
    bool bLevelTrimThrottleAvailable{};

    // residuals
    double Udot{kNa}, Vdot{kNa}, Wdot{kNa}, Pdot{kNa}, Qdot{kNa}, Rdot{kNa};
    double MaxLinearResidual{kNa}, MaxAngularResidual{kNa};

    // policy flags (independent of the primary classification)
    bool bPitchPolicyExceeded{}, bThrottlePolicyExceeded{};
    bool bElevatorLimitExceeded{}, bAileronLimitExceeded{}, bRudderLimitExceeded{};
    bool bCommandableUnderCurrentPolicy{};

    // source-table flags
    bool bAlphaTableExceeded{}, bEngineMachTableExceeded{}, bDensityAltTableExceeded{};
    bool bAboveMilitaryMachTable{};

    // quality
    bool bWow{}, bSpeedbrakeUnexpected{};
    std::uint64_t ExternalWriteAttempts{};
    bool bProcessSurvived{true};
};

class FVerticalPlant {
public:
    bool Load(const std::string &root, FPoint &p)
    {
        Exec = std::make_unique<JSBSim::FGFDMExec>();
        Owned = Exec.get();
        Exec->SetDebugLevel(0);
        Exec->SetRootDir(SGPath(root));
        Exec->SetAircraftPath(SGPath("aircraft"));
        Exec->SetEnginePath(SGPath("engine"));
        Exec->SetSystemsPath(SGPath("systems"));
        if (!Exec->LoadModel("f16")) { p.Diagnostic = "LoadModel(f16) failed"; return false; }
        Exec->Setdt(kFdmDtS);
        IC = Exec->GetIC(); Fcs = Exec->GetFCS(); Prop = Exec->GetPropagate();
        Aux = Exec->GetAuxiliary(); Atm = Exec->GetAtmosphere(); Pls = Exec->GetPropulsion();
        Grd = Exec->GetGroundReactions(); Mass = Exec->GetMassBalance(); Acc = Exec->GetAccelerations();
        if (!IC || !Fcs || !Prop || !Aux || !Atm || !Pls || !Grd || !Mass || !Acc) {
            p.Diagnostic = "required JSBSim model pointer is null"; return false;
        }
        if (Pls->GetNumTanks() != 4) { p.Diagnostic = "f16 model does not declare the expected 4 fuel tanks"; return false; }
        const std::array<double, 4> want{kTank0Lb, kTank1Lb, kTank2Lb, kTank3Lb};
        for (unsigned i = 0; i < 4; ++i) {
            auto tank = Pls->GetTank(i);
            if (!tank) { p.Diagnostic = "null fuel tank"; return false; }
            if (want[i] < 0.0 || want[i] > tank->GetCapacity() + 1e-9) {
                p.Diagnostic = "requested tank fuel exceeds the declared model capacity"; return false;
            }
            tank->SetContents(want[i]);
        }
        return true;
    }

    bool RunIc(double easMps, FPoint &p)
    {
        IC->SetGeodLatitudeDegIC(kLatitudeDeg);
        IC->SetLongitudeDegIC(kLongitudeDeg);
        IC->SetAltitudeASLFtIC(kAltitudeFt);
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
        if (!p.bRunIcOk) { p.Diagnostic = "RunIC failed"; return false; }
        Pls->InitRunning(-1);
        Pls->SetFuelFreeze(true);
        Fcs->SetGearCmd(0.0); Fcs->SetGearPos(0.0);
        Fcs->SetDfCmd(0.0); Fcs->SetDfPos(JSBSim::ofNorm, 0.0);
        Fcs->SetDsbCmd(0.0);                       // speedbrake retracted, clean configuration
        return true;
    }

    // Level reference: the official tFull trim at gamma = 0. Its throttle is a horizontal-equilibrium
    // reference only -- it is NEVER reused as a fixed-throttle vertical profile and is NEVER
    // recommended as a production throttle_trim.
    bool LevelTrim(FPoint &p)
    {
        Fcs->SetThrottleCmd(0, 0.6);
        p.bAllAddStatesOk = true;                  // tFull configures its own axes
        p.bTrimAttempted = true;
        JSBSim::FGTrim trim(Exec.get(), JSBSim::tFull);
        trim.SetTolerance(kTrimTolerance);
        p.bTrimSuccess = trim.DoTrim();
        return p.bTrimSuccess;
    }

    // Fixed-throttle equilibrium: throttle pinned by the caller, gamma solved.
    bool FixedThrottleTrim(double throttleCmd, FPoint &p)
    {
        Fcs->SetThrottleCmd(0, throttleCmd);
        p.bTrimAttempted = true;
        JSBSim::FGTrim trim(Exec.get(), JSBSim::tCustom);
        trim.SetTolerance(kTrimTolerance);
        trim.ClearStates();
        const bool a1 = trim.AddState(JSBSim::tUdot, JSBSim::tGamma);
        const bool a2 = trim.AddState(JSBSim::tWdot, JSBSim::tAlpha);
        const bool a3 = trim.AddState(JSBSim::tQdot, JSBSim::tElevator);
        const bool a4 = trim.AddState(JSBSim::tVdot, JSBSim::tPhi);
        const bool a5 = trim.AddState(JSBSim::tPdot, JSBSim::tAileron);
        const bool a6 = trim.AddState(JSBSim::tRdot, JSBSim::tRudder);
        const bool a7 = trim.AddState(JSBSim::tHmgt, JSBSim::tBeta);
        p.bAllAddStatesOk = a1 && a2 && a3 && a4 && a5 && a6 && a7;
        if (!p.bAllAddStatesOk) { p.Diagnostic = "FGTrim::AddState failed"; return false; }
        p.bTrimSuccess = trim.DoTrim();
        return p.bTrimSuccess;
    }

    void Read(FPoint &p) const
    {
        p.ActualEasMps = Aux->GetVequivalentKTS() * kKnotToMps;
        p.TasMps = Aux->GetVt() * kFtToM;
        p.Mach = Aux->GetMach();
        p.AltitudeM = Prop->GetAltitudeASL() * kFtToM;
        p.DensityAltitudeFt = Exec->GetPropertyValue("atmosphere/density-altitude");
        p.DensitySlugFt3 = Atm->GetDensity();
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
        // The f16.xml speedbrake channel auto-deploys ONLY in a deep stall (alpha >= 53 deg and
        // v <= 18 fps). Anything else with a commanded-retracted speedbrake is a violation.
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

void ApplyPolicyAndRangeFlags(FPoint &p)
{
    // Layer 2: compatibility with the CURRENT production policy. Read-only; nothing is changed.
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

    // Absolute model source-table bounds.
    p.bAlphaTableExceeded = Finite(p.AlphaRad) &&
                            (p.AlphaRad < kAeroAlphaMinRad || p.AlphaRad > kAeroAlphaMaxRad);
    p.bEngineMachTableExceeded = Finite(p.Mach) && (p.Mach < 0.0 || p.Mach > kEngineMachMaxAug);
    p.bDensityAltTableExceeded = Finite(p.DensityAltitudeFt) &&
                                 (p.DensityAltitudeFt < kEngineDensityAltMinFt ||
                                  p.DensityAltitudeFt > kEngineDensityAltMaxFt);
    // Above the MILITARY thrust table's Mach extent but still inside the augmented table: this is a
    // source-range caveat for the dry-power profile, not an absolute model-range violation.
    p.bAboveMilitaryMachTable = Finite(p.Mach) && p.Mach > kEngineMachMaxMil;
}

FPoint RunPoint(const std::string &root, int id, EProfile profile, double throttleCmd, double easMps,
                double levelTrimThrottle, bool levelTrimThrottleAvailable)
{
    FPoint p{};
    p.Id = id;
    p.Profile = profile;
    p.RequestedThrottleCmd = throttleCmd;
    p.RequestedEasMps = easMps;
    p.LevelTrimThrottleAtSameEas = levelTrimThrottle;
    p.bLevelTrimThrottleAvailable = levelTrimThrottleAvailable;

    if (!Finite(easMps) || easMps <= 0.0 || !Finite(throttleCmd) || throttleCmd < 0.0 || throttleCmd > 1.0) {
        p.Classification = EClass::InvalidInput;
        p.Diagnostic = "requested EAS or throttle command is invalid";
        return p;
    }

    try {
        FVerticalPlant plant;
        if (!plant.Load(root, p)) { p.Classification = EClass::FdmRunFailed; return p; }
        if (!plant.RunIc(easMps, p)) { p.Classification = EClass::FdmRunFailed; return p; }
        if (!plant.OwnedIdentityOk()) {
            ++p.ExternalWriteAttempts;
            p.Classification = EClass::FdmRunFailed;
            p.Diagnostic = "write target was not the owned FGFDMExec";
            return p;
        }

        const bool trimmed = (profile == EProfile::LevelReference) ? plant.LevelTrim(p)
                                                                   : plant.FixedThrottleTrim(throttleCmd, p);
        if (!p.bAllAddStatesOk) { p.Classification = EClass::InvalidInput; return p; }
        plant.Read(p);
        ApplyPolicyAndRangeFlags(p);
        if (!trimmed) {
            p.Classification = EClass::TrimFailed;
            if (p.Diagnostic.empty()) p.Diagnostic = "FGTrim::DoTrim() returned false";
            return p;
        }
        if (!plant.StateFinite(p)) {
            p.Classification = EClass::NonFinite;
            p.Diagnostic = "non-finite state after trim";
            return p;
        }
        if (p.bWow) {
            p.Classification = EClass::UnexpectedWOW;
            p.Diagnostic = "WOW after airborne trim";
            return p;
        }
        // The fixed-throttle contract: the solver must NOT have moved the throttle, and the FCS gain
        // of 2 must hold (f16.xml: throttle-pos-norm = 2 * throttle-cmd-norm).
        if (profile != EProfile::LevelReference) {
            const bool cmdHeld = std::abs(p.ThrottleCmdNorm - throttleCmd) <= 1e-12;
            const bool posOk = std::abs(p.ThrottlePosNorm - 2.0 * throttleCmd) <= 1e-9;
            if (!cmdHeld || !posOk) {
                p.Classification = EClass::ThrottleConditionMismatch;
                p.Diagnostic = cmdHeld ? "throttle position is not 2x the command"
                                       : "solver moved the throttle command";
                return p;
            }
        }
        if (!Finite(p.EasMismatchMps) || p.EasMismatchMps > kEasMismatchTolMps) {
            p.Classification = EClass::AirspeedNotHeld;
            p.Diagnostic = "post-trim EAS does not match the requested EAS";
            return p;
        }
        if (!Finite(p.MaxLinearResidual) || p.MaxLinearResidual > kLinearResidualTol ||
            !Finite(p.MaxAngularResidual) || p.MaxAngularResidual > kAngularResidualTol) {
            p.Classification = EClass::SteadyStateNotReached;
            p.Diagnostic = "residual acceleration exceeds the declared trim tolerance";
            return p;
        }
        if (p.bAlphaTableExceeded || p.bEngineMachTableExceeded || p.bDensityAltTableExceeded) {
            p.Classification = EClass::ModelRangeExceeded;
            p.Diagnostic = "trim state is outside an absolute model source-table bound";
            return p;
        }
        // Control saturation is boundary characterization data, not a defect -- but it IS a distinct
        // primary class, because a saturated surface means the equilibrium sits on a control limit.
        if (p.bElevatorLimitExceeded || p.bAileronLimitExceeded || p.bRudderLimitExceeded ||
            std::abs(p.ElevatorCmd) >= kPolicySurfaceCmdLimit - 1e-12) {
            p.Classification = EClass::ControlSaturated;
            p.Diagnostic = "a surface command is at or beyond its limit";
            return p;
        }
        if (!Finite(p.ClimbMismatchMps) || p.ClimbMismatchMps > kClimbConsistencyTolMps) {
            p.Classification = EClass::SteadyStateNotReached;
            p.Diagnostic = "climb rate is inconsistent with TAS * sin(gamma)";
            return p;
        }
        if (p.ClimbRateMps > kLevelClimbThresholdMps) p.Classification = EClass::SteadyClimbFeasible;
        else if (p.ClimbRateMps < -kLevelClimbThresholdMps) p.Classification = EClass::SteadySinkFeasible;
        else p.Classification = EClass::LevelFeasible;
        return p;
    } catch (const std::exception &e) {
        p.Classification = EClass::TrimFailed;
        p.Diagnostic = std::string("exception: ") + e.what();
        return p;
    } catch (...) {
        p.Classification = EClass::TrimFailed;
        p.Diagnostic = "unknown exception";
        return p;
    }
}

void WriteNumber(std::ostream &out, double v, int precision)
{
    if (Finite(v)) out << std::fixed << std::setprecision(precision) << v;
    else out << "NA";
}

void WriteCsv(const std::string &path, const std::vector<FPoint> &points, int precision)
{
    std::ofstream out(path);
    out << "point_id,throttle_profile,throttle_cmd_norm_requested,requested_eas_mps,actual_eas_mps,tas_mps,mach,"
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
    for (const FPoint &p : points) {
        out << p.Id << ',' << ProfileName(p.Profile) << ',';
        WriteNumber(out, p.RequestedThrottleCmd, precision); out << ',';
        WriteNumber(out, p.RequestedEasMps, precision); out << ',';
        WriteNumber(out, p.ActualEasMps, precision); out << ',';
        WriteNumber(out, p.TasMps, precision); out << ',';
        WriteNumber(out, p.Mach, precision); out << ',';
        WriteNumber(out, kAltitudeFt, precision); out << ',';
        WriteNumber(out, kAltitudeFt * kFtToM, precision); out << ',';
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
        WriteNumber(out, p.ThrustLb, precision); out << ",1,";      // fuel_freeze is always on
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
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s <JSBSim-root> <raw.csv> <quantized.csv> <summary.txt>\n", argv[0]);
        return 2;
    }
    const std::string root = argv[1];

    // ---- EAS samples --------------------------------------------------------------------------
    // These are SCAN SAMPLES ONLY. They are not EasMin, EasTrim, EasMax, a stall speed, or a VNE.
    //
    // Selected from the committed trim sensitivity matrix (3824ea2), baseline condition
    // 10,000 ft / 3,000 lb, coarse (non-refined) scan set:
    //   lower  110.0 m/s      -- TrimSuccessStable; 47.5 m/s clear of the low-speed solver-failure
    //                            boundary (60 fails, 62.5 is the lowest feasible sample); alpha
    //                            +0.0922 rad; the minimum level-flight throttle sample (0.2061);
    //                            Mach 0.390; throttle margin 0.206/0.794; elevator margin 1.000.
    //   anchor 189.070713 m/s -- the known-good C1 plant-harness point.
    //   higher 235.0 m/s      -- centre of the 220-245 m/s formation profile band; alpha -0.0017;
    //                            Mach 0.833; throttle margin 0.347/0.653; elevator margin 1.000.
    //                            The matrix classified 215-255 m/s as TrimSuccessDrifting; that was
    //                            diagnosed as a roll-drift numerical-floor artifact (the anchor's own
    //                            roll drift is 1.95e-6 rad, so the 1e-5 rad floor governs), not a
    //                            solver branch change. Recorded, not treated as a defect.
    constexpr double kLowerEasMps = 110.0;
    constexpr double kAnchorEasMps = 189.070713;
    constexpr double kHigherEasMps = 235.0;
    // Idle diagnostic sweep: broad enough to expose the shape of the idle sink curve.
    const std::vector<double> idleSweep{70.0, 90.0, 110.0, 130.0, 150.0, 170.0,
                                        189.070713, 220.0, 235.0, 250.0, 280.0};
    const std::vector<double> poweredEas{kLowerEasMps, kAnchorEasMps, kHigherEasMps};

    // ---- pass 1: level reference (tFull, gamma = 0) at every EAS ---------------------------------
    // Horizontal-equilibrium reference only. Never reused as a fixed-throttle vertical profile and
    // never recommended as a production throttle_trim.
    std::vector<FPoint> points;
    std::map<double, double> levelThrottle;
    std::map<double, bool> levelThrottleOk;
    int id = 0;
    for (double eas : idleSweep) {
        FPoint p = RunPoint(root, id++, EProfile::LevelReference, 0.0, eas, kNa, false);
        levelThrottleOk[eas] = Equilibrium(p.Classification);
        levelThrottle[eas] = Equilibrium(p.Classification) ? p.ThrottleCmdNorm : kNa;
        points.push_back(p);
    }
    auto levelFor = [&](double eas) {
        const auto it = levelThrottle.find(eas);
        return it == levelThrottle.end() ? kNa : it->second;
    };
    auto levelOkFor = [&](double eas) {
        const auto it = levelThrottleOk.find(eas);
        return it != levelThrottleOk.end() && it->second;
    };

    // ---- pass 2: fixed-throttle equilibria --------------------------------------------------------
    for (double eas : idleSweep)
        points.push_back(RunPoint(root, id++, EProfile::Idle, 0.0, eas, levelFor(eas), levelOkFor(eas)));
    for (double eas : poweredEas)
        points.push_back(RunPoint(root, id++, EProfile::Military, 0.5, eas, levelFor(eas), levelOkFor(eas)));
    for (double eas : poweredEas)
        points.push_back(RunPoint(root, id++, EProfile::Augmented, 1.0, eas, levelFor(eas), levelOkFor(eas)));

    // ---- gates -----------------------------------------------------------------------------------
    std::uint64_t failures = 0;
    auto require = [&failures](bool c) { if (!c) ++failures; };

    std::map<EClass, std::size_t> totals;
    std::uint64_t wow = 0, speedbrake = 0, nonFinite = 0, external = 0, throttleMismatch = 0, fdmFailed = 0;
    std::size_t policyCompatible = 0, equilibria = 0;
    for (const FPoint &p : points) {
        ++totals[p.Classification];
        if (p.bWow) ++wow;
        if (p.bSpeedbrakeUnexpected) ++speedbrake;
        if (p.Classification == EClass::NonFinite) ++nonFinite;
        if (p.Classification == EClass::ThrottleConditionMismatch) ++throttleMismatch;
        if (p.Classification == EClass::FdmRunFailed) ++fdmFailed;
        external += p.ExternalWriteAttempts;
        require(p.ExternalWriteAttempts == 0);
        require(p.bAllAddStatesOk);                       // every AddState must have succeeded
        if (Equilibrium(p.Classification)) {
            ++equilibria;
            if (p.bCommandableUnderCurrentPolicy) ++policyCompatible;
            require(p.EasMismatchMps <= kEasMismatchTolMps);
            require(p.MaxLinearResidual <= kLinearResidualTol);
            require(p.MaxAngularResidual <= kAngularResidualTol);
            require(p.ClimbMismatchMps <= kClimbConsistencyTolMps);
            require(!p.bWow && !p.bSpeedbrakeUnexpected);
            require(std::abs(p.SpeedbrakePos) <= 1e-9);
            require(std::abs(p.GearPos) <= 1e-9 && std::abs(p.FlapPosNorm) <= 1e-9);
            require(p.bEngineRunning);
            require(Finite(p.ThrustLb));
            require(Finite(p.TankLb[0]) && p.TankLb[0] <= kInternalTankCapacityLb + 1e-9);
        }
        if (p.Profile != EProfile::LevelReference && p.bTrimSuccess) {
            // the fixed-throttle contract holds even on points that are later reclassified
            require(std::abs(p.ThrottleCmdNorm - p.RequestedThrottleCmd) <= 1e-12);
            require(std::abs(p.ThrottlePosNorm - 2.0 * p.RequestedThrottleCmd) <= 1e-9);
        }
    }
    std::size_t classified = 0;
    for (EClass c : kAllClasses) classified += totals[c];
    require(classified == points.size());
    require(wow == 0 && speedbrake == 0 && nonFinite == 0 && external == 0 && throttleMismatch == 0 && fdmFailed == 0);

    // At least one of each equilibrium kind must exist. NOT required: that every solver equilibrium
    // is inside the current pitch policy, that every EAS trims, or that any particular number is hit.
    const FPoint *anchorLevel = nullptr;
    std::size_t idleSink = 0, milClimb = 0, augClimb = 0;
    for (const FPoint &p : points) {
        if (p.Profile == EProfile::LevelReference && std::abs(p.RequestedEasMps - kAnchorEasMps) < 1e-9 &&
            p.Classification == EClass::LevelFeasible) anchorLevel = &p;
        if (p.Profile == EProfile::Idle && p.Classification == EClass::SteadySinkFeasible) ++idleSink;
        if (p.Profile == EProfile::Military && p.Classification == EClass::SteadyClimbFeasible) ++milClimb;
        if (p.Profile == EProfile::Augmented && p.Classification == EClass::SteadyClimbFeasible) ++augClimb;
    }
    require(anchorLevel != nullptr);
    require(idleSink >= 1);
    require(milClimb >= 1);
    require(augClimb >= 1);

    // ---- output ----------------------------------------------------------------------------------
    WriteCsv(argv[2], points, 15);
    WriteCsv(argv[3], points, 9);

    auto byProfile = [&](EProfile prof) {
        std::vector<const FPoint *> v;
        for (const FPoint &p : points) if (p.Profile == prof) v.push_back(&p);
        std::sort(v.begin(), v.end(), [](const FPoint *a, const FPoint *b) {
            return a->RequestedEasMps < b->RequestedEasMps; });
        return v;
    };

    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "F16_VERTICAL_PERFORMANCE_V2\n";
    out << kInterpretation << '\n';
    out << kSeparation << '\n';
    out << "model=f16 data=Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim "
           "solver=FGTrim(tCustom){tUdot<-tGamma,tWdot<-tAlpha,tQdot<-tElevator,tVdot<-tPhi,"
           "tPdot<-tAileron,tRdot<-tRudder,tHmgt<-tBeta} level_reference_solver=FGTrim(tFull)\n";
    out << "condition altitude_ft=" << kAltitudeFt << " altitude_m=" << kAltitudeFt * kFtToM
        << " fuel_lb=3000 tanks_lb=" << kTank0Lb << '|' << kTank1Lb << '|' << kTank2Lb << '|' << kTank3Lb
        << " wind_mps=0 gear=up flap=clean speedbrake=retracted fuel=frozen"
        << " heading_deg=" << kHeadingDeg << " fdm_dt_s=" << kFdmDtS << '\n';
    out << "eas_samples_mps lower=" << kLowerEasMps << " anchor=" << kAnchorEasMps
        << " higher=" << kHigherEasMps
        << " idle_sweep=70,90,110,130,150,170,189.070713,220,235,250,280"
        << " note=scan_samples_only_not_EasMin_EasTrim_EasMax_stall_or_VNE\n";
    out << "throttle_profiles idle_cmd=0.0 military_dry_cmd=0.5 augmented_cmd=1.0"
           " mapping=f16xml_fcs_gain_2_and_F100_augmethod_2"
           " position=2x_command idle_pos=0.0 military_pos=1.0 augmented_pos=2.0"
           " production_ThrottleMax=1.0_admits_full_afterburner_recorded_not_decided\n";
    out << "tolerances trim_tolerance=" << kTrimTolerance
        << " linear_residual_ftps2=" << kLinearResidualTol
        << " angular_residual_radps2=" << kAngularResidualTol
        << " eas_mismatch_mps=" << kEasMismatchTolMps
        << " climb_consistency_mps=" << kClimbConsistencyTolMps << '\n';
    out << "current_policy pitch_min_rad=" << kPolicyPitchMinRad << " pitch_max_rad=" << kPolicyPitchMaxRad
        << " throttle_min=" << kPolicyThrottleMin << " throttle_max=" << kPolicyThrottleMax
        << " surface_cmd_limit=" << kPolicySurfaceCmdLimit << " source=FGuidanceConfigV2_FF16StickConfigV2_defaults\n";
    out << "total_points=" << points.size() << '\n';
    out << "classifications";
    for (EClass c : kAllClasses) out << ' ' << ClassName(c) << '=' << totals[c];
    out << '\n';
    out << "classified=" << classified << " unclassified=" << (points.size() - classified)
        << " solver_equilibria=" << equilibria
        << " commandable_under_current_policy=" << policyCompatible
        << " solver_feasible_but_outside_current_policy=" << (equilibria - policyCompatible)
        << " failures=" << failures << '\n';

    // per-profile detail
    for (EProfile prof : {EProfile::LevelReference, EProfile::Idle, EProfile::Military, EProfile::Augmented}) {
        for (const FPoint *p : byProfile(prof)) {
            out << "point profile=" << ProfileName(prof)
                << " eas_mps=" << p->RequestedEasMps
                << " thr_cmd=" << p->RequestedThrottleCmd
                << " thr_pos=" << p->ThrottlePosNorm
                << " class=" << ClassName(p->Classification)
                << " gamma_rad=" << p->GammaRad
                << " climb_mps=" << p->ClimbRateMps
                << " pitch_rad=" << p->PitchRad
                << " alpha_rad=" << p->AlphaRad
                << " elev_cmd=" << p->ElevatorCmd
                << " thrust_lb=" << p->ThrustLb
                << " mach=" << p->Mach
                << " max_lin_res=" << p->MaxLinearResidual
                << " max_ang_res=" << p->MaxAngularResidual
                << " eas_mismatch=" << p->EasMismatchMps
                << " climb_mismatch=" << p->ClimbMismatchMps
                << " pitch_policy_exceeded=" << (p->bPitchPolicyExceeded ? 1 : 0)
                << " commandable=" << (p->bCommandableUnderCurrentPolicy ? 1 : 0)
                << " above_mil_mach_table=" << (p->bAboveMilitaryMachTable ? 1 : 0)
                << " level_trim_throttle=" << p->LevelTrimThrottleAtSameEas << '\n';
        }
    }

    // idle sink scan -- language is deliberately restricted to "samples within this scan".
    {
        const FPoint *lowestMag = nullptr, *highestMag = nullptr;
        int slopeSignChanges = 0; const FPoint *prev = nullptr; int prevSign = 0;
        for (const FPoint *p : byProfile(EProfile::Idle)) {
            if (p->Classification != EClass::SteadySinkFeasible) continue;
            const double mag = std::abs(p->ClimbRateMps);
            if (!lowestMag || mag < std::abs(lowestMag->ClimbRateMps)) lowestMag = p;
            if (!highestMag || mag > std::abs(highestMag->ClimbRateMps)) highestMag = p;
            if (prev) {
                const double d = std::abs(p->ClimbRateMps) - std::abs(prev->ClimbRateMps);
                const int sign = d > 0 ? 1 : (d < 0 ? -1 : 0);
                if (prevSign != 0 && sign != 0 && sign != prevSign) ++slopeSignChanges;
                if (sign != 0) prevSign = sign;
            }
            prev = p;
        }
        out << "idle_sink_scan"
            << " lowest_magnitude_sample_mps=" << (lowestMag ? std::abs(lowestMag->ClimbRateMps) : kNa)
            << " at_eas_mps=" << (lowestMag ? lowestMag->RequestedEasMps : kNa)
            << " highest_magnitude_sample_mps=" << (highestMag ? std::abs(highestMag->ClimbRateMps) : kNa)
            << " at_eas_mps=" << (highestMag ? highestMag->RequestedEasMps : kNa)
            << " magnitude_slope_sign_changes=" << slopeSignChanges
            << " local_u_shape_within_scan=" << (slopeSignChanges >= 1 ? 1 : 0)
            << " note=samples_within_this_scan_only_not_min_sink_speed_or_TECS_min_sink_rate\n";
    }

    // military / augmented climb -- solver-feasible vs currently policy-compatible, reported apart.
    for (EProfile prof : {EProfile::Military, EProfile::Augmented}) {
        const FPoint *largest = nullptr, *largestPolicy = nullptr;
        for (const FPoint *p : byProfile(prof)) {
            if (p->Classification != EClass::SteadyClimbFeasible) continue;
            if (!largest || p->ClimbRateMps > largest->ClimbRateMps) largest = p;
            if (p->bCommandableUnderCurrentPolicy &&
                (!largestPolicy || p->ClimbRateMps > largestPolicy->ClimbRateMps)) largestPolicy = p;
        }
        out << (prof == EProfile::Military ? "military_climb" : "augmented_climb")
            << " largest_solver_feasible_sample_mps=" << (largest ? largest->ClimbRateMps : kNa)
            << " at_eas_mps=" << (largest ? largest->RequestedEasMps : kNa)
            << " required_pitch_rad=" << (largest ? largest->PitchRad : kNa)
            << " largest_currently_policy_compatible_sample_mps=" << (largestPolicy ? largestPolicy->ClimbRateMps : kNa)
            << " at_eas_mps=" << (largestPolicy ? largestPolicy->RequestedEasMps : kNa)
            << " required_pitch_rad=" << (largestPolicy ? largestPolicy->PitchRad : kNa)
            << " note=not_an_F16_maximum_climb_rate_not_a_TECS_max_climb_rate_not_operational_capability\n";
    }

    // extreme solver equilibria that the current policy cannot command
    out << "extreme_solver_equilibria_outside_current_policy";
    for (const FPoint &p : points)
        if (Equilibrium(p.Classification) && !p.bCommandableUnderCurrentPolicy)
            out << ' ' << ProfileName(p.Profile) << '@' << p.RequestedEasMps
                << ":gamma=" << p.GammaRad << ",pitch=" << p.PitchRad << ",climb=" << p.ClimbRateMps;
    out << '\n';

    double worstLin = 0.0, worstAng = 0.0, worstEas = 0.0, worstClimb = 0.0;
    for (const FPoint &p : points) {
        if (!Equilibrium(p.Classification)) continue;
        worstLin = std::max(worstLin, p.MaxLinearResidual);
        worstAng = std::max(worstAng, p.MaxAngularResidual);
        worstEas = std::max(worstEas, p.EasMismatchMps);
        worstClimb = std::max(worstClimb, p.ClimbMismatchMps);
    }
    out << "worst_over_equilibria linear_residual_ftps2=" << worstLin
        << " angular_residual_radps2=" << worstAng
        << " eas_mismatch_mps=" << worstEas
        << " climb_consistency_mps=" << worstClimb << '\n';

    std::size_t alphaEx = 0, machEx = 0, densEx = 0, aboveMil = 0;
    for (const FPoint &p : points) {
        if (p.bAlphaTableExceeded) ++alphaEx;
        if (p.bEngineMachTableExceeded) ++machEx;
        if (p.bDensityAltTableExceeded) ++densEx;
        if (p.bAboveMilitaryMachTable) ++aboveMil;
    }
    out << "source_range alpha_table_exceeded=" << alphaEx
        << " engine_mach_table_exceeded=" << machEx
        << " density_altitude_table_exceeded=" << densEx
        << " above_military_mach_table=" << aboveMil
        << " bounds_alpha_rad=" << kAeroAlphaMinRad << ',' << kAeroAlphaMaxRad
        << " bounds_mach_aug=" << kEngineMachMaxAug << " bounds_mach_mil=" << kEngineMachMaxMil
        << " bounds_density_alt_ft=" << kEngineDensityAltMinFt << ',' << kEngineDensityAltMaxFt << '\n';
    out << "quality unexpected_wow=" << wow << " unexpected_speedbrake_deployment=" << speedbrake
        << " non_finite=" << nonFinite << " fdm_run_failed=" << fdmFailed
        << " throttle_condition_mismatch=" << throttleMismatch
        << " external_write_attempts=" << external << '\n';
    out << "production_writer_invocations=0 ue_world_loaded=0 game_pawns_searched=0 active_connection=0 "
           "tecs_parameters_modified=0 aircraft_xml_modified=0 surface_position_direct_writes=0 "
           "aerodynamic_property_direct_writes=0\n";

    const std::string summary = out.str();
    std::ofstream sf(argv[4]); sf << summary;
    std::fputs(summary.c_str(), stdout);
    std::printf("F16_VERTICAL_PERFORMANCE_V2_RESULT=%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
