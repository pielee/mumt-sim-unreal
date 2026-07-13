// verify_f16_trim_sensitivity_matrix_v2.cpp
//
// F-16 level-flight trim sensitivity matrix: solver-feasible level-flight trim characterization
// across model-derived altitude and fuel conditions.
//
// This matrix characterizes solver-feasible level-flight trim behavior across selected altitude and
// fuel conditions. It is not an operational flight envelope and is not used directly as a TECS
// parameter set.
//
// Method (identical in kind to the committed single-condition level trim map, extended over a grid):
//   * the actual f16.xml model shipped with the JSBSim plugin,
//   * the official full-aircraft solver FGTrim(FGFDMExec*, tFull).DoTrim(),
//   * a post-trim fixed-step hold at 1/120 s with the trim commands re-asserted every frame.
// Every point owns a fresh FGFDMExec; no FDM, FCS, engine, fuel, command or timestamp state is
// shared between points. No UE World, Pawn, or production writer is touched.
//
// Grid rationale, read out of the shipped model (not assumed):
//   * fuel      -- f16.xml declares four FUEL tanks. Tanks 0 and 1 are internal (x=-174.4 in,
//                  y=+/-65.0 in, z=+5.0 in) with capacity 3486 lb each and XML contents 1500 lb
//                  each, i.e. the 3000 lb baseline. Tanks 2 and 3 are external (z=-15.0 in) with
//                  capacity 2991 lb each and XML contents 0. The fuel grid therefore varies the two
//                  internal tanks symmetrically inside their declared capacity and leaves the
//                  external tanks empty. JSBSim recomputes mass/CG/inertia from the tank contents;
//                  total mass is never forced directly.
//   * altitude  -- engine/F100-PW-229.xml indexes IdleThrust/MilThrust/AugThrust on
//                  atmosphere/density-altitude over [-10000, 60000] ft. The altitude grid stays
//                  strictly inside that source range and well clear of the ground.
//   * mach      -- the engine thrust tables span Mach [0, 1.0] (idle), [0, 1.4] (mil) and
//                  [0, 2.6] (aug). The f16.xml aerodynamics carry no Mach breakpoints at all.
//   * alpha     -- the f16.xml aerodynamic tables span alpha [-0.5236, 0.7850] rad.
// ModelRangeExceeded is raised only against those literal source-table bounds.
#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "initialization/FGTrim.h"
#include "models/FGAtmosphere.h"
#include "models/FGAuxiliary.h"
#include "models/FGFCS.h"
#include "models/FGGroundReactions.h"
#include "models/FGMassBalance.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"
#include "models/propulsion/FGTank.h"
#include "math/FGMatrix33.h"
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
constexpr double kInitialLatitudeDeg = 47.0;
constexpr double kInitialLongitudeDeg = -122.0;
constexpr double kHeadingDeg = 90.0;

// Scan bounds. These are CHARACTERIZATION SCAN BOUNDS ONLY. A success at a bound is not a physical
// minimum or maximum speed of any kind.
constexpr double kScanLowerMps = 60.0;
constexpr double kScanUpperMps = 300.0;
constexpr double kCoarseStepMps = 5.0;     // identical to the committed baseline map's scan
constexpr double kRefineStepMps = 2.5;     // phase B, only around observed transitions
constexpr double kKnownGoodAnchorMps = 189.070713;

constexpr double kSettleS = 5.0;
constexpr double kMeasureS = 10.0;
constexpr double kTargetMismatchToleranceMps = 1.0e-6;
constexpr double kAnchorDriftMultiplier = 5.0;
constexpr double kAltitudeDriftFloorM = 1.0e-3;
constexpr double kSpeedDriftFloorMps = 1.0e-4;
constexpr double kAngleDriftFloorRad = 1.0e-5;
constexpr double kClimbRateFloorMps = 1.0e-4;
constexpr double kFiniteLimit = 1.0e12;

// Literal source-table bounds read out of the shipped model files (see header comment).
constexpr double kAeroAlphaMinRad = -0.5236;          // f16.xml aerodynamics
constexpr double kAeroAlphaMaxRad = 0.7850;           // f16.xml aerodynamics
constexpr double kEngineMachMax = 2.6;                // F100-PW-229.xml AugThrust (widest table)
constexpr double kEngineMilMachMax = 1.4;             // F100-PW-229.xml MilThrust (recorded, not a class)
constexpr double kEngineDensityAltMinFt = -10000.0;   // F100-PW-229.xml thrust tables
constexpr double kEngineDensityAltMaxFt = 60000.0;    // F100-PW-229.xml thrust tables

// Internal tank capacity declared in f16.xml (tanks 0 and 1).
constexpr double kInternalTankCapacityLb = 3486.0;

// Committed baseline reference, commit 524ad91 (F16_LEVEL_TRIM_MAP_V2, 10000 ft / 3000 lb).
constexpr double kBaselineAltitudeFt = 10000.0;
constexpr double kBaselineFuelLb = 3000.0;
constexpr std::size_t kBaselineStable = 40;
constexpr std::size_t kBaselineDrifting = 9;
constexpr std::size_t kBaselineFailed = 1;
constexpr double kBaselineFailedEasMps = 60.0;
constexpr double kBaselineLowestFeasibleMps = 65.0;
constexpr double kBaselineHighestFeasibleMps = 300.0;
constexpr double kBaselineDriftingLowMps = 215.0;
constexpr double kBaselineDriftingHighMps = 255.0;
constexpr double kBaselineAnchorActualEasMps = 189.070713;

const char *kInterpretation =
    "This matrix characterizes solver-feasible level-flight trim behavior across selected altitude "
    "and fuel conditions. It is not an operational flight envelope and is not used directly as a "
    "TECS parameter set.";

bool Finite(double value)
{
    return std::isfinite(value) && std::abs(value) < kFiniteLimit;
}

enum class EPointClass : std::uint8_t {
    TrimSuccessStable,
    TrimSuccessDrifting,
    TrimFailed,
    RunICFailed,
    PostTrimRunFailed,
    NonFinite,
    UnexpectedWOW,
    TargetMismatch,
    InvalidInput,
    ModelRangeExceeded
};

const char *ClassName(EPointClass value)
{
    switch (value) {
    case EPointClass::TrimSuccessStable: return "TrimSuccessStable";
    case EPointClass::TrimSuccessDrifting: return "TrimSuccessDrifting";
    case EPointClass::TrimFailed: return "TrimFailed";
    case EPointClass::RunICFailed: return "RunICFailed";
    case EPointClass::PostTrimRunFailed: return "PostTrimRunFailed";
    case EPointClass::NonFinite: return "NonFinite";
    case EPointClass::UnexpectedWOW: return "UnexpectedWOW";
    case EPointClass::TargetMismatch: return "TargetMismatch";
    case EPointClass::InvalidInput: return "InvalidInput";
    case EPointClass::ModelRangeExceeded: return "ModelRangeExceeded";
    }
    return "InvalidInput";
}

constexpr std::array<EPointClass, 10> kAllClasses{
    EPointClass::TrimSuccessStable, EPointClass::TrimSuccessDrifting, EPointClass::TrimFailed,
    EPointClass::RunICFailed, EPointClass::PostTrimRunFailed, EPointClass::NonFinite,
    EPointClass::UnexpectedWOW, EPointClass::TargetMismatch, EPointClass::InvalidInput,
    EPointClass::ModelRangeExceeded};

bool SolverFeasible(EPointClass value)
{
    return value == EPointClass::TrimSuccessStable || value == EPointClass::TrimSuccessDrifting;
}

struct FCondition {
    int Id{};
    double AltitudeFt{};
    double RequestedFuelLb{};
    double Tank0Lb{}, Tank1Lb{}, Tank2Lb{}, Tank3Lb{};

    bool IsBaseline() const
    {
        return std::abs(AltitudeFt - kBaselineAltitudeFt) < 1e-9 &&
               std::abs(RequestedFuelLb - kBaselineFuelLb) < 1e-9;
    }
};

struct FState {
    double SimTimeS{std::numeric_limits<double>::quiet_NaN()};
    double EasMps{std::numeric_limits<double>::quiet_NaN()};
    double TasMps{std::numeric_limits<double>::quiet_NaN()};
    double GroundSpeedMps{std::numeric_limits<double>::quiet_NaN()};
    double Mach{std::numeric_limits<double>::quiet_NaN()};
    double AltitudeM{std::numeric_limits<double>::quiet_NaN()};
    double DensityAltitudeFt{std::numeric_limits<double>::quiet_NaN()};
    double DensitySlugFt3{std::numeric_limits<double>::quiet_NaN()};
    double DensityRatio{std::numeric_limits<double>::quiet_NaN()};
    double SoundSpeedMps{std::numeric_limits<double>::quiet_NaN()};
    double FlightPathAngleRad{std::numeric_limits<double>::quiet_NaN()};
    double PitchRad{std::numeric_limits<double>::quiet_NaN()};
    double RollRad{std::numeric_limits<double>::quiet_NaN()};
    double HeadingRad{std::numeric_limits<double>::quiet_NaN()};
    double AlphaRad{std::numeric_limits<double>::quiet_NaN()};
    double BetaRad{std::numeric_limits<double>::quiet_NaN()};
    double P{std::numeric_limits<double>::quiet_NaN()};
    double Q{std::numeric_limits<double>::quiet_NaN()};
    double R{std::numeric_limits<double>::quiet_NaN()};
    double ClimbRateMps{std::numeric_limits<double>::quiet_NaN()};
    double LoadFactor{std::numeric_limits<double>::quiet_NaN()};
    double ThrottleCmd{std::numeric_limits<double>::quiet_NaN()};
    double ThrottlePosition{std::numeric_limits<double>::quiet_NaN()};
    double ElevatorCmd{std::numeric_limits<double>::quiet_NaN()};
    double ElevatorPositionRad{std::numeric_limits<double>::quiet_NaN()};
    double AileronCmd{std::numeric_limits<double>::quiet_NaN()};
    double AileronPositionRad{std::numeric_limits<double>::quiet_NaN()};
    double RudderCmd{std::numeric_limits<double>::quiet_NaN()};
    double RudderPositionRad{std::numeric_limits<double>::quiet_NaN()};
    double PitchTrimCmd{std::numeric_limits<double>::quiet_NaN()};
    double RollTrimCmd{std::numeric_limits<double>::quiet_NaN()};
    double YawTrimCmd{std::numeric_limits<double>::quiet_NaN()};
    bool bWow{};

    bool IsFinite() const
    {
        const std::array<double, 31> values{
            SimTimeS, EasMps, TasMps, GroundSpeedMps, Mach, AltitudeM, DensitySlugFt3,
            DensityRatio, SoundSpeedMps, FlightPathAngleRad, PitchRad, RollRad, HeadingRad,
            AlphaRad, BetaRad, P, Q, R, ClimbRateMps, LoadFactor,
            ThrottleCmd, ThrottlePosition, ElevatorCmd, ElevatorPositionRad,
            AileronCmd, AileronPositionRad, RudderCmd, RudderPositionRad,
            PitchTrimCmd, RollTrimCmd, YawTrimCmd};
        return std::all_of(values.begin(), values.end(), Finite);
    }
};

struct FHoldMetrics {
    double AltitudeMin{std::numeric_limits<double>::infinity()}, AltitudeMax{-std::numeric_limits<double>::infinity()};
    double EasMin{std::numeric_limits<double>::infinity()}, EasMax{-std::numeric_limits<double>::infinity()};
    double TasMin{std::numeric_limits<double>::infinity()}, TasMax{-std::numeric_limits<double>::infinity()};
    double PitchMin{std::numeric_limits<double>::infinity()}, PitchMax{-std::numeric_limits<double>::infinity()};
    double RollMin{std::numeric_limits<double>::infinity()}, RollMax{-std::numeric_limits<double>::infinity()};
    double ClimbMin{std::numeric_limits<double>::infinity()}, ClimbMax{-std::numeric_limits<double>::infinity()};
    double AlphaMin{std::numeric_limits<double>::infinity()}, AlphaMax{-std::numeric_limits<double>::infinity()};
    double ThrottleMin{std::numeric_limits<double>::infinity()}, ThrottleMax{-std::numeric_limits<double>::infinity()};
    double StabilatorMin{std::numeric_limits<double>::infinity()}, StabilatorMax{-std::numeric_limits<double>::infinity()};
    std::uint64_t Samples{}, WowCount{}, RunFailures{}, NonFiniteCount{};

    void Add(const FState &state)
    {
        ++Samples;
        AltitudeMin = std::min(AltitudeMin, state.AltitudeM); AltitudeMax = std::max(AltitudeMax, state.AltitudeM);
        EasMin = std::min(EasMin, state.EasMps); EasMax = std::max(EasMax, state.EasMps);
        TasMin = std::min(TasMin, state.TasMps); TasMax = std::max(TasMax, state.TasMps);
        PitchMin = std::min(PitchMin, state.PitchRad); PitchMax = std::max(PitchMax, state.PitchRad);
        RollMin = std::min(RollMin, state.RollRad); RollMax = std::max(RollMax, state.RollRad);
        ClimbMin = std::min(ClimbMin, state.ClimbRateMps); ClimbMax = std::max(ClimbMax, state.ClimbRateMps);
        AlphaMin = std::min(AlphaMin, state.AlphaRad); AlphaMax = std::max(AlphaMax, state.AlphaRad);
        ThrottleMin = std::min(ThrottleMin, state.ThrottleCmd); ThrottleMax = std::max(ThrottleMax, state.ThrottleCmd);
        StabilatorMin = std::min(StabilatorMin, state.ElevatorPositionRad);
        StabilatorMax = std::max(StabilatorMax, state.ElevatorPositionRad);
        if (state.bWow) ++WowCount;
        if (!state.IsFinite()) ++NonFiniteCount;
    }

    double AltitudeDrift() const { return AltitudeMax - AltitudeMin; }
    double EasDrift() const { return EasMax - EasMin; }
    double TasDrift() const { return TasMax - TasMin; }
    double PitchDrift() const { return PitchMax - PitchMin; }
    double RollDrift() const { return RollMax - RollMin; }
    double MaxAbsClimb() const { return std::max(std::abs(ClimbMin), std::abs(ClimbMax)); }
};

struct FStabilityLimits {
    double AltitudeM{}, EasMps{}, TasMps{}, PitchRad{}, RollRad{}, ClimbRateMps{};
};

struct FPoint {
    int ConditionId{};
    double AltitudeFt{}, RequestedFuelLb{};
    double RequestedEasMps{};
    bool bAnchor{}, bRefined{};
    EPointClass Classification{EPointClass::InvalidInput};
    bool bRunIcSuccess{}, bTrimAttempted{}, bTrimSuccess{}, bPostTrimRunSuccess{};
    bool bProcessSurvived{true};
    bool bModelRangeExceeded{};
    bool bMachAboveMilTable{};
    std::string Diagnostic;
    FState TrimState{};
    FHoldMetrics Hold{};
    double TargetMismatchMps{std::numeric_limits<double>::quiet_NaN()};

    // condition state, read back from the model
    double MassSlugs{std::numeric_limits<double>::quiet_NaN()};
    double MassKg{std::numeric_limits<double>::quiet_NaN()};
    double WeightLb{std::numeric_limits<double>::quiet_NaN()};
    double EmptyWeightLb{std::numeric_limits<double>::quiet_NaN()};
    double FuelTotalLb{std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 4> TankLb{{std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN()}};
    double CgXIn{std::numeric_limits<double>::quiet_NaN()};
    double CgYIn{std::numeric_limits<double>::quiet_NaN()};
    double CgZIn{std::numeric_limits<double>::quiet_NaN()};
    double Ixx{std::numeric_limits<double>::quiet_NaN()};
    double Iyy{std::numeric_limits<double>::quiet_NaN()};
    double Izz{std::numeric_limits<double>::quiet_NaN()};
    double Ixz{std::numeric_limits<double>::quiet_NaN()};
    bool bInertiaAvailable{};
    double GearPos{std::numeric_limits<double>::quiet_NaN()};
    double FlapPosNorm{std::numeric_limits<double>::quiet_NaN()};
    double SpeedbrakePos{std::numeric_limits<double>::quiet_NaN()};
    double EngineThrustLb{std::numeric_limits<double>::quiet_NaN()};
    bool bEngineRunning{};
    double WindMps{0.0};

    double ThrottleLowerMargin{std::numeric_limits<double>::quiet_NaN()};
    double ThrottleUpperMargin{std::numeric_limits<double>::quiet_NaN()};
    double ElevatorInputMargin{std::numeric_limits<double>::quiet_NaN()};
    double AileronInputMargin{std::numeric_limits<double>::quiet_NaN()};
    double RudderInputMargin{std::numeric_limits<double>::quiet_NaN()};
    std::uint64_t OwnedCommandFrames{}, ExternalWriteAttempts{};
};

class FTrimPlant {
public:
    bool Initialize(const std::string &root, const FCondition &condition, double requestedEasMps, FPoint &point)
    {
        Exec = std::make_unique<JSBSim::FGFDMExec>();
        OwnedIdentity = Exec.get();
        Exec->SetDebugLevel(0);
        Exec->SetRootDir(SGPath(root));
        Exec->SetAircraftPath(SGPath("aircraft"));
        Exec->SetEnginePath(SGPath("engine"));
        Exec->SetSystemsPath(SGPath("systems"));
        if (!Exec->LoadModel("f16")) {
            point.Diagnostic = "LoadModel(f16) failed";
            return false;
        }
        Exec->Setdt(kFdmDtS);
        IC = Exec->GetIC();
        Fcs = Exec->GetFCS();
        Propagate = Exec->GetPropagate();
        Auxiliary = Exec->GetAuxiliary();
        Atmosphere = Exec->GetAtmosphere();
        Propulsion = Exec->GetPropulsion();
        Ground = Exec->GetGroundReactions();
        MassBalance = Exec->GetMassBalance();
        if (!IC || !Fcs || !Propagate || !Auxiliary || !Atmosphere || !Propulsion || !Ground || !MassBalance) {
            point.Diagnostic = "required JSBSim model pointer is null";
            return false;
        }

        // Fuel distribution: set the declared tank contents and let JSBSim recompute mass/CG/inertia.
        // The total mass is never forced directly.
        if (Propulsion->GetNumTanks() != 4) {
            point.Diagnostic = "f16 model does not declare the expected 4 fuel tanks";
            return false;
        }
        const std::array<double, 4> requested{condition.Tank0Lb, condition.Tank1Lb, condition.Tank2Lb, condition.Tank3Lb};
        for (unsigned int index = 0; index < 4; ++index) {
            auto tank = Propulsion->GetTank(index);
            if (!tank) { point.Diagnostic = "null fuel tank"; return false; }
            if (requested[index] < 0.0 || requested[index] > tank->GetCapacity() + 1e-9) {
                point.Diagnostic = "requested tank fuel exceeds the declared model capacity";
                return false;
            }
            tank->SetContents(requested[index]);
        }

        IC->SetGeodLatitudeDegIC(kInitialLatitudeDeg);
        IC->SetLongitudeDegIC(kInitialLongitudeDeg);
        IC->SetAltitudeASLFtIC(condition.AltitudeFt);
        IC->SetPsiDegIC(kHeadingDeg);
        IC->SetPhiDegIC(0.0);
        IC->SetThetaDegIC(0.0);
        IC->SetWindDirDegIC(0.0);
        IC->SetWindMagKtsIC(0.0);
        IC->SetWindDownKtsIC(0.0);
        Exec->SetPropertyValue("ic/gamma-deg", 0.0);
        Exec->SetPropertyValue("ic/p-rad_sec", 0.0);
        Exec->SetPropertyValue("ic/q-rad_sec", 0.0);
        Exec->SetPropertyValue("ic/r-rad_sec", 0.0);
        IC->SetVequivalentKtsIC(requestedEasMps / kKnotToMps);
        point.bRunIcSuccess = Exec->RunIC();
        if (!point.bRunIcSuccess) {
            point.Diagnostic = "RunIC failed";
            return false;
        }

        Propulsion->InitRunning(-1);
        Propulsion->SetFuelFreeze(true);
        Fcs->SetGearCmd(0.0);
        Fcs->SetGearPos(0.0);
        Fcs->SetDfCmd(0.0);
        Fcs->SetDfPos(JSBSim::ofNorm, 0.0);
        Fcs->SetDsbCmd(0.0);
        Fcs->SetThrottleCmd(0, 0.6);
        ReadCondition(point);

        point.bTrimAttempted = true;
        JSBSim::FGTrim trim(Exec.get(), JSBSim::tFull);
        point.bTrimSuccess = trim.DoTrim();
        if (!point.bTrimSuccess) {
            trim.Report();
            trim.TrimStats();
            point.Diagnostic = "FGTrim(tFull).DoTrim() returned false";
            return true;
        }

        HoldDa = Fcs->GetDaCmd(); HoldDe = Fcs->GetDeCmd(); HoldDr = Fcs->GetDrCmd();
        HoldThrottle = Fcs->GetThrottleCmd(0);
        HoldPitchTrim = Fcs->GetPitchTrimCmd(); HoldRollTrim = Fcs->GetRollTrimCmd(); HoldYawTrim = Fcs->GetYawTrimCmd();
        point.TrimState = Read();
        ReadCondition(point);
        point.TargetMismatchMps = std::abs(point.TrimState.EasMps - requestedEasMps);
        point.ThrottleLowerMargin = HoldThrottle;
        point.ThrottleUpperMargin = 1.0 - HoldThrottle;
        point.ElevatorInputMargin = 1.0 - std::abs(HoldDe);
        point.AileronInputMargin = 1.0 - std::abs(HoldDa);
        point.RudderInputMargin = 1.0 - std::abs(HoldDr);
        return true;
    }

    void ReadCondition(FPoint &point) const
    {
        point.MassSlugs = MassBalance->GetMass();
        point.MassKg = MassBalance->GetMass() * 14.593902937206363;   // slug -> kg
        point.WeightLb = MassBalance->GetWeight();
        point.EmptyWeightLb = MassBalance->GetEmptyWeight();
        point.FuelTotalLb = Exec->GetPropertyValue("propulsion/total-fuel-lbs");
        for (unsigned int index = 0; index < 4; ++index) {
            auto tank = Propulsion->GetTank(index);
            point.TankLb[index] = tank ? tank->GetContents() : std::numeric_limits<double>::quiet_NaN();
        }
        point.CgXIn = MassBalance->GetXYZcg(1);
        point.CgYIn = MassBalance->GetXYZcg(2);
        point.CgZIn = MassBalance->GetXYZcg(3);
        const JSBSim::FGMatrix33 &inertia = MassBalance->GetJ();
        point.Ixx = inertia(1, 1); point.Iyy = inertia(2, 2);
        point.Izz = inertia(3, 3); point.Ixz = inertia(1, 3);
        point.bInertiaAvailable = Finite(point.Ixx) && Finite(point.Iyy) && Finite(point.Izz) && Finite(point.Ixz);
        point.GearPos = Fcs->GetGearPos();
        point.FlapPosNorm = Fcs->GetDfPos(JSBSim::ofNorm);
        point.SpeedbrakePos = Fcs->GetDsbPos(JSBSim::ofNorm);
        auto engine = Propulsion->GetEngine(0);
        point.bEngineRunning = engine ? engine->GetRunning() : false;
        point.EngineThrustLb = engine ? engine->GetThrust() : std::numeric_limits<double>::quiet_NaN();
        point.WindMps = 0.0;
    }

    FState Read() const
    {
        FState state{};
        state.SimTimeS = Exec->GetSimTime();
        state.EasMps = Auxiliary->GetVequivalentKTS() * kKnotToMps;
        state.TasMps = Auxiliary->GetVt() * kFtToM;
        state.GroundSpeedMps = Auxiliary->GetVground() * kFtToM;
        state.Mach = Auxiliary->GetMach();
        state.AltitudeM = Propagate->GetAltitudeASL() * kFtToM;
        state.DensityAltitudeFt = Exec->GetPropertyValue("atmosphere/density-altitude");
        state.DensitySlugFt3 = Atmosphere->GetDensity();
        state.DensityRatio = Atmosphere->GetDensityRatio();
        state.SoundSpeedMps = Atmosphere->GetSoundSpeed() * kFtToM;
        state.FlightPathAngleRad = Exec->GetPropertyValue("flight-path/gamma-rad");
        state.PitchRad = Propagate->GetEuler(JSBSim::FGJSBBase::eTht);
        state.RollRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePhi);
        state.HeadingRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePsi);
        state.AlphaRad = Auxiliary->Getalpha(); state.BetaRad = Auxiliary->Getbeta();
        state.P = Propagate->GetPQR(1); state.Q = Propagate->GetPQR(2); state.R = Propagate->GetPQR(3);
        state.ClimbRateMps = Propagate->Gethdot() * kFtToM;
        state.LoadFactor = Auxiliary->GetNlf();
        state.ThrottleCmd = Fcs->GetThrottleCmd(0); state.ThrottlePosition = Fcs->GetThrottlePos(0);
        state.ElevatorCmd = Fcs->GetDeCmd(); state.ElevatorPositionRad = Fcs->GetDePos(JSBSim::ofRad);
        state.AileronCmd = Fcs->GetDaCmd(); state.AileronPositionRad = Fcs->GetDaLPos(JSBSim::ofRad);
        state.RudderCmd = Fcs->GetDrCmd(); state.RudderPositionRad = Fcs->GetDrPos(JSBSim::ofRad);
        state.PitchTrimCmd = Fcs->GetPitchTrimCmd(); state.RollTrimCmd = Fcs->GetRollTrimCmd(); state.YawTrimCmd = Fcs->GetYawTrimCmd();
        state.bWow = Ground->GetWOW();
        return state;
    }

    bool WriteOwnedTrimCommands(FPoint &point)
    {
        if (Exec.get() != OwnedIdentity) {
            ++point.ExternalWriteAttempts;
            return false;
        }
        Fcs->SetDaCmd(HoldDa); Fcs->SetDeCmd(HoldDe); Fcs->SetDrCmd(HoldDr);
        Fcs->SetThrottleCmd(0, HoldThrottle);
        Fcs->SetPitchTrimCmd(HoldPitchTrim); Fcs->SetRollTrimCmd(HoldRollTrim); Fcs->SetYawTrimCmd(HoldYawTrim);
        ++point.OwnedCommandFrames;
        return true;
    }

    bool Run() { return Exec->Run(); }

private:
    std::unique_ptr<JSBSim::FGFDMExec> Exec;
    JSBSim::FGFDMExec *OwnedIdentity{};
    std::shared_ptr<JSBSim::FGInitialCondition> IC;
    std::shared_ptr<JSBSim::FGFCS> Fcs;
    std::shared_ptr<JSBSim::FGPropagate> Propagate;
    std::shared_ptr<JSBSim::FGAuxiliary> Auxiliary;
    std::shared_ptr<JSBSim::FGAtmosphere> Atmosphere;
    std::shared_ptr<JSBSim::FGPropulsion> Propulsion;
    std::shared_ptr<JSBSim::FGGroundReactions> Ground;
    std::shared_ptr<JSBSim::FGMassBalance> MassBalance;
    double HoldDa{}, HoldDe{}, HoldDr{}, HoldThrottle{};
    double HoldPitchTrim{}, HoldRollTrim{}, HoldYawTrim{};
};

FStabilityLimits LimitsFromAnchor(const FPoint &anchor)
{
    return {
        std::max(kAltitudeDriftFloorM, kAnchorDriftMultiplier * anchor.Hold.AltitudeDrift()),
        std::max(kSpeedDriftFloorMps, kAnchorDriftMultiplier * anchor.Hold.EasDrift()),
        std::max(kSpeedDriftFloorMps, kAnchorDriftMultiplier * anchor.Hold.TasDrift()),
        std::max(kAngleDriftFloorRad, kAnchorDriftMultiplier * anchor.Hold.PitchDrift()),
        std::max(kAngleDriftFloorRad, kAnchorDriftMultiplier * anchor.Hold.RollDrift()),
        std::max(kClimbRateFloorMps, kAnchorDriftMultiplier * anchor.Hold.MaxAbsClimb())};
}

bool StableUnder(const FPoint &point, const FStabilityLimits &limits)
{
    return point.Hold.AltitudeDrift() <= limits.AltitudeM &&
           point.Hold.EasDrift() <= limits.EasMps &&
           point.Hold.TasDrift() <= limits.TasMps &&
           point.Hold.PitchDrift() <= limits.PitchRad &&
           point.Hold.RollDrift() <= limits.RollRad &&
           point.Hold.MaxAbsClimb() <= limits.ClimbRateMps;
}

// Literal source-table bounds only. No estimated or inferred envelope is used here.
bool OutsideModelSourceTables(const FState &state)
{
    if (!Finite(state.AlphaRad) || !Finite(state.Mach) || !Finite(state.DensityAltitudeFt)) return false;
    if (state.AlphaRad < kAeroAlphaMinRad || state.AlphaRad > kAeroAlphaMaxRad) return true;
    if (state.Mach < 0.0 || state.Mach > kEngineMachMax) return true;
    if (state.DensityAltitudeFt < kEngineDensityAltMinFt || state.DensityAltitudeFt > kEngineDensityAltMaxFt) return true;
    return false;
}

FPoint RunPoint(const std::string &root, const FCondition &condition, double requestedEasMps,
                bool anchor, bool refined, const FStabilityLimits *stabilityLimits)
{
    FPoint point{};
    point.ConditionId = condition.Id;
    point.AltitudeFt = condition.AltitudeFt;
    point.RequestedFuelLb = condition.RequestedFuelLb;
    point.RequestedEasMps = requestedEasMps;
    point.bAnchor = anchor;
    point.bRefined = refined;
    if (!Finite(requestedEasMps) || requestedEasMps <= 0.0) {
        point.Classification = EPointClass::InvalidInput;
        point.Diagnostic = "requested EAS is invalid";
        return point;
    }

    try {
        FTrimPlant plant;
        if (!plant.Initialize(root, condition, requestedEasMps, point)) {
            point.Classification = point.bRunIcSuccess ? EPointClass::InvalidInput : EPointClass::RunICFailed;
            return point;
        }
        if (!point.bTrimSuccess) {
            point.Classification = EPointClass::TrimFailed;
            return point;
        }
        if (!point.TrimState.IsFinite()) {
            point.Classification = EPointClass::NonFinite;
            point.Diagnostic = "non-finite state immediately after trim";
            return point;
        }
        if (point.TrimState.bWow) {
            point.Classification = EPointClass::UnexpectedWOW;
            point.Diagnostic = "WOW immediately after airborne trim";
            return point;
        }
        if (!Finite(point.TargetMismatchMps) || point.TargetMismatchMps > kTargetMismatchToleranceMps) {
            point.Classification = EPointClass::TargetMismatch;
            point.Diagnostic = "post-trim EAS does not match requested EAS";
            return point;
        }
        point.bMachAboveMilTable = Finite(point.TrimState.Mach) && point.TrimState.Mach > kEngineMilMachMax;
        if (OutsideModelSourceTables(point.TrimState)) {
            point.bModelRangeExceeded = true;
            point.Classification = EPointClass::ModelRangeExceeded;
            point.Diagnostic = "trim state is outside the f16 aerodynamic alpha table or the engine mach/density-altitude tables";
            return point;
        }

        const int totalSteps = static_cast<int>(std::llround((kSettleS + kMeasureS) / kFdmDtS));
        const int settleSteps = static_cast<int>(std::llround(kSettleS / kFdmDtS));
        for (int step = 0; step < totalSteps; ++step) {
            if (!plant.WriteOwnedTrimCommands(point)) {
                point.Classification = EPointClass::PostTrimRunFailed;
                point.Diagnostic = "write target was not the owned FGFDMExec";
                return point;
            }
            if (!plant.Run()) {
                ++point.Hold.RunFailures;
                point.Classification = EPointClass::PostTrimRunFailed;
                point.Diagnostic = "FGFDMExec::Run returned false";
                return point;
            }
            const FState state = plant.Read();
            if (!state.IsFinite()) {
                ++point.Hold.NonFiniteCount;
                point.Classification = EPointClass::NonFinite;
                point.Diagnostic = "non-finite state during post-trim hold";
                return point;
            }
            if (state.bWow) {
                ++point.Hold.WowCount;
                point.Classification = EPointClass::UnexpectedWOW;
                point.Diagnostic = "WOW during post-trim airborne hold";
                return point;
            }
            if (step >= settleSteps) point.Hold.Add(state);
        }
        if (point.Hold.Samples == 0 || point.ExternalWriteAttempts != 0) {
            point.Classification = EPointClass::PostTrimRunFailed;
            point.Diagnostic = "post-trim measurement or ownership accounting failed";
            return point;
        }
        point.bPostTrimRunSuccess = true;
        point.Classification = stabilityLimits && !StableUnder(point, *stabilityLimits)
            ? EPointClass::TrimSuccessDrifting : EPointClass::TrimSuccessStable;
        return point;
    } catch (const std::exception &exception) {
        point.bProcessSurvived = true;
        point.Classification = EPointClass::TrimFailed;
        point.Diagnostic = std::string("exception: ") + exception.what();
        return point;
    } catch (...) {
        point.bProcessSurvived = true;
        point.Classification = EPointClass::TrimFailed;
        point.Diagnostic = "unknown exception";
        return point;
    }
}

std::vector<FCondition> BuildConditions()
{
    // Altitudes stay strictly inside the engine thrust tables' density-altitude range
    // [-10000, 60000] ft and well clear of the ground.
    const std::array<double, 5> altitudesFt{5000.0, 10000.0, 20000.0, 30000.0, 40000.0};
    // Fuel is carried in the two internal tanks only, split evenly, inside the declared 3486 lb
    // per-tank capacity. The external tanks stay empty, as in the shipped model.
    const std::array<double, 3> fuelLb{1000.0, 3000.0, 6000.0};

    std::vector<FCondition> conditions;
    int id = 0;
    for (double altitude : altitudesFt) {
        for (double fuel : fuelLb) {
            FCondition condition{};
            condition.Id = id++;
            condition.AltitudeFt = altitude;
            condition.RequestedFuelLb = fuel;
            condition.Tank0Lb = fuel * 0.5;
            condition.Tank1Lb = fuel * 0.5;
            condition.Tank2Lb = 0.0;
            condition.Tank3Lb = 0.0;
            conditions.push_back(condition);
        }
    }
    return conditions;
}

std::vector<double> CoarsePoints()
{
    std::vector<double> points;
    for (double speed = kScanLowerMps; speed <= kScanUpperMps + 1e-9; speed += kCoarseStepMps)
        points.push_back(speed);
    return points;
}

void WriteNumber(std::ostream &out, double value, int precision)
{
    if (Finite(value)) out << std::fixed << std::setprecision(precision) << value;
    else out << "NA";
}

void WriteMatrix(const std::string &path, const std::vector<FPoint> &points, int precision)
{
    std::ofstream out(path);
    out << "condition_id,altitude_ft,altitude_m,requested_fuel_lb,fuel_total_lb,tank_fuel_distribution_lb,"
           "mass_slug,mass_kg,weight_lb,empty_weight_lb,cg_x_in,cg_y_in,cg_z_in,ixx_slugft2,iyy_slugft2,"
           "izz_slugft2,ixz_slugft2,inertia_available,density_slug_ft3,density_ratio,density_altitude_ft,"
           "speed_of_sound_mps,wind_mps,gear_pos,flap_pos_norm,speedbrake_pos,engine_running,engine_thrust_lb,"
           "requested_eas_mps,actual_eas_mps,actual_tas_mps,mach,actual_altitude_m,"
           "classification,is_anchor,is_refined,trim_attempted,trim_success,run_ic_success,post_trim_run_success,"
           "target_mismatch_mps,model_range_exceeded,mach_above_mil_thrust_table,"
           "pitch_rad,roll_rad,heading_rad,flight_path_angle_rad,alpha_rad,beta_rad,p_radps,q_radps,r_radps,"
           "climb_rate_mps,ground_speed_mps,load_factor,throttle_cmd_norm,throttle_position_norm,"
           "elevator_cmd_norm,elevator_position_rad,aileron_cmd_norm,aileron_position_rad,"
           "rudder_cmd_norm,rudder_position_rad,pitch_trim_cmd,roll_trim_cmd,yaw_trim_cmd,"
           "throttle_lower_margin,throttle_upper_margin,elevator_input_margin,aileron_input_margin,rudder_input_margin,"
           "hold_altitude_drift_m,hold_eas_drift_mps,hold_tas_drift_mps,hold_pitch_drift_rad,hold_roll_drift_rad,"
           "hold_climb_rate_min_mps,hold_climb_rate_max_mps,hold_alpha_min_rad,hold_alpha_max_rad,"
           "hold_throttle_min,hold_throttle_max,hold_stabilator_min_rad,hold_stabilator_max_rad,"
           "wow_count,fdm_run_failures,nonfinite_count,owned_command_frames,external_write_attempts,"
           "process_survived,diagnostic\n";
    for (const FPoint &p : points) {
        out << p.ConditionId << ',';
        WriteNumber(out, p.AltitudeFt, precision); out << ',';
        WriteNumber(out, p.AltitudeFt * kFtToM, precision); out << ',';
        WriteNumber(out, p.RequestedFuelLb, precision); out << ',';
        WriteNumber(out, p.FuelTotalLb, precision); out << ',';
        for (std::size_t i = 0; i < 4; ++i) { WriteNumber(out, p.TankLb[i], precision); if (i + 1 < 4) out << '|'; }
        out << ',';
        const std::array<double, 10> mass{p.MassSlugs, p.MassKg, p.WeightLb, p.EmptyWeightLb,
                                          p.CgXIn, p.CgYIn, p.CgZIn, p.Ixx, p.Iyy, p.Izz};
        for (double value : mass) { WriteNumber(out, value, precision); out << ','; }
        WriteNumber(out, p.Ixz, precision); out << ',' << (p.bInertiaAvailable ? 1 : 0) << ',';
        const std::array<double, 3> atmos{p.TrimState.DensitySlugFt3, p.TrimState.DensityRatio, p.TrimState.DensityAltitudeFt};
        for (double value : atmos) { WriteNumber(out, value, precision); out << ','; }
        WriteNumber(out, p.TrimState.SoundSpeedMps, precision); out << ',';
        WriteNumber(out, p.WindMps, precision); out << ',';
        const std::array<double, 3> config{p.GearPos, p.FlapPosNorm, p.SpeedbrakePos};
        for (double value : config) { WriteNumber(out, value, precision); out << ','; }
        out << (p.bEngineRunning ? 1 : 0) << ',';
        WriteNumber(out, p.EngineThrustLb, precision); out << ',';
        WriteNumber(out, p.RequestedEasMps, precision); out << ',';
        WriteNumber(out, p.TrimState.EasMps, precision); out << ',';
        WriteNumber(out, p.TrimState.TasMps, precision); out << ',';
        WriteNumber(out, p.TrimState.Mach, precision); out << ',';
        WriteNumber(out, p.TrimState.AltitudeM, precision); out << ',';
        out << ClassName(p.Classification) << ',' << (p.bAnchor ? 1 : 0) << ',' << (p.bRefined ? 1 : 0) << ','
            << (p.bTrimAttempted ? 1 : 0) << ',' << (p.bTrimSuccess ? 1 : 0) << ','
            << (p.bRunIcSuccess ? 1 : 0) << ',' << (p.bPostTrimRunSuccess ? 1 : 0) << ',';
        WriteNumber(out, p.TargetMismatchMps, precision); out << ',';
        out << (p.bModelRangeExceeded ? 1 : 0) << ',' << (p.bMachAboveMilTable ? 1 : 0) << ',';
        const std::array<double, 21> trim{
            p.TrimState.PitchRad, p.TrimState.RollRad, p.TrimState.HeadingRad, p.TrimState.FlightPathAngleRad,
            p.TrimState.AlphaRad, p.TrimState.BetaRad, p.TrimState.P, p.TrimState.Q, p.TrimState.R,
            p.TrimState.ClimbRateMps, p.TrimState.GroundSpeedMps, p.TrimState.LoadFactor,
            p.TrimState.ThrottleCmd, p.TrimState.ThrottlePosition, p.TrimState.ElevatorCmd,
            p.TrimState.ElevatorPositionRad, p.TrimState.AileronCmd, p.TrimState.AileronPositionRad,
            p.TrimState.RudderCmd, p.TrimState.RudderPositionRad, p.TrimState.PitchTrimCmd};
        for (double value : trim) { WriteNumber(out, value, precision); out << ','; }
        WriteNumber(out, p.TrimState.RollTrimCmd, precision); out << ',';
        WriteNumber(out, p.TrimState.YawTrimCmd, precision); out << ',';
        const std::array<double, 5> margins{p.ThrottleLowerMargin, p.ThrottleUpperMargin,
                                            p.ElevatorInputMargin, p.AileronInputMargin, p.RudderInputMargin};
        for (double value : margins) { WriteNumber(out, value, precision); out << ','; }
        const std::array<double, 13> hold{
            p.Hold.AltitudeDrift(), p.Hold.EasDrift(), p.Hold.TasDrift(), p.Hold.PitchDrift(), p.Hold.RollDrift(),
            p.Hold.ClimbMin, p.Hold.ClimbMax, p.Hold.AlphaMin, p.Hold.AlphaMax,
            p.Hold.ThrottleMin, p.Hold.ThrottleMax, p.Hold.StabilatorMin, p.Hold.StabilatorMax};
        for (double value : hold) { WriteNumber(out, value, precision); out << ','; }
        out << p.Hold.WowCount << ',' << p.Hold.RunFailures << ',' << p.Hold.NonFiniteCount << ','
            << p.OwnedCommandFrames << ',' << p.ExternalWriteAttempts << ','
            << (p.bProcessSurvived ? 1 : 0) << ',' << p.Diagnostic << '\n';
    }
}

template <typename Getter>
std::pair<double, double> FeasibleRange(const std::vector<const FPoint *> &points, Getter getter)
{
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (const FPoint *point : points) {
        if (!SolverFeasible(point->Classification)) continue;
        const double value = getter(*point);
        if (!Finite(value)) continue;
        minimum = std::min(minimum, value); maximum = std::max(maximum, value);
    }
    return {minimum, maximum};
}

// coarseOnly restricts the view to the phase-A scan set, which is deliberately the SAME 5 m/s scan
// the committed baseline map used. The baseline reproduction gate must compare like with like: a
// phase-B refined sample that the committed 5 m/s grid could not see is a NEW measurement, not a
// disagreement with it.
std::vector<const FPoint *> SortedByEas(const std::vector<FPoint> &points, int conditionId, bool coarseOnly = false)
{
    std::vector<const FPoint *> sorted;
    for (const FPoint &point : points) {
        if (point.ConditionId != conditionId) continue;
        if (coarseOnly && point.bRefined) continue;
        sorted.push_back(&point);
    }
    std::sort(sorted.begin(), sorted.end(), [](const FPoint *a, const FPoint *b) {
        if (a->RequestedEasMps != b->RequestedEasMps) return a->RequestedEasMps < b->RequestedEasMps;
        return a->bAnchor && !b->bAnchor;
    });
    return sorted;
}

std::pair<double, double> ContiguousAroundAnchor(const std::vector<const FPoint *> &sorted)
{
    std::size_t anchorIndex = sorted.size();
    for (std::size_t i = 0; i < sorted.size(); ++i) if (sorted[i]->bAnchor) anchorIndex = i;
    if (anchorIndex == sorted.size() || !SolverFeasible(sorted[anchorIndex]->Classification))
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    std::size_t lower = anchorIndex, upper = anchorIndex;
    while (lower > 0 && SolverFeasible(sorted[lower - 1]->Classification) &&
           sorted[lower]->RequestedEasMps - sorted[lower - 1]->RequestedEasMps <= kCoarseStepMps + 1e-6) --lower;
    while (upper + 1 < sorted.size() && SolverFeasible(sorted[upper + 1]->Classification) &&
           sorted[upper + 1]->RequestedEasMps - sorted[upper]->RequestedEasMps <= kCoarseStepMps + 1e-6) ++upper;
    return {sorted[lower]->RequestedEasMps, sorted[upper]->RequestedEasMps};
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s <JSBSim-root> <raw.csv> <quantized.csv> <summary.txt>\n", argv[0]);
        return 2;
    }
    const std::string root = argv[1];
    const std::vector<FCondition> conditions = BuildConditions();
    const std::vector<double> coarse = CoarsePoints();

    // Pass 1: one anchor per condition. The baseline anchor supplies the fallback stability basis
    // for any condition whose own anchor is not solver-feasible.
    std::vector<FPoint> anchors;
    anchors.reserve(conditions.size());
    for (const FCondition &condition : conditions)
        anchors.push_back(RunPoint(root, condition, kKnownGoodAnchorMps, true, false, nullptr));

    const FPoint *baselineAnchor = nullptr;
    for (std::size_t i = 0; i < conditions.size(); ++i)
        if (conditions[i].IsBaseline()) baselineAnchor = &anchors[i];
    const bool baselineAnchorFeasible = baselineAnchor && SolverFeasible(baselineAnchor->Classification);
    const FStabilityLimits fallbackLimits =
        baselineAnchorFeasible ? LimitsFromAnchor(*baselineAnchor) : FStabilityLimits{};

    // Pass 2: coarse scan, then refinement around observed classification transitions.
    std::vector<FPoint> points;
    std::vector<FStabilityLimits> conditionLimits(conditions.size());
    std::vector<bool> conditionOwnBasis(conditions.size(), false);
    std::vector<std::size_t> conditionRefinedCount(conditions.size(), 0);
    std::vector<std::string> conditionTransitions(conditions.size());

    for (std::size_t c = 0; c < conditions.size(); ++c) {
        const FCondition &condition = conditions[c];
        const FPoint &anchor = anchors[c];
        const bool ownBasis = SolverFeasible(anchor.Classification);
        conditionOwnBasis[c] = ownBasis;
        conditionLimits[c] = ownBasis ? LimitsFromAnchor(anchor) : fallbackLimits;
        const FStabilityLimits *limits =
            (ownBasis || baselineAnchorFeasible) ? &conditionLimits[c] : nullptr;

        points.push_back(anchor);
        for (double requested : coarse) {
            if (std::abs(requested - kKnownGoodAnchorMps) < 1e-9) continue;
            points.push_back(RunPoint(root, condition, requested, false, false, limits));
        }

        // Phase B: re-measure at the finer step wherever adjacent coarse points changed class.
        const std::vector<const FPoint *> sorted = SortedByEas(points, condition.Id);
        std::vector<double> refineAt;
        std::ostringstream transitions;
        for (std::size_t i = 0; i + 1 < sorted.size(); ++i) {
            if (sorted[i]->Classification == sorted[i + 1]->Classification) continue;
            const double lower = sorted[i]->RequestedEasMps, upper = sorted[i + 1]->RequestedEasMps;
            if (upper - lower <= kRefineStepMps + 1e-9) continue;
            transitions << ClassName(sorted[i]->Classification) << "->" << ClassName(sorted[i + 1]->Classification)
                        << '@' << std::fixed << std::setprecision(3) << lower << '-' << upper << ';';
            for (double speed = lower + kRefineStepMps; speed < upper - 1e-9; speed += kRefineStepMps)
                refineAt.push_back(speed);
        }
        conditionTransitions[c] = transitions.str();
        std::sort(refineAt.begin(), refineAt.end());
        refineAt.erase(std::unique(refineAt.begin(), refineAt.end()), refineAt.end());
        for (double speed : refineAt) points.push_back(RunPoint(root, condition, speed, false, true, limits));
        conditionRefinedCount[c] = refineAt.size();
    }

    // ---- gates ----
    std::uint64_t failures = 0;
    auto require = [&failures](bool condition) { if (!condition) ++failures; };

    std::map<EPointClass, std::size_t> totals;
    std::uint64_t wow = 0, fdmFailures = 0, nonFinite = 0, external = 0;
    for (const FPoint &point : points) {
        ++totals[point.Classification];
        wow += point.Hold.WowCount; fdmFailures += point.Hold.RunFailures;
        nonFinite += point.Hold.NonFiniteCount; external += point.ExternalWriteAttempts;
        require(point.ExternalWriteAttempts == 0);
        if (SolverFeasible(point.Classification)) {
            require(point.TrimState.IsFinite());
            require(point.Hold.NonFiniteCount == 0 && point.Hold.WowCount == 0 && point.Hold.RunFailures == 0);
        }
        // Fuel must stay inside the declared tank capacities and must be reflected in the mass.
        if (point.bRunIcSuccess) {
            require(Finite(point.FuelTotalLb) && point.FuelTotalLb >= -1e-9);
            require(Finite(point.TankLb[0]) && point.TankLb[0] <= kInternalTankCapacityLb + 1e-9);
            require(Finite(point.TankLb[1]) && point.TankLb[1] <= kInternalTankCapacityLb + 1e-9);
            require(Finite(point.MassSlugs) && point.MassSlugs > 0.0);
        }
    }
    std::size_t classified = 0;
    for (EPointClass value : kAllClasses) classified += totals[value];
    require(classified == points.size());

    // Fuel must actually move the mass: at a fixed altitude, more fuel must mean more mass.
    {
        std::map<std::pair<int, int>, double> massByAltFuel;   // (altitude_ft, fuel_lb) -> mass
        for (const FPoint &point : points)
            if (point.bAnchor && Finite(point.MassSlugs))
                massByAltFuel[{static_cast<int>(std::llround(point.AltitudeFt)),
                               static_cast<int>(std::llround(point.RequestedFuelLb))}] = point.MassSlugs;
        for (int altitude : {5000, 10000, 20000, 30000, 40000}) {
            const auto low = massByAltFuel.find({altitude, 1000});
            const auto mid = massByAltFuel.find({altitude, 3000});
            const auto high = massByAltFuel.find({altitude, 6000});
            require(low != massByAltFuel.end() && mid != massByAltFuel.end() && high != massByAltFuel.end());
            if (low != massByAltFuel.end() && mid != massByAltFuel.end() && high != massByAltFuel.end())
                require(low->second < mid->second && mid->second < high->second);
        }
    }

    // ---- baseline reproduction gate (commit 524ad91, 10000 ft / 3000 lb) ----
    // The coarse scan is deliberately the SAME 5 m/s scan the committed map used, so the committed
    // point set is reproduced exactly and can be compared point for point.
    int baselineId = -1;
    for (const FCondition &condition : conditions) if (condition.IsBaseline()) baselineId = condition.Id;
    require(baselineId >= 0);
    std::size_t baselineStable = 0, baselineDrifting = 0, baselineFailed = 0, baselineOther = 0;
    double baselineLowestFeasible = std::numeric_limits<double>::infinity();
    double baselineHighestFeasible = -std::numeric_limits<double>::infinity();
    double baselineDriftLow = std::numeric_limits<double>::infinity();
    double baselineDriftHigh = -std::numeric_limits<double>::infinity();
    const FPoint *baselineFailedPoint = nullptr;
    const FPoint *baselineAnchorPoint = nullptr;
    std::pair<double, double> baselineContiguous{std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN()};
    std::pair<double, double> baselineContiguousRefined{std::numeric_limits<double>::quiet_NaN(),
                                                        std::numeric_limits<double>::quiet_NaN()};
    if (baselineId >= 0) {
        for (const FPoint &point : points) {
            if (point.ConditionId != baselineId || point.bRefined) continue;   // committed scan set only
            switch (point.Classification) {
            case EPointClass::TrimSuccessStable: ++baselineStable; break;
            case EPointClass::TrimSuccessDrifting: ++baselineDrifting; break;
            case EPointClass::TrimFailed: ++baselineFailed; baselineFailedPoint = &point; break;
            default: ++baselineOther; break;
            }
            if (point.bAnchor) baselineAnchorPoint = &point;
            if (SolverFeasible(point.Classification)) {
                baselineLowestFeasible = std::min(baselineLowestFeasible, point.RequestedEasMps);
                baselineHighestFeasible = std::max(baselineHighestFeasible, point.RequestedEasMps);
            }
            if (point.Classification == EPointClass::TrimSuccessDrifting) {
                baselineDriftLow = std::min(baselineDriftLow, point.RequestedEasMps);
                baselineDriftHigh = std::max(baselineDriftHigh, point.RequestedEasMps);
            }
        }
        // Committed-scan-set view: this is what the 5 m/s map in 524ad91 could observe.
        baselineContiguous = ContiguousAroundAnchor(SortedByEas(points, baselineId, /*coarseOnly=*/true));
        // Refined view: phase B adds 2.5 m/s samples the committed grid could not see. Reported as a
        // NEW measurement; it is not compared against the committed map.
        baselineContiguousRefined = ContiguousAroundAnchor(SortedByEas(points, baselineId));
        require(baselineStable == kBaselineStable);
        require(baselineDrifting == kBaselineDrifting);
        require(baselineFailed == kBaselineFailed);
        require(baselineOther == 0);
        require(baselineFailedPoint && std::abs(baselineFailedPoint->RequestedEasMps - kBaselineFailedEasMps) < 1e-9);
        require(baselineAnchorPoint && baselineAnchorPoint->bTrimSuccess &&
                baselineAnchorPoint->Classification == EPointClass::TrimSuccessStable);
        require(baselineAnchorPoint && Finite(baselineAnchorPoint->TargetMismatchMps) &&
                baselineAnchorPoint->TargetMismatchMps <= kTargetMismatchToleranceMps);
        require(baselineAnchorPoint &&
                std::abs(baselineAnchorPoint->TrimState.EasMps - kBaselineAnchorActualEasMps) <= 1.0e-6);
        require(baselineAnchorPoint && baselineAnchorPoint->Hold.WowCount == 0 &&
                baselineAnchorPoint->Hold.RunFailures == 0 && baselineAnchorPoint->Hold.NonFiniteCount == 0);
        require(std::abs(baselineLowestFeasible - kBaselineLowestFeasibleMps) < 1e-9);
        require(std::abs(baselineHighestFeasible - kBaselineHighestFeasibleMps) < 1e-9);
        require(std::abs(baselineContiguous.first - kBaselineLowestFeasibleMps) < 1e-9);
        require(std::abs(baselineContiguous.second - kBaselineHighestFeasibleMps) < 1e-9);
        require(std::abs(baselineDriftLow - kBaselineDriftingLowMps) < 1e-9);
        require(std::abs(baselineDriftHigh - kBaselineDriftingHighMps) < 1e-9);
    }

    // ---- output ----
    WriteMatrix(argv[2], points, 15);
    WriteMatrix(argv[3], points, 9);

    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "F16_TRIM_SENSITIVITY_MATRIX_V2\n";
    out << kInterpretation << '\n';
    out << "model=f16 data=Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim solver=FGTrim(tFull).DoTrim "
           "fdm_dt_s=" << kFdmDtS << " settle_s=" << kSettleS << " measure_s=" << kMeasureS
        << " latitude_deg=" << kInitialLatitudeDeg << " longitude_deg=" << kInitialLongitudeDeg
        << " heading_deg=" << kHeadingDeg << " wind_mps=0 gear=up flap=clean speedbrake=retracted fuel=frozen\n";
    out << "altitude_grid_ft=5000,10000,20000,30000,40000 fuel_grid_lb=1000,3000,6000 "
           "fuel_distribution=internal_tanks_0_and_1_split_evenly external_tanks=empty\n";
    out << "model_source_bounds aero_alpha_rad=" << kAeroAlphaMinRad << ',' << kAeroAlphaMaxRad
        << " engine_density_altitude_ft=" << kEngineDensityAltMinFt << ',' << kEngineDensityAltMaxFt
        << " engine_mach_max_aug=" << kEngineMachMax << " engine_mach_max_mil=" << kEngineMilMachMax
        << " internal_tank_capacity_lb=" << kInternalTankCapacityLb << " aero_mach_tables=none\n";
    out << "scan_lower_bound_mps=" << kScanLowerMps << " scan_upper_bound_mps=" << kScanUpperMps
        << " coarse_step_mps=" << kCoarseStepMps << " refine_step_mps=" << kRefineStepMps
        << " known_good_anchor_mps=" << kKnownGoodAnchorMps
        << " scan_bounds_are_characterization_only=1\n";
    out << "total_conditions=" << conditions.size() << " total_points=" << points.size() << '\n';

    for (std::size_t c = 0; c < conditions.size(); ++c) {
        const FCondition &condition = conditions[c];
        const std::vector<const FPoint *> sorted = SortedByEas(points, condition.Id);
        std::map<EPointClass, std::size_t> counts;
        for (const FPoint *point : sorted) ++counts[point->Classification];
        std::size_t conditionClassified = 0;
        for (EPointClass value : kAllClasses) conditionClassified += counts[value];
        const auto feasibleSpeeds = FeasibleRange(sorted, [](const FPoint &p) { return p.RequestedEasMps; });
        const auto throttleRange = FeasibleRange(sorted, [](const FPoint &p) { return p.TrimState.ThrottleCmd; });
        const auto alphaRange = FeasibleRange(sorted, [](const FPoint &p) { return p.TrimState.AlphaRad; });
        const auto stabilatorRange = FeasibleRange(sorted, [](const FPoint &p) { return p.TrimState.ElevatorPositionRad; });
        const auto machRange = FeasibleRange(sorted, [](const FPoint &p) { return p.TrimState.Mach; });
        const auto altitudeDrift = FeasibleRange(sorted, [](const FPoint &p) { return p.Hold.AltitudeDrift(); });
        const auto easDrift = FeasibleRange(sorted, [](const FPoint &p) { return p.Hold.EasDrift(); });
        const auto pitchDrift = FeasibleRange(sorted, [](const FPoint &p) { return p.Hold.PitchDrift(); });
        const auto climbDrift = FeasibleRange(sorted, [](const FPoint &p) { return p.Hold.MaxAbsClimb(); });
        const auto contiguousCoarse = ContiguousAroundAnchor(SortedByEas(points, condition.Id, /*coarseOnly=*/true));
        const auto contiguous = ContiguousAroundAnchor(sorted);
        const FPoint *anchor = nullptr;
        for (const FPoint *point : sorted) if (point->bAnchor) anchor = point;
        std::uint64_t conditionWow = 0, conditionFdm = 0, conditionNonFinite = 0, conditionMismatch = 0;
        for (const FPoint *point : sorted) {
            conditionWow += point->Hold.WowCount; conditionFdm += point->Hold.RunFailures;
            conditionNonFinite += point->Hold.NonFiniteCount;
            if (point->Classification == EPointClass::TargetMismatch) ++conditionMismatch;
        }

        out << "condition id=" << condition.Id
            << " altitude_ft=" << condition.AltitudeFt
            << " altitude_m=" << condition.AltitudeFt * kFtToM
            << " requested_fuel_lb=" << condition.RequestedFuelLb
            << " tanks_lb=" << condition.Tank0Lb << '|' << condition.Tank1Lb << '|'
            << condition.Tank2Lb << '|' << condition.Tank3Lb
            << " baseline=" << (condition.IsBaseline() ? 1 : 0) << '\n';
        out << "  fuel_total_lb=" << (anchor ? anchor->FuelTotalLb : std::numeric_limits<double>::quiet_NaN())
            << " mass_slug=" << (anchor ? anchor->MassSlugs : std::numeric_limits<double>::quiet_NaN())
            << " mass_kg=" << (anchor ? anchor->MassKg : std::numeric_limits<double>::quiet_NaN())
            << " weight_lb=" << (anchor ? anchor->WeightLb : std::numeric_limits<double>::quiet_NaN())
            << " cg_in=" << (anchor ? anchor->CgXIn : std::numeric_limits<double>::quiet_NaN()) << ','
            << (anchor ? anchor->CgYIn : std::numeric_limits<double>::quiet_NaN()) << ','
            << (anchor ? anchor->CgZIn : std::numeric_limits<double>::quiet_NaN())
            << " inertia_available=" << (anchor && anchor->bInertiaAvailable ? 1 : 0) << '\n';
        out << "  total_points=" << sorted.size() << " refined_points=" << conditionRefinedCount[c]
            << " classified=" << conditionClassified << " unclassified=" << (sorted.size() - conditionClassified)
            << " stability_basis=" << (conditionOwnBasis[c] ? "own_anchor_drift_x5" : "baseline_anchor_drift_x5_fallback") << '\n';
        out << "  classifications";
        for (EPointClass value : kAllClasses) out << ' ' << ClassName(value) << '=' << counts[value];
        out << '\n';
        out << "  anchor_classification=" << (anchor ? ClassName(anchor->Classification) : "missing")
            << " anchor_actual_eas_mps=" << (anchor ? anchor->TrimState.EasMps : std::numeric_limits<double>::quiet_NaN())
            << " anchor_trim_success=" << (anchor && anchor->bTrimSuccess ? 1 : 0) << '\n';
        out << "  lowest_solver_feasible_sample_mps=" << feasibleSpeeds.first
            << " highest_solver_feasible_sample_mps=" << feasibleSpeeds.second
            << " contiguous_anchor_region_coarse_mps=" << contiguousCoarse.first << ',' << contiguousCoarse.second
            << " contiguous_anchor_region_refined_mps=" << contiguous.first << ',' << contiguous.second << '\n';
        out << "  refined_transitions=" << (conditionTransitions[c].empty() ? "none" : conditionTransitions[c]) << '\n';
        out << "  throttle_range=" << throttleRange.first << ',' << throttleRange.second
            << " alpha_range_rad=" << alphaRange.first << ',' << alphaRange.second
            << " stabilator_range_rad=" << stabilatorRange.first << ',' << stabilatorRange.second
            << " mach_range=" << machRange.first << ',' << machRange.second << '\n';
        out << "  hold_altitude_drift_range_m=" << altitudeDrift.first << ',' << altitudeDrift.second
            << " hold_eas_drift_range_mps=" << easDrift.first << ',' << easDrift.second
            << " hold_pitch_drift_range_rad=" << pitchDrift.first << ',' << pitchDrift.second
            << " hold_max_abs_climb_range_mps=" << climbDrift.first << ',' << climbDrift.second << '\n';
        out << "  unexpected_wow=" << conditionWow << " fdm_run_failures=" << conditionFdm
            << " non_finite_count=" << conditionNonFinite << " target_mismatch=" << conditionMismatch << '\n';
    }

    std::vector<const FPoint *> all;
    for (const FPoint &point : points) all.push_back(&point);
    const auto massRange = FeasibleRange(all, [](const FPoint &p) { return p.MassSlugs; });
    const auto cgRange = FeasibleRange(all, [](const FPoint &p) { return p.CgXIn; });
    const auto throttleAll = FeasibleRange(all, [](const FPoint &p) { return p.TrimState.ThrottleCmd; });
    const auto alphaAll = FeasibleRange(all, [](const FPoint &p) { return p.TrimState.AlphaRad; });
    const auto stabilatorAll = FeasibleRange(all, [](const FPoint &p) { return p.TrimState.ElevatorPositionRad; });
    const auto altitudeDriftAll = FeasibleRange(all, [](const FPoint &p) { return p.Hold.AltitudeDrift(); });
    const auto easDriftAll = FeasibleRange(all, [](const FPoint &p) { return p.Hold.EasDrift(); });
    const auto climbAll = FeasibleRange(all, [](const FPoint &p) { return p.Hold.MaxAbsClimb(); });

    out << "global classifications";
    for (EPointClass value : kAllClasses) out << ' ' << ClassName(value) << '=' << totals[value];
    out << '\n';
    out << "global classified=" << classified << " unclassified=" << (points.size() - classified)
        << " failures=" << failures << '\n';
    out << "global mass_slug_range=" << massRange.first << ',' << massRange.second
        << " cg_x_in_range=" << cgRange.first << ',' << cgRange.second << '\n';
    out << "global throttle_range=" << throttleAll.first << ',' << throttleAll.second
        << " alpha_range_rad=" << alphaAll.first << ',' << alphaAll.second
        << " stabilator_range_rad=" << stabilatorAll.first << ',' << stabilatorAll.second << '\n';
    out << "global hold_altitude_drift_range_m=" << altitudeDriftAll.first << ',' << altitudeDriftAll.second
        << " hold_eas_drift_range_mps=" << easDriftAll.first << ',' << easDriftAll.second
        << " hold_max_abs_climb_range_mps=" << climbAll.first << ',' << climbAll.second << '\n';
    out << "global unexpected_wow=" << wow << " fdm_run_failures=" << fdmFailures
        << " non_finite_count=" << nonFinite << " external_write_attempts=" << external << '\n';
    out << "baseline_reproduction condition_id=" << baselineId
        << " stable=" << baselineStable << " drifting=" << baselineDrifting << " failed=" << baselineFailed
        << " other=" << baselineOther
        << " failed_at_mps=" << (baselineFailedPoint ? baselineFailedPoint->RequestedEasMps : std::numeric_limits<double>::quiet_NaN())
        << " lowest_feasible_mps=" << baselineLowestFeasible << " highest_feasible_mps=" << baselineHighestFeasible
        << " contiguous_coarse_mps=" << baselineContiguous.first << ',' << baselineContiguous.second
        << " contiguous_refined_mps=" << baselineContiguousRefined.first << ',' << baselineContiguousRefined.second
        << " drifting_span_mps=" << baselineDriftLow << ',' << baselineDriftHigh
        << " anchor_actual_eas_mps="
        << (baselineAnchorPoint ? baselineAnchorPoint->TrimState.EasMps : std::numeric_limits<double>::quiet_NaN())
        << " reference_commit=524ad91\n";
    out << "production_writer_invocations=0 ue_world_loaded=0 game_pawns_searched=0 active_connection=0 "
           "aircraft_xml_modified=0 tecs_parameters_modified=0\n";

    const std::string summary = out.str();
    std::ofstream summaryFile(argv[4]); summaryFile << summary;
    std::fputs(summary.c_str(), stdout);
    std::printf("F16_TRIM_SENSITIVITY_MATRIX_V2_RESULT=%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
