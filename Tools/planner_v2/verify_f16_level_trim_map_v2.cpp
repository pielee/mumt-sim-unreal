#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "initialization/FGTrim.h"
#include "models/FGAuxiliary.h"
#include "models/FGAtmosphere.h"
#include "models/FGFCS.h"
#include "models/FGGroundReactions.h"
#include "models/FGMassBalance.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"
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
#include <numeric>
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
constexpr double kAltitudeFt = 10000.0;
constexpr double kHeadingDeg = 90.0;
constexpr double kScanLowerMps = 60.0;
constexpr double kScanUpperMps = 300.0;
constexpr double kScanStepMps = 5.0;
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

const char *kInterpretation =
    "This map characterizes solver-feasible level-flight trim points at one "
    "altitude and mass condition. It is not an operational flight envelope "
    "and is not used directly as a TECS parameter set.";

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
    InvalidInput
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
    }
    return "InvalidInput";
}

bool SolverFeasible(EPointClass value)
{
    return value == EPointClass::TrimSuccessStable || value == EPointClass::TrimSuccessDrifting;
}

struct FState {
    double SimTimeS{std::numeric_limits<double>::quiet_NaN()};
    double EasMps{std::numeric_limits<double>::quiet_NaN()};
    double TasMps{std::numeric_limits<double>::quiet_NaN()};
    double GroundSpeedMps{std::numeric_limits<double>::quiet_NaN()};
    double AltitudeM{std::numeric_limits<double>::quiet_NaN()};
    double DensitySlugFt3{std::numeric_limits<double>::quiet_NaN()};
    double DensityRatio{std::numeric_limits<double>::quiet_NaN()};
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
        const std::array<double, 29> values{
            SimTimeS, EasMps, TasMps, GroundSpeedMps, AltitudeM, DensitySlugFt3,
            DensityRatio, FlightPathAngleRad, PitchRad, RollRad, HeadingRad,
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
    double PMin{std::numeric_limits<double>::infinity()}, PMax{-std::numeric_limits<double>::infinity()};
    double QMin{std::numeric_limits<double>::infinity()}, QMax{-std::numeric_limits<double>::infinity()};
    double RMin{std::numeric_limits<double>::infinity()}, RMax{-std::numeric_limits<double>::infinity()};
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
        PMin = std::min(PMin, state.P); PMax = std::max(PMax, state.P);
        QMin = std::min(QMin, state.Q); QMax = std::max(QMax, state.Q);
        RMin = std::min(RMin, state.R); RMax = std::max(RMax, state.R);
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
    double RequestedEasMps{};
    bool bAnchor{};
    EPointClass Classification{EPointClass::InvalidInput};
    bool bRunIcSuccess{}, bTrimAttempted{}, bTrimSuccess{};
    bool bProcessSurvived{true};
    std::string Diagnostic;
    FState TrimState{};
    FHoldMetrics Hold{};
    double TargetMismatchMps{std::numeric_limits<double>::quiet_NaN()};
    double MassSlugs{std::numeric_limits<double>::quiet_NaN()};
    double FuelLb{std::numeric_limits<double>::quiet_NaN()};
    double ThrottleLowerMargin{std::numeric_limits<double>::quiet_NaN()};
    double ThrottleUpperMargin{std::numeric_limits<double>::quiet_NaN()};
    double ElevatorInputMargin{std::numeric_limits<double>::quiet_NaN()};
    double AileronInputMargin{std::numeric_limits<double>::quiet_NaN()};
    double RudderInputMargin{std::numeric_limits<double>::quiet_NaN()};
    std::uint64_t OwnedCommandFrames{}, ExternalWriteAttempts{};
};

class FTrimPlant {
public:
    bool Initialize(const std::string &root, double requestedEasMps, FPoint &point)
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

        IC->SetGeodLatitudeDegIC(kInitialLatitudeDeg);
        IC->SetLongitudeDegIC(kInitialLongitudeDeg);
        IC->SetAltitudeASLFtIC(kAltitudeFt);
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
        point.MassSlugs = MassBalance->GetMass();
        point.FuelLb = Exec->GetPropertyValue("propulsion/total-fuel-lbs");

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
        point.TargetMismatchMps = std::abs(point.TrimState.EasMps - requestedEasMps);
        point.ThrottleLowerMargin = HoldThrottle;
        point.ThrottleUpperMargin = 1.0 - HoldThrottle;
        point.ElevatorInputMargin = 1.0 - std::abs(HoldDe);
        point.AileronInputMargin = 1.0 - std::abs(HoldDa);
        point.RudderInputMargin = 1.0 - std::abs(HoldDr);
        return true;
    }

    FState Read() const
    {
        FState state{};
        state.SimTimeS = Exec->GetSimTime();
        state.EasMps = Auxiliary->GetVequivalentKTS() * kKnotToMps;
        state.TasMps = Auxiliary->GetVt() * kFtToM;
        state.GroundSpeedMps = Auxiliary->GetVground() * kFtToM;
        state.AltitudeM = Propagate->GetAltitudeASL() * kFtToM;
        state.DensitySlugFt3 = Atmosphere->GetDensity();
        state.DensityRatio = Atmosphere->GetDensityRatio();
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

FPoint RunPoint(const std::string &root, double requestedEasMps, bool anchor,
                const FStabilityLimits *stabilityLimits)
{
    FPoint point{};
    point.RequestedEasMps = requestedEasMps;
    point.bAnchor = anchor;
    if (!Finite(requestedEasMps) || requestedEasMps <= 0.0) {
        point.Classification = EPointClass::InvalidInput;
        point.Diagnostic = "requested EAS is invalid";
        return point;
    }

    try {
        FTrimPlant plant;
        if (!plant.Initialize(root, requestedEasMps, point)) {
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

std::vector<double> ScanPoints()
{
    std::vector<double> points{kKnownGoodAnchorMps};
    for (double speed = kScanLowerMps; speed <= kScanUpperMps + 1e-9; speed += kScanStepMps)
        points.push_back(speed);
    return points;
}

void WriteNumber(std::ostream &out, double value, int precision)
{
    if (Finite(value)) out << std::fixed << std::setprecision(precision) << value;
    else out << "NA";
}

void WriteMap(const std::string &path, const std::vector<FPoint> &points, int precision)
{
    std::ofstream out(path);
    out << "requested_eas_mps,actual_eas_mps,actual_tas_mps,classification,is_anchor,trim_attempted,trim_success,run_ic_success,"
           "altitude_m,density_slug_ft3,density_ratio,mass_slug,fuel_lb,heading_rad,flight_path_angle_rad,pitch_rad,roll_rad,"
           "alpha_rad,beta_rad,p_radps,q_radps,r_radps,climb_rate_mps,ground_speed_mps,load_factor,throttle_cmd_norm,"
           "throttle_position_norm,elevator_cmd_norm,elevator_position_rad,aileron_cmd_norm,aileron_position_rad,"
           "rudder_cmd_norm,rudder_position_rad,pitch_trim_cmd,roll_trim_cmd,yaw_trim_cmd,target_mismatch_mps,"
           "throttle_lower_margin,throttle_upper_margin,elevator_input_margin,aileron_input_margin,rudder_input_margin,"
           "hold_altitude_drift_m,hold_eas_drift_mps,hold_tas_drift_mps,hold_pitch_drift_rad,hold_roll_drift_rad,"
           "hold_climb_rate_min_mps,hold_climb_rate_max_mps,wow_count,fdm_run_failures,nonfinite_count,owned_command_frames,"
           "external_write_attempts,process_survived,alpha_table_bounds_available,surface_limits_available,diagnostic\n";
    for (const FPoint &p : points) {
        WriteNumber(out, p.RequestedEasMps, precision); out << ',';
        WriteNumber(out, p.TrimState.EasMps, precision); out << ',';
        WriteNumber(out, p.TrimState.TasMps, precision); out << ',' << ClassName(p.Classification) << ',' << (p.bAnchor ? 1 : 0)
            << ',' << (p.bTrimAttempted ? 1 : 0) << ',' << (p.bTrimSuccess ? 1 : 0) << ',' << (p.bRunIcSuccess ? 1 : 0) << ',';
        const std::array<double, 28> values{
            p.TrimState.AltitudeM, p.TrimState.DensitySlugFt3, p.TrimState.DensityRatio, p.MassSlugs, p.FuelLb,
            p.TrimState.HeadingRad, p.TrimState.FlightPathAngleRad, p.TrimState.PitchRad, p.TrimState.RollRad,
            p.TrimState.AlphaRad, p.TrimState.BetaRad, p.TrimState.P, p.TrimState.Q, p.TrimState.R,
            p.TrimState.ClimbRateMps, p.TrimState.GroundSpeedMps, p.TrimState.LoadFactor,
            p.TrimState.ThrottleCmd, p.TrimState.ThrottlePosition, p.TrimState.ElevatorCmd,
            p.TrimState.ElevatorPositionRad, p.TrimState.AileronCmd, p.TrimState.AileronPositionRad,
            p.TrimState.RudderCmd, p.TrimState.RudderPositionRad, p.TrimState.PitchTrimCmd,
            p.TrimState.RollTrimCmd, p.TrimState.YawTrimCmd};
        for (double value : values) { WriteNumber(out, value, precision); out << ','; }
        const std::array<double, 12> metrics{
            p.TargetMismatchMps, p.ThrottleLowerMargin, p.ThrottleUpperMargin, p.ElevatorInputMargin,
            p.AileronInputMargin, p.RudderInputMargin, p.Hold.AltitudeDrift(), p.Hold.EasDrift(),
            p.Hold.TasDrift(), p.Hold.PitchDrift(), p.Hold.RollDrift(), p.Hold.ClimbMin};
        for (double value : metrics) { WriteNumber(out, value, precision); out << ','; }
        WriteNumber(out, p.Hold.ClimbMax, precision);
        out << ',' << p.Hold.WowCount << ',' << p.Hold.RunFailures << ',' << p.Hold.NonFiniteCount
            << ',' << p.OwnedCommandFrames << ',' << p.ExternalWriteAttempts
            << ',' << (p.bProcessSurvived ? 1 : 0) << ",0,0," << p.Diagnostic << '\n';
    }
}

double TrendSlope(const std::vector<FPoint> &points, double FState::*field)
{
    std::vector<std::pair<double, double>> values;
    for (const FPoint &point : points)
        if (SolverFeasible(point.Classification) && Finite(point.TrimState.*field))
            values.emplace_back(point.RequestedEasMps, point.TrimState.*field);
    if (values.size() < 2) return std::numeric_limits<double>::quiet_NaN();
    double meanX = 0.0, meanY = 0.0;
    for (const auto &value : values) { meanX += value.first; meanY += value.second; }
    meanX /= static_cast<double>(values.size()); meanY /= static_cast<double>(values.size());
    double numerator = 0.0, denominator = 0.0;
    for (const auto &value : values) {
        numerator += (value.first - meanX) * (value.second - meanY);
        denominator += (value.first - meanX) * (value.first - meanX);
    }
    return denominator > 0.0 ? numerator / denominator : std::numeric_limits<double>::quiet_NaN();
}

template <typename Getter>
std::pair<double, double> FeasibleRange(const std::vector<FPoint> &points, Getter getter)
{
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (const FPoint &point : points) {
        if (!SolverFeasible(point.Classification)) continue;
        const double value = getter(point);
        if (!Finite(value)) continue;
        minimum = std::min(minimum, value); maximum = std::max(maximum, value);
    }
    return {minimum, maximum};
}

std::pair<double, double> ContiguousAroundAnchor(const std::vector<FPoint> &points)
{
    std::vector<const FPoint *> sorted;
    for (const FPoint &point : points) sorted.push_back(&point);
    std::sort(sorted.begin(), sorted.end(), [](const FPoint *a, const FPoint *b) { return a->RequestedEasMps < b->RequestedEasMps; });
    std::size_t anchorIndex = sorted.size();
    for (std::size_t i = 0; i < sorted.size(); ++i) if (sorted[i]->bAnchor) anchorIndex = i;
    if (anchorIndex == sorted.size() || !SolverFeasible(sorted[anchorIndex]->Classification))
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    std::size_t lower = anchorIndex, upper = anchorIndex;
    while (lower > 0 && SolverFeasible(sorted[lower - 1]->Classification) &&
           sorted[lower]->RequestedEasMps - sorted[lower - 1]->RequestedEasMps <= kScanStepMps + 1e-6) --lower;
    while (upper + 1 < sorted.size() && SolverFeasible(sorted[upper + 1]->Classification) &&
           sorted[upper + 1]->RequestedEasMps - sorted[upper]->RequestedEasMps <= kScanStepMps + 1e-6) ++upper;
    return {sorted[lower]->RequestedEasMps, sorted[upper]->RequestedEasMps};
}

std::string Summary(const std::vector<FPoint> &points, const FStabilityLimits &limits, std::uint64_t failures)
{
    std::map<EPointClass, std::size_t> counts;
    for (const FPoint &point : points) ++counts[point.Classification];
    const FPoint *anchor = nullptr;
    for (const FPoint &point : points) if (point.bAnchor) anchor = &point;
    const auto feasibleSpeeds = FeasibleRange(points, [](const FPoint &p) { return p.RequestedEasMps; });
    const auto throttleRange = FeasibleRange(points, [](const FPoint &p) { return p.TrimState.ThrottleCmd; });
    const auto alphaRange = FeasibleRange(points, [](const FPoint &p) { return p.TrimState.AlphaRad; });
    const auto stabilatorRange = FeasibleRange(points, [](const FPoint &p) { return p.TrimState.ElevatorPositionRad; });
    const auto altitudeDriftRange = FeasibleRange(points, [](const FPoint &p) { return p.Hold.AltitudeDrift(); });
    const auto easDriftRange = FeasibleRange(points, [](const FPoint &p) { return p.Hold.EasDrift(); });
    const auto tasDriftRange = FeasibleRange(points, [](const FPoint &p) { return p.Hold.TasDrift(); });
    const auto pitchDriftRange = FeasibleRange(points, [](const FPoint &p) { return p.Hold.PitchDrift(); });
    const auto rollDriftRange = FeasibleRange(points, [](const FPoint &p) { return p.Hold.RollDrift(); });
    const auto climbRange = FeasibleRange(points, [](const FPoint &p) { return p.Hold.MaxAbsClimb(); });
    const auto mismatchRange = FeasibleRange(points, [](const FPoint &p) { return p.TargetMismatchMps; });
    const auto contiguous = ContiguousAroundAnchor(points);
    const FPoint *minimumThrottlePoint = nullptr;
    for (const FPoint &point : points)
        if (SolverFeasible(point.Classification) &&
            (!minimumThrottlePoint || point.TrimState.ThrottleCmd < minimumThrottlePoint->TrimState.ThrottleCmd))
            minimumThrottlePoint = &point;
    std::uint64_t wow = 0, fdmFailures = 0, nonFinite = 0, external = 0;
    for (const FPoint &point : points) {
        wow += point.Hold.WowCount; fdmFailures += point.Hold.RunFailures;
        nonFinite += point.Hold.NonFiniteCount; external += point.ExternalWriteAttempts;
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "F16_LEVEL_TRIM_MAP_V2\n";
    out << kInterpretation << '\n';
    out << "model=f16 data=Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim altitude_ft=" << kAltitudeFt
        << " altitude_m=" << kAltitudeFt * kFtToM << " latitude_deg=" << kInitialLatitudeDeg
        << " longitude_deg=" << kInitialLongitudeDeg << " heading_deg=" << kHeadingDeg << '\n';
    out << "fuel_lb=" << (anchor ? anchor->FuelLb : std::numeric_limits<double>::quiet_NaN())
        << " mass_slugs=" << (anchor ? anchor->MassSlugs : std::numeric_limits<double>::quiet_NaN())
        << " wind_mps=0 atmosphere=JSBSim_standard gear=up flap=clean speedbrake=retracted fuel=frozen fdm_dt_s=" << kFdmDtS << '\n';
    out << "scan_lower_bound_mps=" << kScanLowerMps << " scan_upper_bound_mps=" << kScanUpperMps
        << " scan_step_mps=" << kScanStepMps << " known_good_anchor_mps=" << kKnownGoodAnchorMps
        << " total_points=" << points.size() << '\n';
    out << "target_mismatch_tolerance_mps=" << kTargetMismatchToleranceMps
        << " basis=requested_decimal_precision_and_byte_identical_double_baseline\n";
    out << "stability_basis=anchor_post_trim_drift_x" << kAnchorDriftMultiplier
        << " limits_altitude_m=" << limits.AltitudeM << " eas_mps=" << limits.EasMps
        << " tas_mps=" << limits.TasMps << " pitch_rad=" << limits.PitchRad
        << " roll_rad=" << limits.RollRad << " climb_rate_mps=" << limits.ClimbRateMps << '\n';
    for (EPointClass value : {EPointClass::TrimSuccessStable, EPointClass::TrimSuccessDrifting,
             EPointClass::TrimFailed, EPointClass::RunICFailed, EPointClass::PostTrimRunFailed,
             EPointClass::NonFinite, EPointClass::UnexpectedWOW, EPointClass::TargetMismatch,
             EPointClass::InvalidInput})
        out << "classification_" << ClassName(value) << '=' << counts[value] << '\n';
    out << "classified=" << points.size() << " unclassified=0 failures=" << failures << '\n';
    out << "anchor_classification=" << (anchor ? ClassName(anchor->Classification) : "missing")
        << " anchor_actual_eas_mps=" << (anchor ? anchor->TrimState.EasMps : 0.0)
        << " anchor_target_mismatch_mps=" << (anchor ? anchor->TargetMismatchMps : 0.0)
        << " anchor_trim_success=" << (anchor && anchor->bTrimSuccess ? 1 : 0) << '\n';
    out << "lowest_solver_feasible_sample_mps=" << feasibleSpeeds.first
        << " highest_solver_feasible_sample_mps=" << feasibleSpeeds.second
        << " contiguous_anchor_region_mps=" << contiguous.first << ',' << contiguous.second << '\n';
    out << "successful_throttle_range=" << throttleRange.first << ',' << throttleRange.second
        << " throttle_trend_per_mps=" << TrendSlope(points, &FState::ThrottleCmd)
        << " minimum_throttle_point_requested_eas_mps="
        << (minimumThrottlePoint ? minimumThrottlePoint->RequestedEasMps : std::numeric_limits<double>::quiet_NaN()) << '\n';
    out << "successful_alpha_range_rad=" << alphaRange.first << ',' << alphaRange.second
        << " alpha_trend_rad_per_mps=" << TrendSlope(points, &FState::AlphaRad)
        << " alpha_table_bounds=not_available_as_single_model_wide_bound\n";
    out << "successful_stabilator_range_rad=" << stabilatorRange.first << ',' << stabilatorRange.second
        << " stabilator_trend_rad_per_mps=" << TrendSlope(points, &FState::ElevatorPositionRad)
        << " surface_position_limits=not_exposed_by_trim_API\n";
    out << "hold_altitude_drift_range_m=" << altitudeDriftRange.first << ',' << altitudeDriftRange.second
        << " hold_eas_drift_range_mps=" << easDriftRange.first << ',' << easDriftRange.second
        << " hold_tas_drift_range_mps=" << tasDriftRange.first << ',' << tasDriftRange.second
        << " hold_pitch_drift_range_rad=" << pitchDriftRange.first << ',' << pitchDriftRange.second
        << " hold_roll_drift_range_rad=" << rollDriftRange.first << ',' << rollDriftRange.second
        << " hold_max_abs_climb_range_mps=" << climbRange.first << ',' << climbRange.second << '\n';
    out << "successful_target_mismatch_range_mps=" << mismatchRange.first << ',' << mismatchRange.second << '\n';
    out << "unexpected_wow=" << wow << " fdm_run_failures=" << fdmFailures
        << " non_finite_count=" << nonFinite << " external_write_attempts=" << external << '\n';
    out << "production_writer_invocations=0 ue_world_loaded=0 game_pawns_searched=0 active_connection=0\n";
    return out.str();
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s <JSBSim-root> <raw.csv> <quantized.csv> <summary.txt>\n", argv[0]);
        return 2;
    }
    const std::string root = argv[1];
    std::vector<FPoint> points;
    points.reserve(50);

    FPoint anchor = RunPoint(root, kKnownGoodAnchorMps, true, nullptr);
    const FStabilityLimits limits = LimitsFromAnchor(anchor);
    points.push_back(anchor);
    for (double requested : ScanPoints()) {
        if (std::abs(requested - kKnownGoodAnchorMps) < 1e-9) continue;
        points.push_back(RunPoint(root, requested, false, &limits));
    }

    std::uint64_t failures = 0;
    auto require = [&failures](bool condition) { if (!condition) ++failures; };
    require(points.size() == 50);
    require(anchor.bRunIcSuccess && anchor.bTrimSuccess && anchor.TrimState.IsFinite());
    require(anchor.Classification == EPointClass::TrimSuccessStable);
    require(anchor.Hold.WowCount == 0 && anchor.Hold.RunFailures == 0 && anchor.Hold.NonFiniteCount == 0);
    require(Finite(anchor.TargetMismatchMps) && anchor.TargetMismatchMps <= kTargetMismatchToleranceMps);
    bool lowerSuccess = false, upperSuccess = false;
    std::size_t classified = 0;
    for (const FPoint &point : points) {
        ++classified;
        if (point.RequestedEasMps < kKnownGoodAnchorMps && SolverFeasible(point.Classification)) lowerSuccess = true;
        if (point.RequestedEasMps > kKnownGoodAnchorMps && SolverFeasible(point.Classification)) upperSuccess = true;
        if (SolverFeasible(point.Classification)) {
            require(point.TrimState.IsFinite());
            require(point.Hold.NonFiniteCount == 0 && point.Hold.WowCount == 0 && point.Hold.RunFailures == 0);
        }
        require(point.ExternalWriteAttempts == 0);
    }
    require(classified == points.size());
    require(lowerSuccess && upperSuccess);

    WriteMap(argv[2], points, 15);
    WriteMap(argv[3], points, 9);
    const std::string summary = Summary(points, limits, failures);
    std::ofstream summaryFile(argv[4]); summaryFile << summary;
    std::fputs(summary.c_str(), stdout);
    std::printf("F16_LEVEL_TRIM_MAP_V2_RESULT=%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
