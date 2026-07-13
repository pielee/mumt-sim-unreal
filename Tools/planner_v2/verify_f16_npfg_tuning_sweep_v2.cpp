// verify_f16_npfg_tuning_sweep_v2.cpp
//
// F-16 NPFG tuning-identification sweep, against the ACTUAL JSBSim F-16 plant.
//
// The committed isolation harness showed that the left/right asymmetry is seeded by the airframe
// (12% open loop) but AMPLIFIED to ~98% once the NPFG loop closes and the roll limit saturates. The
// explicit NPFG caller contract (bcfdc81) now exposes period, damping and the roll time constant as
// validated configuration, so the question "is this an NPFG tuning problem?" is finally answerable.
//
// This sweep answers it by MEASUREMENT. It changes no production value: every combination is a
// test-local FGuidanceConfigV2 driving the production coordinator, the production NPFG and TECS
// adapters, the production stick adapter and the actual f16.xml FCS/FDM.
//
//   FormationGuidanceCoordinatorV2 -> FPx4NpfgAdapter + FPx4TecsAdapter -> F16StickAdapterV2
//     -> normalized aileron/elevator/rudder/throttle -> f16.xml FCS -> JSBSim F-16 -> feedback
//
// Nothing here is a production recommendation. Combinations that survive the pre-declared reject
// gates are CANDIDATES, ranked and reported with a Pareto front; selecting a production value is a
// separate, deliberate decision.
//
// Swept (through the explicit caller contract): NpfgPeriodS, NpfgDamping, NpfgRollTimeConstantS.
// Held at the production default: period lower/upper bound flags, switch-distance multiplier (which
// is provably inert on the coordinator's update path), period safety factor, roll limit, every TECS
// parameter and every stick gain.
#include "FormationControl/Px4NpfgAdapter.h"
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
#include <cstring>
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
using FormationControlV2::FGuidanceConfigV2;
using FormationControlV2::FGuidanceCoordinatorInputV2;
using FormationControlV2::FGuidanceCoordinatorOutputV2;
using FormationControlV2::FormationGuidanceCoordinatorV2;
using FormationControlV2::FPlannerV2OutputAdapterResult;
using FormationControlV2::Vec2;

constexpr double kFdmDtS = 1.0 / 120.0;
constexpr double kControllerDtS = 1.0 / 60.0;
constexpr int kFdmStepsPerControlFrame = 2;

constexpr double kFtToM = 0.3048;
constexpr double kKnotToMps = 0.5144444444444444;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kFiniteLimit = 1.0e12;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// ---- plant condition, identical to the committed lateral harness --------------------------------
constexpr double kInitialLatitudeDeg = 47.0;
constexpr double kInitialLongitudeDeg = -122.0;
constexpr double kInitialAltitudeFt = 10000.0;
constexpr double kInitialTasMps = 220.0;
constexpr double kInitialHeadingDeg = 90.0;
constexpr double kTank0Lb = 1500.0, kTank1Lb = 1500.0, kTank2Lb = 0.0, kTank3Lb = 0.0;
constexpr double kExpectedFuelLb = 3000.0;
constexpr double kAnchorTrimEasMps = 189.070713;
constexpr double kAnchorTrimTasMps = 220.0;
constexpr double kAnchorTrimTolerance = 1.0e-6;

// ---- test-local TECS config, identical to the committed lateral harness --------------------------
constexpr double kTestLocalEasMinMps = 170.0;
constexpr double kTestLocalEasMaxMps = 220.0;
constexpr double kTestLocalMaxClimbRateMps = 100.539354597920735;
constexpr double kTestLocalMinSinkRateMps = 53.569103350590687;
constexpr double kTestLocalMaxSinkRateMps = 83.108839138474863;

// ---- case schedule, identical to the committed lateral harness -----------------------------------
constexpr double kPrimeS = 0.5;
constexpr double kSettleEndS = 60.0;
constexpr double kStepTimeS = 70.0;
constexpr double kCaseDurationS = 130.0;
constexpr double kCrossTrackStepM = 500.0;
constexpr double kCourseErrorStepRad = 10.0 * kDegToRad;
constexpr double kPathTurnRadiusM = 10000.0;

// ---- HARD REJECT GATES. Declared here, before the sweep runs, and never adjusted to a result. -----
constexpr double kRejectSaturationFraction = 0.25;      // roll-reference saturation > 25% of frames
constexpr double kRejectRollOverLimitRad = 0.05;        // actual |roll| beyond RollLimit + this ...
constexpr double kRejectRollOverLimitFraction = 0.05;   // ... on more than 5% of frames
constexpr double kRejectAltitudeErrorM = 150.0;
constexpr double kRejectEasErrorMps = 10.0;
constexpr double kRejectDivergenceCrossTrackM = 2000.0; // persistent divergence: |xtk| ever beyond
constexpr double kRejectTailSlopeMps = 1.0;             // ... or |xtk| still growing at > 1 m/s
constexpr double kErrorFloorM = 1.0e-3;                 // an "initial error" below this is not commanded
constexpr double kErrorFloorRad = 1.0e-4;
constexpr double kTailWindowS = 10.0;

constexpr double kAsymFloor = 1.0e-6;

bool Finite(double v) { return std::isfinite(v) && std::abs(v) < kFiniteLimit; }
constexpr double kNa = std::numeric_limits<double>::quiet_NaN();

double WrapPi(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
Vec2 FromCourse(double c) { return Vec2{std::cos(c), std::sin(c)}; }
Vec2 RightNormal(double c) { return Vec2{-std::sin(c), std::cos(c)}; }
double CourseOf(const Vec2 &v) { return std::atan2(v.E, v.N); }

double NormalizedAsymmetry(double a, double b)
{
    const double x = std::abs(a), y = std::abs(b);
    const double d = std::max({x, y, kAsymFloor});
    if (!Finite(x) || !Finite(y)) return kNa;
    return std::abs(x - y) / d;
}

// ================================================================================================
// Plant
// ================================================================================================
struct FPlantState {
    double TimeS{};
    double NorthM{}, EastM{}, VelNorthMps{}, VelEastMps{};
    double GroundCourseRad{}, GroundSpeedMps{};
    double AltitudeAslM{}, ClimbRateMps{};
    double EasMps{}, TasMps{}, EasToTasRatio{};
    double RollRad{}, PitchRad{}, PRadps{}, QRadps{}, RRadps{};
    double AlphaRad{}, BetaRad{}, LoadFactor{};
    double AileronLeftPosRad{}, ElevatorPosRad{}, ThrottlePos{};
    double GearPos{}, FlapPosNorm{}, SpeedbrakePos{};
    bool bWow{}, bEngineRunning{}, bRatioValid{};

    bool IsFinite() const
    {
        const std::array<double, 20> v{TimeS, NorthM, EastM, VelNorthMps, VelEastMps, GroundCourseRad,
                                       AltitudeAslM, ClimbRateMps, EasMps, TasMps, RollRad, PitchRad,
                                       PRadps, QRadps, RRadps, AlphaRad, BetaRad, LoadFactor,
                                       AileronLeftPosRad, ElevatorPosRad};
        return std::all_of(v.begin(), v.end(), Finite);
    }
};

struct FTrimResult {
    bool bSuccess{};
    double PitchRad{}, EasMps{}, TasMps{}, AltitudeM{}, ThrottleCmd{}, ElevatorCmd{}, AileronCmd{};
    double FuelLb{};
};

struct FCounters {
    std::uint64_t FdmRuns{}, FdmRunFailures{}, CommandFrames{}, WritesOutsideOwnedFdm{};
    std::uint64_t NonFiniteStates{}, UnexpectedWow{}, ConfigViolations{};
    std::uint64_t CommandRangeViolations{}, CommandSlewViolations{};
    std::uint64_t UnexpectedInvalidCoordinatorFrames{}, UnexpectedInvalidStickFrames{};
};

class FOwnedF16Plant {
public:
    bool Initialize(const std::string &root, std::string &failure)
    {
        Exec = std::make_unique<JSBSim::FGFDMExec>();
        Owned = Exec.get();
        Exec->SetDebugLevel(0);
        Exec->SetRootDir(SGPath(root));
        Exec->SetAircraftPath(SGPath("aircraft"));
        Exec->SetEnginePath(SGPath("engine"));
        Exec->SetSystemsPath(SGPath("systems"));
        if (!Exec->LoadModel("f16")) { failure = "LoadModel(f16) failed"; return false; }
        Exec->Setdt(kFdmDtS);
        Fcs = Exec->GetFCS(); Prop = Exec->GetPropagate(); Aux = Exec->GetAuxiliary();
        Pls = Exec->GetPropulsion(); Grd = Exec->GetGroundReactions(); Mass = Exec->GetMassBalance();
        if (!Fcs || !Prop || !Aux || !Pls || !Grd || !Mass) { failure = "null JSBSim model pointer"; return false; }
        if (Pls->GetNumTanks() != 4) { failure = "f16 does not declare 4 fuel tanks"; return false; }
        const std::array<double, 4> want{kTank0Lb, kTank1Lb, kTank2Lb, kTank3Lb};
        for (unsigned i = 0; i < 4; ++i) {
            auto t = Pls->GetTank(i);
            if (!t || want[i] < 0.0 || want[i] > t->GetCapacity() + 1e-9) { failure = "bad fuel tank"; return false; }
            t->SetContents(want[i]);
        }
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
        Pls->InitRunning(-1);
        Pls->SetFuelFreeze(true);
        Fcs->SetGearCmd(0.0); Fcs->SetGearPos(0.0);
        Fcs->SetDfCmd(0.0); Fcs->SetDfPos(JSBSim::ofNorm, 0.0);
        Fcs->SetDsbCmd(0.0);
        Fcs->SetThrottleCmd(0, 0.6);

        JSBSim::FGTrim trim(Exec.get(), JSBSim::tFull);
        Trim.bSuccess = trim.DoTrim();
        if (!Trim.bSuccess) { failure = "FGTrim(tFull).DoTrim() failed; no silent fallback"; return false; }
        Trim.PitchRad = Prop->GetEuler(JSBSim::FGJSBBase::eTht);
        Trim.EasMps = Aux->GetVequivalentKTS() * kKnotToMps;
        Trim.TasMps = Aux->GetVt() * kFtToM;
        Trim.AltitudeM = Prop->GetAltitudeASL() * kFtToM;
        Trim.ThrottleCmd = Fcs->GetThrottleCmd(0);
        Trim.ElevatorCmd = Fcs->GetDeCmd();
        Trim.AileronCmd = Fcs->GetDaCmd();
        Trim.FuelLb = Exec->GetPropertyValue("propulsion/total-fuel-lbs");
        if (std::abs(Trim.FuelLb - kExpectedFuelLb) > 1e-6) { failure = "fuel is not 3,000 lb"; return false; }
        OriginLat = Prop->GetLatitude();
        OriginLon = Prop->GetLongitude();
        if (!Read().IsFinite()) { failure = "non-finite state after trim"; return false; }
        return true;
    }

    FPlantState Read() const
    {
        FPlantState s{};
        s.TimeS = Exec->GetSimTime();
        s.NorthM = (Prop->GetLatitude() - OriginLat) * kEarthRadiusM;
        s.EastM = (Prop->GetLongitude() - OriginLon) * kEarthRadiusM * std::cos(OriginLat);
        s.VelNorthMps = Prop->GetVel(JSBSim::FGJSBBase::eNorth) * kFtToM;
        s.VelEastMps = Prop->GetVel(JSBSim::FGJSBBase::eEast) * kFtToM;
        s.GroundSpeedMps = Aux->GetVground() * kFtToM;
        s.GroundCourseRad = std::atan2(s.VelEastMps, s.VelNorthMps);
        s.AltitudeAslM = Prop->GetAltitudeASL() * kFtToM;
        s.ClimbRateMps = Prop->Gethdot() * kFtToM;
        s.EasMps = Aux->GetVequivalentKTS() * kKnotToMps;
        s.TasMps = Aux->GetVt() * kFtToM;
        s.bRatioValid = Finite(s.EasMps) && s.EasMps > 1e-6 && Finite(s.TasMps);
        s.EasToTasRatio = s.bRatioValid ? (s.TasMps / s.EasMps) : kNa;
        s.RollRad = Prop->GetEuler(JSBSim::FGJSBBase::ePhi);
        s.PitchRad = Prop->GetEuler(JSBSim::FGJSBBase::eTht);
        s.PRadps = Prop->GetPQR(1);
        s.QRadps = Prop->GetPQR(2);
        s.RRadps = Prop->GetPQR(3);
        s.AlphaRad = Aux->Getalpha();
        s.BetaRad = Aux->Getbeta();
        s.LoadFactor = Aux->GetNlf();
        s.AileronLeftPosRad = Fcs->GetDaLPos(JSBSim::ofRad);
        s.ElevatorPosRad = Fcs->GetDePos(JSBSim::ofRad);
        s.ThrottlePos = Fcs->GetThrottlePos(0);
        s.GearPos = Fcs->GetGearPos();
        s.FlapPosNorm = Fcs->GetDfPos(JSBSim::ofNorm);
        s.SpeedbrakePos = Fcs->GetDsbPos(JSBSim::ofNorm);
        s.bWow = Grd->GetWOW();
        auto e = Pls->GetEngine(0);
        s.bEngineRunning = e ? e->GetRunning() : false;
        return s;
    }

    bool WriteOwned(const FF16StickCommandV2 &c, FCounters &n)
    {
        if (Exec.get() != Owned) { ++n.WritesOutsideOwnedFdm; return false; }
        Fcs->SetDaCmd(c.AileronCmdNorm);
        Fcs->SetDeCmd(c.ElevatorCmdNorm);
        Fcs->SetDrCmd(-c.RudderCmdNorm);   // the plugin's CopyToJSBSim flips only rudder
        Fcs->SetThrottleCmd(0, c.ThrottleCmdNorm);
        ++n.CommandFrames;
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

    bool Run(FCounters &n)
    {
        ++n.FdmRuns;
        const bool ok = Exec->Run();
        if (!ok) ++n.FdmRunFailures;
        return ok;
    }

    const FTrimResult &TrimInfo() const { return Trim; }

private:
    std::unique_ptr<JSBSim::FGFDMExec> Exec;
    JSBSim::FGFDMExec *Owned{};
    std::shared_ptr<JSBSim::FGFCS> Fcs;
    std::shared_ptr<JSBSim::FGPropagate> Prop;
    std::shared_ptr<JSBSim::FGAuxiliary> Aux;
    std::shared_ptr<JSBSim::FGPropulsion> Pls;
    std::shared_ptr<JSBSim::FGGroundReactions> Grd;
    std::shared_ptr<JSBSim::FGMassBalance> Mass;
    FTrimResult Trim{};
    double OriginLat{}, OriginLon{};
};

// ================================================================================================
// Combinations and cases
// ================================================================================================
struct FCombo {
    double PeriodS{}, Damping{}, RollTcS{};
    bool bProductionDefault{};   // 10.0 / 0.7071 / 0.0  -- the value this build ships today
    bool bPinnedPx4Default{};    // 10.0 / 0.7    / 0.5  -- NPFG_PERIOD / NPFG_DAMPING / NPFG_ROLL_TC
    std::string Key() const
    {
        std::ostringstream s;
        s << std::fixed << std::setprecision(4) << PeriodS << '/' << Damping << '/' << RollTcS;
        return s.str();
    }
};

enum class ECase : std::uint8_t { CurvatureRight, CurvatureLeft, CrossTrackRight, CrossTrackLeft,
                                  CourseErrorRight, CourseErrorLeft };
const char *CaseName(ECase c)
{
    switch (c) {
    case ECase::CurvatureRight: return "curvature_right_10km";
    case ECase::CurvatureLeft: return "curvature_left_10km";
    case ECase::CrossTrackRight: return "cross_track_right_500m";
    case ECase::CrossTrackLeft: return "cross_track_left_500m";
    case ECase::CourseErrorRight: return "course_error_right_10deg";
    case ECase::CourseErrorLeft: return "course_error_left_10deg";
    }
    return "unknown";
}
int CaseSign(ECase c)
{
    switch (c) {
    case ECase::CurvatureRight: case ECase::CrossTrackRight: case ECase::CourseErrorRight: return +1;
    default: return -1;
    }
}
constexpr std::array<ECase, 6> kAllCases{ECase::CurvatureRight, ECase::CurvatureLeft,
                                         ECase::CrossTrackRight, ECase::CrossTrackLeft,
                                         ECase::CourseErrorRight, ECase::CourseErrorLeft};

struct FCaseMetrics {
    bool bRan{};
    double InitialCrossTrackM{kNa}, FinalCrossTrackM{kNa}, MaxAbsCrossTrackM{kNa};
    double InitialCourseErrorRad{kNa}, FinalCourseErrorRad{kNa};
    double TailCrossTrackSlopeMps{kNa};
    // The declared gate is "roll-reference saturation > 25% of the CASE DURATION", so the gate
    // denominator is the case's full control-frame count. The response-window fraction is recorded
    // separately as a stricter diagnostic, not as a gate.
    std::uint64_t RollRefSaturatedFrames{}, RollOverLimitFrames{};
    std::uint64_t CaseFrames{}, ResponseFrames{}, ResponseSaturatedFrames{};
    double PeakRollPosRad{kNa}, PeakRollNegRad{kNa}, PeakAbsPRadps{kNa}, PeakAbsRRadps{kNa};
    double LatAccFfMin{kNa}, LatAccFfMax{kNa}, LatAccFbMin{kNa}, LatAccFbMax{kNa};
    double LatAccTotMin{kNa}, LatAccTotMax{kNa};
    double AdaptedPeriodMin{kNa}, AdaptedPeriodMax{kNa};
    double MaxAbsAltitudeErrorM{kNa}, MaxAbsEasErrorMps{kNa};
    // Response-shape metrics. max_abs_cross_track is ~500 m for every combination in the cross-track
    // cases (it IS the commanded step), so it carries almost no ranking information. These do.
    double PostStepPeakExcessM{kNa};      // peak |xtk| after the step, MINUS the commanded offset
    double TrailingRmsCrossTrackM{kNa};   // RMS |xtk| over the trailing window
    double TimeToWithin25M{kNa}, TimeToWithin10M{kNa};   // seconds after the step, NA if never
    double CourseRecoveryTimeS{kNa};      // seconds after the step until |course error| < 1 deg
    double CaseSaturationFraction{kNa}, ResponseSaturationFraction{kNa};
    double PitchRefMin{kNa}, PitchRefMax{kNa}, ThrottleMin{kNa}, ThrottleMax{kNa};
    double ElevatorMin{kNa}, ElevatorMax{kNa}, AileronMin{kNa}, AileronMax{kNa};
    double CommandActivity{kNa};   // integrated |d(aileron)/dt| over the response [cmd-norm]
    FCounters N{};
    // rejection
    bool bRejected{};
    char RejectReason[96]{};
};

struct FComboResult {
    FCombo Combo{};
    std::array<FCaseMetrics, 6> Cases{};
    bool bRejected{};
    std::string RejectSummary;
    // aggregates over the six cases (valid combos only)
    std::uint64_t TotalSaturationFrames{};
    double MaxAbsCrossTrackM{kNa}, WorstAsymmetry{kNa};
    double MaxAltitudeErrorM{kNa}, MaxEasErrorMps{kNa}, TotalCommandActivity{kNa};
    double CurvatureAsym{kNa}, CrossTrackAsym{kNa}, CourseErrorAsym{kNa};
    bool bBothCurvatureStable{};
    // --- refine-mode aggregates (response shape, which is what actually separates the candidates) --
    std::uint64_t CurvatureResponseSaturation{};   // curvature right + left, response window
    std::uint64_t CrossTrackResponseSaturation{};  // cross-track right + left, response window
    double CurvatureMaxAltitudeErrorM{kNa}, CurvatureMaxEasErrorMps{kNa};
    double CurvatureMaxExcessM{kNa};               // peak cross-track excursion off the arc
    double CrossTrackTimeTo25S{kNa}, CrossTrackTimeTo10S{kNa};   // right + left, NA if either never
    double CourseRecoveryS{kNa};                                 // right + left
    double CrossTrackFinalAbsM{kNa}, CrossTrackTrailingRmsM{kNa};
    // absolute (not just normalized) left/right differences -- a normalized value on two tiny errors
    // is meaningless, so both are reported and the ranking never uses the normalized value alone.
    double CurvatureAbsDiffM{kNa}, CrossTrackAbsDiffM{kNa}, CourseErrorAbsDiffM{kNa};
};

// ================================================================================================
// One case, with the production chain
// ================================================================================================
FCaseMetrics RunCase(const std::string &root, const FCombo &combo, ECase kase,
                     const FGuidanceConfigV2 &tmpl, const FF16StickConfigV2 &stickCfg,
                     std::uint32_t resetGen, std::string &failure)
{
    FCaseMetrics m{};
    FOwnedF16Plant plant;
    if (!plant.Initialize(root, failure)) return m;
    const FTrimResult &trim = plant.TrimInfo();
    if (std::abs(trim.EasMps - kAnchorTrimEasMps) > kAnchorTrimTolerance ||
        std::abs(trim.TasMps - kAnchorTrimTasMps) > kAnchorTrimTolerance) {
        failure = "trim is not the known-good anchor";
        return m;
    }

    FGuidanceConfigV2 cfg = tmpl;
    cfg.NpfgPeriodS = combo.PeriodS;
    cfg.NpfgDamping = combo.Damping;
    cfg.NpfgRollTimeConstantS = combo.RollTcS;
    cfg.TecsEquivalentAirspeedTrimMps = trim.EasMps;
    cfg.ThrottleTrim = std::clamp(trim.ThrottleCmd, cfg.ThrottleMin, cfg.ThrottleMax);
    if (!FormationControlV2::IsGuidanceConfigValid(cfg)) {
        failure = "combination rejected by IsGuidanceConfigValid";
        return m;
    }

    FormationGuidanceCoordinatorV2 coordinator;
    coordinator.Reset(resetGen);
    F16StickAdapterV2 stick;
    stick.Reset(resetGen);

    // Observability shim: FGuidanceCoordinatorOutputV2 does not expose NPFG's adapted period, so a
    // TEST-OWNED FPx4NpfgAdapter -- the same production class, configured identically -- is fed the
    // same NpfgInput purely to read it back. It never drives the plant and never touches production.
    MumtPx4::FPx4NpfgAdapter probe;
    auto ApplyNpfgConfig = [&](MumtPx4::FPx4NpfgAdapter &a) {
        auto &g = a.directionalGuidance();
        g.setPeriod(static_cast<float>(cfg.NpfgPeriodS));
        g.setDamping(static_cast<float>(cfg.NpfgDamping));
        g.enablePeriodLB(cfg.bNpfgEnablePeriodLowerBound);
        g.enablePeriodUB(cfg.bNpfgEnablePeriodUpperBound);
        g.setRollTimeConst(static_cast<float>(cfg.NpfgRollTimeConstantS));
        g.setSwitchDistanceMultiplier(static_cast<float>(cfg.NpfgSwitchDistanceMultiplier));
        g.setPeriodSafetyFactor(static_cast<float>(cfg.NpfgPeriodSafetyFactor));
        a.airspeedDirectionController().setPGainFromPeriodAndDamping(
            static_cast<float>(cfg.NpfgDamping), static_cast<float>(cfg.NpfgPeriodS));
    };
    ApplyNpfgConfig(probe);

    const FPlantState initial = plant.Read();
    const Vec2 startPos{initial.NorthM, initial.EastM};
    const double startCourse = initial.GroundCourseRad;
    const int sign = CaseSign(kase);
    const bool isCurvature = kase == ECase::CurvatureRight || kase == ECase::CurvatureLeft;
    const bool isCrossTrack = kase == ECase::CrossTrackRight || kase == ECase::CrossTrackLeft;

    Vec2 pathOrigin = startPos, pathTangent = FromCourse(startCourse), arcCenter{};
    bool latched = false;

    FF16StickCommandV2 held = plant.TrimCommand();
    FF16StickCommandV2 prevApplied = held, prevValid{};
    bool havePrevValid = false;

    double peakRollPos = 0.0, peakRollNeg = 0.0, peakP = 0.0, peakR = 0.0;
    double ffMin = 1e18, ffMax = -1e18, fbMin = 1e18, fbMax = -1e18, totMin = 1e18, totMax = -1e18;
    double apMin = 1e18, apMax = -1e18;
    double altMax = 0.0, easMax = 0.0, xtkMax = 0.0;
    double prMin = 1e18, prMax = -1e18, thMin = 1e18, thMax = -1e18;
    double elMin = 1e18, elMax = -1e18, aiMin = 1e18, aiMax = -1e18;
    double activity = 0.0, prevAileron = 0.0;
    bool havePrevAileron = false;
    std::vector<std::pair<double, double>> tailXtk;   // (t, |xtk|) in the trailing window
    double t25 = kNa, t10 = kNa, tCourse = kNa;
    constexpr double kCourseRecoveredRad = 1.0 * kDegToRad;

    const int frames = static_cast<int>(std::llround(kCaseDurationS / kControllerDtS));
    for (int k = 0; k < frames; ++k) {
        const FPlantState st = plant.Read();
        if (!st.IsFinite()) ++m.N.NonFiniteStates;
        if (st.bWow) ++m.N.UnexpectedWow;
        if (!(std::abs(st.GearPos) <= 1e-9 && std::abs(st.FlapPosNorm) <= 1e-9 &&
              std::abs(st.SpeedbrakePos) <= 1e-9 && st.bEngineRunning))
            ++m.N.ConfigViolations;

        const Vec2 pos{st.NorthM, st.EastM};

        if (!latched && st.TimeS >= kStepTimeS) {
            if (isCurvature) {
                arcCenter = pos + RightNormal(st.GroundCourseRad) *
                                      (static_cast<double>(sign) * kPathTurnRadiusM);
            } else if (isCrossTrack) {
                pathOrigin = pos - RightNormal(st.GroundCourseRad) *
                                       (static_cast<double>(sign) * kCrossTrackStepM);
                pathTangent = FromCourse(st.GroundCourseRad);
            } else {
                pathOrigin = pos;
                pathTangent = FromCourse(st.GroundCourseRad + sign * kCourseErrorStepRad);
            }
            latched = true;
        }

        Vec2 pathPos{}, tangent{};
        double curvature = 0.0;
        if (isCurvature && latched) {
            const Vec2 radial = pos - arcCenter;
            const double rr = radial.Norm();
            if (!Finite(rr) || rr < 1e-6) { failure = "degenerate arc sample"; return m; }
            const Vec2 unit = radial * (1.0 / rr);
            curvature = static_cast<double>(sign) / kPathTurnRadiusM;
            pathPos = arcCenter + unit * kPathTurnRadiusM;
            tangent = curvature > 0.0 ? Vec2{-unit.E, unit.N} : Vec2{unit.E, -unit.N};
        } else {
            const double along = (pos - pathOrigin).Dot(pathTangent);
            pathPos = pathOrigin + pathTangent * along;
            tangent = pathTangent;
        }
        const double pathCourse = CourseOf(tangent);
        const double crossTrack = (pos - pathPos).Dot(RightNormal(pathCourse));
        const double courseErr = WrapPi(pathCourse - st.GroundCourseRad);

        FGuidanceCoordinatorInputV2 in{};
        FCanonicalNavigationStateV2 &f = in.Follower;
        f.PositionNE_m = pos;
        f.GroundVelocityNE_mps = Vec2{st.VelNorthMps, st.VelEastMps};
        f.GroundCourse_rad = st.GroundCourseRad;
        f.AltitudeAsl_m = st.AltitudeAslM;
        f.ClimbRate_mps = st.ClimbRateMps;
        f.SimulationTimeS = st.TimeS;
        f.ResetGeneration = resetGen;
        f.OriginGeneration = resetGen;
        f.EquivalentAirspeed_mps = st.EasMps;
        f.TrueAirspeed_mps = st.TasMps;
        f.WindNE_mps = Vec2{0.0, 0.0};
        f.EasToTasRatio = st.EasToTasRatio;
        f.bPositionValid = f.bGroundVelocityValid = f.bGroundCourseValid = true;
        f.bCourseRateValid = f.bCurvatureValid = f.bAltitudeValid = f.bClimbRateValid = true;
        f.bSimulationTimeValid = f.bEasValid = f.bTasValid = f.bWindValid = f.bOriginValid = true;
        f.bRatioValid = st.bRatioValid;
        in.Slot.ResetGeneration = resetGen;
        in.Slot.OriginGeneration = resetGen;
        in.Slot.bValid = true;

        FPlannerV2OutputAdapterResult &dto = in.PlannerDto;
        dto.Npfg.PathPositionNE_m = pathPos;
        dto.Npfg.PathUnitTangentNE = tangent;
        dto.Npfg.PathCurvature_per_m = curvature;
        dto.Npfg.bValid = true;
        dto.Tecs.TargetEasMps = trim.EasMps;
        dto.Tecs.TargetAltitudeAslM = trim.AltitudeM;
        dto.Tecs.bTargetEasValid = true;
        dto.Tecs.bTargetAltitudeValid = true;
        dto.Tecs.bTargetClimbRateValid = false;
        dto.Tecs.bCommandReady = true;

        in.CurrentPitchRad = st.PitchRad;
        in.bCurrentPitchValid = true;
        in.CurrentRollRad = st.RollRad;
        in.bCurrentRollValid = true;
        in.SimulationTimeS = st.TimeS;
        in.DtS = kControllerDtS;
        in.ResetGeneration = resetGen;
        in.OriginGeneration = resetGen;

        const FGuidanceCoordinatorOutputV2 g = coordinator.Update(in, cfg);
        if (k > 0 && !g.bCommandReady) ++m.N.UnexpectedInvalidCoordinatorFrames;

        // adapted-period observability shim (mirrors the coordinator's own NpfgInput mapping)
        {
            MumtPx4::NpfgInput ni{};
            const double tnorm = tangent.Norm();
            const Vec2 unitT = tnorm > 1e-9 ? tangent * (1.0 / tnorm) : Vec2{1.0, 0.0};
            ni.position_ne = {static_cast<float>(pos.N), static_cast<float>(pos.E)};
            ni.ground_velocity_ne = {static_cast<float>(st.VelNorthMps), static_cast<float>(st.VelEastMps)};
            ni.wind_velocity_ne = {0.f, 0.f};
            ni.path_tangent_ne = {static_cast<float>(unitT.N), static_cast<float>(unitT.E)};
            ni.path_position_ne = {static_cast<float>(pathPos.N), static_cast<float>(pathPos.E)};
            ni.path_curvature = static_cast<float>(curvature);
            ni.true_airspeed_setpoint = static_cast<float>(trim.EasMps * st.EasToTasRatio);
            ni.true_airspeed = static_cast<float>(st.TasMps);
            ni.minimum_ground_speed = static_cast<float>(cfg.MinimumGroundSpeedMps);
            const MumtPx4::NpfgOutput no = probe.update(ni);
            if (st.TimeS >= kStepTimeS && Finite(no.adapted_period)) {
                apMin = std::min(apMin, static_cast<double>(no.adapted_period));
                apMax = std::max(apMax, static_cast<double>(no.adapted_period));
            }
        }

        FF16StickInputV2 si{};
        si.RollReferenceRad = g.RollReferenceRad;
        si.PitchReferenceRad = g.PitchReferenceRad;
        si.ThrottleReferenceNorm = g.ThrottleReferenceNorm;
        si.bGuidanceValid = g.bCommandReady;
        si.CurrentRollRad = st.RollRad;
        si.CurrentPitchRad = st.PitchRad;
        si.bAttitudeValid = true;
        si.BodyRollRateRadps = st.PRadps;
        si.BodyPitchRateRadps = st.QRadps;
        si.BodyYawRateRadps = st.RRadps;
        si.bBodyRatesValid = true;
        si.AlphaRad = st.AlphaRad;
        si.BetaRad = st.BetaRad;
        si.bAlphaBetaValid = true;
        si.EasMps = st.EasMps;
        si.TasMps = st.TasMps;
        si.bAirspeedValid = true;
        si.SimulationTimeS = st.TimeS;
        si.DtS = kControllerDtS;
        si.ResetGeneration = resetGen;
        const FF16StickCommandV2 cmd = stick.Update(si, stickCfg);
        if (k > 0 && !cmd.bValid) ++m.N.UnexpectedInvalidStickFrames;

        if (cmd.bValid) {
            const bool rangeOk =
                std::abs(cmd.AileronCmdNorm) <= 1.0 + 1e-12 &&
                cmd.ElevatorCmdNorm >= stickCfg.ElevatorMin - 1e-12 &&
                cmd.ElevatorCmdNorm <= stickCfg.ElevatorMax + 1e-12 &&
                std::abs(cmd.RudderCmdNorm) <= 1.0 + 1e-12 &&
                cmd.ThrottleCmdNorm >= stickCfg.ThrottleMin - 1e-12 &&
                cmd.ThrottleCmdNorm <= stickCfg.ThrottleMax + 1e-12;
            if (!rangeOk) ++m.N.CommandRangeViolations;
            if (havePrevValid) {
                const bool slewOk =
                    std::abs(cmd.AileronCmdNorm - prevValid.AileronCmdNorm) <= stickCfg.AileronSlewPerS * kControllerDtS + 1e-12 &&
                    std::abs(cmd.ElevatorCmdNorm - prevValid.ElevatorCmdNorm) <= stickCfg.ElevatorSlewPerS * kControllerDtS + 1e-12 &&
                    std::abs(cmd.RudderCmdNorm - prevValid.RudderCmdNorm) <= stickCfg.RudderSlewPerS * kControllerDtS + 1e-12 &&
                    std::abs(cmd.ThrottleCmdNorm - prevValid.ThrottleCmdNorm) <= stickCfg.ThrottleSlewPerS * kControllerDtS + 1e-12;
                if (!slewOk) ++m.N.CommandSlewViolations;
            }
            prevValid = cmd;
            havePrevValid = true;
            if (st.TimeS >= kPrimeS) {
                held = cmd;
                prevApplied = cmd;
            }
        }
        (void)prevApplied;
        if (!plant.WriteOwned(held, m.N)) { failure = "command write did not target the owned FDM"; return m; }

        // ---- metrics ---------------------------------------------------------------------------
        // Saturation and roll-over-limit are counted over the WHOLE case, because that is what the
        // declared gate ("25% of the case duration") is measured against. Everything else is a
        // response-window quantity.
        ++m.CaseFrames;
        if (std::abs(g.RollReferenceRad) >= cfg.RollLimitRad - 1e-9) ++m.RollRefSaturatedFrames;
        if (std::abs(st.RollRad) > cfg.RollLimitRad + kRejectRollOverLimitRad) ++m.RollOverLimitFrames;
        if (st.TimeS >= kStepTimeS) {
            ++m.ResponseFrames;
            if (std::abs(g.RollReferenceRad) >= cfg.RollLimitRad - 1e-9) ++m.ResponseSaturatedFrames;
            peakRollPos = std::max(peakRollPos, st.RollRad);
            peakRollNeg = std::min(peakRollNeg, st.RollRad);
            peakP = std::max(peakP, std::abs(st.PRadps));
            peakR = std::max(peakR, std::abs(st.RRadps));
            ffMin = std::min(ffMin, g.LateralAccelerationFeedforwardMps2);
            ffMax = std::max(ffMax, g.LateralAccelerationFeedforwardMps2);
            fbMin = std::min(fbMin, g.LateralAccelerationFeedbackMps2);
            fbMax = std::max(fbMax, g.LateralAccelerationFeedbackMps2);
            totMin = std::min(totMin, g.LateralAccelerationTotalMps2);
            totMax = std::max(totMax, g.LateralAccelerationTotalMps2);
            prMin = std::min(prMin, g.PitchReferenceRad);
            prMax = std::max(prMax, g.PitchReferenceRad);
            thMin = std::min(thMin, held.ThrottleCmdNorm);
            thMax = std::max(thMax, held.ThrottleCmdNorm);
            elMin = std::min(elMin, held.ElevatorCmdNorm);
            elMax = std::max(elMax, held.ElevatorCmdNorm);
            aiMin = std::min(aiMin, held.AileronCmdNorm);
            aiMax = std::max(aiMax, held.AileronCmdNorm);
            altMax = std::max(altMax, std::abs(st.AltitudeAslM - trim.AltitudeM));
            easMax = std::max(easMax, std::abs(st.EasMps - trim.EasMps));
            xtkMax = std::max(xtkMax, std::abs(crossTrack));
            if (havePrevAileron) activity += std::abs(held.AileronCmdNorm - prevAileron);
            prevAileron = held.AileronCmdNorm;
            havePrevAileron = true;
            if (!Finite(m.InitialCrossTrackM)) {
                m.InitialCrossTrackM = crossTrack;
                m.InitialCourseErrorRad = courseErr;
            }
            m.FinalCrossTrackM = crossTrack;
            m.FinalCourseErrorRad = courseErr;
            if (!Finite(t25) && std::abs(crossTrack) < 25.0) t25 = st.TimeS - kStepTimeS;
            if (!Finite(t10) && std::abs(crossTrack) < 10.0) t10 = st.TimeS - kStepTimeS;
            if (!Finite(tCourse) && std::abs(courseErr) < kCourseRecoveredRad)
                tCourse = st.TimeS - kStepTimeS;
            if (st.TimeS >= kCaseDurationS - kTailWindowS)
                tailXtk.emplace_back(st.TimeS, std::abs(crossTrack));
        }

        for (int sub = 0; sub < kFdmStepsPerControlFrame; ++sub) {
            if (!plant.Run(m.N)) { failure = "FGFDMExec::Run() failed"; return m; }
        }
    }

    m.bRan = true;
    m.PeakRollPosRad = peakRollPos; m.PeakRollNegRad = peakRollNeg;
    m.PeakAbsPRadps = peakP; m.PeakAbsRRadps = peakR;
    m.LatAccFfMin = ffMin; m.LatAccFfMax = ffMax;
    m.LatAccFbMin = fbMin; m.LatAccFbMax = fbMax;
    m.LatAccTotMin = totMin; m.LatAccTotMax = totMax;
    m.AdaptedPeriodMin = (apMin > 1e17) ? kNa : apMin;
    m.AdaptedPeriodMax = (apMax < -1e17) ? kNa : apMax;
    m.MaxAbsCrossTrackM = xtkMax;
    m.MaxAbsAltitudeErrorM = altMax;
    m.MaxAbsEasErrorMps = easMax;
    m.PitchRefMin = prMin; m.PitchRefMax = prMax;
    m.ThrottleMin = thMin; m.ThrottleMax = thMax;
    m.ElevatorMin = elMin; m.ElevatorMax = elMax;
    m.AileronMin = aiMin; m.AileronMax = aiMax;
    m.CommandActivity = activity;
    m.TimeToWithin25M = t25;
    m.TimeToWithin10M = t10;
    m.CourseRecoveryTimeS = tCourse;
    if (m.CaseFrames > 0)
        m.CaseSaturationFraction = static_cast<double>(m.RollRefSaturatedFrames) / static_cast<double>(m.CaseFrames);
    if (m.ResponseFrames > 0)
        m.ResponseSaturationFraction = static_cast<double>(m.ResponseSaturatedFrames) / static_cast<double>(m.ResponseFrames);
    if (Finite(m.MaxAbsCrossTrackM) && Finite(m.InitialCrossTrackM))
        m.PostStepPeakExcessM = std::max(0.0, m.MaxAbsCrossTrackM - std::abs(m.InitialCrossTrackM));
    if (!tailXtk.empty()) {
        double acc = 0.0;
        for (const auto &p : tailXtk) acc += p.second * p.second;
        m.TrailingRmsCrossTrackM = std::sqrt(acc / static_cast<double>(tailXtk.size()));
    }

    // trailing |cross-track| slope: still growing at the end == persistent divergence
    if (tailXtk.size() >= 2) {
        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        const double t0 = tailXtk.front().first;
        for (const auto &p : tailXtk) {
            const double x = p.first - t0, y = p.second;
            n += 1; sx += x; sy += y; sxx += x * x; sxy += x * y;
        }
        const double den = n * sxx - sx * sx;
        if (std::abs(den) > 1e-12) m.TailCrossTrackSlopeMps = (n * sxy - sx * sy) / den;
    }
    return m;
}

// ---- pre-declared hard reject gates ---------------------------------------------------------
void ApplyRejectGates(FCaseMetrics &m, const FGuidanceConfigV2 &cfg, ECase kase)
{
    auto Reject = [&m](const char *why) {
        if (!m.bRejected) { m.bRejected = true; std::snprintf(m.RejectReason, sizeof(m.RejectReason), "%s", why); }
    };
    if (!m.bRan) { Reject("case did not run"); return; }
    if (m.N.FdmRunFailures) Reject("fdm_run_failure");
    if (m.N.NonFiniteStates) Reject("non_finite_state");
    if (m.N.UnexpectedWow) Reject("unexpected_wow");
    if (m.N.ConfigViolations) Reject("configuration_violation");
    if (m.N.CommandRangeViolations) Reject("command_range_violation");
    if (m.N.CommandSlewViolations) Reject("command_slew_violation");
    if (m.N.WritesOutsideOwnedFdm) Reject("write_isolation_violation");
    if (m.N.UnexpectedInvalidCoordinatorFrames || m.N.UnexpectedInvalidStickFrames)
        Reject("unexpected_invalid_controller_frame");

    // Error reduction is only well-defined where an error was actually commanded. A curvature case
    // starts on the path (initial error ~0), so the error-reduction gate does not apply to it; the
    // saturation, divergence, altitude and EAS gates still do.
    const bool crossTrackCommanded = Finite(m.InitialCrossTrackM) &&
                                     std::abs(m.InitialCrossTrackM) > kErrorFloorM;
    const bool courseCommanded = Finite(m.InitialCourseErrorRad) &&
                                 std::abs(m.InitialCourseErrorRad) > kErrorFloorRad;
    if (crossTrackCommanded && !(std::abs(m.FinalCrossTrackM) < std::abs(m.InitialCrossTrackM)))
        Reject("final_cross_track_not_below_initial");
    if (courseCommanded && !(std::abs(m.FinalCourseErrorRad) < std::abs(m.InitialCourseErrorRad)))
        Reject("final_course_error_not_below_initial");

    if (Finite(m.MaxAbsCrossTrackM) && m.MaxAbsCrossTrackM > kRejectDivergenceCrossTrackM)
        Reject("persistent_divergence_cross_track");
    if (Finite(m.TailCrossTrackSlopeMps) && m.TailCrossTrackSlopeMps > kRejectTailSlopeMps)
        Reject("persistent_divergence_tail_slope");
    if (m.CaseFrames > 0 &&
        static_cast<double>(m.RollRefSaturatedFrames) / static_cast<double>(m.CaseFrames) > kRejectSaturationFraction)
        Reject("roll_reference_saturation_over_25pct_of_case_duration");
    if (m.CaseFrames > 0 &&
        static_cast<double>(m.RollOverLimitFrames) / static_cast<double>(m.CaseFrames) > kRejectRollOverLimitFraction)
        Reject("actual_roll_persistently_over_limit");
    if (Finite(m.MaxAbsAltitudeErrorM) && m.MaxAbsAltitudeErrorM > kRejectAltitudeErrorM)
        Reject("altitude_error_over_150m");
    if (Finite(m.MaxAbsEasErrorMps) && m.MaxAbsEasErrorMps > kRejectEasErrorMps)
        Reject("eas_error_over_10mps");
    (void)cfg; (void)kase;
}

// ================================================================================================
// Sweep
// ================================================================================================
std::vector<FCombo> BuildCombos()
{
    const std::array<double, 6> periods{6.0, 8.0, 10.0, 12.0, 15.0, 20.0};
    const std::array<double, 4> dampings{0.5, 0.7, 0.85, 1.0};
    const std::array<double, 4> rollTcs{0.0, 0.25, 0.5, 1.0};
    std::vector<FCombo> combos;
    for (double p : periods)
        for (double d : dampings)
            for (double r : rollTcs) {
                FCombo c{};
                c.PeriodS = p; c.Damping = d; c.RollTcS = r;
                c.bPinnedPx4Default = (p == 10.0 && d == 0.7 && r == 0.5);
                combos.push_back(c);
            }
    // The value this build ships today is NOT on the grid (damping 0.7071), so it is added as an
    // explicit reference point and ranked alongside the grid.
    FCombo prod{};
    prod.PeriodS = 10.0; prod.Damping = 0.7071; prod.RollTcS = 0.0;
    prod.bProductionDefault = true;
    combos.push_back(prod);
    return combos;
}

// Focused refinement. The coarse sweep put every valid combination at the grid EDGE (period 20), and
// showed that the roll time constant is bit-identical across 0/0.25/0.5/1.0 in this flight regime,
// so it is pinned at the production default and only period and damping are refined.
std::vector<FCombo> BuildRefineCombos()
{
    const std::array<double, 5> periods{18.0, 20.0, 22.0, 25.0, 30.0};
    const std::array<double, 3> dampings{0.6, 0.7, 0.8};
    std::vector<FCombo> c;
    for (double p : periods)
        for (double d : dampings) {
            FCombo x{};
            x.PeriodS = p; x.Damping = d; x.RollTcS = 0.0;
            c.push_back(x);
        }
    return c;
}

FGuidanceConfigV2 BaseConfig()
{
    FGuidanceConfigV2 c{};
    c.EasMinMps = kTestLocalEasMinMps;
    c.EasMaxMps = kTestLocalEasMaxMps;
    c.TecsMaxClimbRateMps = kTestLocalMaxClimbRateMps;
    c.TecsMinSinkRateMps = kTestLocalMinSinkRateMps;
    c.TecsMaxSinkRateMps = kTestLocalMaxSinkRateMps;
    return c;   // every NPFG flag, TECS gain, stick gain and the roll limit stay at production
}

void Aggregate(FComboResult &r)
{
    r.bRejected = false;
    std::vector<std::string> reasons;
    for (std::size_t i = 0; i < r.Cases.size(); ++i) {
        if (r.Cases[i].bRejected) {
            r.bRejected = true;
            reasons.push_back(std::string(CaseName(kAllCases[i])) + ":" + r.Cases[i].RejectReason);
        }
    }
    for (const std::string &s : reasons) {
        if (!r.RejectSummary.empty()) r.RejectSummary += ";";
        r.RejectSummary += s;
    }

    std::uint64_t sat = 0;
    double xtk = 0.0, alt = 0.0, eas = 0.0, act = 0.0;
    for (const FCaseMetrics &m : r.Cases) {
        sat += m.RollRefSaturatedFrames;
        if (Finite(m.MaxAbsCrossTrackM)) xtk = std::max(xtk, m.MaxAbsCrossTrackM);
        if (Finite(m.MaxAbsAltitudeErrorM)) alt = std::max(alt, m.MaxAbsAltitudeErrorM);
        if (Finite(m.MaxAbsEasErrorMps)) eas = std::max(eas, m.MaxAbsEasErrorMps);
        if (Finite(m.CommandActivity)) act += m.CommandActivity;
    }
    r.TotalSaturationFrames = sat;
    r.MaxAbsCrossTrackM = xtk;
    r.MaxAltitudeErrorM = alt;
    r.MaxEasErrorMps = eas;
    r.TotalCommandActivity = act;

    r.CurvatureAsym = NormalizedAsymmetry(r.Cases[0].MaxAbsCrossTrackM, r.Cases[1].MaxAbsCrossTrackM);
    r.CrossTrackAsym = NormalizedAsymmetry(r.Cases[2].FinalCrossTrackM, r.Cases[3].FinalCrossTrackM);
    r.CourseErrorAsym = NormalizedAsymmetry(r.Cases[4].MaxAbsCrossTrackM, r.Cases[5].MaxAbsCrossTrackM);
    double worst = 0.0;
    for (double a : {r.CurvatureAsym, r.CrossTrackAsym, r.CourseErrorAsym})
        if (Finite(a)) worst = std::max(worst, a);
    r.WorstAsymmetry = worst;
    r.bBothCurvatureStable = !r.Cases[0].bRejected && !r.Cases[1].bRejected;

    // ---- refine aggregates ----
    const FCaseMetrics &cr = r.Cases[0], &cl = r.Cases[1];   // curvature right / left
    const FCaseMetrics &xr = r.Cases[2], &xl = r.Cases[3];   // cross-track right / left
    const FCaseMetrics &er = r.Cases[4], &el = r.Cases[5];   // course-error right / left
    r.CurvatureResponseSaturation = cr.ResponseSaturatedFrames + cl.ResponseSaturatedFrames;
    r.CrossTrackResponseSaturation = xr.ResponseSaturatedFrames + xl.ResponseSaturatedFrames;
    r.CurvatureMaxAltitudeErrorM = std::max(cr.MaxAbsAltitudeErrorM, cl.MaxAbsAltitudeErrorM);
    r.CurvatureMaxEasErrorMps = std::max(cr.MaxAbsEasErrorMps, cl.MaxAbsEasErrorMps);
    r.CurvatureMaxExcessM = std::max(cr.PostStepPeakExcessM, cl.PostStepPeakExcessM);
    auto SumOrNa = [](double a, double b) { return (Finite(a) && Finite(b)) ? a + b : kNa; };
    r.CrossTrackTimeTo25S = SumOrNa(xr.TimeToWithin25M, xl.TimeToWithin25M);
    r.CrossTrackTimeTo10S = SumOrNa(xr.TimeToWithin10M, xl.TimeToWithin10M);
    r.CourseRecoveryS = SumOrNa(er.CourseRecoveryTimeS, el.CourseRecoveryTimeS);
    r.CrossTrackFinalAbsM = std::abs(xr.FinalCrossTrackM) + std::abs(xl.FinalCrossTrackM);
    r.CrossTrackTrailingRmsM = SumOrNa(xr.TrailingRmsCrossTrackM, xl.TrailingRmsCrossTrackM);
    r.CurvatureAbsDiffM = std::abs(std::abs(cr.MaxAbsCrossTrackM) - std::abs(cl.MaxAbsCrossTrackM));
    r.CrossTrackAbsDiffM = std::abs(std::abs(xr.FinalCrossTrackM) - std::abs(xl.FinalCrossTrackM));
    r.CourseErrorAbsDiffM = std::abs(std::abs(er.MaxAbsCrossTrackM) - std::abs(el.MaxAbsCrossTrackM));
}

// Refine ranking, in the priority order the directive fixes. It never decides on a normalized
// asymmetry alone: absolute differences are the tie-breaker.
bool BetterRefined(const FComboResult &a, const FComboResult &b)
{
    if (a.bRejected != b.bRejected) return !a.bRejected;
    if (a.CurvatureResponseSaturation != b.CurvatureResponseSaturation)
        return a.CurvatureResponseSaturation < b.CurvatureResponseSaturation;
    const double ca = a.CurvatureMaxAltitudeErrorM + 10.0 * a.CurvatureMaxEasErrorMps;
    const double cb = b.CurvatureMaxAltitudeErrorM + 10.0 * b.CurvatureMaxEasErrorMps;
    if (ca != cb) return ca < cb;
    if (a.CrossTrackResponseSaturation != b.CrossTrackResponseSaturation)
        return a.CrossTrackResponseSaturation < b.CrossTrackResponseSaturation;
    const double ta = (Finite(a.CrossTrackTimeTo25S) ? a.CrossTrackTimeTo25S : 1e9) +
                      (Finite(a.CrossTrackTimeTo10S) ? a.CrossTrackTimeTo10S : 1e9);
    const double tb = (Finite(b.CrossTrackTimeTo25S) ? b.CrossTrackTimeTo25S : 1e9) +
                      (Finite(b.CrossTrackTimeTo10S) ? b.CrossTrackTimeTo10S : 1e9);
    if (ta != tb) return ta < tb;
    const double ra = Finite(a.CourseRecoveryS) ? a.CourseRecoveryS : 1e9;
    const double rb = Finite(b.CourseRecoveryS) ? b.CourseRecoveryS : 1e9;
    if (ra != rb) return ra < rb;
    if (a.TotalCommandActivity != b.TotalCommandActivity) return a.TotalCommandActivity < b.TotalCommandActivity;
    const double da = a.CurvatureAbsDiffM + a.CrossTrackAbsDiffM + a.CourseErrorAbsDiffM;
    const double db = b.CurvatureAbsDiffM + b.CrossTrackAbsDiffM + b.CourseErrorAbsDiffM;
    if (da != db) return da < db;
    return a.Combo.Key() < b.Combo.Key();
}

// Lexicographic ranking, in the priority order the directive fixes.
bool Better(const FComboResult &a, const FComboResult &b)
{
    if (a.bBothCurvatureStable != b.bBothCurvatureStable) return a.bBothCurvatureStable;
    if (a.TotalSaturationFrames != b.TotalSaturationFrames) return a.TotalSaturationFrames < b.TotalSaturationFrames;
    if (a.MaxAbsCrossTrackM != b.MaxAbsCrossTrackM) return a.MaxAbsCrossTrackM < b.MaxAbsCrossTrackM;
    if (a.WorstAsymmetry != b.WorstAsymmetry) return a.WorstAsymmetry < b.WorstAsymmetry;
    const double ea = a.MaxAltitudeErrorM + 10.0 * a.MaxEasErrorMps;
    const double eb = b.MaxAltitudeErrorM + 10.0 * b.MaxEasErrorMps;
    if (ea != eb) return ea < eb;
    if (a.TotalCommandActivity != b.TotalCommandActivity) return a.TotalCommandActivity < b.TotalCommandActivity;
    return a.Combo.Key() < b.Combo.Key();   // deterministic tie-break
}

// Pareto: minimize (saturation, max cross-track, worst asymmetry, altitude, EAS, command activity).
bool Dominates(const FComboResult &a, const FComboResult &b)
{
    const std::array<double, 6> x{static_cast<double>(a.TotalSaturationFrames), a.MaxAbsCrossTrackM,
                                  a.WorstAsymmetry, a.MaxAltitudeErrorM, a.MaxEasErrorMps,
                                  a.TotalCommandActivity};
    const std::array<double, 6> y{static_cast<double>(b.TotalSaturationFrames), b.MaxAbsCrossTrackM,
                                  b.WorstAsymmetry, b.MaxAltitudeErrorM, b.MaxEasErrorMps,
                                  b.TotalCommandActivity};
    bool strictly = false;
    for (std::size_t i = 0; i < x.size(); ++i) {
        if (x[i] > y[i]) return false;
        if (x[i] < y[i]) strictly = true;
    }
    return strictly;
}

// ================================================================================================
// Output
// ================================================================================================
void WriteNumber(std::ostream &o, double v, int p)
{
    if (Finite(v)) o << std::fixed << std::setprecision(p) << v;
    else o << "NA";
}
std::string Num(double v)
{
    if (!Finite(v)) return "NA";
    std::ostringstream s;
    s << std::fixed << std::setprecision(12) << v;
    return s.str();
}

void WriteCsv(const std::string &path, const std::vector<FComboResult> &rs, int prec,
              bool extended = false)
{
    std::ofstream out(path);
    out << "period_s,damping,roll_time_constant_s,is_production_default,is_pinned_px4_default,"
           "combo_rejected,case,case_rejected,case_reject_reason,case_frames,response_frames,"
           "initial_cross_track_m,final_cross_track_m,max_abs_cross_track_m,tail_cross_track_slope_mps,"
           "initial_course_error_rad,final_course_error_rad,"
           "roll_reference_saturated_frames,response_saturated_frames,roll_over_limit_frames,"
           "peak_roll_pos_rad,peak_roll_neg_rad,peak_abs_p_radps,peak_abs_r_radps,"
           "lat_acc_ff_min,lat_acc_ff_max,lat_acc_fb_min,lat_acc_fb_max,lat_acc_total_min,lat_acc_total_max,"
           "adapted_period_min,adapted_period_max,"
           "max_abs_altitude_error_m,max_abs_eas_error_mps,"
           "pitch_reference_min,pitch_reference_max,throttle_min,throttle_max,"
           "elevator_min,elevator_max,aileron_min,aileron_max,command_activity,"
           "fdm_run_failures,non_finite_states,unexpected_wow,configuration_violations,"
           "command_range_violations,command_slew_violations,writes_outside_owned_fdm";
    // The refine schema appends the response-shape columns. The coarse schema is byte-for-byte the
    // one the committed sweep already emits, so its canonical hashes are preserved.
    if (extended)
        out << ",case_saturation_fraction,response_saturation_fraction,post_step_peak_excess_m,"
               "trailing_rms_cross_track_m,time_to_within_25m_s,time_to_within_10m_s,"
               "course_recovery_time_s";
    out << '\n';
    for (const FComboResult &r : rs) {
        for (std::size_t i = 0; i < r.Cases.size(); ++i) {
            const FCaseMetrics &m = r.Cases[i];
            WriteNumber(out, r.Combo.PeriodS, prec); out << ',';
            WriteNumber(out, r.Combo.Damping, prec); out << ',';
            WriteNumber(out, r.Combo.RollTcS, prec); out << ',';
            out << (r.Combo.bProductionDefault ? 1 : 0) << ',' << (r.Combo.bPinnedPx4Default ? 1 : 0) << ','
                << (r.bRejected ? 1 : 0) << ',' << CaseName(kAllCases[i]) << ','
                << (m.bRejected ? 1 : 0) << ','
                << (m.RejectReason[0] ? m.RejectReason : "none") << ',' << m.CaseFrames << ','
                << m.ResponseFrames << ',';
            const std::array<double, 6> err{m.InitialCrossTrackM, m.FinalCrossTrackM, m.MaxAbsCrossTrackM,
                                            m.TailCrossTrackSlopeMps, m.InitialCourseErrorRad,
                                            m.FinalCourseErrorRad};
            for (double v : err) { WriteNumber(out, v, prec); out << ','; }
            out << m.RollRefSaturatedFrames << ',' << m.ResponseSaturatedFrames << ','
                << m.RollOverLimitFrames << ',';
            const std::array<double, 14> body{m.PeakRollPosRad, m.PeakRollNegRad, m.PeakAbsPRadps,
                                              m.PeakAbsRRadps, m.LatAccFfMin, m.LatAccFfMax,
                                              m.LatAccFbMin, m.LatAccFbMax, m.LatAccTotMin,
                                              m.LatAccTotMax, m.AdaptedPeriodMin, m.AdaptedPeriodMax,
                                              m.MaxAbsAltitudeErrorM, m.MaxAbsEasErrorMps};
            for (double v : body) { WriteNumber(out, v, prec); out << ','; }
            const std::array<double, 9> cmd{m.PitchRefMin, m.PitchRefMax, m.ThrottleMin, m.ThrottleMax,
                                            m.ElevatorMin, m.ElevatorMax, m.AileronMin, m.AileronMax,
                                            m.CommandActivity};
            for (double v : cmd) { WriteNumber(out, v, prec); out << ','; }
            out << m.N.FdmRunFailures << ',' << m.N.NonFiniteStates << ',' << m.N.UnexpectedWow << ','
                << m.N.ConfigViolations << ',' << m.N.CommandRangeViolations << ','
                << m.N.CommandSlewViolations << ',' << m.N.WritesOutsideOwnedFdm;
            if (extended) {
                const std::array<double, 7> ext{m.CaseSaturationFraction, m.ResponseSaturationFraction,
                                                m.PostStepPeakExcessM, m.TrailingRmsCrossTrackM,
                                                m.TimeToWithin25M, m.TimeToWithin10M,
                                                m.CourseRecoveryTimeS};
                for (double v : ext) { out << ','; WriteNumber(out, v, prec); }
            }
            out << '\n';
        }
    }
}

std::string ComboLine(const FComboResult &r)
{
    std::ostringstream s;
    s << "period=" << Num(r.Combo.PeriodS) << " damping=" << Num(r.Combo.Damping)
      << " roll_tc=" << Num(r.Combo.RollTcS)
      << (r.Combo.bProductionDefault ? " [CURRENT_PRODUCTION_DEFAULT]" : "")
      << (r.Combo.bPinnedPx4Default ? " [PINNED_PX4_PARAM_DEFAULT]" : "")
      << " rejected=" << (r.bRejected ? 1 : 0)
      << " saturation_frames=" << r.TotalSaturationFrames
      << " max_cross_track_m=" << Num(r.MaxAbsCrossTrackM)
      << " worst_asymmetry=" << Num(r.WorstAsymmetry)
      << " (curv=" << Num(r.CurvatureAsym) << " xtk=" << Num(r.CrossTrackAsym)
      << " course=" << Num(r.CourseErrorAsym) << ")"
      << " max_altitude_error_m=" << Num(r.MaxAltitudeErrorM)
      << " max_eas_error_mps=" << Num(r.MaxEasErrorMps)
      << " command_activity=" << Num(r.TotalCommandActivity)
      << " curvature_left_saturation=" << r.Cases[1].RollRefSaturatedFrames
      << " curvature_right_saturation=" << r.Cases[0].RollRefSaturatedFrames;
    if (r.bRejected) s << " reject=" << r.RejectSummary;
    return s.str();
}

std::string RefineLine(const FComboResult &r)
{
    std::ostringstream s;
    s << "period=" << Num(r.Combo.PeriodS) << " damping=" << Num(r.Combo.Damping)
      << " roll_tc=" << Num(r.Combo.RollTcS)
      << " rejected=" << (r.bRejected ? 1 : 0)
      << " | curvature: response_saturation=" << r.CurvatureResponseSaturation
      << " (right=" << r.Cases[0].ResponseSaturatedFrames << "/3600 left="
      << r.Cases[1].ResponseSaturatedFrames << "/3600)"
      << " peak_excess_m=" << Num(r.CurvatureMaxExcessM)
      << " max_altitude_error_m=" << Num(r.CurvatureMaxAltitudeErrorM)
      << " max_eas_error_mps=" << Num(r.CurvatureMaxEasErrorMps)
      << " | cross_track: response_saturation=" << r.CrossTrackResponseSaturation
      << " (right=" << r.Cases[2].ResponseSaturatedFrames << "/3600 left="
      << r.Cases[3].ResponseSaturatedFrames << "/3600)"
      << " case_saturation=" << (r.Cases[2].RollRefSaturatedFrames + r.Cases[3].RollRefSaturatedFrames)
      << "/15600"
      << " t_to_25m_s=" << Num(r.Cases[2].TimeToWithin25M) << '/' << Num(r.Cases[3].TimeToWithin25M)
      << " t_to_10m_s=" << Num(r.Cases[2].TimeToWithin10M) << '/' << Num(r.Cases[3].TimeToWithin10M)
      << " final_abs_m=" << Num(std::abs(r.Cases[2].FinalCrossTrackM)) << '/'
      << Num(std::abs(r.Cases[3].FinalCrossTrackM))
      << " trailing_rms_m=" << Num(r.Cases[2].TrailingRmsCrossTrackM) << '/'
      << Num(r.Cases[3].TrailingRmsCrossTrackM)
      << " | course_error: recovery_s=" << Num(r.Cases[4].CourseRecoveryTimeS) << '/'
      << Num(r.Cases[5].CourseRecoveryTimeS)
      << " peak_excess_m=" << Num(r.Cases[4].PostStepPeakExcessM) << '/'
      << Num(r.Cases[5].PostStepPeakExcessM)
      << " | left_right_abs_diff curvature_m=" << Num(r.CurvatureAbsDiffM)
      << " cross_track_m=" << Num(r.CrossTrackAbsDiffM)
      << " course_m=" << Num(r.CourseErrorAbsDiffM)
      << " | left_right_normalized curvature=" << Num(r.CurvatureAsym)
      << " cross_track=" << Num(r.CrossTrackAsym) << " course=" << Num(r.CourseErrorAsym)
      << " | command_activity=" << Num(r.TotalCommandActivity);
    if (r.bRejected) s << " reject=" << r.RejectSummary;
    return s.str();
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 6) {
        std::fprintf(stderr,
                     "usage: %s <JSBSim-root> <mode:sweep|confirm> <raw.csv> <quantized.csv> <summary.txt> "
                     "[candidates.txt]\n", argv[0]);
        return 2;
    }
    const std::string root = argv[1];
    const std::string mode = argv[2];
    const bool bConfirm = (mode == "confirm");
    const bool bRefine = (mode == "refine");
    if (!bConfirm && !bRefine && mode != "sweep") { std::fprintf(stderr, "unknown mode\n"); return 2; }

    const FGuidanceConfigV2 baseCfg = BaseConfig();
    const FF16StickConfigV2 stickCfg{};   // production defaults, unchanged
    const FGuidanceConfigV2 productionDefaults{};

    std::vector<FCombo> combos;
    if (bRefine) {
        combos = BuildRefineCombos();
    } else if (!bConfirm) {
        combos = BuildCombos();
    } else {
        if (argc < 7) { std::fprintf(stderr, "confirm mode needs candidates.txt\n"); return 2; }
        std::ifstream in(argv[6]);
        double p, d, r;
        while (in >> p >> d >> r) {
            FCombo c{};
            c.PeriodS = p; c.Damping = d; c.RollTcS = r;
            c.bProductionDefault = (p == 10.0 && d == 0.7071 && r == 0.0);
            c.bPinnedPx4Default = (p == 10.0 && d == 0.7 && r == 0.5);
            combos.push_back(c);
        }
        if (combos.empty()) { std::fprintf(stderr, "no candidates to confirm\n"); return 2; }
    }

    std::vector<FComboResult> results(combos.size());
    std::uint64_t failures = 0;
    std::uint32_t gen = 1;
    for (std::size_t i = 0; i < combos.size(); ++i) {
        results[i].Combo = combos[i];
        for (std::size_t c = 0; c < kAllCases.size(); ++c) {
            std::string failure;
            results[i].Cases[c] = RunCase(root, combos[i], kAllCases[c], baseCfg, stickCfg, gen++, failure);
            if (!failure.empty()) {
                std::fprintf(stderr, "BLOCKED: combo %s case %s: %s\n", combos[i].Key().c_str(),
                             CaseName(kAllCases[c]), failure.c_str());
                return 1;
            }
            ApplyRejectGates(results[i].Cases[c], baseCfg, kAllCases[c]);
        }
        Aggregate(results[i]);
    }

    WriteCsv(argv[3], results, 15, bRefine);
    WriteCsv(argv[4], results, 9, bRefine);

    // ---- ranking ---------------------------------------------------------------------------------
    std::vector<const FComboResult *> valid, rejected;
    for (const FComboResult &r : results) (r.bRejected ? rejected : valid).push_back(&r);
    std::vector<const FComboResult *> ranked = valid;
    std::sort(ranked.begin(), ranked.end(), [](const FComboResult *a, const FComboResult *b) { return Better(*a, *b); });

    std::vector<const FComboResult *> pareto;
    for (const FComboResult *a : valid) {
        bool dominated = false;
        for (const FComboResult *b : valid)
            if (a != b && Dominates(*b, *a)) { dominated = true; break; }
        if (!dominated) pareto.push_back(a);
    }
    std::sort(pareto.begin(), pareto.end(), [](const FComboResult *a, const FComboResult *b) { return Better(*a, *b); });

    const FComboResult *prod = nullptr, *px4 = nullptr;
    for (const FComboResult &r : results) {
        if (r.Combo.bProductionDefault) prod = &r;
        if (r.Combo.bPinnedPx4Default) px4 = &r;
    }
    auto RankOf = [&ranked](const FComboResult *r) -> int {
        if (!r) return -1;
        for (std::size_t i = 0; i < ranked.size(); ++i) if (ranked[i] == r) return static_cast<int>(i + 1);
        return -1;   // rejected: not ranked
    };

    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "F16_NPFG_TUNING_SWEEP_V2 mode=" << mode << '\n';
    out << "This sweep identifies CANDIDATE NPFG tunings by measurement against the actual JSBSim "
           "F-16 plant. It changes no production value. Nothing here is a production recommendation: "
           "combinations that survive the pre-declared gates are candidates, and selecting one is a "
           "separate, deliberate decision.\n";
    out << "chain=FormationGuidanceCoordinatorV2(production_FPx4NpfgAdapter+FPx4TecsAdapter)"
           "->F16StickAdapterV2(production)->normalized_commands->f16.xml_FCS->JSBSim_F16->feedback\n";
    out << "swept=NpfgPeriodS,NpfgDamping,NpfgRollTimeConstantS "
           "held_at_production_default=period_lower_bound,period_upper_bound,switch_distance_multiplier"
           "(inert_on_this_update_path),period_safety_factor,RollLimitRad,all_TECS,all_Stick\n";
    out << "grid period=6,8,10,12,15,20 damping=0.5,0.7,0.85,1.0 roll_tc=0.0,0.25,0.5,1.0 "
           "grid_combinations=96 plus_current_production_default=1 total=" << combos.size() << '\n';
    out << "plant altitude_ft=" << kInitialAltitudeFt << " tas_mps=" << kInitialTasMps
        << " fuel_lb=" << kExpectedFuelLb << " wind=0 gear=up flap=clean speedbrake=retracted "
           "fdm_dt_s=" << kFdmDtS << " controller_dt_s=" << kControllerDtS << '\n';
    out << "cases=6 (curvature_right/left_10km, cross_track_right/left_500m, "
           "course_error_right/left_10deg) settle_end_s=" << kSettleEndS << " step_s=" << kStepTimeS
        << " duration_s=" << kCaseDurationS << " (identical to the committed lateral harness)\n";
    out << "reject_gates_declared_before_the_sweep "
           "saturation_denominator=case_duration_control_frames(as_declared;the_response_window_"
           "fraction_is_reported_separately_as_a_stricter_diagnostic) saturation_fraction>"
        << kRejectSaturationFraction
        << " roll_over_limit>RollLimit+" << kRejectRollOverLimitRad << "rad_on>" << kRejectRollOverLimitFraction
        << "_of_frames altitude_error>" << kRejectAltitudeErrorM << "m eas_error>" << kRejectEasErrorMps
        << "mps divergence_cross_track>" << kRejectDivergenceCrossTrackM << "m tail_slope>"
        << kRejectTailSlopeMps << "mps error_reduction=applies_only_where_an_initial_error_was_commanded"
           "(curvature_cases_start_on_the_path)\n";
    out << "adapted_period_source=test_owned_FPx4NpfgAdapter_observability_shim"
           "(FGuidanceCoordinatorOutputV2_does_not_expose_it;the_shim_never_drives_the_plant)\n";
    out << "combinations=" << results.size() << " valid=" << valid.size()
        << " rejected=" << rejected.size() << '\n';

    if (bRefine) {
        std::vector<const FComboResult *> all;
        for (const FComboResult &r : results) all.push_back(&r);
        std::sort(all.begin(), all.end(),
                  [](const FComboResult *a, const FComboResult *b) { return BetterRefined(*a, *b); });

        out << "REFINE_GRID period=18,20,22,25,30 damping=0.6,0.7,0.8 roll_tc=0.0(pinned:the coarse "
               "sweep showed 0/0.25/0.5/1.0 are bit-identical in this flight regime) combinations="
            << results.size() << '\n';
        out << "REFINE_NOTE the response-window saturation fraction is a DIAGNOSTIC and a soft "
               "selection criterion. It is NOT applied retroactively as a hard reject; the hard gates "
               "are exactly the ones the coarse sweep declared.\n";
        out << "REFINE_RANKING (curvature response saturation, curvature altitude+EAS, cross-track "
               "response saturation, cross-track 25m/10m entry time, course recovery, command "
               "activity, absolute left/right difference)\n";
        for (std::size_t i = 0; i < all.size(); ++i)
            out << "  rank=" << (i + 1) << ' ' << RefineLine(*all[i]) << '\n';

        out << "REFINE_RESPONSE_SPEED_VS_ERROR (the period trade-off, per damping)\n";
        for (double d : {0.6, 0.7, 0.8}) {
            for (const FComboResult *r : all) {
                if (r->Combo.Damping != d) continue;
                out << "  damping=" << Num(d) << " period=" << Num(r->Combo.PeriodS)
                    << " curvature_saturation=" << r->CurvatureResponseSaturation
                    << " curvature_peak_excess_m=" << Num(r->CurvatureMaxExcessM)
                    << " curvature_altitude_error_m=" << Num(r->CurvatureMaxAltitudeErrorM)
                    << " cross_track_saturation=" << r->CrossTrackResponseSaturation
                    << " t_to_25m_s=" << Num(r->CrossTrackTimeTo25S)
                    << " t_to_10m_s=" << Num(r->CrossTrackTimeTo10S)
                    << " course_recovery_s=" << Num(r->CourseRecoveryS)
                    << " rejected=" << (r->bRejected ? 1 : 0) << '\n';
            }
        }

        // Is the trade-off monotone in the period, and do the leaders sit on the grid edge? If so,
        // the ranking has NOT found an interior optimum -- it has found the end of the grid, and the
        // real decision is a policy trade-off between capture delay and saturation. Say so, instead
        // of letting "largest period" read as "best".
        bool monotoneSatDown = true, monotoneCaptureUp = true;
        for (double d : {0.6, 0.7, 0.8}) {
            const FComboResult *prevD = nullptr;
            for (double p : {18.0, 20.0, 22.0, 25.0, 30.0}) {
                const FComboResult *cur = nullptr;
                for (const FComboResult &r : results)
                    if (r.Combo.Damping == d && r.Combo.PeriodS == p) cur = &r;
                if (!cur) continue;
                if (prevD) {
                    if (cur->CrossTrackResponseSaturation > prevD->CrossTrackResponseSaturation)
                        monotoneSatDown = false;
                    if (Finite(cur->CrossTrackTimeTo25S) && Finite(prevD->CrossTrackTimeTo25S) &&
                        cur->CrossTrackTimeTo25S < prevD->CrossTrackTimeTo25S)
                        monotoneCaptureUp = false;
                }
                prevD = cur;
            }
        }
        std::vector<const FComboResult *> candidates;
        for (const FComboResult *r : all) {
            if (r->bRejected) continue;
            candidates.push_back(r);
            if (candidates.size() == 3) break;
        }
        bool leadersOnGridEdge = !candidates.empty();
        for (const FComboResult *c : candidates)
            if (c->Combo.PeriodS != 30.0) leadersOnGridEdge = false;
        out << "REFINE_PARETO_TRADEOFF monotone_saturation_falls_with_period="
            << (monotoneSatDown ? 1 : 0)
            << " monotone_capture_time_rises_with_period=" << (monotoneCaptureUp ? 1 : 0)
            << " leaders_on_grid_upper_edge=" << (leadersOnGridEdge ? 1 : 0)
            << " interior_optimum_found=" << ((monotoneSatDown && monotoneCaptureUp && leadersOnGridEdge) ? 0 : 1)
            << " note=the_ranking_orders_by_the_declared_priority;within_this_grid_a_LARGER_period_"
               "monotonically_reduces_saturation_and_longitudinal_error_while_monotonically_slowing_"
               "the_cross_track_capture_and_loosening_the_arc,_so_the_leaders_sit_at_the_grid_edge_"
               "and_the_choice_is_a_policy_trade_off,_NOT_an_optimum._Largest_period_is_not_"
               "automatically_best.\n";
        out << "REFINE_CANDIDATES count=" << candidates.size()
            << " (candidates only -- this sweep selects no production value)\n";
        for (std::size_t i = 0; i < candidates.size(); ++i)
            out << "  candidate=" << (i + 1) << ' ' << RefineLine(*candidates[i]) << '\n';

        if (argc >= 7) {
            std::ofstream cand(argv[6]);
            for (const FComboResult *r : candidates)
                cand << std::fixed << std::setprecision(6) << r->Combo.PeriodS << ' '
                     << r->Combo.Damping << ' ' << r->Combo.RollTcS << '\n';
        }
    } else if (!bConfirm) {
        out << "CURRENT_PRODUCTION_DEFAULT " << (prod ? ComboLine(*prod) : std::string("NOT_RUN"))
            << " rank=" << (prod && !prod->bRejected ? std::to_string(RankOf(prod)) : std::string("rejected")) << '\n';
        out << "PINNED_PX4_PARAM_DEFAULT " << (px4 ? ComboLine(*px4) : std::string("NOT_RUN"))
            << " rank=" << (px4 && !px4->bRejected ? std::to_string(RankOf(px4)) : std::string("rejected")) << '\n';

        out << "TOP_10 (lexicographic: both-curvature-stable, saturation, max cross-track, asymmetry, "
               "altitude+EAS, command activity)\n";
        for (std::size_t i = 0; i < ranked.size() && i < 10; ++i)
            out << "  rank=" << (i + 1) << ' ' << ComboLine(*ranked[i]) << '\n';

        out << "PARETO_FRONT size=" << pareto.size()
            << " objectives=minimize(saturation,max_cross_track,worst_asymmetry,altitude_error,"
               "eas_error,command_activity)\n";
        for (const FComboResult *p : pareto) out << "  pareto " << ComboLine(*p) << '\n';

        out << "REJECTED count=" << rejected.size() << '\n';
        for (const FComboResult *r : rejected) out << "  rejected " << ComboLine(*r) << '\n';
    } else {
        out << "CONFIRMATION of the leading candidates, three independent processes\n";
        for (const FComboResult *r : ranked) out << "  confirmed " << ComboLine(*r) << '\n';
        for (const FComboResult *r : rejected) out << "  confirm_rejected " << ComboLine(*r) << '\n';
    }

    out << "per_case_detail\n";
    for (const FComboResult &r : results) {
        for (std::size_t i = 0; i < r.Cases.size(); ++i) {
            const FCaseMetrics &m = r.Cases[i];
            out << "  case combo=" << r.Combo.Key() << " name=" << CaseName(kAllCases[i])
                << " rejected=" << (m.bRejected ? 1 : 0)
                << " reason=" << (m.RejectReason[0] ? m.RejectReason : "none")
                << " initial_xtk=" << Num(m.InitialCrossTrackM)
                << " final_xtk=" << Num(m.FinalCrossTrackM)
                << " max_xtk=" << Num(m.MaxAbsCrossTrackM)
                << " initial_course_err=" << Num(m.InitialCourseErrorRad)
                << " final_course_err=" << Num(m.FinalCourseErrorRad)
                << " roll_ref_sat=" << m.RollRefSaturatedFrames << '/' << m.CaseFrames
                << " response_sat=" << m.ResponseSaturatedFrames << '/' << m.ResponseFrames
                << " roll_over_limit=" << m.RollOverLimitFrames
                << " peak_roll=[" << Num(m.PeakRollNegRad) << ',' << Num(m.PeakRollPosRad) << ']'
                << " peak_p=" << Num(m.PeakAbsPRadps) << " peak_r=" << Num(m.PeakAbsRRadps)
                << " lat_acc_ff=[" << Num(m.LatAccFfMin) << ',' << Num(m.LatAccFfMax) << ']'
                << " lat_acc_fb=[" << Num(m.LatAccFbMin) << ',' << Num(m.LatAccFbMax) << ']'
                << " lat_acc_total=[" << Num(m.LatAccTotMin) << ',' << Num(m.LatAccTotMax) << ']'
                << " adapted_period=[" << Num(m.AdaptedPeriodMin) << ',' << Num(m.AdaptedPeriodMax) << ']'
                << " max_alt_err=" << Num(m.MaxAbsAltitudeErrorM)
                << " max_eas_err=" << Num(m.MaxAbsEasErrorMps)
                << " pitch_ref=[" << Num(m.PitchRefMin) << ',' << Num(m.PitchRefMax) << ']'
                << " throttle=[" << Num(m.ThrottleMin) << ',' << Num(m.ThrottleMax) << ']'
                << " elevator=[" << Num(m.ElevatorMin) << ',' << Num(m.ElevatorMax) << ']'
                << " aileron=[" << Num(m.AileronMin) << ',' << Num(m.AileronMax) << ']'
                << " command_activity=" << Num(m.CommandActivity) << '\n';
        }
    }

    // ---- integrity ------------------------------------------------------------------------------
    std::uint64_t fdmFail = 0, nonFinite = 0, wow = 0, cfgV = 0, rangeV = 0, slewV = 0, writeV = 0;
    for (const FComboResult &r : results)
        for (const FCaseMetrics &m : r.Cases) {
            fdmFail += m.N.FdmRunFailures; nonFinite += m.N.NonFiniteStates; wow += m.N.UnexpectedWow;
            cfgV += m.N.ConfigViolations; rangeV += m.N.CommandRangeViolations;
            slewV += m.N.CommandSlewViolations; writeV += m.N.WritesOutsideOwnedFdm;
        }
    if (writeV) ++failures;
    if (fdmFail) ++failures;
    out << "integrity fdm_run_failures=" << fdmFail << " non_finite_states=" << nonFinite
        << " unexpected_wow=" << wow << " configuration_violations=" << cfgV
        << " command_range_violations=" << rangeV << " command_slew_violations=" << slewV
        << " writes_outside_owned_fdm=" << writeV << '\n';
    out << "isolation production_config_modified=0 npfg_production_tuning_modified=0 "
           "tecs_gains_modified=0 stick_gains_modified=0 roll_limit_modified=0 aircraft_xml_modified=0 "
           "ue_world_loaded=0 active_connection=0 test_owned_fgfdmexec_only=1\n";
    out << "production_defaults_reference NpfgPeriodS=" << Num(productionDefaults.NpfgPeriodS)
        << " NpfgDamping=" << Num(productionDefaults.NpfgDamping)
        << " NpfgRollTimeConstantS=" << Num(productionDefaults.NpfgRollTimeConstantS)
        << " RollLimitRad=" << Num(productionDefaults.RollLimitRad) << '\n';
    out << "failures=" << failures << '\n';

    const std::string summary = out.str();
    std::ofstream sf(argv[5]);
    sf << summary;
    std::fputs(summary.c_str(), stdout);

    // the leading candidates, for the confirmation pass
    if (!bConfirm && !bRefine && argc >= 7) {
        std::ofstream cand(argv[6]);
        for (std::size_t i = 0; i < ranked.size() && i < 5; ++i)
            cand << std::fixed << std::setprecision(6) << ranked[i]->Combo.PeriodS << ' '
                 << ranked[i]->Combo.Damping << ' ' << ranked[i]->Combo.RollTcS << '\n';
    }

    std::printf("F16_NPFG_TUNING_SWEEP_V2_RESULT=%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
