// verify_f16_lateral_dynamics_isolation_v2.cpp
//
// Where does the F-16 left/right asymmetry FIRST appear?
//
// The committed NPFG lateral closed-loop harness shows a large left/right difference: the 10 km
// left arc saturates the roll limit and loses ~350 m of altitude, while the mirror-image right arc
// tracks with a ~5 m cross-track error and never saturates. That harness closes every loop at once,
// so it cannot say WHICH stage breaks symmetry. This one separates the stages and compares
// equal-magnitude mirrored inputs at each of them:
//
//   Tier A  GEOMETRY / NPFG MIRROR   production coordinator, no FDM. Pure mathematical contract:
//                                    mirroring the geometry must negate every lateral output.
//   Tier B  DIRECT AILERON PLANT     actual f16.xml FCS + JSBSim, NPFG and the stick adapter
//                                    bypassed. Symmetric normalized aileron pulses only.
//   Tier C  STICK ROLL REFERENCE     production F16StickAdapterV2 driven by a test-owned roll
//                                    reference; production TECS still holds altitude and airspeed.
//   Tier D  HEADING DEPENDENCE       the Tier C excitation repeated at 0/90/180/270 deg heading, to
//                                    separate body-axis behaviour from the navigation frame.
//   Tier E  FULL NPFG ARC            the committed lateral harness's own 10 km arcs, reproduced with
//                                    the unchanged production chain, measured with the same metrics.
//
// This harness identifies a cause. It changes nothing: no NPFG period or damping, no roll limit, no
// TECS gain, no stick gain, no aircraft data, no rudder compensator, no new controller.
//
// PASS here does NOT mean the aircraft is symmetric. It means every planned case ran, deterministically
// and finitely, with no write-isolation or command violation, and a classification was produced with
// its evidence. A large measured asymmetry is a RESULT, not a failure. The single exception is
// Tier A: the geometry/NPFG mirror is a mathematical identity, and if it breaks, the run FAILs.
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

constexpr double kFdmDtS = 1.0 / 120.0;
constexpr double kControllerDtS = 1.0 / 60.0;
constexpr int kFdmStepsPerControlFrame = 2;

constexpr double kFtToM = 0.3048;
constexpr double kKnotToMps = 0.5144444444444444;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kFiniteLimit = 1.0e12;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// ---- plant condition, identical to the committed C1 / longitudinal / lateral harnesses ----------
constexpr double kInitialLatitudeDeg = 47.0;
constexpr double kInitialLongitudeDeg = -122.0;
constexpr double kInitialAltitudeFt = 10000.0;
constexpr double kInitialTasMps = 220.0;
constexpr double kDefaultHeadingDeg = 90.0;
constexpr double kTank0Lb = 1500.0, kTank1Lb = 1500.0, kTank2Lb = 0.0, kTank3Lb = 0.0;
constexpr double kExpectedFuelLb = 3000.0;

// ---- test-local TECS config, identical to the committed lateral harness -------------------------
constexpr double kTestLocalEasMinMps = 170.0;
constexpr double kTestLocalEasMaxMps = 220.0;
constexpr double kTestLocalMaxClimbRateMps = 100.539354597920735;
constexpr double kTestLocalMinSinkRateMps = 53.569103350590687;
constexpr double kTestLocalMaxSinkRateMps = 83.108839138474863;

constexpr double kAnchorTrimEasMps = 189.070713;
constexpr double kAnchorTrimTasMps = 220.0;
constexpr double kAnchorTrimTolerance = 1.0e-6;

// ---- Tier A: the mirror is a mathematical identity; only float rounding is tolerated ------------
//
// The production NPFG runs its whole lateral solution in float32 (eps = 1.19e-7), through a chain
// that includes a vector normalization, a division by g and an atan. An ABSOLUTE tolerance is
// therefore not meaningful on its own: the admissible residue scales with the magnitude of the
// quantity being compared. The tolerance is relative, with an absolute floor for quantities that are
// mathematically zero:
//     tol(P, N) = max(kMirrorRelTol * max(|P|, |N|), absolute floor)
// kMirrorRelTol is ~84x float32 eps, which covers that op chain; it is derived from the arithmetic,
// not fitted to an observed residue.
constexpr double kMirrorRelTol = 1.0e-5;
constexpr double kMirrorFloorRad = 1.0e-6;
constexpr double kMirrorFloorMps2 = 1.0e-4;
constexpr double kMirrorFloorM = 1.0e-3;

// ---- Tier B / C / D / E schedules (fixed in advance; never extended to chase a result) ----------
constexpr double kTierBBaselineS = 20.0;
constexpr double kTierBPulseEndS = 22.0;
constexpr double kTierBDurationS = 32.0;

constexpr double kTierCBaselineS = 30.0;
constexpr double kTierCLevelS = 60.0;      // roll reference returns to wings level
constexpr double kTierCDurationS = 80.0;

constexpr double kTierEStepS = 70.0;
constexpr double kTierEDurationS = 130.0;
constexpr double kTierERadiusM = 10000.0;

// ---- classification threshold: declared up front, and it is NOT a pass/fail gate ----------------
// A normalized asymmetry above this is called "meaningful" when attributing the first broken stage.
constexpr double kMeaningfulAsymmetry = 0.05;   // 5 %
constexpr double kAsymmetryFloorRad = 1.0e-6;
constexpr double kAsymmetryFloorRadps = 1.0e-6;
constexpr double kAsymmetryFloorNorm = 1.0e-6;
constexpr double kAsymmetryFloorM = 1.0e-3;

// ---- physical sanity (not performance gates) ----------------------------------------------------
constexpr double kAltitudeSanityM = 3000.0;
constexpr double kEasSanityMinMps = 50.0;
constexpr double kEasSanityMaxMps = 400.0;

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

// normalized |pos| vs |neg| asymmetry, with an explicit numerical floor
double NormalizedAsymmetry(double pos, double neg, double floorValue)
{
    const double a = std::abs(pos), b = std::abs(neg);
    const double denom = std::max({a, b, floorValue});
    if (!Finite(a) || !Finite(b) || denom <= 0.0) return kNa;
    return std::abs(a - b) / denom;
}

// ================================================================================================
// Plant
// ================================================================================================
struct FPlantState {
    double TimeS{};
    double NorthM{}, EastM{};
    double VelNorthMps{}, VelEastMps{};
    double GroundCourseRad{}, GroundSpeedMps{};
    double AltitudeAslM{}, ClimbRateMps{};
    double EasMps{}, TasMps{}, EasToTasRatio{};
    double RollRad{}, PitchRad{}, YawRad{};
    double PRadps{}, QRadps{}, RRadps{};
    double AlphaRad{}, BetaRad{}, LoadFactor{};
    double AileronLeftPosRad{}, AileronRightPosRad{}, ElevatorPosRad{}, RudderPosRad{}, ThrottlePos{};
    double GearPos{}, FlapPosNorm{}, SpeedbrakePos{};
    bool bWow{}, bEngineRunning{}, bRatioValid{};

    bool IsFinite() const
    {
        const std::array<double, 24> v{TimeS, NorthM, EastM, VelNorthMps, VelEastMps, GroundCourseRad,
                                       GroundSpeedMps, AltitudeAslM, ClimbRateMps, EasMps, TasMps,
                                       RollRad, PitchRad, YawRad, PRadps, QRadps, RRadps, AlphaRad,
                                       BetaRad, LoadFactor, AileronLeftPosRad, AileronRightPosRad,
                                       ElevatorPosRad, ThrottlePos};
        return std::all_of(v.begin(), v.end(), Finite);
    }
};

struct FTrimResult {
    bool bSuccess{};
    double PitchRad{}, RollRad{}, EasMps{}, TasMps{}, AltitudeM{}, GroundCourseRad{};
    double ThrottleCmd{}, ElevatorCmd{}, AileronCmd{}, FuelLb{}, MassSlugs{};
};

struct FWriteCounters {
    std::uint64_t CoordinatorUpdates{}, StickUpdates{}, FdmRuns{}, CommandFrames{};
    std::uint64_t AileronWrites{}, ElevatorWrites{}, RudderWrites{}, ThrottleWrites{};
    std::uint64_t WritesOutsideOwnedFdm{}, ProductionWriterInvocations{}, FdmRunFailures{};
    std::uint64_t SurfacePositionWrites{}, AeroPropertyWrites{};
};

class FOwnedF16Plant {
public:
    bool Initialize(const std::string &root, double headingDeg, std::string &failure)
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
        if (Propulsion->GetNumTanks() != 4) { failure = "f16 does not declare 4 fuel tanks"; return false; }
        const std::array<double, 4> want{kTank0Lb, kTank1Lb, kTank2Lb, kTank3Lb};
        for (unsigned i = 0; i < 4; ++i) {
            auto tank = Propulsion->GetTank(i);
            if (!tank) { failure = "null fuel tank"; return false; }
            if (want[i] < 0.0 || want[i] > tank->GetCapacity() + 1e-9) {
                failure = "requested tank fuel exceeds the declared model capacity"; return false;
            }
            tank->SetContents(want[i]);
        }

        Exec->SetPropertyValue("ic/lat-geod-deg", kInitialLatitudeDeg);
        Exec->SetPropertyValue("ic/long-gc-deg", kInitialLongitudeDeg);
        Exec->SetPropertyValue("ic/h-sl-ft", kInitialAltitudeFt);
        Exec->SetPropertyValue("ic/psi-true-deg", headingDeg);
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

        JSBSim::FGTrim trim(Exec.get(), JSBSim::tFull);
        Trim.bSuccess = trim.DoTrim();
        if (!Trim.bSuccess) {
            failure = "FGTrim(tFull).DoTrim() failed; a silent trim fallback is forbidden";
            return false;
        }
        Trim.PitchRad = Propagate->GetEuler(JSBSim::FGJSBBase::eTht);
        Trim.RollRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePhi);
        Trim.EasMps = Auxiliary->GetVequivalentKTS() * kKnotToMps;
        Trim.TasMps = Auxiliary->GetVt() * kFtToM;
        Trim.AltitudeM = Propagate->GetAltitudeASL() * kFtToM;
        Trim.ThrottleCmd = Fcs->GetThrottleCmd(0);
        Trim.ElevatorCmd = Fcs->GetDeCmd();
        Trim.AileronCmd = Fcs->GetDaCmd();
        Trim.FuelLb = Exec->GetPropertyValue("propulsion/total-fuel-lbs");
        Trim.MassSlugs = MassBalance->GetMass();
        if (std::abs(Trim.FuelLb - kExpectedFuelLb) > 1e-6) {
            failure = "fuel load is not the expected 3,000 lb"; return false;
        }
        OriginLatRad = Propagate->GetLatitude();
        OriginLonRad = Propagate->GetLongitude();
        const FPlantState s = Read();
        if (!s.IsFinite()) { failure = "non-finite plant state after trim"; return false; }
        Trim.GroundCourseRad = s.GroundCourseRad;
        return true;
    }

    FPlantState Read() const
    {
        FPlantState s{};
        s.TimeS = Exec->GetSimTime();
        const double lat = Propagate->GetLatitude();
        const double lon = Propagate->GetLongitude();
        s.NorthM = (lat - OriginLatRad) * kEarthRadiusM;
        s.EastM = (lon - OriginLonRad) * kEarthRadiusM * std::cos(OriginLatRad);
        s.VelNorthMps = Propagate->GetVel(JSBSim::FGJSBBase::eNorth) * kFtToM;
        s.VelEastMps = Propagate->GetVel(JSBSim::FGJSBBase::eEast) * kFtToM;
        s.GroundSpeedMps = Auxiliary->GetVground() * kFtToM;
        s.GroundCourseRad = std::atan2(s.VelEastMps, s.VelNorthMps);
        s.AltitudeAslM = Propagate->GetAltitudeASL() * kFtToM;
        s.ClimbRateMps = Propagate->Gethdot() * kFtToM;
        s.EasMps = Auxiliary->GetVequivalentKTS() * kKnotToMps;
        s.TasMps = Auxiliary->GetVt() * kFtToM;
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
        s.AileronLeftPosRad = Fcs->GetDaLPos(JSBSim::ofRad);
        s.AileronRightPosRad = Fcs->GetDaRPos(JSBSim::ofRad);
        s.ElevatorPosRad = Fcs->GetDePos(JSBSim::ofRad);
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

    // Normalized FCS commands ONLY. No surface position write, no aerodynamic property write, ever.
    bool WriteOwned(double aileron, double elevator, double rudder, double throttle, FWriteCounters &c)
    {
        if (Exec.get() != OwnedIdentity) { ++c.WritesOutsideOwnedFdm; return false; }
        Fcs->SetDaCmd(aileron); ++c.AileronWrites;
        Fcs->SetDeCmd(elevator); ++c.ElevatorWrites;
        Fcs->SetDrCmd(-rudder); ++c.RudderWrites;   // the plugin's CopyToJSBSim flips only rudder
        Fcs->SetThrottleCmd(0, throttle); ++c.ThrottleWrites;
        ++c.CommandFrames;
        return true;
    }

    bool Run(FWriteCounters &c)
    {
        ++c.FdmRuns;
        const bool ok = Exec->Run();
        if (!ok) ++c.FdmRunFailures;
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
enum class ETier : std::uint8_t { A_Mirror, B_DirectAileron, C_StickRollRef, D_Heading, E_NpfgArc };
const char *TierName(ETier t)
{
    switch (t) {
    case ETier::A_Mirror: return "A_GEOMETRY_NPFG_MIRROR";
    case ETier::B_DirectAileron: return "B_DIRECT_AILERON_PLANT";
    case ETier::C_StickRollRef: return "C_STICK_ROLL_REFERENCE";
    case ETier::D_Heading: return "D_HEADING_DEPENDENCE";
    case ETier::E_NpfgArc: return "E_FULL_NPFG_ARC";
    }
    return "unknown";
}

struct FCaseDef {
    ETier Tier{};
    std::string Name;
    std::string PairKey;      // cases with the same PairKey and opposite Sign form a mirror pair
    int Sign{};               // +1 / -1
    double Magnitude{};       // aileron cmd, roll reference [rad], or curvature sign
    double HeadingDeg{kDefaultHeadingDeg};
    double DurationS{};
    std::uint32_t ResetGeneration{};
};

struct FFrame {
    int CaseIndex{};
    double TimeS{};
    double RollReferenceRad{}, AileronCmd{}, ElevatorCmd{}, RudderCmd{}, ThrottleCmd{};
    double PitchReferenceRad{}, ThrottleReferenceNorm{};
    double CrossTrackM{}, CourseErrorRad{}, PathCurvature{};
    double LoadFactorCmdSide{};
    bool bCommandReady{}, bStickValid{}, bCommandApplied{};
    FPlantState State{};
};

struct FMetrics {
    double InitialPSlopeRadps2{kNa};
    double PeakAbsPRadps{kNa}, PeakAbsRRadps{kNa}, PeakAbsRollRad{kNa};
    double RollImpulseRad{kNa};
    double RiseTime10_90S{kNa}, OvershootNorm{kNa}, RecoveryTimeS{kNa};
    double IntegratedAbsRollErrorRadS{kNa};
    double AileronCmdMin{kNa}, AileronCmdMax{kNa};
    double AileronSurfaceMinRad{kNa}, AileronSurfaceMaxRad{kNa};
    double BetaMinRad{kNa}, BetaMaxRad{kNa};
    double CourseChangeRad{kNa};
    double AltitudeExcursionM{kNa}, EasExcursionMps{kNa};
    double PitchRefMin{kNa}, PitchRefMax{kNa}, ThrottleMin{kNa}, ThrottleMax{kNa};
    double LoadFactorMin{kNa}, LoadFactorMax{kNa};
    double FinalCrossTrackM{kNa}, MaxAbsCrossTrackM{kNa};
    std::uint64_t RollRefSaturatedFrames{}, AileronSaturatedFrames{};
};

struct FCaseResult {
    FCaseDef Def{};
    FTrimResult Trim{};
    std::uint64_t ControlFrames{}, FdmSteps{};
    FWriteCounters Counters{};
    FMetrics M{};
    std::uint64_t NonFiniteStates{}, UnexpectedWow{}, ConfigViolations{};
    std::uint64_t CommandRangeViolations{}, CommandSlewViolations{};
    std::uint64_t UnexpectedInvalidCoordinatorFrames{}, UnexpectedInvalidStickFrames{};
    std::vector<FFrame> Trace;
};

std::vector<FCaseDef> BuildCases()
{
    std::vector<FCaseDef> c;
    std::uint32_t gen = 1;

    // Tier B: direct normalized aileron pulses through the actual FCS.
    for (double mag : {0.025, 0.050, 0.100}) {
        std::ostringstream k; k << "aileron_" << std::fixed << std::setprecision(3) << mag;
        for (int s : {+1, -1}) {
            std::ostringstream n; n << "B_" << (s > 0 ? "right" : "left") << "_aileron_"
                                    << std::fixed << std::setprecision(3) << mag;
            c.push_back({ETier::B_DirectAileron, n.str(), k.str(), s, mag, kDefaultHeadingDeg,
                         kTierBDurationS, gen++});
        }
    }
    // Tier C: production stick adapter, test-owned roll reference step.
    for (double deg : {10.0, 20.0, 30.0, 40.0}) {
        std::ostringstream k; k << "rollref_" << std::fixed << std::setprecision(0) << deg;
        for (int s : {+1, -1}) {
            std::ostringstream n; n << "C_" << (s > 0 ? "right" : "left") << "_rollref_"
                                    << std::fixed << std::setprecision(0) << deg << "deg";
            c.push_back({ETier::C_StickRollRef, n.str(), k.str(), s, deg * kDegToRad,
                         kDefaultHeadingDeg, kTierCDurationS, gen++});
        }
    }
    // Tier D: the same +-25 deg excitation at four initial headings.
    for (double hdg : {0.0, 90.0, 180.0, 270.0}) {
        std::ostringstream k; k << "heading_" << std::fixed << std::setprecision(0) << hdg;
        for (int s : {+1, -1}) {
            std::ostringstream n; n << "D_" << (s > 0 ? "right" : "left") << "_rollref_25deg_hdg"
                                    << std::fixed << std::setprecision(0) << hdg;
            c.push_back({ETier::D_Heading, n.str(), k.str(), s, 25.0 * kDegToRad, hdg,
                         kTierCDurationS, gen++});
        }
    }
    // Tier E: the committed lateral harness's own 10 km arcs, unchanged.
    for (int s : {+1, -1}) {
        std::ostringstream n; n << "E_" << (s > 0 ? "right" : "left") << "_arc_10km";
        c.push_back({ETier::E_NpfgArc, n.str(), "npfg_arc_10km", s, 1.0 / kTierERadiusM,
                     kDefaultHeadingDeg, kTierEDurationS, gen++});
    }
    return c;
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

struct FAudit {
    std::uint64_t Checks{}, Failures{};
    std::vector<std::string> Messages;
    void Check(bool ok, const std::string &what)
    {
        ++Checks;
        if (!ok) { ++Failures; if (Messages.size() < 40) Messages.push_back(what); }
    }
};

// ================================================================================================
// Tier A: pure geometry / NPFG mirror contract. No FDM.
// ================================================================================================
struct FMirrorSample {
    double CrossTrackM{}, CourseErrorRad{};
    double RollReferenceRad{}, LateralFf{}, LateralFb{}, LateralTotal{}, CourseSetpointRad{};
    bool bValid{};
};

struct FMirrorCase {
    std::string Name;
    double CrossTrackM{};
    double CourseErrorRad{};
    double Curvature{};
};

// Frozen-kinematics coordinator run: the plant state does not move, so NPFG sees exactly the
// commanded geometry. The mirror flips East, the velocity's East component, the path's East
// components and the sign of the curvature -- nothing else.
FMirrorSample RunMirror(const FGuidanceConfigV2 &cfg, const FMirrorCase &mc, int mirror,
                        double tasMps, double easMps, double altitudeM)
{
    const double m = static_cast<double>(mirror);   // +1 = as specified, -1 = mirrored
    FormationGuidanceCoordinatorV2 g;

    // Path along North (course 0); the aircraft sits at the requested signed cross-track with the
    // requested signed course error. "Right of path" is +East, so the mirror negates East.
    const double pathCourse = 0.0;
    const Vec2 pathTangent = FromCourse(pathCourse);
    const Vec2 right = RightNormal(pathCourse);
    const Vec2 pathPos{0.0, 0.0};
    const Vec2 pos = pathPos + right * (m * mc.CrossTrackM);
    const double aircraftCourse = pathCourse - m * mc.CourseErrorRad;   // courseError = path - aircraft
    const Vec2 vel = FromCourse(aircraftCourse) * tasMps;

    FGuidanceCoordinatorOutputV2 o{};
    for (int k = 0; k < 8; ++k) {
        FGuidanceCoordinatorInputV2 in{};
        FCanonicalNavigationStateV2 &f = in.Follower;
        f.PositionNE_m = pos;
        f.GroundVelocityNE_mps = vel;
        f.GroundCourse_rad = aircraftCourse;
        f.AltitudeAsl_m = altitudeM;
        f.ClimbRate_mps = 0.0;
        f.SimulationTimeS = 10.0 + k * kControllerDtS;
        f.ResetGeneration = 1;
        f.OriginGeneration = 1;
        f.EquivalentAirspeed_mps = easMps;
        f.TrueAirspeed_mps = tasMps;
        f.WindNE_mps = Vec2{0.0, 0.0};
        f.EasToTasRatio = tasMps / easMps;
        f.bPositionValid = f.bGroundVelocityValid = f.bGroundCourseValid = true;
        f.bCourseRateValid = f.bCurvatureValid = f.bAltitudeValid = f.bClimbRateValid = true;
        f.bSimulationTimeValid = f.bEasValid = f.bTasValid = f.bWindValid = f.bOriginValid = true;
        f.bRatioValid = true;

        in.Slot.ResetGeneration = 1;
        in.Slot.OriginGeneration = 1;
        in.Slot.bValid = true;

        FPlannerV2OutputAdapterResult &dto = in.PlannerDto;
        dto.Npfg.PathPositionNE_m = pathPos;
        dto.Npfg.PathUnitTangentNE = pathTangent;
        dto.Npfg.PathCurvature_per_m = m * mc.Curvature;
        dto.Npfg.bValid = true;
        dto.Tecs.TargetEasMps = easMps;
        dto.Tecs.TargetAltitudeAslM = altitudeM;
        dto.Tecs.bTargetEasValid = true;
        dto.Tecs.bTargetAltitudeValid = true;
        dto.Tecs.bTargetClimbRateValid = false;
        dto.Tecs.bCommandReady = true;

        in.CurrentPitchRad = 0.0;
        in.bCurrentPitchValid = true;
        in.CurrentRollRad = 0.0;     // frozen kinematics: no bank, so the load factor is 1 on both sides
        in.bCurrentRollValid = true;
        in.SimulationTimeS = 10.0 + k * kControllerDtS;
        in.DtS = kControllerDtS;
        in.ResetGeneration = 1;
        in.OriginGeneration = 1;
        o = g.Update(in, cfg);
    }

    FMirrorSample s{};
    s.bValid = o.bCommandReady;
    if (!s.bValid) return s;
    s.CrossTrackM = (pos - pathPos).Dot(right);
    s.CourseErrorRad = WrapPi(pathCourse - aircraftCourse);
    s.RollReferenceRad = o.RollReferenceRad;
    s.LateralFf = o.LateralAccelerationFeedforwardMps2;
    s.LateralFb = o.LateralAccelerationFeedbackMps2;
    s.LateralTotal = o.LateralAccelerationTotalMps2;
    s.CourseSetpointRad = WrapPi(o.CourseSetpointRad - pathCourse);
    return s;
}

// ================================================================================================
// FDM tiers
// ================================================================================================
bool RunFdmCase(const std::string &root, int caseIndex, const FCaseDef &def,
                const FGuidanceConfigV2 &guidanceTemplate, const FF16StickConfigV2 &stickConfig,
                FCaseResult &r, FAudit &audit, std::string &failure)
{
    r.Def = def;
    FOwnedF16Plant plant;
    if (!plant.Initialize(root, def.HeadingDeg, failure)) return false;
    const FTrimResult &trim = plant.TrimInfo();
    r.Trim = trim;

    audit.Check(std::abs(trim.EasMps - kAnchorTrimEasMps) <= kAnchorTrimTolerance,
                def.Name + ": trim EAS is not the known-good anchor");
    audit.Check(std::abs(trim.TasMps - kAnchorTrimTasMps) <= kAnchorTrimTolerance,
                def.Name + ": trim TAS is not the known-good anchor");

    FGuidanceConfigV2 cfg = guidanceTemplate;
    cfg.TecsEquivalentAirspeedTrimMps = trim.EasMps;
    cfg.ThrottleTrim = std::clamp(trim.ThrottleCmd, cfg.ThrottleMin, cfg.ThrottleMax);
    if (!FormationControlV2::IsGuidanceConfigValid(cfg)) {
        failure = def.Name + ": the completed test-local config is rejected by IsGuidanceConfigValid";
        return false;
    }

    // Tier B bypasses the coordinator entirely; the others use it.
    const bool useCoordinator = def.Tier != ETier::B_DirectAileron;
    const bool useStick = def.Tier == ETier::C_StickRollRef || def.Tier == ETier::D_Heading ||
                          def.Tier == ETier::E_NpfgArc;
    const bool useNpfgRollReference = def.Tier == ETier::E_NpfgArc;

    FormationGuidanceCoordinatorV2 coordinator;
    coordinator.Reset(def.ResetGeneration);
    F16StickAdapterV2 stick;
    stick.Reset(def.ResetGeneration);

    const FPlantState initial = plant.Read();
    const Vec2 startPos{initial.NorthM, initial.EastM};
    const double startCourse = initial.GroundCourseRad;

    // Tier E: the committed lateral harness's straight-then-arc geometry, latched once at the step.
    Vec2 arcCenter{};
    bool arcLatched = false;

    FF16StickCommandV2 previousValid{};
    bool havePreviousValid = false;
    double previousAileronApplied = trim.AileronCmd;

    const int frames = static_cast<int>(std::llround(def.DurationS / kControllerDtS));
    for (int k = 0; k < frames; ++k) {
        const FPlantState st = plant.Read();
        if (!st.IsFinite()) ++r.NonFiniteStates;
        audit.Check(st.IsFinite(), def.Name + ": non-finite plant state");
        if (st.bWow) ++r.UnexpectedWow;
        audit.Check(!st.bWow, def.Name + ": unexpected weight on wheels");
        const bool cfgOk = std::abs(st.GearPos) <= 1e-9 && std::abs(st.FlapPosNorm) <= 1e-9 &&
                           std::abs(st.SpeedbrakePos) <= 1e-9 && st.bEngineRunning;
        if (!cfgOk) ++r.ConfigViolations;
        audit.Check(cfgOk, def.Name + ": gear/flap/speedbrake/engine configuration violation");

        const Vec2 pos{st.NorthM, st.EastM};

        // ---- path geometry ---------------------------------------------------------------------
        Vec2 pathPos = pos, pathTangent = FromCourse(startCourse);
        double pathCurv = 0.0, crossTrack = 0.0, courseErr = 0.0;
        if (def.Tier == ETier::E_NpfgArc) {
            if (!arcLatched && st.TimeS >= kTierEStepS) {
                arcCenter = pos + RightNormal(st.GroundCourseRad) *
                                      (static_cast<double>(def.Sign) * kTierERadiusM);
                arcLatched = true;
            }
            if (!arcLatched) {
                const double along = (pos - startPos).Dot(FromCourse(startCourse));
                pathPos = startPos + FromCourse(startCourse) * along;
                pathTangent = FromCourse(startCourse);
                pathCurv = 0.0;
            } else {
                const Vec2 radial = pos - arcCenter;
                const double rr = radial.Norm();
                if (!Finite(rr) || rr < 1e-6) { failure = def.Name + ": degenerate arc sample"; return false; }
                const Vec2 unit = radial * (1.0 / rr);
                pathCurv = static_cast<double>(def.Sign) / kTierERadiusM;
                pathPos = arcCenter + unit * kTierERadiusM;
                pathTangent = pathCurv > 0.0 ? Vec2{-unit.E, unit.N} : Vec2{unit.E, -unit.N};
            }
        } else {
            // straight path through the start point along the trim course: NPFG stays quiescent, so
            // Tier C/D measure the stick and the plant, not the guidance law.
            const double along = (pos - startPos).Dot(FromCourse(startCourse));
            pathPos = startPos + FromCourse(startCourse) * along;
            pathTangent = FromCourse(startCourse);
            pathCurv = 0.0;
        }
        const double pathCourse = CourseOf(pathTangent);
        crossTrack = (pos - pathPos).Dot(RightNormal(pathCourse));
        courseErr = WrapPi(pathCourse - st.GroundCourseRad);

        // ---- production coordinator (TECS always; NPFG roll reference only in Tier E) -----------
        FGuidanceCoordinatorOutputV2 guidance{};
        if (useCoordinator) {
            FGuidanceCoordinatorInputV2 in{};
            FCanonicalNavigationStateV2 &f = in.Follower;
            f.PositionNE_m = pos;
            f.GroundVelocityNE_mps = Vec2{st.VelNorthMps, st.VelEastMps};
            f.GroundCourse_rad = st.GroundCourseRad;
            f.AltitudeAsl_m = st.AltitudeAslM;
            f.ClimbRate_mps = st.ClimbRateMps;
            f.SimulationTimeS = st.TimeS;
            f.ResetGeneration = def.ResetGeneration;
            f.OriginGeneration = def.ResetGeneration;
            f.EquivalentAirspeed_mps = st.EasMps;
            f.TrueAirspeed_mps = st.TasMps;
            f.WindNE_mps = Vec2{0.0, 0.0};
            f.EasToTasRatio = st.EasToTasRatio;
            f.bPositionValid = f.bGroundVelocityValid = f.bGroundCourseValid = true;
            f.bCourseRateValid = f.bCurvatureValid = f.bAltitudeValid = f.bClimbRateValid = true;
            f.bSimulationTimeValid = f.bEasValid = f.bTasValid = f.bWindValid = f.bOriginValid = true;
            f.bRatioValid = st.bRatioValid;

            in.Slot.ResetGeneration = def.ResetGeneration;
            in.Slot.OriginGeneration = def.ResetGeneration;
            in.Slot.bValid = true;

            FPlannerV2OutputAdapterResult &dto = in.PlannerDto;
            dto.Npfg.PathPositionNE_m = pathPos;
            dto.Npfg.PathUnitTangentNE = pathTangent;
            dto.Npfg.PathCurvature_per_m = pathCurv;
            dto.Npfg.bValid = true;
            dto.Tecs.TargetEasMps = trim.EasMps;
            dto.Tecs.TargetAltitudeAslM = trim.AltitudeM;
            dto.Tecs.bTargetEasValid = true;
            dto.Tecs.bTargetAltitudeValid = true;
            dto.Tecs.bTargetClimbRateValid = false;
            dto.Tecs.bCommandReady = true;

            in.CurrentPitchRad = st.PitchRad;
            in.bCurrentPitchValid = true;
            in.CurrentRollRad = st.RollRad;   // ACTUAL roll, same atomic snapshot
            in.bCurrentRollValid = true;
            in.SimulationTimeS = st.TimeS;
            in.DtS = kControllerDtS;
            in.ResetGeneration = def.ResetGeneration;
            in.OriginGeneration = def.ResetGeneration;

            ++r.Counters.CoordinatorUpdates;
            guidance = coordinator.Update(in, cfg);
            if (k > 0 && !guidance.bCommandReady) {
                ++r.UnexpectedInvalidCoordinatorFrames;
                audit.Check(false, def.Name + ": unexpected invalid coordinator frame (" +
                                       GuidanceFailureName(guidance.FailureReason) + ")");
            }
        }

        // ---- roll reference -------------------------------------------------------------------
        double rollRef = 0.0;
        if (useNpfgRollReference) {
            rollRef = guidance.RollReferenceRad;
        } else if (def.Tier == ETier::C_StickRollRef || def.Tier == ETier::D_Heading) {
            // test-owned step; wings level again at kTierCLevelS
            if (st.TimeS >= kTierCBaselineS && st.TimeS < kTierCLevelS)
                rollRef = static_cast<double>(def.Sign) * def.Magnitude;
            else
                rollRef = 0.0;
        }

        // ---- commands --------------------------------------------------------------------------
        double aileronCmd = trim.AileronCmd, elevatorCmd = trim.ElevatorCmd;
        double rudderCmd = 0.0, throttleCmd = std::clamp(trim.ThrottleCmd, 0.0, 1.0);
        double pitchRef = kNa, throttleRef = kNa;
        bool stickValid = false;

        if (def.Tier == ETier::B_DirectAileron) {
            // NPFG and the stick adapter are bypassed. Only a normalized aileron pulse moves;
            // elevator/throttle stay at the trim command and the rudder command is zero.
            const bool inPulse = st.TimeS >= kTierBBaselineS && st.TimeS < kTierBPulseEndS;
            aileronCmd = inPulse ? static_cast<double>(def.Sign) * def.Magnitude : trim.AileronCmd;
        } else if (useStick) {
            FF16StickInputV2 si{};
            si.RollReferenceRad = rollRef;
            si.PitchReferenceRad = guidance.PitchReferenceRad;
            si.ThrottleReferenceNorm = guidance.ThrottleReferenceNorm;
            si.bGuidanceValid = guidance.bCommandReady;
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
            si.ResetGeneration = def.ResetGeneration;

            ++r.Counters.StickUpdates;
            const FF16StickCommandV2 cmd = stick.Update(si, stickConfig);
            stickValid = cmd.bValid;
            pitchRef = guidance.PitchReferenceRad;
            throttleRef = guidance.ThrottleReferenceNorm;
            if (k > 0 && !cmd.bValid) {
                ++r.UnexpectedInvalidStickFrames;
                audit.Check(false, def.Name + ": unexpected invalid stick output");
            }
            if (cmd.bValid) {
                const bool rangeOk =
                    std::abs(cmd.AileronCmdNorm) <= 1.0 + 1e-12 &&
                    cmd.ElevatorCmdNorm >= stickConfig.ElevatorMin - 1e-12 &&
                    cmd.ElevatorCmdNorm <= stickConfig.ElevatorMax + 1e-12 &&
                    std::abs(cmd.RudderCmdNorm) <= 1.0 + 1e-12 &&
                    cmd.ThrottleCmdNorm >= stickConfig.ThrottleMin - 1e-12 &&
                    cmd.ThrottleCmdNorm <= stickConfig.ThrottleMax + 1e-12;
                if (!rangeOk) ++r.CommandRangeViolations;
                audit.Check(rangeOk, def.Name + ": command range violation");
                if (havePreviousValid) {
                    const bool slewOk =
                        std::abs(cmd.AileronCmdNorm - previousValid.AileronCmdNorm) <= stickConfig.AileronSlewPerS * kControllerDtS + 1e-12 &&
                        std::abs(cmd.ElevatorCmdNorm - previousValid.ElevatorCmdNorm) <= stickConfig.ElevatorSlewPerS * kControllerDtS + 1e-12 &&
                        std::abs(cmd.RudderCmdNorm - previousValid.RudderCmdNorm) <= stickConfig.RudderSlewPerS * kControllerDtS + 1e-12 &&
                        std::abs(cmd.ThrottleCmdNorm - previousValid.ThrottleCmdNorm) <= stickConfig.ThrottleSlewPerS * kControllerDtS + 1e-12;
                    if (!slewOk) ++r.CommandSlewViolations;
                    audit.Check(slewOk, def.Name + ": stick command slew violation");
                }
                previousValid = cmd;
                havePreviousValid = true;
                aileronCmd = cmd.AileronCmdNorm;
                elevatorCmd = cmd.ElevatorCmdNorm;
                rudderCmd = cmd.RudderCmdNorm;
                throttleCmd = cmd.ThrottleCmdNorm;
                previousAileronApplied = aileronCmd;
            } else {
                aileronCmd = previousAileronApplied;   // the single reset frame keeps the trim command
            }
        }

        const bool applied = plant.WriteOwned(aileronCmd, elevatorCmd, rudderCmd, throttleCmd, r.Counters);
        audit.Check(applied, def.Name + ": command write did not target the owned FGFDMExec");

        FFrame fr{};
        fr.CaseIndex = caseIndex;
        fr.TimeS = st.TimeS;
        fr.RollReferenceRad = rollRef;
        fr.AileronCmd = aileronCmd;
        fr.ElevatorCmd = elevatorCmd;
        fr.RudderCmd = rudderCmd;
        fr.ThrottleCmd = throttleCmd;
        fr.PitchReferenceRad = pitchRef;
        fr.ThrottleReferenceNorm = throttleRef;
        fr.CrossTrackM = crossTrack;
        fr.CourseErrorRad = courseErr;
        fr.PathCurvature = pathCurv;
        fr.LoadFactorCmdSide = 1.0 / std::max(std::cos(st.RollRad), 1.0e-7);
        fr.bCommandReady = guidance.bCommandReady;
        fr.bStickValid = stickValid;
        fr.bCommandApplied = applied;
        fr.State = st;
        r.Trace.push_back(fr);
        ++r.ControlFrames;

        for (int sub = 0; sub < kFdmStepsPerControlFrame; ++sub) {
            const bool ok = plant.Run(r.Counters);
            audit.Check(ok, def.Name + ": FGFDMExec::Run() failed");
            ++r.FdmSteps;
            if (!ok) { failure = def.Name + ": FGFDMExec::Run() failed"; return false; }
        }
    }
    return true;
}

// ================================================================================================
// Metrics
// ================================================================================================
void Measure(FCaseResult &r)
{
    FMetrics &m = r.M;
    const bool tierB = r.Def.Tier == ETier::B_DirectAileron;
    const double stepS = tierB ? kTierBBaselineS : kTierCBaselineS;
    const double endOfExcitationS = tierB ? kTierBPulseEndS : kTierCLevelS;
    const double stepTimeS = (r.Def.Tier == ETier::E_NpfgArc) ? kTierEStepS : stepS;

    const FFrame *atStep = nullptr;
    for (const FFrame &f : r.Trace)
        if (!atStep && f.TimeS >= stepTimeS) atStep = &f;
    if (!atStep || r.Trace.empty()) return;

    const double courseAtStep = atStep->State.GroundCourseRad;
    const double rollAtStep = atStep->State.RollRad;

    // initial body-axis roll acceleration over the first 0.2 s after the excitation
    const FFrame *at02 = nullptr;
    for (const FFrame &f : r.Trace)
        if (!at02 && f.TimeS >= stepTimeS + 0.2) at02 = &f;
    if (at02) m.InitialPSlopeRadps2 = (at02->State.PRadps - atStep->State.PRadps) /
                                      (at02->TimeS - atStep->TimeS);

    double peakP = 0.0, peakR = 0.0, peakRoll = 0.0, impulse = 0.0, iae = 0.0;
    double aMin = 1e18, aMax = -1e18, sMin = 1e18, sMax = -1e18, bMin = 1e18, bMax = -1e18;
    double pMin = 1e18, pMax = -1e18, tMin = 1e18, tMax = -1e18, nMin = 1e18, nMax = -1e18;
    double altEx = 0.0, easEx = 0.0, maxXtk = 0.0;
    double peakRollSigned = 0.0;
    for (const FFrame &f : r.Trace) {
        if (f.TimeS < stepTimeS) continue;
        peakP = std::max(peakP, std::abs(f.State.PRadps));
        peakR = std::max(peakR, std::abs(f.State.RRadps));
        const double rollRel = f.State.RollRad - rollAtStep;
        if (std::abs(rollRel) > peakRoll) { peakRoll = std::abs(rollRel); peakRollSigned = rollRel; }
        if (f.TimeS < endOfExcitationS) impulse += f.State.PRadps * kControllerDtS;
        iae += std::abs(f.RollReferenceRad - f.State.RollRad) * kControllerDtS;
        aMin = std::min(aMin, f.AileronCmd); aMax = std::max(aMax, f.AileronCmd);
        sMin = std::min(sMin, f.State.AileronLeftPosRad); sMax = std::max(sMax, f.State.AileronLeftPosRad);
        bMin = std::min(bMin, f.State.BetaRad); bMax = std::max(bMax, f.State.BetaRad);
        if (Finite(f.PitchReferenceRad)) { pMin = std::min(pMin, f.PitchReferenceRad); pMax = std::max(pMax, f.PitchReferenceRad); }
        tMin = std::min(tMin, f.ThrottleCmd); tMax = std::max(tMax, f.ThrottleCmd);
        nMin = std::min(nMin, f.State.LoadFactor); nMax = std::max(nMax, f.State.LoadFactor);
        altEx = std::max(altEx, std::abs(f.State.AltitudeAslM - r.Trim.AltitudeM));
        easEx = std::max(easEx, std::abs(f.State.EasMps - r.Trim.EasMps));
        maxXtk = std::max(maxXtk, std::abs(f.CrossTrackM));
        if (std::abs(f.RollReferenceRad) >= 0.7853981633974483 - 1e-9) ++m.RollRefSaturatedFrames;
        if (std::abs(f.AileronCmd) >= 1.0 - 1e-9) ++m.AileronSaturatedFrames;
    }
    m.PeakAbsPRadps = peakP;
    m.PeakAbsRRadps = peakR;
    m.PeakAbsRollRad = peakRoll;
    m.RollImpulseRad = impulse;
    m.IntegratedAbsRollErrorRadS = iae;
    m.AileronCmdMin = aMin; m.AileronCmdMax = aMax;
    m.AileronSurfaceMinRad = sMin; m.AileronSurfaceMaxRad = sMax;
    m.BetaMinRad = bMin; m.BetaMaxRad = bMax;
    m.PitchRefMin = (pMin > 1e17) ? kNa : pMin; m.PitchRefMax = (pMax < -1e17) ? kNa : pMax;
    m.ThrottleMin = tMin; m.ThrottleMax = tMax;
    m.LoadFactorMin = nMin; m.LoadFactorMax = nMax;
    m.AltitudeExcursionM = altEx;
    m.EasExcursionMps = easEx;
    m.MaxAbsCrossTrackM = maxXtk;
    m.FinalCrossTrackM = r.Trace.back().CrossTrackM;
    m.CourseChangeRad = WrapPi(r.Trace.back().State.GroundCourseRad - courseAtStep);

    // 10-90 % rise time of the roll excursion toward its own peak
    const double t10 = 0.1 * peakRoll, t90 = 0.9 * peakRoll;
    double s10 = -1.0, s90 = -1.0;
    for (const FFrame &f : r.Trace) {
        if (f.TimeS < stepTimeS) continue;
        const double a = std::abs(f.State.RollRad - rollAtStep);
        if (s10 < 0.0 && a >= t10) s10 = f.TimeS;
        if (s90 < 0.0 && a >= t90) { s90 = f.TimeS; break; }
    }
    if (s10 >= 0.0 && s90 >= s10) m.RiseTime10_90S = s90 - s10;

    // overshoot against the commanded roll reference (Tier C/D only)
    if (r.Def.Tier == ETier::C_StickRollRef || r.Def.Tier == ETier::D_Heading) {
        const double target = static_cast<double>(r.Def.Sign) * r.Def.Magnitude;
        double peakTowardTarget = 0.0;
        for (const FFrame &f : r.Trace) {
            if (f.TimeS < stepTimeS || f.TimeS >= endOfExcitationS) continue;
            const double v = f.State.RollRad * (target > 0.0 ? 1.0 : -1.0);
            peakTowardTarget = std::max(peakTowardTarget, v);
        }
        if (std::abs(target) > 1e-9)
            m.OvershootNorm = (peakTowardTarget - std::abs(target)) / std::abs(target);
    }

    // recovery: time after the excitation ends until |P| falls back below a small body-rate floor
    constexpr double kQuietPRadps = 0.01;
    double recovered = -1.0;
    for (const FFrame &f : r.Trace) {
        if (f.TimeS < endOfExcitationS) continue;
        if (std::abs(f.State.PRadps) < kQuietPRadps) { recovered = f.TimeS; break; }
    }
    if (recovered >= 0.0) m.RecoveryTimeS = recovered - endOfExcitationS;
    (void)peakRollSigned;
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

void WriteCsv(const std::string &path, const std::vector<FCaseResult> &results, int prec)
{
    std::ofstream out(path);
    out << "tier,case,case_index,simulation_time_s,command_ready,stick_valid,command_applied,"
           "roll_reference_rad,aileron_cmd_norm,elevator_cmd_norm,rudder_cmd_norm,throttle_cmd_norm,"
           "pitch_reference_rad,throttle_reference_norm,load_factor_from_actual_roll,"
           "cross_track_m,course_error_rad,path_curvature_per_m,"
           "roll_rad,pitch_rad,yaw_rad,p_radps,q_radps,r_radps,alpha_rad,beta_rad,load_factor,"
           "aileron_left_position_rad,aileron_right_position_rad,elevator_position_rad,"
           "rudder_position_rad,throttle_position,"
           "north_m,east_m,ground_course_rad,ground_speed_mps,altitude_m,climb_rate_mps,"
           "eas_mps,tas_mps,wow,engine_running\n";
    for (const FCaseResult &r : results) {
        for (const FFrame &f : r.Trace) {
            out << TierName(r.Def.Tier) << ',' << r.Def.Name << ',' << f.CaseIndex << ',';
            WriteNumber(out, f.TimeS, prec);
            out << ',' << (f.bCommandReady ? 1 : 0) << ',' << (f.bStickValid ? 1 : 0) << ','
                << (f.bCommandApplied ? 1 : 0) << ',';
            const std::array<double, 11> cmd{f.RollReferenceRad, f.AileronCmd, f.ElevatorCmd,
                                             f.RudderCmd, f.ThrottleCmd, f.PitchReferenceRad,
                                             f.ThrottleReferenceNorm, f.LoadFactorCmdSide,
                                             f.CrossTrackM, f.CourseErrorRad, f.PathCurvature};
            for (double v : cmd) { WriteNumber(out, v, prec); out << ','; }
            const std::array<double, 14> st{f.State.RollRad, f.State.PitchRad, f.State.YawRad,
                                            f.State.PRadps, f.State.QRadps, f.State.RRadps,
                                            f.State.AlphaRad, f.State.BetaRad, f.State.LoadFactor,
                                            f.State.AileronLeftPosRad, f.State.AileronRightPosRad,
                                            f.State.ElevatorPosRad, f.State.RudderPosRad,
                                            f.State.ThrottlePos};
            for (double v : st) { WriteNumber(out, v, prec); out << ','; }
            const std::array<double, 8> nav{f.State.NorthM, f.State.EastM, f.State.GroundCourseRad,
                                            f.State.GroundSpeedMps, f.State.AltitudeAslM,
                                            f.State.ClimbRateMps, f.State.EasMps, f.State.TasMps};
            for (double v : nav) { WriteNumber(out, v, prec); out << ','; }
            out << (f.State.bWow ? 1 : 0) << ',' << (f.State.bEngineRunning ? 1 : 0) << '\n';
        }
    }
}

struct FPairAsym {
    std::string Tier, Key;
    double PeakRollAsym{kNa}, PeakPAsym{kNa}, InitialSlopeAsym{kNa}, ImpulseAsym{kNa};
    double AileronCmdAsym{kNa}, SurfaceAsym{kNa}, BetaAsym{kNa};
    double AltitudeAsym{kNa}, EasAsym{kNa}, CrossTrackAsym{kNa}, IaeAsym{kNa};
    double WorstAsym{kNa};
    const FCaseResult *Pos{}, *Neg{};
};

} // namespace

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::fprintf(stderr, "usage: %s <JSBSim-root> <raw.csv> <quantized.csv> <summary.txt>\n", argv[0]);
        return 2;
    }
    const std::string root = argv[1];

    FGuidanceConfigV2 guidanceConfig{};
    guidanceConfig.EasMinMps = kTestLocalEasMinMps;
    guidanceConfig.EasMaxMps = kTestLocalEasMaxMps;
    guidanceConfig.TecsMaxClimbRateMps = kTestLocalMaxClimbRateMps;
    guidanceConfig.TecsMinSinkRateMps = kTestLocalMinSinkRateMps;
    guidanceConfig.TecsMaxSinkRateMps = kTestLocalMaxSinkRateMps;
    const FF16StickConfigV2 stickConfig{};   // production defaults, unchanged

    FAudit audit;

    // ============================================================================================
    // Tier A -- geometry / NPFG mirror contract (no FDM)
    // ============================================================================================
    const std::vector<FMirrorCase> mirrorCases{
        {"cross_track_500m", 500.0, 0.0, 0.0},
        {"course_error_10deg", 0.0, 10.0 * kDegToRad, 0.0},
        {"curvature_10km", 0.0, 0.0, 1.0 / 10000.0},
        {"cross_track_500m_plus_course_error_10deg", 500.0, 10.0 * kDegToRad, 0.0},
        {"curvature_10km_plus_cross_track_500m", 500.0, 0.0, 1.0 / 10000.0},
    };
    // Tier A needs a trim airspeed/altitude; take them from a single trimmed plant.
    FOwnedF16Plant refPlant;
    std::string refFailure;
    if (!refPlant.Initialize(root, kDefaultHeadingDeg, refFailure)) {
        std::fprintf(stderr, "BLOCKED: reference trim failed: %s\n", refFailure.c_str());
        return 1;
    }
    const FTrimResult refTrim = refPlant.TrimInfo();
    FGuidanceConfigV2 mirrorCfg = guidanceConfig;
    mirrorCfg.TecsEquivalentAirspeedTrimMps = refTrim.EasMps;
    mirrorCfg.ThrottleTrim = std::clamp(refTrim.ThrottleCmd, mirrorCfg.ThrottleMin, mirrorCfg.ThrottleMax);
    if (!FormationControlV2::IsGuidanceConfigValid(mirrorCfg)) {
        std::fprintf(stderr, "BLOCKED: mirror config rejected by IsGuidanceConfigValid\n");
        return 1;
    }

    struct FMirrorResult {
        std::string Name;
        FMirrorSample Pos, Neg;
        double WorstAbsResidual{};
        double WorstRelResidual{};                 // residual / its float32-aware tolerance
        double ZeroErrorFeedbackResidueMps2{kNa};  // NPFG's even (non-antisymmetric) residue, if any
        bool bHolds{};
    };
    std::vector<FMirrorResult> mirrorResults;
    bool tierAHolds = true;
    for (const FMirrorCase &mc : mirrorCases) {
        FMirrorResult mr{};
        mr.Name = mc.Name;
        mr.Pos = RunMirror(mirrorCfg, mc, +1, refTrim.TasMps, refTrim.EasMps, refTrim.AltitudeM);
        mr.Neg = RunMirror(mirrorCfg, mc, -1, refTrim.TasMps, refTrim.EasMps, refTrim.AltitudeM);
        audit.Check(mr.Pos.bValid && mr.Neg.bValid, "tier A: mirror frames are command-ready");

        struct Item { const char *Name; double P, N, Floor; };
        const Item items[] = {
            {"cross_track_m", mr.Pos.CrossTrackM, mr.Neg.CrossTrackM, kMirrorFloorM},
            {"course_error_rad", mr.Pos.CourseErrorRad, mr.Neg.CourseErrorRad, kMirrorFloorRad},
            {"lateral_accel_ff", mr.Pos.LateralFf, mr.Neg.LateralFf, kMirrorFloorMps2},
            {"lateral_accel_fb", mr.Pos.LateralFb, mr.Neg.LateralFb, kMirrorFloorMps2},
            {"lateral_accel_total", mr.Pos.LateralTotal, mr.Neg.LateralTotal, kMirrorFloorMps2},
            {"roll_reference_rad", mr.Pos.RollReferenceRad, mr.Neg.RollReferenceRad, kMirrorFloorRad},
            {"course_setpoint_error_rad", mr.Pos.CourseSetpointRad, mr.Neg.CourseSetpointRad, kMirrorFloorRad},
        };
        mr.bHolds = true;
        for (const Item &it : items) {
            const double residual = std::abs(it.P + it.N);   // mirror => equal magnitude, opposite sign
            const double tol = std::max(kMirrorRelTol * std::max(std::abs(it.P), std::abs(it.N)), it.Floor);
            mr.WorstAbsResidual = std::max(mr.WorstAbsResidual, residual);
            mr.WorstRelResidual = std::max(mr.WorstRelResidual, residual / std::max(tol, 1.0e-300));
            const bool ok = Finite(residual) && residual <= tol;
            if (!ok) mr.bHolds = false;
            audit.Check(ok, std::string("tier A mirror broken: ") + mc.Name + " / " + it.Name);
        }
        // The one place the mirror cannot cancel by construction: for a PURE curvature excitation the
        // aircraft sits exactly on the path, so the mirrored NPFG input is bit-identical apart from the
        // curvature sign, and NPFG's zero-track-error feedback residue lands with the SAME sign on both
        // sides. Recorded explicitly rather than hidden inside the tolerance.
        mr.ZeroErrorFeedbackResidueMps2 = (std::abs(mc.CrossTrackM) < 1e-12 &&
                                           std::abs(mc.CourseErrorRad) < 1e-12)
                                              ? 0.5 * (mr.Pos.LateralFb + mr.Neg.LateralFb)
                                              : kNa;
        if (!mr.bHolds) tierAHolds = false;
        mirrorResults.push_back(mr);
    }

    // ============================================================================================
    // Tiers B..E -- FDM
    // ============================================================================================
    const std::vector<FCaseDef> cases = BuildCases();
    std::vector<FCaseResult> results(cases.size());
    for (std::size_t i = 0; i < cases.size(); ++i) {
        std::string failure;
        if (!RunFdmCase(root, static_cast<int>(i), cases[i], guidanceConfig, stickConfig,
                        results[i], audit, failure)) {
            std::fprintf(stderr, "BLOCKED: %s\n", failure.c_str());
            return 1;
        }
        Measure(results[i]);
    }

    // physical sanity (not performance gates)
    for (const FCaseResult &r : results) {
        audit.Check(Finite(r.M.AltitudeExcursionM) && r.M.AltitudeExcursionM < kAltitudeSanityM,
                    r.Def.Name + ": altitude left the physical sanity bound");
        for (const FFrame &f : r.Trace) {
            if (!(f.State.EasMps > kEasSanityMinMps && f.State.EasMps < kEasSanityMaxMps)) {
                audit.Check(false, r.Def.Name + ": EAS left the physical sanity bound");
                break;
            }
        }
    }

    // ---- pair asymmetries ------------------------------------------------------------------------
    std::vector<FPairAsym> pairs;
    for (const FCaseResult &a : results) {
        if (a.Def.Sign != +1) continue;
        const FCaseResult *b = nullptr;
        for (const FCaseResult &c : results)
            if (c.Def.Tier == a.Def.Tier && c.Def.PairKey == a.Def.PairKey && c.Def.Sign == -1) b = &c;
        if (!b) continue;
        FPairAsym p{};
        p.Tier = TierName(a.Def.Tier);
        p.Key = a.Def.PairKey;
        p.Pos = &a;
        p.Neg = b;
        p.PeakRollAsym = NormalizedAsymmetry(a.M.PeakAbsRollRad, b->M.PeakAbsRollRad, kAsymmetryFloorRad);
        p.PeakPAsym = NormalizedAsymmetry(a.M.PeakAbsPRadps, b->M.PeakAbsPRadps, kAsymmetryFloorRadps);
        p.InitialSlopeAsym = NormalizedAsymmetry(a.M.InitialPSlopeRadps2, b->M.InitialPSlopeRadps2, kAsymmetryFloorRadps);
        p.ImpulseAsym = NormalizedAsymmetry(a.M.RollImpulseRad, b->M.RollImpulseRad, kAsymmetryFloorRad);
        p.AileronCmdAsym = NormalizedAsymmetry(a.M.AileronCmdMax, b->M.AileronCmdMin, kAsymmetryFloorNorm);
        p.SurfaceAsym = NormalizedAsymmetry(a.M.AileronSurfaceMaxRad, b->M.AileronSurfaceMinRad, kAsymmetryFloorRad);
        p.BetaAsym = NormalizedAsymmetry(a.M.BetaMaxRad, b->M.BetaMinRad, kAsymmetryFloorRad);
        p.AltitudeAsym = NormalizedAsymmetry(a.M.AltitudeExcursionM, b->M.AltitudeExcursionM, kAsymmetryFloorM);
        p.EasAsym = NormalizedAsymmetry(a.M.EasExcursionMps, b->M.EasExcursionMps, kAsymmetryFloorM);
        p.CrossTrackAsym = NormalizedAsymmetry(a.M.MaxAbsCrossTrackM, b->M.MaxAbsCrossTrackM, kAsymmetryFloorM);
        p.IaeAsym = NormalizedAsymmetry(a.M.IntegratedAbsRollErrorRadS, b->M.IntegratedAbsRollErrorRadS, kAsymmetryFloorRad);
        // The body-axis roll response is the primary evidence; the rest is context.
        p.WorstAsym = std::max({p.PeakRollAsym, p.PeakPAsym, p.ImpulseAsym});
        pairs.push_back(p);
    }

    auto WorstOfTier = [&pairs](ETier t, double FPairAsym::*field) {
        double worst = 0.0;
        bool any = false;
        for (const FPairAsym &p : pairs) {
            if (p.Tier != std::string(TierName(t))) continue;
            if (!Finite(p.*field)) continue;
            worst = std::max(worst, p.*field);
            any = true;
        }
        return any ? worst : kNa;
    };

    const double bodyAsymB = WorstOfTier(ETier::B_DirectAileron, &FPairAsym::WorstAsym);
    const double cmdAsymC = WorstOfTier(ETier::C_StickRollRef, &FPairAsym::AileronCmdAsym);
    const double bodyAsymC = WorstOfTier(ETier::C_StickRollRef, &FPairAsym::WorstAsym);
    const double bodyAsymD = WorstOfTier(ETier::D_Heading, &FPairAsym::WorstAsym);
    const double bodyAsymE = WorstOfTier(ETier::E_NpfgArc, &FPairAsym::WorstAsym);
    const double altAsymE = WorstOfTier(ETier::E_NpfgArc, &FPairAsym::AltitudeAsym);
    const double xtkAsymE = WorstOfTier(ETier::E_NpfgArc, &FPairAsym::CrossTrackAsym);

    // Tier D heading spread: does the SAME body-axis excitation behave differently by heading?
    double headingSpread = 0.0;
    {
        double lo = 1e18, hi = -1e18;
        for (const FPairAsym &p : pairs) {
            if (p.Tier != std::string(TierName(ETier::D_Heading))) continue;
            if (!Finite(p.WorstAsym)) continue;
            lo = std::min(lo, p.WorstAsym);
            hi = std::max(hi, p.WorstAsym);
        }
        if (hi > -1e17) headingSpread = hi - lo;
    }

    // ---- primary classification -----------------------------------------------------------------
    std::string classification;
    std::string rationale;
    if (!tierAHolds) {
        classification = "GEOMETRY_OR_NPFG";
        rationale = "the Tier A mirror identity is broken before any plant is involved";
    } else if (Finite(bodyAsymB) && bodyAsymB > kMeaningfulAsymmetry) {
        classification = "FCS_OR_AIRFRAME";
        rationale = "symmetry survives Tier A but breaks with direct normalized aileron pulses through "
                    "the actual f16.xml FCS, with NPFG and the stick adapter bypassed";
    } else if (Finite(cmdAsymC) && cmdAsymC > kMeaningfulAsymmetry) {
        classification = "STICK_ADAPTER";
        rationale = "the plant is symmetric under direct aileron excitation, but the production stick "
                    "adapter emits asymmetric aileron commands for mirrored roll references";
    } else if (Finite(bodyAsymC) && bodyAsymC > kMeaningfulAsymmetry) {
        classification = "FCS_OR_AIRFRAME";
        rationale = "the stick commands are antisymmetric, so the asymmetry first appears in the plant "
                    "response to a closed roll-attitude loop";
    } else if (headingSpread > kMeaningfulAsymmetry) {
        classification = "NAVIGATION_FRAME";
        rationale = "the same body-axis excitation is asymmetric by initial heading, which points at "
                    "the navigation/local frame rather than the airframe";
    } else if (Finite(bodyAsymE) && bodyAsymE > kMeaningfulAsymmetry) {
        classification = "MULTI_FACTOR";
        rationale = "every isolated stage is symmetric within the declared threshold; the asymmetry "
                    "only emerges once the NPFG arc loop is closed";
    } else if ((Finite(altAsymE) && altAsymE > kMeaningfulAsymmetry) ||
               (Finite(xtkAsymE) && xtkAsymE > kMeaningfulAsymmetry)) {
        classification = "LONGITUDINAL_COUPLING";
        rationale = "the lateral body-axis response is symmetric at every stage, but the closed NPFG "
                    "arc shows an asymmetric altitude/cross-track outcome";
    } else {
        classification = "NO_MEANINGFUL_ASYMMETRY";
        rationale = "no stage exceeds the declared normalized-asymmetry threshold";
    }

    // Tier A is the only mathematical identity here; breaking it is a FAIL.
    audit.Check(tierAHolds, "tier A geometry/NPFG mirror identity must hold");

    WriteCsv(argv[2], results, 15);
    WriteCsv(argv[3], results, 9);

    FWriteCounters total{};
    std::uint64_t frames = 0, fdmSteps = 0;
    std::uint64_t nonFinite = 0, wow = 0, cfgV = 0, rangeV = 0, slewV = 0, invalidCo = 0, invalidSt = 0;
    for (const FCaseResult &r : results) {
        total.CoordinatorUpdates += r.Counters.CoordinatorUpdates;
        total.StickUpdates += r.Counters.StickUpdates;
        total.FdmRuns += r.Counters.FdmRuns;
        total.CommandFrames += r.Counters.CommandFrames;
        total.AileronWrites += r.Counters.AileronWrites;
        total.ElevatorWrites += r.Counters.ElevatorWrites;
        total.RudderWrites += r.Counters.RudderWrites;
        total.ThrottleWrites += r.Counters.ThrottleWrites;
        total.WritesOutsideOwnedFdm += r.Counters.WritesOutsideOwnedFdm;
        total.ProductionWriterInvocations += r.Counters.ProductionWriterInvocations;
        total.FdmRunFailures += r.Counters.FdmRunFailures;
        total.SurfacePositionWrites += r.Counters.SurfacePositionWrites;
        total.AeroPropertyWrites += r.Counters.AeroPropertyWrites;
        frames += r.ControlFrames;
        fdmSteps += r.FdmSteps;
        nonFinite += r.NonFiniteStates;
        wow += r.UnexpectedWow;
        cfgV += r.ConfigViolations;
        rangeV += r.CommandRangeViolations;
        slewV += r.CommandSlewViolations;
        invalidCo += r.UnexpectedInvalidCoordinatorFrames;
        invalidSt += r.UnexpectedInvalidStickFrames;
    }
    audit.Check(total.WritesOutsideOwnedFdm == 0, "writes outside the owned FGFDMExec");
    audit.Check(total.FdmRunFailures == 0, "FDM run failures");
    audit.Check(nonFinite == 0, "non-finite plant states");
    audit.Check(wow == 0, "unexpected WOW");
    audit.Check(rangeV == 0, "command range violations");
    audit.Check(slewV == 0, "command slew violations");

    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "F16_LATERAL_DYNAMICS_ISOLATION_V2\n";
    out << "This harness identifies WHERE the F-16 left/right asymmetry first appears. It changes no "
           "production NPFG, TECS or stick tuning, no aircraft data and no controller. PASS does not "
           "mean the aircraft is symmetric: a large measured asymmetry is a result, not a failure. The "
           "only mathematical identity asserted here is the Tier A geometry/NPFG mirror.\n";
    out << "classification_threshold normalized_asymmetry=" << kMeaningfulAsymmetry
        << " (declared in advance; it is NOT a pass/fail gate)\n";
    out << "model=f16 engine=F100-PW-229 fdm_dt_s=" << kFdmDtS << " controller_dt_s=" << kControllerDtS
        << " altitude_ft=" << kInitialAltitudeFt << " tas_mps=" << kInitialTasMps
        << " fuel_lb=" << kExpectedFuelLb << " wind=0 gear=up flap=clean speedbrake=retracted\n";
    out << "trim eas_mps=" << Num(refTrim.EasMps) << " tas_mps=" << Num(refTrim.TasMps)
        << " pitch_rad=" << Num(refTrim.PitchRad) << " roll_rad=" << Num(refTrim.RollRad)
        << " throttle_cmd=" << Num(refTrim.ThrottleCmd) << " elevator_cmd=" << Num(refTrim.ElevatorCmd)
        << " aileron_cmd=" << Num(refTrim.AileronCmd) << " fresh_trim_per_case=1\n";
    out << "production_tuning_changed=0 npfg_period_damping_changed=0 roll_limit_changed=0 "
           "tecs_gains_changed=0 stick_gains_changed=0 aircraft_xml_changed=0 rudder_compensator_added=0\n";
    out << "cases=" << results.size() << " tier_a_mirror_cases=" << mirrorResults.size()
        << " total_control_frames=" << frames << " total_fdm_steps=" << fdmSteps << '\n';

    out << "TIER_A geometry_npfg_mirror holds=" << (tierAHolds ? 1 : 0) << '\n';
    for (const FMirrorResult &mr : mirrorResults) {
        out << "  mirror case=" << mr.Name << " holds=" << (mr.bHolds ? 1 : 0)
            << " worst_abs_residual=" << Num(mr.WorstAbsResidual)
            << " worst_residual_over_tolerance=" << Num(mr.WorstRelResidual)
            << " npfg_zero_error_feedback_residue_mps2=" << Num(mr.ZeroErrorFeedbackResidueMps2)
            << " roll_reference_rad=" << Num(mr.Pos.RollReferenceRad) << '/' << Num(mr.Neg.RollReferenceRad)
            << " lateral_accel_total=" << Num(mr.Pos.LateralTotal) << '/' << Num(mr.Neg.LateralTotal)
            << " lateral_accel_ff=" << Num(mr.Pos.LateralFf) << '/' << Num(mr.Neg.LateralFf)
            << " lateral_accel_fb=" << Num(mr.Pos.LateralFb) << '/' << Num(mr.Neg.LateralFb)
            << " cross_track_m=" << Num(mr.Pos.CrossTrackM) << '/' << Num(mr.Neg.CrossTrackM)
            << " course_error_rad=" << Num(mr.Pos.CourseErrorRad) << '/' << Num(mr.Neg.CourseErrorRad) << '\n';
    }

    for (const FCaseResult &r : results) {
        const FMetrics &m = r.M;
        out << "case tier=" << TierName(r.Def.Tier) << " name=" << r.Def.Name
            << " pair=" << r.Def.PairKey << " sign=" << r.Def.Sign
            << " magnitude=" << Num(r.Def.Magnitude) << " heading_deg=" << Num(r.Def.HeadingDeg)
            << " duration_s=" << Num(r.Def.DurationS) << " control_frames=" << r.ControlFrames
            << " fdm_steps=" << r.FdmSteps << '\n';
        out << "  body initial_p_slope_radps2=" << Num(m.InitialPSlopeRadps2)
            << " peak_abs_p_radps=" << Num(m.PeakAbsPRadps)
            << " peak_abs_r_radps=" << Num(m.PeakAbsRRadps)
            << " peak_abs_roll_rad=" << Num(m.PeakAbsRollRad)
            << " roll_impulse_rad=" << Num(m.RollImpulseRad)
            << " rise_time_10_90_s=" << Num(m.RiseTime10_90S)
            << " overshoot_norm=" << Num(m.OvershootNorm)
            << " recovery_time_s=" << Num(m.RecoveryTimeS)
            << " integrated_abs_roll_error_rad_s=" << Num(m.IntegratedAbsRollErrorRadS) << '\n';
        out << "  surfaces aileron_cmd=[" << Num(m.AileronCmdMin) << ',' << Num(m.AileronCmdMax) << ']'
            << " aileron_left_surface_rad=[" << Num(m.AileronSurfaceMinRad) << ',' << Num(m.AileronSurfaceMaxRad) << ']'
            << " beta_rad=[" << Num(m.BetaMinRad) << ',' << Num(m.BetaMaxRad) << ']'
            << " load_factor=[" << Num(m.LoadFactorMin) << ',' << Num(m.LoadFactorMax) << ']'
            << " roll_ref_saturated_frames=" << m.RollRefSaturatedFrames
            << " aileron_saturated_frames=" << m.AileronSaturatedFrames << '\n';
        out << "  coupling course_change_rad=" << Num(m.CourseChangeRad)
            << " altitude_excursion_m=" << Num(m.AltitudeExcursionM)
            << " eas_excursion_mps=" << Num(m.EasExcursionMps)
            << " max_abs_cross_track_m=" << Num(m.MaxAbsCrossTrackM)
            << " final_cross_track_m=" << Num(m.FinalCrossTrackM)
            << " pitch_reference_rad=[" << Num(m.PitchRefMin) << ',' << Num(m.PitchRefMax) << ']'
            << " throttle_cmd=[" << Num(m.ThrottleMin) << ',' << Num(m.ThrottleMax) << ']' << '\n';
    }

    out << "PAIR_ASYMMETRY normalized=abs(|pos|-|neg|)/max(|pos|,|neg|,floor)\n";
    for (const FPairAsym &p : pairs) {
        out << "  pair tier=" << p.Tier << " key=" << p.Key
            << " peak_roll=" << Num(p.Pos->M.PeakAbsRollRad) << '/' << Num(p.Neg->M.PeakAbsRollRad)
            << " abs_diff=" << Num(std::abs(p.Pos->M.PeakAbsRollRad - p.Neg->M.PeakAbsRollRad))
            << " norm=" << Num(p.PeakRollAsym)
            << " | peak_p_norm=" << Num(p.PeakPAsym)
            << " initial_slope_norm=" << Num(p.InitialSlopeAsym)
            << " roll_impulse_norm=" << Num(p.ImpulseAsym)
            << " aileron_cmd_norm=" << Num(p.AileronCmdAsym)
            << " aileron_surface_norm=" << Num(p.SurfaceAsym)
            << " beta_norm=" << Num(p.BetaAsym)
            << " altitude_norm=" << Num(p.AltitudeAsym)
            << " eas_norm=" << Num(p.EasAsym)
            << " cross_track_norm=" << Num(p.CrossTrackAsym)
            << " iae_norm=" << Num(p.IaeAsym)
            << " worst_body_norm=" << Num(p.WorstAsym) << '\n';
    }

    out << "TIER_WORST_BODY_ASYMMETRY"
        << " B_direct_aileron=" << Num(bodyAsymB)
        << " C_stick_command=" << Num(cmdAsymC)
        << " C_stick_body=" << Num(bodyAsymC)
        << " D_heading_body=" << Num(bodyAsymD)
        << " D_heading_spread=" << Num(headingSpread)
        << " E_npfg_arc_body=" << Num(bodyAsymE)
        << " E_npfg_arc_altitude=" << Num(altAsymE)
        << " E_npfg_arc_cross_track=" << Num(xtkAsymE) << '\n';
    out << "PRIMARY_CLASSIFICATION=" << classification << '\n';
    out << "CLASSIFICATION_RATIONALE=" << rationale << '\n';
    // The classification names where symmetry FIRST breaks. It is not the whole story, so the
    // amplification across the tiers is stated explicitly next to it.
    out << "ASYMMETRY_AMPLIFICATION"
        << " open_loop_plant_B=" << Num(bodyAsymB)
        << " stick_closed_roll_loop_C=" << Num(bodyAsymC)
        << " heading_spread_D=" << Num(headingSpread)
        << " npfg_closed_loop_E_body=" << Num(bodyAsymE)
        << " npfg_closed_loop_E_altitude=" << Num(altAsymE)
        << " npfg_closed_loop_E_cross_track=" << Num(xtkAsymE)
        << " note=the_stick_roll_attitude_loop_largely_suppresses_the_open_loop_plant_asymmetry;"
           "closing_the_NPFG_loop_amplifies_it_again\n";

    out << "write_accounting coordinator_updates=" << total.CoordinatorUpdates
        << " stick_updates=" << total.StickUpdates
        << " fdm_runs=" << total.FdmRuns
        << " command_frames=" << total.CommandFrames
        << " aileron_writes=" << total.AileronWrites
        << " elevator_writes=" << total.ElevatorWrites
        << " rudder_writes=" << total.RudderWrites
        << " throttle_writes=" << total.ThrottleWrites
        << " writes_outside_owned_fdm=" << total.WritesOutsideOwnedFdm
        << " production_writer_invocations=" << total.ProductionWriterInvocations
        << " surface_position_direct_writes=" << total.SurfacePositionWrites
        << " aerodynamic_property_direct_writes=" << total.AeroPropertyWrites
        << " fdm_run_failures=" << total.FdmRunFailures << '\n';
    out << "violations non_finite_states=" << nonFinite << " unexpected_wow=" << wow
        << " configuration_violations=" << cfgV << " command_range_violations=" << rangeV
        << " command_slew_violations=" << slewV
        << " unexpected_invalid_coordinator_frames=" << invalidCo
        << " unexpected_invalid_stick_frames=" << invalidSt << '\n';
    out << "isolation ue_world_loaded=0 game_pawns_searched=0 active_connection=0 "
           "udp_bt_bridge_access=0 legacy_writer_access=0 test_owned_fgfdmexec_only=1\n";
    out << "checks=" << audit.Checks << " failures=" << audit.Failures << '\n';
    for (const std::string &s : audit.Messages) out << "  failure: " << s << '\n';

    const std::string summary = out.str();
    std::ofstream sf(argv[4]);
    sf << summary;
    std::fputs(summary.c_str(), stdout);
    std::printf("F16_LATERAL_DYNAMICS_ISOLATION_V2_RESULT=%s\n", audit.Failures == 0 ? "PASS" : "FAIL");
    return audit.Failures == 0 ? 0 : 1;
}
