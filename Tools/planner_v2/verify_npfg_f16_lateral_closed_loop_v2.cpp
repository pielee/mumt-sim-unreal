// verify_npfg_f16_lateral_closed_loop_v2.cpp
//
// Lateral closed loop: the PRODUCTION guidance chain driving the ACTUAL JSBSim F-16 plant.
//
//   FormationGuidanceCoordinatorV2  (production; owns the real FPx4NpfgAdapter and FPx4TecsAdapter)
//     -> RollReferenceRad          (NPFG)          -> lateral channel, the subject of this harness
//     -> PitchReferenceRad / ThrottleReferenceNorm (TECS) -> holds altitude and EAS while it turns
//   F16StickAdapterV2               (production)
//     -> normalized aileron / elevator / rudder / throttle commands
//   actual f16.xml FCS -> actual surfaces -> actual aerodynamics -> actual JSBSim F-16 FDM
//     -> position / ground velocity / roll / body rates / altitude / climb / EAS / pitch
//   -> back into FormationGuidanceCoordinatorV2
//
// This harness verifies the production NPFG caller and F16StickAdapterV2 against the actual JSBSim
// F-16 plant in isolated lateral cases. It does not select or tune production NPFG or TECS
// parameters.
//
// The TECS performance values used by this host fixture are derived from the committed
// single-condition solver characterization. They are test-local controller-excitation values, not
// production recommendations or operational limits.
//
// Generic NPFG and TECS control gains, damping, filters and time constants remain at the committed
// production defaults.
//
// No guidance or stick control law is reimplemented here: every roll/pitch/throttle reference comes
// out of the production coordinator, and every normalized command comes out of the production stick
// adapter. The harness owns only the FGFDMExec, the navigation frame, the path geometry, the
// setpoint schedule and the measurement.
//
// Sign chain under test (NE frame, course measured from North, clockwise positive):
//   cross-track error > 0  (aircraft RIGHT of path)  -> roll reference < 0 -> aileron < 0
//     -> body roll rate P < 0 -> roll < 0 -> ground course decreases (turn LEFT, back to the path)
//   course error > 0       (path course RIGHT of the aircraft's course) -> roll reference > 0 -> ...
//   path curvature > 0     (right-hand turn)         -> roll reference > 0 -> ...
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
constexpr double kPi = 3.14159265358979323846;

// ---- airborne initial condition, identical to the committed C1 and longitudinal harnesses ------
constexpr double kInitialLatitudeDeg = 47.0;
constexpr double kInitialLongitudeDeg = -122.0;
constexpr double kInitialAltitudeFt = 10000.0;   // 3,048 m
constexpr double kInitialTasMps = 220.0;
constexpr double kInitialHeadingDeg = 90.0;      // due east
constexpr double kTank0Lb = 1500.0, kTank1Lb = 1500.0, kTank2Lb = 0.0, kTank3Lb = 0.0;  // 3,000 lb
constexpr double kExpectedFuelLb = 3000.0;

// ---- test-local TECS configuration -------------------------------------------------------------
// Identical to the committed longitudinal harness: the longitudinal channel must simply hold while
// the lateral channel is exercised. Provenance (committed verify_f16_vertical_performance_v2,
// 10,000 ft / 3,000 lb / wind 0 / clean / speedbrake retracted):
//   max climb  -> Military (throttle-cmd 0.5) @ trim EAS 189.070713
//   min sink   -> Idle (throttle-cmd 0.0) @ configured trim EAS 189.070713
//   max sink   -> Idle (throttle-cmd 0.0) @ configured EasMax 220.0
// These are test-local controller-excitation values, not production recommendations.
constexpr double kTestLocalEasMinMps = 170.0;
constexpr double kTestLocalEasMaxMps = 220.0;
constexpr double kTestLocalMaxClimbRateMps = 100.539354597920735;
constexpr double kTestLocalMinSinkRateMps = 53.569103350590687;
constexpr double kTestLocalMaxSinkRateMps = 83.108839138474863;

// Known-good trim anchor (committed C1 stick plant harness); asserted, not assumed.
constexpr double kAnchorTrimEasMps = 189.070713;
constexpr double kAnchorTrimTasMps = 220.0;
constexpr double kAnchorTrimThrottleCmd = 0.296895857;
constexpr double kAnchorTrimThrottlePos = 0.593791714;
constexpr double kAnchorTrimPitchRad = 0.012764009;
constexpr double kAnchorTrimTolerance = 1.0e-6;

// ---- case schedule -----------------------------------------------------------------------------
// The settle window is long on purpose: engaging the loop at the trim point excites a lightly damped
// energy oscillation, and the baseline window must measure the settled hold -- numerical noise,
// natural drift and controller engagement bias -- not that transient.
constexpr double kPrimeS = 0.5;
constexpr double kSettleEndS = 60.0;
constexpr double kStepTimeS = 70.0;       // measurement window is [kSettleEndS, kStepTimeS)
constexpr double kCaseDurationS = 130.0;
constexpr double kReversalTimeS = 100.0;
constexpr double kReversalDurationS = 160.0;

constexpr double kCrossTrackStepM = 500.0;
constexpr double kCourseErrorStepRad = 10.0 * kPi / 180.0;
constexpr double kPathTurnRadiusM = 10000.0;

// ---- numerical floors: a threshold never falls below these --------------------------------------
constexpr double kAngleFloorRad = 1.0e-7;
constexpr double kRateFloorRadps = 1.0e-7;
constexpr double kCommandFloorNorm = 1.0e-6;
constexpr double kDistanceFloorM = 1.0e-3;
constexpr double kNoiseMultiplier = 5.0;

// ---- physical sanity bounds (NOT performance gates) ---------------------------------------------
constexpr double kCrossTrackSanityM = 5000.0;
constexpr double kAltitudeErrorSanityM = 1000.0;
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
// Right of a course, in the NE frame with course measured clockwise from North.
Vec2 RightNormal(double c) { return Vec2{-std::sin(c), std::cos(c)}; }
double CourseOf(const Vec2 &v) { return std::atan2(v.E, v.N); }

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
    double AileronLeftPosRad{}, AileronRightPosRad{}, ElevatorPosRad{}, RudderPosRad{}, ThrottlePos{};
    double GearPos{}, FlapPosNorm{}, SpeedbrakePos{};
    bool bWow{}, bEngineRunning{};
    bool bRatioValid{};

    bool IsFinite() const
    {
        const std::array<double, 26> v{
            TimeS, NorthM, EastM, VelNorthMps, VelEastMps, GroundCourseRad, GroundSpeedMps,
            AltitudeAslM, AltitudeAglM, ClimbRateMps, EasMps, TasMps, RollRad, PitchRad, YawRad,
            PRadps, QRadps, RRadps, AlphaRad, BetaRad, LoadFactor,
            AileronLeftPosRad, AileronRightPosRad, ElevatorPosRad, RudderPosRad, ThrottlePos};
        return std::all_of(v.begin(), v.end(), Finite);
    }
};

struct FTrimResult {
    bool bAttempted{}, bSuccess{};
    double PitchRad{}, RollRad{}, EasMps{}, TasMps{}, AltitudeM{}, GroundCourseRad{};
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
        const FPlantState s = Read();
        if (!s.IsFinite()) { failure = "non-finite plant state after successful trim"; return false; }
        Trim.GroundCourseRad = s.GroundCourseRad;
        return true;
    }

    FPlantState Read() const
    {
        FPlantState s{};
        s.TimeS = Exec->GetSimTime();
        // Harness-owned local NE frame: a flat-earth transform of the ACTUAL FDM geodetic state.
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
// Path geometry. The harness owns the path; NPFG inside the production coordinator does the
// guidance. The path is latched once at the step (and once at the reversal) from the ACTUAL plant
// state, so the geometry is a fixed object afterwards, not something re-anchored every frame.
// ================================================================================================
struct FPath {
    bool bArc{};
    Vec2 Origin{};      // straight: a point on the line
    Vec2 Tangent{};     // straight: unit tangent
    Vec2 Center{};      // arc: centre
    double RadiusM{};   // arc: radius
    double Curvature{}; // signed, right-positive (Planner and PX4 NPFG share this convention)
};

FPath MakeStraight(const Vec2 &through, double courseRad)
{
    FPath p{};
    p.bArc = false;
    p.Origin = through;
    p.Tangent = FromCourse(courseRad);
    p.Curvature = 0.0;
    return p;
}
// A right-hand turn (curvature > 0) puts the centre to the RIGHT of the current course.
FPath MakeArc(const Vec2 &through, double courseRad, double radiusM, int turnSign)
{
    FPath p{};
    p.bArc = true;
    p.RadiusM = radiusM;
    p.Curvature = static_cast<double>(turnSign) / radiusM;
    const Vec2 right = RightNormal(courseRad);
    p.Center = through + right * (static_cast<double>(turnSign) * radiusM);
    return p;
}

struct FPathSample {
    Vec2 PositionNE{};
    Vec2 TangentNE{};
    double Curvature{};
    double CourseRad{};
    double CrossTrackM{};   // signed: + = aircraft is to the RIGHT of the path
    bool bValid{};
};

FPathSample SamplePath(const FPath &path, const Vec2 &pos)
{
    FPathSample s{};
    s.Curvature = path.Curvature;
    if (!path.bArc) {
        const double along = (pos - path.Origin).Dot(path.Tangent);
        s.PositionNE = path.Origin + path.Tangent * along;
        s.TangentNE = path.Tangent;
    } else {
        const Vec2 radial = pos - path.Center;
        const double r = radial.Norm();
        if (!Finite(r) || r < 1e-6) return s;   // degenerate: reported invalid, never faked
        const Vec2 unit = radial * (1.0 / r);
        s.PositionNE = path.Center + unit * path.RadiusM;
        // rightNormal(tangent) must point from the path point toward the centre for a right turn.
        if (path.Curvature > 0.0) s.TangentNE = Vec2{-unit.E, unit.N};
        else s.TangentNE = Vec2{unit.E, -unit.N};
    }
    const double n = s.TangentNE.Norm();
    if (!Finite(n) || n < 1e-9) return s;
    s.TangentNE = s.TangentNE * (1.0 / n);
    s.CourseRad = CourseOf(s.TangentNE);
    s.CrossTrackM = (pos - s.PositionNE).Dot(RightNormal(s.CourseRad));
    s.bValid = Finite(s.PositionNE.N) && Finite(s.PositionNE.E) && Finite(s.CrossTrackM);
    return s;
}

// ================================================================================================
// Cases
// ================================================================================================
enum class EStep : std::uint8_t { None, CrossTrack, CourseError, Curvature };

struct FCaseDef {
    std::string Name;
    std::uint32_t ResetGeneration{};
    double DurationS{kCaseDurationS};
    EStep Step{EStep::None};
    int Sign{0};            // +1 = right / positive, -1 = left / negative
    bool bReversal{};       // the cross-track offset is mirrored at kReversalTimeS
};

std::vector<FCaseDef> BuildCases()
{
    std::vector<FCaseDef> c;
    c.push_back({"baseline_hold_a", 1, kCaseDurationS, EStep::None, 0, false});
    c.push_back({"baseline_hold_b", 2, kCaseDurationS, EStep::None, 0, false});
    c.push_back({"baseline_hold_c", 3, kCaseDurationS, EStep::None, 0, false});
    c.push_back({"cross_track_right_500m", 4, kCaseDurationS, EStep::CrossTrack, +1, false});
    c.push_back({"cross_track_left_500m", 5, kCaseDurationS, EStep::CrossTrack, -1, false});
    c.push_back({"course_error_right_10deg", 6, kCaseDurationS, EStep::CourseError, +1, false});
    c.push_back({"course_error_left_10deg", 7, kCaseDurationS, EStep::CourseError, -1, false});
    c.push_back({"curvature_right_10km", 8, kCaseDurationS, EStep::Curvature, +1, false});
    c.push_back({"curvature_left_10km", 9, kCaseDurationS, EStep::Curvature, -1, false});
    c.push_back({"lateral_reversal", 10, kReversalDurationS, EStep::CrossTrack, +1, true});
    return c;
}

struct FFrame {
    int CaseIndex{};
    double TimeS{};
    std::uint32_t ResetGeneration{};
    bool bCommandReady{};
    int GuidanceFailure{}, StickFailure{};
    bool bStickValid{};
    // path / lateral geometry
    double PathCourseRad{}, PathCurvature{}, CrossTrackM{}, CourseErrorRad{}, UnwrappedCourseRad{};
    double PathPositionN{}, PathPositionE{};
    // guidance
    double RollReferenceRad{}, PitchReferenceRad{}, ThrottleReferenceNorm{};
    double CourseSetpointRad{}, LateralAccelTotalMps2{}, LateralAccelFfMps2{}, LateralAccelFbMps2{};
    double WindFeasibility{}, UnderspeedRatio{}, FastDescendRatio{};
    // longitudinal hold
    double AltitudeSetpointM{}, AltitudeErrorM{}, EasSetpointMps{}, EasErrorMps{};
    // commands
    double AileronCmd{}, ElevatorCmd{}, RudderCmd{}, ThrottleCmd{};
    bool bCommandApplied{};
    FPlantState State{};
};

struct FNoise {
    double RollRefP2p{}, AileronP2p{}, PP2p{}, RollP2p{}, CourseP2p{}, CrossTrackP2p{}, CourseErrorP2p{};
    double CrossTrackDriftM{}, CourseDriftRad{};
};

struct FThresholds {
    double RollRef{}, Aileron{}, P{}, Roll{}, Course{}, CrossTrack{}, CourseError{};
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
    case EF16StickFailureV2::PrimeIdentityMismatch: return "PrimeIdentityMismatch";
    }
    return "Unknown";
}
const char *StepName(EStep s)
{
    switch (s) {
    case EStep::None: return "none";
    case EStep::CrossTrack: return "cross_track";
    case EStep::CourseError: return "course_error";
    case EStep::Curvature: return "curvature";
    }
    return "unknown";
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
    FTrimResult Trim{};
    std::uint64_t ControlFrames{}, FdmSteps{};
    FWriteCounters Counters{};
    FNoise Noise{};
    // settled pre-step references
    double RefRollRef{kNa}, RefAileron{kNa}, RefP{kNa}, RefRoll{kNa}, RefCourse{kNa};
    double RefCrossTrack{kNa}, RefCourseError{kNa};
    // step response
    double InitialCrossTrackM{kNa}, FinalCrossTrackM{kNa}, MaxAbsCrossTrackM{kNa};
    double InitialCourseErrorRad{kNa}, FinalCourseErrorRad{kNa};
    double ReversalCrossTrackM{kNa};
    double DetectRollRefS{-1.0}, DetectAileronS{-1.0}, DetectPS{-1.0}, DetectRollS{-1.0}, DetectCourseS{-1.0};
    double ReversalDetectRollRefS{-1.0}, ReversalDetectAileronS{-1.0}, ReversalDetectPS{-1.0};
    double ReversalDetectRollS{-1.0}, ReversalDetectCourseS{-1.0};
    double RollRefMin{kNa}, RollRefMax{kNa}, AileronMin{kNa}, AileronMax{kNa};
    double RollMin{kNa}, RollMax{kNa}, RudderAbsMax{kNa};
    double SteadyStateRollRad{kNa}, RequiredCurvatureRollRad{kNa};
    double MaxAbsAltitudeErrorM{kNa}, MaxAbsEasErrorMps{kNa};
    std::uint64_t RollSaturatedFrames{}, AileronSaturatedFrames{};
    std::uint64_t NonFiniteStates{}, UnexpectedWow{}, ConfigViolations{}, InvalidPathSamples{};
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
    const FTrimResult &trim = plant.TrimInfo();
    result.Trim = trim;

    FGuidanceConfigV2 guidanceConfig = guidanceConfigTemplate;
    guidanceConfig.TecsEquivalentAirspeedTrimMps = trim.EasMps;
    guidanceConfig.ThrottleTrim = std::clamp(trim.ThrottleCmd, guidanceConfig.ThrottleMin, guidanceConfig.ThrottleMax);

    audit.Check(std::abs(trim.EasMps - kAnchorTrimEasMps) <= kAnchorTrimTolerance, def.Name + ": trim EAS is not the known-good anchor");
    audit.Check(std::abs(trim.TasMps - kAnchorTrimTasMps) <= kAnchorTrimTolerance, def.Name + ": trim TAS is not the known-good anchor");
    audit.Check(std::abs(trim.ThrottleCmd - kAnchorTrimThrottleCmd) <= kAnchorTrimTolerance, def.Name + ": trim throttle command is not the known-good anchor");
    audit.Check(std::abs(trim.ThrottlePos - kAnchorTrimThrottlePos) <= kAnchorTrimTolerance, def.Name + ": trim throttle position is not the known-good anchor");
    audit.Check(std::abs(trim.PitchRad - kAnchorTrimPitchRad) <= kAnchorTrimTolerance, def.Name + ": trim pitch is not the known-good anchor");

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

    const FPlantState initial = plant.Read();
    FPath path = MakeStraight(Vec2{initial.NorthM, initial.EastM}, initial.GroundCourseRad);
    bool bStepLatched = false, bReversalLatched = false;

    FF16StickCommandV2 heldCommand = plant.TrimCommand();
    FF16StickCommandV2 previousApplied = heldCommand;
    bool havePreviousApplied = true;
    FF16StickCommandV2 previousValid{};
    bool havePreviousValid = false;

    double previousTimeS = -1.0;
    double unwrappedCourse = initial.GroundCourseRad;
    double previousCourse = initial.GroundCourseRad;
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

        const Vec2 pos{state.NorthM, state.EastM};

        // ---- latch the commanded geometry once, from the ACTUAL plant state ---------------------
        if (!bStepLatched && state.TimeS >= kStepTimeS && def.Step != EStep::None) {
            const double s = static_cast<double>(def.Sign);
            switch (def.Step) {
            case EStep::CrossTrack:
                // Offset the path so the aircraft sits exactly kCrossTrackStepM to the given side of
                // it, with zero course error: a pure cross-track excitation.
                path = MakeStraight(pos - RightNormal(state.GroundCourseRad) * (s * kCrossTrackStepM),
                                    state.GroundCourseRad);
                break;
            case EStep::CourseError:
                // Rotate the path course, keeping the aircraft on the path: a pure course-error
                // excitation. Sign > 0 => the path heads to the RIGHT of the aircraft's course.
                path = MakeStraight(pos, state.GroundCourseRad + s * kCourseErrorStepRad);
                break;
            case EStep::Curvature:
                // A constant-radius arc through the aircraft's current position and course: zero
                // initial cross-track and course error, pure curvature.
                path = MakeArc(pos, state.GroundCourseRad, kPathTurnRadiusM, def.Sign);
                break;
            case EStep::None:
                break;
            }
            bStepLatched = true;
        }
        if (def.bReversal && !bReversalLatched && state.TimeS >= kReversalTimeS) {
            // Mirror the offset: the aircraft is now the same distance to the OTHER side.
            const double s = -static_cast<double>(def.Sign);
            path = MakeStraight(pos - RightNormal(state.GroundCourseRad) * (s * kCrossTrackStepM),
                                state.GroundCourseRad);
            bReversalLatched = true;
        }

        const FPathSample sample = SamplePath(path, pos);
        if (!sample.bValid) ++result.InvalidPathSamples;
        audit.Check(sample.bValid, def.Name + ": degenerate path sample");
        if (!sample.bValid) { failure = def.Name + ": degenerate path sample"; return false; }

        unwrappedCourse += WrapPi(state.GroundCourseRad - previousCourse);
        previousCourse = state.GroundCourseRad;
        const double courseError = WrapPi(sample.CourseRad - state.GroundCourseRad);

        // ---- production coordinator input, built from the ACTUAL plant state ------------------
        FGuidanceCoordinatorInputV2 in{};
        FCanonicalNavigationStateV2 &f = in.Follower;
        f.PositionNE_m = pos;
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
        f.bRatioValid = state.bRatioValid;

        FFormationSlotStateV2 &slot = in.Slot;
        slot.ResetGeneration = def.ResetGeneration;
        slot.OriginGeneration = def.ResetGeneration;
        slot.bValid = true;

        FPlannerV2OutputAdapterResult &dto = in.PlannerDto;
        dto.Npfg.PathPositionNE_m = sample.PositionNE;
        dto.Npfg.PathUnitTangentNE = sample.TangentNE;
        dto.Npfg.PathCurvature_per_m = sample.Curvature;
        dto.Npfg.bValid = true;
        // The longitudinal channel simply holds: the real TECS keeps altitude and EAS at the trim
        // point while NPFG turns the aircraft.
        dto.Tecs.TargetEasMps = trimEasMps;
        dto.Tecs.TargetAltitudeAslM = trimAltitudeM;
        dto.Tecs.TargetClimbRateMps = 0.0;
        dto.Tecs.bTargetEasValid = true;
        dto.Tecs.bTargetAltitudeValid = true;
        dto.Tecs.bTargetClimbRateValid = false;
        dto.Tecs.bCommandReady = true;

        in.CurrentPitchRad = state.PitchRad;
        in.bCurrentPitchValid = true;
        // ACTUAL roll, from the same atomic plant snapshot that feeds the stick adapter. The
        // production coordinator derives the TECS turn load factor from it (pinned PX4 contract).
        in.CurrentRollRad = state.RollRad;
        in.bCurrentRollValid = true;
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
            audit.Check(!guidance.bCommandReady && guidance.FailureReason == EGuidanceFailureV2::ResetFrame,
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
            const bool rollInPolicy = std::abs(guidance.RollReferenceRad) <= guidanceConfig.RollLimitRad + 1e-12;
            audit.Check(rollInPolicy, def.Name + ": roll reference outside the configured roll limit");
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

        FFrame fr{};
        fr.CaseIndex = caseIndex;
        fr.TimeS = state.TimeS;
        fr.ResetGeneration = def.ResetGeneration;
        fr.bCommandReady = guidance.bCommandReady;
        fr.GuidanceFailure = static_cast<int>(guidance.FailureReason);
        fr.StickFailure = static_cast<int>(command.FailureReason);
        fr.bStickValid = command.bValid;
        fr.PathCourseRad = sample.CourseRad;
        fr.PathCurvature = sample.Curvature;
        fr.PathPositionN = sample.PositionNE.N;
        fr.PathPositionE = sample.PositionNE.E;
        fr.CrossTrackM = sample.CrossTrackM;
        fr.CourseErrorRad = courseError;
        fr.UnwrappedCourseRad = unwrappedCourse;
        fr.RollReferenceRad = guidance.RollReferenceRad;
        fr.PitchReferenceRad = guidance.PitchReferenceRad;
        fr.ThrottleReferenceNorm = guidance.ThrottleReferenceNorm;
        fr.CourseSetpointRad = guidance.CourseSetpointRad;
        fr.LateralAccelTotalMps2 = guidance.LateralAccelerationTotalMps2;
        fr.LateralAccelFfMps2 = guidance.LateralAccelerationFeedforwardMps2;
        fr.LateralAccelFbMps2 = guidance.LateralAccelerationFeedbackMps2;
        fr.WindFeasibility = guidance.WindFeasibility;
        fr.UnderspeedRatio = guidance.UnderspeedRatio;
        fr.FastDescendRatio = guidance.FastDescendRatio;
        fr.AltitudeSetpointM = trimAltitudeM;
        fr.AltitudeErrorM = trimAltitudeM - state.AltitudeAslM;
        fr.EasSetpointMps = trimEasMps;
        fr.EasErrorMps = trimEasMps - state.EasMps;
        fr.AileronCmd = heldCommand.AileronCmdNorm;
        fr.ElevatorCmd = heldCommand.ElevatorCmdNorm;
        fr.RudderCmd = heldCommand.RudderCmdNorm;
        fr.ThrottleCmd = heldCommand.ThrottleCmdNorm;
        fr.bCommandApplied = applied;
        fr.State = state;
        result.Trace.push_back(fr);
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
    std::vector<double> rollRef, aileron, p, roll, course, crossTrack, courseError;
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS < kSettleEndS || fr.TimeS >= kStepTimeS) continue;
        rollRef.push_back(fr.RollReferenceRad);
        aileron.push_back(fr.AileronCmd);
        p.push_back(fr.State.PRadps);
        roll.push_back(fr.State.RollRad);
        course.push_back(fr.UnwrappedCourseRad);
        crossTrack.push_back(fr.CrossTrackM);
        courseError.push_back(fr.CourseErrorRad);
    }
    r.Noise.RollRefP2p = PeakToPeak(rollRef);
    r.Noise.AileronP2p = PeakToPeak(aileron);
    r.Noise.PP2p = PeakToPeak(p);
    r.Noise.RollP2p = PeakToPeak(roll);
    r.Noise.CourseP2p = PeakToPeak(course);
    r.Noise.CrossTrackP2p = PeakToPeak(crossTrack);
    r.Noise.CourseErrorP2p = PeakToPeak(courseError);

    r.RefRollRef = Mean(rollRef);
    r.RefAileron = Mean(aileron);
    r.RefP = Mean(p);
    r.RefRoll = Mean(roll);
    r.RefCourse = Mean(course);
    r.RefCrossTrack = Mean(crossTrack);
    r.RefCourseError = Mean(courseError);

    const FFrame *settle = nullptr;
    const FFrame *last = r.Trace.empty() ? nullptr : &r.Trace.back();
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS >= kSettleEndS) { settle = &fr; break; }
    }
    if (settle && last) {
        r.Noise.CrossTrackDriftM = last->CrossTrackM - settle->CrossTrackM;
        r.Noise.CourseDriftRad = last->UnwrappedCourseRad - settle->UnwrappedCourseRad;
    }
}

FThresholds DeriveThresholds(const std::vector<FCaseResult> &baselines)
{
    FNoise w{};
    for (const FCaseResult &b : baselines) {
        w.RollRefP2p = std::max(w.RollRefP2p, b.Noise.RollRefP2p);
        w.AileronP2p = std::max(w.AileronP2p, b.Noise.AileronP2p);
        w.PP2p = std::max(w.PP2p, b.Noise.PP2p);
        w.RollP2p = std::max(w.RollP2p, b.Noise.RollP2p);
        w.CourseP2p = std::max(w.CourseP2p, b.Noise.CourseP2p);
        w.CrossTrackP2p = std::max(w.CrossTrackP2p, b.Noise.CrossTrackP2p);
        w.CourseErrorP2p = std::max(w.CourseErrorP2p, b.Noise.CourseErrorP2p);
    }
    FThresholds t{};
    t.RollRef = std::max(kNoiseMultiplier * w.RollRefP2p, kAngleFloorRad);
    t.Aileron = std::max(kNoiseMultiplier * w.AileronP2p, kCommandFloorNorm);
    t.P = std::max(kNoiseMultiplier * w.PP2p, kRateFloorRadps);
    t.Roll = std::max(kNoiseMultiplier * w.RollP2p, kAngleFloorRad);
    t.Course = std::max(kNoiseMultiplier * w.CourseP2p, kAngleFloorRad);
    t.CrossTrack = std::max(kNoiseMultiplier * w.CrossTrackP2p, kDistanceFloorM);
    t.CourseError = std::max(kNoiseMultiplier * w.CourseErrorP2p, kAngleFloorRad);
    return t;
}

double DetectDirection(const std::vector<FFrame> &trace, double fromS, double reference,
                       double threshold, int sign, double FFrame::*member)
{
    for (const FFrame &fr : trace) {
        if (fr.TimeS < fromS) continue;
        if ((fr.*member - reference) * static_cast<double>(sign) > threshold) return fr.TimeS;
    }
    return -1.0;
}
double DetectStateDirection(const std::vector<FFrame> &trace, double fromS, double reference,
                            double threshold, int sign, double FPlantState::*member)
{
    for (const FFrame &fr : trace) {
        if (fr.TimeS < fromS) continue;
        if ((fr.State.*member - reference) * static_cast<double>(sign) > threshold) return fr.TimeS;
    }
    return -1.0;
}

void Summarize(FCaseResult &r)
{
    double rollRefMin = 1e18, rollRefMax = -1e18, aileronMin = 1e18, aileronMax = -1e18;
    double rollMin = 1e18, rollMax = -1e18, rudderAbs = 0.0;
    double maxAbsXtk = 0.0, maxAbsAlt = 0.0, maxAbsEas = 0.0;
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS < kPrimeS) continue;
        rollRefMin = std::min(rollRefMin, fr.RollReferenceRad);
        rollRefMax = std::max(rollRefMax, fr.RollReferenceRad);
        aileronMin = std::min(aileronMin, fr.AileronCmd);
        aileronMax = std::max(aileronMax, fr.AileronCmd);
        rollMin = std::min(rollMin, fr.State.RollRad);
        rollMax = std::max(rollMax, fr.State.RollRad);
        rudderAbs = std::max(rudderAbs, std::abs(fr.RudderCmd));
        maxAbsXtk = std::max(maxAbsXtk, std::abs(fr.CrossTrackM));
        maxAbsAlt = std::max(maxAbsAlt, std::abs(fr.AltitudeErrorM));
        maxAbsEas = std::max(maxAbsEas, std::abs(fr.EasErrorMps));
        if (std::abs(fr.RollReferenceRad) >= 0.7853981633974483 - 1e-9) ++r.RollSaturatedFrames;
        if (std::abs(fr.AileronCmd) >= 1.0 - 1e-9) ++r.AileronSaturatedFrames;
    }
    r.RollRefMin = rollRefMin; r.RollRefMax = rollRefMax;
    r.AileronMin = aileronMin; r.AileronMax = aileronMax;
    r.RollMin = rollMin; r.RollMax = rollMax;
    r.RudderAbsMax = rudderAbs;
    r.MaxAbsCrossTrackM = maxAbsXtk;
    r.MaxAbsAltitudeErrorM = maxAbsAlt;
    r.MaxAbsEasErrorMps = maxAbsEas;

    const FFrame *atStep = nullptr, *atReversal = nullptr;
    for (const FFrame &fr : r.Trace) {
        if (!atStep && fr.TimeS >= kStepTimeS) atStep = &fr;
        if (r.Def.bReversal && !atReversal && fr.TimeS >= kReversalTimeS) atReversal = &fr;
    }
    if (atStep) {
        r.InitialCrossTrackM = atStep->CrossTrackM;
        r.InitialCourseErrorRad = atStep->CourseErrorRad;
    }
    if (atReversal) r.ReversalCrossTrackM = atReversal->CrossTrackM;
    if (!r.Trace.empty()) {
        r.FinalCrossTrackM = r.Trace.back().CrossTrackM;
        r.FinalCourseErrorRad = r.Trace.back().CourseErrorRad;
        // Settled bank in the last 10 s -- the curvature cases should hold a steady turn.
        std::vector<double> tailRoll;
        const double tailStartS = r.Trace.back().TimeS - 10.0;
        for (const FFrame &fr : r.Trace)
            if (fr.TimeS >= tailStartS) tailRoll.push_back(fr.State.RollRad);
        r.SteadyStateRollRad = Mean(tailRoll);
    }
    if (r.Def.Step == EStep::Curvature && !r.Trace.empty()) {
        // Coordinated-turn bank for this radius and this true airspeed: atan(V^2 / (R g)).
        const double v = r.Trace.back().State.TasMps;
        r.RequiredCurvatureRollRad = static_cast<double>(r.Def.Sign) *
                                     std::atan(v * v / (kPathTurnRadiusM * 9.80665));
    }
}

void GradeCase(FCaseResult &r, const FThresholds &t, FAudit &audit)
{
    const std::string &n = r.Def.Name;

    // Hard invariants (every case)
    audit.Check(r.NonFiniteStates == 0, n + ": non-finite plant state count is not zero");
    audit.Check(r.UnexpectedWow == 0, n + ": unexpected WOW count is not zero");
    audit.Check(r.ConfigViolations == 0, n + ": gear/flap/speedbrake/engine violation count is not zero");
    audit.Check(r.InvalidPathSamples == 0, n + ": degenerate path sample count is not zero");
    audit.Check(r.CommandRangeViolations == 0, n + ": command range violation count is not zero");
    audit.Check(r.CommandSlewViolations == 0, n + ": command slew violation count is not zero");
    audit.Check(r.TimestampRegressions == 0, n + ": timestamp regression count is not zero");
    audit.Check(r.ResetGenerationMismatches == 0, n + ": reset-generation mismatch count is not zero");
    audit.Check(r.Counters.FdmRunFailures == 0, n + ": FDM run failure count is not zero");
    audit.Check(r.Counters.UnexpectedInvalidCoordinatorFrames == 0, n + ": unexpected invalid coordinator frames");
    audit.Check(r.Counters.UnexpectedInvalidStickFrames == 0, n + ": unexpected invalid stick frames");
    audit.Check(r.Counters.ExpectedResetFrames == 1, n + ": expected exactly one reset frame");
    audit.Check(r.Counters.WritesOutsideOwnedFdm == 0, n + ": writes outside the owned FGFDMExec");
    audit.Check(r.Counters.ProductionWriterInvocations == 0, n + ": production writer invocations");

    // Physical sanity (NOT performance gates)
    audit.Check(Finite(r.MaxAbsCrossTrackM) && r.MaxAbsCrossTrackM < kCrossTrackSanityM,
                n + ": cross-track error left the physical sanity bound");
    audit.Check(Finite(r.MaxAbsAltitudeErrorM) && r.MaxAbsAltitudeErrorM < kAltitudeErrorSanityM,
                n + ": the longitudinal hold left the physical sanity bound");
    for (const FFrame &fr : r.Trace) {
        if (fr.TimeS < kPrimeS) continue;
        if (!(fr.State.EasMps > kEasSanityMinMps && fr.State.EasMps < kEasSanityMaxMps)) {
            audit.Check(false, n + ": EAS left the physical sanity bound");
            break;
        }
    }

    if (r.Def.Step == EStep::None) {
        audit.Check(Finite(r.FinalCrossTrackM), n + ": baseline cross-track is not finite");
        return;
    }

    // The full sign chain: geometry -> roll reference -> aileron -> P -> roll -> ground course.
    //   cross-track > 0 (right of path) => turn LEFT  => every lateral signal negative
    //   course error > 0 (path to the right) => turn RIGHT => every lateral signal positive
    //   curvature   > 0 (right-hand turn)    => turn RIGHT => every lateral signal positive
    int chain = 0;
    switch (r.Def.Step) {
    case EStep::CrossTrack: chain = -r.Def.Sign; break;   // offset right => bank left
    case EStep::CourseError: chain = r.Def.Sign; break;
    case EStep::Curvature: chain = r.Def.Sign; break;
    case EStep::None: break;
    }

    r.DetectRollRefS = DetectDirection(r.Trace, kStepTimeS, r.RefRollRef, t.RollRef, chain, &FFrame::RollReferenceRad);
    r.DetectAileronS = DetectDirection(r.Trace, kStepTimeS, r.RefAileron, t.Aileron, chain, &FFrame::AileronCmd);
    r.DetectPS = DetectStateDirection(r.Trace, kStepTimeS, r.RefP, t.P, chain, &FPlantState::PRadps);
    r.DetectRollS = DetectStateDirection(r.Trace, kStepTimeS, r.RefRoll, t.Roll, chain, &FPlantState::RollRad);
    r.DetectCourseS = DetectDirection(r.Trace, kStepTimeS, r.RefCourse, t.Course, chain, &FFrame::UnwrappedCourseRad);

    audit.Check(r.DetectRollRefS >= 0.0, n + ": no roll-reference response in the commanded direction");
    audit.Check(r.DetectAileronS >= 0.0, n + ": no aileron-command response in the commanded direction");
    audit.Check(r.DetectPS >= 0.0, n + ": no body roll-rate response in the commanded direction");
    audit.Check(r.DetectRollS >= 0.0, n + ": no roll response in the commanded direction");
    audit.Check(r.DetectCourseS >= 0.0, n + ": no ground-course response in the commanded direction");
    // Ordering of the chain: the reference must not lag the plant it drives.
    audit.Check(r.DetectRollRefS <= r.DetectRollS + 1e-9, n + ": roll responded before the roll reference");
    audit.Check(r.DetectAileronS <= r.DetectRollS + 1e-9, n + ": roll responded before the aileron command");
    audit.Check(r.DetectPS <= r.DetectRollS + 1e-9, n + ": roll responded before the body roll rate");
    audit.Check(r.DetectRollS <= r.DetectCourseS + 1e-9, n + ": ground course responded before the roll");

    if (r.Def.Step == EStep::CrossTrack) {
        audit.Check(Finite(r.InitialCrossTrackM) && Finite(r.FinalCrossTrackM) &&
                        std::abs(r.FinalCrossTrackM) < std::abs(r.InitialCrossTrackM),
                    n + ": final cross-track error is not smaller than the initial one");
        audit.Check(std::abs(std::abs(r.InitialCrossTrackM) - kCrossTrackStepM) < 1.0,
                    n + ": the commanded cross-track offset was not established");
    }
    if (r.Def.Step == EStep::CourseError) {
        audit.Check(Finite(r.InitialCourseErrorRad) && Finite(r.FinalCourseErrorRad) &&
                        std::abs(r.FinalCourseErrorRad) < std::abs(r.InitialCourseErrorRad),
                    n + ": final course error is not smaller than the initial one");
        audit.Check(std::abs(std::abs(r.InitialCourseErrorRad) - kCourseErrorStepRad) < 0.02,
                    n + ": the commanded course error was not established");
        // A course error also drives the aircraft off the path; the loop must still bring it back.
        audit.Check(Finite(r.FinalCrossTrackM) && std::abs(r.FinalCrossTrackM) < kCrossTrackSanityM,
                    n + ": cross-track diverged while correcting the course error");
    }
    if (r.Def.Step == EStep::Curvature) {
        // A steady turn: the settled bank must have the sign of the turn and be within a factor of
        // two of the coordinated-turn bank for this radius and airspeed. This is a sign/magnitude
        // sanity check on the geometry->bank chain, not a tracking-performance gate.
        audit.Check(Finite(r.SteadyStateRollRad) && Finite(r.RequiredCurvatureRollRad) &&
                        r.SteadyStateRollRad * r.RequiredCurvatureRollRad > 0.0,
                    n + ": the settled bank does not have the sign of the commanded turn");
        audit.Check(Finite(r.SteadyStateRollRad) &&
                        std::abs(r.SteadyStateRollRad) > 0.5 * std::abs(r.RequiredCurvatureRollRad) &&
                        std::abs(r.SteadyStateRollRad) < 2.0 * std::abs(r.RequiredCurvatureRollRad),
                    n + ": the settled bank is not near the coordinated-turn bank for this radius");
        audit.Check(Finite(r.FinalCrossTrackM) && std::abs(r.FinalCrossTrackM) < kCrossTrackSanityM,
                    n + ": cross-track diverged while following the arc");
    }

    if (r.Def.bReversal) {
        const int reverseChain = -chain;
        r.ReversalDetectRollRefS = DetectDirection(r.Trace, kReversalTimeS, r.RefRollRef, t.RollRef, reverseChain, &FFrame::RollReferenceRad);
        r.ReversalDetectAileronS = DetectDirection(r.Trace, kReversalTimeS, r.RefAileron, t.Aileron, reverseChain, &FFrame::AileronCmd);
        r.ReversalDetectPS = DetectStateDirection(r.Trace, kReversalTimeS, r.RefP, t.P, reverseChain, &FPlantState::PRadps);
        r.ReversalDetectRollS = DetectStateDirection(r.Trace, kReversalTimeS, r.RefRoll, t.Roll, reverseChain, &FPlantState::RollRad);
        r.ReversalDetectCourseS = DetectDirection(r.Trace, kReversalTimeS, r.RefCourse, t.Course, reverseChain, &FFrame::UnwrappedCourseRad);
        audit.Check(r.ReversalDetectRollRefS >= 0.0, n + ": roll reference did not reverse");
        audit.Check(r.ReversalDetectAileronS >= 0.0, n + ": aileron command did not reverse");
        audit.Check(r.ReversalDetectPS >= 0.0, n + ": body roll rate did not reverse");
        audit.Check(r.ReversalDetectRollS >= 0.0, n + ": roll did not reverse");
        audit.Check(Finite(r.ReversalCrossTrackM) && Finite(r.FinalCrossTrackM) &&
                        std::abs(r.FinalCrossTrackM) < std::abs(r.ReversalCrossTrackM),
                    n + ": final cross-track error is not smaller than the error at the reversal");
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
           "path_position_n_m,path_position_e_m,path_course_rad,path_curvature_per_m,"
           "cross_track_m,course_error_rad,unwrapped_course_rad,"
           "north_m,east_m,ground_course_rad,ground_speed_mps,"
           "roll_reference_rad,course_setpoint_rad,lateral_accel_total_mps2,"
           "lateral_accel_feedforward_mps2,lateral_accel_feedback_mps2,wind_feasibility,"
           "pitch_reference_rad,throttle_reference_norm,underspeed_ratio,fast_descend_ratio,"
           "altitude_setpoint_m,actual_altitude_m,altitude_error_m,"
           "eas_setpoint_mps,actual_eas_mps,actual_tas_mps,eas_to_tas_ratio,eas_error_mps,"
           "roll_rad,pitch_rad,yaw_rad,p_radps,q_radps,r_radps,"
           "aileron_cmd_norm,elevator_cmd_norm,rudder_cmd_norm,throttle_cmd_norm,command_applied,"
           "aileron_left_position_rad,aileron_right_position_rad,elevator_position_rad,"
           "rudder_position_rad,throttle_position,alpha_rad,beta_rad,load_factor,"
           "wow,engine_running,gear_pos,flap_pos_norm,speedbrake_pos\n";
    for (const FCaseResult &r : results) {
        for (const FFrame &f : r.Trace) {
            out << r.Def.Name << ',' << f.CaseIndex << ',';
            WriteNumber(out, f.TimeS, precision);
            out << ',' << f.ResetGeneration << ',' << (f.bCommandReady ? 1 : 0) << ','
                << GuidanceFailureName(static_cast<EGuidanceFailureV2>(f.GuidanceFailure)) << ','
                << (f.bStickValid ? 1 : 0) << ','
                << StickFailureName(static_cast<EF16StickFailureV2>(f.StickFailure)) << ',';
            const std::array<double, 7> geom{f.PathPositionN, f.PathPositionE, f.PathCourseRad,
                                             f.PathCurvature, f.CrossTrackM, f.CourseErrorRad,
                                             f.UnwrappedCourseRad};
            for (double v : geom) { WriteNumber(out, v, precision); out << ','; }
            const std::array<double, 4> nav{f.State.NorthM, f.State.EastM, f.State.GroundCourseRad,
                                            f.State.GroundSpeedMps};
            for (double v : nav) { WriteNumber(out, v, precision); out << ','; }
            const std::array<double, 10> g{f.RollReferenceRad, f.CourseSetpointRad, f.LateralAccelTotalMps2,
                                           f.LateralAccelFfMps2, f.LateralAccelFbMps2, f.WindFeasibility,
                                           f.PitchReferenceRad, f.ThrottleReferenceNorm,
                                           f.UnderspeedRatio, f.FastDescendRatio};
            for (double v : g) { WriteNumber(out, v, precision); out << ','; }
            const std::array<double, 8> lon{f.AltitudeSetpointM, f.State.AltitudeAslM, f.AltitudeErrorM,
                                            f.EasSetpointMps, f.State.EasMps, f.State.TasMps,
                                            f.State.EasToTasRatio, f.EasErrorMps};
            for (double v : lon) { WriteNumber(out, v, precision); out << ','; }
            const std::array<double, 6> att{f.State.RollRad, f.State.PitchRad, f.State.YawRad,
                                            f.State.PRadps, f.State.QRadps, f.State.RRadps};
            for (double v : att) { WriteNumber(out, v, precision); out << ','; }
            const std::array<double, 4> cmd{f.AileronCmd, f.ElevatorCmd, f.RudderCmd, f.ThrottleCmd};
            for (double v : cmd) { WriteNumber(out, v, precision); out << ','; }
            out << (f.bCommandApplied ? 1 : 0) << ',';
            const std::array<double, 8> plant{f.State.AileronLeftPosRad, f.State.AileronRightPosRad,
                                              f.State.ElevatorPosRad, f.State.RudderPosRad,
                                              f.State.ThrottlePos, f.State.AlphaRad, f.State.BetaRad,
                                              f.State.LoadFactor};
            for (double v : plant) { WriteNumber(out, v, precision); out << ','; }
            out << (f.State.bWow ? 1 : 0) << ',' << (f.State.bEngineRunning ? 1 : 0) << ',';
            WriteNumber(out, f.State.GearPos, precision); out << ',';
            WriteNumber(out, f.State.FlapPosNorm, precision); out << ',';
            WriteNumber(out, f.State.SpeedbrakePos, precision); out << '\n';
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

    const FGuidanceConfigV2 productionDefaults{};
    FGuidanceConfigV2 guidanceConfig{};
    guidanceConfig.EasMinMps = kTestLocalEasMinMps;
    guidanceConfig.EasMaxMps = kTestLocalEasMaxMps;
    guidanceConfig.TecsMaxClimbRateMps = kTestLocalMaxClimbRateMps;
    guidanceConfig.TecsMinSinkRateMps = kTestLocalMinSinkRateMps;
    guidanceConfig.TecsMaxSinkRateMps = kTestLocalMaxSinkRateMps;
    // TecsEquivalentAirspeedTrimMps and ThrottleTrim are set per case from the ACTUAL FGTrim result.
    // Every NPFG and TECS gain, damping, filter and time constant, and the roll limit, stay at the
    // committed production defaults.

    const FF16StickConfigV2 stickConfig{};   // production defaults, no gain changes

    FAudit audit;
    const std::vector<FCaseDef> cases = BuildCases();
    std::vector<FCaseResult> results(cases.size());

    // Pass 1: the three baseline holds, which define every directional threshold.
    for (std::size_t i = 0; i < cases.size(); ++i) {
        if (cases[i].Step != EStep::None) continue;
        std::string failure;
        if (!RunCase(root, static_cast<int>(i), cases[i], guidanceConfig, stickConfig, results[i], audit, failure)) {
            std::fprintf(stderr, "BLOCKED: %s\n", failure.c_str());
            return 1;
        }
        MeasureNoise(results[i]);
        Summarize(results[i]);
    }
    std::vector<FCaseResult> baselines;
    for (std::size_t i = 0; i < cases.size(); ++i)
        if (cases[i].Step == EStep::None) baselines.push_back(results[i]);
    const FThresholds thresholds = DeriveThresholds(baselines);

    // Pass 2: the commanded cases. The thresholds are already fixed and were never seen by them.
    for (std::size_t i = 0; i < cases.size(); ++i) {
        if (cases[i].Step == EStep::None) continue;
        std::string failure;
        if (!RunCase(root, static_cast<int>(i), cases[i], guidanceConfig, stickConfig, results[i], audit, failure)) {
            std::fprintf(stderr, "BLOCKED: %s\n", failure.c_str());
            return 1;
        }
        MeasureNoise(results[i]);
        Summarize(results[i]);
    }

    for (FCaseResult &r : results) GradeCase(r, thresholds, audit);

    // The three baselines are the same experiment three times: they must agree exactly.
    for (std::size_t i = 1; i < baselines.size(); ++i) {
        bool identical = results[0].Trace.size() == results[i].Trace.size();
        if (identical) {
            for (std::size_t k = 0; k < results[0].Trace.size(); ++k) {
                if (results[0].Trace[k].CrossTrackM != results[i].Trace[k].CrossTrackM ||
                    results[0].Trace[k].RollReferenceRad != results[i].Trace[k].RollReferenceRad ||
                    results[0].Trace[k].AileronCmd != results[i].Trace[k].AileronCmd) {
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

    const FTrimResult &t0 = results[0].Trim;

    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "NPFG_F16_LATERAL_CLOSED_LOOP_V2\n";
    out << "This harness verifies the production NPFG caller and F16StickAdapterV2 against the actual "
           "JSBSim F-16 plant in isolated lateral cases. It does not select or tune production NPFG or "
           "TECS parameters.\n";
    out << "The TECS performance values used by this host fixture are derived from the committed "
           "single-condition solver characterization. They are test-local controller-excitation "
           "values, not production recommendations or operational limits.\n";
    out << "Generic NPFG and TECS control gains, damping, filters and time constants remain at the "
           "committed production defaults.\n";
    out << "chain=FormationGuidanceCoordinatorV2(production_FPx4NpfgAdapter+FPx4TecsAdapter)"
           "->RollReferenceRad->F16StickAdapterV2(production)->normalized_aileron/rudder/elevator/"
           "throttle->f16.xml_FCS->actual_surfaces->actual_aerodynamics->JSBSim_FDM->position/"
           "ground_velocity/roll/body_rates->FormationGuidanceCoordinatorV2\n";
    out << "sign_chain cross_track>0(right_of_path)->roll_reference<0->aileron<0->P<0->roll<0->"
           "ground_course_decreases; course_error>0(path_right_of_course)->roll_reference>0->...; "
           "path_curvature>0(right_turn)->roll_reference>0->...\n";
    out << "model=f16 engine=F100-PW-229 data=Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim "
           "fdm_dt_s=" << kFdmDtS << " controller_dt_s=" << kControllerDtS
        << " fdm_steps_per_control_frame=" << kFdmStepsPerControlFrame
        << " clock=fixed_step_simulation_time_only\n";
    out << "initialization_mode=FGTrim_tFull initial_altitude_ft=" << kInitialAltitudeFt
        << " initial_tas_mps=" << kInitialTasMps << " heading_deg=" << kInitialHeadingDeg
        << " wind_mps=0 gear=up flap=clean speedbrake=retracted fuel=frozen\n";
    out << "trim solver=FGTrim(tFull) success=" << (t0.bSuccess ? 1 : 0)
        << " altitude_m=" << Num(t0.AltitudeM) << " eas_mps=" << Num(t0.EasMps)
        << " tas_mps=" << Num(t0.TasMps) << " pitch_rad=" << Num(t0.PitchRad)
        << " roll_rad=" << Num(t0.RollRad) << " throttle_cmd=" << Num(t0.ThrottleCmd)
        << " throttle_pos=" << Num(t0.ThrottlePos) << " elevator_pos_rad=" << Num(t0.ElevatorPosRad)
        << " ground_course_rad=" << Num(t0.GroundCourseRad)
        << " mass_slugs=" << Num(t0.MassSlugs) << " fuel_lb=" << Num(t0.FuelLb)
        << " fresh_trim_per_case=1\n";
    out << "test_local_config EasMinMps=" << Num(guidanceConfig.EasMinMps)
        << " EasMaxMps=" << Num(guidanceConfig.EasMaxMps)
        << " TecsEquivalentAirspeedTrimMps=actual_trim_EAS=" << Num(t0.EasMps)
        << " ThrottleTrim=actual_trim_throttle=" << Num(t0.ThrottleCmd)
        << " TecsMaxClimbRateMps=" << Num(guidanceConfig.TecsMaxClimbRateMps)
        << " TecsMinSinkRateMps=" << Num(guidanceConfig.TecsMinSinkRateMps)
        << " TecsMaxSinkRateMps=" << Num(guidanceConfig.TecsMaxSinkRateMps)
        << " RollLimitRad=" << Num(guidanceConfig.RollLimitRad)
        << "(production=" << Num(productionDefaults.RollLimitRad) << ")"
        << " MinimumGroundSpeedMps=" << Num(guidanceConfig.MinimumGroundSpeedMps)
        << "(production=" << Num(productionDefaults.MinimumGroundSpeedMps) << ")"
        << " stick_config=production_defaults_unchanged\n";
    out << "npfg_parameters=production_defaults_unchanged(FPx4NpfgAdapter_is_configured_only_by_the_"
           "coordinator;this_harness_sets_no_NPFG_gain)\n";
    out << "test_local_override_scope=host_fixture_only production_config_modified=0 "
           "npfg_gains_modified=0 tecs_gains_modified=0 stick_gains_modified=0 aircraft_xml_modified=0\n";
    out << "cases=" << results.size() << " total_control_frames=" << totalFrames
        << " total_fdm_steps=" << totalFdmSteps << '\n';
    out << "excitations cross_track_m=" << kCrossTrackStepM
        << " course_error_rad=" << kCourseErrorStepRad
        << " path_turn_radius_m=" << kPathTurnRadiusM
        << " settled_window=[" << Num(kSettleEndS) << ',' << Num(kStepTimeS) << ')'
        << " step_time_s=" << Num(kStepTimeS) << " reversal_time_s=" << Num(kReversalTimeS) << '\n';
    out << "derived_thresholds source=baseline_peak_to_peak_x" << kNoiseMultiplier << "_or_numerical_floor"
        << " roll_reference_rad=" << Num(thresholds.RollRef)
        << " aileron_cmd_norm=" << Num(thresholds.Aileron)
        << " p_radps=" << Num(thresholds.P)
        << " roll_rad=" << Num(thresholds.Roll)
        << " ground_course_rad=" << Num(thresholds.Course)
        << " cross_track_m=" << Num(thresholds.CrossTrack)
        << " course_error_rad=" << Num(thresholds.CourseError) << '\n';

    for (const FCaseResult &r : results) {
        out << "case name=" << r.Def.Name << " reset_generation=" << r.Def.ResetGeneration
            << " step=" << StepName(r.Def.Step) << " sign=" << r.Def.Sign
            << " reversal=" << (r.Def.bReversal ? 1 : 0)
            << " duration_s=" << Num(r.Def.DurationS)
            << " control_frames=" << r.ControlFrames << " fdm_steps=" << r.FdmSteps << '\n';
        out << "  baseline_window_noise roll_ref_p2p=" << Num(r.Noise.RollRefP2p)
            << " aileron_p2p=" << Num(r.Noise.AileronP2p)
            << " p_p2p=" << Num(r.Noise.PP2p)
            << " roll_p2p=" << Num(r.Noise.RollP2p)
            << " course_p2p=" << Num(r.Noise.CourseP2p)
            << " cross_track_p2p=" << Num(r.Noise.CrossTrackP2p)
            << " course_error_p2p=" << Num(r.Noise.CourseErrorP2p)
            << " cross_track_drift_m=" << Num(r.Noise.CrossTrackDriftM)
            << " course_drift_rad=" << Num(r.Noise.CourseDriftRad) << '\n';
        out << "  errors initial_cross_track_m=" << Num(r.InitialCrossTrackM)
            << " final_cross_track_m=" << Num(r.FinalCrossTrackM)
            << " max_abs_cross_track_m=" << Num(r.MaxAbsCrossTrackM)
            << " reversal_cross_track_m=" << Num(r.ReversalCrossTrackM)
            << " initial_course_error_rad=" << Num(r.InitialCourseErrorRad)
            << " final_course_error_rad=" << Num(r.FinalCourseErrorRad) << '\n';
        out << "  direction_detected_s roll_reference=" << OptTime(r.DetectRollRefS)
            << " aileron_cmd=" << OptTime(r.DetectAileronS)
            << " p=" << OptTime(r.DetectPS)
            << " roll=" << OptTime(r.DetectRollS)
            << " ground_course=" << OptTime(r.DetectCourseS) << '\n';
        if (r.Def.bReversal) {
            out << "  reversal_detected_s roll_reference=" << OptTime(r.ReversalDetectRollRefS)
                << " aileron_cmd=" << OptTime(r.ReversalDetectAileronS)
                << " p=" << OptTime(r.ReversalDetectPS)
                << " roll=" << OptTime(r.ReversalDetectRollS)
                << " ground_course=" << OptTime(r.ReversalDetectCourseS) << '\n';
        }
        out << "  ranges roll_reference_rad=[" << Num(r.RollRefMin) << ',' << Num(r.RollRefMax) << ']'
            << " aileron_cmd_norm=[" << Num(r.AileronMin) << ',' << Num(r.AileronMax) << ']'
            << " roll_rad=[" << Num(r.RollMin) << ',' << Num(r.RollMax) << ']'
            << " rudder_cmd_abs_max=" << Num(r.RudderAbsMax)
            << " settled_roll_rad=" << Num(r.SteadyStateRollRad)
            << " coordinated_turn_roll_rad=" << Num(r.RequiredCurvatureRollRad) << '\n';
        out << "  longitudinal_hold max_abs_altitude_error_m=" << Num(r.MaxAbsAltitudeErrorM)
            << " max_abs_eas_error_mps=" << Num(r.MaxAbsEasErrorMps) << '\n';
        out << "  diagnostics roll_reference_saturated_frames=" << r.RollSaturatedFrames
            << " aileron_saturated_frames=" << r.AileronSaturatedFrames << '\n';
        out << "  quality non_finite_states=" << r.NonFiniteStates
            << " unexpected_wow=" << r.UnexpectedWow
            << " configuration_violations=" << r.ConfigViolations
            << " degenerate_path_samples=" << r.InvalidPathSamples
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
    out << "isolation ue_world_loaded=0 game_pawns_searched=0 active_connection=0 "
           "udp_bt_bridge_access=0 legacy_writer_access=0 surface_position_direct_writes=0 "
           "aerodynamic_property_direct_writes=0 test_owned_fgfdmexec_only=1\n";
    out << "checks=" << audit.Checks << " failures=" << audit.Failures << '\n';
    for (const std::string &m : audit.FailureMessages) out << "  failure: " << m << '\n';

    const std::string summary = out.str();
    std::ofstream sf(argv[4]);
    sf << summary;
    std::fputs(summary.c_str(), stdout);
    std::printf("NPFG_F16_LATERAL_CLOSED_LOOP_V2_RESULT=%s\n", audit.Failures == 0 ? "PASS" : "FAIL");
    return audit.Failures == 0 ? 0 : 1;
}
