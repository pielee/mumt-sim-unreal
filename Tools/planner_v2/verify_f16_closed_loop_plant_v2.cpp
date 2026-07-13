#include "FGFDMExec.h"
#include "initialization/FGTrim.h"
#include "models/FGAuxiliary.h"
#include "models/FGFCS.h"
#include "models/FGGroundReactions.h"
#include "models/FGMassBalance.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"
#include "simgear/misc/sg_path.hxx"

#include "FormationControlV2/F16StickAdapterV2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using FormationControlV2::F16StickAdapterV2;
using FormationControlV2::FF16StickCommandV2;
using FormationControlV2::FF16StickConfigV2;
using FormationControlV2::FF16StickInputV2;

constexpr double kFdmDtS = 1.0 / 120.0;
constexpr double kControllerDtS = 1.0 / 60.0;
constexpr double kFtToM = 0.3048;
constexpr double kKnotToMps = 0.5144444444444444;
constexpr double kInitialLatitudeDeg = 47.0;
constexpr double kInitialLongitudeDeg = -122.0;
constexpr double kInitialAltitudeFt = 10000.0;
constexpr double kInitialTasMps = 220.0;
constexpr double kInitialHeadingDeg = 90.0;
constexpr double kStepRad = 0.05;
constexpr double kAdapterPrimeS = 0.5;
constexpr double kSettleS = 5.0;
constexpr double kNumericalAngleFloorRad = 1.0e-7;
constexpr double kNumericalRateFloorRadps = 1.0e-7;
constexpr double kNumericalSurfaceFloorRad = 1.0e-7;
constexpr double kNoiseMultiplier = 5.0;
constexpr double kFiniteLimit = 1.0e12;

bool Finite(double value)
{
    return std::isfinite(value) && std::abs(value) < kFiniteLimit;
}

struct FPlantState {
    double TimeS{};
    double RollRad{}, PitchRad{}, YawRad{};
    double PRadps{}, QRadps{}, RRadps{};
    double AltitudeAslM{}, AltitudeAglM{}, ClimbRateMps{};
    double EasMps{}, TasMps{}, GroundSpeedMps{};
    double AlphaRad{}, BetaRad{}, NormalLoadFactor{};
    double ElevatorPosRad{}, AileronLeftPosRad{}, AileronRightPosRad{}, ThrottlePos{};
    bool bWow{};

    bool IsFinite() const
    {
        const std::array<double, 20> values{
            TimeS, RollRad, PitchRad, YawRad, PRadps, QRadps, RRadps,
            AltitudeAslM, AltitudeAglM, ClimbRateMps, EasMps, TasMps,
            GroundSpeedMps, AlphaRad, BetaRad, NormalLoadFactor,
            ElevatorPosRad, AileronLeftPosRad, AileronRightPosRad, ThrottlePos};
        return std::all_of(values.begin(), values.end(), Finite);
    }
};

struct FWriteCounters {
    std::uint64_t AileronWrites{}, ElevatorWrites{}, RudderWrites{}, ThrottleWrites{};
    std::uint64_t CommandFrames{}, FdmRuns{}, ControllerUpdates{}, ExternalWriteAttempts{};
};

struct FTrimResult {
    bool bAttempted{}, bSuccess{};
    double PitchRad{}, RollRad{}, P{}, Q{}, R{};
    double EasMps{}, TasMps{}, AltitudeM{}, ThrottleCmd{}, ThrottlePos{};
    double ElevatorCmd{}, ElevatorPosRad{}, AileronCmd{}, AileronPosRad{};
    double MassSlugs{}, FuelLb{};
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
        if (!Exec->LoadModel("f16")) {
            failure = "LoadModel(f16) failed";
            return false;
        }
        Exec->Setdt(kFdmDtS);

        // This airborne point is inherited from the already-running host seam: TAS 220 m/s at
        // 10,000 ft is well clear of its runway/low-speed regime. It is a C1 test condition only,
        // not a TECS min/trim/max airspeed recommendation.
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
        if (!Exec->RunIC()) {
            failure = "RunIC failed";
            return false;
        }

        Fcs = Exec->GetFCS();
        Propagate = Exec->GetPropagate();
        Auxiliary = Exec->GetAuxiliary();
        Propulsion = Exec->GetPropulsion();
        Ground = Exec->GetGroundReactions();
        MassBalance = Exec->GetMassBalance();
        if (!Fcs || !Propagate || !Auxiliary || !Propulsion || !Ground || !MassBalance) {
            failure = "required JSBSim model pointer is null";
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

        Trim.bAttempted = true;
        JSBSim::FGTrim trim(Exec.get(), JSBSim::tFull);
        Trim.bSuccess = trim.DoTrim();
        if (!Trim.bSuccess) {
            trim.Report();
            trim.TrimStats();
            failure = "FGTrim(tFull).DoTrim() failed; silent warm-up fallback is forbidden";
            return false;
        }

        Trim.PitchRad = Propagate->GetEuler(JSBSim::FGJSBBase::eTht);
        Trim.RollRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePhi);
        Trim.P = Propagate->GetPQR(1);
        Trim.Q = Propagate->GetPQR(2);
        Trim.R = Propagate->GetPQR(3);
        Trim.EasMps = Auxiliary->GetVequivalentKTS() * kKnotToMps;
        Trim.TasMps = Auxiliary->GetVt() * kFtToM;
        Trim.AltitudeM = Propagate->GetAltitudeASL() * kFtToM;
        Trim.ThrottleCmd = Fcs->GetThrottleCmd(0);
        Trim.ThrottlePos = Fcs->GetThrottlePos(0);
        Trim.ElevatorCmd = Fcs->GetDeCmd();
        Trim.ElevatorPosRad = Fcs->GetDePos(JSBSim::ofRad);
        Trim.AileronCmd = Fcs->GetDaCmd();
        Trim.AileronPosRad = Fcs->GetDaLPos(JSBSim::ofRad);
        Trim.MassSlugs = MassBalance->GetMass();
        Trim.FuelLb = Exec->GetPropertyValue("propulsion/total-fuel-lbs");
        BaselinePitchRefRad = Trim.PitchRad;
        BaselineRollRefRad = Trim.RollRad;
        BaselineThrottle = std::clamp(Trim.ThrottleCmd, 0.0, 1.0);

        if (!Read().IsFinite() || !Finite(BaselineThrottle)) {
            failure = "non-finite state after successful trim";
            return false;
        }
        return true;
    }

    FPlantState Read() const
    {
        FPlantState s{};
        s.TimeS = Exec->GetSimTime();
        s.RollRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePhi);
        s.PitchRad = Propagate->GetEuler(JSBSim::FGJSBBase::eTht);
        s.YawRad = Propagate->GetEuler(JSBSim::FGJSBBase::ePsi);
        s.PRadps = Propagate->GetPQR(1);
        s.QRadps = Propagate->GetPQR(2);
        s.RRadps = Propagate->GetPQR(3);
        s.AltitudeAslM = Propagate->GetAltitudeASL() * kFtToM;
        s.AltitudeAglM = Propagate->GetDistanceAGL() * kFtToM;
        s.ClimbRateMps = Propagate->Gethdot() * kFtToM;
        s.EasMps = Auxiliary->GetVequivalentKTS() * kKnotToMps;
        s.TasMps = Auxiliary->GetVt() * kFtToM;
        s.GroundSpeedMps = Auxiliary->GetVground() * kFtToM;
        s.AlphaRad = Auxiliary->Getalpha();
        s.BetaRad = Auxiliary->Getbeta();
        s.NormalLoadFactor = Auxiliary->GetNlf();
        s.bWow = Ground->GetWOW();
        s.ElevatorPosRad = Fcs->GetDePos(JSBSim::ofRad);
        s.AileronLeftPosRad = Fcs->GetDaLPos(JSBSim::ofRad);
        s.AileronRightPosRad = Fcs->GetDaRPos(JSBSim::ofRad);
        s.ThrottlePos = Fcs->GetThrottlePos(0);
        return s;
    }

    bool WriteOwned(const FF16StickCommandV2 &cmd)
    {
        if (Exec.get() != OwnedIdentity) {
            ++Counters.ExternalWriteAttempts;
            return false;
        }
        Fcs->SetDaCmd(cmd.AileronCmdNorm); ++Counters.AileronWrites;
        // Plugin CopyToJSBSim passes elevator directly and flips only rudder.
        Fcs->SetDeCmd(cmd.ElevatorCmdNorm); ++Counters.ElevatorWrites;
        Fcs->SetDrCmd(-cmd.RudderCmdNorm); ++Counters.RudderWrites;
        Fcs->SetThrottleCmd(0, cmd.ThrottleCmdNorm); ++Counters.ThrottleWrites;
        ++Counters.CommandFrames;
        return true;
    }

    FF16StickCommandV2 GetTrimCommand() const
    {
        FF16StickCommandV2 trimCommand{};
        trimCommand.AileronCmdNorm = Trim.AileronCmd;
        trimCommand.ElevatorCmdNorm = Trim.ElevatorCmd;
        trimCommand.RudderCmdNorm = 0.0;
        trimCommand.ThrottleCmdNorm = BaselineThrottle;
        trimCommand.bValid = true;
        return trimCommand;
    }

    bool Run()
    {
        ++Counters.FdmRuns;
        return Exec->Run();
    }

    void CountControllerUpdate() { ++Counters.ControllerUpdates; }

    const FTrimResult &GetTrim() const { return Trim; }
    const FWriteCounters &GetCounters() const { return Counters; }
    double GetBaselinePitchRef() const { return BaselinePitchRefRad; }
    double GetBaselineRollRef() const { return BaselineRollRefRad; }
    double GetBaselineThrottle() const { return BaselineThrottle; }

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
    FWriteCounters Counters{};
    double BaselinePitchRefRad{}, BaselineRollRefRad{}, BaselineThrottle{};
};

enum class ECaseKind { Baseline, PositiveStep, NegativeStep, Recovery, PiReversal };

struct FCaseDefinition {
    std::string Name;
    ECaseKind Kind{};
    double DurationS{};
    std::uint32_t ResetGeneration{};
};

struct FTraceRow {
    std::string CaseName, Phase;
    std::uint32_t ResetGeneration{};
    FPlantState State{};
    double PitchRef{}, RollRef{};
    FF16StickCommandV2 Command{};
};

struct FCaseMetrics {
    std::string Name;
    bool bTrimAttempted{}, bTrimSuccess{};
    bool bPassed{true};
    std::uint64_t Checks{}, Failures{}, FdmRunFailures{}, NonFiniteStates{};
    std::uint64_t CommandRangeViolations{}, CommandSlewViolations{}, UnexpectedWow{};
    std::uint64_t SaturationFrames{};
    double PitchMin{std::numeric_limits<double>::infinity()}, PitchMax{-std::numeric_limits<double>::infinity()};
    double RollMin{std::numeric_limits<double>::infinity()}, RollMax{-std::numeric_limits<double>::infinity()};
    double PMin{std::numeric_limits<double>::infinity()}, PMax{-std::numeric_limits<double>::infinity()};
    double QMin{std::numeric_limits<double>::infinity()}, QMax{-std::numeric_limits<double>::infinity()};
    double RMin{std::numeric_limits<double>::infinity()}, RMax{-std::numeric_limits<double>::infinity()};
    double AltitudeMin{std::numeric_limits<double>::infinity()}, AltitudeMax{-std::numeric_limits<double>::infinity()};
    double EasMin{std::numeric_limits<double>::infinity()}, EasMax{-std::numeric_limits<double>::infinity()};
    double TasMin{std::numeric_limits<double>::infinity()}, TasMax{-std::numeric_limits<double>::infinity()};
    double ElevatorCmdMin{std::numeric_limits<double>::infinity()}, ElevatorCmdMax{-std::numeric_limits<double>::infinity()};
    double ElevatorPosMin{std::numeric_limits<double>::infinity()}, ElevatorPosMax{-std::numeric_limits<double>::infinity()};
    double AileronCmdMin{std::numeric_limits<double>::infinity()}, AileronCmdMax{-std::numeric_limits<double>::infinity()};
    double ThrottleMin{std::numeric_limits<double>::infinity()}, ThrottleMax{-std::numeric_limits<double>::infinity()};
    double BaselinePitchRef{}, BaselineRollRef{}, BaselineThrottle{};
    double InitialPitch{}, InitialQ{}, InitialElevatorPos{};
    double FirstPitchDirectionTime{-1.0}, FirstQDirectionTime{-1.0}, FirstSurfaceDirectionTime{-1.0};
    double PeakPitchDelta{}, PeakQDirection{}, PeakSurfaceDelta{}, Iae{}, FinalPitchError{};
    double RemovalTime{-1.0}, ErrorAtRemoval{}, MinimumTailError{std::numeric_limits<double>::infinity()};
    double FirstRecoveryTime{-1.0}, ReversalTime{-1.0}, CommandReversalTime{-1.0};
    double QReversalTime{-1.0}, PitchReversalTime{-1.0}, SaturationExitTime{-1.0};
    double PitchJitter{}, QJitter{}, SurfaceJitter{}, AltitudeDrift{}, EasDrift{}, TasDrift{};
    double MeasurePitchMin{std::numeric_limits<double>::infinity()}, MeasurePitchMax{-std::numeric_limits<double>::infinity()};
    double MeasureQMin{std::numeric_limits<double>::infinity()}, MeasureQMax{-std::numeric_limits<double>::infinity()};
    double MeasureSurfaceMin{std::numeric_limits<double>::infinity()}, MeasureSurfaceMax{-std::numeric_limits<double>::infinity()};
    double MeasureAltitudeMin{std::numeric_limits<double>::infinity()}, MeasureAltitudeMax{-std::numeric_limits<double>::infinity()};
    double MeasureEasMin{std::numeric_limits<double>::infinity()}, MeasureEasMax{-std::numeric_limits<double>::infinity()};
    double MeasureTasMin{std::numeric_limits<double>::infinity()}, MeasureTasMax{-std::numeric_limits<double>::infinity()};
    FTrimResult Trim{};
    FWriteCounters Writes{};
};

struct FThresholds {
    double PitchRad{kNumericalAngleFloorRad};
    double QRadps{kNumericalRateFloorRadps};
    double SurfaceRad{kNumericalSurfaceFloorRad};
};

struct FAudit {
    std::uint64_t Checks{}, Failures{};
    std::vector<std::string> FailureMessages;

    void Check(bool condition, const std::string &message, FCaseMetrics *metrics = nullptr)
    {
        ++Checks;
        if (metrics) ++metrics->Checks;
        if (!condition) {
            ++Failures;
            if (metrics) { ++metrics->Failures; metrics->bPassed = false; }
            FailureMessages.push_back(message);
        }
    }
};

std::pair<std::string, double> PhaseAndReference(const FCaseDefinition &definition,
                                                 double timeS, double baselinePitch)
{
    if (timeS < kAdapterPrimeS) return {"adapter_prime", baselinePitch};
    if (timeS < kSettleS) return {"settle", baselinePitch};
    const double active = timeS - kSettleS;
    switch (definition.Kind) {
    case ECaseKind::Baseline:
        return {"measure", baselinePitch};
    case ECaseKind::PositiveStep:
        return {"positive_step", baselinePitch + kStepRad};
    case ECaseKind::NegativeStep:
        return {"negative_step", baselinePitch - kStepRad};
    case ECaseKind::Recovery:
        return active < 5.0 ? std::make_pair(std::string("positive_step"), baselinePitch + kStepRad)
                            : std::make_pair(std::string("recovery"), baselinePitch);
    case ECaseKind::PiReversal:
        return active < 8.0 ? std::make_pair(std::string("positive_memory"), baselinePitch + kStepRad)
                            : std::make_pair(std::string("negative_reversal"), baselinePitch - kStepRad);
    }
    return {"invalid", baselinePitch};
}

void UpdateRanges(FCaseMetrics &m, const FPlantState &s, const FF16StickCommandV2 &cmd)
{
    m.PitchMin = std::min(m.PitchMin, s.PitchRad); m.PitchMax = std::max(m.PitchMax, s.PitchRad);
    m.RollMin = std::min(m.RollMin, s.RollRad); m.RollMax = std::max(m.RollMax, s.RollRad);
    m.PMin = std::min(m.PMin, s.PRadps); m.PMax = std::max(m.PMax, s.PRadps);
    m.QMin = std::min(m.QMin, s.QRadps); m.QMax = std::max(m.QMax, s.QRadps);
    m.RMin = std::min(m.RMin, s.RRadps); m.RMax = std::max(m.RMax, s.RRadps);
    m.AltitudeMin = std::min(m.AltitudeMin, s.AltitudeAslM); m.AltitudeMax = std::max(m.AltitudeMax, s.AltitudeAslM);
    m.EasMin = std::min(m.EasMin, s.EasMps); m.EasMax = std::max(m.EasMax, s.EasMps);
    m.TasMin = std::min(m.TasMin, s.TasMps); m.TasMax = std::max(m.TasMax, s.TasMps);
    m.ElevatorCmdMin = std::min(m.ElevatorCmdMin, cmd.ElevatorCmdNorm); m.ElevatorCmdMax = std::max(m.ElevatorCmdMax, cmd.ElevatorCmdNorm);
    m.ElevatorPosMin = std::min(m.ElevatorPosMin, s.ElevatorPosRad); m.ElevatorPosMax = std::max(m.ElevatorPosMax, s.ElevatorPosRad);
    m.AileronCmdMin = std::min(m.AileronCmdMin, cmd.AileronCmdNorm); m.AileronCmdMax = std::max(m.AileronCmdMax, cmd.AileronCmdNorm);
    m.ThrottleMin = std::min(m.ThrottleMin, cmd.ThrottleCmdNorm); m.ThrottleMax = std::max(m.ThrottleMax, cmd.ThrottleCmdNorm);
}

FCaseMetrics RunCase(const std::string &root, const FCaseDefinition &definition,
                     const FThresholds *thresholds, std::vector<FTraceRow> &trace, FAudit &audit)
{
    FCaseMetrics m{};
    m.Name = definition.Name;
    FOwnedF16Plant plant;
    std::string initFailure;
    const bool initialized = plant.Initialize(root, initFailure);
    audit.Check(initialized, definition.Name + ": " + initFailure, &m);
    if (!initialized) return m;

    m.Trim = plant.GetTrim();
    m.bTrimAttempted = m.Trim.bAttempted;
    m.bTrimSuccess = m.Trim.bSuccess;
    m.BaselinePitchRef = plant.GetBaselinePitchRef();
    m.BaselineRollRef = plant.GetBaselineRollRef();
    m.BaselineThrottle = plant.GetBaselineThrottle();

    F16StickAdapterV2 stick;
    const FF16StickConfigV2 config{};
    stick.Reset(definition.ResetGeneration);
    // The first Stick frame is intentionally invalid (ResetFrame), and its throttle slew anchor is
    // zero. Keep the exact trim command on the owned FDM while the adapter advances for 0.5 s, then
    // hand over only after its throttle output has reached the trim command without an FCS jump.
    FF16StickCommandV2 heldCommand = plant.GetTrimCommand();
    bool haveHeldCommand = true;
    FF16StickCommandV2 previousValid{};
    bool havePreviousValid = false;
    FF16StickCommandV2 previousApplied = heldCommand;
    bool havePreviousApplied = true;
    const int steps = static_cast<int>(std::llround(definition.DurationS / kFdmDtS));
    bool capturedInitial = false;
    bool sawSaturation = false;
    int desiredDirection = 0;

    for (int step = 0; step < steps; ++step) {
        FPlantState before = plant.Read();
        if (!before.IsFinite()) ++m.NonFiniteStates;
        audit.Check(before.IsFinite(), definition.Name + ": non-finite pre-step state", &m);
        if (before.bWow) ++m.UnexpectedWow;

        const auto phaseRef = PhaseAndReference(definition, before.TimeS, m.BaselinePitchRef);
        const std::string &phase = phaseRef.first;
        const double pitchRef = phaseRef.second;

        const bool controllerFrame = (step % 2) == 0;
        if (controllerFrame) {
            FF16StickInputV2 input{};
            input.RollReferenceRad = m.BaselineRollRef;
            input.PitchReferenceRad = pitchRef;
            input.ThrottleReferenceNorm = m.BaselineThrottle;
            input.bGuidanceValid = true;
            input.CurrentRollRad = before.RollRad;
            input.CurrentPitchRad = before.PitchRad;
            input.bAttitudeValid = true;
            input.BodyRollRateRadps = before.PRadps;
            input.BodyPitchRateRadps = before.QRadps;
            input.BodyYawRateRadps = before.RRadps;
            input.bBodyRatesValid = true;
            input.AlphaRad = before.AlphaRad;
            input.BetaRad = before.BetaRad;
            input.bAlphaBetaValid = true;
            input.EasMps = before.EasMps;
            input.TasMps = before.TasMps;
            input.bAirspeedValid = true;
            input.SimulationTimeS = before.TimeS;
            input.DtS = kControllerDtS;
            input.ResetGeneration = definition.ResetGeneration;
            plant.CountControllerUpdate();
            FF16StickCommandV2 command = stick.Update(input, config);
            if (command.bValid) {
                const bool rangeOk = command.AileronCmdNorm >= -1.0 - 1e-12 && command.AileronCmdNorm <= 1.0 + 1e-12 &&
                                     command.ElevatorCmdNorm >= config.ElevatorMin - 1e-12 && command.ElevatorCmdNorm <= config.ElevatorMax + 1e-12 &&
                                     command.RudderCmdNorm >= -1.0 - 1e-12 && command.RudderCmdNorm <= 1.0 + 1e-12 &&
                                     command.ThrottleCmdNorm >= config.ThrottleMin - 1e-12 && command.ThrottleCmdNorm <= config.ThrottleMax + 1e-12;
                if (!rangeOk) ++m.CommandRangeViolations;
                audit.Check(rangeOk, definition.Name + ": command range violation", &m);
                if (havePreviousValid) {
                    const bool slewOk = std::abs(command.AileronCmdNorm - previousValid.AileronCmdNorm) <= config.AileronSlewPerS * kControllerDtS + 1e-12 &&
                                        std::abs(command.ElevatorCmdNorm - previousValid.ElevatorCmdNorm) <= config.ElevatorSlewPerS * kControllerDtS + 1e-12 &&
                                        std::abs(command.RudderCmdNorm - previousValid.RudderCmdNorm) <= config.RudderSlewPerS * kControllerDtS + 1e-12 &&
                                        std::abs(command.ThrottleCmdNorm - previousValid.ThrottleCmdNorm) <= config.ThrottleSlewPerS * kControllerDtS + 1e-12;
                    if (!slewOk) ++m.CommandSlewViolations;
                    audit.Check(slewOk, definition.Name + ": command slew violation", &m);
                }
                previousValid = command;
                havePreviousValid = true;
                if (before.TimeS >= kAdapterPrimeS) {
                    const bool appliedSlewOk = !havePreviousApplied ||
                        (std::abs(command.AileronCmdNorm - previousApplied.AileronCmdNorm) <= config.AileronSlewPerS * kControllerDtS + 1e-12 &&
                         std::abs(command.ElevatorCmdNorm - previousApplied.ElevatorCmdNorm) <= config.ElevatorSlewPerS * kControllerDtS + 1e-12 &&
                         std::abs(command.RudderCmdNorm - previousApplied.RudderCmdNorm) <= config.RudderSlewPerS * kControllerDtS + 1e-12 &&
                         std::abs(command.ThrottleCmdNorm - previousApplied.ThrottleCmdNorm) <= config.ThrottleSlewPerS * kControllerDtS + 1e-12);
                    if (!appliedSlewOk) ++m.CommandSlewViolations;
                    audit.Check(appliedSlewOk, definition.Name + ": applied FCS command slew violation", &m);
                    heldCommand = command;
                    previousApplied = command;
                    havePreviousApplied = true;
                }
            } else {
                audit.Check(command.FailureReason == FormationControlV2::EF16StickFailureV2::ResetFrame && step == 0,
                            definition.Name + ": unexpected invalid Stick output", &m);
            }
        }

        // A 60 Hz command write is held by JSBSim for this and the following 120 Hz FDM step.
        if (controllerFrame)
            audit.Check(haveHeldCommand && plant.WriteOwned(heldCommand), definition.Name + ": owned command write failed", &m);
        if (!plant.Run()) {
            ++m.FdmRunFailures;
            audit.Check(false, definition.Name + ": FGFDMExec::Run failed", &m);
            break;
        }

        const FPlantState after = plant.Read();
        if (!after.IsFinite()) ++m.NonFiniteStates;
        audit.Check(after.IsFinite(), definition.Name + ": non-finite post-step state", &m);
        if (after.bWow) ++m.UnexpectedWow;
        UpdateRanges(m, after, heldCommand);
        if (definition.Kind == ECaseKind::Baseline && phase == "measure") {
            m.MeasurePitchMin = std::min(m.MeasurePitchMin, after.PitchRad);
            m.MeasurePitchMax = std::max(m.MeasurePitchMax, after.PitchRad);
            m.MeasureQMin = std::min(m.MeasureQMin, after.QRadps);
            m.MeasureQMax = std::max(m.MeasureQMax, after.QRadps);
            m.MeasureSurfaceMin = std::min(m.MeasureSurfaceMin, after.ElevatorPosRad);
            m.MeasureSurfaceMax = std::max(m.MeasureSurfaceMax, after.ElevatorPosRad);
            m.MeasureAltitudeMin = std::min(m.MeasureAltitudeMin, after.AltitudeAslM);
            m.MeasureAltitudeMax = std::max(m.MeasureAltitudeMax, after.AltitudeAslM);
            m.MeasureEasMin = std::min(m.MeasureEasMin, after.EasMps);
            m.MeasureEasMax = std::max(m.MeasureEasMax, after.EasMps);
            m.MeasureTasMin = std::min(m.MeasureTasMin, after.TasMps);
            m.MeasureTasMax = std::max(m.MeasureTasMax, after.TasMps);
        }
        trace.push_back({definition.Name, phase, definition.ResetGeneration, after, pitchRef,
                         m.BaselineRollRef, heldCommand});

        const bool activePhase = phase != "adapter_prime" && phase != "settle" && phase != "measure";
        if (activePhase && !capturedInitial) {
            m.InitialPitch = after.PitchRad;
            m.InitialQ = after.QRadps;
            m.InitialElevatorPos = after.ElevatorPosRad;
            capturedInitial = true;
        }
        if (activePhase) {
            m.Iae += std::abs(pitchRef - after.PitchRad) * kFdmDtS;
            if (phase == "positive_step" || phase == "positive_memory") desiredDirection = 1;
            else if (phase == "negative_step" || phase == "negative_reversal") desiredDirection = -1;
            else desiredDirection = 0;
        }
        if (capturedInitial && thresholds && desiredDirection != 0) {
            const double pitchDelta = after.PitchRad - m.InitialPitch;
            const double surfaceDelta = after.ElevatorPosRad - m.InitialElevatorPos;
            m.PeakPitchDelta = desiredDirection > 0 ? std::max(m.PeakPitchDelta, pitchDelta)
                                                    : std::min(m.PeakPitchDelta, pitchDelta);
            m.PeakQDirection = desiredDirection > 0 ? std::max(m.PeakQDirection, after.QRadps)
                                                    : std::min(m.PeakQDirection, after.QRadps);
            m.PeakSurfaceDelta = desiredDirection > 0 ? std::min(m.PeakSurfaceDelta, surfaceDelta)
                                                      : std::max(m.PeakSurfaceDelta, surfaceDelta);
            if (m.FirstPitchDirectionTime < 0.0 && desiredDirection * pitchDelta > thresholds->PitchRad)
                m.FirstPitchDirectionTime = after.TimeS;
            if (m.FirstQDirectionTime < 0.0 && desiredDirection * after.QRadps > thresholds->QRadps)
                m.FirstQDirectionTime = after.TimeS;
            // Negative elevator surface is nose-up, so the expected surface sign is -desiredDirection.
            if (m.FirstSurfaceDirectionTime < 0.0 && -desiredDirection * surfaceDelta > thresholds->SurfaceRad)
                m.FirstSurfaceDirectionTime = after.TimeS;
        }

        const bool saturated = haveHeldCommand &&
            (heldCommand.ElevatorCmdNorm <= config.ElevatorMin + 1e-9 || heldCommand.ElevatorCmdNorm >= config.ElevatorMax - 1e-9);
        if (saturated) { ++m.SaturationFrames; sawSaturation = true; }
        else if (sawSaturation && m.SaturationExitTime < 0.0) m.SaturationExitTime = after.TimeS;

        if (definition.Kind == ECaseKind::Recovery && phase == "recovery") {
            if (m.RemovalTime < 0.0) {
                m.RemovalTime = after.TimeS;
                m.ErrorAtRemoval = std::abs(m.BaselinePitchRef - after.PitchRad);
            }
            const double tailError = std::abs(m.BaselinePitchRef - after.PitchRad);
            m.MinimumTailError = std::min(m.MinimumTailError, tailError);
            if (thresholds && m.FirstRecoveryTime < 0.0 && after.QRadps < -thresholds->QRadps)
                m.FirstRecoveryTime = after.TimeS;
        }
        if (definition.Kind == ECaseKind::PiReversal && phase == "negative_reversal") {
            if (m.ReversalTime < 0.0) m.ReversalTime = after.TimeS;
            if (haveHeldCommand && m.CommandReversalTime < 0.0 && heldCommand.ElevatorCmdNorm > 0.0)
                m.CommandReversalTime = after.TimeS;
            if (thresholds && m.QReversalTime < 0.0 && after.QRadps < -thresholds->QRadps)
                m.QReversalTime = after.TimeS;
            if (thresholds && m.PitchReversalTime < 0.0 && after.PitchRad < m.InitialPitch - thresholds->PitchRad)
                m.PitchReversalTime = after.TimeS;
        }
        m.FinalPitchError = pitchRef - after.PitchRad;
    }

    m.Writes = plant.GetCounters();
    audit.Check(m.FdmRunFailures == 0, definition.Name + ": FDM run failures", &m);
    audit.Check(m.NonFiniteStates == 0, definition.Name + ": non-finite state count", &m);
    audit.Check(m.CommandRangeViolations == 0, definition.Name + ": command range count", &m);
    audit.Check(m.CommandSlewViolations == 0, definition.Name + ": command slew count", &m);
    audit.Check(m.UnexpectedWow == 0, definition.Name + ": unexpected WOW/contact", &m);
    audit.Check(m.Writes.ExternalWriteAttempts == 0, definition.Name + ": external write attempt", &m);
    audit.Check(m.Writes.AileronWrites == m.Writes.CommandFrames &&
                m.Writes.ElevatorWrites == m.Writes.CommandFrames &&
                m.Writes.RudderWrites == m.Writes.CommandFrames &&
                m.Writes.ThrottleWrites == m.Writes.CommandFrames,
                definition.Name + ": incomplete owned command frame", &m);

    if (definition.Kind == ECaseKind::Baseline) {
        m.PitchJitter = m.MeasurePitchMax - m.MeasurePitchMin;
        m.QJitter = m.MeasureQMax - m.MeasureQMin;
        m.SurfaceJitter = m.MeasureSurfaceMax - m.MeasureSurfaceMin;
        m.AltitudeDrift = m.MeasureAltitudeMax - m.MeasureAltitudeMin;
        m.EasDrift = m.MeasureEasMax - m.MeasureEasMin;
        m.TasDrift = m.MeasureTasMax - m.MeasureTasMin;
        audit.Check(Finite(m.PitchJitter) && Finite(m.QJitter) && Finite(m.SurfaceJitter) &&
                    Finite(m.AltitudeDrift) && Finite(m.EasDrift) && Finite(m.TasDrift),
                    definition.Name + ": baseline measurement window is empty or non-finite", &m);
    } else if (thresholds) {
        if (definition.Kind == ECaseKind::PositiveStep || definition.Kind == ECaseKind::NegativeStep) {
            audit.Check(m.FirstPitchDirectionTime >= 0.0, definition.Name + ": no directional pitch response", &m);
            audit.Check(m.FirstQDirectionTime >= 0.0, definition.Name + ": no directional Q response", &m);
            audit.Check(m.FirstSurfaceDirectionTime >= 0.0, definition.Name + ": no directional FCS surface response", &m);
        } else if (definition.Kind == ECaseKind::Recovery) {
            audit.Check(m.RemovalTime >= 0.0 && m.MinimumTailError < m.ErrorAtRemoval,
                        definition.Name + ": pitch error did not decrease after reference removal", &m);
            audit.Check(m.FirstRecoveryTime >= 0.0, definition.Name + ": no recovery-direction Q response", &m);
            audit.Check(m.SaturationFrames == 0 || m.SaturationExitTime >= 0.0,
                        definition.Name + ": command remained hard saturated", &m);
        } else if (definition.Kind == ECaseKind::PiReversal) {
            audit.Check(m.CommandReversalTime >= 0.0, definition.Name + ": command did not reverse", &m);
            audit.Check(m.QReversalTime >= 0.0, definition.Name + ": Q did not reverse", &m);
            audit.Check(m.PitchReversalTime >= 0.0, definition.Name + ": pitch response did not reverse", &m);
            audit.Check(m.SaturationFrames == 0 || m.SaturationExitTime >= 0.0,
                        definition.Name + ": hard saturation did not clear", &m);
        }
    }
    return m;
}

FThresholds DeriveThresholds(const std::vector<FCaseMetrics> &baselines)
{
    FThresholds thresholds{};
    double pitchNoise = 0.0, qNoise = 0.0, surfaceNoise = 0.0;
    for (const FCaseMetrics &m : baselines) {
        pitchNoise = std::max(pitchNoise, m.PitchJitter);
        qNoise = std::max(qNoise, m.QJitter);
        surfaceNoise = std::max(surfaceNoise, m.SurfaceJitter);
    }
    thresholds.PitchRad = std::max(kNumericalAngleFloorRad, kNoiseMultiplier * pitchNoise);
    thresholds.QRadps = std::max(kNumericalRateFloorRadps, kNoiseMultiplier * qNoise);
    thresholds.SurfaceRad = std::max(kNumericalSurfaceFloorRad, kNoiseMultiplier * surfaceNoise);
    return thresholds;
}

void WriteCsv(const std::string &path, const std::vector<FTraceRow> &trace)
{
    std::ofstream out(path);
    out << "case,time_s,phase,pitch_ref_rad,pitch_rad,q_radps,roll_ref_rad,roll_rad,p_radps,"
           "yaw_rad,r_radps,elevator_cmd_norm,elevator_pos_rad,aileron_cmd_norm,aileron_left_pos_rad,"
           "aileron_right_pos_rad,rudder_cmd_norm,throttle_cmd_norm,throttle_pos,altitude_asl_m,"
           "altitude_agl_m,climb_rate_mps,eas_mps,tas_mps,ground_speed_mps,alpha_rad,beta_rad,"
           "normal_load_factor,wow,reset_generation\n";
    out << std::fixed << std::setprecision(12);
    for (const FTraceRow &r : trace) {
        out << r.CaseName << ',' << r.State.TimeS << ',' << r.Phase << ','
            << r.PitchRef << ',' << r.State.PitchRad << ',' << r.State.QRadps << ','
            << r.RollRef << ',' << r.State.RollRad << ',' << r.State.PRadps << ','
            << r.State.YawRad << ',' << r.State.RRadps << ','
            << r.Command.ElevatorCmdNorm << ',' << r.State.ElevatorPosRad << ','
            << r.Command.AileronCmdNorm << ',' << r.State.AileronLeftPosRad << ','
            << r.State.AileronRightPosRad << ',' << r.Command.RudderCmdNorm << ','
            << r.Command.ThrottleCmdNorm << ',' << r.State.ThrottlePos << ','
            << r.State.AltitudeAslM << ',' << r.State.AltitudeAglM << ',' << r.State.ClimbRateMps << ','
            << r.State.EasMps << ',' << r.State.TasMps << ',' << r.State.GroundSpeedMps << ','
            << r.State.AlphaRad << ',' << r.State.BetaRad << ',' << r.State.NormalLoadFactor << ','
            << (r.State.bWow ? 1 : 0) << ',' << r.ResetGeneration << '\n';
    }
}

std::string SummaryText(const std::vector<FCaseMetrics> &metrics, const FThresholds &thresholds,
                        const FAudit &audit, std::size_t traceRows)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(12);
    out << "F16_CLOSED_LOOP_PLANT_V2\n";
    out << "initialization_mode=FGTrim_tFull trim_attempted=1 trim_success="
        << (std::all_of(metrics.begin(), metrics.end(), [](const FCaseMetrics &m) { return m.bTrimSuccess; }) ? 1 : 0)
        << " trim_iterations=not_exposed_by_current_FGTrim_API\n";
    out << "model=f16 data=Plugins/JSBSimFlightDynamicsModel/Resources/JSBSim fdm_dt_s=" << kFdmDtS
        << " controller_dt_s=" << kControllerDtS << " trace_rows=" << traceRows << '\n';
    out << "initial_altitude_ft=" << kInitialAltitudeFt << " initial_tas_mps=" << kInitialTasMps
        << " wind_mps=0 gear=up flap=clean speedbrake=retracted fuel=frozen\n";
    out << "noise_multiplier=" << kNoiseMultiplier << " numerical_angle_floor_rad=" << kNumericalAngleFloorRad
        << " numerical_rate_floor_radps=" << kNumericalRateFloorRadps
        << " numerical_surface_floor_rad=" << kNumericalSurfaceFloorRad << '\n';
    out << "direction_threshold_pitch_rad=" << thresholds.PitchRad
        << " direction_threshold_q_radps=" << thresholds.QRadps
        << " direction_threshold_surface_rad=" << thresholds.SurfaceRad << '\n';

    std::uint64_t fdmFailures = 0, nonFinite = 0, range = 0, slew = 0, wow = 0, writes = 0, external = 0;
    for (const FCaseMetrics &m : metrics) {
        fdmFailures += m.FdmRunFailures; nonFinite += m.NonFiniteStates;
        range += m.CommandRangeViolations; slew += m.CommandSlewViolations; wow += m.UnexpectedWow;
        writes += m.Writes.CommandFrames; external += m.Writes.ExternalWriteAttempts;
        out << "case=" << m.Name << " result=" << (m.bPassed ? "PASS" : "FAIL")
            << " checks=" << m.Checks << " failures=" << m.Failures
            << " trim_pitch_rad=" << m.Trim.PitchRad << " trim_roll_rad=" << m.Trim.RollRad
            << " trim_eas_mps=" << m.Trim.EasMps << " trim_tas_mps=" << m.Trim.TasMps
            << " trim_altitude_m=" << m.Trim.AltitudeM << " trim_throttle_cmd=" << m.Trim.ThrottleCmd
            << " trim_throttle_pos=" << m.Trim.ThrottlePos << " trim_elevator_pos_rad=" << m.Trim.ElevatorPosRad
            << " mass_slugs=" << m.Trim.MassSlugs << " fuel_lb=" << m.Trim.FuelLb
            << " pitch_min=" << m.PitchMin << " pitch_max=" << m.PitchMax
            << " q_min=" << m.QMin << " q_max=" << m.QMax
            << " elevator_cmd_min=" << m.ElevatorCmdMin << " elevator_cmd_max=" << m.ElevatorCmdMax
            << " elevator_pos_min=" << m.ElevatorPosMin << " elevator_pos_max=" << m.ElevatorPosMax
            << " saturation_frames=" << m.SaturationFrames
            << " first_pitch_direction_s=" << m.FirstPitchDirectionTime
            << " first_q_direction_s=" << m.FirstQDirectionTime
            << " first_surface_direction_s=" << m.FirstSurfaceDirectionTime
            << " peak_pitch_delta_rad=" << m.PeakPitchDelta << " peak_q_radps=" << m.PeakQDirection
            << " peak_surface_delta_rad=" << m.PeakSurfaceDelta << " iae_rad_s=" << m.Iae
            << " final_pitch_error_rad=" << m.FinalPitchError
            << " pitch_jitter_rad=" << m.PitchJitter << " q_jitter_radps=" << m.QJitter
            << " surface_jitter_rad=" << m.SurfaceJitter << " altitude_drift_m=" << m.AltitudeDrift
            << " eas_drift_mps=" << m.EasDrift << " tas_drift_mps=" << m.TasDrift
            << " removal_s=" << m.RemovalTime << " error_at_removal_rad=" << m.ErrorAtRemoval
            << " minimum_tail_error_rad=" << m.MinimumTailError << " recovery_direction_s=" << m.FirstRecoveryTime
            << " reversal_s=" << m.ReversalTime << " command_reversal_s=" << m.CommandReversalTime
            << " q_reversal_s=" << m.QReversalTime << " pitch_reversal_s=" << m.PitchReversalTime
            << " saturation_exit_s=" << m.SaturationExitTime
            << " controller_updates=" << m.Writes.ControllerUpdates << " fdm_runs=" << m.Writes.FdmRuns
            << " owned_command_frames=" << m.Writes.CommandFrames << " external_write_attempts=" << m.Writes.ExternalWriteAttempts
            << " integrator_state=not_exposed_by_F16StickAdapterV2\n";
    }
    out << "cases=" << metrics.size() << " checks=" << audit.Checks << " failures=" << audit.Failures
        << " fdm_run_failures=" << fdmFailures << " non_finite_states=" << nonFinite
        << " command_range_violations=" << range << " command_slew_violations=" << slew
        << " unexpected_wow=" << wow << " owned_command_frames=" << writes
        << " external_write_attempts=" << external << '\n';
    out << "production_writer_invocations=0 ue_world_loaded=0 game_pawns_searched=0 active_connection=0\n";
    for (const std::string &failure : audit.FailureMessages) out << "FAILURE: " << failure << '\n';
    return out.str();
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 4) {
        std::fprintf(stderr, "usage: %s <JSBSim-root> <trace.csv> <summary.txt>\n", argv[0]);
        return 2;
    }

    const std::string root = argv[1];
    std::vector<FTraceRow> trace;
    trace.reserve(20000);
    FAudit audit;
    std::vector<FCaseMetrics> metrics;
    std::vector<FCaseMetrics> baselines;
    const std::array<FCaseDefinition, 3> baselineCases{{
        {"baseline_1", ECaseKind::Baseline, 15.0, 1001},
        {"baseline_2", ECaseKind::Baseline, 15.0, 1002},
        {"baseline_3", ECaseKind::Baseline, 15.0, 1003}}};
    for (const FCaseDefinition &definition : baselineCases) {
        FCaseMetrics result = RunCase(root, definition, nullptr, trace, audit);
        baselines.push_back(result);
        metrics.push_back(std::move(result));
    }
    const FThresholds thresholds = DeriveThresholds(baselines);

    const std::array<FCaseDefinition, 4> responseCases{{
        {"positive_pitch_step", ECaseKind::PositiveStep, 13.0, 2001},
        {"negative_pitch_step", ECaseKind::NegativeStep, 13.0, 2002},
        {"reference_removal_recovery", ECaseKind::Recovery, 20.0, 2003},
        {"pi_memory_sustained_reversal", ECaseKind::PiReversal, 25.0, 2004}}};
    for (const FCaseDefinition &definition : responseCases)
        metrics.push_back(RunCase(root, definition, &thresholds, trace, audit));

    audit.Check(metrics.size() == 7, "case count mismatch");
    audit.Check(std::all_of(metrics.begin(), metrics.end(), [](const FCaseMetrics &m) { return m.bTrimSuccess; }),
                "not every case started from successful FGTrim(tFull)");

    WriteCsv(argv[2], trace);
    const std::string summary = SummaryText(metrics, thresholds, audit, trace.size());
    std::ofstream summaryFile(argv[3]);
    summaryFile << summary;
    std::fputs(summary.c_str(), stdout);
    return audit.Failures == 0 ? 0 : 1;
}
