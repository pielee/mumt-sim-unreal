#include "FormationControlV2/F16StickAdapterV2.h"

#include <algorithm>
#include <cmath>

namespace FormationControlV2 {
namespace {
bool Fin(double v) { return std::isfinite(v); }
// Clamp comes from PlannerV2Types.h (FormationControlV2::Clamp).
FF16StickCommandV2 Neutral(EF16StickFailureV2 reason, std::uint32_t generation, double timestamp) {
    FF16StickCommandV2 out{}; // value-initialized: all commands neutral 0, bValid false
    out.FailureReason = reason; out.ResetGeneration = generation;
    if (Fin(timestamp)) out.TimestampS = timestamp;
    return out;
}
} // namespace

void F16StickAdapterV2::ClearState()
{
    PitchIntegrator = 0.0;
    PrevAileron = PrevElevator = PrevRudder = PrevThrottle = 0.0;
    bHavePrevCommands = false;
}

void F16StickAdapterV2::Reset(std::uint32_t resetGeneration)
{
    ClearState();
    LastResetGeneration = resetGeneration;
    bInitialized = false;
    bWasPaused = false;
}

FF16StickCommandV2 F16StickAdapterV2::Update(const FF16StickInputV2 &in, const FF16StickConfigV2 &config)
{
    // --- time validity first (nothing advances on bad time) ---
    if (!Fin(in.SimulationTimeS) || !Fin(in.DtS) || in.SimulationTimeS < 0.0)
        return Neutral(EF16StickFailureV2::InvalidTime, in.ResetGeneration, 0.0);

    // --- lifecycle: first frame / reset generation change -> clear and skip one frame ---
    if (!bInitialized || in.ResetGeneration != LastResetGeneration) {
        ClearState();
        LastResetGeneration = in.ResetGeneration;
        bInitialized = true;
        bWasPaused = in.bPaused;
        return Neutral(EF16StickFailureV2::ResetFrame, in.ResetGeneration, in.SimulationTimeS);
    }

    // --- pause: freeze, no state advance ---
    if (in.bPaused) {
        bWasPaused = true;
        return Neutral(EF16StickFailureV2::Paused, in.ResetGeneration, in.SimulationTimeS);
    }
    // --- resume: clear and re-anchor from neutral, skip one frame (no command reuse) ---
    if (bWasPaused) {
        bWasPaused = false;
        ClearState();
        return Neutral(EF16StickFailureV2::ResetFrame, in.ResetGeneration, in.SimulationTimeS);
    }

    // --- abnormal dt: clear so no stale integrator/slew anchor survives ---
    if (in.DtS < config.DtMinS || in.DtS > config.DtMaxS) {
        ClearState();
        return Neutral(EF16StickFailureV2::AbnormalDt, in.ResetGeneration, in.SimulationTimeS);
    }

    // --- validity gates (invalid input -> fresh neutral invalid and cleared state) ---
    if (!in.bGuidanceValid) { ClearState(); return Neutral(EF16StickFailureV2::InvalidGuidance, in.ResetGeneration, in.SimulationTimeS); }
    if (!in.bAttitudeValid) { ClearState(); return Neutral(EF16StickFailureV2::InvalidAttitude, in.ResetGeneration, in.SimulationTimeS); }
    const bool needBodyRates = (config.RollRateDampingGain != 0.0) || (config.PitchRateDampingGain != 0.0) ||
                               (config.YawRateDampingGain != 0.0);
    if (needBodyRates && !in.bBodyRatesValid) { ClearState(); return Neutral(EF16StickFailureV2::InvalidBodyRates, in.ResetGeneration, in.SimulationTimeS); }
    if (config.BetaDampingGain != 0.0 && !in.bAlphaBetaValid) { ClearState(); return Neutral(EF16StickFailureV2::InvalidAlphaBeta, in.ResetGeneration, in.SimulationTimeS); }

    const bool finite = Fin(in.RollReferenceRad) && Fin(in.PitchReferenceRad) && Fin(in.ThrottleReferenceNorm) &&
                        Fin(in.CurrentRollRad) && Fin(in.CurrentPitchRad) &&
                        (!needBodyRates || (Fin(in.BodyRollRateRadps) && Fin(in.BodyPitchRateRadps) && Fin(in.BodyYawRateRadps))) &&
                        (config.BetaDampingGain == 0.0 || Fin(in.BetaRad));
    if (!finite) { ClearState(); return Neutral(EF16StickFailureV2::NonFiniteInput, in.ResetGeneration, in.SimulationTimeS); }

    const double dt = in.DtS;

    // ---------------- roll channel ----------------
    // attitude error -> desired roll rate -> normalized rate command (FCS closes the rate loop).
    const double rollError = WrapPi(in.RollReferenceRad - in.CurrentRollRad);
    double desiredRollRate = Clamp(config.RollErrorToRateGain * rollError,
                                   -config.MaxRollRateRadps, config.MaxRollRateRadps);
    if (config.RollRateDampingGain != 0.0) desiredRollRate -= config.RollRateDampingGain * in.BodyRollRateRadps;
    const double aileronTarget = Clamp(desiredRollRate / config.RollRateCommandNormRadps, -1.0, 1.0);

    // ---------------- pitch channel ----------------
    // attitude error -> normalized pitch command; EXPLICIT sign inversion: nose-up => negative cmd.
    const double pitchError = WrapPi(in.PitchReferenceRad - in.CurrentPitchRad);
    double elevatorUnsat = -(config.PitchErrorToCmdGain * pitchError + PitchIntegrator);
    if (config.PitchRateDampingGain != 0.0) elevatorUnsat += config.PitchRateDampingGain * in.BodyPitchRateRadps;
    // anti-windup: conditional integration (skip when it would push further into saturation).
    const bool satHi = elevatorUnsat > config.ElevatorMax, satLo = elevatorUnsat < config.ElevatorMin;
    const bool integrationPushesHi = (-config.PitchIntegratorGain * pitchError) > 0.0; // d(elev)/dt from I-term
    if (!(satHi && integrationPushesHi) && !(satLo && !integrationPushesHi)) {
        PitchIntegrator = Clamp(PitchIntegrator + config.PitchIntegratorGain * pitchError * dt,
                                -config.PitchIntegratorLimit, config.PitchIntegratorLimit);
        elevatorUnsat = -(config.PitchErrorToCmdGain * pitchError + PitchIntegrator);
        if (config.PitchRateDampingGain != 0.0) elevatorUnsat += config.PitchRateDampingGain * in.BodyPitchRateRadps;
    }
    const double elevatorTarget = Clamp(elevatorUnsat, config.ElevatorMin, config.ElevatorMax);

    // ---------------- rudder channel ----------------
    // Neutral by default: the f16.xml yaw damper + turn coordination is active (vc >= 10 kts) and
    // there is no basis for heading-error rudder. Optional damping gains default to 0.
    double rudderTarget = 0.0;
    if (config.YawRateDampingGain != 0.0) rudderTarget -= config.YawRateDampingGain * in.BodyYawRateRadps;
    if (config.BetaDampingGain != 0.0) rudderTarget -= config.BetaDampingGain * in.BetaRad;
    rudderTarget = Clamp(rudderTarget, -1.0, 1.0);

    // ---------------- throttle channel ----------------
    const double throttleTarget = Clamp(in.ThrottleReferenceNorm, config.ThrottleMin, config.ThrottleMax);

    // ---------------- slew limiting (anchors are 0 after any clear, so no stale jump) ----------------
    auto slew = [dt](double target, double prev, double ratePerS) {
        return Clamp(target, prev - ratePerS * dt, prev + ratePerS * dt);
    };
    FF16StickCommandV2 out{};
    out.AileronCmdNorm  = Clamp(slew(aileronTarget,  bHavePrevCommands ? PrevAileron  : 0.0, config.AileronSlewPerS), -1.0, 1.0);
    out.ElevatorCmdNorm = Clamp(slew(elevatorTarget, bHavePrevCommands ? PrevElevator : 0.0, config.ElevatorSlewPerS),
                                config.ElevatorMin, config.ElevatorMax);
    out.RudderCmdNorm   = Clamp(slew(rudderTarget,   bHavePrevCommands ? PrevRudder   : 0.0, config.RudderSlewPerS), -1.0, 1.0);
    out.ThrottleCmdNorm = Clamp(slew(throttleTarget, bHavePrevCommands ? PrevThrottle : config.ThrottleMin, config.ThrottleSlewPerS),
                                config.ThrottleMin, config.ThrottleMax);
    out.TimestampS = in.SimulationTimeS;
    out.ResetGeneration = in.ResetGeneration;

    if (!Fin(out.AileronCmdNorm) || !Fin(out.ElevatorCmdNorm) || !Fin(out.RudderCmdNorm) || !Fin(out.ThrottleCmdNorm)) {
        ClearState();
        return Neutral(EF16StickFailureV2::NonFiniteOutput, in.ResetGeneration, in.SimulationTimeS);
    }

    PrevAileron = out.AileronCmdNorm; PrevElevator = out.ElevatorCmdNorm;
    PrevRudder = out.RudderCmdNorm;   PrevThrottle = out.ThrottleCmdNorm;
    bHavePrevCommands = true;
    out.bValid = true;
    out.FailureReason = EF16StickFailureV2::None;
    return out;
}

} // namespace FormationControlV2
