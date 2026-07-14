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

    // A reset destroys the latch. A baseline captured before a reset describes a command the aircraft
    // has since stopped flying; spending it afterwards would be a step, not a handoff.
    bEmitPrimedBaselineOnce = false;
    PrimedBaselineAileron = PrimedBaselineElevator = PrimedBaselineRudder = PrimedBaselineThrottle = 0.0;
    PrimedGeneration_ = 0;
    PrimedConsumeSequence_ = 0;
    // NormalPathUpdates_ is a diagnostic tally, not control state: it deliberately survives ClearState().
}

void F16StickAdapterV2::Reset(std::uint32_t resetGeneration)
{
    ClearState();
    LastResetGeneration = resetGeneration;
    bInitialized = false;
    bWasPaused = false;
}

bool F16StickAdapterV2::PrimeFromResolvedCommand(double resolvedAileron, double resolvedElevator,
                                                 double resolvedRudder, double resolvedThrottle,
                                                 double bodyPitchRateRadps, const FF16StickConfigV2 &config,
                                                 std::uint64_t primeGeneration,
                                                 std::uint64_t consumeSequence,
                                                 std::uint32_t resetGeneration)
{
    // VALIDATE FIRST, MUTATE SECOND. Every check happens before a single field is written, so a rejected
    // prime leaves the controller EXACTLY as it was -- no half-moved slew anchor, no half-seeded
    // integrator, no latch armed against a baseline nobody validated.
    if (primeGeneration == 0 || consumeSequence == 0) return false;   // 0 means "no ticket"

    // THE CONFIG IS AN INPUT TO THE SEED, so it is validated like one. The integrator seed is
    // Clamp(D*q - elevator, +/-Limit) and the range gates below are read straight off the config: a NaN
    // gain would poison the integrator, a negative limit would make Clamp(lo > hi) meaningless, and an
    // inverted Elevator/Throttle range would make every baseline "out of range" -- or, worse, let one
    // through. Validating the baseline against a config nobody checked is validating nothing.
    if (!Fin(config.PitchRateDampingGain) || !Fin(config.PitchIntegratorLimit)) return false;
    if (config.PitchIntegratorLimit < 0.0) return false;
    if (!Fin(config.ElevatorMin) || !Fin(config.ElevatorMax) || config.ElevatorMin > config.ElevatorMax)
        return false;
    if (!Fin(config.ThrottleMin) || !Fin(config.ThrottleMax) || config.ThrottleMin > config.ThrottleMax)
        return false;

    if (!Fin(resolvedAileron) || !Fin(resolvedElevator) || !Fin(resolvedRudder) || !Fin(resolvedThrottle))
        return false;
    if (!Fin(bodyPitchRateRadps)) return false;                       // it feeds the integrator seed
    if (resolvedAileron < -1.0 || resolvedAileron > 1.0) return false;
    if (resolvedRudder  < -1.0 || resolvedRudder  > 1.0) return false;
    if (resolvedElevator < config.ElevatorMin || resolvedElevator > config.ElevatorMax) return false;
    if (resolvedThrottle < config.ThrottleMin || resolvedThrottle > config.ThrottleMax) return false;

    // The slew memory becomes the command the aircraft is ALREADY flying, so the first frame after the
    // handoff cannot step: slew() starts from here rather than from 0.
    PrevAileron  = resolvedAileron;
    PrevElevator = resolvedElevator;
    PrevRudder   = resolvedRudder;
    PrevThrottle = resolvedThrottle;
    bHavePrevCommands = true;

    // Seed the integrator so the pitch loop REPRODUCES that elevator rather than merely slewing toward
    // its own idea of it. Update() computes
    //     elevator = -(PitchErrorToCmdGain * pitchError + PitchIntegrator) + PitchRateDampingGain * q
    // so at the handoff instant, where the reference is the current attitude (pitchError == 0):
    //     PitchIntegrator = PitchRateDampingGain * q - elevator
    const double damping = (config.PitchRateDampingGain != 0.0)
        ? config.PitchRateDampingGain * bodyPitchRateRadps
        : 0.0;
    PitchIntegrator = Clamp(damping - resolvedElevator,
                            -config.PitchIntegratorLimit, config.PitchIntegratorLimit);

    // ARM THE ONE-SHOT LATCH. The seed above only reproduces the baseline if the guidance happens to
    // command zero pitch error on the first frame, which in general it does not: it is a live controller
    // with its own reference. Without the latch the first command is CLOSE, not EQUAL, and the slew
    // limiter would smear that residual step over the following frames instead of preventing it.
    //
    // A new prime overwrites any latch left unspent by an earlier one -- the older baseline is stale by
    // definition.
    bEmitPrimedBaselineOnce = true;
    PrimedBaselineAileron  = resolvedAileron;
    PrimedBaselineElevator = resolvedElevator;
    PrimedBaselineRudder   = resolvedRudder;
    PrimedBaselineThrottle = resolvedThrottle;
    PrimedGeneration_ = primeGeneration;
    PrimedConsumeSequence_ = consumeSequence;

    // The next Update() is a continuation, not a first frame.
    LastResetGeneration = resetGeneration;
    bInitialized = true;
    bWasPaused = false;
    return true;
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

    // ---------------- one-shot primed baseline (BEFORE any control computation) ----------------
    // THE LATCHED FRAME ADVANCES NOTHING. It emits the baseline and returns.
    //
    // This block sits ahead of the roll/pitch channels on purpose. Running the ordinary path first and
    // then overwriting the outputs would still have integrated PitchIntegrator, computed a target and
    // stepped the slew limiter for a frame whose command was never used -- the controller would carry
    // one frame of state it never actually flew, and the "handoff" would start from a lie.
    //
    // Identity is checked, not assumed. A latch is a promise about ONE consume of ONE prime; spending it
    // on any other frame would emit a command the aircraft was not flying. On a mismatch NOTHING is
    // touched -- no baseline, no latch consumption, no Prev*, no integrator -- and the frame is refused
    // explicitly rather than silently producing something plausible.
    if (bEmitPrimedBaselineOnce) {
        if (in.PrimeGeneration != PrimedGeneration_ || in.PrimeConsumeSequence != PrimedConsumeSequence_) {
            return Neutral(EF16StickFailureV2::PrimeIdentityMismatch, in.ResetGeneration, in.SimulationTimeS);
        }

        FF16StickCommandV2 primed{};
        primed.AileronCmdNorm  = PrimedBaselineAileron;
        primed.ElevatorCmdNorm = PrimedBaselineElevator;
        primed.RudderCmdNorm   = PrimedBaselineRudder;
        primed.ThrottleCmdNorm = PrimedBaselineThrottle;
        primed.TimestampS = in.SimulationTimeS;
        primed.ResetGeneration = in.ResetGeneration;

        if (!Fin(primed.AileronCmdNorm) || !Fin(primed.ElevatorCmdNorm) ||
            !Fin(primed.RudderCmdNorm)  || !Fin(primed.ThrottleCmdNorm)) {
            ClearState();
            return Neutral(EF16StickFailureV2::NonFiniteOutput, in.ResetGeneration, in.SimulationTimeS);
        }

        // The slew anchors become the baseline. PitchIntegrator keeps the value the prime seeded -- it is
        // NOT integrated on this frame.
        PrevAileron  = primed.AileronCmdNorm;
        PrevElevator = primed.ElevatorCmdNorm;
        PrevRudder   = primed.RudderCmdNorm;
        PrevThrottle = primed.ThrottleCmdNorm;
        bHavePrevCommands = true;

        bEmitPrimedBaselineOnce = false;   // spent exactly once
        primed.bValid = true;
        primed.FailureReason = EF16StickFailureV2::None;
        return primed;
    }

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
    ++NormalPathUpdates_;   // an ORDINARY frame ran: target, integrator, slew, clamp
    out.bValid = true;
    out.FailureReason = EF16StickFailureV2::None;
    return out;
}

} // namespace FormationControlV2
