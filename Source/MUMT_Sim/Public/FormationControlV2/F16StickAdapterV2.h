#pragma once

// F16StickAdapterV2 — pure C++ (no Unreal, no JSBSim includes) outer-loop stick adapter that
// turns guidance attitude/throttle references into normalized F-16 FCS commands:
//   fcs/aileron-cmd-norm, fcs/elevator-cmd-norm, fcs/rudder-cmd-norm, fcs/throttle-cmd-norm.
//
// Division of labour, from the read-only audit of the runtime-loaded f16.xml "F-16 FC":
//   f16.xml already performs (must NOT be duplicated here):
//     * Roll:  aileron-cmd-norm is consumed as a NORMALIZED ROLL-RATE COMMAND. The FCS computes
//       roll-rate-norm = p-aero * 0.31821, closes a rate PID (kp 3, active vc >= 20 kts), applies
//       [-1,1] clip, a 0.3 s kinematic actuator, Mach-scheduled authority, flaperon mixing and
//       +/-0.375 rad surface limits.
//     * Pitch: elevator-cmd-norm is consumed as a G/pitch-rate blend command (C*-like), with the
//       G-limiter clip [-1, +0.44] (NEGATIVE = nose-up), alpha protection fading authority to zero
//       at +/-30 deg alpha, a g-load PID (active vc >= 5 kts), a 0.3 s actuator, +/-0.436 rad
//       stabilator limits and aileron/elevator mixing.
//     * Yaw:   an internal yaw damper + turn coordination (ground-speed-scheduled yaw-rate plus
//       lateral-G PID, active vc >= 10 kts) consumes rudder-cmd-norm as pilot input, 0.4 s
//       actuator, +/-0.524 rad.
//     * Aerodynamics consumes only the *-pos-rad outputs; nothing bypasses the FCS.
//   This adapter therefore only performs:
//     * Roll:  attitude error -> desired roll rate -> normalized rate command.
//     * Pitch: attitude error -> normalized pitch command with the EXPLICIT sign inversion
//       (positive pitch error / nose-up demand => NEGATIVE elevator-cmd-norm).
//     * Rudder: neutral (0) by default — the internal yaw damper is sufficient; no heading-error
//       rudder. An optional small body-yaw-rate / beta damping term exists but defaults to 0.
//     * Throttle: guidance throttle reference -> clamp -> slew limit.
//   Optional measured-rate damping gains exist for the roll/pitch channels but DEFAULT TO 0
//   because the FCS already closes the rate loops; enabling them reshapes the rate command and
//   must be a deliberate, identified decision.
//
// All gains/limits are provisional (not flight-identified). Shadow/offline use only —
// this adapter must NOT drive JSBSim commands until separately approved (no Active use).
#include "FormationControlV2/PlannerV2Types.h"

#include <cstdint>

namespace FormationControlV2 {

enum class EF16StickFailureV2 : std::uint8_t {
    None, ResetFrame, Paused, InvalidGuidance, InvalidAttitude, InvalidBodyRates,
    InvalidAlphaBeta, InvalidTime, AbnormalDt, NonFiniteInput, NonFiniteOutput,
    // Appended LAST so every existing index stays stable for observers.
    // A primed latch is armed, but this frame does not belong to the prime it was armed for.
    PrimeIdentityMismatch
};

struct FF16StickConfigV2 {
    // --- roll channel (provisional; FCS rate-normalization factor from f16.xml: 0.31821) ---
    double RollErrorToRateGain{1.5};        // [1/s] attitude error -> desired roll rate
    double MaxRollRateRadps{3.14};          // desired-rate clamp; FCS full-scale is 1/0.31821
    double RollRateCommandNormRadps{3.142866}; // 1 / 0.31821: rate that maps to cmd 1.0
    double RollRateDampingGain{0.0};        // DEFAULT 0: f16.xml already closes the rate loop
    double AileronSlewPerS{4.0};            // [cmd-norm/s]
    // --- pitch channel (provisional; elevator-cmd NEGATIVE = nose-up) ---
    double PitchErrorToCmdGain{2.0};        // [cmd-norm/rad] attitude error -> cmd magnitude
    double PitchIntegratorGain{0.2};        // [cmd-norm/(rad*s)] trim integrator
    double PitchIntegratorLimit{0.30};      // integrator clamp [cmd-norm]
    double PitchRateDampingGain{0.0};       // DEFAULT 0: f16.xml g/rate PID active
    double ElevatorSlewPerS{4.0};           // [cmd-norm/s]
    double ElevatorMin{-1.0}, ElevatorMax{1.0}; // FCS additionally clips to [-1, +0.44]
    // --- rudder channel ---
    double YawRateDampingGain{0.0};         // DEFAULT 0: internal yaw damper sufficient
    double BetaDampingGain{0.0};            // DEFAULT 0: no basis for external beta control
    double RudderSlewPerS{4.0};             // [cmd-norm/s]
    // --- throttle channel ---
    double ThrottleMin{0.0}, ThrottleMax{1.0};
    double ThrottleSlewPerS{1.0};           // [cmd-norm/s]
    // --- timing ---
    double DtMinS{1.0e-4}, DtMaxS{1.0};
};

struct FF16StickInputV2 {
    // guidance references (from FormationGuidanceCoordinatorV2)
    double RollReferenceRad{}, PitchReferenceRad{}, ThrottleReferenceNorm{};
    bool bGuidanceValid{};
    // current attitude (same atomic JSBSim snapshot)
    double CurrentRollRad{}, CurrentPitchRad{};
    bool bAttitudeValid{};
    // body rates from FGPropagate::GetPQR (NOT Euler-angle derivatives)
    double BodyRollRateRadps{}, BodyPitchRateRadps{}, BodyYawRateRadps{};
    bool bBodyRatesValid{};
    // aero angles from FGAuxiliary::Getalpha/Getbeta [rad]
    double AlphaRad{}, BetaRad{};
    bool bAlphaBetaValid{};
    // airspeed (carried for future gain scheduling; not gated by the default laws)
    double EasMps{}, TasMps{};
    bool bAirspeedValid{};
    // time / lifecycle
    double SimulationTimeS{}, DtS{};
    bool bPaused{};
    std::uint32_t ResetGeneration{};

    // Prime identity (Phase B). Only meaningful while a baseline latch is armed: the latch is spent
    // ONLY on the frame it was armed for. A latch is a promise about ONE specific consume of ONE
    // specific prime; spending it on any other frame would emit a command the aircraft was never
    // flying, which is a step dressed up as a handoff.
    std::uint64_t PrimeGeneration{};
    std::uint64_t PrimeConsumeSequence{};
};

struct FF16StickCommandV2 {
    double AileronCmdNorm{};   // [-1,1], + = roll right (FCS: + desired p)
    double ElevatorCmdNorm{};  // [-1,1], NEGATIVE = nose-up (FCS G-limiter clips to [-1,+0.44])
    double RudderCmdNorm{};    // [-1,1], neutral 0 by default policy
    double ThrottleCmdNorm{};  // [0,1]
    double TimestampS{};
    std::uint32_t ResetGeneration{};
    bool bValid{};
    EF16StickFailureV2 FailureReason{EF16StickFailureV2::None};
};

class F16StickAdapterV2 {
public:
    FF16StickCommandV2 Update(const FF16StickInputV2 &, const FF16StickConfigV2 & = {});
    void Reset(std::uint32_t resetGeneration = 0);

    // ---- PRIME (bumpless handoff, Phase B) ---------------------------------------------------
    // Reset() zeroes the slew memory and the pitch integrator, so the first command after a reset starts
    // from 0 and the slew limiter drags it toward the target -- a visible step on handover.
    //
    // Prime seeds the state FROM THE COMMAND THE FDM IS ALREADY CONSUMING, and it does so in two parts
    // that are NOT interchangeable:
    //
    //   1. STATE SEED (continuity from the second frame onwards)
    //      * PrevAileron/Elevator/Rudder/Throttle <- the resolved command: the slew limiter starts where
    //        the aircraft already is and cannot jump.
    //      * PitchIntegrator <- solved so the pitch loop reproduces that elevator AT ZERO PITCH ERROR:
    //            elevator = -(Kp*err + I) + D*q,  err = 0  =>  I = D*q - elevator
    //
    //   2. ONE-SHOT BASELINE LATCH (exactness on the FIRST frame)
    //      The seed above is only exact if the guidance happens to command zero pitch error on the very
    //      first frame. It does not, in general: the guidance is a live controller with its own idea of
    //      the reference. Seeding alone therefore makes the first command CLOSE, not EQUAL -- the slew
    //      limiter would smear a residual step out over several frames rather than prevent it.
    //
    //      So the first valid Update() after a prime EMITS THE BASELINE EXACTLY and consumes the latch.
    //      From the second Update() on, the ordinary target / integrator / slew / clamp path runs
    //      untouched. This is not a filter and not a hold: it is one frame, once, by construction.
    //
    // The latch carries the generation and consume sequence it was primed for, so a stale prime cannot
    // be spent later: a new prime replaces it, and Reset() destroys it.
    //
    // This is state initialisation, NOT tuning: no gain, limit, slew rate or trim is touched.
    // Returns FALSE and changes NOTHING if the prime is not usable. A partially-applied prime would be
    // worse than none: the slew anchors would move to a command that was never validated, and the latch
    // would promise a baseline nobody checked.
    //   * generation 0 / consume sequence 0  -- 0 means "no ticket"; a latch keyed to it can never match
    //   * non-finite or out-of-range baseline -- it would be handed straight to the FCS
    //   * non-finite body pitch rate          -- it feeds the integrator seed
    //   * an unusable CONFIG                  -- non-finite damping gain or integrator limit, a negative
    //     integrator limit, or an inverted Elevator/Throttle range. The seed is computed FROM the config,
    //     and the baseline is range-checked AGAINST it, so an unchecked config makes both meaningless.
    bool PrimeFromResolvedCommand(double resolvedAileron, double resolvedElevator, double resolvedRudder,
                                  double resolvedThrottle, double bodyPitchRateRadps,
                                  const FF16StickConfigV2 &config,
                                  std::uint64_t primeGeneration, std::uint64_t consumeSequence,
                                  std::uint32_t resetGeneration = 0);

    // Test/diagnostic accessors: what the prime actually left in the state.
    double PrimedPitchIntegrator() const { return PitchIntegrator; }
    bool HasPrimedCommands() const { return bHavePrevCommands; }
    // State snapshot, so a test can prove a latched frame advanced NOTHING.
    double PrevCommandAileron()  const { return PrevAileron; }
    double PrevCommandElevator() const { return PrevElevator; }
    double PrevCommandRudder()   const { return PrevRudder; }
    double PrevCommandThrottle() const { return PrevThrottle; }
    // Counts ORDINARY updates (target -> integrator -> slew -> clamp). A latched frame does not count.
    // Proving "the normal path ran" by watching the integrator move is unreliable: conditional
    // anti-windup legitimately skips integration under saturation, so a still integrator can mean either
    // "the latch suppressed it" or "the controller correctly refused to wind up". This counter cannot be
    // confused between the two.
    std::uint64_t NormalPathUpdates() const { return NormalPathUpdates_; }
    bool HasPrimedBaselineLatch() const { return bEmitPrimedBaselineOnce; }
    std::uint64_t PrimedGeneration() const { return PrimedGeneration_; }
    std::uint64_t PrimedConsumeSequence() const { return PrimedConsumeSequence_; }

private:
    // Fresh outputs are computed every frame; this state is only the integrator and the
    // slew-limiter anchors. It is cleared on reset/pause-resume/invalid so no previous
    // command can leak through after a failure.
    double PitchIntegrator{};
    double PrevAileron{}, PrevElevator{}, PrevRudder{}, PrevThrottle{};
    bool bHavePrevCommands{};
    std::uint32_t LastResetGeneration{};
    bool bInitialized{}, bWasPaused{};

    // One-shot baseline latch (see PrimeFromResolvedCommand). Spent by the first valid Update() after a
    // prime, destroyed by ClearState() (reset / pause-resume / abnormal dt / invalid input), and
    // overwritten by a newer prime.
    bool bEmitPrimedBaselineOnce{};
    double PrimedBaselineAileron{}, PrimedBaselineElevator{};
    double PrimedBaselineRudder{}, PrimedBaselineThrottle{};
    std::uint64_t PrimedGeneration_{}, PrimedConsumeSequence_{};
    std::uint64_t NormalPathUpdates_{};   // test/diagnostic: ordinary (non-latched) valid updates

    void ClearState();
};

} // namespace FormationControlV2
