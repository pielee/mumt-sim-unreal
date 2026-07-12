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
    InvalidAlphaBeta, InvalidTime, AbnormalDt, NonFiniteInput, NonFiniteOutput
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

private:
    // Fresh outputs are computed every frame; this state is only the integrator and the
    // slew-limiter anchors. It is cleared on reset/pause-resume/invalid so no previous
    // command can leak through after a failure.
    double PitchIntegrator{};
    double PrevAileron{}, PrevElevator{}, PrevRudder{}, PrevThrottle{};
    bool bHavePrevCommands{};
    std::uint32_t LastResetGeneration{};
    bool bInitialized{}, bWasPaused{};

    void ClearState();
};

} // namespace FormationControlV2
