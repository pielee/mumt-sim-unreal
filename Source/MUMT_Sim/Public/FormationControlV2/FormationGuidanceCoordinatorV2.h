#pragma once

#include "FormationControl/Px4NpfgAdapter.h"
#include "FormationControl/Px4TecsAdapter.h"
#include "FormationControlV2/FormationNavigationTypes.h"
#include "FormationControlV2/PlannerV2Adapters.h"

#include <cstdint>
#include <memory>

namespace FormationControlV2 {

enum class EGuidanceFailureV2 : std::uint8_t {
    None, Disabled, ShadowDisabled, Paused, ResetFrame, InvalidFollower,
    InvalidPlannerDto, InvalidWind, InvalidTime, OriginMismatch,
    ResetMismatch, ZeroTangent, NonFiniteInput, NonFiniteOutput,
    // Appended LAST so every existing index stays stable for observers.
    InvalidConfig
};

// ---------------------------------------------------------------------------------------------
// TECS caller contract.
//
// The pinned PX4 v1.17.0 caller (d6f12ad1c4f70ad3230afd7d86e971421e02fef4,
// src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp:91-112,151) configures
// the TECS controller through a fixed set of setters before using it. Anything it does not set keeps
// the TECS *class* default, and those class defaults are placeholders, not a usable configuration:
// they are the values PX4 itself never runs with. Relying on them silently is the same class of
// defect as the vert_accel_limit = 0 frozen-pitch bug, so every parameter the upstream caller sets
// is represented here explicitly and validated.
//
// The upstream caller sources them from three different places, and that distinction is preserved:
//
//   (a) generic controller tuning  -> a PX4 parameter with an official default and range. Those
//       defaults are used verbatim below and are cited per field.
//   (b) aircraft performance       -> PX4 reads these from src/lib/fw_performance_model
//       (getMaximumClimbRate(rho), getMinimumSinkRate(rho), getCalibratedTrimAirspeed()). That
//       library is ABSENT from the pinned partial checkout, so no PX4 law can be reproduced and no
//       F-16 value is asserted here. These fields default to the value the TECS class already used,
//       which preserves current behaviour exactly while making the dependency explicit and
//       configurable. Identifying real F-16 values is a separate, deliberate task.
//   (c) per-frame runtime state    -> load factor (from bank angle), airspeed validity, underspeed
//       disable, altitude-step handling, and the air-density refresh of (b). These are NOT
//       configuration and are deliberately not plumbed through this struct.
// ---------------------------------------------------------------------------------------------

// (a) generic controller tuning -- pinned PX4 parameter ranges.
//
// These bounds apply ONLY to generic controller tuning, where the PX4 parameter range is the range
// the control law was designed for. They are deliberately NOT applied to the aircraft performance
// rates below: PX4's parameter metadata (e.g. FW_T_SINK_MAX @min 1 @max 15 m/s) describes the
// small-fixed-wing envelope its parameter UI targets, not a physical law, and clamping an airframe's
// measured climb/sink performance to it would silently reject a valid aircraft instead of
// configuring it.
inline constexpr double kTecsVerticalAccelLimitMinMps2 = 1.0;   // FW_T_VERT_ACC @min
inline constexpr double kTecsVerticalAccelLimitMaxMps2 = 10.0;  // FW_T_VERT_ACC @max
inline constexpr double kTecsAltitudeErrorTimeConstantMinS = 2.0;   // FW_T_ALT_TC @min
inline constexpr double kTecsAirspeedErrorTimeConstantMinS = 2.0;   // FW_T_TAS_TC @min
inline constexpr double kTecsPitchIntegratorGainMin = 0.0, kTecsPitchIntegratorGainMax = 2.0;   // FW_T_I_GAIN_PIT
inline constexpr double kTecsPitchDampingMin = 0.0, kTecsPitchDampingMax = 2.0;                 // FW_T_PTCH_DAMP
inline constexpr double kTecsThrottleIntegratorGainMin = 0.0, kTecsThrottleIntegratorGainMax = 1.0; // FW_T_THR_INTEG
inline constexpr double kTecsThrottleDampingMin = 0.0, kTecsThrottleDampingMax = 1.0;           // FW_T_THR_DAMPING
inline constexpr double kTecsThrottleSlewRateMinPerS = 0.0, kTecsThrottleSlewRateMaxPerS = 1.0; // FW_THR_SLEW_MAX
inline constexpr double kTecsSteRateTimeConstantMinS = 0.0, kTecsSteRateTimeConstantMaxS = 2.0; // FW_T_STE_R_TC
inline constexpr double kTecsSebRateFeedForwardMin = 0.5, kTecsSebRateFeedForwardMax = 3.0;     // FW_T_SEB_R_FF
inline constexpr double kTecsAltitudeRateFeedForwardMin = 0.0, kTecsAltitudeRateFeedForwardMax = 1.0; // FW_T_HRATE_FF
inline constexpr double kTecsSpeedWeightMin = 0.0, kTecsSpeedWeightMax = 2.0;                   // FW_T_SPDWEIGHT
inline constexpr double kTecsRollToThrottleMin = 0.0, kTecsRollToThrottleMax = 20.0;            // FW_T_RLL2THR
inline constexpr double kTecsAirspeedStdDevMin = 0.01, kTecsAirspeedStdDevMax = 10.0;           // FW_T_SPD_*_STD

struct FGuidanceConfigV2 {
    double RollLimitRad{0.7853981633974483};
    double PitchMinRad{-0.5}, PitchMaxRad{0.5};
    double ThrottleMin{0.0}, ThrottleMax{1.0}, ThrottleTrim{0.5};
    double EasMinMps{10.0}, EasMaxMps{80.0};
    double MinimumGroundSpeedMps{0.0};
    // Reference-trajectory rates, NOT energy limits: PX4 FW_T_CLMB_R_SP / FW_T_SINK_R_SP feed the
    // altitude reference generator. They are a different quantity from TecsMaxClimbRateMps /
    // TecsMaxSinkRateMps below, which bound the specific-total-energy rate, and must never be
    // aliased onto each other.
    double TargetClimbRateMps{10.0}, TargetSinkRateMps{10.0};
    double FastDescendAltitudeErrorM{100.0};   // FW_T_F_ALT_ERR [m]

    // --- (b) aircraft performance: PX4 sources these from the absent fw_performance_model ---
    // These are properties of the airframe, not controller tuning. They are validated as what they
    // are -- finite, strictly positive, and ordered (min sink <= max sink) -- and carry no numeric
    // upper bound: any such bound would be an invented limit on the aircraft, and PX4's own
    // parameter metadata range cannot serve as one (see the note above the tuning bounds).
    //
    // Climb rate produced by max allowed throttle [m/s]. Sets TECSControl STE_rate_max = value * g,
    // which scales the whole throttle/airspeed authority. > 0.
    double TecsMaxClimbRateMps{5.0};             // preserves the TECS class default
    // Minimum sink rate at min throttle and trim speed [m/s]. Sets STE_rate_min = -value * g. > 0.
    double TecsMinSinkRateMps{2.0};              // preserves the TECS class default
    // Maximum sink rate at min throttle and max speed [m/s]. PX4 reads it from FW_T_SINK_MAX, the
    // one performance value upstream takes from a parameter rather than the model; that parameter's
    // metadata range is NOT used as validation here. > 0 and >= TecsMinSinkRateMps.
    double TecsMaxSinkRateMps{5.0};
    // Equivalent cruise airspeed [m/s]. PX4: performance_model.getCalibratedTrimAirspeed().
    // Must satisfy EasMinMps <= this <= EasMaxMps.
    double TecsEquivalentAirspeedTrimMps{15.0};  // preserves the TECS class default

    // --- (a) generic controller tuning: pinned PX4 parameter defaults, used verbatim ---
    double TecsVerticalAccelLimitMps2{7.0};          // FW_T_VERT_ACC   [m/s^2] @min 1 @max 10
    double TecsAltitudeErrorTimeConstantS{5.0};      // FW_T_ALT_TC     [s]     @min 2
    double TecsAirspeedErrorTimeConstantS{5.0};      // FW_T_TAS_TC     [s]     @min 2
    double TecsPitchIntegratorGain{0.1};             // FW_T_I_GAIN_PIT [-]     @min 0 @max 2
    double TecsPitchDamping{0.1};                    // FW_T_PTCH_DAMP  [s]     @min 0 @max 2
    double TecsThrottleIntegratorGain{0.02};         // FW_T_THR_INTEG  [-]     @min 0 @max 1
    double TecsThrottleDamping{0.05};                // FW_T_THR_DAMPING[s]     @min 0 @max 1
    double TecsThrottleSlewRatePerS{0.0};            // FW_THR_SLEW_MAX [1/s]   @min 0 @max 1 (0 = off)
    double TecsSteRateTimeConstantS{0.4};            // FW_T_STE_R_TC   [s]     @min 0 @max 2
    double TecsSebRateFeedForwardGain{1.0};          // FW_T_SEB_R_FF   [-]     @min 0.5 @max 3
    double TecsAltitudeRateFeedForward{0.3};         // FW_T_HRATE_FF   [-]     @min 0 @max 1
    double TecsSpeedWeight{1.0};                     // FW_T_SPDWEIGHT  [-]     @min 0 @max 2
    // Gain from normal load factor increase to total energy rate demand. PX4 FW_T_RLL2THR
    // (@min 0 @max 20, default 15). It multiplies (load_factor - 1), and load factor is per-frame
    // runtime state that this coordinator does not yet feed, so today the term contributes zero.
    // The parameter is still configured so the caller contract matches upstream and the missing
    // runtime path is visible rather than hidden.
    double TecsRollToThrottleCompensation{15.0};     // FW_T_RLL2THR    [m^2/s^3]
    double TecsAirspeedMeasurementStdDevMps{0.07};       // FW_T_SPD_STD     [m/s]   @min 0.01 @max 10
    double TecsAirspeedRateMeasurementStdDevMps2{0.2};   // FW_T_SPD_DEV_STD [m/s^2] @min 0.01 @max 10
    double TecsAirspeedFilterProcessStdDevMps2{0.2};     // FW_T_SPD_PRC_STD [m/s^2] @min 0.01 @max 10

    bool bDetectUnderspeed{true};
};

// Pure: the config contract the coordinator enforces before it will build a controller.
// Exposed so tests grade against the same predicate instead of re-deriving the bounds.
bool IsGuidanceConfigValid(const FGuidanceConfigV2 &config);

struct FGuidanceCoordinatorInputV2 {
    FCanonicalNavigationStateV2 Follower{};
    FFormationSlotStateV2 Slot{};
    FPlannerV2OutputAdapterResult PlannerDto{};
    double CurrentPitchRad{};
    bool bCurrentPitchValid{};
    // ACTUAL aircraft roll [rad], measured from the vehicle attitude / plant state -- NOT the NPFG
    // roll reference and NOT a commanded bank. The pinned PX4 caller
    // (fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp, updateAttitude()) derives
    // the TECS turn load factor from exactly this signal:
    //     load_factor = 1 / max(cos(actual roll), FLT_EPSILON)
    // and applies it before every TECS update. Without it TECS keeps its class default load_factor
    // of 1.0, so the roll-to-throttle compensation term, load_factor_correction * (load_factor - 1),
    // is identically zero and a banked turn gets no energy compensation at all.
    double CurrentRollRad{};
    bool bCurrentRollValid{};
    double SimulationTimeS{}, DtS{};
    std::uint32_t ResetGeneration{}, OriginGeneration{};
    bool bControllerEnabled{true}, bShadowEnabled{true}, bPaused{};
};

struct FGuidanceCoordinatorOutputV2 {
    double RollReferenceRad{}, PitchReferenceRad{}, ThrottleReferenceNorm{};
    double LateralAccelerationFeedforwardMps2{}, LateralAccelerationFeedbackMps2{};
    double LateralAccelerationTotalMps2{}, CourseSetpointRad{}, WindFeasibility{};
    double UnderspeedRatio{}, FastDescendRatio{};
    double TotalEnergyRateSetpoint{}, TotalEnergyRateEstimate{};
    double EnergyBalanceRateSetpoint{}, EnergyBalanceRateEstimate{};
    double PitchIntegrator{}, ThrottleIntegrator{};
    double TimestampS{};
    std::uint32_t ResetGeneration{};
    bool bNpfgValid{}, bTecsValid{}, bCommandReady{};
    EGuidanceFailureV2 FailureReason{EGuidanceFailureV2::None};
};

class FormationGuidanceCoordinatorV2 {
public:
    FormationGuidanceCoordinatorV2();
    ~FormationGuidanceCoordinatorV2();
    FormationGuidanceCoordinatorV2(const FormationGuidanceCoordinatorV2 &) = delete;
    FormationGuidanceCoordinatorV2 &operator=(const FormationGuidanceCoordinatorV2 &) = delete;

    FGuidanceCoordinatorOutputV2 Update(const FGuidanceCoordinatorInputV2 &, const FGuidanceConfigV2 & = {});
    void Reset(std::uint32_t resetGeneration = 0);

private:
    void RecreateControllers(const FGuidanceConfigV2 &);
    std::unique_ptr<MumtPx4::FPx4NpfgAdapter> Npfg;
    std::unique_ptr<MumtPx4::FPx4TecsAdapter> Tecs;
    std::uint32_t LastResetGeneration{};
    double LastSimulationTimeS{};
    bool bInitialized{}, bWasPaused{};
};

} // namespace FormationControlV2
