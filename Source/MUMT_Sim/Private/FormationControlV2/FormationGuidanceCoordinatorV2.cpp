#include "FormationControlV2/FormationGuidanceCoordinatorV2.h"

#include <algorithm>
#include <cmath>

namespace FormationControlV2 {
namespace {
bool IsFiniteGuidance(double v) { return std::isfinite(v); }
FGuidanceCoordinatorOutputV2 Invalid(EGuidanceFailureV2 reason, std::uint32_t generation, double timestamp = 0.0) {
    FGuidanceCoordinatorOutputV2 out{}; out.FailureReason = reason; out.ResetGeneration = generation;
    if (IsFiniteGuidance(timestamp)) out.TimestampS = timestamp;
    return out;
}
bool FiniteVec(Vec2 v) { return v.IsFinite(); }
}

FormationGuidanceCoordinatorV2::FormationGuidanceCoordinatorV2() = default;
FormationGuidanceCoordinatorV2::~FormationGuidanceCoordinatorV2() = default;

namespace {
bool InRange(double v, double lo, double hi) { return IsFiniteGuidance(v) && v >= lo && v <= hi; }
bool AtLeast(double v, double lo) { return IsFiniteGuidance(v) && v >= lo; }
bool Positive(double v) { return IsFiniteGuidance(v) && v > 0.0; }
} // namespace

bool IsGuidanceConfigValid(const FGuidanceConfigV2 &config) {
    // Aircraft performance. These scale the entire TECS energy loop
    // (STE_rate_max = max_climb_rate * g, STE_rate_min = -min_sink_rate * g), so a zero or
    // non-finite value silently destroys the airspeed and throttle authority rather than failing.
    //
    // All three rates are validated identically, as the airframe properties they are: finite,
    // strictly positive, and ordered. No numeric ceiling is imposed. PX4's FW_T_SINK_MAX metadata
    // range (1-15 m/s) is a small-fixed-wing parameter range, not an aircraft performance limit, and
    // is deliberately not used here -- an airframe whose measured sink rate exceeds it must be
    // configurable, not silently rejected or clamped.
    if (!Positive(config.TecsMaxClimbRateMps)) return false;
    if (!Positive(config.TecsMinSinkRateMps)) return false;
    if (!Positive(config.TecsMaxSinkRateMps)) return false;
    if (config.TecsMinSinkRateMps > config.TecsMaxSinkRateMps) return false;
    // The trim airspeed must lie inside the demanded-airspeed envelope the same config declares.
    if (!Positive(config.EasMinMps) || !Positive(config.EasMaxMps)) return false;
    if (config.EasMinMps > config.EasMaxMps) return false;
    if (!Positive(config.TecsEquivalentAirspeedTrimMps)) return false;
    if (config.TecsEquivalentAirspeedTrimMps < config.EasMinMps ||
        config.TecsEquivalentAirspeedTrimMps > config.EasMaxMps) return false;

    // Generic controller tuning, bounded by the pinned PX4 parameter ranges.
    // vert_accel_limit: TECS defaults it to 0, which collapses the pitch slew bound in
    // TECSControl::_calcPitchControl to [_pitch_setpoint, _pitch_setpoint] -- the pitch setpoint
    // then never leaves its initial value and PitchReference is frozen at 0 forever. There is no
    // safe silent fallback, so an out-of-range value is a config error.
    if (!InRange(config.TecsVerticalAccelLimitMps2, kTecsVerticalAccelLimitMinMps2,
                 kTecsVerticalAccelLimitMaxMps2)) return false;
    if (!AtLeast(config.TecsAltitudeErrorTimeConstantS, kTecsAltitudeErrorTimeConstantMinS)) return false;
    if (!AtLeast(config.TecsAirspeedErrorTimeConstantS, kTecsAirspeedErrorTimeConstantMinS)) return false;
    if (!InRange(config.TecsPitchIntegratorGain, kTecsPitchIntegratorGainMin, kTecsPitchIntegratorGainMax)) return false;
    if (!InRange(config.TecsPitchDamping, kTecsPitchDampingMin, kTecsPitchDampingMax)) return false;
    if (!InRange(config.TecsThrottleIntegratorGain, kTecsThrottleIntegratorGainMin,
                 kTecsThrottleIntegratorGainMax)) return false;
    if (!InRange(config.TecsThrottleDamping, kTecsThrottleDampingMin, kTecsThrottleDampingMax)) return false;
    if (!InRange(config.TecsThrottleSlewRatePerS, kTecsThrottleSlewRateMinPerS,
                 kTecsThrottleSlewRateMaxPerS)) return false;
    if (!InRange(config.TecsSteRateTimeConstantS, kTecsSteRateTimeConstantMinS,
                 kTecsSteRateTimeConstantMaxS)) return false;
    if (!InRange(config.TecsSebRateFeedForwardGain, kTecsSebRateFeedForwardMin,
                 kTecsSebRateFeedForwardMax)) return false;
    if (!InRange(config.TecsAltitudeRateFeedForward, kTecsAltitudeRateFeedForwardMin,
                 kTecsAltitudeRateFeedForwardMax)) return false;
    if (!InRange(config.TecsSpeedWeight, kTecsSpeedWeightMin, kTecsSpeedWeightMax)) return false;
    if (!InRange(config.TecsRollToThrottleCompensation, kTecsRollToThrottleMin,
                 kTecsRollToThrottleMax)) return false;
    if (!InRange(config.TecsAirspeedMeasurementStdDevMps, kTecsAirspeedStdDevMin,
                 kTecsAirspeedStdDevMax)) return false;
    if (!InRange(config.TecsAirspeedRateMeasurementStdDevMps2, kTecsAirspeedStdDevMin,
                 kTecsAirspeedStdDevMax)) return false;
    if (!InRange(config.TecsAirspeedFilterProcessStdDevMps2, kTecsAirspeedStdDevMin,
                 kTecsAirspeedStdDevMax)) return false;

    // Reference-trajectory rates are a separate quantity from the energy limits above.
    if (!Positive(config.TargetClimbRateMps) || !Positive(config.TargetSinkRateMps)) return false;
    if (!IsFiniteGuidance(config.FastDescendAltitudeErrorM)) return false;
    return true;
}

// Mirrors the pinned PX4 v1.17.0 caller
// (fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp:91-112,151). Every value comes
// from FGuidanceConfigV2 -- no numeric literal is configured here, and nothing is left on a hidden
// TECS class default. The config is validated in Update() before this runs, so no invalid value can
// reach the controller and there is no fallback path.
//
// Deliberately NOT set here (per-frame runtime state, not configuration):
//   set_load_factor(bank angle), handle_alt_step(), the air-density refresh of max climb / min sink,
//   and the fast-descend slew of the altitude time constant.
void FormationGuidanceCoordinatorV2::RecreateControllers(const FGuidanceConfigV2 &config) {
    Npfg = std::make_unique<MumtPx4::FPx4NpfgAdapter>();
    Tecs = std::make_unique<MumtPx4::FPx4TecsAdapter>();
    auto &tecs = Tecs->controller();

    tecs.enable_airspeed(true);
    tecs.set_detect_underspeed_enabled(config.bDetectUnderspeed);

    // Aircraft performance envelope (PX4: fw_performance_model, absent from the pinned reference).
    tecs.set_max_climb_rate(static_cast<float>(config.TecsMaxClimbRateMps));
    tecs.set_min_sink_rate(static_cast<float>(config.TecsMinSinkRateMps));
    tecs.set_max_sink_rate(static_cast<float>(config.TecsMaxSinkRateMps));
    tecs.set_equivalent_airspeed_trim(static_cast<float>(config.TecsEquivalentAirspeedTrimMps));
    tecs.set_equivalent_airspeed_min(static_cast<float>(config.EasMinMps));
    tecs.set_equivalent_airspeed_max(static_cast<float>(config.EasMaxMps));

    // Generic controller tuning (PX4 parameters).
    tecs.set_vertical_accel_limit(static_cast<float>(config.TecsVerticalAccelLimitMps2));
    tecs.set_altitude_error_time_constant(static_cast<float>(config.TecsAltitudeErrorTimeConstantS));
    tecs.set_airspeed_error_time_constant(static_cast<float>(config.TecsAirspeedErrorTimeConstantS));
    tecs.set_integrator_gain_pitch(static_cast<float>(config.TecsPitchIntegratorGain));
    tecs.set_pitch_damping(static_cast<float>(config.TecsPitchDamping));
    tecs.set_integrator_gain_throttle(static_cast<float>(config.TecsThrottleIntegratorGain));
    tecs.set_throttle_damp(static_cast<float>(config.TecsThrottleDamping));
    tecs.set_throttle_slewrate(static_cast<float>(config.TecsThrottleSlewRatePerS));
    tecs.set_ste_rate_time_const(static_cast<float>(config.TecsSteRateTimeConstantS));
    tecs.set_seb_rate_ff_gain(static_cast<float>(config.TecsSebRateFeedForwardGain));
    tecs.set_altitude_rate_ff(static_cast<float>(config.TecsAltitudeRateFeedForward));
    tecs.set_speed_weight(static_cast<float>(config.TecsSpeedWeight));
    tecs.set_roll_throttle_compensation(static_cast<float>(config.TecsRollToThrottleCompensation));
    tecs.set_fast_descend_altitude_error(static_cast<float>(config.FastDescendAltitudeErrorM));
    tecs.set_airspeed_measurement_std_dev(static_cast<float>(config.TecsAirspeedMeasurementStdDevMps));
    tecs.set_airspeed_rate_measurement_std_dev(static_cast<float>(config.TecsAirspeedRateMeasurementStdDevMps2));
    tecs.set_airspeed_filter_process_std_dev(static_cast<float>(config.TecsAirspeedFilterProcessStdDevMps2));
}

void FormationGuidanceCoordinatorV2::Reset(std::uint32_t generation) {
    Npfg.reset(); Tecs.reset(); LastResetGeneration = generation; LastSimulationTimeS = 0.0;
    bInitialized = false; bWasPaused = false;
}

FGuidanceCoordinatorOutputV2 FormationGuidanceCoordinatorV2::Update(const FGuidanceCoordinatorInputV2 &in,
                                                                    const FGuidanceConfigV2 &config) {
    if (!in.bControllerEnabled) return Invalid(EGuidanceFailureV2::Disabled, in.ResetGeneration, in.SimulationTimeS);
    if (!in.bShadowEnabled) return Invalid(EGuidanceFailureV2::ShadowDisabled, in.ResetGeneration, in.SimulationTimeS);
    // Rejected BEFORE RecreateControllers, so a controller is never built with an unusable TECS
    // envelope. No silent fallback: a bad limit is a config error, not something to paper over.
    if (!IsGuidanceConfigValid(config)) return Invalid(EGuidanceFailureV2::InvalidConfig, in.ResetGeneration, in.SimulationTimeS);
    if (!IsFiniteGuidance(in.SimulationTimeS) || !IsFiniteGuidance(in.DtS) || in.SimulationTimeS < 0.0 || in.DtS < 0.0)
        return Invalid(EGuidanceFailureV2::InvalidTime, in.ResetGeneration);
    if (!bInitialized || in.ResetGeneration != LastResetGeneration) {
        RecreateControllers(config); LastResetGeneration = in.ResetGeneration; LastSimulationTimeS = in.SimulationTimeS;
        bInitialized = true; bWasPaused = in.bPaused || in.Follower.bPaused;
        return Invalid(EGuidanceFailureV2::ResetFrame, in.ResetGeneration, in.SimulationTimeS);
    }
    if (in.bPaused || in.Follower.bPaused) {
        bWasPaused = true; LastSimulationTimeS = in.SimulationTimeS;
        return Invalid(EGuidanceFailureV2::Paused, in.ResetGeneration, in.SimulationTimeS);
    }
    if (bWasPaused) {
        bWasPaused = false; LastSimulationTimeS = in.SimulationTimeS;
        return Invalid(EGuidanceFailureV2::ResetFrame, in.ResetGeneration, in.SimulationTimeS);
    }
    if (in.ResetGeneration != in.Follower.ResetGeneration || in.ResetGeneration != in.Slot.ResetGeneration)
        return Invalid(EGuidanceFailureV2::ResetMismatch, in.ResetGeneration, in.SimulationTimeS);
    if (!in.Follower.bOriginValid || in.OriginGeneration != in.Follower.OriginGeneration ||
        in.OriginGeneration != in.Slot.OriginGeneration)
        return Invalid(EGuidanceFailureV2::OriginMismatch, in.ResetGeneration, in.SimulationTimeS);
    if (!in.Follower.bPositionValid || !in.Follower.bGroundVelocityValid || !in.Follower.bEasValid ||
        !in.Follower.bTasValid || !in.Follower.bAltitudeValid || !in.Follower.bClimbRateValid ||
        !in.Follower.bSimulationTimeValid || !in.bCurrentPitchValid)
        return Invalid(EGuidanceFailureV2::InvalidFollower, in.ResetGeneration, in.SimulationTimeS);
    if (!in.Follower.bWindValid) return Invalid(EGuidanceFailureV2::InvalidWind, in.ResetGeneration, in.SimulationTimeS);
    if (!in.Follower.bRatioValid || !in.PlannerDto.Npfg.bValid || !in.PlannerDto.Tecs.bCommandReady ||
        !in.PlannerDto.Tecs.bTargetEasValid || !in.PlannerDto.Tecs.bTargetAltitudeValid)
        return Invalid(EGuidanceFailureV2::InvalidPlannerDto, in.ResetGeneration, in.SimulationTimeS);
    const double tangentNorm = in.PlannerDto.Npfg.PathUnitTangentNE.Norm();
    if (!IsFiniteGuidance(tangentNorm) || tangentNorm < 1e-6)
        return Invalid(EGuidanceFailureV2::ZeroTangent, in.ResetGeneration, in.SimulationTimeS);
    const bool finite = FiniteVec(in.Follower.PositionNE_m) && FiniteVec(in.Follower.GroundVelocityNE_mps) &&
        FiniteVec(in.Follower.WindNE_mps) && FiniteVec(in.PlannerDto.Npfg.PathPositionNE_m) &&
        FiniteVec(in.PlannerDto.Npfg.PathUnitTangentNE) && IsFiniteGuidance(in.PlannerDto.Npfg.PathCurvature_per_m) &&
        IsFiniteGuidance(in.PlannerDto.Tecs.TargetEasMps) && IsFiniteGuidance(in.PlannerDto.Tecs.TargetAltitudeAslM) &&
        IsFiniteGuidance(in.Follower.EquivalentAirspeed_mps) && IsFiniteGuidance(in.Follower.TrueAirspeed_mps) &&
        IsFiniteGuidance(in.Follower.EasToTasRatio) && in.Follower.EasToTasRatio > 0.0 &&
        IsFiniteGuidance(in.CurrentPitchRad) && IsFiniteGuidance(in.Follower.AltitudeAsl_m) &&
        IsFiniteGuidance(in.Follower.ClimbRate_mps);
    if (!finite) return Invalid(EGuidanceFailureV2::NonFiniteInput, in.ResetGeneration, in.SimulationTimeS);

    const Vec2 tangent = in.PlannerDto.Npfg.PathUnitTangentNE * (1.0 / tangentNorm);
    MumtPx4::NpfgInput ni{};
    ni.position_ne = {static_cast<float>(in.Follower.PositionNE_m.N), static_cast<float>(in.Follower.PositionNE_m.E)};
    ni.ground_velocity_ne = {static_cast<float>(in.Follower.GroundVelocityNE_mps.N), static_cast<float>(in.Follower.GroundVelocityNE_mps.E)};
    ni.wind_velocity_ne = {static_cast<float>(in.Follower.WindNE_mps.N), static_cast<float>(in.Follower.WindNE_mps.E)};
    ni.path_tangent_ne = {static_cast<float>(tangent.N), static_cast<float>(tangent.E)};
    ni.path_position_ne = {static_cast<float>(in.PlannerDto.Npfg.PathPositionNE_m.N), static_cast<float>(in.PlannerDto.Npfg.PathPositionNE_m.E)};
    // Both Planner and PX4 NPFG use N/E and right-positive signed curvature; no sign inversion.
    ni.path_curvature = static_cast<float>(in.PlannerDto.Npfg.PathCurvature_per_m);
    ni.true_airspeed_setpoint = static_cast<float>(in.PlannerDto.Tecs.TargetEasMps * in.Follower.EasToTasRatio);
    ni.true_airspeed = static_cast<float>(in.Follower.TrueAirspeed_mps);
    ni.minimum_ground_speed = static_cast<float>(config.MinimumGroundSpeedMps);
    const MumtPx4::NpfgOutput no = Npfg->update(ni);

    MumtPx4::TecsInput ti{};
    ti.timestamp_us = static_cast<std::uint64_t>(std::llround(in.SimulationTimeS * 1.0e6));
    ti.pitch = static_cast<float>(in.CurrentPitchRad); ti.altitude = static_cast<float>(in.Follower.AltitudeAsl_m);
    ti.altitude_setpoint = static_cast<float>(in.PlannerDto.Tecs.TargetAltitudeAslM);
    ti.eas_setpoint = static_cast<float>(in.PlannerDto.Tecs.TargetEasMps);
    ti.equivalent_airspeed = static_cast<float>(in.Follower.EquivalentAirspeed_mps);
    ti.eas_to_tas = static_cast<float>(in.Follower.EasToTasRatio);
    ti.throttle_min = static_cast<float>(config.ThrottleMin); ti.throttle_max = static_cast<float>(config.ThrottleMax);
    ti.throttle_trim = static_cast<float>(config.ThrottleTrim); ti.pitch_min = static_cast<float>(config.PitchMinRad);
    ti.pitch_max = static_cast<float>(config.PitchMaxRad); ti.target_climb_rate = static_cast<float>(config.TargetClimbRateMps);
    ti.target_sink_rate = static_cast<float>(config.TargetSinkRateMps); ti.forward_airspeed_acceleration = 0.0f;
    ti.altitude_rate = static_cast<float>(in.Follower.ClimbRate_mps);
    ti.altitude_rate_setpoint = in.PlannerDto.Tecs.bTargetClimbRateValid ? static_cast<float>(in.PlannerDto.Tecs.TargetClimbRateMps) : NAN;
    const MumtPx4::TecsOutput to = Tecs->update(ti);

    FGuidanceCoordinatorOutputV2 out{};
    out.RollReferenceRad = std::clamp<double>(no.roll_setpoint, -config.RollLimitRad, config.RollLimitRad);
    out.PitchReferenceRad = to.pitch_setpoint; out.ThrottleReferenceNorm = to.throttle_setpoint;
    out.LateralAccelerationFeedforwardMps2 = no.lateral_acceleration_feedforward;
    out.LateralAccelerationFeedbackMps2 = no.lateral_acceleration_feedback;
    out.LateralAccelerationTotalMps2 = no.lateral_acceleration_total; out.CourseSetpointRad = no.course_setpoint;
    out.WindFeasibility = no.wind_feasibility; out.UnderspeedRatio = to.underspeed_ratio; out.FastDescendRatio = to.fast_descend;
    out.TotalEnergyRateSetpoint = to.total_energy_rate_sp; out.TotalEnergyRateEstimate = to.total_energy_rate_estimate;
    out.EnergyBalanceRateSetpoint = to.energy_balance_rate_sp; out.EnergyBalanceRateEstimate = to.energy_balance_rate_estimate;
    out.PitchIntegrator = to.pitch_integrator; out.ThrottleIntegrator = to.throttle_integrator;
    out.TimestampS = in.SimulationTimeS; out.ResetGeneration = in.ResetGeneration;
    const double values[] = {out.RollReferenceRad,out.PitchReferenceRad,out.ThrottleReferenceNorm,out.LateralAccelerationFeedforwardMps2,
        out.LateralAccelerationFeedbackMps2,out.LateralAccelerationTotalMps2,out.CourseSetpointRad,out.WindFeasibility,
        out.UnderspeedRatio,out.FastDescendRatio,out.TotalEnergyRateSetpoint,out.TotalEnergyRateEstimate,
        out.EnergyBalanceRateSetpoint,out.EnergyBalanceRateEstimate,out.PitchIntegrator,out.ThrottleIntegrator};
    for (double value : values) if (!IsFiniteGuidance(value)) return Invalid(EGuidanceFailureV2::NonFiniteOutput, in.ResetGeneration, in.SimulationTimeS);
    out.bNpfgValid = true; out.bTecsValid = true; out.bCommandReady = true; out.FailureReason = EGuidanceFailureV2::None;
    LastSimulationTimeS = in.SimulationTimeS; return out;
}

} // namespace FormationControlV2
