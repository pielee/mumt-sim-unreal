// verify_tecs_caller_contract_v2.cpp — the TECS *caller* contract, not the TECS core.
//
// The 594-row TECS equivalence audit proves the ported control law matches PX4. It does NOT prove
// that FormationGuidanceCoordinatorV2 configures that controller the way PX4's own caller does:
// the equivalence harness sets the TECS parameters itself. This audit closes exactly that gap.
//
// It drives the production caller path — FormationGuidanceCoordinatorV2 -> FPx4TecsAdapter -> the
// ported TECS — using a manually constructed input DTO that satisfies the coordinator's actual
// contract. It does not prove the general PlannerV2OutputAdapter output path end to end.
//
// This audit proves the vertical-acceleration config and setter wiring, stateful pitch response in
// altitude-demand and rate-valid modes, and invalid-config/reset/pause/finite/range/slew contracts.
// It does not prove F-16 closed-loop response, Active convergence, or that the configured F-16
// climb/sink/EAS performance envelopes are appropriate.
//
// The defect it locks down: TECS defaults vert_accel_limit to 0, and
// TECSControl::_calcPitchControl derives the pitch slew rate from it
//     pitch_increment = dt * vert_accel_limit / max(tas, eps)
// so at 0 the slew bound collapses to [_pitch_setpoint, _pitch_setpoint] and the pitch setpoint is
// frozen at its initial value forever. An unconfigured caller therefore reports PitchReference == 0
// on every frame, in every flight condition. Pinned PX4 v1.17.0
// (d6f12ad1c4f70ad3230afd7d86e971421e02fef4) sets it from FW_T_VERT_ACC (@min 1, @max 10, def 7).
#include "FormationControlV2/FormationGuidanceCoordinatorV2.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>

using namespace FormationControlV2;
namespace {
int Failures{}, Checks{};
void Check(bool v, const char *name) { ++Checks; if (!v) { ++Failures; std::cerr << "FAIL " << name << '\n'; } }

constexpr double kDt = 1.0 / 60.0;
constexpr double kBaseAltM = 800.0;
constexpr double kEasMps = 60.0;          // inside the production EasMin/EasMax envelope (10..80)
constexpr double kEasToTas = 1.05;
constexpr std::uint32_t kOriginGen = 7;

// Build a valid coordinator input DTO by hand; this is not a PlannerV2OutputAdapter E2E oracle.
FGuidanceCoordinatorInputV2 MakeInput(double timeS, std::uint32_t resetGen, double altitudeM,
                                      double climbRateMps, double targetAltitudeM, double targetEasMps,
                                      double easMps, double currentPitchRad,
                                      bool targetClimbRateValid = false, double targetClimbRateMps = 0.0) {
    FGuidanceCoordinatorInputV2 in{};
    auto &f = in.Follower;
    f.PositionNE_m = {0.0, 0.0};
    f.GroundVelocityNE_mps = {easMps * kEasToTas, 0.0};
    f.GroundCourse_rad = 0.0;
    f.AltitudeAsl_m = altitudeM;
    f.ClimbRate_mps = climbRateMps;
    f.SimulationTimeS = timeS;
    f.ResetGeneration = resetGen;
    f.OriginGeneration = kOriginGen;
    f.EquivalentAirspeed_mps = easMps;
    f.TrueAirspeed_mps = easMps * kEasToTas;
    f.WindNE_mps = {0.0, 0.0};
    f.EasToTasRatio = kEasToTas;
    f.bPositionValid = f.bGroundVelocityValid = f.bGroundCourseValid = true;
    f.bAltitudeValid = f.bClimbRateValid = f.bSimulationTimeValid = true;
    f.bEasValid = f.bTasValid = f.bWindValid = f.bRatioValid = f.bOriginValid = true;

    in.Slot.ResetGeneration = resetGen;
    in.Slot.OriginGeneration = kOriginGen;

    in.PlannerDto.Npfg.PathPositionNE_m = {0.0, 0.0};
    in.PlannerDto.Npfg.PathUnitTangentNE = {1.0, 0.0};
    in.PlannerDto.Npfg.PathCurvature_per_m = 0.0;
    in.PlannerDto.Npfg.bValid = true;

    in.PlannerDto.Tecs.TargetEasMps = targetEasMps;
    in.PlannerDto.Tecs.TargetAltitudeAslM = targetAltitudeM;
    in.PlannerDto.Tecs.TargetClimbRateMps = targetClimbRateMps;
    in.PlannerDto.Tecs.bTargetEasValid = true;
    in.PlannerDto.Tecs.bTargetAltitudeValid = true;
    in.PlannerDto.Tecs.bTargetClimbRateValid = targetClimbRateValid;
    in.PlannerDto.Tecs.bCommandReady = true;

    in.CurrentPitchRad = currentPitchRad;
    in.bCurrentPitchValid = true;
    in.SimulationTimeS = timeS;
    in.DtS = kDt;
    in.ResetGeneration = resetGen;
    in.OriginGeneration = kOriginGen;
    in.bControllerEnabled = true;
    in.bShadowEnabled = true;
    return in;
}

struct Run {
    double PitchMin{1e9}, PitchMax{-1e9}, ThrMin{1e9}, ThrMax{-1e9};
    double FirstNonZeroS{-1.0}, MaxAbsPitchStep{0.0}, SettledPitch{0.0};
    int ValidFrames{}, NonFiniteFrames{}, StaleGenFrames{}, SlewViolations{}, ThrottleRangeViolations{};
    // Public coordinator outputs only. A configured TECS parameter that never reaches the controller
    // cannot move any of these, so a difference here IS the proof that the setter is wired.
    double LastPitch{}, LastThrottle{}, LastSteRateSp{}, LastSteRateEstimate{}, LastSebRateSp{};
    double LastPitchIntegrator{}, LastThrottleIntegrator{}, LastUnderspeed{};
    double MaxAbsThrottleStep{0.0};

    // Exact equality: the coordinator is deterministic, so any wiring difference shows up bit-exact.
    bool SameSignature(const Run &o) const {
        return LastPitch == o.LastPitch && LastThrottle == o.LastThrottle &&
               LastSteRateSp == o.LastSteRateSp && LastSteRateEstimate == o.LastSteRateEstimate &&
               LastSebRateSp == o.LastSebRateSp && LastPitchIntegrator == o.LastPitchIntegrator &&
               LastThrottleIntegrator == o.LastThrottleIntegrator && LastUnderspeed == o.LastUnderspeed;
    }
};

// One 10 s stateful run of the PRODUCTION coordinator.
// easRampMpsPerS ramps the MEASURED airspeed. Filter and time-constant parameters only change the
// output while the signal they filter is actually moving: with a constant measurement they all
// settle on the same steady state, so a static scenario cannot prove they are wired.
Run Fly(const FGuidanceConfigV2 &cfg, double targetAltitudeM, double climbRateMps, double targetEasMps,
        double easMps = kEasMps, int frames = 600,
        bool targetClimbRateValid = false, double targetClimbRateMps = 0.0,
        double easRampMpsPerS = 0.0) {
    FormationGuidanceCoordinatorV2 g;
    Run r{};
    double prevPitch = 0.0, prevThrottle = 0.0; bool havePrev = false;
    for (int k = 0; k < frames; ++k) {
        const double t = 10.0 + k * kDt;
        const double easNow = easMps + easRampMpsPerS * (k * kDt);
        const auto in = MakeInput(t, 1u, kBaseAltM, climbRateMps, targetAltitudeM, targetEasMps,
                                  easNow, 0.0, targetClimbRateValid, targetClimbRateMps);
        const auto o = g.Update(in, cfg);
        if (!o.bCommandReady) continue;                 // frame 0 is the ResetFrame by contract
        ++r.ValidFrames;
        if (o.ResetGeneration != in.ResetGeneration) ++r.StaleGenFrames;
        if (!std::isfinite(o.PitchReferenceRad) || !std::isfinite(o.ThrottleReferenceNorm)) { ++r.NonFiniteFrames; continue; }
        r.PitchMin = std::min(r.PitchMin, o.PitchReferenceRad);
        r.PitchMax = std::max(r.PitchMax, o.PitchReferenceRad);
        r.ThrMin = std::min(r.ThrMin, o.ThrottleReferenceNorm);
        r.ThrMax = std::max(r.ThrMax, o.ThrottleReferenceNorm);
        if (o.ThrottleReferenceNorm < cfg.ThrottleMin - 1e-9 || o.ThrottleReferenceNorm > cfg.ThrottleMax + 1e-9)
            ++r.ThrottleRangeViolations;
        if (r.FirstNonZeroS < 0.0 && std::abs(o.PitchReferenceRad) > 1e-6) r.FirstNonZeroS = k * kDt;
        if (havePrev) {
            const double step = std::abs(o.PitchReferenceRad - prevPitch);
            r.MaxAbsPitchStep = std::max(r.MaxAbsPitchStep, step);
            // TECS bounds the pitch setpoint change to dt * vert_accel_limit / TAS per frame.
            const double bound = kDt * cfg.TecsVerticalAccelLimitMps2 / (easNow * kEasToTas);
            if (step > bound + 1e-6) ++r.SlewViolations;
        }
        if (havePrev) r.MaxAbsThrottleStep = std::max(r.MaxAbsThrottleStep, std::abs(o.ThrottleReferenceNorm - prevThrottle));
        prevPitch = o.PitchReferenceRad; prevThrottle = o.ThrottleReferenceNorm; havePrev = true;
        r.SettledPitch = o.PitchReferenceRad;
        r.LastPitch = o.PitchReferenceRad;
        r.LastThrottle = o.ThrottleReferenceNorm;
        r.LastSteRateSp = o.TotalEnergyRateSetpoint;
        r.LastSteRateEstimate = o.TotalEnergyRateEstimate;
        r.LastSebRateSp = o.EnergyBalanceRateSetpoint;
        r.LastPitchIntegrator = o.PitchIntegrator;
        r.LastThrottleIntegrator = o.ThrottleIntegrator;
        r.LastUnderspeed = o.UnderspeedRatio;
    }
    return r;
}
} // namespace

int main() {
    FGuidanceConfigV2 cfg{};    // production defaults
    Check(cfg.TecsVerticalAccelLimitMps2 == 7.0, "production_default_vert_accel_limit_is_px4_fw_t_vert_acc");
    Check(kTecsVerticalAccelLimitMinMps2 == 1.0 && kTecsVerticalAccelLimitMaxMps2 == 10.0,
          "config_bounds_match_px4_fw_t_vert_acc_range");
    Check(IsGuidanceConfigValid(cfg), "production_default_config_valid");

    std::printf("# TECS caller contract (production coordinator -> FPx4TecsAdapter -> ported TECS)\n");
    std::printf("# vertAccelLimit=%.1f m/s^2 (PX4 FW_T_VERT_ACC), eas=%.0f m/s, tas=%.1f m/s, 10 s @ 60 Hz\n",
                cfg.TecsVerticalAccelLimitMps2, kEasMps, kEasMps * kEasToTas);
    std::printf("%-26s %10s %10s %9s %9s %11s %10s\n",
                "case", "pitchMin", "pitchMax", "thrMin", "thrMax", "1stNonZeroS", "maxStep");

    struct Case {
        const char *Group;
        const char *Name;
        double TargetAlt, MeasuredClimbRate, TargetEas;
        bool TargetClimbRateValid;
        double TargetClimbRate;
    };
    const Case cases[] = {
        {"ALTITUDE_DEMAND", "1 altitude hold", kBaseAltM, 0.0, kEasMps, false, 0.0},
        {"ALTITUDE_DEMAND", "2 climb +100 m", kBaseAltM + 100.0, 0.0, kEasMps, false, 0.0},
        {"ALTITUDE_DEMAND", "3 descent -100 m", kBaseAltM - 100.0, 0.0, kEasMps, false, 0.0},
        {"DISTURBANCE", "4 measured climb-rate disturbance +10", kBaseAltM, +10.0, kEasMps, false, 0.0},
        {"DISTURBANCE", "5 measured climb-rate disturbance -10", kBaseAltM, -10.0, kEasMps, false, 0.0},
        {"DISTURBANCE", "6 EAS below target", kBaseAltM, 0.0, kEasMps + 15.0, false, 0.0},
        {"DISTURBANCE", "7 EAS above target", kBaseAltM, 0.0, kEasMps - 15.0, false, 0.0},
        {"RATE_VALID", "8 target climb +5", kBaseAltM, 0.0, kEasMps, true, +5.0},
        {"RATE_VALID", "9 target climb -5", kBaseAltM, 0.0, kEasMps, true, -5.0},
    };
    constexpr int kCaseCount = static_cast<int>(sizeof(cases) / sizeof(cases[0]));
    Run runs[kCaseCount];
    const char *lastGroup = nullptr;
    for (int k = 0; k < kCaseCount; ++k) {
        if (lastGroup == nullptr || std::string(cases[k].Group) != lastGroup) {
            std::printf("\n[%s]\n", cases[k].Group);
            lastGroup = cases[k].Group;
        }
        runs[k] = Fly(cfg, cases[k].TargetAlt, cases[k].MeasuredClimbRate, cases[k].TargetEas,
                      kEasMps, 600, cases[k].TargetClimbRateValid, cases[k].TargetClimbRate);
        const Run &r = runs[k];
        std::printf("%-26s %10.6f %10.6f %9.4f %9.4f %11s %10.6f\n", cases[k].Name,
                    r.PitchMin, r.PitchMax, r.ThrMin, r.ThrMax,
                    r.FirstNonZeroS < 0 ? "never" : std::to_string(r.FirstNonZeroS).substr(0, 5).c_str(),
                    r.MaxAbsPitchStep);
    }

    // ---- every case: finite, in range, no stale generation, slew bound respected ----
    for (int k = 0; k < kCaseCount; ++k) {
        const Run &r = runs[k];
        Check(r.ValidFrames > 500, "sufficient_valid_frames");
        Check(r.NonFiniteFrames == 0, "pitch_and_throttle_finite");
        Check(r.StaleGenFrames == 0, "no_stale_reset_generation");
        Check(r.ThrottleRangeViolations == 0, "throttle_within_configured_range");
        Check(r.SlewViolations == 0, "pitch_step_within_vertical_accel_slew_bound");
    }

    // ---- THE regression this audit exists for ----
    // Climb demand must raise the nose; descent demand must lower it. Both were identically 0
    // before set_vertical_accel_limit was called.
    Check(runs[1].PitchMax > 1e-4, "climb_command_produces_positive_pitch_reference");
    Check(runs[2].PitchMin < -1e-4, "descent_command_produces_negative_pitch_reference");
    Check(runs[1].FirstNonZeroS >= 0.0 && runs[1].FirstNonZeroS <= 0.5, "climb_pitch_responds_within_500ms");
    Check(runs[2].FirstNonZeroS >= 0.0 && runs[2].FirstNonZeroS <= 0.5, "descent_pitch_responds_within_500ms");
    Check(runs[3].FirstNonZeroS >= 0.0, "positive_climbrate_disturbance_moves_pitch");
    Check(runs[4].FirstNonZeroS >= 0.0, "negative_climbrate_disturbance_moves_pitch");
    Check(runs[5].FirstNonZeroS >= 0.0, "eas_below_target_moves_pitch");
    Check(runs[6].FirstNonZeroS >= 0.0, "eas_above_target_moves_pitch");
    Check(runs[7].PitchMax > 1e-4, "positive_target_climb_rate_produces_positive_pitch_reference");
    Check(runs[8].PitchMin < -1e-4, "negative_target_climb_rate_produces_negative_pitch_reference");
    Check(runs[7].FirstNonZeroS >= 0.0 && runs[7].FirstNonZeroS <= 0.5,
          "positive_target_climb_rate_responds_within_500ms");
    Check(runs[8].FirstNonZeroS >= 0.0 && runs[8].FirstNonZeroS <= 0.5,
          "negative_target_climb_rate_responds_within_500ms");
    // Both pitch-reference directions must be exercised across the caller-contract suite.
    bool anyPos = false, anyNeg = false;
    for (const Run &r : runs) { if (r.PitchMax > 1e-4) anyPos = true; if (r.PitchMin < -1e-4) anyNeg = true; }
    Check(anyPos && anyNeg, "pitch_reference_covers_both_directions");
    // Altitude hold must not sit on a large standing bias.
    Check(std::abs(runs[0].SettledPitch) < 0.05, "altitude_hold_has_no_large_pitch_bias");

    // ---- invalid config is REJECTED, never silently substituted ----
    // A zero limit is exactly the frozen-pitch defect; it must be a config error, not a mode.
    struct Bad { const char *Name; double Limit; };
    const Bad bad[] = {
        {"zero", 0.0},
        {"negative", -1.0},
        {"below_px4_min", 0.5},
        {"above_px4_max", 25.0},
        {"nan", std::numeric_limits<double>::quiet_NaN()},
        {"inf", std::numeric_limits<double>::infinity()},
    };
    for (const Bad &b : bad) {
        FGuidanceConfigV2 c{}; c.TecsVerticalAccelLimitMps2 = b.Limit;
        Check(!IsGuidanceConfigValid(c), "config_predicate_rejects_bad_limit");
        FormationGuidanceCoordinatorV2 g;
        const auto o = g.Update(MakeInput(10.0, 1u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0), c);
        Check(!o.bCommandReady && o.FailureReason == EGuidanceFailureV2::InvalidConfig,
              "coordinator_rejects_bad_limit_as_InvalidConfig");
        Check(o.PitchReferenceRad == 0.0 && o.ThrottleReferenceNorm == 0.0, "rejected_frame_emits_no_command");
    }
    // Boundary values of the PX4 range are accepted.
    for (double v : {kTecsVerticalAccelLimitMinMps2, kTecsVerticalAccelLimitMaxMps2}) {
        FGuidanceConfigV2 c{}; c.TecsVerticalAccelLimitMps2 = v;
        Check(IsGuidanceConfigValid(c), "px4_range_boundary_accepted");
    }

    // ---- reset generation change re-initialises cleanly ----
    {
        FormationGuidanceCoordinatorV2 g;
        for (int k = 0; k < 120; ++k)
            g.Update(MakeInput(10.0 + k * kDt, 1u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0), cfg);
        const auto reset = g.Update(MakeInput(12.0, 2u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0), cfg);
        Check(!reset.bCommandReady && reset.FailureReason == EGuidanceFailureV2::ResetFrame, "reset_generation_change_yields_reset_frame");
        const auto after = g.Update(MakeInput(12.0 + kDt, 2u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0), cfg);
        Check(after.bCommandReady && after.ResetGeneration == 2u, "planner_valid_again_after_reset");
        Check(std::isfinite(after.PitchReferenceRad), "pitch_finite_after_reset");
    }

    // ---- pause holds; dt == 0 must not advance or corrupt state ----
    {
        FormationGuidanceCoordinatorV2 g;
        for (int k = 0; k < 120; ++k)
            g.Update(MakeInput(10.0 + k * kDt, 1u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0), cfg);
        auto paused = MakeInput(12.0, 1u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0);
        paused.bPaused = true; paused.DtS = 0.0;
        const auto po = g.Update(paused, cfg);
        Check(!po.bCommandReady && po.FailureReason == EGuidanceFailureV2::Paused, "pause_reports_paused");
        Check(po.PitchReferenceRad == 0.0, "paused_frame_emits_no_command");
        const auto resume = g.Update(MakeInput(12.0 + kDt, 1u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0), cfg);
        Check(!resume.bCommandReady && resume.FailureReason == EGuidanceFailureV2::ResetFrame, "resume_is_a_reset_frame");
        const auto next = g.Update(MakeInput(12.0 + 2 * kDt, 1u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0), cfg);
        Check(next.bCommandReady && std::isfinite(next.PitchReferenceRad), "valid_again_after_resume");
    }

    // ============================================================================================
    // Caller-contract completeness.
    //
    // The pinned PX4 v1.17.0 caller (FwLateralLongitudinalControl.cpp:91-112,151) configures TECS
    // through a fixed set of setters. Anything the coordinator does not set keeps the TECS *class*
    // default -- a placeholder PX4 itself never runs with. This section proves each parameter is
    // actually plumbed from FGuidanceConfigV2, using PUBLIC coordinator outputs only: a parameter
    // that never reaches the controller cannot change any output, so an observable difference
    // between two configs IS the wiring proof. No production getter or debug hook is added.
    // ============================================================================================

    // ---- (a) generic tuning: production defaults are the pinned PX4 parameter defaults ----
    Check(cfg.TecsAltitudeErrorTimeConstantS == 5.0, "default_alt_error_tc_is_px4_FW_T_ALT_TC");
    Check(cfg.TecsAirspeedErrorTimeConstantS == 5.0, "default_tas_error_tc_is_px4_FW_T_TAS_TC");
    Check(cfg.TecsPitchIntegratorGain == 0.1, "default_pitch_integrator_is_px4_FW_T_I_GAIN_PIT");
    Check(cfg.TecsPitchDamping == 0.1, "default_pitch_damping_is_px4_FW_T_PTCH_DAMP");
    Check(cfg.TecsThrottleIntegratorGain == 0.02, "default_throttle_integrator_is_px4_FW_T_THR_INTEG");
    Check(cfg.TecsThrottleDamping == 0.05, "default_throttle_damping_is_px4_FW_T_THR_DAMPING");
    Check(cfg.TecsThrottleSlewRatePerS == 0.0, "default_throttle_slewrate_is_px4_FW_THR_SLEW_MAX");
    Check(cfg.TecsSteRateTimeConstantS == 0.4, "default_ste_rate_tc_is_px4_FW_T_STE_R_TC");
    Check(cfg.TecsSebRateFeedForwardGain == 1.0, "default_seb_rate_ff_is_px4_FW_T_SEB_R_FF");
    Check(cfg.TecsAltitudeRateFeedForward == 0.3, "default_altitude_rate_ff_is_px4_FW_T_HRATE_FF");
    Check(cfg.TecsSpeedWeight == 1.0, "default_speed_weight_is_px4_FW_T_SPDWEIGHT");
    Check(cfg.TecsRollToThrottleCompensation == 15.0, "default_roll_to_throttle_is_px4_FW_T_RLL2THR");
    Check(cfg.TecsAirspeedMeasurementStdDevMps == 0.07, "default_airspeed_std_dev_is_px4_FW_T_SPD_STD");
    Check(cfg.TecsAirspeedRateMeasurementStdDevMps2 == 0.2, "default_airspeed_rate_std_dev_is_px4_FW_T_SPD_DEV_STD");
    Check(cfg.TecsAirspeedFilterProcessStdDevMps2 == 0.2, "default_airspeed_process_std_dev_is_px4_FW_T_SPD_PRC_STD");
    Check(cfg.TecsMaxSinkRateMps == 5.0, "default_max_sink_rate_is_px4_FW_T_SINK_MAX");

    // ---- (b) aircraft performance: PX4 reads these from the ABSENT fw_performance_model, so the
    // defaults deliberately reproduce the previous TECS class defaults. They are now explicit,
    // validated and configurable instead of hidden, and they assert no F-16 value.
    Check(cfg.TecsMaxClimbRateMps == 5.0, "default_max_climb_rate_preserves_prior_tecs_behaviour");
    Check(cfg.TecsMinSinkRateMps == 2.0, "default_min_sink_rate_preserves_prior_tecs_behaviour");
    Check(cfg.TecsEquivalentAirspeedTrimMps == 15.0, "default_eas_trim_preserves_prior_tecs_behaviour");
    // Reference-trajectory rates are a DIFFERENT quantity from the energy limits and must not alias.
    Check(cfg.TargetClimbRateMps != cfg.TecsMaxClimbRateMps, "target_climb_rate_is_not_max_climb_rate");
    Check(cfg.TargetSinkRateMps != cfg.TecsMaxSinkRateMps, "target_sink_rate_is_not_max_sink_rate");

    // ---- plumbing: changing a parameter must move a public output ----
    const Run baseClimb = Fly(cfg, kBaseAltM + 100.0, 0.0, kEasMps);
    const Run baseDescent = Fly(cfg, kBaseAltM - 100.0, 0.0, kEasMps);
    constexpr double kUnderspeedEasMps = 6.5;   // measured EAS below EasMin: exercises _detectUnderspeed
    const Run baseUnderspeed = Fly(cfg, kBaseAltM, 0.0, kEasMps, kUnderspeedEasMps);

    auto ClimbDiffers = [&](double FGuidanceConfigV2::*field, double value, const char *name) {
        FGuidanceConfigV2 c = cfg; c.*field = value;
        Check(IsGuidanceConfigValid(c), "variant_config_valid");
        Check(!Fly(c, kBaseAltM + 100.0, 0.0, kEasMps).SameSignature(baseClimb), name);
    };
    auto DescentDiffers = [&](double FGuidanceConfigV2::*field, double value, const char *name) {
        FGuidanceConfigV2 c = cfg; c.*field = value;
        Check(IsGuidanceConfigValid(c), "variant_config_valid");
        Check(!Fly(c, kBaseAltM - 100.0, 0.0, kEasMps).SameSignature(baseDescent), name);
    };

    ClimbDiffers(&FGuidanceConfigV2::TecsMaxClimbRateMps, 12.0, "max_climb_rate_reaches_tecs");
    DescentDiffers(&FGuidanceConfigV2::TecsMinSinkRateMps, 4.0, "min_sink_rate_reaches_tecs");
    DescentDiffers(&FGuidanceConfigV2::TecsMaxSinkRateMps, 12.0, "max_sink_rate_reaches_tecs");
    ClimbDiffers(&FGuidanceConfigV2::TecsAltitudeErrorTimeConstantS, 12.0, "altitude_error_tc_reaches_tecs");
    ClimbDiffers(&FGuidanceConfigV2::TecsPitchIntegratorGain, 0.8, "pitch_integrator_gain_reaches_tecs");
    ClimbDiffers(&FGuidanceConfigV2::TecsPitchDamping, 1.0, "pitch_damping_reaches_tecs");
    ClimbDiffers(&FGuidanceConfigV2::TecsThrottleIntegratorGain, 0.5, "throttle_integrator_gain_reaches_tecs");
    ClimbDiffers(&FGuidanceConfigV2::TecsThrottleDamping, 0.6, "throttle_damping_reaches_tecs");
    ClimbDiffers(&FGuidanceConfigV2::TecsSebRateFeedForwardGain, 2.5, "seb_rate_ff_gain_reaches_tecs");
    ClimbDiffers(&FGuidanceConfigV2::TecsSpeedWeight, 0.2, "speed_weight_reaches_tecs");

    // The airspeed error time constant, the STE-rate filter and the three airspeed-filter standard
    // deviations only shape a MOVING signal: with a constant, error-free measurement they all settle
    // on the same steady state and are invisible by construction. They are therefore exercised
    // against a demanded airspeed error and a ramping measured airspeed -- a dynamic scenario, not a
    // relaxed threshold.
    constexpr double kEasRampMpsPerS = 2.0;
    const double kDemandedEasMps = kEasMps + 15.0;
    const Run baseDynamic = Fly(cfg, kBaseAltM + 100.0, 0.0, kDemandedEasMps, kEasMps, 600, false, 0.0,
                                kEasRampMpsPerS);
    auto DynamicDiffers = [&](double FGuidanceConfigV2::*field, double value, const char *name) {
        FGuidanceConfigV2 c = cfg; c.*field = value;
        Check(IsGuidanceConfigValid(c), "variant_config_valid");
        Check(!Fly(c, kBaseAltM + 100.0, 0.0, kDemandedEasMps, kEasMps, 600, false, 0.0, kEasRampMpsPerS)
                   .SameSignature(baseDynamic), name);
    };
    DynamicDiffers(&FGuidanceConfigV2::TecsAirspeedErrorTimeConstantS, 12.0, "airspeed_error_tc_reaches_tecs");
    DynamicDiffers(&FGuidanceConfigV2::TecsSteRateTimeConstantS, 1.5, "ste_rate_time_constant_reaches_tecs");
    DynamicDiffers(&FGuidanceConfigV2::TecsAirspeedMeasurementStdDevMps, 5.0, "airspeed_measurement_std_dev_reaches_tecs");
    DynamicDiffers(&FGuidanceConfigV2::TecsAirspeedRateMeasurementStdDevMps2, 5.0, "airspeed_rate_std_dev_reaches_tecs");
    DynamicDiffers(&FGuidanceConfigV2::TecsAirspeedFilterProcessStdDevMps2, 5.0, "airspeed_process_std_dev_reaches_tecs");

    // altitude_rate_ff scales the reference model's altitude-rate feed-forward, so it only shows up
    // while the reference trajectory is still moving -- a climb demand does exactly that.
    ClimbDiffers(&FGuidanceConfigV2::TecsAltitudeRateFeedForward, 1.0, "altitude_rate_ff_reaches_tecs");

    // The throttle slew limit is off by PX4 default (0). Turning it on must bound the throttle step.
    {
        FGuidanceConfigV2 c = cfg; c.TecsThrottleSlewRatePerS = 0.05;
        Check(IsGuidanceConfigValid(c), "throttle_slewrate_variant_valid");
        const Run slewed = Fly(c, kBaseAltM + 100.0, 0.0, kEasMps);
        Check(!slewed.SameSignature(baseClimb), "throttle_slewrate_reaches_tecs");
        Check(slewed.MaxAbsThrottleStep <= 0.05 * kDt + 1e-6, "throttle_slewrate_bounds_the_throttle_step");
        Check(slewed.MaxAbsThrottleStep < baseClimb.MaxAbsThrottleStep, "throttle_slewrate_reduces_the_throttle_step");
    }

    // equivalent_airspeed_trim only enters through the underspeed bounds while an airspeed sensor is
    // enabled (TECSControl::_detectUnderspeed), so it is observed there, on UnderspeedRatio.
    {
        FGuidanceConfigV2 c = cfg; c.TecsEquivalentAirspeedTrimMps = 40.0;
        Check(IsGuidanceConfigValid(c), "eas_trim_variant_valid");
        const Run r = Fly(c, kBaseAltM, 0.0, kEasMps, kUnderspeedEasMps);
        Check(baseUnderspeed.LastUnderspeed > 0.0, "underspeed_condition_actually_engaged");
        Check(r.LastUnderspeed != baseUnderspeed.LastUnderspeed,
              "eas_trim_reaches_tecs_and_is_not_a_hidden_default");
    }

    // Roll-to-throttle compensation multiplies (load_factor - 1). Load factor is per-frame runtime
    // state that this coordinator does not feed, so the term is provably inert today. The parameter
    // is still configured so the caller contract matches upstream; this check records the gap
    // instead of hiding it, and will start failing the moment a load-factor path is added.
    {
        FGuidanceConfigV2 c = cfg; c.TecsRollToThrottleCompensation = 0.0;
        Check(IsGuidanceConfigValid(c), "roll_to_throttle_variant_valid");
        Check(Fly(c, kBaseAltM + 100.0, 0.0, kEasMps).SameSignature(baseClimb),
              "roll_to_throttle_is_inert_until_a_per_frame_load_factor_is_fed");
    }

    // ---- directional effect of the energy limits (not just "something changed") ----
    {
        FGuidanceConfigV2 lo = cfg; lo.TecsMaxClimbRateMps = 3.0;
        FGuidanceConfigV2 hi = cfg; hi.TecsMaxClimbRateMps = 12.0;
        // A +100 m demand saturates the altitude-rate clamp, so the total-energy-rate setpoint
        // tracks STE_rate_max = max_climb_rate * g.
        Check(Fly(hi, kBaseAltM + 100.0, 0.0, kEasMps).LastSteRateSp >
                  Fly(lo, kBaseAltM + 100.0, 0.0, kEasMps).LastSteRateSp,
              "larger_max_climb_rate_raises_the_total_energy_rate_setpoint");
    }
    {
        FGuidanceConfigV2 lo = cfg; lo.TecsMinSinkRateMps = 1.0;
        FGuidanceConfigV2 hi = cfg; hi.TecsMinSinkRateMps = 4.0;
        // On a descent demand the setpoint is floored at STE_rate_min = -min_sink_rate * g.
        Check(Fly(hi, kBaseAltM - 100.0, 0.0, kEasMps).LastSteRateSp <
                  Fly(lo, kBaseAltM - 100.0, 0.0, kEasMps).LastSteRateSp,
              "larger_min_sink_rate_lowers_the_total_energy_rate_setpoint");
    }

    // ---- the same config is re-applied deterministically after a reset ----
    {
        const Run a = Fly(cfg, kBaseAltM + 100.0, 0.0, kEasMps);
        const Run b = Fly(cfg, kBaseAltM + 100.0, 0.0, kEasMps);
        Check(a.SameSignature(b), "same_config_is_reapplied_deterministically_after_reset");
    }

    // ---- every new parameter rejects out-of-range values as InvalidConfig ----
    struct BadField { const char *Name; double FGuidanceConfigV2::*Field; double Value; };
    const double kNan = std::numeric_limits<double>::quiet_NaN();
    const BadField badFields[] = {
        {"max_climb_zero", &FGuidanceConfigV2::TecsMaxClimbRateMps, 0.0},
        {"max_climb_negative", &FGuidanceConfigV2::TecsMaxClimbRateMps, -1.0},
        {"max_climb_nan", &FGuidanceConfigV2::TecsMaxClimbRateMps, kNan},
        {"min_sink_zero", &FGuidanceConfigV2::TecsMinSinkRateMps, 0.0},
        {"min_sink_above_max_sink", &FGuidanceConfigV2::TecsMinSinkRateMps, 9.0},
        {"max_sink_below_px4_min", &FGuidanceConfigV2::TecsMaxSinkRateMps, 0.5},
        {"max_sink_above_px4_max", &FGuidanceConfigV2::TecsMaxSinkRateMps, 20.0},
        {"eas_trim_below_eas_min", &FGuidanceConfigV2::TecsEquivalentAirspeedTrimMps, 5.0},
        {"eas_trim_above_eas_max", &FGuidanceConfigV2::TecsEquivalentAirspeedTrimMps, 95.0},
        {"eas_trim_nan", &FGuidanceConfigV2::TecsEquivalentAirspeedTrimMps, kNan},
        {"alt_error_tc_below_px4_min", &FGuidanceConfigV2::TecsAltitudeErrorTimeConstantS, 1.0},
        {"alt_error_tc_zero", &FGuidanceConfigV2::TecsAltitudeErrorTimeConstantS, 0.0},
        {"tas_error_tc_below_px4_min", &FGuidanceConfigV2::TecsAirspeedErrorTimeConstantS, 1.0},
        {"pitch_integrator_above_px4_max", &FGuidanceConfigV2::TecsPitchIntegratorGain, 3.0},
        {"pitch_integrator_negative", &FGuidanceConfigV2::TecsPitchIntegratorGain, -0.1},
        {"pitch_damping_above_px4_max", &FGuidanceConfigV2::TecsPitchDamping, 3.0},
        {"throttle_integrator_above_px4_max", &FGuidanceConfigV2::TecsThrottleIntegratorGain, 2.0},
        {"throttle_damping_above_px4_max", &FGuidanceConfigV2::TecsThrottleDamping, 2.0},
        {"throttle_slewrate_above_px4_max", &FGuidanceConfigV2::TecsThrottleSlewRatePerS, 2.0},
        {"throttle_slewrate_negative", &FGuidanceConfigV2::TecsThrottleSlewRatePerS, -0.1},
        {"ste_rate_tc_above_px4_max", &FGuidanceConfigV2::TecsSteRateTimeConstantS, 3.0},
        {"seb_rate_ff_below_px4_min", &FGuidanceConfigV2::TecsSebRateFeedForwardGain, 0.1},
        {"seb_rate_ff_above_px4_max", &FGuidanceConfigV2::TecsSebRateFeedForwardGain, 4.0},
        {"altitude_rate_ff_above_px4_max", &FGuidanceConfigV2::TecsAltitudeRateFeedForward, 2.0},
        {"speed_weight_above_px4_max", &FGuidanceConfigV2::TecsSpeedWeight, 3.0},
        {"roll_to_throttle_above_px4_max", &FGuidanceConfigV2::TecsRollToThrottleCompensation, 25.0},
        {"airspeed_std_dev_below_px4_min", &FGuidanceConfigV2::TecsAirspeedMeasurementStdDevMps, 0.001},
        {"airspeed_rate_std_dev_above_px4_max", &FGuidanceConfigV2::TecsAirspeedRateMeasurementStdDevMps2, 20.0},
        {"airspeed_process_std_dev_nan", &FGuidanceConfigV2::TecsAirspeedFilterProcessStdDevMps2, kNan},
        {"target_climb_rate_zero", &FGuidanceConfigV2::TargetClimbRateMps, 0.0},
        {"target_sink_rate_negative", &FGuidanceConfigV2::TargetSinkRateMps, -1.0},
        {"eas_min_above_eas_max", &FGuidanceConfigV2::EasMinMps, 100.0},
    };
    for (const BadField &b : badFields) {
        FGuidanceConfigV2 c{}; c.*b.Field = b.Value;
        Check(!IsGuidanceConfigValid(c), "config_predicate_rejects_out_of_range_parameter");
        FormationGuidanceCoordinatorV2 g;
        const auto o = g.Update(MakeInput(10.0, 1u, kBaseAltM, 0.0, kBaseAltM + 100.0, kEasMps, kEasMps, 0.0), c);
        Check(!o.bCommandReady && o.FailureReason == EGuidanceFailureV2::InvalidConfig,
              "coordinator_rejects_out_of_range_parameter_as_InvalidConfig");
        Check(o.PitchReferenceRad == 0.0 && o.ThrottleReferenceNorm == 0.0,
              "rejected_frame_emits_no_command");
    }

    std::printf("TECS_CALLER_CONTRACT_V2 checks=%d failures=%d vertAccelLimit=%.1f (PX4 FW_T_VERT_ACC min=%.1f max=%.1f)\n",
                Checks, Failures, cfg.TecsVerticalAccelLimitMps2,
                kTecsVerticalAccelLimitMinMps2, kTecsVerticalAccelLimitMaxMps2);
    std::printf("TECS_CALLER_CONTRACT_V2 configured_setters=19 aircraft_performance_fields=4 "
                "generic_tuning_fields=15 runtime_state_excluded=load_factor,alt_step,air_density_refresh\n");
    return Failures ? 1 : 0;
}
