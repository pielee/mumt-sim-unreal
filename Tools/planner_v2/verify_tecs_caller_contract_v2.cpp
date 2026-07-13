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
};

// One 10 s stateful run of the PRODUCTION coordinator.
Run Fly(const FGuidanceConfigV2 &cfg, double targetAltitudeM, double climbRateMps, double targetEasMps,
        double easMps = kEasMps, int frames = 600,
        bool targetClimbRateValid = false, double targetClimbRateMps = 0.0) {
    FormationGuidanceCoordinatorV2 g;
    Run r{};
    double prevPitch = 0.0; bool havePrev = false;
    for (int k = 0; k < frames; ++k) {
        const double t = 10.0 + k * kDt;
        const auto in = MakeInput(t, 1u, kBaseAltM, climbRateMps, targetAltitudeM, targetEasMps,
                                  easMps, 0.0, targetClimbRateValid, targetClimbRateMps);
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
            const double bound = kDt * cfg.TecsVerticalAccelLimitMps2 / (easMps * kEasToTas);
            if (step > bound + 1e-6) ++r.SlewViolations;
        }
        prevPitch = o.PitchReferenceRad; havePrev = true;
        r.SettledPitch = o.PitchReferenceRad;
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

    std::printf("TECS_CALLER_CONTRACT_V2 checks=%d failures=%d vertAccelLimit=%.1f (PX4 FW_T_VERT_ACC min=%.1f max=%.1f)\n",
                Checks, Failures, cfg.TecsVerticalAccelLimitMps2,
                kTecsVerticalAccelLimitMinMps2, kTecsVerticalAccelLimitMaxMps2);
    return Failures ? 1 : 0;
}
