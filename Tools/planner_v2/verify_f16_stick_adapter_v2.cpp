// verify_f16_stick_adapter_v2.cpp — independent offline audit of F16StickAdapterV2.
// Pure host build (g++), no Unreal/JSBSim. Exit 0 iff every check passes.
#include "FormationControlV2/F16StickAdapterV2.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>

using namespace FormationControlV2;
namespace {
int Failures{}, Checks{};
void Check(bool value, const char *name) { ++Checks; if (!value) { ++Failures; std::cerr << "FAIL " << name << '\n'; } }

FF16StickInputV2 Base(double time = 10.0, double dt = 1.0 / 60.0) {
    FF16StickInputV2 in{};
    in.bGuidanceValid = in.bAttitudeValid = in.bBodyRatesValid = in.bAlphaBetaValid = in.bAirspeedValid = true;
    in.EasMps = 120.0; in.TasMps = 150.0; in.ThrottleReferenceNorm = 0.5;
    in.SimulationTimeS = time; in.DtS = dt; in.ResetGeneration = 1;
    return in;
}
// Step the adapter N frames with a constant input (time advancing), return last output.
FF16StickCommandV2 Run(F16StickAdapterV2 &a, FF16StickInputV2 in, int frames, const FF16StickConfigV2 &c = {}) {
    FF16StickCommandV2 out{};
    for (int i = 0; i < frames; ++i) { out = a.Update(in, c); in.SimulationTimeS += in.DtS; }
    return out;
}
bool FiniteCmd(const FF16StickCommandV2 &o) {
    return std::isfinite(o.AileronCmdNorm) && std::isfinite(o.ElevatorCmdNorm) &&
           std::isfinite(o.RudderCmdNorm) && std::isfinite(o.ThrottleCmdNorm);
}
bool InRange(const FF16StickCommandV2 &o) {
    return o.AileronCmdNorm >= -1 && o.AileronCmdNorm <= 1 && o.ElevatorCmdNorm >= -1 && o.ElevatorCmdNorm <= 1 &&
           o.RudderCmdNorm >= -1 && o.RudderCmdNorm <= 1 && o.ThrottleCmdNorm >= 0 && o.ThrottleCmdNorm <= 1;
}
} // namespace

int main() {
    const FF16StickConfigV2 config{};
    const double NaN = std::numeric_limits<double>::quiet_NaN();

    // 1. first frame is a reset frame (invalid, neutral)
    { F16StickAdapterV2 a; auto o = a.Update(Base(), config);
      Check(!o.bValid && o.FailureReason == EF16StickFailureV2::ResetFrame &&
            o.AileronCmdNorm == 0 && o.ElevatorCmdNorm == 0 && o.RudderCmdNorm == 0 && o.ThrottleCmdNorm == 0,
            "first_frame_reset_neutral"); }
    // 2. zero error -> neutral surfaces (throttle tracks reference)
    { F16StickAdapterV2 a; auto o = Run(a, Base(), 120);
      Check(o.bValid && std::abs(o.AileronCmdNorm) < 1e-9 && std::abs(o.ElevatorCmdNorm) < 1e-9 &&
            o.RudderCmdNorm == 0 && std::abs(o.ThrottleCmdNorm - 0.5) < 1e-9, "zero_error_neutral"); }
    // 3./4. positive/negative roll error -> +/- aileron, symmetric
    { F16StickAdapterV2 a, b; auto ip = Base(); ip.RollReferenceRad = 0.3; auto im = Base(); im.RollReferenceRad = -0.3;
      auto op = Run(a, ip, 60), om = Run(b, im, 60);
      Check(op.bValid && op.AileronCmdNorm > 0.05, "roll_error_positive_aileron");
      Check(om.bValid && om.AileronCmdNorm < -0.05, "roll_error_negative_aileron");
      Check(std::abs(op.AileronCmdNorm + om.AileronCmdNorm) < 1e-9, "left_right_symmetry"); }
    // 5./6. positive pitch error (nose-up demand) -> NEGATIVE elevator; symmetric
    { F16StickAdapterV2 a, b; auto ip = Base(); ip.PitchReferenceRad = 0.2; auto im = Base(); im.PitchReferenceRad = -0.2;
      auto op = Run(a, ip, 60), om = Run(b, im, 60);
      Check(op.bValid && op.ElevatorCmdNorm < -0.05, "pitch_up_negative_elevator");
      Check(om.bValid && om.ElevatorCmdNorm > 0.05, "pitch_down_positive_elevator");
      Check(std::abs(op.ElevatorCmdNorm + om.ElevatorCmdNorm) < 1e-9, "pitch_symmetry"); }
    // 7. angle wrap: reference +pi-0.1 vs current -pi+0.1 -> small error (-0.2), not near 2*pi
    { F16StickAdapterV2 a; auto in = Base(); in.RollReferenceRad = Pi - 0.1; in.CurrentRollRad = -Pi + 0.1;
      auto o = Run(a, in, 60);
      Check(o.bValid && o.AileronCmdNorm < 0 && std::abs(o.AileronCmdNorm) < 0.5, "angle_wrap_short_way"); }
    // 8. roll rate damping (enabled) reduces command vs disabled
    { FF16StickConfigV2 damp{}; damp.RollRateDampingGain = 0.5;
      F16StickAdapterV2 a, b; auto in = Base(); in.RollReferenceRad = 0.3; in.BodyRollRateRadps = 1.0;
      auto od = Run(a, in, 60, damp), on = Run(b, in, 60, config);
      Check(od.bValid && on.bValid && od.AileronCmdNorm < on.AileronCmdNorm, "roll_rate_damping_reduces"); }
    // 9. pitch rate damping (enabled) moves elevator positive (less nose-up) under +q
    { FF16StickConfigV2 damp{}; damp.PitchRateDampingGain = 0.5;
      F16StickAdapterV2 a, b; auto in = Base(); in.PitchReferenceRad = 0.2; in.BodyPitchRateRadps = 0.5;
      auto od = Run(a, in, 60, damp), on = Run(b, in, 60, config);
      Check(od.bValid && on.bValid && od.ElevatorCmdNorm > on.ElevatorCmdNorm, "pitch_rate_damping_reduces"); }
    // 10. saturation: huge roll error -> aileron clamped to exactly 1 (after slew settles)
    { F16StickAdapterV2 a; auto in = Base(); in.RollReferenceRad = 3.0;
      auto o = Run(a, in, 120);
      Check(o.bValid && o.AileronCmdNorm <= 1.0 && o.AileronCmdNorm > 0.99, "aileron_saturation"); }
    // 11. slew limit: per-frame command step <= slew*dt on every channel
    { F16StickAdapterV2 a; auto in = Base(); in.RollReferenceRad = 3.0; in.PitchReferenceRad = -1.0; in.ThrottleReferenceNorm = 1.0;
      FF16StickCommandV2 prev{}; bool ok = true; double t = 10.0;
      for (int i = 0; i < 90; ++i) {
          in.SimulationTimeS = t; auto o = a.Update(in, config); t += in.DtS;
          if (o.bValid) {
              ok = ok && std::abs(o.AileronCmdNorm - prev.AileronCmdNorm) <= config.AileronSlewPerS * in.DtS + 1e-12 &&
                   std::abs(o.ElevatorCmdNorm - prev.ElevatorCmdNorm) <= config.ElevatorSlewPerS * in.DtS + 1e-12 &&
                   std::abs(o.ThrottleCmdNorm - prev.ThrottleCmdNorm) <= config.ThrottleSlewPerS * in.DtS + 1e-12;
              prev = o;
          }
      }
      Check(ok, "slew_limit_all_channels"); }
    // 12. PI memory: final stateful actuator sign need not equal the instantaneous error's P sign.
    // Drive a long negative error, switch to a small positive error for one frame, then sustain that
    // opposite error and observe recovery. Only real adapter outputs are inspected; no controller
    // equation or assumed recovery time is copied into this test.
    { F16StickAdapterV2 a; auto memory = Base(); memory.PitchReferenceRad = -0.2;
      auto memoryOut = Run(a, memory, 1000, config);
      auto opposite = Base(10.0 + 1000.0 * memory.DtS); opposite.PitchReferenceRad = 0.03;
      auto firstOpposite = a.Update(opposite, config); opposite.SimulationTimeS += opposite.DtS;
      Check(memoryOut.bValid && memoryOut.ElevatorCmdNorm > 0.0, "pi_memory_negative_error_positive_elevator");
      Check(firstOpposite.bValid && firstOpposite.ElevatorCmdNorm > 0.0,
            "pi_memory_can_override_instantaneous_positive_error_sign");

      FF16StickCommandV2 prev = firstOpposite, out = firstOpposite;
      int reversalFrame = -1, rangeViolations = 0, slewViolations = 0, nonFinite = 0;
      double maxAbsCommand = std::abs(firstOpposite.ElevatorCmdNorm);
      constexpr int RecoveryObservationFrames = 7200; // finite observation window, not an expected time
      for (int frame = 0; frame < RecoveryObservationFrames; ++frame) {
          out = a.Update(opposite, config); opposite.SimulationTimeS += opposite.DtS;
          if (!out.bValid || !FiniteCmd(out)) ++nonFinite;
          if (out.bValid && !InRange(out)) ++rangeViolations;
          if (out.bValid && prev.bValid &&
              std::abs(out.ElevatorCmdNorm - prev.ElevatorCmdNorm) > config.ElevatorSlewPerS * opposite.DtS + 1e-12)
              ++slewViolations;
          if (out.bValid) {
              maxAbsCommand = std::max(maxAbsCommand, std::abs(out.ElevatorCmdNorm));
              if (reversalFrame < 0 && out.ElevatorCmdNorm < 0.0) reversalFrame = frame;
          }
          prev = out;
          if (reversalFrame >= 0) break;
      }
      const double reversalTimeS = reversalFrame >= 0 ? (reversalFrame + 1) * opposite.DtS : -1.0;
      std::cout << "PI_MEMORY first_opposite_elevator=" << firstOpposite.ElevatorCmdNorm
                << " reversal_time_s=" << reversalTimeS << " max_abs_command=" << maxAbsCommand
                << " finite_violations=" << nonFinite << " range_violations=" << rangeViolations
                << " slew_violations=" << slewViolations << '\n';
      Check(reversalFrame >= 0, "sustained_opposite_error_eventually_reverses_command");
      Check(nonFinite == 0 && rangeViolations == 0 && slewViolations == 0,
            "pi_memory_recovery_finite_range_slew"); }
    // 13. throttle clamp: reference above max / below min clamps into configured range
    { FF16StickConfigV2 c{}; c.ThrottleMin = 0.1; c.ThrottleMax = 0.9;
      F16StickAdapterV2 a, b; auto ih = Base(); ih.ThrottleReferenceNorm = 1.5; auto il = Base(); il.ThrottleReferenceNorm = -0.5;
      auto oh = Run(a, ih, 120, c), ol = Run(b, il, 120, c);
      Check(oh.bValid && std::abs(oh.ThrottleCmdNorm - 0.9) < 1e-9, "throttle_clamp_max");
      Check(ol.bValid && std::abs(ol.ThrottleCmdNorm - 0.1) < 1e-9, "throttle_clamp_min"); }
    // 14. rudder neutral policy by default; damping term engages only when configured
    { F16StickAdapterV2 a; auto in = Base(); in.BodyYawRateRadps = 0.5; in.BetaRad = 0.1;
      auto o = Run(a, in, 60, config); Check(o.bValid && o.RudderCmdNorm == 0.0, "rudder_neutral_default");
      FF16StickConfigV2 c{}; c.YawRateDampingGain = 0.2; F16StickAdapterV2 b;
      auto od = Run(b, in, 60, c); Check(od.bValid && od.RudderCmdNorm < 0.0, "rudder_damping_optional"); }
    // 15. pause: invalid neutral output and NO state advance while paused; resume skips one frame
    { F16StickAdapterV2 a; auto in = Base(); in.RollReferenceRad = 0.3; auto before = Run(a, in, 60);
      auto paused = in; paused.bPaused = true; paused.SimulationTimeS += 1.0;
      auto p1 = a.Update(paused, config), p2 = a.Update(paused, config);
      Check(!p1.bValid && p1.FailureReason == EF16StickFailureV2::Paused && p1.AileronCmdNorm == 0 &&
            !p2.bValid && p2.AileronCmdNorm == 0, "pause_neutral_invalid");
      auto resumed = in; resumed.SimulationTimeS = paused.SimulationTimeS + in.DtS;
      auto r1 = a.Update(resumed, config);
      Check(!r1.bValid && r1.FailureReason == EF16StickFailureV2::ResetFrame, "resume_skips_one_frame");
      (void)before; }
    // 16. reset generation change clears state and skips one frame
    { F16StickAdapterV2 a; auto in = Base(); in.RollReferenceRad = 0.5; Run(a, in, 60);
      auto r = in; r.ResetGeneration = 2; auto o = a.Update(r, config);
      Check(!o.bValid && o.FailureReason == EF16StickFailureV2::ResetFrame && o.AileronCmdNorm == 0, "reset_generation_clear");
      r.SimulationTimeS += r.DtS; auto o2 = a.Update(r, config);
      Check(o2.bValid && std::abs(o2.AileronCmdNorm) <= config.AileronSlewPerS * r.DtS + 1e-12, "post_reset_slew_from_neutral"); }
    // 17. abnormal dt: too small and too large both rejected, state cleared
    { F16StickAdapterV2 a; auto in = Base(); Run(a, in, 30);
      auto d1 = in; d1.DtS = 1e-6; auto o1 = a.Update(d1, config);
      Check(!o1.bValid && o1.FailureReason == EF16StickFailureV2::AbnormalDt, "dt_too_small_rejected");
      F16StickAdapterV2 b; Run(b, in, 30); auto d2 = in; d2.DtS = 5.0; auto o2 = b.Update(d2, config);
      Check(!o2.bValid && o2.FailureReason == EF16StickFailureV2::AbnormalDt, "dt_too_large_rejected"); }
    // 18. invalid inputs -> fresh neutral invalid (no stale command survives)
    { F16StickAdapterV2 a; auto in = Base(); in.RollReferenceRad = 0.5; in.ThrottleReferenceNorm = 1.0; Run(a, in, 120);
      auto bad = in; bad.bGuidanceValid = false; auto o = a.Update(bad, config);
      Check(!o.bValid && o.FailureReason == EF16StickFailureV2::InvalidGuidance &&
            o.AileronCmdNorm == 0 && o.ElevatorCmdNorm == 0 && o.RudderCmdNorm == 0 && o.ThrottleCmdNorm == 0,
            "invalid_guidance_fresh_neutral");
      auto bad2 = in; bad2.bAttitudeValid = false; auto o2 = a.Update(bad2, config);
      Check(!o2.bValid && o2.FailureReason == EF16StickFailureV2::InvalidAttitude && o2.ThrottleCmdNorm == 0, "invalid_attitude_fresh_neutral"); }
    // 19. NaN/Inf inputs rejected
    { F16StickAdapterV2 a; auto in = Base(); Run(a, in, 30);
      auto bad = in; bad.RollReferenceRad = NaN; auto o = a.Update(bad, config);
      Check(!o.bValid && o.FailureReason == EF16StickFailureV2::NonFiniteInput, "nan_reference_rejected");
      auto bad2 = in; bad2.CurrentPitchRad = std::numeric_limits<double>::infinity(); auto o2 = a.Update(bad2, config);
      Check(!o2.bValid && o2.FailureReason == EF16StickFailureV2::NonFiniteInput, "inf_attitude_rejected"); }
    // 20. stale output clearing: after invalid, next valid frame re-slews from neutral (not old value)
    { F16StickAdapterV2 a; auto in = Base(); in.RollReferenceRad = 3.0; Run(a, in, 120); // aileron ~1
      auto bad = in; bad.bGuidanceValid = false; a.Update(bad, config);
      auto good = in; good.SimulationTimeS += 2.0 * in.DtS; auto o = a.Update(good, config);
      Check(o.bValid && std::abs(o.AileronCmdNorm) <= config.AileronSlewPerS * in.DtS + 1e-12, "stale_cleared_reslew_from_neutral"); }
    // 21. body rates required only when damping gains enabled
    { F16StickAdapterV2 a; auto in = Base(); in.bBodyRatesValid = false; auto o = Run(a, in, 30, config);
      Check(o.bValid, "body_rates_not_required_by_default");
      FF16StickConfigV2 c{}; c.YawRateDampingGain = 0.2; F16StickAdapterV2 b;
      b.Update(in, c); auto o2 = b.Update(in, c);
      Check(!o2.bValid && o2.FailureReason == EF16StickFailureV2::InvalidBodyRates, "body_rates_required_when_damping"); }

    // 22. fixed-seed random sweep (3000): range, finiteness, slew, symmetry-of-sign under mirror
    constexpr std::uint64_t Seed = 0x4631365354494B5FULL; // "F16STIK_"
    std::mt19937_64 rng(Seed);
    std::uniform_real_distribution<double> ang(-Pi, Pi), rate(-2.0, 2.0), thr(-0.2, 1.2), dts(0.005, 0.05);
    constexpr int Cases = 3000;
    int randomFailures = 0;
    double aMin = 1e9, aMax = -1e9, eMin = 1e9, eMax = -1e9, rMin = 1e9, rMax = -1e9, tMin = 1e9, tMax = -1e9;
    for (int i = 0; i < Cases; ++i) {
        FF16StickInputV2 in = Base(10.0, dts(rng));
        in.RollReferenceRad = ang(rng); in.PitchReferenceRad = ang(rng) * 0.25;
        in.CurrentRollRad = ang(rng);   in.CurrentPitchRad = ang(rng) * 0.25;
        in.BodyRollRateRadps = rate(rng); in.BodyPitchRateRadps = rate(rng); in.BodyYawRateRadps = rate(rng);
        in.ThrottleReferenceNorm = thr(rng);
        F16StickAdapterV2 a; FF16StickCommandV2 o{}, prev{}; bool ok = true; int validFrames = 0;
        for (int f = 0; f < 40; ++f) {
            o = a.Update(in, config); in.SimulationTimeS += in.DtS;
            if (o.bValid) {
                ok = ok && FiniteCmd(o) && InRange(o) &&
                     std::abs(o.AileronCmdNorm - prev.AileronCmdNorm) <= config.AileronSlewPerS * in.DtS + 1e-12;
                prev = o; ++validFrames;
            }
        }
        ok = ok && validFrames > 0;
        // mirrored roll input must produce mirrored aileron (fresh adapter, same frame count)
        FF16StickInputV2 mi = in; mi.SimulationTimeS = 10.0;
        mi.RollReferenceRad = -in.RollReferenceRad + 2.0 * in.CurrentRollRad; // mirror error about current
        F16StickAdapterV2 m; FF16StickCommandV2 mo{};
        { FF16StickInputV2 run = mi; for (int f = 0; f < 40; ++f) { mo = m.Update(run, config); run.SimulationTimeS += run.DtS; } }
        // (only check when both valid; mirrored error -> opposite aileron)
        if (o.bValid && mo.bValid) ok = ok && std::abs(o.AileronCmdNorm + mo.AileronCmdNorm) < 1e-6;
        randomFailures += !ok;
        aMin = std::min(aMin, o.AileronCmdNorm); aMax = std::max(aMax, o.AileronCmdNorm);
        eMin = std::min(eMin, o.ElevatorCmdNorm); eMax = std::max(eMax, o.ElevatorCmdNorm);
        rMin = std::min(rMin, o.RudderCmdNorm); rMax = std::max(rMax, o.RudderCmdNorm);
        tMin = std::min(tMin, o.ThrottleCmdNorm); tMax = std::max(tMax, o.ThrottleCmdNorm);
    }
    Failures += randomFailures;
    std::cout << "F16_STICK_V2 checks=" << Checks << " seed=" << Seed << " random_cases=" << Cases
              << " random_failures=" << randomFailures << " failures=" << Failures
              << " aileron_range=[" << aMin << ',' << aMax << "] elevator_range=[" << eMin << ',' << eMax
              << "] rudder_range=[" << rMin << ',' << rMax << "] throttle_range=[" << tMin << ',' << tMax << "]\n";
    return Failures ? 1 : 0;
}
