// verify_npfg_caller_contract_v2.cpp — the NPFG *caller* contract, not the NPFG core.
//
// The 16-row NPFG equivalence audit proves the ported lateral law matches PX4. It does NOT prove
// that FormationGuidanceCoordinatorV2 configures that law the way PX4's own callers do: the
// equivalence harness sets the NPFG parameters itself. This audit closes exactly that gap.
//
// Pinned PX4 v1.17.0 (d6f12ad1c4f70ad3230afd7d86e971421e02fef4) splits the lateral guidance across
// two modules, and BOTH configure it:
//
//   src/modules/fw_mode_manager/FixedWingModeManager.cpp:100-106   -> DirectionalGuidance
//       setPeriod(NPFG_PERIOD)                    setDamping(NPFG_DAMPING)
//       enablePeriodLB(NPFG_LB_PERIOD)            enablePeriodUB(NPFG_UB_PERIOD)
//       setRollTimeConst(NPFG_ROLL_TC)            setSwitchDistanceMultiplier(NPFG_SW_DST_MLT)
//       setPeriodSafetyFactor(NPFG_PERIOD_SF)
//
//   src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp:119
//       _airspeed_direction_control.setPGainFromPeriodAndDamping(NPFG_DAMPING, NPFG_PERIOD)
//
// FPx4NpfgAdapter fuses DirectionalGuidance + AirspeedDirectionController + CourseToAirspeedRefMapper,
// so the contract it must satisfy is the union: eight static setters. The coordinator previously
// called NONE of them; every value came from a vendored class initializer.
//
// The one place the seam cannot be bit-preserving: AirspeedDirectionController's constructor
// initializes p_gain_ to the literal 0.8885f, which is a ROUNDED form of its own public formula
//     4 * pi * damping / period  =  4 * pi * 0.7071 / 10  =  0.888568044f
// Wiring the public setter therefore normalizes that rounding (relative change 7.7e-5). That is a
// consequence of applying the pinned formula, not a tuning change, and it is asserted here.
//
// CourseToAirspeedRefMapper has no static setter in the pinned caller, so it gets no config.
#include "FormationControl/Px4NpfgAdapter.h"
#include "FormationControlV2/FormationGuidanceCoordinatorV2.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace FormationControlV2;

namespace {

int Failures{}, Checks{};
void Check(bool v, const char *name)
{
    ++Checks;
    if (!v) { ++Failures; std::cerr << "FAIL " << name << '\n'; }
}

constexpr double kDt = 1.0 / 60.0;
constexpr double kBaseAltM = 3048.0;
constexpr double kEasMps = 189.070713;   // the committed F-16 host-fixture trim point
constexpr double kTasMps = 220.0;
constexpr std::uint32_t kOriginGen = 7;

// A test-local config that lets the coordinator be built at the F-16 trim point. It exists only so
// the NPFG contract can be exercised at a realistic airspeed; it recommends nothing.
FGuidanceConfigV2 BaseConfig()
{
    FGuidanceConfigV2 c{};
    c.EasMinMps = 170.0;
    c.EasMaxMps = 220.0;
    c.TecsEquivalentAirspeedTrimMps = kEasMps;
    c.TecsMaxClimbRateMps = 100.539354597920735;
    c.TecsMinSinkRateMps = 53.569103350590687;
    c.TecsMaxSinkRateMps = 83.108839138474863;
    c.ThrottleTrim = 0.296895856953;
    return c;
}

// Curved path with a cross-track offset and a course error: this is the regime in which NPFG's
// period adaptation (the lower/upper bounds, the roll time constant and the safety factor) is
// actually engaged, so a static parameter that is wired can move the output.
FGuidanceCoordinatorInputV2 MakeInput(double timeS, std::uint32_t resetGen, double crossTrackM,
                                      double courseErrorRad, double curvature, double windEastMps = 0.0)
{
    FGuidanceCoordinatorInputV2 in{};
    auto &f = in.Follower;
    // Path runs North through the origin; +East is "right of path".
    const double aircraftCourse = -courseErrorRad;   // courseError = pathCourse(0) - aircraftCourse
    f.PositionNE_m = {0.0, crossTrackM};
    f.GroundVelocityNE_mps = {kTasMps * std::cos(aircraftCourse), kTasMps * std::sin(aircraftCourse)};
    f.GroundCourse_rad = aircraftCourse;
    f.AltitudeAsl_m = kBaseAltM;
    f.ClimbRate_mps = 0.0;
    f.SimulationTimeS = timeS;
    f.ResetGeneration = resetGen;
    f.OriginGeneration = kOriginGen;
    f.EquivalentAirspeed_mps = kEasMps;
    f.TrueAirspeed_mps = kTasMps;
    f.WindNE_mps = {0.0, windEastMps};
    f.EasToTasRatio = kTasMps / kEasMps;
    f.bPositionValid = f.bGroundVelocityValid = f.bGroundCourseValid = true;
    f.bCourseRateValid = f.bCurvatureValid = f.bAltitudeValid = f.bClimbRateValid = true;
    f.bSimulationTimeValid = f.bEasValid = f.bTasValid = f.bWindValid = f.bOriginValid = true;
    f.bRatioValid = true;

    in.Slot.ResetGeneration = resetGen;
    in.Slot.OriginGeneration = kOriginGen;
    in.Slot.bValid = true;

    in.PlannerDto.Npfg.PathPositionNE_m = {0.0, 0.0};
    in.PlannerDto.Npfg.PathUnitTangentNE = {1.0, 0.0};
    in.PlannerDto.Npfg.PathCurvature_per_m = curvature;
    in.PlannerDto.Npfg.bValid = true;

    in.PlannerDto.Tecs.TargetEasMps = kEasMps;
    in.PlannerDto.Tecs.TargetAltitudeAslM = kBaseAltM;
    in.PlannerDto.Tecs.bTargetEasValid = true;
    in.PlannerDto.Tecs.bTargetAltitudeValid = true;
    in.PlannerDto.Tecs.bTargetClimbRateValid = false;
    in.PlannerDto.Tecs.bCommandReady = true;

    in.CurrentPitchRad = 0.0;
    in.bCurrentPitchValid = true;
    in.CurrentRollRad = 0.0;
    in.bCurrentRollValid = true;
    in.SimulationTimeS = timeS;
    in.DtS = kDt;
    in.ResetGeneration = resetGen;
    in.OriginGeneration = kOriginGen;
    return in;
}

// The NPFG-facing part of the coordinator's public output. A static NPFG parameter that never
// reaches the controller cannot move any of these, so a difference here IS the wiring proof. No
// production getter or debug hook is added.
struct NpfgSig {
    double RollRef{}, Ff{}, Fb{}, Total{}, CourseSp{}, WindFeas{};
    bool bValid{};
    bool Same(const NpfgSig &o) const
    {
        return RollRef == o.RollRef && Ff == o.Ff && Fb == o.Fb && Total == o.Total &&
               CourseSp == o.CourseSp && WindFeas == o.WindFeas;
    }
};

// Frozen kinematics: the state does not move, so only the configuration can change the output.
NpfgSig Fly(const FGuidanceConfigV2 &cfg, double crossTrackM = 200.0,
            double courseErrorRad = 0.15, double curvature = 1.0e-4, int frames = 6,
            double windEastMps = 0.0)
{
    FormationGuidanceCoordinatorV2 g;
    FGuidanceCoordinatorOutputV2 o{};
    for (int k = 0; k < frames; ++k)
        o = g.Update(MakeInput(10.0 + k * kDt, 1u, crossTrackM, courseErrorRad, curvature, windEastMps), cfg);
    NpfgSig s{};
    s.bValid = o.bCommandReady;
    s.RollRef = o.RollReferenceRad;
    s.Ff = o.LateralAccelerationFeedforwardMps2;
    s.Fb = o.LateralAccelerationFeedbackMps2;
    s.Total = o.LateralAccelerationTotalMps2;
    s.CourseSp = o.CourseSetpointRad;
    s.WindFeas = o.WindFeasibility;
    return s;
}

} // namespace

int main()
{
    const FGuidanceConfigV2 cfg = BaseConfig();
    Check(IsGuidanceConfigValid(cfg), "test_local_base_config_is_valid");

    // ---- 1. defaults reproduce the CURRENT hidden behaviour (vendored class initializers) --------
    // These are deliberately NOT the PX4 parameter defaults: NPFG_DAMPING defaults to 0.7 and
    // NPFG_ROLL_TC to 0.5 upstream, while this build has always run on the vendored initializers
    // 0.7071 and 0.0. This commit changes wiring, not tuning, so the initializers are preserved.
    Check(cfg.NpfgPeriodS == 10.0, "default_period_matches_hidden_DirectionalGuidance_initializer");
    Check(cfg.NpfgDamping == 0.7071, "default_damping_matches_hidden_DirectionalGuidance_initializer");
    Check(cfg.bNpfgEnablePeriodLowerBound, "default_period_lower_bound_matches_hidden_initializer");
    Check(cfg.bNpfgEnablePeriodUpperBound, "default_period_upper_bound_matches_hidden_initializer");
    Check(cfg.NpfgRollTimeConstantS == 0.0, "default_roll_time_constant_matches_hidden_initializer");
    Check(cfg.NpfgSwitchDistanceMultiplier == 0.32, "default_switch_distance_multiplier_matches_hidden_initializer");
    Check(cfg.NpfgPeriodSafetyFactor == 1.5, "default_period_safety_factor_matches_hidden_initializer");

    // ---- 2. the heading-controller P gain comes from the pinned formula, not the literal ---------
    // AirspeedDirectionController exposes no gain getter, but controlHeading() is linear in p_gain_,
    // so the ratio of two headings' outputs is the ratio of the gains. That is enough to prove both
    // that the setter is wired and that the constructor literal is no longer in effect.
    {
        constexpr float kHeadingSp = 0.30f, kHeading = 0.10f, kAirspeed = 220.0f;
        MumtPx4::FPx4NpfgAdapter fresh;   // constructor literal p_gain_ = 0.8885f
        const float fbLiteral = fresh.airspeedDirectionController().controlHeading(kHeadingSp, kHeading, kAirspeed);

        MumtPx4::FPx4NpfgAdapter wired;
        wired.airspeedDirectionController().setPGainFromPeriodAndDamping(
            static_cast<float>(cfg.NpfgDamping), static_cast<float>(cfg.NpfgPeriodS));
        const float fbWired = wired.airspeedDirectionController().controlHeading(kHeadingSp, kHeading, kAirspeed);

        const float pgainLiteral = 0.8885f;
        const float pgainFormula = 4.f * static_cast<float>(M_PI) * static_cast<float>(cfg.NpfgDamping) /
                                   static_cast<float>(cfg.NpfgPeriodS);
        Check(std::abs(fbLiteral) > 1e-6f, "control_heading_produces_a_non_zero_feedback");
        Check(pgainFormula != pgainLiteral, "pinned_formula_does_not_reproduce_the_constructor_literal");
        // linear in the gain -> the output ratio must equal the gain ratio
        const double observedRatio = static_cast<double>(fbWired) / static_cast<double>(fbLiteral);
        const double expectedRatio = static_cast<double>(pgainFormula) / static_cast<double>(pgainLiteral);
        Check(std::abs(observedRatio - expectedRatio) < 1e-6,
              "heading_controller_p_gain_equals_4pi_damping_over_period");
        Check(std::abs(observedRatio - 1.0) > 1e-8,
              "heading_controller_p_gain_is_no_longer_the_constructor_literal");

        // A different (damping, period) must move it again, and in the direction the formula says.
        MumtPx4::FPx4NpfgAdapter softer;
        softer.airspeedDirectionController().setPGainFromPeriodAndDamping(
            static_cast<float>(cfg.NpfgDamping), static_cast<float>(cfg.NpfgPeriodS * 2.0));
        const float fbSofter = softer.airspeedDirectionController().controlHeading(kHeadingSp, kHeading, kAirspeed);
        Check(std::abs(fbSofter) < std::abs(fbWired), "doubling_the_period_halves_the_heading_p_gain");
    }

    // ---- 3. every static setter is driven from config: changing it must move a public output -----
    const NpfgSig base = Fly(cfg);
    Check(base.bValid, "base_npfg_frame_is_command_ready");

    auto Differs = [&](FGuidanceConfigV2 c, const char *name) {
        Check(IsGuidanceConfigValid(c), "variant_config_valid");
        const NpfgSig s = Fly(c);
        Check(s.bValid, "variant_npfg_frame_is_command_ready");
        Check(!s.Same(base), name);
        return s;
    };

    FGuidanceConfigV2 cPeriod = cfg;  cPeriod.NpfgPeriodS = 25.0;
    const NpfgSig sPeriod = Differs(cPeriod, "period_reaches_npfg");
    FGuidanceConfigV2 cDamp = cfg;    cDamp.NpfgDamping = 0.35;
    const NpfgSig sDamp = Differs(cDamp, "damping_reaches_npfg");

    // The roll time constant is the GATE on NPFG's whole period-adaptation block:
    //     DirectionalGuidance::adaptPeriod  ->  if (en_period_lb_ && roll_time_const_ > NPFG_EPSILON)
    // The vendored initializer is 0.0 (PX4's NPFG_ROLL_TC parameter defaults to 0.5), so with the
    // committed defaults the stability-bound machinery -- the lower bound, the upper bound and the
    // safety factor -- is entirely inert, and the adapted period is always the nominal period.
    // Turning it on must therefore move the output, and the other three become observable only then.
    FGuidanceConfigV2 cRollTc = cfg;  cRollTc.NpfgRollTimeConstantS = 3.0;
    const NpfgSig sRollTc = Differs(cRollTc, "roll_time_constant_reaches_npfg");
    Check(cfg.NpfgRollTimeConstantS == 0.0,
          "committed_default_roll_time_constant_disables_npfg_period_adaptation");

    // With adaptation ON, the remaining three parameters must each move the output.
    const NpfgSig adaptBase = Fly(cRollTc);
    auto DiffersFromAdapt = [&](FGuidanceConfigV2 c, const char *name) {
        Check(IsGuidanceConfigValid(c), "adaptation_variant_config_valid");
        const NpfgSig s = Fly(c);
        Check(s.bValid, "adaptation_variant_is_command_ready");
        Check(!s.Same(adaptBase), name);
        return s;
    };
    FGuidanceConfigV2 cSafety = cRollTc; cSafety.NpfgPeriodSafetyFactor = 4.0;
    const NpfgSig sSafety = DiffersFromAdapt(cSafety, "period_safety_factor_reaches_npfg");
    FGuidanceConfigV2 cLb = cRollTc;     cLb.bNpfgEnablePeriodLowerBound = false;
    const NpfgSig sLb = DiffersFromAdapt(cLb, "period_lower_bound_flag_reaches_npfg");
    // The upper bound is doubly constrained by the pinned algorithm:
    //   1. periodUpperBound() returns INFINITY unless air_turn_rate * wind_factor > NPFG_EPSILON, and
    //      windFactor(airspeed, 0) == 0, so with ZERO WIND the bound is infinite and can never bind.
    //   2. when it does bind, adaptPeriod takes max(period_lb, period_ub), so a large lower bound
    //      swallows it. It can only act while period_lb < period_ub < nominal period.
    // It is therefore exercised in the only regime the algorithm allows: a SMALL roll time constant
    // (so the lower bound stays under the nominal period), wind, and a tight arc.
    constexpr double kUbRollTcS = 0.5;          // small enough that period_lb stays below the period
    constexpr double kWindEastMps = 100.0;      // windFactor(220, 100) != 0
    constexpr double kTightCurvature = 2.0e-2;  // 50 m radius: a large air turn rate
    FGuidanceConfigV2 cUbOn = cfg;   cUbOn.NpfgRollTimeConstantS = kUbRollTcS;
    FGuidanceConfigV2 cUb = cUbOn;   cUb.bNpfgEnablePeriodUpperBound = false;
    Check(IsGuidanceConfigValid(cUbOn) && IsGuidanceConfigValid(cUb), "upper_bound_variant_config_valid");
    const NpfgSig windyUbOn = Fly(cUbOn, 200.0, 0.15, kTightCurvature, 6, kWindEastMps);
    const NpfgSig sUb = Fly(cUb, 200.0, 0.15, kTightCurvature, 6, kWindEastMps);
    Check(windyUbOn.bValid && sUb.bValid, "windy_upper_bound_frames_are_command_ready");
    Check(!sUb.Same(windyUbOn), "period_upper_bound_flag_reaches_npfg");
    // ... and it is provably inert with zero wind, because the bound is infinite there.
    Check(Fly(cUb, 200.0, 0.15, kTightCurvature).Same(Fly(cUbOn, 200.0, 0.15, kTightCurvature)),
          "period_upper_bound_flag_is_inert_without_wind_because_the_bound_is_infinite");

    // With adaptation OFF (the committed default), those same three are provably inert. Recorded
    // rather than hidden: these checks start failing the moment the roll time constant is raised.
    {
        FGuidanceConfigV2 c = cfg; c.NpfgPeriodSafetyFactor = 4.0;
        Check(Fly(c).Same(base), "period_safety_factor_is_inert_while_roll_time_constant_is_zero");
        c = cfg; c.bNpfgEnablePeriodLowerBound = false;
        Check(Fly(c).Same(base), "period_lower_bound_flag_is_inert_while_roll_time_constant_is_zero");
        c = cfg; c.bNpfgEnablePeriodUpperBound = false;
        Check(Fly(c).Same(base), "period_upper_bound_flag_is_inert_while_roll_time_constant_is_zero");
    }

    // ---- 4. no aliasing: distinct parameters must not collapse onto one another ------------------
    Check(!sPeriod.Same(sDamp), "period_and_damping_are_not_aliased");
    Check(!sPeriod.Same(sRollTc), "period_and_roll_time_constant_are_not_aliased");
    Check(!sDamp.Same(sRollTc), "damping_and_roll_time_constant_are_not_aliased");
    Check(!sSafety.Same(sLb), "period_safety_factor_and_lower_bound_flag_are_not_aliased");
    // The two period-bound flags are distinct fields gating distinct code paths. The upper bound is
    // NESTED inside the lower bound in the pinned algorithm, so they are separated in the regime
    // where each one actually acts, not by asserting they always differ:
    //   - lower-bound regime (large roll time constant, no wind): disabling LB moves the output,
    //     disabling UB does not (periodUpperBound is infinite without wind).
    //   - upper-bound regime (small roll time constant, wind, tight arc): disabling UB moves it.
    {
        FGuidanceConfigV2 ubOffAtLbRegime = cRollTc;
        ubOffAtLbRegime.bNpfgEnablePeriodUpperBound = false;
        Check(!sLb.Same(adaptBase), "lower_bound_flag_acts_in_the_lower_bound_regime");
        Check(Fly(ubOffAtLbRegime).Same(adaptBase),
              "upper_bound_flag_does_not_act_in_the_lower_bound_regime");
        Check(!sUb.Same(windyUbOn), "upper_bound_flag_acts_in_the_upper_bound_regime");
        Check(!sLb.Same(sUb), "period_lower_and_upper_bound_flags_are_not_aliased");
    }

    // ---- 5. the switch-distance multiplier is wired but PROVABLY INERT on this update path -------
    // DirectionalGuidance::switchDistance() is the only consumer, and FPx4NpfgAdapter::update() never
    // calls it (the coordinator does not do waypoint switching). The setter is still configured so the
    // caller contract matches upstream; this records the gap instead of hiding it, and the check will
    // start failing the moment a switch-distance consumer is added to the update path.
    {
        FGuidanceConfigV2 c = cfg;
        c.NpfgSwitchDistanceMultiplier = 0.9;
        Check(IsGuidanceConfigValid(c), "switch_distance_multiplier_variant_valid");
        Check(Fly(c).Same(base),
              "switch_distance_multiplier_is_inert_on_the_coordinator_update_path");
        // ... but the setter itself is functional, proven on a test-owned adapter.
        MumtPx4::FPx4NpfgAdapter a, b;
        a.directionalGuidance().setSwitchDistanceMultiplier(static_cast<float>(cfg.NpfgSwitchDistanceMultiplier));
        b.directionalGuidance().setSwitchDistanceMultiplier(static_cast<float>(c.NpfgSwitchDistanceMultiplier));
        Check(a.directionalGuidance().switchDistance(500.0f) != b.directionalGuidance().switchDistance(500.0f),
              "switch_distance_multiplier_setter_is_functional");
    }

    // ---- 6. runtime navigation state is separate from static tuning ------------------------------
    {
        // Same config, different runtime geometry -> the output must move.
        Check(!Fly(cfg, 400.0, 0.15, 1.0e-4).Same(base), "runtime_cross_track_moves_the_npfg_output");
        Check(!Fly(cfg, 200.0, -0.15, 1.0e-4).Same(base), "runtime_course_error_moves_the_npfg_output");
        Check(!Fly(cfg, 200.0, 0.15, -1.0e-4).Same(base), "runtime_path_curvature_moves_the_npfg_output");
        // Same config, same geometry -> deterministic.
        Check(Fly(cfg).Same(base), "same_config_and_geometry_is_deterministic");
    }

    // ---- 7. RollLimitRad is a policy clamp, NOT an NPFG tuning parameter -------------------------
    {
        FGuidanceConfigV2 wide = cfg;   wide.RollLimitRad = 1.2;
        FGuidanceConfigV2 tight = cfg;  tight.RollLimitRad = 0.05;
        const NpfgSig w = Fly(wide);
        const NpfgSig t = Fly(tight);
        Check(w.Ff == t.Ff && w.Fb == t.Fb && w.Total == t.Total && w.CourseSp == t.CourseSp,
              "roll_limit_does_not_change_any_npfg_lateral_acceleration_or_course_output");
        Check(std::abs(t.RollRef) <= tight.RollLimitRad + 1e-12,
              "roll_limit_clamps_only_the_roll_reference");
        Check(std::abs(w.RollRef) > std::abs(t.RollRef), "a_wider_roll_limit_admits_a_larger_roll_reference");
    }

    // ---- 8. the config is re-applied after a reset / controller recreation -----------------------
    {
        FormationGuidanceCoordinatorV2 g;
        FGuidanceCoordinatorOutputV2 o{};
        for (int k = 0; k < 6; ++k)
            o = g.Update(MakeInput(10.0 + k * kDt, 1u, 200.0, 0.15, 1.0e-4), cPeriod);
        const double afterFirstGeneration = o.LateralAccelerationTotalMps2;
        Check(o.bCommandReady, "first_generation_is_command_ready");

        // New reset generation -> RecreateControllers -> all eight setters must run again.
        const auto resetFrame = g.Update(MakeInput(20.0, 2u, 200.0, 0.15, 1.0e-4), cPeriod);
        Check(!resetFrame.bCommandReady && resetFrame.FailureReason == EGuidanceFailureV2::ResetFrame,
              "reset_generation_change_yields_reset_frame");
        for (int k = 1; k < 7; ++k)
            o = g.Update(MakeInput(20.0 + k * kDt, 2u, 200.0, 0.15, 1.0e-4), cPeriod);
        Check(o.bCommandReady, "second_generation_is_command_ready");
        Check(o.LateralAccelerationTotalMps2 == afterFirstGeneration,
              "the_same_npfg_config_is_reapplied_after_a_reset");

        // The boolean flags survive a reset too.
        FormationGuidanceCoordinatorV2 h;
        FGuidanceCoordinatorOutputV2 p{};
        for (int k = 0; k < 6; ++k)
            p = h.Update(MakeInput(10.0 + k * kDt, 1u, 200.0, 0.15, 1.0e-4), cLb);
        h.Reset(5u);
        FGuidanceCoordinatorOutputV2 q{};
        for (int k = 0; k < 7; ++k)
            q = h.Update(MakeInput(30.0 + k * kDt, 5u, 200.0, 0.15, 1.0e-4), cLb);
        Check(q.bCommandReady && q.LateralAccelerationTotalMps2 == p.LateralAccelerationTotalMps2,
              "boolean_npfg_flags_are_reapplied_after_an_explicit_reset");
    }

    // ---- 9. validation: the algorithm's own hard clamps are rejected, not silently substituted ---
    // DirectionalGuidance::setPeriod/setDamping/setSwitchDistanceMultiplier/setPeriodSafetyFactor all
    // clamp silently. Silent substitution is exactly what this contract exists to prevent, so an
    // out-of-domain value is a config error. The PX4 parameter METADATA ranges (period 1..100,
    // roll_tc 0..2) are a small-fixed-wing UI range, not a physical law, and are NOT used as bounds.
    const double kNan = std::numeric_limits<double>::quiet_NaN();
    const double kInf = std::numeric_limits<double>::infinity();
    struct Bad { const char *Name; double FGuidanceConfigV2::*Field; double Value; };
    const Bad bad[] = {
        {"period_zero", &FGuidanceConfigV2::NpfgPeriodS, 0.0},
        {"period_negative", &FGuidanceConfigV2::NpfgPeriodS, -1.0},
        {"period_nan", &FGuidanceConfigV2::NpfgPeriodS, kNan},
        {"period_inf", &FGuidanceConfigV2::NpfgPeriodS, kInf},
        {"damping_zero", &FGuidanceConfigV2::NpfgDamping, 0.0},
        {"damping_negative", &FGuidanceConfigV2::NpfgDamping, -0.5},
        {"damping_above_one", &FGuidanceConfigV2::NpfgDamping, 1.5},
        {"damping_nan", &FGuidanceConfigV2::NpfgDamping, kNan},
        {"roll_tc_negative", &FGuidanceConfigV2::NpfgRollTimeConstantS, -0.1},
        {"roll_tc_nan", &FGuidanceConfigV2::NpfgRollTimeConstantS, kNan},
        {"switch_distance_multiplier_below_algorithmic_min", &FGuidanceConfigV2::NpfgSwitchDistanceMultiplier, 0.05},
        {"switch_distance_multiplier_nan", &FGuidanceConfigV2::NpfgSwitchDistanceMultiplier, kNan},
        {"period_safety_factor_below_algorithmic_min", &FGuidanceConfigV2::NpfgPeriodSafetyFactor, 0.5},
        {"period_safety_factor_inf", &FGuidanceConfigV2::NpfgPeriodSafetyFactor, kInf},
    };
    for (const Bad &b : bad) {
        FGuidanceConfigV2 c = cfg;
        c.*b.Field = b.Value;
        Check(!IsGuidanceConfigValid(c), "config_predicate_rejects_out_of_domain_npfg_parameter");
        FormationGuidanceCoordinatorV2 g;
        const auto o = g.Update(MakeInput(10.0, 1u, 200.0, 0.15, 1.0e-4), c);
        Check(!o.bCommandReady && o.FailureReason == EGuidanceFailureV2::InvalidConfig,
              "coordinator_rejects_out_of_domain_npfg_parameter_as_InvalidConfig");
        Check(o.RollReferenceRad == 0.0 && o.LateralAccelerationTotalMps2 == 0.0,
              "rejected_npfg_frame_emits_no_command");
    }
    // Values outside the PX4 METADATA range but inside the algorithm's domain stay valid: an
    // aircraft outside PX4's parameter design range must be configurable, not silently rejected.
    {
        FGuidanceConfigV2 c = cfg;
        c.NpfgPeriodS = 0.5;   // below NPFG_PERIOD's metadata @min of 1.0, still a legal period
        Check(IsGuidanceConfigValid(c), "px4_metadata_range_is_not_used_as_a_hard_npfg_bound");
        c.NpfgRollTimeConstantS = 5.0;   // above NPFG_ROLL_TC's metadata @max of 2.0
        Check(IsGuidanceConfigValid(c), "px4_metadata_range_is_not_used_as_a_hard_roll_tc_bound");
    }

    std::printf("NPFG_CALLER_CONTRACT_V2 checks=%d failures=%d\n", Checks, Failures);
    std::printf("NPFG_CALLER_CONTRACT_V2 configured_static_setters=8 "
                "(DirectionalGuidance: setPeriod,setDamping,enablePeriodLB,enablePeriodUB,"
                "setRollTimeConst,setSwitchDistanceMultiplier,setPeriodSafetyFactor; "
                "AirspeedDirectionController: setPGainFromPeriodAndDamping) "
                "CourseToAirspeedRefMapper=no_static_setter_in_the_pinned_caller "
                "runtime_navigation_state=position,ground_velocity,wind,path_geometry,airspeed\n");
    std::printf("NPFG_CALLER_CONTRACT_V2 defaults period=%.4f damping=%.4f period_lb=%d period_ub=%d "
                "roll_tc=%.4f switch_distance_multiplier=%.4f period_safety_factor=%.4f "
                "(vendored initializers preserved; PX4 param defaults NPFG_DAMPING=0.7 and "
                "NPFG_ROLL_TC=0.5 are deliberately NOT adopted in this wiring commit)\n",
                cfg.NpfgPeriodS, cfg.NpfgDamping, cfg.bNpfgEnablePeriodLowerBound ? 1 : 0,
                cfg.bNpfgEnablePeriodUpperBound ? 1 : 0, cfg.NpfgRollTimeConstantS,
                cfg.NpfgSwitchDistanceMultiplier, cfg.NpfgPeriodSafetyFactor);
    return Failures ? 1 : 0;
}
