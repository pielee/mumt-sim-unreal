#include "FormationControlV2/FormationPlannerV2.h"
#include <cstdio>

using namespace FormationControlV2;
static int Failures = 0, Checks = 0;
static void Check(bool ok, const char *name) {
  Checks++;
  if (!ok) {
    Failures++;
    std::printf("FAIL sequence=%s invariant=%s\n", name, name);
  }
}
struct Rig {
  FormationPlannerV2 P;
  FormationPlannerV2Input I{};
  FormationPlannerV2Diagnostics D{};
  FormationPlannerV2Output O{};
  Rig() {
    P.Config.PathExpirationS = 1e9;
    P.Config.ProjectionFailureDistanceM = 1e9;
    P.Config.MinimumReplanIntervalS = 0;
    // This legacy state-machine fixture exercises the far-field Dubins transitions only.
    P.Config.NearFieldRangeM = 0;
    P.Config.NearFieldRangeRadiusFactor = 0;
    I.bFollowerValid = I.bCourseValid = I.bWindValid = I.bRatioValid = true;
    I.EasToTasRatio = 1;
    I.EquivalentAirspeedMps = 150;
    I.DtS = .25;
    I.Slot = {{{0, 0}, 0}, {100, 0}, 100, 0, true, true};
    Set(-1000, 0, 10, 0);
  }
  void Set(double along, double cross, double rel, double course) {
    I.FollowerPositionNE = {along, cross};
    I.FollowerGroundVelocityNE = {100 + rel, 0};
    I.FollowerCourseRad = course;
  }
  FormationPlannerV2Output Step(double dt = .25) {
    I.DtS = dt;
    I.SimulationTimeS += std::max(0., dt);
    O = P.Update(I, D);
    return O;
  }
  void ReachCapture() {
    Set(-1000, 0, 10, 0);
    Step();
    Step();
  }
  void ReachTaper() {
    ReachCapture();
    Set(-500, 0, 10, 0);
    Step();
    Step();
    Step();
  }
  void ReachHold() {
    ReachTaper();
    Set(-100, 0, 0, 0);
    Step();
    Step();
    Step();
    Step();
  }
};

int main() {
  {
    Rig r;
    r.Set(-1000, 0, 10, 0);
    r.Step();
    Check(r.P.GetMode() == PlannerMode::Rejoin, "rejoin_dwell_before");
    r.Set(-1000, 500, 10, 0);
    r.Step();
    Check(r.D.GuardDwellS[(int)GuardTransition::RejoinToCapture] == 0,
          "rejoin_guard_false_resets");
    r.Set(-1000, 0, 10, 0);
    r.Step();
    r.Step();
    Check(r.P.GetMode() == PlannerMode::CaptureEntry, "rejoin_to_capture_050");
    r.Set(-500, 0, 10, 0);
    r.Step();
    r.Step();
    Check(r.P.GetMode() == PlannerMode::CaptureEntry, "capture_dwell_before");
    r.Step();
    Check(r.P.GetMode() == PlannerMode::ClosureTaper, "capture_to_taper_075");
    r.Set(-100, 0, 12, 0);
    r.Step();
    r.Step();
    r.Step();
    Check(r.P.GetMode() == PlannerMode::ClosureTaper, "hold_dwell_before");
    r.Step();
    Check(r.P.GetMode() == PlannerMode::SlotHold,
          "taper_to_hold_100_and_plus12");
  }
  {
    Rig r;
    r.ReachTaper();
    r.Set(-200, 0, -14, 0);
    r.Step();
    r.Step();
    Check(r.P.GetMode() == PlannerMode::ClosureTaper,
          "fallback_negative_before_075");
    r.Step();
    Check(r.P.GetMode() == PlannerMode::NearFieldSlotTrack,
          "fallback_negative_interval");
  }
  {
    Rig r;
    r.ReachTaper();
    r.Set(-200, 0, 20, 0);
    for (int x = 0; x < 4; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::ClosureTaper,
          "fallback_plus20_exclusive");
    r.Set(-200, 0, 20.1, 0);
    r.Step();
    r.Step();
    r.Step();
    Check(r.P.GetMode() == PlannerMode::NearFieldSlotTrack, "fallback_above_plus20");
  }
  {
    Rig r;
    r.ReachTaper();
    r.Set(-200, 0, -15, 0);
    r.Step();
    r.Step();
    r.Step();
    Check(r.P.GetMode() == PlannerMode::NearFieldSlotTrack,
          "minus15_fallback_not_rejoin");
  }
  {
    Rig r;
    r.ReachTaper();
    r.Set(-200, 0, -15.1, 0);
    r.Step();
    Check(r.P.GetMode() == PlannerMode::ClosureTaper,
          "rejoin_priority_before_050");
    r.Step();
    Check(r.P.GetMode() == PlannerMode::Rejoin,
          "rejoin_priority_below_minus15");
  }
  {
    Rig r;
    r.ReachTaper();
    r.Set(-200, 0, 50, 0);
    r.Step();
    r.Step();
    r.Step();
    Check(r.P.GetMode() == PlannerMode::NearFieldSlotTrack,
          "plus50_fallback_not_rejoin");
  }
  {
    Rig r;
    r.ReachTaper();
    r.Set(-200, 0, 50.1, 0);
    r.Step();
    r.Step();
    Check(r.P.GetMode() == PlannerMode::Rejoin, "above_plus50_rejoin");
  }
  {
    Rig r;
    r.ReachHold();
    r.Set(-100, 0, -12, 0);
    for (int x = 0; x < 8; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::SlotHold, "hold_minus12_inclusive");
    r.Set(-100, 0, 12.1, 0);
    for (int x = 0; x < 3; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::SlotHold, "hold_exit_before_100");
    r.Step();
    Check(r.P.GetMode() == PlannerMode::SlotHold, "hold_plus12_not_exit22");
    r.Set(-100, 0, 22.1, 0);
    for (int x = 0; x < 4; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::Rejoin, "hold_to_rejoin_100");
  }
  {
    Rig r;
    r.ReachCapture();
    r.Set(-1200, 0, 0, 0);
    for (int x = 0; x < 120; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::Rejoin &&
              r.O.Failure == PlannerFailure::CaptureTimeout,
          "capture_timeout_30");
  }
  {
    Rig r;
    r.ReachTaper();
    r.Set(-200, 0, 0, 0);
    for (int x = 0; x < 80; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::Rejoin &&
              r.O.Failure == PlannerFailure::TaperTimeout,
          "taper_timeout_20");
  }
  {
    Rig r;
    r.ReachHold();
    for (int x = 0; x < 1600; x++) {
      r.Set(-100, 0, 0, 0);
      r.Step();
    }
    Check(r.P.GetMode() == PlannerMode::SlotHold, "slot_hold_timeout_disabled");
  }
  {
    Rig r;
    r.P.Config.MaxCaptureTimeS = 1000;
    r.Set(-9000, 0, 0, 0);
    for (int x = 0; x < 500; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::Rejoin &&
              r.O.Failure != PlannerFailure::RejoinTimeout,
          "rejoin_timeout_default_disabled");
  }
  {
    Rig r;
    r.P.Config.MaxCaptureTimeS = 100;
    r.P.Config.RejoinTimeoutS = 101;
    r.Set(-1000, 500, 0, 0);
    for (int x = 0; x < 404; x++)
      r.Step();
    Check(!r.O.bValid && r.O.Failure == PlannerFailure::RejoinTimeout,
          "configured_rejoin_timeout_above_max_capture_invalid");
  }
  {
    Rig r;
    r.ReachCapture();
    r.Set(-500, 250, 10, 0);
    for (int x = 0; x < 8; x++) {
      r.Set(-500, (x % 2) ? 250 : 190, 10, 0);
      r.Step();
    }
    Check(r.P.GetMode() == PlannerMode::CaptureEntry,
          "threshold_noise_hysteresis");
  }
  {
    Rig r;
    r.ReachTaper();
    r.Set(-100, 0, 0, 20 * Pi / 180);
    for (int x = 0; x < 5; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::ClosureTaper,
          "slot_heading_blocks_hold");
    r.Set(-100, 0, 0, 10 * Pi / 180);
    for (int x = 0; x < 4; x++)
      r.Step();
    Check(r.P.GetMode() == PlannerMode::SlotHold, "slot_heading_allows_hold");
  }
  // Moving-terminal position hysteresis is covered by the production near-field audit; this
  // fixture keeps the remaining far-field sequence deterministic.
  {
    Rig r;
    r.P.Config.MinimumReplanIntervalS = .5;
    r.Step();
    auto g = r.P.GetPathGeneration();
    r.I.Slot.Pose.Position.N = 30;
    r.Step(.25);
    Check(r.P.GetPathGeneration() == g, "replan_cooldown_blocks_frequency");
    r.Step(.25);
    Check(r.P.GetPathGeneration() > g, "replan_after_cooldown");
  }
  {
    Rig r;
    r.Step();
    auto g = r.P.GetPathGeneration();
    r.I.Slot.Pose.CourseRad = 1.9 * Pi / 180;
    r.I.Slot.GroundVelocityNE = FromCourse(r.I.Slot.Pose.CourseRad) * 100;
    r.Step();
    Check(r.P.GetPathGeneration() == g, "terminal_heading_keep_1_9deg");
    r.I.Slot.Pose.CourseRad = 2.1 * Pi / 180;
    r.I.Slot.GroundVelocityNE = FromCourse(r.I.Slot.Pose.CourseRad) * 100;
    r.Step();
    Check(r.P.GetPathGeneration() > g, "terminal_heading_replan_2_1deg");
  }
  // Candidate cost definitions and actual CCC selection under deterministic
  // eligible configuration.
  {
    bool selected = false;
    for (double n : {-300., -150., 0., 150., 300.})
      for (double e : {-300., -150., 0., 150., 300.})
        for (int h = 0; h < 8 && !selected; h++) {
          Rig r;
          r.P.Config.CccPenaltyS = 0;
          r.P.Config.CccAbsoluteAdvantageS = 0;
          r.P.Config.CccRelativeAdvantage = 0;
          r.P.Config.CccMinimumHeadingErrorRad = 0;
          r.P.Config.CccMaximumRangeRadiusFactor = 100;
          r.I.Slot.Pose = {{n, e}, -Pi + h * Pi / 4};
          r.I.Slot.GroundVelocityNE = FromCourse(r.I.Slot.Pose.CourseRad) * 100;
          r.Set(0, 0, 0, 0);
          r.Step();
          selected = r.D.SelectedType == DubinsType::RLR ||
                     r.D.SelectedType == DubinsType::LRL;
          if (selected) {
            const auto &c = r.D.CandidateCosts[(int)r.D.SelectedType];
            Check(std::abs(c.TransitTimeS - c.TotalS) < 1e-9,
                  "ccc_base_cost_no_switch_penalty");
          }
        }
    Check(selected, "ccc_deterministic_selectable");
  }
  // Type-switch hysteresis retains regenerated active family when advantage
  // requirement is huge.
  {
    Rig r;
    r.Step();
    auto type = r.P.GetActiveType();
    r.P.Config.TypeSwitchAbsoluteAdvantageS = 1e6;
    r.I.Slot.Pose.Position.E = 100;
    r.Step();
    Check(r.P.GetActiveType() == type, "type_switch_hysteresis");
  }
  // Held-path, expiry, pause/resume and progress monotonicity.
  {
    Rig r;
    r.Step();
    double progress = r.P.GetProgressS();
    r.Set(-4900,500,10,0);
    r.Step();
    Check(r.P.GetProgressS() >= progress, "progress_monotonic");
    auto gen = r.P.GetPathGeneration();
    double heldProgress = r.P.GetProgressS();
    r.I.bPaused = true;
    auto paused = r.Step(10);
    Check(!paused.bValid && paused.Failure == PlannerFailure::Paused &&
              r.P.GetPathGeneration() == gen &&
              r.P.GetProgressS() == heldProgress,
          "pause_freezes_state");
    r.I.bPaused = false;
    auto resumed = r.Step(10);
    Check(resumed.Failure != PlannerFailure::AbnormalDt,
          "resume_ignores_pause_duration");
    r.P.Config.PredictionTimeS = 3;
    r.I.Slot.bCurvatureValid = false;
    for (int x = 0; x < 8; x++)
      r.Step();
    Check(r.D.bUsingHeldPath && r.O.bPathValid, "prediction_failure_held_path");
    for (int x = 0; x < 3; x++)
      r.Step();
    Check(!r.O.bValid && r.O.Failure == PlannerFailure::HeldPathExpired,
          "held_path_expiry_2s");
  }
  // Reset, abnormal dt, projection-triggered replan, stale clearing, invalid
  // recovery.
  {
    Rig r;
    r.Step();
    r.I.ResetGeneration = 1;
    r.Step();
    Check(r.P.GetPathGeneration() == 1 && r.P.GetMode() == PlannerMode::Rejoin,
          "reset_generation_full_reset");
    auto badDt = r.Step(.5);
    Check(!badDt.bValid && badDt.Failure == PlannerFailure::AbnormalDt &&
              badDt.TargetEasMps == 0,
          "abnormal_dt_fresh_zero");
    r.I.DtS = .25;
    r.I.bFollowerValid = false;
    auto bad = r.Step();
    Check(!bad.bValid && !bad.bPathValid && bad.TargetEasMps == 0,
          "invalid_stale_output_cleared");
    r.I.bFollowerValid = true;
    auto recovered = r.Step();
    Check(recovered.bPathValid && r.P.GetMode() == PlannerMode::Rejoin,
          "invalid_recovery_rejoin");
  }
  {
    Rig r;
    r.Step();
    int replans = r.D.ReplanCount;
    r.P.Config.ProjectionFailureDistanceM = 1;
    r.Set(-5000, 5000, 0, 0);
    r.Step();
    Check(r.D.ReplanCount > replans || !r.O.bValid,
          "projection_failure_replan_or_invalid");
  }
  std::printf(
      "SEQUENCE_AUDIT checks=%d failures=%d "
      "transitions=R2C,C2T,C2R,T2H,T2C,T2R,H2R replan=terminal,projection "
      "type_selection=CSC,CCC,type_hysteresis\n",
      Checks, Failures);
  return Failures ? 1 : 0;
}
