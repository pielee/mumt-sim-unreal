#include "FormationControlV2/FormationPlannerV2.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
using namespace FormationControlV2;
namespace {
int Failures{}, Checks{};
void Check(bool x, const char *n) {
  ++Checks;
  if (!x) {
    ++Failures;
    std::cerr << "FAIL " << n << '\n';
  }
}
FormationPlannerV2Input Input(double along, double cross, double speed,
                              double slotSpeed, double course, double time,
                              double dt) {
  FormationPlannerV2Input i{};
  const Vec2 t = FromCourse(course), r = RightNormal(course);
  i.Slot.Pose = {{0, 0}, course};
  i.Slot.GroundVelocityNE = t * slotSpeed;
  i.Slot.GroundSpeedMps = slotSpeed;
  i.Slot.CurvaturePerM = 0;
  i.Slot.bCurvatureValid = true;
  i.Slot.bValid = true;
  i.FollowerPositionNE = t * along + r * cross;
  i.FollowerCourseRad = course;
  i.FollowerGroundVelocityNE = t * speed;
  i.WindVelocityNE = {};
  i.EasToTasRatio = 1;
  i.EquivalentAirspeedMps = speed;
  i.SimulationTimeS = time;
  i.DtS = dt;
  i.ResetGeneration = 1;
  i.bFollowerValid = i.bCourseValid = i.bWindValid = i.bRatioValid = true;
  return i;
}
} // namespace
int main() {
  FormationPlannerV2Config cfg{};
  const double ranges[] = {25, 50, 100, 200, 500, 1000, 2000},
               speeds[] = {50, 100, 170, 220, 245},
               headings[] = {0, 10, -10, 45, -45, 90, -90, 180};
  struct G {
    double a, c;
  };
  const G geos[] = {{-1, 0}, {-1, -1}, {-1, 1}, {1, 0},
                    {1, -1}, {1, 1},   {0, 0},  {0, 1}};
  int grid = 0, candidateFailed = 0, storm = 0, stale = 0, nonfinite = 0,
      curv = 0, slotFailures = 0, exactPoseNear = 0, maxEval = 0, modes[5] = {}, candidateMode[5] = {};
  for (double range : ranges)
    for (double speed : speeds)
      for (double hd : headings)
        for (auto g : geos) {
          double a = g.a == 0 ? (g.c == 0 ? 0 : .1 * range * g.c)
                              : (.7071 * range * g.a),
                 c = g.c == 0 ? (g.a == 0 ? range : 0) : .7071 * range * g.c;
          FormationPlannerV2 p;
          p.Config = cfg;
          double time = 10;
          int eval = 0;
          for (int f = 0; f < 120; f++) {
            auto in = Input(a, c, speed, speed, hd * Pi / 180, time, 1. / 60);
            FormationPlannerV2Diagnostics d{};
            auto o = p.Update(in, d);
            eval = d.CandidateEvaluationCount;
            if (o.Failure == PlannerFailure::CandidateSelectionFailed) {
              candidateFailed++;
              candidateMode[(int)o.Mode]++;
            }
            if (!o.bValid && (o.Path.bValid || o.TargetEasMps != 0))
              stale++;
            if (o.bValid) {
              modes[(int)o.Mode]++;
              if (!o.Path.Position.IsFinite() ||
                  !o.Path.UnitTangent.IsFinite() || !Finite(o.TargetEasMps))
                nonfinite++;
              if (std::abs(o.Path.SignedCurvaturePerM) > 1 / d.RminM + 1e-9)
                curv++;
              if ((o.Mode == PlannerMode::NearFieldSlotTrack ||
                   o.Mode == PlannerMode::SlotHold ||
                   o.Mode == PlannerMode::ClosureTaper) &&
                  !d.bSlotLocalPath)
                slotFailures++;
              if (d.bNearFieldExactPoseDubinsCalled)
                exactPoseNear++;
              const double expected = std::max(
                  cfg.MinimumTurnRadiusM,
                  cfg.TurnRadiusSafetyFactor *
                      std::max(cfg.PlanningSpeedFloorMps, speed) *
                      std::max(cfg.PlanningSpeedFloorMps, speed) /
                      (GravityMps2 * std::tan(cfg.PlannerBankLimitRad)));
              if (std::abs(d.RminM - expected) > 1e-6)
                curv++;
            }
            time += 1. / 60;
          }
          maxEval = std::max(maxEval, eval);
          storm += eval > 60;
          grid++;
        }
  Check(grid == 2240, "grid_2240");
  Check(candidateFailed == 0, "candidate_selection_failed_zero");
  Check(storm == 0 && maxEval <= 5, "no_evaluation_storm");
  Check(stale == 0 && nonfinite == 0 && curv == 0, "finite_stale_curvature");
  Check(slotFailures == 0 && exactPoseNear == 0,
        "slot_path_and_no_near_dubins");
  Check(modes[(int)PlannerMode::NearFieldSlotTrack] > 0, "nearfield_mode_seen");
  constexpr std::uint64_t Seed = 0x4E4541524649454CULL;
  std::mt19937_64 rng(Seed);
  std::uniform_real_distribution<double> pos(-2500, 2500), spd(45, 250),
      ang(-Pi, Pi);
  int randomFail = 0;
  for (int k = 0; k < 3000; k++) {
    FormationPlannerV2 p;
    p.Config = cfg;
    double a = pos(rng), c = pos(rng), s = spd(rng), course = ang(rng),
           time = 20;
    bool valid = false;
    for (int f = 0; f < 60; f++) {
      auto in = Input(a, c, s, s, course, time, 1. / 60);
      FormationPlannerV2Diagnostics d{};
      auto o = p.Update(in, d);
      if (o.Failure == PlannerFailure::CandidateSelectionFailed ||
          d.bNearFieldExactPoseDubinsCalled ||
          (o.bValid &&
           (!o.Path.Position.IsFinite() || !Finite(o.TargetEasMps))))
        randomFail++;
      valid |= o.bValid;
      time += 1. / 60;
    }
    if (!valid)
      randomFail++;
  }
  Check(randomFail == 0, "random_3000");
  {
    FormationPlannerV2 p;
    p.Config = cfg;
    double time = 0;
    bool hold = false;
    for (int f = 0; f < 180; f++) {
      auto in = Input(20, 10, 100, 100, 0, time, 1. / 60);
      FormationPlannerV2Diagnostics d{};
      auto o = p.Update(in, d);
      hold |= o.Mode == PlannerMode::SlotHold;
      time += 1. / 60;
    }
    Check(hold, "strict_direct_slot_hold");
  }
  {
    FormationPlannerV2 p;
    p.Config = cfg;
    double time = 0;
    bool nf = false, taper = false;
    for (int f = 0; f < 240; f++) {
      double a = f < 90 ? 1500 : -200, c = f < 90 ? 500 : 80;
      auto in = Input(a, c, 245, 245, 0, time, 1. / 60);
      FormationPlannerV2Diagnostics d{};
      auto o = p.Update(in, d);
      nf |= o.Mode == PlannerMode::NearFieldSlotTrack;
      taper |= o.Mode == PlannerMode::ClosureTaper;
      time += 1. / 60;
    }
    Check(nf, "rejoin_to_nearfield");
    Check(taper, "nearfield_to_taper");
  }
  {
    FormationPlannerV2 p;
    p.Config = cfg;
    double time = 0;
    auto in = Input(300, 100, 220, 220, 0, time, 1. / 60);
    FormationPlannerV2Diagnostics d{};
    p.Update(in, d);
    in.bPaused = true;
    in.SimulationTimeS += 10;
    auto paused = p.Update(in, d);
    Check(!paused.bValid && paused.Failure == PlannerFailure::Paused,
          "pause_invalid");
    in.bPaused = false;
    in.DtS = 1. / 60;
    auto resumed = p.Update(in, d);
    Check(resumed.Failure != PlannerFailure::AbnormalDt, "resume_no_pause_dt");
    in.ResetGeneration = 2;
    auto reset = p.Update(in, d);
    Check(reset.PathGeneration <= 2, "reset_generation_clears_state");
  }
  {
    auto in = Input(-200, 0, 100, 100, 0, 0, .1);
    NearFieldSpeedInputV2 s{};
    s.SlotGroundVelocityNE = in.Slot.GroundVelocityNE;
    s.SlotUnitTangentNE = FromCourse(0);
    s.FollowerGroundVelocityNE = in.FollowerGroundVelocityNE;
    s.WindVelocityNE = {};
    s.AlongErrorM = -200;
    s.EasToTasRatio = 1;
    s.DtS = .1;
    s.bWindValid = s.bRatioValid = s.bEnvelopeValid = true;
    auto behind = NearFieldSpeedPlannerV2::Compute(s);
    s.AlongErrorM = 200;
    auto ahead = NearFieldSpeedPlannerV2::Compute(s);
    Check(behind.ClosureDeltaMps > 0 && ahead.ClosureDeltaMps < 0,
          "behind_ahead_speed_sign");
    s.bWindValid = false;
    Check(!NearFieldSpeedPlannerV2::Compute(s).bValid, "invalid_wind");
    s.bWindValid = true;
    s.bRatioValid = false;
    Check(!NearFieldSpeedPlannerV2::Compute(s).bValid, "invalid_ratio");
  }
  std::cout << "NEARFIELD_SLOT_TRACK checks=" << Checks << " grid=" << grid
            << " random_seed=" << Seed
            << " random_cases=3000 failures=" << Failures
            << " candidate_fail=" << candidateFailed << " max_eval=" << maxEval
            << " storm=" << storm << " stale=" << stale
            << " nonfinite=" << nonfinite << " curvature=" << curv
            << " slot_fail=" << slotFailures << " near_dubins=" << exactPoseNear
            << " modes=" << modes[0] << '/' << modes[1] << '/' << modes[2]
            << '/' << modes[3] << '/' << modes[4] << '\n';
  std::cout << "candidate_fail_modes=" << candidateMode[0] << '/' << candidateMode[1] << '/'
            << candidateMode[2] << '/' << candidateMode[3] << '/' << candidateMode[4] << '\n';
  return Failures ? 1 : 0;
}
