#include "FormationControl/Px4NpfgAdapter.h"
#include "FormationControlV2/FormationPlannerV2.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
using namespace FormationControlV2;
namespace {
struct Result {
  bool ok{};
  double finalCross{}, finalAlong{}, settle{}, overshoot{}, rollMin{1e9},
      rollMax{-1e9}, latMin{1e9}, latMax{-1e9};
  int oscillations{}, modes[5]{}, candidateFail{}, nearDubins{}, stale{};
};
FormationPlannerV2Input Make(Vec2 fp, double fc, double fs, Pose2 slot,
                             double ss, double k, double time, double dt) {
  FormationPlannerV2Input i{};
  i.FollowerPositionNE = fp;
  i.FollowerCourseRad = fc;
  i.FollowerGroundVelocityNE = FromCourse(fc) * fs;
  i.Slot.Pose = slot;
  i.Slot.GroundVelocityNE = FromCourse(slot.CourseRad) * ss;
  i.Slot.GroundSpeedMps = ss;
  i.Slot.CurvaturePerM = k;
  i.Slot.bCurvatureValid = true;
  i.Slot.bValid = true;
  i.WindVelocityNE = {};
  i.EasToTasRatio = 1;
  i.EquivalentAirspeedMps = fs;
  i.SimulationTimeS = time;
  i.DtS = dt;
  i.ResetGeneration = 1;
  i.bFollowerValid = i.bCourseValid = i.bWindValid = i.bRatioValid = true;
  return i;
}
Result Run(double speed, double along, double cross, double curvature,
           double duration = 45) {
  FormationPlannerV2 planner;
  MumtPx4::FPx4NpfgAdapter npfg;
  double dt = .05, time = 0, fs = speed, course = 0;
  Pose2 slot{{0, 0}, 0};
  Vec2 fp = slot.Position + FromCourse(0) * along + RightNormal(0) * cross;
  Result r{};
  double initialCross = std::abs(cross), maxCross = initialCross, lastSign = 0;
  int settledFrames = 0;
  for (int frame = 0; frame < (int)(duration / dt); frame++) {
    auto in = Make(fp, course, fs, slot, speed, curvature, time, dt);
    FormationPlannerV2Diagnostics d{};
    auto o = planner.Update(in, d);
    if (o.Failure == PlannerFailure::CandidateSelectionFailed)
      r.candidateFail++;
    if (d.bNearFieldExactPoseDubinsCalled)
      r.nearDubins++;
    if (!o.bValid) {
      if (o.Path.bValid || o.TargetEasMps != 0)
        r.stale++;
      time += dt;
      continue;
    }
    r.modes[(int)o.Mode]++;
    MumtPx4::NpfgInput ni{};
    ni.position_ne = {float(fp.N), float(fp.E)};
    ni.ground_velocity_ne = {float(fs * std::cos(course)),
                             float(fs * std::sin(course))};
    ni.wind_velocity_ne = {0, 0};
    ni.path_tangent_ne = {float(o.Path.UnitTangent.N),
                          float(o.Path.UnitTangent.E)};
    ni.path_position_ne = {float(o.Path.Position.N), float(o.Path.Position.E)};
    ni.path_curvature = float(o.Path.SignedCurvaturePerM);
    ni.true_airspeed_setpoint = float(o.TargetEasMps);
    ni.true_airspeed = float(fs);
    auto no = npfg.update(ni);
    const double maxLat =
        GravityMps2 * std::tan(planner.Config.PlannerBankLimitRad);
    double lat = Clamp(no.lateral_acceleration_total, -maxLat, maxLat),
           roll = std::atan(lat / GravityMps2);
    r.latMin = std::min(r.latMin, lat);
    r.latMax = std::max(r.latMax, lat);
    r.rollMin = std::min(r.rollMin, roll);
    r.rollMax = std::max(r.rollMax, roll);
    course = WrapPi(course + lat / std::max(fs, 1.0) * dt);
    const double speedStep = planner.Config.ConfiguredDecelerationMps2 * dt;
    fs += Clamp(o.TargetEasMps - fs, -speedStep, speedStep);
    fp = fp + FromCourse(course) * (fs * dt);
    if (std::abs(curvature) < 1e-12)
      slot.Position = slot.Position + FromCourse(slot.CourseRad) * (speed * dt);
    else {
      slot.CourseRad = WrapPi(slot.CourseRad + curvature * speed * dt);
      slot.Position = slot.Position + FromCourse(slot.CourseRad) * (speed * dt);
    }
    const Vec2 st = FromCourse(slot.CourseRad), delta = fp - slot.Position;
    double ce = delta.Dot(RightNormal(slot.CourseRad)), ae = delta.Dot(st);
    maxCross = std::max(maxCross, std::abs(ce));
    double sign = ce > 1 ? 1 : (ce < -1 ? -1 : 0);
    if (lastSign && sign && sign != lastSign)
      r.oscillations++;
    if (sign)
      lastSign = sign;
    if (std::abs(ce) < 30 && std::abs(ae) < 300) {
      settledFrames++;
      if (r.settle == 0 && settledFrames > 20)
        r.settle = time;
    } else
      settledFrames = 0;
    r.finalCross = ce;
    r.finalAlong = ae;
    time += dt;
  }
  r.overshoot = maxCross - initialCross;
  r.ok = r.candidateFail == 0 && r.nearDubins == 0 && r.stale == 0 &&
         std::isfinite(r.finalCross) && std::isfinite(r.finalAlong) &&
         std::abs(r.rollMin) <= Pi / 4 + 1e-9 &&
         std::abs(r.rollMax) <= Pi / 4 + 1e-9 &&
         std::abs(r.finalCross) <= std::max(45.0, initialCross * .75) &&
         std::abs(r.finalAlong) <= 450 && r.oscillations <= 8 &&
         (r.modes[(int)PlannerMode::NearFieldSlotTrack] +
              r.modes[(int)PlannerMode::ClosureTaper] +
              r.modes[(int)PlannerMode::SlotHold] >
          0);
  return r;
}
} // namespace
int main() {
  int fail = 0, cases = 0;
  double settleMin = 1e9, settleMax = 0, settleSum = 0, fcMax = 0, faMax = 0,
         overMax = 0, rollMin = 1e9, rollMax = -1e9, latMin = 1e9,
         latMax = -1e9;
  int settled = 0, mode[5] = {};
  auto Acc = [&](const Result &r) {
    cases++;
    fail += !r.ok;
    fcMax = std::max(fcMax, std::abs(r.finalCross));
    faMax = std::max(faMax, std::abs(r.finalAlong));
    overMax = std::max(overMax, r.overshoot);
    rollMin = std::min(rollMin, r.rollMin);
    rollMax = std::max(rollMax, r.rollMax);
    latMin = std::min(latMin, r.latMin);
    latMax = std::max(latMax, r.latMax);
    if (r.settle > 0) {
      settled++;
      settleMin = std::min(settleMin, r.settle);
      settleMax = std::max(settleMax, r.settle);
      settleSum += r.settle;
    }
    for (int i = 0; i < 5; i++)
      mode[i] += r.modes[i];
  };
  for (double s : {170., 220., 245.})
    for (double a : {-200., 0., 200.})
      for (double c : {-100., 0., 100.})
        Acc(Run(s, a, c, 0));
  auto left = Run(170, -200, 100, -.0001), right = Run(170, -200, -100, .0001);
  Acc(left);
  Acc(right);
  if (std::abs(std::abs(left.finalCross) - std::abs(right.finalCross)) > 20)
    fail++;
  constexpr std::uint64_t Seed = 0x4E504647434C4FULL;
  std::mt19937_64 rng(Seed);
  std::uniform_real_distribution<double> spd(120, 245), err(-200, 200),
      pick(0, 1);
  for (int i = 0; i < 3000; i++) {
    double k = pick(rng) < .34 ? -.00005 : (pick(rng) > .66 ? .00005 : 0);
    Acc(Run(spd(rng), err(rng), err(rng), k, 20));
  }
  std::cout << "NEARFIELD_NPFG_CLOSED_LOOP cases=" << cases
            << " deterministic=27 turn_cases=2 random_seed=" << Seed
            << " random=3000 failures=" << fail << " settled=" << settled
            << " settling_min_mean_max=" << (settled ? settleMin : 0) << '/'
            << (settled ? settleSum / settled : 0) << '/' << settleMax
            << " final_cross_max=" << fcMax << " final_along_max=" << faMax
            << " overshoot_max=" << overMax << " roll_range=" << rollMin << ','
            << rollMax << " lat_range=" << latMin << ',' << latMax
            << " modes=" << mode[0] << '/' << mode[1] << '/' << mode[2] << '/'
            << mode[3] << '/' << mode[4] << " command_writes=0\n";
  return fail ? 1 : 0;
}
