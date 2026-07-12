// verify_guidance_stick_integration_v2.cpp — synthetic offline pipeline audit:
//   Navigation -> Slot -> Planner V2 -> Planner DTO -> Guidance (real PX4 NPFG/TECS)
//   -> Roll/Pitch/Throttle references -> F16StickAdapterV2 -> command DTO.
// Computation stops at the command DTO: nothing is written to JSBSim or the property manager.
#include "FormationControlV2/CanonicalNavigationAdapterV2.h"
#include "FormationControlV2/F16StickAdapterV2.h"
#include "FormationControlV2/FormationGuidanceCoordinatorV2.h"
#include "FormationControlV2/FormationPlannerV2.h"
#include "FormationControlV2/FormationSlotGeneratorV2.h"
#include "FormationControlV2/MissionNavigationFrameV2.h"
#include "FormationControlV2/PlannerV2Adapters.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>

using namespace FormationControlV2;
namespace {
int Failures{}, Checks{};
void Check(bool value, const char *name) { ++Checks; if (!value) { ++Failures; std::cerr << "FAIL " << name << '\n'; } }
constexpr double Ft = 0.3048, Kt = 0.514444444444444;

FNavigationRawSnapshotV2 Raw(const MissionNavigationFrameV2 &frame, Vec3dV2 ned, Vec3dV2 velocity,
                             double time, double eas, Vec2 wind = {}) {
    bool valid{}; const auto ep = frame.MissionNedPositionToEcefM(ned, valid);
    const auto ev = frame.MissionNedVelocityToEcefMps(velocity, valid); FNavigationRawSnapshotV2 r{};
    r.VehicleCgEcefFt = {ep.X/Ft,ep.Y/Ft,ep.Z/Ft}; r.EcefVelocityFps = {ev.X/Ft,ev.Y/Ft,ev.Z/Ft};
    r.EquivalentAirspeedKts=eas/Kt; r.TrueAirspeedFps=eas/Ft; r.WindNEDFps={wind.N/Ft,wind.E/Ft};
    r.AltitudeAslFt=(3000.0-ned.Z)/Ft; r.ClimbRateFps=-velocity.Z/Ft; r.SimulationTimeS=time; r.bValidFrame=true; return r;
}
struct Pipeline {
    MissionNavigationFrameV2 Frame; FCanonicalNavigationTrackerV2 LeaderTracker, FollowerTracker;
    FormationPlannerV2 Planner; FormationGuidanceCoordinatorV2 Guidance; F16StickAdapterV2 Stick;
    std::uint32_t Reset{1}; double Time{10.0};
    Pipeline() { Frame.SetOrigin({0.6,2.2,100.0,7,true}); Planner.Config.MaxCaptureTimeS=600; Planner.Config.MaxPathLengthM=200000; }
    // one full frame: build guidance input from geometry, update guidance, feed the stick adapter.
    struct Frame2 { FGuidanceCoordinatorOutputV2 Guidance; FF16StickCommandV2 Stick; };
    Frame2 Step(double course, double cross, double along, double roll, double pitch,
                const FF16StickConfigV2 &stickConfig = {}, bool forceGuidanceInvalid = false, double dt = .02) {
        const Vec2 t=FromCourse(course), right{-t.E,t.N}; const Vec3dV2 leader{1000,2000,-2900};
        const Vec3dV2 follower{leader.X-along*t.N+cross*right.N,leader.Y-along*t.E+cross*right.E,-2900};
        const Vec3dV2 velocity{100*t.N,100*t.E,0};
        auto lr=Raw(Frame,leader,velocity,Time,100),fr=Raw(Frame,follower,velocity,Time,100);
        auto l=CanonicalNavigationAdapterV2::Convert(lr,Frame,Reset,LeaderTracker);
        auto f=CanonicalNavigationAdapterV2::Convert(fr,Frame,Reset,FollowerTracker);
        auto slot=FormationSlotGeneratorV2::Calculate(l,{-300.0,100.0,50.0,Time,1,true},Time);
        auto pi=PlannerV2InputAdapter::Build({f,slot,Time,dt}); FormationPlannerV2Diagnostics d{};
        auto po=pi.bValid?Planner.Update(pi.Input,d):FormationPlannerV2Output{};
        auto dto=PlannerV2OutputAdapter::Build(po,slot,f);
        FGuidanceCoordinatorInputV2 gi{}; gi.Follower=f;gi.Slot=slot;gi.PlannerDto=dto;
        gi.CurrentPitchRad=pitch;gi.bCurrentPitchValid=true;gi.SimulationTimeS=Time;gi.DtS=dt;
        gi.ResetGeneration=Reset;gi.OriginGeneration=7;
        if (forceGuidanceInvalid) gi.Follower.bWindValid=false;
        const auto go=Guidance.Update(gi);
        FF16StickInputV2 si{};
        si.RollReferenceRad=go.RollReferenceRad; si.PitchReferenceRad=go.PitchReferenceRad;
        si.ThrottleReferenceNorm=go.ThrottleReferenceNorm; si.bGuidanceValid=go.bCommandReady;
        si.CurrentRollRad=roll; si.CurrentPitchRad=pitch; si.bAttitudeValid=true;
        si.bBodyRatesValid=true; si.bAlphaBetaValid=true; si.bAirspeedValid=true;
        si.EasMps=100; si.TasMps=100;
        si.SimulationTimeS=Time; si.DtS=dt; si.ResetGeneration=go.ResetGeneration;
        const auto so=Stick.Update(si,stickConfig);
        Time+=dt;
        return {go,so};
    }
};
} // namespace

int main() {
    // warm up: frame 1 is a reset frame for BOTH guidance and stick (stick consumes its init
    // frame while guidance is resetting), so the chain is ready from frame 2 onward.
    { Pipeline p; auto f1 = p.Step(0, 200, 2000, 0, 0);
      Check(!f1.Guidance.bCommandReady && !f1.Stick.bValid &&
            f1.Stick.FailureReason == EF16StickFailureV2::ResetFrame, "first_frames_reset");
      auto f2 = p.Step(0, 200, 2000, 0, 0);
      Check(f2.Guidance.bCommandReady && f2.Stick.bValid, "pipeline_ready_second_frame");
      auto f3 = p.Step(0, 200, 2000, 0, 0);
      Check(f3.Guidance.bCommandReady && f3.Stick.bValid, "pipeline_stays_ready"); }

    // Roll reference sign -> aileron sign (current roll 0, sustained frames)
    { Pipeline left, right; FF16StickCommandV2 lo{}, ro{}; double lroll = 0, rroll = 0;
      for (int i = 0; i < 40; ++i) { lo = left.Step(0, -400, 2500, 0, 0).Stick; ro = right.Step(0, 400, 2500, 0, 0).Stick; }
      auto lg = left.Step(0, -400, 2500, 0, 0), rg = right.Step(0, 400, 2500, 0, 0);
      lroll = lg.Guidance.RollReferenceRad; rroll = rg.Guidance.RollReferenceRad; lo = lg.Stick; ro = rg.Stick;
      Check(lo.bValid && ro.bValid && lroll * rroll <= 0.0, "mirrored_roll_references");
      Check((lroll > 0) == (lo.AileronCmdNorm > 0) && (rroll > 0) == (ro.AileronCmdNorm > 0), "roll_sign_maps_to_aileron_sign"); }

    // Pitch reference sign -> elevator sign INVERTED (nose-up ref (+) => negative elevator)
    { Pipeline p; FF16StickCommandV2 o{}; FGuidanceCoordinatorOutputV2 g{};
      // hold current pitch below reference by feeding pitch = -0.1 while guidance pitch ref ~ 0
      for (int i = 0; i < 40; ++i) { auto f = p.Step(0, 0, 2500, 0, -0.1); g = f.Guidance; o = f.Stick; }
      const double pitchErr = g.PitchReferenceRad - (-0.1);
      Check(o.bValid && (pitchErr > 1e-6 ? o.ElevatorCmdNorm < 0 : true) &&
            (pitchErr < -1e-6 ? o.ElevatorCmdNorm > 0 : true), "pitch_sign_inverts_to_elevator"); }

    // throttle range mapping
    { Pipeline p; FF16StickCommandV2 o{}; FGuidanceCoordinatorOutputV2 g{};
      for (int i = 0; i < 60; ++i) { auto f = p.Step(0, 0, 2500, 0, 0); g = f.Guidance; o = f.Stick; }
      Check(o.bValid && o.ThrottleCmdNorm >= 0.0 && o.ThrottleCmdNorm <= 1.0 &&
            std::isfinite(g.ThrottleReferenceNorm), "throttle_within_range"); }

    // invalid guidance output -> stick outputs fresh neutral invalid (no stale command)
    { Pipeline p; for (int i = 0; i < 40; ++i) p.Step(0, 300, 2500, 0, 0);
      auto bad = p.Step(0, 300, 2500, 0, 0, {}, /*forceGuidanceInvalid=*/true);
      Check(!bad.Guidance.bCommandReady && !bad.Stick.bValid &&
            bad.Stick.AileronCmdNorm == 0 && bad.Stick.ElevatorCmdNorm == 0 &&
            bad.Stick.RudderCmdNorm == 0 && bad.Stick.ThrottleCmdNorm == 0,
            "invalid_guidance_gives_fresh_neutral_stick");
      auto again = p.Step(0, 300, 2500, 0, 0);
      Check(again.Guidance.bCommandReady && again.Stick.bValid, "recovers_after_invalid"); }

    // reset generation propagates guidance -> stick
    { Pipeline p; for (int i = 0; i < 10; ++i) p.Step(0, 100, 2500, 0, 0);
      auto f = p.Step(0, 100, 2500, 0, 0);
      Check(f.Stick.ResetGeneration == f.Guidance.ResetGeneration && f.Guidance.ResetGeneration == 1,
            "reset_generation_match"); }

    // fixed-seed random sweep: whole chain, command DTO validity + ranges + finiteness
    constexpr std::uint64_t Seed = 0x475549445F53544BULL; // "GUID_STK"
    std::mt19937_64 rng(Seed);
    std::uniform_real_distribution<double> angle(-Pi, Pi), cross(-800, 800), along(1500, 5000), att(-0.3, 0.3);
    constexpr int Cases = 1500;
    int randomFailures = 0;
    double aMin = 1e9, aMax = -1e9, eMin = 1e9, eMax = -1e9, tMin = 1e9, tMax = -1e9;
    for (int i = 0; i < Cases; ++i) {
        Pipeline q; const double c = angle(rng), x = cross(rng), s = along(rng), roll = att(rng), pitch = att(rng) * 0.5;
        FF16StickCommandV2 o{}; FGuidanceCoordinatorOutputV2 g{};
        for (int f = 0; f < 8; ++f) { auto fr = q.Step(c, x, s, roll, pitch); g = fr.Guidance; o = fr.Stick; }
        bool ok = g.bCommandReady && o.bValid &&
                  std::isfinite(o.AileronCmdNorm) && std::isfinite(o.ElevatorCmdNorm) &&
                  std::isfinite(o.RudderCmdNorm) && std::isfinite(o.ThrottleCmdNorm) &&
                  o.AileronCmdNorm >= -1 && o.AileronCmdNorm <= 1 &&
                  o.ElevatorCmdNorm >= -1 && o.ElevatorCmdNorm <= 1 &&
                  o.RudderCmdNorm >= -1 && o.RudderCmdNorm <= 1 &&
                  o.ThrottleCmdNorm >= 0 && o.ThrottleCmdNorm <= 1 &&
                  o.ResetGeneration == g.ResetGeneration;
        randomFailures += !ok;
        aMin = std::min(aMin, o.AileronCmdNorm); aMax = std::max(aMax, o.AileronCmdNorm);
        eMin = std::min(eMin, o.ElevatorCmdNorm); eMax = std::max(eMax, o.ElevatorCmdNorm);
        tMin = std::min(tMin, o.ThrottleCmdNorm); tMax = std::max(tMax, o.ThrottleCmdNorm);
    }
    Failures += randomFailures;
    std::cout << "GUIDANCE_STICK_V2 checks=" << Checks << " seed=" << Seed << " random_cases=" << Cases
              << " random_failures=" << randomFailures << " failures=" << Failures
              << " aileron_range=[" << aMin << ',' << aMax << "] elevator_range=[" << eMin << ',' << eMax
              << "] throttle_range=[" << tMin << ',' << tMax << "]\n";
    return Failures ? 1 : 0;
}
