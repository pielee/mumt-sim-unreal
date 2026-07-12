#include "FormationControlV2/CanonicalNavigationAdapterV2.h"
#include "FormationControlV2/FormationGuidanceCoordinatorV2.h"
#include "FormationControlV2/FormationPlannerV2.h"
#include "FormationControlV2/FormationSlotGeneratorV2.h"
#include "FormationControlV2/MissionNavigationFrameV2.h"
#include "FormationControlV2/PlannerV2Adapters.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
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
FFormationSlotCommandV2 Command(double time) { return {-300.0,100.0,50.0,time,1,true}; }
struct Pipeline {
    MissionNavigationFrameV2 Frame; FCanonicalNavigationTrackerV2 LeaderTracker, FollowerTracker;
    FormationPlannerV2 Planner; FormationGuidanceCoordinatorV2 Guidance; std::uint32_t Reset{1}; double Time{10.0};
    Pipeline() { Frame.SetOrigin({0.6,2.2,100.0,7,true}); Planner.Config.MaxCaptureTimeS=600; Planner.Config.MaxPathLengthM=200000; }
    FGuidanceCoordinatorInputV2 Make(double course, double cross, double along, double altitudeDelta=0,
                                     double eas=100, Vec2 wind={}, double dt=.02) {
        const Vec2 t=FromCourse(course), right{-t.E,t.N}; const Vec3dV2 leader{1000,2000,-2900};
        const Vec3dV2 follower{leader.X-along*t.N+cross*right.N,leader.Y-along*t.E+cross*right.E,-2900-altitudeDelta};
        const Vec3dV2 velocity{100*t.N,100*t.E,0};
        auto lr=Raw(Frame,leader,velocity,Time,eas,wind),fr=Raw(Frame,follower,velocity,Time,eas,wind);
        auto l=CanonicalNavigationAdapterV2::Convert(lr,Frame,Reset,LeaderTracker);
        auto f=CanonicalNavigationAdapterV2::Convert(fr,Frame,Reset,FollowerTracker);
        auto slot=FormationSlotGeneratorV2::Calculate(l,Command(Time),Time);
        auto pi=PlannerV2InputAdapter::Build({f,slot,Time,dt}); FormationPlannerV2Diagnostics d{};
        auto po=pi.bValid?Planner.Update(pi.Input,d):FormationPlannerV2Output{};
        auto dto=PlannerV2OutputAdapter::Build(po,slot,f); FGuidanceCoordinatorInputV2 gi{};
        gi.Follower=f;gi.Slot=slot;gi.PlannerDto=dto;gi.CurrentPitchRad=0;gi.bCurrentPitchValid=true;
        gi.SimulationTimeS=Time;gi.DtS=dt;gi.ResetGeneration=Reset;gi.OriginGeneration=7;return gi;
    }
    FGuidanceCoordinatorOutputV2 Step(FGuidanceCoordinatorInputV2 in, const FGuidanceConfigV2 &c={}) {
        auto o=Guidance.Update(in,c);Time+=std::max(0.0,in.DtS);return o;
    }
};
bool FiniteOutput(const FGuidanceCoordinatorOutputV2&o) { return std::isfinite(o.RollReferenceRad)&&std::isfinite(o.PitchReferenceRad)&&std::isfinite(o.ThrottleReferenceNorm); }
}

int main() {
    FGuidanceConfigV2 config{}; Pipeline p; auto in=p.Make(0,100,2000); Check(!p.Step(in,config).bCommandReady,"initial_reset_frame");
    in=p.Make(0,100,2000);auto north=p.Step(in,config);Check(north.bCommandReady&&FiniteOutput(north),"north_pipeline");
    const double courses[]={Pi/2,Pi,-Pi/2};for(double c:courses){Pipeline cardinal;auto ci=cardinal.Make(c,100,2000);cardinal.Step(ci,config);ci=cardinal.Make(c,100,2000);Check(cardinal.Step(ci,config).bCommandReady,"east_south_west");}
    Pipeline left,right;auto li=left.Make(0,-200,2000),ri=right.Make(0,200,2000);left.Step(li);right.Step(ri);li=left.Make(0,-200,2000);ri=right.Make(0,200,2000);auto lo=left.Step(li),ro=right.Step(ri);Check(lo.bCommandReady&&ro.bCommandReady&&lo.RollReferenceRad*ro.RollReferenceRad<=0,"cross_track_symmetry");
    Pipeline altLow,altHigh;auto al=altLow.Make(0,0,2000,-500,70),ah=altHigh.Make(0,0,2000,500,130);altLow.Step(al);altHigh.Step(ah);al=altLow.Make(0,0,2000,-500,70);ah=altHigh.Make(0,0,2000,500,130);Check(altLow.Step(al).bCommandReady&&altHigh.Step(ah).bCommandReady,"altitude_eas_cases");
    Pipeline windy;auto wi=windy.Make(0,0,2000,0,80,{0,120});windy.Step(wi);wi=windy.Make(0,0,2000,0,80,{0,120});auto wo=windy.Step(wi);Check(wo.bCommandReady&&wo.WindFeasibility<=1,"strong_wind_feasibility");
    auto bad=wi;bad.PlannerDto.Npfg.PathUnitTangentNE={};Check(!windy.Step(bad).bCommandReady,"zero_tangent");
    bad=wi;bad.Follower.bWindValid=false;Check(!windy.Step(bad).bCommandReady,"invalid_wind");
    bad=wi;bad.Follower.bRatioValid=false;Check(!windy.Step(bad).bCommandReady,"invalid_ratio");
    bad=wi;bad.PlannerDto.Tecs.bTargetEasValid=false;Check(!windy.Step(bad).bCommandReady,"invalid_target_eas");
    bad=wi;bad.PlannerDto.Tecs.bTargetAltitudeValid=false;Check(!windy.Step(bad).bCommandReady,"invalid_altitude");
    bad=wi;bad.OriginGeneration=8;Check(!windy.Step(bad).bCommandReady,"origin_mismatch");
    bad=wi;bad.bPaused=true;Check(!windy.Step(bad).bCommandReady,"pause");bad.bPaused=false;Check(!windy.Step(bad).bCommandReady,"resume_first_frame");
    bad=wi;bad.ResetGeneration=2;bad.Follower.ResetGeneration=2;bad.Slot.ResetGeneration=2;Check(!windy.Step(bad).bCommandReady,"reset_generation");
    bad=wi;bad.PlannerDto.Tecs.TargetEasMps=std::numeric_limits<double>::quiet_NaN();Check(!windy.Step(bad).bCommandReady,"nan_and_stale_clear");
    Pipeline dtCase;auto di=dtCase.Make(0,0,2000,0,100,{},0.0005);dtCase.Step(di);di=dtCase.Make(0,0,2000,0,100,{},0.0005);Check(dtCase.Step(di).bCommandReady,"dt_below_tecs_min_safe");di.DtS=1.1;di.SimulationTimeS+=1.1;di.Follower.SimulationTimeS=di.SimulationTimeS;Check(dtCase.Step(di).bCommandReady,"dt_above_tecs_max_reset_safe");
    Pipeline protection;auto pr=protection.Make(0,0,2000,0,100);protection.Step(pr);pr=protection.Make(0,0,2000,0,100);
    pr.Follower.EquivalentAirspeed_mps=5;pr.Follower.TrueAirspeed_mps=5;pr.PlannerDto.Tecs.TargetEasMps=80;
    FGuidanceCoordinatorOutputV2 protectedOut{};for(int i=0;i<160;++i){pr.SimulationTimeS+=.02;pr.Follower.SimulationTimeS=pr.SimulationTimeS;protectedOut=protection.Step(pr);}
    std::cout<<" protection underspeed="<<protectedOut.UnderspeedRatio<<" fast="<<protectedOut.FastDescendRatio<<" ready="<<protectedOut.bCommandReady;
    Check(protectedOut.UnderspeedRatio>0,"tecs_underspeed_entry");
    Pipeline descend;auto fd=descend.Make(0,0,2000,0,100);descend.Step(fd);fd=descend.Make(0,0,2000,0,100);fd.Follower.AltitudeAsl_m=fd.PlannerDto.Tecs.TargetAltitudeAslM+500;
    FGuidanceCoordinatorOutputV2 fastOut{};for(int i=0;i<160;++i){fd.SimulationTimeS+=.02;fd.Follower.SimulationTimeS=fd.SimulationTimeS;fastOut=descend.Step(fd);}
    std::cout<<" descend underspeed="<<fastOut.UnderspeedRatio<<" fast="<<fastOut.FastDescendRatio<<" ready="<<fastOut.bCommandReady<<'\n';Check(fastOut.FastDescendRatio>0,"tecs_fast_descend_entry");

    constexpr std::uint64_t Seed=0x47554944414E4345ULL;constexpr int Cases=3000;std::mt19937_64 rng(Seed);
    std::uniform_real_distribution<double> angle(-Pi,Pi),cross(-1000,1000),along(1200,6000),alt(-800,800),eas(45,160),wind(-50,50);
    int randomFailures=0;double rollMin=1e9,rollMax=-1e9,pitchMin=1e9,pitchMax=-1e9,thrMin=1e9,thrMax=-1e9;
    for(int i=0;i<Cases;++i){Pipeline q;const double a=angle(rng),x=cross(rng),s=along(rng),h=alt(rng),v=eas(rng);const Vec2 w{wind(rng),wind(rng)};auto qi=q.Make(a,x,s,h,v,w);q.Step(qi,config);qi=q.Make(a,x,s,h,v,w);auto qo=q.Step(qi,config);
        bool ok=qo.bCommandReady&&FiniteOutput(qo)&&std::abs(qo.RollReferenceRad)<=config.RollLimitRad+1e-12&&qo.PitchReferenceRad>=config.PitchMinRad-1e-6&&qo.PitchReferenceRad<=config.PitchMaxRad+1e-6&&qo.ThrottleReferenceNorm>=config.ThrottleMin-1e-6&&qo.ThrottleReferenceNorm<=config.ThrottleMax+1e-6;
        if(i%17==0){qi.Follower.bWindValid=false;auto inv=q.Step(qi,config);ok=ok&&!inv.bCommandReady&&inv.RollReferenceRad==0&&inv.PitchReferenceRad==0&&inv.ThrottleReferenceNorm==0;}
        randomFailures+=!ok;rollMin=std::min(rollMin,qo.RollReferenceRad);rollMax=std::max(rollMax,qo.RollReferenceRad);pitchMin=std::min(pitchMin,qo.PitchReferenceRad);pitchMax=std::max(pitchMax,qo.PitchReferenceRad);thrMin=std::min(thrMin,qo.ThrottleReferenceNorm);thrMax=std::max(thrMax,qo.ThrottleReferenceNorm);}
    Failures+=randomFailures;
    std::cout<<"GUIDANCE_INTEGRATION_V2 checks="<<Checks<<" seed="<<Seed<<" random_cases="<<Cases<<" random_failures="<<randomFailures<<" failures="<<Failures
             <<" roll_range=["<<rollMin<<','<<rollMax<<"] pitch_range=["<<pitchMin<<','<<pitchMax<<"] throttle_range=["<<thrMin<<','<<thrMax<<"]\n";
    return Failures?1:0;
}
