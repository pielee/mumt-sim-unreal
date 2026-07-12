#include "FormationControlV2/CanonicalNavigationAdapterV2.h"
#include "FormationControlV2/FormationPlannerV2.h"
#include "FormationControlV2/FormationSlotGeneratorV2.h"
#include "FormationControlV2/PlannerV2Adapters.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
using namespace FormationControlV2;
namespace {int fail=0,checks=0;void C(bool x,const char*n){checks++;if(!x){fail++;std::cerr<<"FAIL "<<n<<'\n';}}
constexpr double Ft=0.3048;
FNavigationRawSnapshotV2 Raw(const MissionNavigationFrameV2&f,Vec3dV2 ned,Vec3dV2 vel,double t,double courseSpeed=200){
 bool ok=false;auto ep=f.MissionNedPositionToEcefM(ned,ok);auto ev=f.MissionNedVelocityToEcefMps(vel,ok);FNavigationRawSnapshotV2 r{};
 r.VehicleCgEcefFt={ep.X/Ft,ep.Y/Ft,ep.Z/Ft};r.EcefVelocityFps={ev.X/Ft,ev.Y/Ft,ev.Z/Ft};r.EquivalentAirspeedKts=courseSpeed/0.514444444444444;r.TrueAirspeedFps=courseSpeed/Ft;r.WindNEDFps={10/Ft,-5/Ft};r.AltitudeAslFt=3000/Ft;r.ClimbRateFps=2/Ft;r.SimulationTimeS=t;r.bValidFrame=true;return r;}
FFormationSlotCommandV2 Cmd(double t){return {-200,100,50,t,9,true};}
}
int main(){MissionNavigationFrameV2 frame;C(frame.SetOrigin({0.6,2.2,100,3,true}),"origin");
 FCanonicalNavigationTrackerV2 lt,ft;auto lr=Raw(frame,{1000,2000,-2900},{100,0,-2},10,100);auto fr=Raw(frame,{-1000,1800,-2900},{105,0,0},10,105);
 auto l=CanonicalNavigationAdapterV2::Convert(lr,frame,1,lt);auto f=CanonicalNavigationAdapterV2::Convert(fr,frame,1,ft);
 C(l.bPositionValid&&std::abs(l.PositionNE_m.N-1000)<1e-8&&std::abs(l.PositionNE_m.E-2000)<1e-8,"ecef_position_ne");
 C(l.bGroundVelocityValid&&std::abs(l.GroundVelocityNE_mps.N-100)<1e-10,"ecef_velocity_ne");C(l.bGroundCourseValid&&std::abs(l.GroundCourse_rad)<1e-12,"course_north");C(l.bWindValid&&l.WindNE_mps.N>0&&l.WindNE_mps.E<0,"wind_toward");
 lr=Raw(frame,{1010,2000,-2900},{100*std::cos(.001),100*std::sin(.001),-2},10.1,100);l=CanonicalNavigationAdapterV2::Convert(lr,frame,1,lt);C(l.bCourseRateValid&&l.CourseRate_radps>0&&l.bCurvatureValid,"right_course_rate_curvature");
 auto slot=FormationSlotGeneratorV2::Calculate(l,Cmd(10.1),10.1);C(slot.bValid&&slot.bCurvatureValid&&slot.OriginGeneration==3,"moving_slot");
 auto in=PlannerV2InputAdapter::Build({f,slot,10.1,.1});C(in.bValid,"planner_input");FormationPlannerV2 planner;planner.Config.MaxCaptureTimeS=600;planner.Config.MaxPathLengthM=200000;FormationPlannerV2Diagnostics diag{};auto po=planner.Update(in.Input,diag);auto out=PlannerV2OutputAdapter::Build(po,slot,f);C(po.bPathValid&&po.Path.bValid,"planner_path");C(out.Npfg.bValid&&out.Tecs.bCommandReady&&out.Tecs.TargetAltitudeAslM==slot.AltitudeAsl_m,"output_contracts");C(out.Tecs.TargetEasMps>0,"target_eas");
 auto paused=lr;paused.bHolding=true;auto lp=CanonicalNavigationAdapterV2::Convert(paused,frame,1,lt);C(lp.bPaused&&!lp.bCourseRateValid,"pause");
 auto reset=CanonicalNavigationAdapterV2::Convert(lr,frame,2,lt);C(!reset.bCourseRateValid&&reset.ResetGeneration==2,"reset_generation");
 MissionNavigationFrameV2 frame2;frame2.SetOrigin({.6,2.2,100,4,true});auto changed=CanonicalNavigationAdapterV2::Convert(lr,frame2,2,lt);C(!changed.bCourseRateValid&&changed.OriginGeneration==4,"origin_generation_change");
 auto low=Raw(frame,{0,0,0},{.1,.1,0},20,.2);FCanonicalNavigationTrackerV2 lowt;auto lows=CanonicalNavigationAdapterV2::Convert(low,frame,1,lowt);C(!lows.bGroundCourseValid,"low_speed_course_invalid");
 auto stale=PlannerV2InputAdapter::Build({f,slot,20,.1});C(!stale.bValid,"stale_source");FormationPlannerV2Output bad{};auto cleared=PlannerV2OutputAdapter::Build(bad,slot,f);C(!cleared.Npfg.bValid&&!cleared.Tecs.bCommandReady&&cleared.Tecs.TargetEasMps==0,"stale_output_zero");
 constexpr std::uint64_t seed=0x494E544547ULL;constexpr int cases=2000;std::mt19937_64 rng(seed);std::uniform_real_distribution<double> pos(-10000,10000),ang(-Pi,Pi),spd(80,130),range(1500,5000),wind(-20,20);int randomFail=0,canonicalFail=0,slotFail=0,inputFail=0,pathFail=0,outputFail=0;
 for(int i=0;i<cases;i++){double a=ang(rng),s=spd(rng),t=100+i;Vec2 tv=FromCourse(a);Vec3dV2 ln{pos(rng),pos(rng),-3000},lv{s*tv.N,s*tv.E,0},fn{ln.X-range(rng)*tv.N,ln.Y-range(rng)*tv.E,-3000};
  FCanonicalNavigationTrackerV2 ltr,ftr;auto rr=Raw(frame,ln,lv,t,s),rf=Raw(frame,fn,{lv.X+5*tv.N,lv.Y+5*tv.E,0},t,s);rr.WindNEDFps={wind(rng)/Ft,wind(rng)/Ft};rf.WindNEDFps=rr.WindNEDFps;
  auto lc=CanonicalNavigationAdapterV2::Convert(rr,frame,1,ltr),fc=CanonicalNavigationAdapterV2::Convert(rf,frame,1,ftr);auto sl=FormationSlotGeneratorV2::Calculate(lc,Cmd(t),t);auto pi=PlannerV2InputAdapter::Build({fc,sl,t,.02});FormationPlannerV2 p;p.Config.MaxCaptureTimeS=600;p.Config.MaxPathLengthM=200000;FormationPlannerV2Diagnostics pd{};auto pp=pi.bValid?p.Update(pi.Input,pd):FormationPlannerV2Output{};auto oo=PlannerV2OutputAdapter::Build(pp,sl,fc);
  const bool badCanonical=!lc.bPositionValid||!fc.bPositionValid,badSlot=!sl.bValid,badInput=!pi.bValid,badPath=!pp.bPathValid,badOutput=!oo.Npfg.bValid||!oo.Tecs.bCommandReady||!oo.Npfg.PathPositionNE_m.IsFinite()||!oo.Npfg.PathUnitTangentNE.IsFinite();
  canonicalFail+=badCanonical;slotFail+=badSlot;inputFail+=badInput;pathFail+=badPath;outputFail+=badOutput;if(badCanonical||badSlot||badInput||badPath||badOutput)randomFail++;}
 fail+=randomFail;std::cout<<"NAV_INTEGRATION_V2 checks="<<checks<<" seed="<<seed<<" random_cases="<<cases<<" random_failures="<<randomFail<<" stage_failures=canonical:"<<canonicalFail<<",slot:"<<slotFail<<",input:"<<inputFail<<",path:"<<pathFail<<",output:"<<outputFail<<" failures="<<fail<<'\n';return fail?1:0;}
