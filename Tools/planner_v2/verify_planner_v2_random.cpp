#include "FormationControlV2/DubinsPath.h"
#include "FormationControlV2/MovingSlotPredictor.h"
#include "FormationControlV2/CaptureSpeedPlanner.h"
#include <array>
#include <cstdio>
#include <random>

using namespace FormationControlV2;
static const char*TN(DubinsType t){const char*n[]={"LSL","RSR","LSR","RSL","RLR","LRL","INVALID"};return n[(int)t];}
int main(){constexpr unsigned Seed=0x5A17D2B5u;constexpr int Count=2500;std::mt19937 rng(Seed);std::uniform_real_distribution<double> pos(-10000,10000),ang(-Pi,Pi),rad(50,5000);int fail=0;auto check=[&](bool ok,const char*inv){if(!ok){std::printf("FAIL seed=%u case=dedicated invariant=%s\n",Seed,inv);fail++;}};std::array<int,6> coverage{};double maxPe=0,maxHe=0,maxNorm=0,maxRatio=0;double previousProgress=0;
 for(int c=0;c<Count;c++){Pose2 s{{pos(rng),pos(rng)},ang(rng)},g{{pos(rng),pos(rng)},ang(rng)};double r=rad(rng);DubinsPath p;bool ok=p.Build(s,g,r);if(ok){coverage[(int)p.GetType()]++;for(int i=0;i<=200;i++){auto x=p.Sample(p.GetLength()*i/200.);ok&=x.bValid;maxNorm=std::max(maxNorm,std::abs(x.UnitTangent.Norm()-1));maxRatio=std::max(maxRatio,std::abs(x.SignedCurvaturePerM)*r);}auto e=p.Sample(p.GetLength());double pe=(e.Position-g.Position).Norm(),he=std::abs(WrapPi(std::atan2(e.UnitTangent.E,e.UnitTangent.N)-g.CourseRad));maxPe=std::max(maxPe,pe);maxHe=std::max(maxHe,he);ok&=pe<1e-5&&he<1e-8;
   previousProgress=0;for(int j=0;j<20&&ok;j++){double target=p.GetLength()*(j+1)/20.;auto truth=p.Sample(target);auto pr=p.Project(truth.Position+Vec2{3,-2},previousProgress,10,std::max(100.,p.GetLength()/10.),true,100);ok&=pr.bValid&&pr.S+1e-12>=previousProgress;previousProgress=pr.S;}
  }if(!ok){fail++;const auto sg=p.GetSegments();std::printf("FAIL seed=%u case=%d start=(%.9g,%.9g,%.9g) terminal=(%.9g,%.9g,%.9g) Rmin=%.9g type=%s seg=(%.9g,%.9g,%.9g) ProgressS=%.9g invariant=random_geometry\n",Seed,c,s.Position.N,s.Position.E,s.CourseRad,g.Position.N,g.Position.E,g.CourseRad,r,TN(p.GetType()),sg[0].LengthM,sg[1].LengthM,sg[2].LengthM,previousProgress);}
 }
 // Moving-slot exact straight and constant-curvature predictions.
 PredictionConfig pc;MovingSlotState straight{{{10,20},0.3},FromCourse(.3)*100,100,0,true,true};auto ps=MovingSlotPredictor::PredictAtTime(straight,2,pc);check(ps.bValid&&(ps.TerminalPose.Position-(straight.Pose.Position+straight.GroundVelocityNE*2)).Norm()<=1e-9,"straight_prediction");
 MovingSlotState curve{{{0,0},0},FromCourse(0)*120,120,1./1000,true,true};auto pcv=MovingSlotPredictor::PredictAtTime(curve,3,pc);const double w=.12,expectedN=120/w*std::sin(w*3),expectedE=120/w*(1-std::cos(w*3));check(pcv.bValid&&(pcv.TerminalPose.Position-Vec2{expectedN,expectedE}).Norm()<=1e-9&&std::abs(WrapPi(pcv.TerminalPose.CourseRad-w*3))<=1e-12,"constant_curvature_prediction");
 PredictionConfig cc=pc;cc.MaxIterations=20;auto conv=MovingSlotPredictor::SolveCaptureTime(straight,{-1000,20},5,200,cc);check(conv.bValid&&conv.bConverged,"prediction_convergence");
 auto limitedIterations=MovingSlotPredictor::SolveCaptureTime(straight,{-1000,20},5,200,pc);check(!limitedIterations.bValid&&limitedIterations.Failure==PredictionFailure::NotConverged,"prediction_iteration_failure");
 auto cap=MovingSlotPredictor::PredictAtTime(curve,pc.PredictionTimeCapS+1,pc);check(!cap.bValid&&cap.Failure==PredictionFailure::TimeCapReached,"prediction_cap");
 MovingSlotState invalid=curve;invalid.Pose.Position.N=NAN;auto inv=MovingSlotPredictor::PredictAtTime(invalid,1,pc);check(!inv.bValid&&inv.TerminalPose.Position.N==0&&inv.TerminalPose.Position.E==0,"prediction_invalid_fresh_zero");
 // Along-only wind-aware target EAS and invalid boundary clearing.
 CaptureSpeedInput si;si.PredictedSlotGroundVelocityNE={200,0};si.PathUnitTangentNE={1,0};si.WindVelocityNE={-20,30};si.EasToTasRatio=1.2;si.DesiredCaptureClosureMps=25;si.RemainingAlongDistanceM=1000;si.SafetyDistanceM=100;si.ConfiguredDecelerationMps2=2;si.MaxClosureSpeedMps=40;si.MinTargetEasMps=50;si.MaxTargetEasMps=350;si.bWindValid=si.bRatioValid=si.bDecelerationEnvelopeValid=true;auto so=CaptureSpeedPlanner::Compute(si);const Vec2 expectedGround{225,0},expectedAir{245,-30};check(so.bTargetEasValid&&(so.DesiredGroundVelocityNE-expectedGround).Norm()<=1e-12&&(so.DesiredAirVelocityNE-expectedAir).Norm()<=1e-12&&std::abs(so.TargetEasMps-expectedAir.Norm()/1.2)<=1e-12,"wind_aware_target_eas");
 si.bWindValid=false;auto noWind=CaptureSpeedPlanner::Compute(si);check(!noWind.bTargetEasValid&&noWind.TargetEasMps==0&&noWind.DesiredGroundVelocityNE.N==0,"invalid_wind_fresh_zero");si.bWindValid=true;si.bRatioValid=false;auto noRatio=CaptureSpeedPlanner::Compute(si);check(!noRatio.bTargetEasValid&&noRatio.TargetEasMps==0,"invalid_ratio_fresh_zero");
 // NaN/Inf and fresh-zero invalid outputs, with no stale data retained between calls.
 si.bRatioValid=true;si.WindVelocityNE.N=INFINITY;auto nonfinite=CaptureSpeedPlanner::Compute(si);check(!nonfinite.bTargetEasValid&&nonfinite.TargetEasMps==0&&nonfinite.TargetTasMps==0,"nonfinite_fresh_zero");
 std::printf("RANDOM_AUDIT seed=%u cases=%d failures=%d max_endpoint_pos=%.12g max_endpoint_hdg=%.12g max_tangent_norm_error=%.12g max_curvature_ratio=%.12g coverage=LSL:%d,RSR:%d,LSR:%d,RSL:%d,RLR:%d,LRL:%d\n",Seed,Count,fail,maxPe,maxHe,maxNorm,maxRatio,coverage[0],coverage[1],coverage[2],coverage[3],coverage[4],coverage[5]);
 for(int i=0;i<6;i++){if(coverage[i]==0){std::printf("FAIL seed=%u invariant=type_coverage type=%s\n",Seed,TN((DubinsType)i));fail++;}}
 return fail?1:0;}
