#include "FormationControlV2/DubinsPath.h"
#include <array>
#include <cstdio>
#include <limits>
#include <string>

using namespace FormationControlV2;
static int Failures=0;
static void Check(bool ok,const char*name){if(!ok){std::printf("FAIL invariant=%s\n",name);Failures++;}}
static const char*Name(DubinsType t){const char*n[]={"LSL","RSR","LSR","RSL","RLR","LRL","INVALID"};return n[(int)t];}

int main(){
 const double r=100;const Pose2 start{{0,0},0};const DubinsType types[]={DubinsType::LSL,DubinsType::RSR,DubinsType::LSR,DubinsType::RSL,DubinsType::RLR,DubinsType::LRL};
 for(DubinsType type:types){bool found=false;DubinsPath path;Pose2 goal{};
  for(double n:{-300.,-150.,0.,150.,300.})for(double e:{-300.,-150.,0.,150.,300.})for(int h=0;h<8&&!found;h++){goal={{n,e},-Pi+h*Pi/4};if(path.BuildType(start,goal,r,type))found=true;}
  Check(found,(std::string("canonical_")+Name(type)).c_str());if(!found)continue;
  const auto end=path.Sample(path.GetLength());const double pe=(end.Position-goal.Position).Norm(),he=std::abs(WrapPi(std::atan2(end.UnitTangent.E,end.UnitTangent.N)-goal.CourseRad));
  Check(pe<1e-6,"endpoint_position");Check(he<1e-8,"endpoint_heading");
  double maxK=0,maxNorm=0;for(int i=0;i<=2000;i++){auto s=path.Sample(path.GetLength()*i/2000.);Check(s.bValid,"sample_valid");maxK=std::max(maxK,std::abs(s.SignedCurvaturePerM));maxNorm=std::max(maxNorm,std::abs(s.UnitTangent.Norm()-1));}
  Check(maxK<=1/r+1e-12,"curvature_bound");Check(maxNorm<1e-12,"tangent_norm");
  for(double j:path.GetSegmentJunctions()){const double eps=1e-6;auto a=path.Sample(std::max(0.,j-eps)),b=path.Sample(std::min(path.GetLength(),j+eps));Check((a.Position-b.Position).Norm()<3e-6,"junction_position_G1");Check(std::abs(WrapPi(std::atan2(a.UnitTangent.E,a.UnitTangent.N)-std::atan2(b.UnitTangent.E,b.UnitTangent.N)))<3e-8,"junction_tangent_G1");}
  const auto seg=path.GetSegments();std::printf("CANONICAL type=%s goal=(%.0f,%.0f,%.3f) length=%.9f seg=(%.9f,%.9f,%.9f) pos_err=%.3g hdg_err=%.3g max_k=%.9g\n",Name(type),goal.Position.N,goal.Position.E,goal.CourseRad,path.GetLength(),seg[0].LengthM,seg[1].LengthM,seg[2].LengthM,pe,he,maxK);
 }
 DubinsPath straight;Check(straight.Build(start,{{1000,0},0},r),"zero_length_build");auto z=straight.GetSegments();Check(z[0].LengthM<1e-9&&z[2].LengthM<1e-9,"zero_length_segments");
 auto p=straight.Project({450,25},0,0,1000,true);Check(p.bValid&&std::abs(p.S-450)<1e-9&&std::abs(p.DistanceM-25)<1e-9,"analytic_line_projection");
 auto limited=straight.Project({900,0},100,10,150,true);Check(limited.bValid&&limited.S<=250+1e-9,"projection_window");
 auto monotonic=straight.Project({50,0},500,500,100,true);Check(monotonic.bValid&&monotonic.S>=500,"projection_monotonic");
 ProjectionWindowConfig wc;bool windowValid=false;const double fw=DubinsPath::ComputeForwardSearchWindow(240,0.2,wc,windowValid);Check(windowValid&&fw>=240*.2*wc.AdvanceFactor,"groundspeed_dt_projection_window");
 DubinsPath::ComputeForwardSearchWindow(240,1.0,wc,windowValid);Check(!windowValid,"abnormal_dt_projection_rejected");
 DubinsPath bad;Check(!bad.Build(start,{{NAN,0},0},r),"nan_rejected");Check(!bad.Build(start,{{0,0},INFINITY},r),"inf_rejected");Check(!bad.Build(start,{{1,1},0},0),"zero_radius_rejected");
 DubinsPath stale;Check(stale.Build(start,{{1000,0},0},r),"stale_setup");Check(!stale.Build(start,{{NAN,0},0},r)&&!stale.IsValid()&&stale.GetLength()==0&&!stale.Sample(0).bValid,"invalid_build_clears_stale_path");
 std::printf("DUBINS_AUDIT failures=%d\n",Failures);return Failures?1:0;
}
