#include "FormationControlV2/DubinsPath.h"
#include "FormationControlV2/MovingSlotPredictor.h"
#include <cstdio>

using namespace FormationControlV2;
static const char*TypeName(DubinsType t){const char*n[]={"LSL","RSR","LSR","RSL","RLR","LRL","INVALID"};return n[(int)t];}
int main(){const double speeds[]={120,180,240},banks[]={30,45,60},headings[]={90,180},sides[]={-1,1},slotK[]={-1./3000,1./3000};int cases=0,fail=0;double worst=0,maxPos=0,maxHdg=0;
 for(double speed:speeds)for(double bank:banks)for(double hdg:headings)for(double side:sides)for(double k:slotK){cases++;
  char id[160];std::snprintf(id,sizeof(id),"BASE_SIDE_%c_HDG_%03.0f_D_2773_V_%.0f_B_%.0f_SK_%c",side<0?'L':'R',hdg,speed,bank,k<0?'L':'R');
  const Pose2 start{{-2500,side*1200},hdg*Pi/180};MovingSlotState slot{{{0,0},0},{220,0},220,k,true,true};PredictionConfig pc;const double initial=(slot.Pose.Position-start.Position).Norm()/std::max(30.,speed);
  auto pred=MovingSlotPredictor::PredictAtTime(slot,std::min(initial,pc.PredictionTimeCapS),pc);
  const double rmin=std::max(100.,1.25*speed*speed/(GravityMps2*std::tan(bank*Pi/180)));
  Pose2 terminal=pred.TerminalPose;terminal.Position=terminal.Position-FromCourse(terminal.CourseRad)*std::max(500.,rmin);
  DubinsPath path;bool ok=pred.bValid&&path.Build(start,terminal,rmin);double maxK=0,normErr=0;
  if(ok)for(int i=0;i<=3000;i++){auto s=path.Sample(path.GetLength()*i/3000.);ok&=s.bValid;maxK=std::max(maxK,std::abs(s.SignedCurvaturePerM));normErr=std::max(normErr,std::abs(s.UnitTangent.Norm()-1));}
  const auto end=path.Sample(path.GetLength());const double pe=ok?(end.Position-terminal.Position).Norm():INFINITY,he=ok?std::abs(WrapPi(std::atan2(end.UnitTangent.E,end.UnitTangent.N)-terminal.CourseRad)):INFINITY;
  const bool pass=ok&&maxK<=1/rmin+1e-12&&normErr<1e-10&&pe<1e-5&&he<1e-8;worst=std::max(worst,maxK*rmin);maxPos=std::max(maxPos,pe);maxHdg=std::max(maxHdg,he);
  if(!pass){fail++;const auto sg=path.GetSegments();std::printf("FAIL seed=0 case=%s start=(%.9g,%.9g,%.9g) terminal=(%.9g,%.9g,%.9g) Rmin=%.9g type=%s seg=(%.9g,%.9g,%.9g) ProgressS=0 invariant=geometry pred=%d path=%d maxK=%.9g norm=%.9g pe=%.9g he=%.9g\n",id,start.Position.N,start.Position.E,start.CourseRad,terminal.Position.N,terminal.Position.E,terminal.CourseRad,rmin,TypeName(path.GetType()),sg[0].LengthM,sg[1].LengthM,sg[2].LengthM,pred.bValid,path.IsValid(),maxK,normErr,pe,he);}
 }
 std::printf("V2_72 same_scenario_matrix=%d pass=%d fail=%d worst_curvature_ratio=%.12g max_endpoint_position_error=%.12g max_endpoint_heading_error=%.12g\n",cases,cases-fail,fail,worst,maxPos,maxHdg);return fail?1:0;}
