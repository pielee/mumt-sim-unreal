// DEPRECATED / NON-AUTHORITATIVE (Phase 2A-R audit): single-combined-binary harness with a
// COMDAT symbol-folding risk (PX4 reference + MUMT port share same-named matrix::/math:: inline
// symbols in one executable). Authoritative independent harness: Tools/equiv/**. Kept only for
// historical comparison.
#include "FormationControl/Px4NpfgAdapter.h"
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

struct Trace { const char *name; matrix::Vector2f p,g,w,t,q; float k,sp,tas,min_gs; };
extern "C" void npfg_reference(const Trace*,float*);
static float diff(float a,float b,bool angle=false){if(std::isnan(a)&&std::isnan(b))return 0; if(!std::isfinite(a)||!std::isfinite(b))return a==b?0:INFINITY; float d=fabsf(a-b);if(angle)d=fabsf(matrix::wrap_pi(a-b));return d;}
int main(){
 const float nan=std::numeric_limits<float>::quiet_NaN(), inf=std::numeric_limits<float>::infinity();
 std::vector<Trace> v={
  {"straight",{0,10},{40,0},{0,0},{1,0},{0,0},0,40,40,5},
  {"right_curve",{0,50},{45,2},{4,-3},{1,0},{0,0},.003f,45,41,5},
  {"left_curve",{0,-50},{45,-2},{-4,3},{1,0},{0,0},-.003f,45,41,5},
  {"low_speed",{20,5},{.01f,.01f},{8,0},{0,1},{0,0},0,25,8,5},
  {"headwind",{5,20},{30,0},{-28,0},{1,0},{0,0},0,35,58,5},
  {"tailwind",{-5,-20},{30,0},{28,0},{1,0},{0,0},0,35,2,5},
  {"crosswind",{30,5},{20,0},{0,35},{1,0},{0,0},0,25,40,5},
  {"wrap",{10,1},{-30,.01f},{0,0},{-1,-.00001f},{0,0},0,30,30,5},
  {"degenerate",{0,0},{0,0},{0,0},{0,0},{0,0},0,20,0,0},
  {"nan",{nan,0},{20,0},{0,0},{1,0},{0,0},0,20,20,5},
  {"inf",{0,0},{inf,0},{0,0},{1,0},{0,0},0,20,20,5}};
 float mx[10]{}; int failures=0;
 for(const auto &x:v){
  float ref[10]; npfg_reference(&x,ref);
  MumtPx4::FPx4NpfgAdapter port; MumtPx4::NpfgInput in{x.p,x.g,x.w,x.t,x.q,x.k,x.sp,x.tas,x.min_gs};const auto o=port.update(in);
  const float got[]={o.course_setpoint,o.lateral_acceleration_feedforward,o.lateral_acceleration_feedback,o.lateral_acceleration_total,o.track_error,o.track_error_bound,o.wind_feasibility,o.adapted_period,o.airspeed_direction,o.minimum_required_airspeed};
  const float tol[]={1e-6f,1e-5f,1e-5f,1e-5f,1e-5f,1e-5f,1e-6f,1e-6f,1e-6f,1e-5f};
  for(int i=0;i<10;i++){float e=diff(ref[i],got[i],i==0||i==8);mx[i]=fmaxf(mx[i],e);if(e>tol[i]){failures++;std::printf("FAIL %s output=%d ref=%g port=%g err=%g\n",x.name,i,ref[i],got[i],e);}}
 }
 const char*n[]={"course","lat_ff","lat_fb","lat_total","track_error","track_bound","feasibility","period","air_direction","min_airspeed"};for(int i=0;i<10;i++)std::printf("MAX %-14s %.9g\n",n[i],mx[i]);
 std::printf("NPFG traces=%zu failures=%d\n",v.size(),failures);return failures?1:0;
}
