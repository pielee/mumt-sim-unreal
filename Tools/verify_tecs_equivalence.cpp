// DEPRECATED / NON-AUTHORITATIVE (Phase 2A-R audit): single-combined-binary harness with a
// COMDAT symbol-folding risk (PX4 reference + MUMT port share same-named matrix::/math:: inline
// symbols in one executable). Authoritative independent harness: Tools/equiv/**. Kept only for
// historical comparison.
#include "FormationControl/Px4TecsAdapter.h"
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>
struct TecsTrace {uint64_t us;float pitch,alt,alt_sp,eas_sp,eas,ratio,tmin,tmax,ttrim,pmin,pmax,climb,sink,accel,hgt_rate,hgt_rate_sp;int action;};
extern "C" void tecs_reference(const TecsTrace*,float*);
static float error(float a,float b){if(std::isnan(a)&&std::isnan(b))return 0;if(!std::isfinite(a)||!std::isfinite(b))return a==b?0:INFINITY;return fabsf(a-b);}
int main(){const float N=std::numeric_limits<float>::quiet_NaN();std::vector<TecsTrace> traces;uint64_t t=1000000;
 auto add=[&](float alt,float asp,float eas,float hr,float accel=0,float hsp=NAN,int action=0,uint64_t step=20000){t+=step;traces.push_back({t,0.03f,alt,asp,35,eas,1.12f,0,1,.5f,-.45f,.45f,8,5,accel,hr,hsp,action});};
 add(1000,1000,35,0,0,N,1,0);for(int i=0;i<20;i++)add(1000,1000,35,0); // hold
 add(1000,1080,35,0);for(int i=0;i<20;i++)add(1000+i,1080,35,2); // altitude step/climb
 add(1030,980,35,-3);for(int i=0;i<15;i++)add(1030-i,980,35,-3); // sink/fast descend
 for(int i=0;i<150;i++)add(1000,1000,0,0);add(1000,1000,55,0); // underspeed and speed step
 add(1000,1100,N,0); // invalid EAS
 add(1000,1100,25,0,0,N,2); // integrator reset
 add(950,1100,25,0,0,N,4); // estimator altitude step
 add(950,1100,25,0,0,N,0,500); // dt < .001 hold
 add(950,1100,25,0,0,N,0,1500000); // dt > 1 initialize
 add(950,1100,25,0,0,N,0,0); // pause
 add(950,1100,25,0,0,N,1,20000); // restart
 auto make_port=[](){auto p=std::make_unique<MumtPx4::FPx4TecsAdapter>();p->controller().enable_airspeed(true);p->controller().set_equivalent_airspeed_min(40.f);p->controller().set_equivalent_airspeed_max(60.f);p->controller().set_fast_descend_altitude_error(20.f);p->controller().set_detect_underspeed_enabled(true);return p;};
 auto port=make_port();float mx[13]{};int fail=0;bool saw_underspeed=false,saw_fast_descend=false;
 for(size_t n=0;n<traces.size();n++){const auto &x=traces[n];if(x.action&1)port=make_port();if(x.action&2)port->controller().resetIntegrals();if(x.action&4)port->controller().handle_alt_step(x.alt,x.hgt_rate);
  float ref[13];tecs_reference(&x,ref);MumtPx4::TecsInput in{x.us,x.pitch,x.alt,x.alt_sp,x.eas_sp,x.eas,x.ratio,x.tmin,x.tmax,x.ttrim,x.pmin,x.pmax,x.climb,x.sink,x.accel,x.hgt_rate,x.hgt_rate_sp};const auto o=port->update(in);
  const float got[]={o.pitch_setpoint,o.throttle_setpoint,o.underspeed_ratio,o.filtered_tas,o.filtered_tas_rate,o.total_energy_rate_sp,o.total_energy_rate_estimate,o.energy_balance_rate_sp,o.energy_balance_rate_estimate,o.pitch_integrator,o.throttle_integrator,o.fast_descend,float(o.state_timestamp_us)};
  saw_underspeed|=ref[2]>0.f;saw_fast_descend|=ref[11]>0.f;
  const float tol[]={1e-5f,1e-5f,1e-5f,1e-5f,1e-5f,1e-4f,1e-4f,1e-4f,1e-4f,1e-6f,1e-6f,0,0};for(int i=0;i<13;i++){float e=error(ref[i],got[i]);mx[i]=fmaxf(mx[i],e);if(e>tol[i]){fail++;printf("FAIL trace=%zu output=%d ref=%g port=%g err=%g\n",n,i,ref[i],got[i],e);}}
 }
 if(!saw_underspeed){puts("FAIL coverage: underspeed never engaged");fail++;}if(!saw_fast_descend){puts("FAIL coverage: fast descend never engaged");fail++;}
 const char*n[]={"pitch","throttle","underspeed","filtered_tas","tas_rate","te_rate_sp","te_rate_est","eb_rate_sp","eb_rate_est","pitch_i","throttle_i","fast_descend","timestamp"};for(int i=0;i<13;i++)printf("MAX %-14s %.9g\n",n[i],mx[i]);printf("TECS traces=%zu failures=%d underspeed=%d fast_descend=%d\n",traces.size(),fail,saw_underspeed,saw_fast_descend);return fail?1:0;}
