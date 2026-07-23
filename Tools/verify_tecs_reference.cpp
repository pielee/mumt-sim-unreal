// DEPRECATED / NON-AUTHORITATIVE (Phase 2A-R audit): reference side of the single-combined-binary
// harness (COMDAT symbol-folding risk). Authoritative independent harness: Tools/equiv/**.
#include <lib/tecs/TECS.hpp>
#include <cstdint>
uint64_t px4_equivalence_time_us=0;
struct TecsTrace {uint64_t us;float pitch,alt,alt_sp,eas_sp,eas,ratio,tmin,tmax,ttrim,pmin,pmax,climb,sink,accel,hgt_rate,hgt_rate_sp;int action;};
static TECS *controller=nullptr;
extern "C" void tecs_reference(const TecsTrace*x,float*out){
 if(!controller||(x->action&1)){delete controller;controller=new TECS();controller->enable_airspeed(true);controller->set_equivalent_airspeed_min(40.f);controller->set_equivalent_airspeed_max(60.f);controller->set_fast_descend_altitude_error(20.f);controller->set_detect_underspeed_enabled(true);}
 if(x->action&2)controller->resetIntegrals();if(x->action&4)controller->handle_alt_step(x->alt,x->hgt_rate);
 px4_equivalence_time_us=x->us;
 controller->update(x->pitch,x->alt,x->alt_sp,x->eas_sp,x->eas,x->ratio,x->tmin,x->tmax,x->ttrim,x->pmin,x->pmax,x->climb,x->sink,x->accel,x->hgt_rate,x->hgt_rate_sp);
 const auto &d=controller->getStatus();const float v[]={controller->get_pitch_setpoint(),controller->get_throttle_setpoint(),controller->get_underspeed_ratio(),d.true_airspeed_filtered,d.true_airspeed_derivative,d.control.total_energy_rate_sp,d.control.total_energy_rate_estimate,d.control.energy_balance_rate_sp,d.control.energy_balance_rate_estimate,d.control.pitch_integrator,d.control.throttle_integrator,d.fast_descend,float(controller->timestamp())};
 for(int i=0;i<13;i++)out[i]=v[i];
}
