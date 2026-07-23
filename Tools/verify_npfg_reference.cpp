// DEPRECATED / NON-AUTHORITATIVE (Phase 2A-R audit): reference side of the single-combined-binary
// harness (COMDAT symbol-folding risk). Authoritative independent harness: Tools/equiv/**.
#include <lib/npfg/DirectionalGuidance.hpp>
#include <lib/npfg/AirspeedDirectionController.hpp>
#include <lib/npfg/CourseToAirspeedRefMapper.hpp>
struct Trace { const char *name; matrix::Vector2f p,g,w,t,q; float k,sp,tas,min_gs; };
extern "C" void npfg_reference(const Trace*x,float*out){
 DirectionalGuidance gd;AirspeedDirectionController hc;CourseToAirspeedRefMapper mp;
 const auto d=gd.guideToPath(x->p,x->g,x->w,x->t,x->q,x->k);
 const float hdg=mp.mapCourseSetpointToHeadingSetpoint(d.course_setpoint,x->w,x->sp);
 const float current=atan2f(x->g(1)-x->w(1),x->g(0)-x->w(0));
 const float fb=hc.controlHeading(hdg,current,x->tas);
 const float values[]={d.course_setpoint,d.lateral_acceleration_feedforward,fb,fb+d.lateral_acceleration_feedforward,
  gd.getSignedTrackError(),gd.getTrackErrorBound(),gd.getBearingFeasibility(),gd.getAdaptedPeriod(),hdg,
  mp.getMinAirspeedForCurrentBearing(d.course_setpoint,x->w,x->sp,x->min_gs)};
 for(int i=0;i<10;i++)out[i]=values[i];
}
