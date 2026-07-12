#include "FormationControlV2/PlannerV2Adapters.h"

namespace FormationControlV2 {
FPlannerV2InputAdapterResult PlannerV2InputAdapter::Build(const FPlannerV2InputAdapterRequest&r,const FPlannerV2InputAdapterConfig&c){
 FPlannerV2InputAdapterResult x{};const double fa=r.CurrentSimulationTimeS-r.Follower.SimulationTimeS;
 if(!Finite(r.CurrentSimulationTimeS)||!Finite(r.DtS)||r.DtS<0||!Finite(c.MaxFollowerAgeS)||!Finite(c.MaxSlotAgeS)||fa<0||fa>c.MaxFollowerAgeS
 ||!r.Follower.bPositionValid||!r.Follower.bGroundVelocityValid||!r.Follower.bGroundCourseValid||!r.Follower.bSimulationTimeValid||!r.Follower.bOriginValid
 ||!r.Slot.bValid||!r.Slot.bPositionValid||!r.Slot.bGroundVelocityValid||!r.Slot.bGroundCourseValid||!r.Slot.bTimestampValid
 ||r.Slot.StateAgeS<0||r.Slot.StateAgeS>c.MaxSlotAgeS||r.Follower.OriginGeneration!=r.Slot.OriginGeneration)return x;
 auto&i=x.Input;i.FollowerPositionNE=r.Follower.PositionNE_m;i.FollowerGroundVelocityNE=r.Follower.GroundVelocityNE_mps;i.FollowerCourseRad=r.Follower.GroundCourse_rad;
 i.Slot.Pose={r.Slot.PositionNE_m,r.Slot.GroundCourse_rad};i.Slot.GroundVelocityNE=r.Slot.GroundVelocityNE_mps;i.Slot.GroundSpeedMps=r.Slot.GroundVelocityNE_mps.Norm();i.Slot.CurvaturePerM=r.Slot.Curvature_per_m;i.Slot.bCurvatureValid=r.Slot.bCurvatureValid;i.Slot.bValid=true;
 i.WindVelocityNE=r.Follower.WindNE_mps;i.EasToTasRatio=r.Follower.EasToTasRatio;i.EquivalentAirspeedMps=r.Follower.EquivalentAirspeed_mps;i.SimulationTimeS=r.CurrentSimulationTimeS;i.DtS=r.DtS;i.ResetGeneration=r.Follower.ResetGeneration;
 i.bFollowerValid=true;i.bCourseValid=true;i.bWindValid=r.Follower.bWindValid;i.bRatioValid=r.Follower.bRatioValid;i.bPaused=r.Follower.bPaused;x.bValid=true;return x;
}
FPlannerV2OutputAdapterResult PlannerV2OutputAdapter::Build(const FormationPlannerV2Output&p,const FFormationSlotStateV2&s,const FCanonicalNavigationStateV2&f){
 FPlannerV2OutputAdapterResult o{};if(!p.bValid||!p.bPathValid||!p.bTargetEasValid||!p.Path.bValid||!s.bAltitudeValid||!f.bEasValid||!f.bRatioValid)return o;
 o.Npfg={p.Path.Position,p.Path.UnitTangent,p.Path.SignedCurvaturePerM,true};o.Tecs.TargetEasMps=p.TargetEasMps;o.Tecs.TargetAltitudeAslM=s.AltitudeAsl_m;o.Tecs.TargetClimbRateMps=s.ClimbRate_mps;o.Tecs.bTargetEasValid=true;o.Tecs.bTargetAltitudeValid=true;o.Tecs.bTargetClimbRateValid=s.bClimbRateValid;o.Tecs.bCommandReady=true;return o;
}
} // namespace FormationControlV2
