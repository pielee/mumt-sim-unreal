#include "FormationControlV2/CaptureSpeedPlanner.h"
#include <algorithm>

namespace FormationControlV2 {
CaptureSpeedOutput CaptureSpeedPlanner::Compute(const CaptureSpeedInput&i){
    CaptureSpeedOutput o;
    const bool common=i.PredictedSlotGroundVelocityNE.IsFinite()&&i.PathUnitTangentNE.IsFinite()&&
        Finite(i.DesiredCaptureClosureMps)&&i.DesiredCaptureClosureMps>=0&&Finite(i.RemainingAlongDistanceM)&&
        Finite(i.SafetyDistanceM)&&i.SafetyDistanceM>=0&&Finite(i.MaxClosureSpeedMps)&&i.MaxClosureSpeedMps>=0&&
        Finite(i.MinTargetEasMps)&&Finite(i.MaxTargetEasMps)&&i.MinTargetEasMps>=0&&i.MaxTargetEasMps>=i.MinTargetEasMps;
    const Vec2 tangent=i.PathUnitTangentNE.Normalized();if(!common||tangent.Norm()<0.999999){o.Failure=SpeedPlanFailure::InvalidInput;return o;}
    if(!i.bWindValid||!i.WindVelocityNE.IsFinite()){o.Failure=SpeedPlanFailure::InvalidWind;return o;}
    if(!i.bRatioValid||!Finite(i.EasToTasRatio)||i.EasToTasRatio<=0){o.Failure=SpeedPlanFailure::InvalidEasToTasRatio;return o;}
    if(!i.bDecelerationEnvelopeValid||!Finite(i.ConfiguredDecelerationMps2)||i.ConfiguredDecelerationMps2<=0){o.Failure=SpeedPlanFailure::InvalidDecelerationEnvelope;return o;}
    const double remaining=std::max(0.0,i.RemainingAlongDistanceM-i.SafetyDistanceM);
    const double allowed=std::sqrt(2*i.ConfiguredDecelerationMps2*remaining);
    o.CommandedClosureMps=std::min({i.DesiredCaptureClosureMps,i.MaxClosureSpeedMps,allowed});
    o.DesiredGroundVelocityNE=i.PredictedSlotGroundVelocityNE+tangent*o.CommandedClosureMps;
    o.DesiredAirVelocityNE=o.DesiredGroundVelocityNE-i.WindVelocityNE;o.TargetTasMps=o.DesiredAirVelocityNE.Norm();
    o.TargetEasMps=Clamp(o.TargetTasMps/i.EasToTasRatio,i.MinTargetEasMps,i.MaxTargetEasMps);
    o.bTargetEasValid=Finite(o.TargetTasMps)&&Finite(o.TargetEasMps);o.Failure=o.bTargetEasValid?SpeedPlanFailure::None:SpeedPlanFailure::InvalidInput;return o;
}
}
