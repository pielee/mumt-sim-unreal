#include "FormationControlV2/CanonicalNavigationAdapterV2.h"

#include <cmath>

namespace FormationControlV2 {
namespace { constexpr double FtToM=0.3048, KtToMps=0.514444444444444; }

FCanonicalNavigationStateV2 CanonicalNavigationAdapterV2::Convert(const FNavigationRawSnapshotV2 &r,
    const MissionNavigationFrameV2 &frame, std::uint32_t resetGeneration,
    FCanonicalNavigationTrackerV2 &tracker, const FCanonicalNavigationAdapterV2Config &c)
{
    FCanonicalNavigationStateV2 o{};
    const bool configValid=Finite(c.MinimumGroundSpeedMps)&&c.MinimumGroundSpeedMps>=0&&Finite(c.MinimumEasMps)&&c.MinimumEasMps>0
        &&Finite(c.MinimumDerivativeDtS)&&Finite(c.MaximumDerivativeDtS)&&c.MinimumDerivativeDtS>0&&c.MaximumDerivativeDtS>=c.MinimumDerivativeDtS;
    if(!r.bValidFrame||!frame.IsValid()||!configValid||!r.VehicleCgEcefFt.IsFinite()||!r.EcefVelocityFps.IsFinite()
       ||!Finite(r.SimulationTimeS)||r.SimulationTimeS<0){tracker.Reset();return o;}
    const auto p=frame.EcefPositionToMissionNedM({r.VehicleCgEcefFt.X*FtToM,r.VehicleCgEcefFt.Y*FtToM,r.VehicleCgEcefFt.Z*FtToM});
    const auto v=frame.EcefVelocityToMissionNedMps({r.EcefVelocityFps.X*FtToM,r.EcefVelocityFps.Y*FtToM,r.EcefVelocityFps.Z*FtToM});
    if(!p.bValid||!v.bValid){tracker.Reset();return o;}
    o.PositionNE_m={p.Ned.X,p.Ned.Y};o.GroundVelocityNE_mps={v.Ned.X,v.Ned.Y};o.bPositionValid=o.bGroundVelocityValid=true;
    o.SimulationTimeS=r.SimulationTimeS;o.bSimulationTimeValid=true;o.ResetGeneration=resetGeneration;o.OriginGeneration=frame.GetOriginGeneration();o.bOriginValid=true;o.bPaused=r.bHolding;
    const double gs=o.GroundVelocityNE_mps.Norm();
    if(gs>=c.MinimumGroundSpeedMps){o.GroundCourse_rad=WrapPi(std::atan2(o.GroundVelocityNE_mps.E,o.GroundVelocityNE_mps.N));o.bGroundCourseValid=true;}
    o.AltitudeAsl_m=r.AltitudeAslFt*FtToM;o.ClimbRate_mps=r.ClimbRateFps*FtToM;
    o.bAltitudeValid=Finite(o.AltitudeAsl_m);o.bClimbRateValid=Finite(o.ClimbRate_mps);
    o.EquivalentAirspeed_mps=r.EquivalentAirspeedKts*KtToMps;o.TrueAirspeed_mps=r.TrueAirspeedFps*FtToM;
    o.bEasValid=Finite(o.EquivalentAirspeed_mps)&&o.EquivalentAirspeed_mps>=0;o.bTasValid=Finite(o.TrueAirspeed_mps)&&o.TrueAirspeed_mps>=0;
    o.WindNE_mps={r.WindNEDFps.N*FtToM,r.WindNEDFps.E*FtToM};o.bWindValid=o.WindNE_mps.IsFinite();
    if(o.bEasValid&&o.bTasValid&&o.EquivalentAirspeed_mps>=c.MinimumEasMps){o.EasToTasRatio=o.TrueAirspeed_mps/o.EquivalentAirspeed_mps;o.bRatioValid=Finite(o.EasToTasRatio)&&o.EasToTasRatio>0;if(!o.bRatioValid)o.EasToTasRatio=1;}
    const double dt=r.SimulationTimeS-tracker.PreviousSimulationTimeS;
    if(o.bGroundCourseValid&&tracker.bPreviousCourseValid&&!r.bHolding&&resetGeneration==tracker.PreviousResetGeneration
       &&o.OriginGeneration==tracker.PreviousOriginGeneration&&dt>=c.MinimumDerivativeDtS&&dt<=c.MaximumDerivativeDtS){
        o.CourseRate_radps=WrapPi(o.GroundCourse_rad-tracker.PreviousCourseRad)/dt;o.bCourseRateValid=Finite(o.CourseRate_radps);
        if(o.bCourseRateValid&&gs>=c.MinimumGroundSpeedMps){o.Curvature_per_m=o.CourseRate_radps/gs;o.bCurvatureValid=Finite(o.Curvature_per_m);}
    }
    if(o.bGroundCourseValid&&!r.bHolding){tracker.PreviousCourseRad=o.GroundCourse_rad;tracker.PreviousSimulationTimeS=r.SimulationTimeS;tracker.PreviousResetGeneration=resetGeneration;tracker.PreviousOriginGeneration=o.OriginGeneration;tracker.bPreviousCourseValid=true;}
    else if(r.bHolding) tracker.bPreviousCourseValid=false;
    return o;
}
} // namespace FormationControlV2
