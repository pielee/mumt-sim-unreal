#include "FormationControlV2/SlotLocalPathPrimitiveV2.h"

#include <algorithm>
#include <cmath>

namespace FormationControlV2 {
namespace {
SlotLocalPathProjectionV2 Fail(SlotLocalPathFailureV2 failure, const SlotLocalPathInputV2 &in) {
    SlotLocalPathProjectionV2 out{}; out.Failure=failure; out.Generation=in.Generation;
    if(Finite(in.SimulationTimeS))out.TimestampS=in.SimulationTimeS;
    return out;
}
Vec2 RotateRight(const Vec2 &v){return {-v.E,v.N};}
}

// Single source of truth for the turn-bound feasibility contract (see header). Pure.
SlotCurvatureFeasibilityV2 ClassifySlotCurvature(double slotCurvaturePerM, bool bSlotCurvatureValid,
                                                double followerTurnRadiusBoundM, double relTolerance) {
    SlotCurvatureFeasibilityV2 f{};
    f.SlotCurvaturePerM=slotCurvaturePerM;
    f.TurnBoundRadiusM=followerTurnRadiusBoundM;
    f.bCurvatureValid=bSlotCurvatureValid&&Finite(slotCurvaturePerM);
    if(!Finite(followerTurnRadiusBoundM)||followerTurnRadiusBoundM<=0){
        f.Class=SlotCurvatureClassV2::CurvatureUnavailable;return f;}
    f.MaxFeasibleCurvaturePerM=1.0/followerTurnRadiusBoundM;
    f.ToleranceCurvaturePerM=std::abs(relTolerance)*f.MaxFeasibleCurvaturePerM;
    if(!f.bCurvatureValid){f.Class=SlotCurvatureClassV2::CurvatureUnavailable;return f;}
    const double absK=std::abs(slotCurvaturePerM);
    f.SlotRadiusM=absK<1e-12?std::numeric_limits<double>::infinity():1.0/absK;
    // PRODUCTION PREDICATE, verbatim: a slot circle tighter than the follower's turn bound.
    // (absK < 1e-12 is the straight case, which the primitive never rejects.)
    f.bRejectedByTurnBound=absK>=1e-12&&(f.SlotRadiusM+kSlotCurvatureRadiusEpsM<followerTurnRadiusBoundM);
    if(absK<=f.MaxFeasibleCurvaturePerM-f.ToleranceCurvaturePerM)     f.Class=SlotCurvatureClassV2::Feasible;
    else if(absK> f.MaxFeasibleCurvaturePerM+f.ToleranceCurvaturePerM)f.Class=SlotCurvatureClassV2::Infeasible;
    else                                                              f.Class=SlotCurvatureClassV2::Boundary;
    return f;
}

SlotLocalPathProjectionV2 SlotLocalPathPrimitiveV2::Project(const SlotLocalPathInputV2 &in) {
    if(!in.SlotPose.IsFinite()||!in.FollowerPositionNE.IsFinite()||!Finite(in.FollowerTurnRadiusBoundM)||
       in.FollowerTurnRadiusBoundM<=0||!Finite(in.SimulationTimeS))return Fail(SlotLocalPathFailureV2::InvalidInput,in);
    SlotLocalPathProjectionV2 out{};out.Generation=in.Generation;out.TimestampS=in.SimulationTimeS;
    const Vec2 slotT=FromCourse(in.SlotPose.CourseRad),delta=in.FollowerPositionNE-in.SlotPose.Position;
    if(!in.bSlotCurvatureValid){
        if(!in.bAllowStraightAssumption)return Fail(SlotLocalPathFailureV2::CurvatureUnavailable,in);
        out.Kind=SlotLocalPathKindV2::Straight;out.bStraightAssumed=true;
        out.LocalCoordinate=delta.Dot(slotT);out.Path.Position=in.SlotPose.Position+slotT*out.LocalCoordinate;
        out.Path.UnitTangent=slotT;out.Path.SignedCurvaturePerM=0;out.Path.S=out.LocalCoordinate;out.Path.SegmentIndex=0;
    }else{
        if(!Finite(in.SlotCurvaturePerM))return Fail(SlotLocalPathFailureV2::InvalidInput,in);
        if(std::abs(in.SlotCurvaturePerM)<1e-12){out.Kind=SlotLocalPathKindV2::Straight;out.LocalCoordinate=delta.Dot(slotT);
            out.Path.Position=in.SlotPose.Position+slotT*out.LocalCoordinate;out.Path.UnitTangent=slotT;out.Path.SignedCurvaturePerM=0;out.Path.S=out.LocalCoordinate;out.Path.SegmentIndex=0;
        }else{
            const double radius=1.0/std::abs(in.SlotCurvaturePerM);
            // Turn-bound rejection goes through the shared contract, so the airborne acceptance
            // tests grade against exactly the predicate the planner enforces here.
            if(ClassifySlotCurvature(in.SlotCurvaturePerM,true,in.FollowerTurnRadiusBoundM).bRejectedByTurnBound)
                return Fail(SlotLocalPathFailureV2::CurvatureInfeasible,in);
            const Vec2 center=in.SlotPose.Position+RightNormal(in.SlotPose.CourseRad)*(1.0/in.SlotCurvaturePerM);
            const Vec2 radial=in.FollowerPositionNE-center;const double radialNorm=radial.Norm();
            if(!Finite(radialNorm)||radialNorm<std::max(1e-6,radius*1e-9))return Fail(SlotLocalPathFailureV2::NearCircleCenter,in);
            const Vec2 unitRadial=radial/radialNorm;const double turnSign=in.SlotCurvaturePerM>0?1.0:-1.0;
            out.Kind=SlotLocalPathKindV2::ConstantCurvatureCircle;out.CircleCenterNE=center;out.CircleRadiusM=radius;
            out.Path.Position=center+unitRadial*radius;out.Path.UnitTangent=RotateRight(unitRadial)*turnSign;
            out.Path.SignedCurvaturePerM=in.SlotCurvaturePerM;out.Path.SegmentIndex=0;
            const Vec2 slotRadial=(in.SlotPose.Position-center)/radius;
            const double angle=std::atan2(slotRadial.Cross(unitRadial)*turnSign,slotRadial.Dot(unitRadial));
            out.LocalCoordinate=angle*radius;out.Path.S=out.LocalCoordinate;
        }
    }
    out.SignedCrossTrackErrorM=out.Path.UnitTangent.Cross(in.FollowerPositionNE-out.Path.Position);
    out.ProjectionDistanceM=(in.FollowerPositionNE-out.Path.Position).Norm();out.Path.bValid=true;
    if(!out.Path.Position.IsFinite()||!out.Path.UnitTangent.IsFinite()||!Finite(out.Path.SignedCurvaturePerM)||
       !Finite(out.SignedCrossTrackErrorM)||!Finite(out.ProjectionDistanceM))return Fail(SlotLocalPathFailureV2::NonFiniteResult,in);
    out.bValid=true;return out;
}

NearFieldSpeedOutputV2 NearFieldSpeedPlannerV2::Compute(const NearFieldSpeedInputV2 &in){
    NearFieldSpeedOutputV2 out{};
    if(!in.SlotGroundVelocityNE.IsFinite()||!in.SlotUnitTangentNE.IsFinite()||!in.FollowerGroundVelocityNE.IsFinite()||
       !Finite(in.AlongErrorM)||!Finite(in.DtS)||in.DtS<0||!Finite(in.ClosureTimeConstantS)||in.ClosureTimeConstantS<=0||
       !Finite(in.HoldTaperDistanceM)||in.HoldTaperDistanceM<=0||!Finite(in.MaxClosureMps)||in.MaxClosureMps<0){out.Failure=NearFieldSpeedFailureV2::InvalidInput;return out;}
    if(!in.bWindValid||!in.WindVelocityNE.IsFinite()){out.Failure=NearFieldSpeedFailureV2::InvalidWind;return out;}
    if(!in.bRatioValid||!Finite(in.EasToTasRatio)||in.EasToTasRatio<=0){out.Failure=NearFieldSpeedFailureV2::InvalidRatio;return out;}
    if(!in.bEnvelopeValid||!Finite(in.AccelerationMps2)||!Finite(in.DecelerationMps2)||in.AccelerationMps2<=0||in.DecelerationMps2<=0){out.Failure=NearFieldSpeedFailureV2::InvalidEnvelope;return out;}
    const Vec2 t=in.SlotUnitTangentNE.Normalized();if(t.Norm()<0.99){out.Failure=NearFieldSpeedFailureV2::InvalidInput;return out;}
    const double slotAlong=in.SlotGroundVelocityNE.Dot(t),followerAlong=in.FollowerGroundVelocityNE.Dot(t);
    out.TaperRatio=Clamp(std::abs(in.AlongErrorM)/in.HoldTaperDistanceM,0.0,1.0);
    const double rawDelta=Clamp(-in.AlongErrorM/in.ClosureTimeConstantS,-in.MaxClosureMps,in.MaxClosureMps);
    out.ClosureDeltaMps=rawDelta*out.TaperRatio;
    const double unconstrained=std::max(0.0,slotAlong+out.ClosureDeltaMps);
    const double lo=std::max(0.0,followerAlong-in.DecelerationMps2*in.DtS),hi=followerAlong+in.AccelerationMps2*in.DtS;
    out.TargetGroundSpeedMps=Clamp(unconstrained,lo,hi);out.TargetGroundVelocityNE=t*out.TargetGroundSpeedMps;
    out.TargetAirVelocityNE=out.TargetGroundVelocityNE-in.WindVelocityNE;out.TargetTasMps=out.TargetAirVelocityNE.Norm();
    out.TargetEasMps=Clamp(out.TargetTasMps/in.EasToTasRatio,in.MinTargetEasMps,in.MaxTargetEasMps);
    if(!out.TargetGroundVelocityNE.IsFinite()||!out.TargetAirVelocityNE.IsFinite()||!Finite(out.TargetEasMps)){out={};out.Failure=NearFieldSpeedFailureV2::NonFiniteResult;return out;}
    out.bValid=true;return out;
}
} // namespace FormationControlV2
