#include "FormationControlV2/MovingSlotPredictor.h"

namespace FormationControlV2 {
PredictionResult MovingSlotPredictor::PredictAtTime(const MovingSlotState&s,double t,const PredictionConfig&c){
    PredictionResult o;if(!s.bValid||!s.Pose.IsFinite()||!s.GroundVelocityNE.IsFinite()||!Finite(s.GroundSpeedMps)||s.GroundSpeedMps<0){o.Failure=PredictionFailure::InvalidInput;return o;}
    if(!Finite(t)||t<0){o.Failure=PredictionFailure::NonFiniteTime;return o;}if(t>c.PredictionTimeCapS){o.Failure=PredictionFailure::TimeCapReached;return o;}
    double k=0;PredictionModel model=PredictionModel::Straight;
    if(s.bCurvatureValid){if(!Finite(s.CurvaturePerM)){o.Failure=PredictionFailure::InvalidKinematics;return o;}k=s.CurvaturePerM;model=std::abs(k)>1e-12?PredictionModel::ConstantCurvature:PredictionModel::Straight;}
    else {if(!c.bAllowStraightWithoutCurvature||t>c.StraightAssumptionMaxTimeS){o.Failure=PredictionFailure::InvalidKinematics;return o;}model=PredictionModel::StraightAssumedCurvatureUnavailable;}
    const double v=s.GroundSpeedMps,omega=k*v;Pose2 p=s.Pose;
    if(std::abs(omega)<=1e-12)p.Position=p.Position+s.GroundVelocityNE*t;
    else {const double next=p.CourseRad+omega*t;p.Position.N+=(v/omega)*(std::sin(next)-std::sin(p.CourseRad));p.Position.E+=(v/omega)*(std::cos(p.CourseRad)-std::cos(next));p.CourseRad=WrapPi(next);}
    o.TerminalPose=p;o.TerminalGroundVelocityNE=FromCourse(p.CourseRad)*v;o.PredictionTimeS=t;o.Iterations=1;o.Model=model;o.bConverged=true;o.bValid=p.IsFinite()&&o.TerminalGroundVelocityNE.IsFinite();if(!o.bValid)o.Failure=PredictionFailure::NonFiniteTerminal;return o;
}
PredictionResult MovingSlotPredictor::SolveCaptureTime(const MovingSlotState&s,const Vec2&follower,double initial,double captureSpeed,const PredictionConfig&c){
    PredictionResult out;if(!follower.IsFinite()||!Finite(initial)||!Finite(captureSpeed)||captureSpeed<=0||c.MaxIterations<=0){out.Failure=PredictionFailure::InvalidInput;return out;}
    double time=initial;Pose2 previous{};bool have=false;
    for(int i=1;i<=c.MaxIterations;i++){
        auto p=PredictAtTime(s,time,c);p.Iterations=i;if(!p.bValid)return p;
        const Vec2 toTerminal=p.TerminalPose.Position-follower;
        const double effective=std::max(captureSpeed,c.MinimumClosingGroundSpeedMps);
        double next=toTerminal.Norm()/effective;
        if(!Finite(next)){p.bValid=false;p.Failure=PredictionFailure::NonFiniteTime;return p;}if(next>c.PredictionTimeCapS){p.bValid=false;p.bConverged=false;p.Failure=PredictionFailure::TimeCapReached;p.PredictionTimeS=next;return p;}
        const double posDelta=have?(p.TerminalPose.Position-previous.Position).Norm():std::numeric_limits<double>::infinity();
        const double hdgDelta=have?std::abs(WrapPi(p.TerminalPose.CourseRad-previous.CourseRad)):std::numeric_limits<double>::infinity();
        if(have&&posDelta<=c.PositionToleranceM&&hdgDelta<=c.HeadingToleranceRad&&std::abs(next-time)<=c.TimeToleranceS){p.bConverged=true;p.bValid=true;p.Failure=PredictionFailure::None;return p;}
        previous=p.TerminalPose;have=true;time=next;out=p;
    }
    out.bValid=false;out.bConverged=false;out.Failure=PredictionFailure::NotConverged;out.Iterations=c.MaxIterations;return out;
}
}
