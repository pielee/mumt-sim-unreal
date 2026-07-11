#include "FormationControlV2/DubinsPath.h"

#include <algorithm>

namespace FormationControlV2 {
namespace {
using Params = std::array<double, 3>;

bool LSL(double a, double b, double d, Params &o) {
    const double tmp = d + std::sin(a) - std::sin(b);
    const double p2 = 2 + d*d - 2*std::cos(a-b) + 2*d*(std::sin(a)-std::sin(b));
    if (p2 < -1e-12) return false;
    const double x = std::atan2(std::cos(b)-std::cos(a), tmp);
    o = {Mod2Pi(-a+x), std::sqrt(std::max(0.0,p2)), Mod2Pi(b-x)}; return true;
}
bool RSR(double a, double b, double d, Params &o) {
    const double tmp = d - std::sin(a) + std::sin(b);
    const double p2 = 2 + d*d - 2*std::cos(a-b) + 2*d*(-std::sin(a)+std::sin(b));
    if (p2 < -1e-12) return false;
    const double x = std::atan2(std::cos(a)-std::cos(b), tmp);
    o = {Mod2Pi(a-x), std::sqrt(std::max(0.0,p2)), Mod2Pi(-b+x)}; return true;
}
bool LSR(double a, double b, double d, Params &o) {
    const double p2 = -2 + d*d + 2*std::cos(a-b) + 2*d*(std::sin(a)+std::sin(b));
    if (p2 < -1e-12) return false;
    const double p = std::sqrt(std::max(0.0,p2));
    const double x = std::atan2(-std::cos(a)-std::cos(b), d+std::sin(a)+std::sin(b))-std::atan2(-2.0,p);
    o = {Mod2Pi(-a+x), p, Mod2Pi(-b+x)}; return true;
}
bool RSL(double a, double b, double d, Params &o) {
    const double p2 = d*d - 2 + 2*std::cos(a-b) - 2*d*(std::sin(a)+std::sin(b));
    if (p2 < -1e-12) return false;
    const double p = std::sqrt(std::max(0.0,p2));
    const double x = std::atan2(std::cos(a)+std::cos(b), d-std::sin(a)-std::sin(b))-std::atan2(2.0,p);
    o = {Mod2Pi(a-x), p, Mod2Pi(b-x)}; return true;
}
bool RLR(double a, double b, double d, Params &o) {
    const double x = (6-d*d+2*std::cos(a-b)+2*d*(std::sin(a)-std::sin(b)))/8;
    if (x < -1-1e-12 || x > 1+1e-12) return false;
    const double p = Mod2Pi(TwoPi-std::acos(Clamp(x,-1.0,1.0)));
    const double t = Mod2Pi(a-std::atan2(std::cos(a)-std::cos(b),d-std::sin(a)+std::sin(b))+p/2);
    o={t,p,Mod2Pi(a-b-t+p)}; return true;
}
bool LRL(double a, double b, double d, Params &o) {
    const double x = (6-d*d+2*std::cos(a-b)+2*d*(-std::sin(a)+std::sin(b)))/8;
    if (x < -1-1e-12 || x > 1+1e-12) return false;
    const double p = Mod2Pi(TwoPi-std::acos(Clamp(x,-1.0,1.0)));
    const double t = Mod2Pi(-a-std::atan2(std::cos(a)-std::cos(b),d+std::sin(a)-std::sin(b))+p/2);
    o={t,p,Mod2Pi(b-a-t+p)}; return true;
}

std::array<SegmentType,3> SegmentTypes(DubinsType t) {
    switch(t) {
    case DubinsType::LSL:return {SegmentType::LeftArc,SegmentType::Straight,SegmentType::LeftArc};
    case DubinsType::RSR:return {SegmentType::RightArc,SegmentType::Straight,SegmentType::RightArc};
    case DubinsType::LSR:return {SegmentType::LeftArc,SegmentType::Straight,SegmentType::RightArc};
    case DubinsType::RSL:return {SegmentType::RightArc,SegmentType::Straight,SegmentType::LeftArc};
    case DubinsType::RLR:return {SegmentType::RightArc,SegmentType::LeftArc,SegmentType::RightArc};
    case DubinsType::LRL:return {SegmentType::LeftArc,SegmentType::RightArc,SegmentType::LeftArc};
    default:return {SegmentType::Straight,SegmentType::Straight,SegmentType::Straight};
    }
}
double Curvature(SegmentType t,double r){return t==SegmentType::RightArc?1/r:(t==SegmentType::LeftArc?-1/r:0);}
}

std::vector<DubinsCandidate> DubinsPath::BuildCandidates(const Pose2 &s,const Pose2 &g,double r) {
    std::vector<DubinsCandidate> out;
    if(!s.IsFinite()||!g.IsFinite()||!Finite(r)||r<=0)return out;
    const Vec2 delta=g.Position-s.Position; const double range=delta.Norm();
    const double line=range>1e-12?std::atan2(delta.E,delta.N):s.CourseRad;
    const double a=Mod2Pi(s.CourseRad-line), b=Mod2Pi(g.CourseRad-line), d=range/r;
    const DubinsType types[]={DubinsType::LSL,DubinsType::RSR,DubinsType::LSR,DubinsType::RSL,DubinsType::RLR,DubinsType::LRL};
    for(DubinsType type:types){Params p{};bool ok=false;switch(type){
        // The closed-form formulas use mathematical positive-angle (left) turns.
        // Geographic N/E course is positive clockwise/right, so physical L/R
        // families use the opposite closed-form family.
        case DubinsType::LSL:ok=RSR(a,b,d,p);break;case DubinsType::RSR:ok=LSL(a,b,d,p);break;
        case DubinsType::LSR:ok=RSL(a,b,d,p);break;case DubinsType::RSL:ok=LSR(a,b,d,p);break;
        case DubinsType::RLR:ok=LRL(a,b,d,p);break;case DubinsType::LRL:ok=RLR(a,b,d,p);break;default:break;}
        if(ok&&Finite(p[0])&&Finite(p[1])&&Finite(p[2]))out.push_back({type,p,(p[0]+p[1]+p[2])*r,true});
    } return out;
}

double DubinsPath::ComputeForwardSearchWindow(double speed,double dt,const ProjectionWindowConfig&c,bool&valid){
    valid=Finite(speed)&&speed>=0&&Finite(dt)&&dt>=c.MinimumDtS&&dt<=c.MaximumDtS&&
        Finite(c.AdvanceFactor)&&c.AdvanceFactor>=1&&Finite(c.ForwardMarginM)&&c.ForwardMarginM>=0&&
        Finite(c.MinimumForwardWindowM)&&Finite(c.MaximumForwardWindowM)&&
        c.MinimumForwardWindowM>=0&&c.MaximumForwardWindowM>=c.MinimumForwardWindowM;
    if(!valid)return 0.0;
    const double expected=speed*dt*c.AdvanceFactor;
    return Clamp(std::max(c.MinimumForwardWindowM,expected+c.ForwardMarginM),c.MinimumForwardWindowM,c.MaximumForwardWindowM);
}

bool DubinsPath::Build(const Pose2&s,const Pose2&g,double r){
    auto c=BuildCandidates(s,g,r);if(c.empty()){*this={};return false;}
    const auto best=std::min_element(c.begin(),c.end(),[](const auto&a,const auto&b){return a.LengthM<b.LengthM;});
    return BuildCandidate(s,g,r,*best);
}
bool DubinsPath::BuildType(const Pose2&s,const Pose2&g,double r,DubinsType type){
    const auto c=BuildCandidates(s,g,r);for(const auto&x:c)if(x.Type==type)return BuildCandidate(s,g,r,x);*this={};return false;
}
bool DubinsPath::BuildCandidate(const Pose2&s,const Pose2&g,double r,const DubinsCandidate&c){
    *this={};if(!c.bValid)return false;Start=s;Terminal=g;RadiusM=r;Type=c.Type;const auto types=SegmentTypes(Type);Pose2 p=s;double accum=0;
    for(int i=0;i<3;i++){Segments[i]={types[i],c.NormalizedLengths[i]*r,accum,p};p=Advance(p,types[i],Segments[i].LengthM,r);accum+=Segments[i].LengthM;}
    LengthM=accum;const double posErr=(p.Position-g.Position).Norm(),hdgErr=std::abs(WrapPi(p.CourseRad-g.CourseRad));
    bValid=Finite(LengthM)&&LengthM>=0&&posErr<=1e-6*std::max(1.0,LengthM)&&hdgErr<=1e-8;
    return bValid;
}
Pose2 DubinsPath::Advance(const Pose2&p,SegmentType type,double ds,double r){
    if(type==SegmentType::Straight)return {p.Position+FromCourse(p.CourseRad)*ds,p.CourseRad};
    const double k=Curvature(type,r),next=p.CourseRad+k*ds;
    const Vec2 q{p.Position.N+(std::sin(next)-std::sin(p.CourseRad))/k,
                 p.Position.E+(-std::cos(next)+std::cos(p.CourseRad))/k};
    return {q,WrapPi(next)};
}
PathSample DubinsPath::Sample(double s)const{
    PathSample o;if(!bValid||!Finite(s))return o;s=Clamp(s,0.0,LengthM);int index=2;
    if(s<Segments[1].StartS-1e-12)index=0;else if(s<Segments[2].StartS-1e-12)index=1;
    while(index<2&&Segments[index].LengthM<=1e-12)index++;
    const auto&seg=Segments[index];const Pose2 p=Advance(seg.StartPose,seg.Type,Clamp(s-seg.StartS,0.0,seg.LengthM),RadiusM);
    o={p.Position,FromCourse(p.CourseRad),Curvature(seg.Type,RadiusM),s,index,true};return o;
}
std::array<double,2> DubinsPath::GetSegmentJunctions()const{return {Segments[1].StartS,Segments[2].StartS};}

ProjectionResult DubinsPath::ProjectSegment(const Vec2&x,const DubinsSegment&seg,double lo,double hi)const{
    ProjectionResult o;if(lo>hi)return o;double local=lo;
    if(seg.Type==SegmentType::Straight){const Vec2 t=FromCourse(seg.StartPose.CourseRad);local=Clamp((x-seg.StartPose.Position).Dot(t),lo,hi);}
    else {const double k=Curvature(seg.Type,RadiusM);const Vec2 center=seg.StartPose.Position+RightNormal(seg.StartPose.CourseRad)*(1.0/k);
        const Vec2 radial=x-center;if(radial.Norm()>1e-12){const double radialAngle=std::atan2(radial.E,radial.N);
            const double course=radialAngle+(k>0?Pi/2:-Pi/2);const double angular=Mod2Pi((k>0?1:-1)*(course-seg.StartPose.CourseRad));
            local=Clamp(angular/std::abs(k),lo,hi);}}
    const Pose2 p=Advance(seg.StartPose,seg.Type,local,RadiusM);o.S=seg.StartS+local;o.DistanceM=(x-p.Position).Norm();o.bValid=Finite(o.DistanceM);return o;
}
ProjectionResult DubinsPath::Project(const Vec2&x,double previousS,double back,double forward,bool monotonic,double maxDist)const{
    ProjectionResult best;best.DistanceM=std::numeric_limits<double>::infinity();
    if(!bValid||!x.IsFinite()||!Finite(previousS)){best.Failure=ProjectionFailure::InvalidInput;return best;}
    if(!Finite(back)||!Finite(forward)||back<0||forward<0){best.Failure=ProjectionFailure::InvalidWindow;return best;}
    const double minS=Clamp(previousS-back,0.0,LengthM),maxS=Clamp(previousS+forward,0.0,LengthM);if(minS>maxS){best.Failure=ProjectionFailure::InvalidWindow;return best;}
    for(int i=0;i<3;i++){const auto&s=Segments[i];const double lo=std::max(0.0,minS-s.StartS),hi=std::min(s.LengthM,maxS-s.StartS);if(lo<=hi){auto c=ProjectSegment(x,s,lo,hi);c.SegmentIndex=i;if(c.bValid&&c.DistanceM<best.DistanceM)best=c;}}
    if(!best.bValid){best.Failure=ProjectionFailure::NoCandidate;return best;}if(monotonic&&best.S<previousS)best.S=previousS;
    const auto sample=Sample(best.S);best.DistanceM=(x-sample.Position).Norm();if(!Finite(best.DistanceM)||best.DistanceM>maxDist){best.bValid=false;best.Failure=ProjectionFailure::ExcessiveDistance;return best;}
    best.SegmentIndex=sample.SegmentIndex;best.Failure=ProjectionFailure::None;return best;
}
} // namespace FormationControlV2
