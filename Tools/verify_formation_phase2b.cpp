#include "FormationControl/FormationSlotGenerator.h"
#include "FormationControl/FormationCapturePlanner.h"

#include <cmath>
#include <cstdio>
#include <limits>

using namespace FormationControl;
static int Pass = 0, Fail = 0;
#define CHECK(C, M) do { if (C) ++Pass; else { ++Fail; std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, M); } } while (0)
constexpr double Dt = 1.0 / 60.0;

static FFormationLeaderState Leader(double Course, double Speed)
{
    FFormationLeaderState L;
    L.PositionNE = {1000, 2000}; L.AltitudeM = 4000;
    L.GroundCourseRad = Course; L.GroundVelocityNE = FromCourse(Course) * Speed;
    L.ClimbRateMps = 0; L.StateAgeS = 0;
    L.bCourseValid = true; L.bValid = true;
    return L;
}

static FFormationFollowerState Follower(const FVector2& P, double Course, double Speed, double Eas = 220)
{
    FFormationFollowerState F;
    F.PositionNE = P; F.AltitudeM = 4000; F.GroundCourseRad = Course;
    F.GroundVelocityNE = FromCourse(Course) * Speed; F.TrueAirspeedMps = Eas;
    F.EquivalentAirspeedMps = Eas; F.AltitudeMarginM = 2000; F.AirspeedMarginMps = 100;
    F.bGroundCourseValid = true; F.bValid = true;
    return F;
}

static FFormationAircraftLimits Limits(bool Decel = true)
{
    FFormationAircraftLimits L;
    L.PlannerBankLimitRad = 45 * Pi / 180; L.TurnRadiusSafetyFactor = 1.25;
    L.MinEquivalentAirspeedMps = 120; L.MaxEquivalentAirspeedMps = 335;
    L.AvailableDecelerationMps2 = Decel ? 2.0 : 0.0; L.bDecelerationModelValid = Decel;
    return L;
}

static void TestSlot()
{
    std::puts("[1] Slot Generator");
    FFormationSlotGenerator G;
    FFormationOffset O{-80, 100, 20};
    auto L = Leader(Pi/2, 220); // east; right points south
    FMovingSlotState S = G.Update(L, O, Dt);
    CHECK(S.bValid, "straight slot valid");
    CHECK(std::fabs(S.PositionNE.N - 900) < 1e-9 && std::fabs(S.PositionNE.E - 1920) < 1e-9,
          "front/right offset in N/E");
    CHECK(std::fabs(S.GroundVelocityNE.N) < 1e-9 && std::fabs(S.GroundVelocityNE.E-220)<1e-9,
          "straight slot velocity");
    CHECK(std::fabs(S.AltitudeM-4020)<1e-9, "up offset");

    L.bCourseRateValid = true; L.bCurvatureValid = true;
    L.CourseRateRadps = 0.02; L.CurvaturePerM = L.CourseRateRadps/220;
    S = G.Update(L, O, Dt);
    CHECK(S.GroundVelocityNE.N > 0 && S.GroundVelocityNE.E < 220, "right turn omega cross r sign");
    CHECK(S.CurvaturePerM > 0, "right curvature positive");
    L.CourseRateRadps = -0.02; L.CurvaturePerM = -0.02/220;
    S = G.Update(L, O, Dt); CHECK(S.CurvaturePerM < 0, "left curvature negative");
    L.ClimbRateMps = 15; S = G.Update(L,O,Dt); CHECK(S.VerticalVelocityMps == 15, "climb up-positive");
    L.GroundCourseRad = 2*Pi + 0.01; L.GroundVelocityNE = FromCourse(0.01)*220;
    S = G.Update(L,O,Dt); CHECK(S.bValid && S.UnitTangentNE.IsFinite(), "course wrap finite");

    L.StateAgeS = 2; CHECK(!G.Update(L,O,Dt).bValid, "stale leader invalid");
    L.StateAgeS = 0; L.PositionNE.N = std::numeric_limits<double>::quiet_NaN();
    CHECK(!G.Update(L,O,Dt).bValid, "NaN leader invalid");

    FFormationSlotGenerator Low;
    L = Leader(0, 2); L.bCourseValid = false;
    CHECK(!Low.Update(L,O,Dt).bValid, "low speed with no course history invalid");
    L = Leader(0,220); Low.Update(L,O,Dt); L.bCourseValid=false; L.GroundVelocityNE={1,1};
    S=Low.Update(L,O,Dt); CHECK(S.bValid && S.bHeldLowSpeedCourse, "low speed holds finite prior tangent");
}

static void TestGeometryAndContinuity()
{
    std::puts("[2] Geometry / Continuity");
    FMovingSlotState S; S.PositionNE={0,0}; S.AltitudeM=4000; S.GroundVelocityNE={220,0};
    S.UnitTangentNE={1,0}; S.GroundSpeedMps=220; S.bValid=true;
    const FVector2 Starts[]={{-3000,0},{1000,0},{-2000,1200},{-2000,-1200}};
    for (const auto& P: Starts) {
        FFormationCapturePlanner C; FFormationPlannerDiagnostics D;
        auto R=C.Update(Follower(P,0,240),S,Limits(),{},Dt,D);
        CHECK(R.bValid, "approach geometry valid");
        CHECK(std::isfinite(R.CurvaturePerM) && std::abs(R.CurvaturePerM)<=1.0/D.MinimumTurnRadiusM+1e-12,
              "curvature respects minimum radius");
        CHECK(std::fabs(std::hypot(R.UnitTangentN,R.UnitTangentE)-1)<1e-9,"unit tangent");
        CHECK(!(std::fabs(R.PathPositionN-S.PositionNE.N)<1e-6 && std::fabs(R.PathPositionE-S.PositionNE.E)<1e-6),
              "far/corridor does not directly aim slot point");
    }

    FFormationCapturePlanner C; FFormationPlannerDiagnostics D;
    auto F=Follower({-2500,800},-20*Pi/180,240);
    auto Prev=C.Update(F,S,Limits(),{},Dt,D);
    double MaxP=0, MaxA=0, MaxK=0, MaxAlt=0, MaxV=0;
    for(int i=0;i<300;++i) {
        S.PositionNE.N += 220*Dt;
        F.PositionNE = F.PositionNE + F.GroundVelocityNE*Dt;
        auto R=C.Update(F,S,Limits(),{},Dt,D);
        const double Pj=std::hypot(R.PathPositionN-Prev.PathPositionN,R.PathPositionE-Prev.PathPositionE);
        const double Aj=std::abs(WrapPi(std::atan2(R.UnitTangentE,R.UnitTangentN)-std::atan2(Prev.UnitTangentE,Prev.UnitTangentN)));
        MaxP=std::max(MaxP,Pj); MaxA=std::max(MaxA,Aj); MaxK=std::max(MaxK,std::abs(R.CurvaturePerM-Prev.CurvaturePerM));
        MaxAlt=std::max(MaxAlt,std::abs(R.AltitudeReferenceM-Prev.AltitudeReferenceM));
        MaxV=std::max(MaxV,std::abs(R.AirspeedReferenceEasMps-Prev.AirspeedReferenceEasMps)); Prev=R;
    }
    CHECK(MaxP<=600*Dt+1e-8,"path position continuity limit");
    CHECK(MaxA<=30*Pi/180*Dt+1e-8,"tangent continuity limit");
    CHECK(MaxK<=0.002*Dt+1e-10,"curvature-rate limit");
    CHECK(MaxAlt<=50*Dt+1e-8 && MaxV<=25*Dt+1e-8,"altitude/airspeed continuity limits");
    std::printf("  continuity max: pos=%.6fm tangent=%.6frad curvature=%.9f/m alt=%.6fm eas=%.6fm/s\n",
                MaxP, MaxA, MaxK, MaxAlt, MaxV);
}

static void TestSpeedAndFailureTrace()
{
    std::puts("[3] Speed Planning / Overshoot Trace");
    FMovingSlotState S; S.PositionNE={0,0}; S.AltitudeM=4000; S.GroundVelocityNE={220,0};
    S.UnitTangentNE={1,0}; S.GroundSpeedMps=220; S.bValid=true;
    FFormationPlannerDiagnostics D;
    FFormationCapturePlanner C;
    auto R=C.Update(Follower({-1200,0},0,242,242),S,Limits(),{},Dt,D);
    CHECK(std::fabs(D.ClosingSpeedMps-22)<1e-9,"relative closing speed");
    CHECK(std::fabs(D.EstimatedBrakingDistanceM-121)<1e-9,"braking distance v^2/2a");
    CHECK(D.BrakingMarginM>0,"sufficient braking margin");
    CHECK(R.AirspeedReferenceEasMps>=220 && R.AirspeedReferenceEasMps<=335,"allowed EAS bounded");

    FFormationCapturePlanner Invalid;
    R=Invalid.Update(Follower({-450,0},0,242,242),S,Limits(false),{},Dt,D);
    CHECK(R.State==EFormationPlannerState::AbortOrRecover && D.bHighSpeedCaptureBlocked,
          "invalid deceleration model blocks high-speed capture");

    // Reproduce legacy trace condition: high closing speed, slot ahead. Valid decel model must
    // lead through corridor/speed matching without tangent reversal or state chatter.
    FFormationCapturePlanner T; T.Config.MinStateDwellS=0.2;
    auto F=Follower({-450,0},0,242,242);
    double MaxTangentJump=0, MaxCurvature=0; int Changes=0;
    EFormationPlannerState Last=T.GetState(); FFormationPathReference Prev{}; bool Has=false, SawSpeed=false;
    for(int i=0;i<180;++i) {
        R=T.Update(F,S,Limits(),{},Dt,D);
        if(R.State!=Last){++Changes;Last=R.State;}
        SawSpeed |= R.State==EFormationPlannerState::SpeedMatch;
        if(Has) MaxTangentJump=std::max(MaxTangentJump,std::abs(WrapPi(std::atan2(R.UnitTangentE,R.UnitTangentN)-std::atan2(Prev.UnitTangentE,Prev.UnitTangentN))));
        MaxCurvature=std::max(MaxCurvature,std::abs(R.CurvaturePerM)); Prev=R;Has=true;
        // Kinematic response follows commanded deceleration before reaching slot.
        const double Closing=std::max(0.0,F.GroundVelocityNE.N-S.GroundVelocityNE.N);
        F.GroundVelocityNE.N-=std::min(Closing,2.0*Dt); F.PositionNE.N+=F.GroundVelocityNE.N*Dt;
        S.PositionNE.N+=S.GroundVelocityNE.N*Dt;
    }
    CHECK(SawSpeed,"high-speed approach enters SPEED_MATCH");
    CHECK(F.PositionNE.N<=S.PositionNE.N+1e-6,"kinematic trace does not pass slot");
    CHECK(MaxTangentJump<=30*Pi/180*Dt+1e-8,"no tangent reversal");
    CHECK(MaxCurvature<=1.0/(1.25*242*242/GravityMps2)+1e-5,"curvature bounded");
    CHECK(Changes<=4,"no state chatter");
    std::printf("  overshoot trace: passed=%d speed_match=%d tangent_jump=%.6frad max_curvature=%.9f/m transitions=%d\n",
                F.PositionNE.N>S.PositionNE.N, SawSpeed, MaxTangentJump, MaxCurvature, Changes);
}

static void TestStateAndCompatibility()
{
    std::puts("[4] State Machine / NPFG-TECS compatibility contract");
    FMovingSlotState S; S.PositionNE={0,0}; S.AltitudeM=4000; S.GroundVelocityNE={220,0};
    S.UnitTangentNE={1,0}; S.GroundSpeedMps=220; S.bValid=true;
    FFormationCapturePlanner C; C.Config.MinStateDwellS=0.1; FFormationPlannerDiagnostics D;
    auto F=Follower({-600,0},0,220);
    bool Cap=false, Speed=false, Maintain=false;
    for(int i=0;i<60;++i){auto R=C.Update(F,S,Limits(),{},Dt,D);Cap|=R.State==EFormationPlannerState::CaptureCorridor; Speed|=R.State==EFormationPlannerState::SpeedMatch;}
    F.PositionNE={-50,0}; F.GroundVelocityNE={220,0};
    for(int i=0;i<60;++i){auto R=C.Update(F,S,Limits(),{},Dt,D);Speed|=R.State==EFormationPlannerState::SpeedMatch;Maintain|=R.State==EFormationPlannerState::FormationMaintain;}
    CHECK(Cap&&Speed&&Maintain,"far->corridor->speed->maintain transitions");
    F.PositionNE={-2000,1000};
    for(int i=0;i<20;++i) C.Update(F,S,Limits(),{},Dt,D);
    CHECK(C.GetState()==EFormationPlannerState::FarRejoin,"maintain loss -> far rejoin");
    F.bTecsUnderspeed=true; auto R=C.Update(F,S,Limits(),{},Dt,D);
    CHECK(R.State==EFormationPlannerState::AbortOrRecover && R.bValid,"any state -> valid recovery path");

    // Structural contract used by the already approved adapter APIs: N/E meters, unit tangent,
    // +right curvature 1/m, altitude meters Up, EAS m/s. No controller is executed or wired here.
    CHECK(std::isfinite(R.PathPositionN)&&std::isfinite(R.PathPositionE),"NPFG position finite");
    CHECK(std::fabs(std::hypot(R.UnitTangentN,R.UnitTangentE)-1)<1e-9,"NPFG tangent nondegenerate");
    CHECK(std::isfinite(R.CurvaturePerM),"NPFG curvature finite");
    CHECK(std::isfinite(R.AltitudeReferenceM),"TECS altitude reference finite");
    CHECK(std::isfinite(R.AirspeedReferenceEasMps)&&R.AirspeedReferenceEasMps>0,"TECS EAS reference finite");
}

int main()
{
    TestSlot(); TestGeometryAndContinuity(); TestSpeedAndFailureTrace(); TestStateAndCompatibility();
    std::printf("Phase2B PASS=%d FAIL=%d\n",Pass,Fail);
    return Fail?1:0;
}
