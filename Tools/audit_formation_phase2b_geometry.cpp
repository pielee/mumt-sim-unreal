#include "FormationControl/FormationCapturePlanner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace FormationControl;

static double HermiteCurvature(const FVector2& P0, const FVector2& T0,
                               const FVector2& P1, const FVector2& T1, double U)
{
    const double D = (P1-P0).Norm();
    const FVector2 M0=T0.Normalized()*D, M1=T1.Normalized()*D;
    const double U2=U*U;
    const FVector2 D1=P0*(6*U2-6*U)+M0*(3*U2-4*U+1)+P1*(-6*U2+6*U)+M1*(3*U2-2*U);
    const FVector2 D2=P0*(12*U-6)+M0*(6*U-4)+P1*(-12*U+6)+M1*(6*U-2);
    const double L=D1.Norm();
    return L>1e-9 ? D1.Cross(D2)/(L*L*L) : INFINITY;
}

int main()
{
    constexpr double Dt=1.0/60.0;
    const double Speeds[]={120,180,240};
    const double Banks[]={30,45,60};
    const double CourseErrors[]={90,180};
    const double Sides[]={-1,1};
    const double SlotCurvatures[]={-1.0/3000.0,1.0/3000.0};
    int Cases=0, GeometryPass=0;
    double WorstRatio=0, MaxTangentJump=0, MaxCurvatureJump=0;

    for(double Speed:Speeds) for(double BankDeg:Banks) for(double CourseErrDeg:CourseErrors)
    for(double Side:Sides) for(double SlotK:SlotCurvatures) {
        FMovingSlotState Slot;
        Slot.PositionNE={0,0}; Slot.AltitudeM=4000; Slot.GroundVelocityNE={220,0};
        Slot.UnitTangentNE={1,0}; Slot.GroundSpeedMps=220; Slot.CurvaturePerM=SlotK; Slot.bValid=true;
        FFormationFollowerState F;
        F.PositionNE={-2500,Side*1200}; F.AltitudeM=4000;
        F.GroundCourseRad=CourseErrDeg*Pi/180.0; F.GroundVelocityNE=FromCourse(F.GroundCourseRad)*Speed;
        F.TrueAirspeedMps=Speed; F.EquivalentAirspeedMps=Speed;
        F.AltitudeMarginM=2000; F.AirspeedMarginMps=100; F.bGroundCourseValid=true; F.bValid=true;
        FFormationAircraftLimits L;
        L.PlannerBankLimitRad=BankDeg*Pi/180.0; L.TurnRadiusSafetyFactor=1.25;
        L.MinEquivalentAirspeedMps=100; L.MaxEquivalentAirspeedMps=350;
        L.AvailableDecelerationMps2=2; L.bDecelerationModelValid=true;
        FFormationCapturePlanner Planner;
        FFormationPlannerDiagnostics D;
        const FFormationPathReference R=Planner.Update(F,Slot,L,{},Dt,D);

        const FVector2 Target=Slot.PositionNE+Slot.GroundVelocityNE*Planner.Config.PredictionTimeS-
                              Slot.UnitTangentNE*D.EntryDistanceM;
        double ActualMaxK=0;
        for(int I=0;I<=2000;++I)
            ActualMaxK=std::max(ActualMaxK,std::abs(HermiteCurvature(F.PositionNE,FromCourse(F.GroundCourseRad),
                                                                    Target,Slot.UnitTangentNE,I/2000.0)));
        const double KLimit=1.0/D.MinimumTurnRadiusM;
        const bool Pass=std::isfinite(ActualMaxK)&&ActualMaxK<=KLimit*(1.0+1e-6);
        GeometryPass+=Pass; ++Cases;
        WorstRatio=std::max(WorstRatio,ActualMaxK/KLimit);
        const double TangentJump=std::abs(WrapPi(std::atan2(R.UnitTangentE,R.UnitTangentN)-F.GroundCourseRad));
        MaxTangentJump=std::max(MaxTangentJump,TangentJump);
        MaxCurvatureJump=std::max(MaxCurvatureJump,std::abs(R.CurvaturePerM));
        std::printf("speed=%3.0f bank=%2.0f course=%3.0f side=%+1.0f slotK=%+.7f "
                    "Rmin=%7.1f actualK=%9.7f limit=%9.7f outputK=%9.7f %s\n",
                    Speed,BankDeg,CourseErrDeg,Side,SlotK,D.MinimumTurnRadiusM,ActualMaxK,KLimit,
                    R.CurvaturePerM,Pass?"PASS":"GEOMETRY_FAIL");
    }
    std::printf("AUDIT cases=%d geometry_pass=%d geometry_fail=%d worst_ratio=%.3f "
                "max_tangent_jump=%.6frad max_output_curvature_jump=%.9f/m\n",
                Cases,GeometryPass,Cases-GeometryPass,WorstRatio,MaxTangentJump,MaxCurvatureJump);
    // Audit reports evidence; a geometry failure is expected for the current output-only clamp and
    // must not be mistaken for a production test gate until the planner geometry is redesigned.
    return 0;
}
