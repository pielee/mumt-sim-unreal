#include "FormationControl/FormationSlotGenerator.h"

namespace FormationControl
{
void FFormationSlotGenerator::Reset()
{
    LastTangent = {1.0, 0.0};
    bHasLastTangent = false;
}

FMovingSlotState FFormationSlotGenerator::Update(
    const FFormationLeaderState& L, const FFormationOffset& O, double DtS)
{
    FMovingSlotState S;
    S.LeaderStateAgeS = L.StateAgeS;
    const bool bFinite = L.PositionNE.IsFinite() && L.GroundVelocityNE.IsFinite() &&
        std::isfinite(L.AltitudeM) && std::isfinite(L.ClimbRateMps) &&
        std::isfinite(L.GroundCourseRad) && std::isfinite(L.CourseRateRadps) &&
        std::isfinite(L.CurvaturePerM) && L.AccelerationNE.IsFinite() &&
        std::isfinite(L.VerticalAccelerationMps2) && std::isfinite(L.StateAgeS) &&
        std::isfinite(O.FrontM) && std::isfinite(O.RightM) && std::isfinite(O.UpM) &&
        std::isfinite(DtS) && DtS > 0.0 && DtS <= MaxDtS;
    if (!L.bValid || !bFinite || L.StateAgeS < 0.0 || L.StateAgeS > MaxLeaderStateAgeS)
        return S;

    const double LeaderSpeed = L.GroundVelocityNE.Norm();
    FVector2 T;
    if (L.bCourseValid)
        T = FromCourse(L.GroundCourseRad);
    else if (LeaderSpeed >= MinCourseSpeedMps)
        T = L.GroundVelocityNE.Normalized();
    else if (bHasLastTangent) {
        T = LastTangent;
        S.bHeldLowSpeedCourse = true;
    } else {
        return S; // no arbitrary low-speed course
    }
    if (!T.IsFinite() || T.Norm() < 0.99)
        return S;
    LastTangent = T;
    bHasLastTangent = true;

    const FVector2 R{-T.E, T.N}; // right in N/E
    const FVector2 OffsetNE = T * O.FrontM + R * O.RightM;
    S.PositionNE = L.PositionNE + OffsetNE;
    S.AltitudeM = L.AltitudeM + O.UpM;

    double Omega = 0.0;
    if (L.bCourseRateValid)
        Omega = L.CourseRateRadps;
    else if (L.bCurvatureValid)
        Omega = L.CurvaturePerM * LeaderSpeed;

    // N/E uses positive clockwise course; d(r)/dt = omega * rotate-right(r).
    const FVector2 RotatedRight{-OffsetNE.E, OffsetNE.N};
    S.GroundVelocityNE = L.GroundVelocityNE + RotatedRight * Omega;
    S.VerticalVelocityMps = L.ClimbRateMps;
    S.GroundSpeedMps = S.GroundVelocityNE.Norm();
    S.UnitTangentNE = S.GroundSpeedMps >= MinCourseSpeedMps ? S.GroundVelocityNE.Normalized() : T;
    S.CurvaturePerM = L.bCurvatureValid ? L.CurvaturePerM :
        ((S.GroundSpeedMps > 1e-6 && L.bCourseRateValid) ? Omega / S.GroundSpeedMps : 0.0);

    if (L.bAccelerationValid && (L.bCourseRateValid || L.bCurvatureValid)) {
        // Complete only under the explicit constant-omega assumption: leader linear acceleration
        // plus centripetal offset term. Angular acceleration is unavailable.
        S.AccelerationNE = L.AccelerationNE - OffsetNE * (Omega * Omega);
        S.VerticalAccelerationMps2 = L.VerticalAccelerationMps2;
        S.bAccelerationValid = false; // alpha x r is missing, so never claim full acceleration.
    }
    S.bValid = S.PositionNE.IsFinite() && S.GroundVelocityNE.IsFinite() &&
        S.UnitTangentNE.IsFinite() && std::isfinite(S.CurvaturePerM) && S.UnitTangentNE.Norm() > 0.99;
    return S;
}
}
