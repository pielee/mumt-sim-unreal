#include "FormationControl/FormationCapturePlanner.h"

#include <algorithm>

namespace FormationControl
{
namespace
{
struct FHermiteSample { FVector2 P; FVector2 T; double K = 0.0; bool Valid = false; };

FHermiteSample SampleHermite(const FVector2& P0, const FVector2& T0,
                             const FVector2& P1, const FVector2& T1, double U)
{
    FHermiteSample O;
    const double D = (P1 - P0).Norm();
    if (!(D > 1.0) || !P0.IsFinite() || !P1.IsFinite()) return O;
    const FVector2 M0 = T0.Normalized() * D;
    const FVector2 M1 = T1.Normalized() * D;
    U = Clamp(U, 0.0, 1.0);
    const double U2 = U * U, U3 = U2 * U;
    O.P = P0 * (2*U3 - 3*U2 + 1) + M0 * (U3 - 2*U2 + U) +
          P1 * (-2*U3 + 3*U2) + M1 * (U3 - U2);
    const FVector2 D1 = P0 * (6*U2 - 6*U) + M0 * (3*U2 - 4*U + 1) +
                        P1 * (-6*U2 + 6*U) + M1 * (3*U2 - 2*U);
    const FVector2 D2 = P0 * (12*U - 6) + M0 * (6*U - 4) +
                        P1 * (-12*U + 6) + M1 * (6*U - 2);
    const double L = D1.Norm();
    if (!(L > 1e-6)) return O;
    O.T = D1 / L;
    O.K = D1.Cross(D2) / (L * L * L); // +right in N/E
    O.Valid = O.P.IsFinite() && O.T.IsFinite() && std::isfinite(O.K);
    return O;
}

double Slew(double Current, double Target, double MaxDelta)
{
    return Current + Clamp(Target - Current, -MaxDelta, MaxDelta);
}
}

void FFormationCapturePlanner::Reset()
{
    State = EFormationPlannerState::FarRejoin;
    StateAgeS = 0.0;
    DivergenceTimerS = 0.0;
    bHasPrevious = false;
}

void FFormationCapturePlanner::Transition(EFormationPlannerState Next)
{
    if (State != Next) { State = Next; StateAgeS = 0.0; }
}

FFormationPathReference FFormationCapturePlanner::MakeRecovery(
    const FFormationFollowerState& F, const FMovingSlotState& S,
    const FFormationAircraftLimits& L) const
{
    FFormationPathReference R;
    const FVector2 T = F.bGroundCourseValid ? FromCourse(F.GroundCourseRad) : F.GroundVelocityNE.Normalized();
    R.PathPositionN = F.PositionNE.N + T.N * 500.0;
    R.PathPositionE = F.PositionNE.E + T.E * 500.0;
    R.UnitTangentN = T.N; R.UnitTangentE = T.E;
    R.CurvaturePerM = 0.0;
    R.AltitudeReferenceM = std::max(F.AltitudeM, S.AltitudeM);
    R.AirspeedReferenceEasMps = Clamp(std::max(F.EquivalentAirspeedMps, L.MinEquivalentAirspeedMps + 20.0),
                                                L.MinEquivalentAirspeedMps, L.MaxEquivalentAirspeedMps);
    R.PathGroundSpeedMps = F.GroundVelocityNE.Norm();
    R.State = EFormationPlannerState::AbortOrRecover;
    R.bValid = T.IsFinite() && T.Norm() > 0.99 && std::isfinite(R.AltitudeReferenceM) &&
               std::isfinite(R.AirspeedReferenceEasMps);
    return R;
}

FFormationPathReference FFormationCapturePlanner::Update(
    const FFormationFollowerState& F, const FMovingSlotState& S,
    const FFormationAircraftLimits& L, const FFormationPlannerSafetyInput& Safety,
    double Dt, FFormationPlannerDiagnostics& D)
{
    D = {};
    const bool bFinite = F.PositionNE.IsFinite() && F.GroundVelocityNE.IsFinite() &&
        std::isfinite(F.AltitudeM) && std::isfinite(F.EquivalentAirspeedMps) &&
        S.PositionNE.IsFinite() && S.GroundVelocityNE.IsFinite() && S.UnitTangentNE.IsFinite() &&
        std::isfinite(S.AltitudeM) && std::isfinite(S.CurvaturePerM) &&
        std::isfinite(L.PlannerBankLimitRad) && std::isfinite(L.TurnRadiusSafetyFactor) &&
        std::isfinite(L.MinEquivalentAirspeedMps) && std::isfinite(L.MaxEquivalentAirspeedMps) &&
        std::isfinite(Dt) && Dt > 0.0 && Dt <= 0.25;
    const bool bInputsValid = F.bValid && S.bValid && Safety.bPathInputValid && bFinite &&
        S.LeaderStateAgeS <= Config.MaxLeaderStateAgeS && F.bGroundCourseValid;

    const FVector2 T = S.UnitTangentNE.Normalized();
    const FVector2 R{-T.E, T.N};
    const FVector2 SlotToFollower = F.PositionNE - S.PositionNE;
    D.AlongErrorM = SlotToFollower.Dot(T); // + follower ahead of slot
    D.CrossErrorM = SlotToFollower.Dot(R); // + follower right of path
    D.VerticalErrorM = F.AltitudeM - S.AltitudeM;
    D.CourseAlignmentErrorRad = bInputsValid ? WrapPi(F.GroundCourseRad - std::atan2(T.E, T.N)) : 0.0;
    const FVector2 RelV = F.GroundVelocityNE - S.GroundVelocityNE;
    D.RelativeAlongVelocityMps = RelV.Dot(T);
    D.RelativeCrossVelocityMps = RelV.Dot(R);
    D.ClosingSpeedMps = std::max(0.0, D.RelativeAlongVelocityMps);
    D.LeaderStateAgeS = S.LeaderStateAgeS;
    D.bDecelerationModelValid = L.bDecelerationModelValid &&
        std::isfinite(L.AvailableDecelerationMps2) && L.AvailableDecelerationMps2 > 0.0;
    D.EstimatedBrakingDistanceM = D.bDecelerationModelValid ?
        D.ClosingSpeedMps * D.ClosingSpeedMps / (2.0 * L.AvailableDecelerationMps2) : 0.0;
    D.RemainingCaptureDistanceM = std::max(0.0, -D.AlongErrorM - L.SafetyDistanceM);
    D.BrakingMarginM = D.RemainingCaptureDistanceM - D.EstimatedBrakingDistanceM;

    const double Gs = F.GroundVelocityNE.Norm();
    const double TanBank = std::tan(L.PlannerBankLimitRad);
    D.MinimumTurnRadiusM = (Gs > 1.0 && TanBank > 1e-6 && L.TurnRadiusSafetyFactor >= 1.0) ?
        L.TurnRadiusSafetyFactor * Gs * Gs / (GravityMps2 * TanBank) : 0.0;
    const bool bTurnLimitsValid = std::isfinite(D.MinimumTurnRadiusM) && D.MinimumTurnRadiusM > 1.0;

    if (Safety.bSlotDistanceDiverging) DivergenceTimerS += Dt; else DivergenceTimerS = 0.0;
    D.DivergenceTimerS = DivergenceTimerS;
    StateAgeS += Dt;
    const bool bAbort = !bInputsValid || !bTurnLimitsValid || !F.bNpfgValid || F.bTecsUnderspeed ||
        F.AltitudeMarginM < Config.AbortAltitudeMarginM || F.AirspeedMarginMps < Config.AbortAirspeedMarginMps ||
        F.BankSaturationDurationS >= Config.BankSaturationTimeoutS || DivergenceTimerS > 3.0 ||
        Safety.bStateChatterDetected || Safety.bCourseReversalRisk;
    if (bAbort) Transition(EFormationPlannerState::AbortOrRecover);

    const double Distance = SlotToFollower.Norm();
    const bool bDwell = StateAgeS >= Config.MinStateDwellS;
    if (!bAbort) {
        switch (State) {
        case EFormationPlannerState::FarRejoin:
            if (bDwell && std::abs(D.CrossErrorM) < Config.CorridorCrossEnterM &&
                std::abs(D.CourseAlignmentErrorRad) < Config.AlignmentEnterRad)
                Transition(EFormationPlannerState::CaptureCorridor);
            break;
        case EFormationPlannerState::CaptureCorridor:
            if (bDwell && (std::abs(D.CrossErrorM) > Config.CorridorCrossExitM ||
                           std::abs(D.CourseAlignmentErrorRad) > Config.AlignmentExitRad))
                Transition(EFormationPlannerState::FarRejoin);
            else if (bDwell && std::abs(D.AlongErrorM) < Config.SpeedMatchAlongM)
                Transition(EFormationPlannerState::SpeedMatch);
            break;
        case EFormationPlannerState::SpeedMatch:
            if (bDwell && (std::abs(D.CrossErrorM) > Config.CorridorCrossExitM || Distance > Config.RejoinDistanceM))
                Transition(EFormationPlannerState::FarRejoin);
            else if (bDwell && std::abs(D.AlongErrorM) < Config.MaintainAlongM &&
                     std::abs(D.CrossErrorM) < Config.MaintainCrossM &&
                     std::abs(D.VerticalErrorM) < Config.MaintainVerticalM &&
                     std::abs(D.RelativeAlongVelocityMps) < Config.MaintainRelativeSpeedMps &&
                     std::abs(D.CourseAlignmentErrorRad) < Config.AlignmentEnterRad &&
                     D.BrakingMarginM >= 0.0)
                Transition(EFormationPlannerState::FormationMaintain);
            break;
        case EFormationPlannerState::FormationMaintain:
            if (bDwell && (Distance > Config.RejoinDistanceM ||
                           std::abs(D.CrossErrorM) > Config.CorridorCrossExitM ||
                           std::abs(D.CourseAlignmentErrorRad) > Config.AlignmentExitRad))
                Transition(EFormationPlannerState::FarRejoin);
            break;
        case EFormationPlannerState::AbortOrRecover:
            if (StateAgeS >= Config.AbortMinDwellS) Transition(EFormationPlannerState::FarRejoin);
            break;
        }
        if (StateAgeS > Config.StateTimeoutS && State != EFormationPlannerState::FormationMaintain)
            Transition(EFormationPlannerState::AbortOrRecover);
    }

    FFormationPathReference Candidate;
    if (State == EFormationPlannerState::AbortOrRecover)
        Candidate = MakeRecovery(F, S, L);
    else {
        const double AlignmentScale = 1.0 + std::abs(D.CourseAlignmentErrorRad) / Pi;
        D.EntryDistanceM = Clamp(Config.BaseEntryDistanceM + D.EstimatedBrakingDistanceM +
            D.MinimumTurnRadiusM * AlignmentScale + std::max(0.0, D.ClosingSpeedMps) * 2.0,
            Config.MinEntryDistanceM, Config.MaxEntryDistanceM);
        const FVector2 PredictedSlot = S.PositionNE + S.GroundVelocityNE * Config.PredictionTimeS;
        FVector2 Target = PredictedSlot;
        if (State == EFormationPlannerState::FarRejoin || State == EFormationPlannerState::CaptureCorridor)
            Target = PredictedSlot - T * D.EntryDistanceM; // corridor entry, never direct slot aim
        else if (State == EFormationPlannerState::SpeedMatch)
            Target = PredictedSlot - T * std::max(Config.MaintainAlongM, D.EstimatedBrakingDistanceM + L.SafetyDistanceM);

        const FVector2 FollowerT = FromCourse(F.GroundCourseRad);
        const double SegmentLength = (Target - F.PositionNE).Norm();
        const double U = Clamp(std::max(150.0, std::min(D.MinimumTurnRadiusM * 0.5, 600.0)) /
                               std::max(SegmentLength, 1.0), 0.05, 0.35);
        FHermiteSample H = SampleHermite(F.PositionNE, FollowerT, Target, T, U);
        if (!H.Valid) {
            Transition(EFormationPlannerState::AbortOrRecover);
            Candidate = MakeRecovery(F, S, L);
        } else {
            const double KMax = 1.0 / D.MinimumTurnRadiusM;
            H.K = Clamp(H.K, -KMax, KMax);
            Candidate.PathPositionN = H.P.N; Candidate.PathPositionE = H.P.E;
            Candidate.UnitTangentN = H.T.N; Candidate.UnitTangentE = H.T.E;
            Candidate.CurvaturePerM = H.K;
            Candidate.AltitudeReferenceM = S.AltitudeM;
            const double Remaining = D.RemainingCaptureDistanceM;
            // Estimate slot EAS from follower EAS and relative along-ground velocity. This keeps
            // EAS distinct from ground speed and assumes both aircraft share local along-track wind.
            const double EstimatedSlotEas = F.EquivalentAirspeedMps - D.RelativeAlongVelocityMps;
            double EasRef = EstimatedSlotEas;
            if (D.bDecelerationModelValid) {
                const double AllowedClosing = std::sqrt(std::max(0.0,
                    2.0 * L.AvailableDecelerationMps2 * Remaining));
                EasRef += AllowedClosing;
            } else if (D.ClosingSpeedMps > Config.MaintainRelativeSpeedMps) {
                D.bHighSpeedCaptureBlocked = true;
                Transition(EFormationPlannerState::AbortOrRecover);
                Candidate = MakeRecovery(F, S, L);
            }
            Candidate.AirspeedReferenceEasMps = Clamp(EasRef, L.MinEquivalentAirspeedMps, L.MaxEquivalentAirspeedMps);
            Candidate.PathGroundSpeedMps = S.GroundSpeedMps;
            Candidate.State = State;
            Candidate.bValid = true;
        }
    }

    // Receding-horizon continuity limiter: limits the generated path state itself, not a course command.
    if (Candidate.bValid && bHasPrevious && Previous.bValid) {
        FVector2 PrevP{Previous.PathPositionN, Previous.PathPositionE};
        FVector2 NewP{Candidate.PathPositionN, Candidate.PathPositionE};
        const FVector2 Delta = NewP - PrevP;
        const double MaxP = Config.PathPositionSlewMps * Dt;
        if (Delta.Norm() > MaxP) NewP = PrevP + Delta.Normalized() * MaxP;
        const double PrevA = std::atan2(Previous.UnitTangentE, Previous.UnitTangentN);
        const double NewA = std::atan2(Candidate.UnitTangentE, Candidate.UnitTangentN);
        const double A = PrevA + Clamp(WrapPi(NewA - PrevA), -Config.TangentSlewRadps * Dt, Config.TangentSlewRadps * Dt);
        Candidate.PathPositionN = NewP.N; Candidate.PathPositionE = NewP.E;
        Candidate.UnitTangentN = std::cos(A); Candidate.UnitTangentE = std::sin(A);
        Candidate.CurvaturePerM = Slew(Previous.CurvaturePerM, Candidate.CurvaturePerM,
                                       Config.CurvatureSlewPerMs * Dt);
        Candidate.AltitudeReferenceM = Slew(Previous.AltitudeReferenceM, Candidate.AltitudeReferenceM,
                                            Config.AltitudeSlewMps * Dt);
        Candidate.AirspeedReferenceEasMps = Slew(Previous.AirspeedReferenceEasMps,
            Candidate.AirspeedReferenceEasMps, Config.AirspeedSlewMps2 * Dt);
    }
    Candidate.State = State;
    D.PathCurvaturePerM = Candidate.CurvaturePerM;
    D.PlannerStateAgeS = StateAgeS;
    D.bPathInputValid = Candidate.bValid && std::isfinite(Candidate.PathPositionN) &&
        std::isfinite(Candidate.PathPositionE) && std::isfinite(Candidate.UnitTangentN) &&
        std::isfinite(Candidate.UnitTangentE) && std::isfinite(Candidate.CurvaturePerM) &&
        std::abs(std::sqrt(Candidate.UnitTangentN*Candidate.UnitTangentN + Candidate.UnitTangentE*Candidate.UnitTangentE)-1.0) < 1e-6;
    Candidate.bValid = D.bPathInputValid;
    if (Candidate.bValid) { Previous = Candidate; bHasPrevious = true; }
    return Candidate;
}
}
