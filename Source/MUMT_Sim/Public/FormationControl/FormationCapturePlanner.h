#pragma once

#include "FormationControlTypes.h"

namespace FormationControl
{
class FFormationCapturePlanner
{
public:
    struct FConfig
    {
        double PredictionTimeS = 2.0;
        double BaseEntryDistanceM = 500.0;
        double MinEntryDistanceM = 300.0;
        double MaxEntryDistanceM = 5000.0;
        double CorridorCrossEnterM = 180.0;
        double CorridorCrossExitM = 300.0;
        double AlignmentEnterRad = 20.0 * Pi / 180.0;
        double AlignmentExitRad = 35.0 * Pi / 180.0;
        double SpeedMatchAlongM = 700.0;
        double MaintainAlongM = 120.0;
        double MaintainCrossM = 80.0;
        double MaintainVerticalM = 60.0;
        double MaintainRelativeSpeedMps = 12.0;
        double RejoinDistanceM = 900.0;
        double MinStateDwellS = 1.0;
        double StateTimeoutS = 45.0;
        double AbortMinDwellS = 3.0;
        double AbortAltitudeMarginM = 250.0;
        double AbortAirspeedMarginMps = 8.0;
        double BankSaturationTimeoutS = 4.0;
        double MaxLeaderStateAgeS = 0.5;
        double PathPositionSlewMps = 600.0;
        double TangentSlewRadps = 30.0 * Pi / 180.0;
        double CurvatureSlewPerMs = 0.002;
        double AltitudeSlewMps = 50.0;
        double AirspeedSlewMps2 = 25.0;
    } Config;

    void Reset();
    FFormationPathReference Update(
        const FFormationFollowerState& Follower,
        const FMovingSlotState& Slot,
        const FFormationAircraftLimits& Limits,
        const FFormationPlannerSafetyInput& Safety,
        double DtS,
        FFormationPlannerDiagnostics& Diagnostics);

    EFormationPlannerState GetState() const { return State; }

private:
    EFormationPlannerState State = EFormationPlannerState::FarRejoin;
    double StateAgeS = 0.0;
    double DivergenceTimerS = 0.0;
    FFormationPathReference Previous;
    bool bHasPrevious = false;

    void Transition(EFormationPlannerState Next);
    FFormationPathReference MakeRecovery(const FFormationFollowerState& Follower,
                                         const FMovingSlotState& Slot,
                                         const FFormationAircraftLimits& Limits) const;
};
}
