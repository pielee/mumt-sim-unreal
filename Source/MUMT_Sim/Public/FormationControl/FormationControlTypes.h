#pragma once

#include <cmath>
#include <cstdint>

// Phase 2B-only, engine-independent formation planning types.
// Horizontal frame: local NED North/East. Altitude and climb are Up-positive.
namespace FormationControl
{
constexpr double Pi = 3.14159265358979323846;
constexpr double GravityMps2 = 9.80665;

struct FVector2
{
    double N = 0.0;
    double E = 0.0;

    FVector2 operator+(const FVector2& O) const { return {N + O.N, E + O.E}; }
    FVector2 operator-(const FVector2& O) const { return {N - O.N, E - O.E}; }
    FVector2 operator*(double S) const { return {N * S, E * S}; }
    FVector2 operator/(double S) const { return {N / S, E / S}; }
    double Dot(const FVector2& O) const { return N * O.N + E * O.E; }
    double Cross(const FVector2& O) const { return N * O.E - E * O.N; }
    double Norm() const { return std::sqrt(N * N + E * E); }
    bool IsFinite() const { return std::isfinite(N) && std::isfinite(E); }
    FVector2 Normalized() const
    {
        const double L = Norm();
        return (std::isfinite(L) && L > 1e-9) ? *this / L : FVector2{};
    }
};

inline FVector2 FromCourse(double CourseRad) { return {std::cos(CourseRad), std::sin(CourseRad)}; }
inline double WrapPi(double A) { return std::remainder(A, 2.0 * Pi); }
inline double Clamp(double V, double Lo, double Hi) { return V < Lo ? Lo : (V > Hi ? Hi : V); }

struct FFormationLeaderState
{
    FVector2 PositionNE;
    double AltitudeM = 0.0;
    FVector2 GroundVelocityNE;
    double ClimbRateMps = 0.0;
    double GroundCourseRad = 0.0;
    double CourseRateRadps = 0.0; // positive = right/clockwise in N/E
    double CurvaturePerM = 0.0;   // positive = right turn
    FVector2 AccelerationNE;
    double VerticalAccelerationMps2 = 0.0;
    double StateAgeS = 0.0;
    bool bCourseValid = false;
    bool bCourseRateValid = false;
    bool bCurvatureValid = false;
    bool bAccelerationValid = false;
    bool bValid = false;
};

struct FFormationOffset
{
    double FrontM = 0.0;
    double RightM = 0.0;
    double UpM = 0.0;
};

struct FMovingSlotState
{
    FVector2 PositionNE;
    double AltitudeM = 0.0;
    FVector2 GroundVelocityNE;
    double VerticalVelocityMps = 0.0;
    FVector2 AccelerationNE;
    double VerticalAccelerationMps2 = 0.0;
    FVector2 UnitTangentNE;
    double CurvaturePerM = 0.0;
    double GroundSpeedMps = 0.0;
    double LeaderStateAgeS = 0.0;
    bool bAccelerationValid = false;
    bool bHeldLowSpeedCourse = false;
    bool bValid = false;
};

struct FFormationFollowerState
{
    FVector2 PositionNE;
    double AltitudeM = 0.0;
    FVector2 GroundVelocityNE;
    double ClimbRateMps = 0.0;
    double GroundCourseRad = 0.0;
    double TrueAirspeedMps = 0.0;
    double EquivalentAirspeedMps = 0.0;
    double AltitudeMarginM = 0.0;
    double AirspeedMarginMps = 0.0;
    double BankSaturationDurationS = 0.0;
    bool bGroundCourseValid = false;
    bool bNpfgValid = true;
    bool bTecsUnderspeed = false;
    bool bValid = false;
};

struct FFormationAircraftLimits
{
    double PlannerBankLimitRad = 45.0 * Pi / 180.0;
    double TurnRadiusSafetyFactor = 1.25;
    double MinEquivalentAirspeedMps = 120.0;
    double MaxEquivalentAirspeedMps = 335.0;
    double AvailableDecelerationMps2 = 0.0;
    double SafetyDistanceM = 150.0;
    bool bDecelerationModelValid = false;
};

enum class EFormationPlannerState : std::uint8_t
{
    FarRejoin,
    CaptureCorridor,
    SpeedMatch,
    FormationMaintain,
    AbortOrRecover
};

struct FFormationPlannerSafetyInput
{
    bool bPathInputValid = true;
    bool bSlotDistanceDiverging = false;
    bool bStateChatterDetected = false;
    bool bCourseReversalRisk = false;
};

struct FFormationPathReference
{
    double PathPositionN = 0.0;
    double PathPositionE = 0.0;
    double UnitTangentN = 1.0;
    double UnitTangentE = 0.0;
    double CurvaturePerM = 0.0;
    double AltitudeReferenceM = 0.0;
    double AirspeedReferenceEasMps = 0.0;
    double PathGroundSpeedMps = 0.0;
    EFormationPlannerState State = EFormationPlannerState::AbortOrRecover;
    bool bValid = false;
};

struct FFormationPlannerDiagnostics
{
    double AlongErrorM = 0.0;
    double CrossErrorM = 0.0;
    double VerticalErrorM = 0.0;
    double CourseAlignmentErrorRad = 0.0;
    double RelativeAlongVelocityMps = 0.0;
    double RelativeCrossVelocityMps = 0.0;
    double ClosingSpeedMps = 0.0;
    double RemainingCaptureDistanceM = 0.0;
    double EstimatedBrakingDistanceM = 0.0;
    double BrakingMarginM = 0.0;
    double MinimumTurnRadiusM = 0.0;
    double PathCurvaturePerM = 0.0;
    double PlannerStateAgeS = 0.0;
    double LeaderStateAgeS = 0.0;
    double DivergenceTimerS = 0.0;
    double EntryDistanceM = 0.0;
    bool bDecelerationModelValid = false;
    bool bHighSpeedCaptureBlocked = false;
    bool bPathInputValid = false;
};
}
