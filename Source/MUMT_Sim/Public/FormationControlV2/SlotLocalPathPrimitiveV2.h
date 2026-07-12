#pragma once

#include "FormationControlV2/PlannerV2Types.h"

namespace FormationControlV2 {

enum class SlotLocalPathKindV2 : std::uint8_t { Invalid, Straight, ConstantCurvatureCircle };
enum class SlotLocalPathFailureV2 : std::uint8_t {
    None, InvalidInput, CurvatureUnavailable, CurvatureInfeasible, NearCircleCenter, NonFiniteResult
};

// ---------------------------------------------------------------------------------------------
// Authoritative turn-bound feasibility contract.
//
// SINGLE SOURCE OF TRUTH. The production primitive decides whether to reject a slot curvature by
// calling ClassifySlotCurvature, and the airborne acceptance tests classify their frame populations
// by calling the SAME function. Neither may re-derive Rmin, the bank limit, the safety factor or
// the comparison: a test that copies the expression can drift from production and would then be
// grading the planner against a different envelope than the one the planner actually enforces.
// ---------------------------------------------------------------------------------------------
enum class SlotCurvatureClassV2 : std::uint8_t {
    CurvatureUnavailable,   // the slot reported no usable curvature
    Feasible,               // |kappa| <= 1/Rmin - tolerance
    Boundary,               // within +/- tolerance of 1/Rmin: not gradeable either way
    Infeasible              // |kappa| >  1/Rmin + tolerance
};

// Radius epsilon of the PRODUCTION reject predicate (radius + eps < Rmin => reject). Unchanged.
inline constexpr double kSlotCurvatureRadiusEpsM = 1e-9;
// Relative half-width of the Boundary band around 1/Rmin (0.1% of the turn bound). Chosen so that
// Infeasible strictly implies the production predicate rejects, and Feasible strictly implies it
// does not — the Boundary band absorbs the ambiguity instead of mis-grading it.
inline constexpr double kSlotCurvatureRelToleranceV2 = 1e-3;

struct SlotCurvatureFeasibilityV2 {
    SlotCurvatureClassV2 Class{SlotCurvatureClassV2::CurvatureUnavailable};
    double SlotCurvaturePerM{};
    double SlotRadiusM{std::numeric_limits<double>::infinity()};
    double TurnBoundRadiusM{};              // Rmin, from the caller's authoritative turn bound
    double MaxFeasibleCurvaturePerM{};      // 1/Rmin
    double ToleranceCurvaturePerM{};        // half-width of the Boundary band
    bool   bCurvatureValid{};
    // The PRODUCTION predicate, verbatim: a finite slot circle tighter than the follower's turn
    // bound. This is what SlotLocalPathPrimitiveV2::Project acts on.
    bool   bRejectedByTurnBound{};
};

// Pure. rmin must be the planner's authoritative turn bound for the CURRENT actual speed.
SlotCurvatureFeasibilityV2 ClassifySlotCurvature(double slotCurvaturePerM, bool bSlotCurvatureValid,
                                                 double followerTurnRadiusBoundM,
                                                 double relTolerance = kSlotCurvatureRelToleranceV2);

struct SlotLocalPathInputV2 {
    Pose2 SlotPose{};
    double SlotCurvaturePerM{};
    bool bSlotCurvatureValid{};
    Vec2 FollowerPositionNE{};
    double FollowerTurnRadiusBoundM{};
    bool bAllowStraightAssumption{};
    std::uint64_t Generation{};
    double SimulationTimeS{};
};

struct SlotLocalPathProjectionV2 {
    PathSample Path{};
    Vec2 CircleCenterNE{};
    double CircleRadiusM{};
    double SignedCrossTrackErrorM{};
    double ProjectionDistanceM{};
    double LocalCoordinate{};
    std::uint64_t Generation{};
    double TimestampS{};
    SlotLocalPathKindV2 Kind{SlotLocalPathKindV2::Invalid};
    SlotLocalPathFailureV2 Failure{SlotLocalPathFailureV2::None};
    bool bStraightAssumed{};
    bool bValid{};
};

class SlotLocalPathPrimitiveV2 {
public:
    static SlotLocalPathProjectionV2 Project(const SlotLocalPathInputV2 &input);
};

enum class NearFieldSpeedFailureV2 : std::uint8_t {
    None, InvalidInput, InvalidWind, InvalidRatio, InvalidEnvelope, NonFiniteResult
};
struct NearFieldSpeedInputV2 {
    Vec2 SlotGroundVelocityNE{}, SlotUnitTangentNE{}, FollowerGroundVelocityNE{}, WindVelocityNE{};
    double AlongErrorM{}, EasToTasRatio{1.0}, DtS{};
    double ClosureTimeConstantS{8.0}, HoldTaperDistanceM{250.0}, MaxClosureMps{50.0};
    double AccelerationMps2{2.0}, DecelerationMps2{2.0}, MinTargetEasMps{50.0}, MaxTargetEasMps{350.0};
    bool bWindValid{}, bRatioValid{}, bEnvelopeValid{};
};
struct NearFieldSpeedOutputV2 {
    Vec2 TargetGroundVelocityNE{}, TargetAirVelocityNE{};
    double TargetGroundSpeedMps{}, TargetTasMps{}, TargetEasMps{}, ClosureDeltaMps{}, TaperRatio{};
    NearFieldSpeedFailureV2 Failure{NearFieldSpeedFailureV2::None};
    bool bValid{};
};
class NearFieldSpeedPlannerV2 {
public:
    static NearFieldSpeedOutputV2 Compute(const NearFieldSpeedInputV2 &input);
};

} // namespace FormationControlV2
