#pragma once

#include "FormationControlV2/PlannerV2Types.h"

#include <cstdint>

namespace FormationControlV2 {

struct FCanonicalNavigationStateV2 {
    Vec2 PositionNE_m{};
    Vec2 GroundVelocityNE_mps{};
    double GroundCourse_rad{};
    double CourseRate_radps{};
    double Curvature_per_m{};
    double AltitudeAsl_m{};
    double ClimbRate_mps{};
    double SimulationTimeS{};
    std::uint32_t ResetGeneration{};
    std::uint32_t OriginGeneration{};
    double EquivalentAirspeed_mps{};
    double TrueAirspeed_mps{};
    Vec2 WindNE_mps{};
    double EasToTasRatio{1.0};
    bool bPaused{};

    bool bPositionValid{};
    bool bGroundVelocityValid{};
    bool bGroundCourseValid{};
    bool bCourseRateValid{};
    bool bCurvatureValid{};
    bool bAltitudeValid{};
    bool bClimbRateValid{};
    bool bSimulationTimeValid{};
    bool bEasValid{}, bTasValid{}, bWindValid{}, bRatioValid{}, bOriginValid{};
};

struct FFormationSlotCommandV2 {
    double FrontM{}; // Leader ground-course frame: positive forward.
    double RightM{}; // Leader ground-course frame: positive right.
    double UpM{};    // Inertial altitude-up, not a full body-frame offset.
    double CommandReceivedSimulationTimeS{};
    std::uint64_t SourceSequence{};
    bool bValid{};
};

enum class EFormationSlotFailureV2 : std::uint8_t {
    None,
    InvalidLeaderPosition,
    InvalidLeaderVelocity,
    InvalidLeaderCourse,
    InvalidLeaderTimestamp,
    InvalidCommand,
    InvalidCommandTimestamp,
    InvalidCurrentSimulationTime,
    FutureLeaderTimestamp,
    FutureCommandTimestamp,
    StaleLeader,
    StaleCommand,
    NonFiniteResult,
    LowSlotGroundSpeed
};

enum class EFormationSlotOmegaSourceV2 : std::uint8_t {
    UnavailableStraightAssumed,
    LeaderCourseRate,
    LeaderCurvature
};

struct FFormationSlotStateV2 {
    Vec2 PositionNE_m{};
    Vec2 GroundVelocityNE_mps{};
    Vec2 UnitTangentNE{};
    double GroundCourse_rad{};
    double Curvature_per_m{};
    double AltitudeAsl_m{};
    double ClimbRate_mps{};
    double StateTimestampS{};
    double StateAgeS{};
    std::uint32_t ResetGeneration{};
    std::uint32_t OriginGeneration{};
    std::uint64_t SourceSequence{};

    bool bPositionValid{};
    bool bGroundVelocityValid{};
    bool bUnitTangentValid{};
    bool bGroundCourseValid{};
    bool bCurvatureValid{};
    bool bAltitudeValid{};
    bool bClimbRateValid{};
    bool bTimestampValid{};
    bool bValid{};
    EFormationSlotFailureV2 FailureReason{EFormationSlotFailureV2::None};
    EFormationSlotOmegaSourceV2 OmegaSource{EFormationSlotOmegaSourceV2::UnavailableStraightAssumed};
    bool bStraightAssumed{};
};

} // namespace FormationControlV2
