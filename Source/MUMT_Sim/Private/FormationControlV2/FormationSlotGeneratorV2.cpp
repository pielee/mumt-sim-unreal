#include "FormationControlV2/FormationSlotGeneratorV2.h"

#include <cmath>

namespace FormationControlV2 {
namespace {

FFormationSlotStateV2 Invalid(EFormationSlotFailureV2 failure)
{
    FFormationSlotStateV2 output{};
    output.FailureReason = failure;
    return output;
}

bool ValidConfig(const FFormationSlotGeneratorV2Config &config)
{
    return Finite(config.MaxLeaderStateAgeS) && config.MaxLeaderStateAgeS >= 0.0
        && Finite(config.MaxCommandAgeS) && config.MaxCommandAgeS >= 0.0
        && Finite(config.TimestampToleranceS) && config.TimestampToleranceS >= 0.0
        && Finite(config.MinSlotCourseSpeedMps) && config.MinSlotCourseSpeedMps >= 0.0
        && Finite(config.MinSlotCurvatureSpeedMps) && config.MinSlotCurvatureSpeedMps >= 0.0;
}

} // namespace

FFormationSlotStateV2 FormationSlotGeneratorV2::Calculate(
    const FCanonicalNavigationStateV2 &leader,
    const FFormationSlotCommandV2 &command,
    double currentSimulationTimeS,
    const FFormationSlotGeneratorV2Config &config)
{
    if (!ValidConfig(config) || !Finite(currentSimulationTimeS)) {
        return Invalid(EFormationSlotFailureV2::InvalidCurrentSimulationTime);
    }
    if (!leader.bPositionValid || !leader.PositionNE_m.IsFinite()) {
        return Invalid(EFormationSlotFailureV2::InvalidLeaderPosition);
    }
    if (!leader.bGroundVelocityValid || !leader.GroundVelocityNE_mps.IsFinite()) {
        return Invalid(EFormationSlotFailureV2::InvalidLeaderVelocity);
    }
    if (!leader.bGroundCourseValid || !Finite(leader.GroundCourse_rad)) {
        return Invalid(EFormationSlotFailureV2::InvalidLeaderCourse);
    }
    if (!leader.bSimulationTimeValid || !Finite(leader.SimulationTimeS)) {
        return Invalid(EFormationSlotFailureV2::InvalidLeaderTimestamp);
    }
    if (!command.bValid || !Finite(command.FrontM) || !Finite(command.RightM) || !Finite(command.UpM)) {
        return Invalid(EFormationSlotFailureV2::InvalidCommand);
    }
    if (!Finite(command.CommandReceivedSimulationTimeS)) {
        return Invalid(EFormationSlotFailureV2::InvalidCommandTimestamp);
    }
    if ((leader.bCourseRateValid && !Finite(leader.CourseRate_radps))
        || (leader.bCurvatureValid && !Finite(leader.Curvature_per_m))
        || (leader.bAltitudeValid && !Finite(leader.AltitudeAsl_m))
        || (leader.bClimbRateValid && !Finite(leader.ClimbRate_mps))) {
        return Invalid(EFormationSlotFailureV2::NonFiniteResult);
    }

    const double leaderAge = currentSimulationTimeS - leader.SimulationTimeS;
    const double commandAge = currentSimulationTimeS - command.CommandReceivedSimulationTimeS;
    if (leaderAge < -config.TimestampToleranceS) {
        return Invalid(EFormationSlotFailureV2::FutureLeaderTimestamp);
    }
    if (commandAge < -config.TimestampToleranceS) {
        return Invalid(EFormationSlotFailureV2::FutureCommandTimestamp);
    }
    if (leaderAge > config.MaxLeaderStateAgeS) {
        return Invalid(EFormationSlotFailureV2::StaleLeader);
    }
    if (commandAge > config.MaxCommandAgeS) {
        return Invalid(EFormationSlotFailureV2::StaleCommand);
    }

    const Vec2 tangent = FromCourse(leader.GroundCourse_rad);
    const Vec2 right{-tangent.E, tangent.N};
    const Vec2 offset = tangent * command.FrontM + right * command.RightM;

    double omega = 0.0;
    EFormationSlotOmegaSourceV2 omegaSource = EFormationSlotOmegaSourceV2::UnavailableStraightAssumed;
    bool straightAssumed = true;
    if (leader.bCourseRateValid && Finite(leader.CourseRate_radps)) {
        omega = leader.CourseRate_radps;
        omegaSource = EFormationSlotOmegaSourceV2::LeaderCourseRate;
        straightAssumed = false;
    } else if (leader.bCurvatureValid && Finite(leader.Curvature_per_m)) {
        omega = leader.Curvature_per_m * leader.GroundVelocityNE_mps.Norm();
        omegaSource = EFormationSlotOmegaSourceV2::LeaderCurvature;
        straightAssumed = false;
    }

    const Vec2 rotateRight{-offset.E, offset.N};
    const Vec2 slotPosition = leader.PositionNE_m + offset;
    const Vec2 slotVelocity = leader.GroundVelocityNE_mps + omega * rotateRight;
    const double slotGroundSpeed = slotVelocity.Norm();
    const double altitude = leader.AltitudeAsl_m + command.UpM;

    if (!Finite(omega) || !slotPosition.IsFinite() || !slotVelocity.IsFinite()
        || !Finite(slotGroundSpeed)
        || (leader.bAltitudeValid && !Finite(altitude))
        || (leader.bClimbRateValid && !Finite(leader.ClimbRate_mps))) {
        return Invalid(EFormationSlotFailureV2::NonFiniteResult);
    }

    FFormationSlotStateV2 output{};
    output.PositionNE_m = slotPosition;
    output.GroundVelocityNE_mps = slotVelocity;
    output.AltitudeAsl_m = leader.bAltitudeValid ? altitude : 0.0;
    output.ClimbRate_mps = leader.bClimbRateValid ? leader.ClimbRate_mps : 0.0;
    output.StateTimestampS = leader.SimulationTimeS;
    output.StateAgeS = leaderAge < 0.0 ? 0.0 : leaderAge;
    output.ResetGeneration = leader.ResetGeneration;
    output.OriginGeneration = leader.OriginGeneration;
    output.SourceSequence = command.SourceSequence;
    output.bPositionValid = true;
    output.bGroundVelocityValid = true;
    output.bAltitudeValid = leader.bAltitudeValid && Finite(leader.AltitudeAsl_m);
    output.bClimbRateValid = leader.bClimbRateValid && Finite(leader.ClimbRate_mps);
    output.bTimestampValid = true;
    output.bValid = true;
    output.OmegaSource = omegaSource;
    output.bStraightAssumed = straightAssumed;

    if (slotGroundSpeed >= config.MinSlotCourseSpeedMps) {
        output.UnitTangentNE = slotVelocity / slotGroundSpeed;
        output.GroundCourse_rad = WrapPi(std::atan2(output.UnitTangentNE.E, output.UnitTangentNE.N));
        output.bUnitTangentValid = true;
        output.bGroundCourseValid = true;
    }
    if (!straightAssumed && slotGroundSpeed >= config.MinSlotCurvatureSpeedMps) {
        output.Curvature_per_m = omega / slotGroundSpeed;
        output.bCurvatureValid = Finite(output.Curvature_per_m);
    }
    if (!output.bUnitTangentValid || !output.bGroundCourseValid) {
        output.FailureReason = EFormationSlotFailureV2::LowSlotGroundSpeed;
    }
    return output;
}

} // namespace FormationControlV2
