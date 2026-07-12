#include "FormationControlV2/FormationSlotGeneratorV2.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>

using namespace FormationControlV2;

namespace {

int failures = 0;
int checks = 0;

void Check(bool condition, const char *name)
{
    ++checks;
    if (!condition) {
        ++failures;
        std::cerr << "FAIL " << name << '\n';
    }
}

bool Near(double a, double b, double tolerance)
{
    return std::abs(a - b) <= tolerance;
}

bool NearVec(const Vec2 &a, const Vec2 &b, double tolerance)
{
    return (a - b).Norm() <= tolerance;
}

bool ZeroInvalid(const FFormationSlotStateV2 &state)
{
    return !state.bValid && !state.bPositionValid && !state.bGroundVelocityValid
        && !state.bUnitTangentValid && !state.bGroundCourseValid && !state.bCurvatureValid
        && !state.bAltitudeValid && !state.bClimbRateValid && !state.bTimestampValid
        && state.PositionNE_m.N == 0.0 && state.PositionNE_m.E == 0.0
        && state.GroundVelocityNE_mps.N == 0.0 && state.GroundVelocityNE_mps.E == 0.0
        && state.UnitTangentNE.N == 0.0 && state.UnitTangentNE.E == 0.0
        && state.GroundCourse_rad == 0.0 && state.Curvature_per_m == 0.0
        && state.AltitudeAsl_m == 0.0 && state.ClimbRate_mps == 0.0;
}

FCanonicalNavigationStateV2 Leader()
{
    FCanonicalNavigationStateV2 leader{};
    leader.PositionNE_m = {1000.0, -500.0};
    leader.GroundVelocityNE_mps = {100.0, 0.0};
    leader.GroundCourse_rad = 0.0;
    leader.CourseRate_radps = 0.02;
    leader.Curvature_per_m = 0.001;
    leader.AltitudeAsl_m = 3000.0;
    leader.ClimbRate_mps = 5.0;
    leader.SimulationTimeS = 10.0;
    leader.ResetGeneration = 7;
    leader.bPositionValid = leader.bGroundVelocityValid = leader.bGroundCourseValid = true;
    leader.bCourseRateValid = leader.bCurvatureValid = true;
    leader.bAltitudeValid = leader.bClimbRateValid = leader.bSimulationTimeValid = true;
    return leader;
}

FFormationSlotCommandV2 Command()
{
    FFormationSlotCommandV2 command{};
    command.FrontM = 100.0;
    command.RightM = 50.0;
    command.UpM = 20.0;
    command.CommandReceivedSimulationTimeS = 10.0;
    command.SourceSequence = 42;
    command.bValid = true;
    return command;
}

void Deterministic()
{
    const FFormationSlotGeneratorV2Config config{};
    for (const double course : {0.0, Pi / 2.0, -Pi, -Pi / 2.0}) {
        auto leader = Leader();
        leader.GroundCourse_rad = course;
        leader.GroundVelocityNE_mps = 100.0 * FromCourse(course);
        auto output = FormationSlotGeneratorV2::Calculate(leader, Command(), 10.1, config);
        const Vec2 expected = leader.PositionNE_m + FromCourse(course) * 100.0 + RightNormal(course) * 50.0;
        Check(output.bValid && NearVec(output.PositionNE_m, expected, 1e-9), "cardinal_course_geometry");
    }

    auto output = FormationSlotGeneratorV2::Calculate(Leader(), Command(), 10.1, config);
    Check(NearVec(output.PositionNE_m, {1100.0, -450.0}, 1e-9), "front_right_offset");
    Check(Near(output.AltitudeAsl_m, 3020.0, 1e-9) && Near(output.ClimbRate_mps, 5.0, 1e-9), "up_and_climb");
    Check(NearVec(output.GroundVelocityNE_mps, {99.0, 2.0}, 1e-9), "omega_cross_r_velocity_right_turn");
    Check(output.bCurvatureValid && Near(output.Curvature_per_m, 0.02 / std::sqrt(99.0 * 99.0 + 4.0), 1e-12), "slot_curvature_identity");
    Check(output.OmegaSource == EFormationSlotOmegaSourceV2::LeaderCourseRate && !output.bStraightAssumed, "course_rate_priority");
    Check(output.ResetGeneration == 7 && output.SourceSequence == 42 && Near(output.StateAgeS, 0.1, 1e-12), "metadata_passthrough");

    auto left = Leader();
    left.CourseRate_radps = -0.02;
    auto leftOutput = FormationSlotGeneratorV2::Calculate(left, Command(), 10.1, config);
    Check(leftOutput.bCurvatureValid && leftOutput.Curvature_per_m < 0.0, "left_turn_sign");
    Check(output.Curvature_per_m > 0.0, "right_turn_sign");

    auto zeroCommand = Command();
    zeroCommand.FrontM = zeroCommand.RightM = zeroCommand.UpM = 0.0;
    auto zero = FormationSlotGeneratorV2::Calculate(Leader(), zeroCommand, 10.1, config);
    Check(NearVec(zero.PositionNE_m, Leader().PositionNE_m, 1e-12), "zero_offset_position");
    Check(NearVec(zero.GroundVelocityNE_mps, Leader().GroundVelocityNE_mps, 1e-12), "zero_offset_velocity");

    auto large = Command();
    large.FrontM = 100000.0;
    large.RightM = -100000.0;
    auto largeOutput = FormationSlotGeneratorV2::Calculate(Leader(), large, 10.1, config);
    Check(largeOutput.bValid && largeOutput.PositionNE_m.IsFinite() && largeOutput.GroundVelocityNE_mps.IsFinite(), "large_offset_finite");

    auto curvatureFallback = Leader();
    curvatureFallback.bCourseRateValid = false;
    curvatureFallback.Curvature_per_m = 0.001;
    auto fallback = FormationSlotGeneratorV2::Calculate(curvatureFallback, zeroCommand, 10.1, config);
    Check(fallback.OmegaSource == EFormationSlotOmegaSourceV2::LeaderCurvature && Near(fallback.Curvature_per_m, 0.001, 1e-12), "curvature_fallback");

    auto straightLeader = Leader();
    straightLeader.bCourseRateValid = straightLeader.bCurvatureValid = false;
    auto straight = FormationSlotGeneratorV2::Calculate(straightLeader, Command(), 10.1, config);
    Check(straight.bValid && straight.bStraightAssumed && !straight.bCurvatureValid
          && straight.OmegaSource == EFormationSlotOmegaSourceV2::UnavailableStraightAssumed,
          "straight_assumed");
    Check(NearVec(straight.GroundVelocityNE_mps, straightLeader.GroundVelocityNE_mps, 1e-12), "straight_assumed_velocity");

    auto stopped = Leader();
    stopped.GroundVelocityNE_mps = {};
    stopped.CourseRate_radps = 0.0;
    auto low = FormationSlotGeneratorV2::Calculate(stopped, zeroCommand, 10.1, config);
    Check(low.bValid && low.bPositionValid && low.bGroundVelocityValid && low.bAltitudeValid,
          "low_speed_partial_validity");
    Check(!low.bUnitTangentValid && !low.bGroundCourseValid && !low.bCurvatureValid
          && low.FailureReason == EFormationSlotFailureV2::LowSlotGroundSpeed,
          "low_speed_direction_invalid");
    Check(low.UnitTangentNE.N == 0.0 && low.UnitTangentNE.E == 0.0 && low.GroundCourse_rad == 0.0,
          "low_speed_no_hold");

    auto staleLeader = Leader();
    staleLeader.SimulationTimeS = 9.0;
    auto invalid = FormationSlotGeneratorV2::Calculate(staleLeader, Command(), 10.1, config);
    Check(ZeroInvalid(invalid) && invalid.FailureReason == EFormationSlotFailureV2::StaleLeader, "stale_leader");
    auto staleCommand = Command();
    staleCommand.CommandReceivedSimulationTimeS = 8.0;
    invalid = FormationSlotGeneratorV2::Calculate(Leader(), staleCommand, 10.1, config);
    Check(ZeroInvalid(invalid) && invalid.FailureReason == EFormationSlotFailureV2::StaleCommand, "stale_command");
    auto futureLeader = Leader();
    futureLeader.SimulationTimeS = 10.2;
    invalid = FormationSlotGeneratorV2::Calculate(futureLeader, Command(), 10.1, config);
    Check(ZeroInvalid(invalid) && invalid.FailureReason == EFormationSlotFailureV2::FutureLeaderTimestamp, "future_leader_timestamp");
    auto futureCommand = Command();
    futureCommand.CommandReceivedSimulationTimeS = 10.2;
    invalid = FormationSlotGeneratorV2::Calculate(Leader(), futureCommand, 10.1, config);
    Check(ZeroInvalid(invalid) && invalid.FailureReason == EFormationSlotFailureV2::FutureCommandTimestamp, "future_command_timestamp");

    auto nanLeader = Leader();
    nanLeader.PositionNE_m.N = std::numeric_limits<double>::quiet_NaN();
    invalid = FormationSlotGeneratorV2::Calculate(nanLeader, Command(), 10.1, config);
    Check(ZeroInvalid(invalid) && invalid.FailureReason == EFormationSlotFailureV2::InvalidLeaderPosition, "nan_rejected");
    auto infCommand = Command();
    infCommand.RightM = std::numeric_limits<double>::infinity();
    invalid = FormationSlotGeneratorV2::Calculate(Leader(), infCommand, 10.1, config);
    Check(ZeroInvalid(invalid) && invalid.FailureReason == EFormationSlotFailureV2::InvalidCommand, "inf_rejected");

    auto rightCommand = zeroCommand;
    rightCommand.RightM = 80.0;
    auto mirrorCommand = rightCommand;
    mirrorCommand.RightM = -80.0;
    const auto rightSlot = FormationSlotGeneratorV2::Calculate(Leader(), rightCommand, 10.1, config);
    const auto leftSlot = FormationSlotGeneratorV2::Calculate(Leader(), mirrorCommand, 10.1, config);
    Check(NearVec((rightSlot.PositionNE_m + leftSlot.PositionNE_m) * 0.5, Leader().PositionNE_m, 1e-12), "left_right_position_symmetry");
    Check(NearVec((rightSlot.GroundVelocityNE_mps + leftSlot.GroundVelocityNE_mps) * 0.5, Leader().GroundVelocityNE_mps, 1e-12), "left_right_velocity_symmetry");

    const auto recovered = FormationSlotGeneratorV2::Calculate(Leader(), Command(), 10.1, config);
    Check(recovered.bValid && recovered.bPositionValid, "invalid_then_recovery");
    Check(ZeroInvalid(invalid), "invalid_has_no_stale_output");
}

struct ErrorStats {
    double Position{};
    double Velocity{};
    double TangentNorm{};
    double Course{};
    double Curvature{};
    double Altitude{};
    double Climb{};
};

void RandomAudit(std::uint64_t seed, int cases, ErrorStats &maxError)
{
    std::mt19937_64 random(seed);
    std::uniform_real_distribution<double> position(-1.0e6, 1.0e6);
    std::uniform_real_distribution<double> velocity(-350.0, 350.0);
    std::uniform_real_distribution<double> course(-Pi, Pi);
    std::uniform_real_distribution<double> rate(-0.25, 0.25);
    std::uniform_real_distribution<double> offset(-10000.0, 10000.0);
    std::uniform_real_distribution<double> altitude(-500.0, 20000.0);
    std::uniform_real_distribution<double> climb(-100.0, 100.0);

    for (int index = 0; index < cases; ++index) {
        auto leader = Leader();
        leader.PositionNE_m = {position(random), position(random)};
        leader.GroundVelocityNE_mps = {velocity(random), velocity(random)};
        leader.GroundCourse_rad = course(random);
        leader.CourseRate_radps = rate(random);
        leader.AltitudeAsl_m = altitude(random);
        leader.ClimbRate_mps = climb(random);
        leader.SimulationTimeS = 100.0;
        auto command = Command();
        command.FrontM = offset(random);
        command.RightM = offset(random);
        command.UpM = offset(random) * 0.1;
        command.CommandReceivedSimulationTimeS = 100.0;

        const auto output = FormationSlotGeneratorV2::Calculate(leader, command, 100.1);
        const Vec2 tangent = FromCourse(leader.GroundCourse_rad);
        const Vec2 right{-tangent.E, tangent.N};
        const Vec2 expectedOffset = tangent * command.FrontM + right * command.RightM;
        const Vec2 expectedPosition = leader.PositionNE_m + expectedOffset;
        const Vec2 expectedVelocity = leader.GroundVelocityNE_mps
            + leader.CourseRate_radps * Vec2{-expectedOffset.E, expectedOffset.N};
        const double speed = expectedVelocity.Norm();
        const double expectedCourse = WrapPi(std::atan2(expectedVelocity.E, expectedVelocity.N));
        const double expectedCurvature = leader.CourseRate_radps / speed;

        maxError.Position = std::max(maxError.Position, (output.PositionNE_m - expectedPosition).Norm());
        maxError.Velocity = std::max(maxError.Velocity, (output.GroundVelocityNE_mps - expectedVelocity).Norm());
        maxError.TangentNorm = std::max(maxError.TangentNorm, std::abs(output.UnitTangentNE.Norm() - 1.0));
        maxError.Course = std::max(maxError.Course, std::abs(WrapPi(output.GroundCourse_rad - expectedCourse)));
        maxError.Curvature = std::max(maxError.Curvature, std::abs(output.Curvature_per_m - expectedCurvature));
        maxError.Altitude = std::max(maxError.Altitude, std::abs(output.AltitudeAsl_m - (leader.AltitudeAsl_m + command.UpM)));
        maxError.Climb = std::max(maxError.Climb, std::abs(output.ClimbRate_mps - leader.ClimbRate_mps));

        bool finite = output.PositionNE_m.IsFinite() && output.GroundVelocityNE_mps.IsFinite()
            && output.UnitTangentNE.IsFinite() && Finite(output.GroundCourse_rad)
            && Finite(output.Curvature_per_m) && Finite(output.AltitudeAsl_m)
            && Finite(output.ClimbRate_mps) && Finite(output.StateTimestampS) && Finite(output.StateAgeS);
        if (!(output.bValid && output.bUnitTangentValid && output.bGroundCourseValid
              && output.bCurvatureValid && finite
              && maxError.Position <= 1e-9 && maxError.Velocity <= 1e-9
              && maxError.TangentNorm <= 1e-12 && maxError.Course <= 1e-10
              && maxError.Curvature <= 1e-12 && maxError.Altitude <= 1e-9
              && maxError.Climb <= 1e-9)) {
            ++failures;
            std::cerr << "RANDOM_FAIL index=" << index << '\n';
        }

        auto mirroredCommand = command;
        mirroredCommand.RightM = -command.RightM;
        auto centerCommand = command;
        centerCommand.RightM = 0.0;
        const auto mirrored = FormationSlotGeneratorV2::Calculate(leader, mirroredCommand, 100.1);
        const auto center = FormationSlotGeneratorV2::Calculate(leader, centerCommand, 100.1);
        if (!mirrored.bValid || !center.bValid
            || !NearVec((output.PositionNE_m + mirrored.PositionNE_m) * 0.5, center.PositionNE_m, 1e-9)
            || !NearVec((output.GroundVelocityNE_mps + mirrored.GroundVelocityNE_mps) * 0.5,
                        center.GroundVelocityNE_mps, 1e-9)) {
            ++failures;
            std::cerr << "RANDOM_SYMMETRY_FAIL index=" << index << '\n';
        }

        command.FrontM = std::numeric_limits<double>::quiet_NaN();
        const auto injected = FormationSlotGeneratorV2::Calculate(leader, command, 100.1);
        if (!ZeroInvalid(injected)) {
            ++failures;
            std::cerr << "INVALID_INJECTION_FAIL index=" << index << '\n';
        }
    }
}

} // namespace

int main()
{
    constexpr std::uint64_t seed = 0x534C4F545632ULL;
    constexpr int randomCases = 2500;
    Deterministic();
    ErrorStats maxError{};
    RandomAudit(seed, randomCases, maxError);
    std::cout << std::setprecision(12)
              << "FORMATION_SLOT_V2_AUDIT deterministic_checks=" << checks
              << " random_seed=" << seed
              << " random_cases=" << randomCases
              << " failures=" << failures
              << " max_position_error_m=" << maxError.Position
              << " max_velocity_error_mps=" << maxError.Velocity
              << " max_tangent_norm_error=" << maxError.TangentNorm
              << " max_course_error_rad=" << maxError.Course
              << " max_curvature_error_per_m=" << maxError.Curvature
              << " max_altitude_error_m=" << maxError.Altitude
              << " max_climb_error_mps=" << maxError.Climb << '\n';
    return failures == 0 ? 0 : 1;
}
