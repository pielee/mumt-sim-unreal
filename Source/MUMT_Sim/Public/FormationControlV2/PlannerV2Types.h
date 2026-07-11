#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace FormationControlV2 {

constexpr double Pi = 3.1415926535897932384626433832795;
constexpr double TwoPi = 2.0 * Pi;
constexpr double GravityMps2 = 9.80665;

inline bool Finite(double x) { return std::isfinite(x); }
inline double Clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double WrapPi(double x) { return std::remainder(x, TwoPi); }
inline double Mod2Pi(double x) { x = std::fmod(x, TwoPi); return x < 0.0 ? x + TwoPi : x; }

struct Vec2 {
    double N{};
    double E{};
    Vec2 operator+(const Vec2 &o) const { return {N + o.N, E + o.E}; }
    Vec2 operator-(const Vec2 &o) const { return {N - o.N, E - o.E}; }
    Vec2 operator*(double s) const { return {N * s, E * s}; }
    Vec2 operator/(double s) const { return {N / s, E / s}; }
    double Dot(const Vec2 &o) const { return N * o.N + E * o.E; }
    double Cross(const Vec2 &o) const { return N * o.E - E * o.N; }
    double NormSquared() const { return Dot(*this); }
    double Norm() const { return std::sqrt(NormSquared()); }
    bool IsFinite() const { return Finite(N) && Finite(E); }
    Vec2 Normalized() const { const double n = Norm(); return n > 1e-12 && Finite(n) ? *this / n : Vec2{}; }
};
inline Vec2 operator*(double s, const Vec2 &v) { return v * s; }
inline Vec2 FromCourse(double course) { return {std::cos(course), std::sin(course)}; }
inline Vec2 RightNormal(double course) { return {-std::sin(course), std::cos(course)}; }

struct Pose2 {
    Vec2 Position{};
    double CourseRad{}; // North=0, clockwise/right positive, wrapped by consumers.
    bool IsFinite() const { return Position.IsFinite() && Finite(CourseRad); }
};

enum class DubinsType : std::uint8_t { LSL, RSR, LSR, RSL, RLR, LRL, Invalid };
enum class SegmentType : std::uint8_t { LeftArc, Straight, RightArc };

struct PathSample {
    Vec2 Position{};
    Vec2 UnitTangent{};
    double SignedCurvaturePerM{}; // left negative, right positive
    double S{};
    int SegmentIndex{-1};
    bool bValid{};
};

enum class ProjectionFailure : std::uint8_t {
    None, InvalidInput, InvalidWindow, NoCandidate, ExcessiveDistance
};
struct ProjectionResult {
    double S{};
    double DistanceM{};
    int SegmentIndex{-1};
    ProjectionFailure Failure{ProjectionFailure::None};
    bool bValid{};
};

enum class PredictionModel : std::uint8_t { Straight, ConstantCurvature, StraightAssumedCurvatureUnavailable };
enum class PredictionFailure : std::uint8_t {
    None, InvalidInput, InvalidKinematics, NonFiniteTime, TimeCapReached,
    IterationLimitReached, NotConverged, NonFiniteTerminal
};
struct MovingSlotState {
    Pose2 Pose{};
    Vec2 GroundVelocityNE{};
    double GroundSpeedMps{};
    double CurvaturePerM{};
    bool bCurvatureValid{};
    bool bValid{};
};
struct PredictionConfig {
    int MaxIterations{3};
    double PositionToleranceM{1.0};
    double HeadingToleranceRad{0.5 * Pi / 180.0};
    double TimeToleranceS{0.05};
    double PredictionTimeCapS{15.0};
    double MinimumClosingGroundSpeedMps{30.0};
    double StraightAssumptionMaxTimeS{2.0};
    bool bAllowStraightWithoutCurvature{true};
};
struct PredictionResult {
    Pose2 TerminalPose{};
    Vec2 TerminalGroundVelocityNE{};
    double PredictionTimeS{};
    int Iterations{};
    PredictionModel Model{PredictionModel::Straight};
    PredictionFailure Failure{PredictionFailure::None};
    bool bConverged{};
    bool bValid{};
};

enum class SpeedPlanFailure : std::uint8_t {
    None, InvalidInput, InvalidWind, InvalidEasToTasRatio, InvalidDecelerationEnvelope
};
struct CaptureSpeedInput {
    Vec2 PredictedSlotGroundVelocityNE{};
    Vec2 PathUnitTangentNE{1.0, 0.0};
    Vec2 WindVelocityNE{};
    double EasToTasRatio{1.0};
    double DesiredCaptureClosureMps{};
    double RemainingAlongDistanceM{};
    double SafetyDistanceM{};
    double ConfiguredDecelerationMps2{};
    double MaxClosureSpeedMps{};
    double MinTargetEasMps{};
    double MaxTargetEasMps{};
    bool bWindValid{};
    bool bRatioValid{};
    bool bDecelerationEnvelopeValid{};
};
struct CaptureSpeedOutput {
    Vec2 DesiredGroundVelocityNE{};
    Vec2 DesiredAirVelocityNE{};
    double CommandedClosureMps{};
    double TargetTasMps{};
    double TargetEasMps{};
    SpeedPlanFailure Failure{SpeedPlanFailure::None};
    bool bTargetEasValid{};
};

} // namespace FormationControlV2
