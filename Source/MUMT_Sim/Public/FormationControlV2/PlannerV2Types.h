#pragma once

#include <cmath>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace FormationControlV2 {

constexpr double Pi = 3.1415926535897932384626433832795;
constexpr double TwoPi = 2.0 * Pi;
constexpr double GravityMps2 = 9.80665;

inline bool Finite(double x) { return std::isfinite(x); }
inline double Clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double Mod2Pi(double x) { x = std::fmod(x, TwoPi); return x < 0.0 ? x + TwoPi : x; }
inline double WrapPi(double x) { return Mod2Pi(x + Pi) - Pi; }

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
struct ProjectionWindowConfig {
    double AdvanceFactor{2.0},ForwardMarginM{50.0},MinimumForwardWindowM{100.0},MaximumForwardWindowM{2000.0},MinimumDtS{0.0001},MaximumDtS{0.25};
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

enum class PlannerMode : std::uint8_t { Rejoin, CaptureEntry, ClosureTaper, SlotHold };
enum class GuardTransition : std::uint8_t {
    RejoinToCapture, CaptureToTaper, CaptureToRejoin, TaperToHold,
    TaperToCapture, TaperToRejoin, HoldToRejoin, Count
};
enum class PlannerFailure : std::uint8_t {
    None, Paused, AbnormalDt, CriticalInputInvalid, TurnBoundInvalid,
    PredictionRefreshFailed, HeldPathExpired, ProjectionFailed,
    CandidateSelectionFailed, RejoinTimeout, CaptureTimeout, TaperTimeout,
    InvalidWind, InvalidRatio, InvalidDeceleration
};
enum ReplanReason : std::uint32_t {
    ReplanNone=0, TerminalPositionChanged=1u<<0, TerminalHeadingChanged=1u<<1,
    PathDeviationExceeded=1u<<2, PathExpired=1u<<3, ProjectionFailed=1u<<4,
    ModeChanged=1u<<5, PredictionRefreshFailed=1u<<6,
    RemainingPathExhausted=1u<<7, HardCandidateInvalid=1u<<8,
    ResetGenerationChanged=1u<<9, NoActivePath=1u<<10
};

struct FormationPlannerV2Input {
    Vec2 FollowerPositionNE{};
    Vec2 FollowerGroundVelocityNE{};
    double FollowerCourseRad{};
    MovingSlotState Slot{};
    Vec2 WindVelocityNE{};
    double EasToTasRatio{1.0};
    double EquivalentAirspeedMps{};
    double SimulationTimeS{};
    double DtS{};
    std::uint32_t ResetGeneration{};
    bool bFollowerValid{};
    bool bCourseValid{};
    bool bWindValid{};
    bool bRatioValid{};
    bool bPaused{};
};

struct FormationPlannerV2Config {
    double PlannerBankLimitRad{45.0*Pi/180.0},MinimumTurnRadiusM{100.0},TurnRadiusSafetyFactor{1.25},PlanningSpeedFloorMps{30.0};
    double PredictionTimeS{2.0},MaximumSlotStateAgeS{0.5}; PredictionConfig Prediction{}; ProjectionWindowConfig Projection{};
    double ProjectionBacktrackM{20.0},ProjectionFailureDistanceM{150.0};
    double TerminalPositionReplanM{25.0},TerminalHeadingReplanRad{2.0*Pi/180.0},PathExpirationS{5.0},MinimumReplanIntervalS{0.5},RemainingPathReplanM{50.0};
    double MaxHeldPathAgeS{2.0},HeldPathMinimumRemainingM{100.0};
    double MaxPathLengthM{50000.0},MaxCaptureTimeS{120.0},ReferenceGroundSpeedFloorMps{30.0};
    double CaptureTimeWeight{1.0},CccPenaltyS{4.0},CccAbsoluteAdvantageS{3.0},CccRelativeAdvantage{0.15},CccMinimumHeadingErrorRad{120.0*Pi/180.0},CccMaximumRangeRadiusFactor{6.0};
    double TypeSwitchAbsoluteAdvantageS{2.0},TypeSwitchRelativeAdvantage{0.10};
    double DesiredClosureMps{25.0},MaxClosureMps{50.0},SafetyDistanceM{150.0},ConfiguredDecelerationMps2{2.0},MinTargetEasMps{50.0},MaxTargetEasMps{350.0};
    bool bDecelerationEnvelopeValid{true};
    double RejoinTimeoutS{0.0},CaptureEntryTimeoutS{30.0},ClosureTaperTimeoutS{20.0};
};

struct PlannerCandidateCost {
    DubinsType Type{DubinsType::Invalid}; double TransitTimeS{},ClosureTimeS{},CccPenaltyS{},HeadingExcursionPenaltyS{},TerminalClosurePenaltyS{},TotalS{std::numeric_limits<double>::infinity()}; bool bCsc{},bValid{};
};
struct FormationPlannerV2Diagnostics {
    double AlongErrorM{},CrossErrorM{},RangeM{},PathHeadingErrorRad{},SlotHeadingErrorRad{},RelativeAlongSpeedMps{};
    double RminM{},ProjectionDistanceM{},StateAgeS{},HeldPathAgeS{},RegeneratedActiveTypeCostS{std::numeric_limits<double>::infinity()},ActivePathRemainingCostS{std::numeric_limits<double>::infinity()};
    std::array<double,(std::size_t)GuardTransition::Count> GuardDwellS{};
    std::array<PlannerCandidateCost,6> CandidateCosts{};
    std::uint32_t ReplanReasons{}; int ReplanCount{}; DubinsType SelectedType{DubinsType::Invalid}; bool bUsingHeldPath{},bCccEligible{},bTypeSwitched{};
};
struct FormationPlannerV2Output {
    PathSample Path{}; double TargetEasMps{}; PlannerMode Mode{PlannerMode::Rejoin}; PlannerFailure Failure{PlannerFailure::None};
    std::uint64_t PathGeneration{}; double ProgressS{}; bool bPathValid{},bTargetEasValid{},bValid{};
};

} // namespace FormationControlV2
