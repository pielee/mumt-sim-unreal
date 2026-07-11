#pragma once

#include "PlannerV2Types.h"
#include <array>
#include <vector>

namespace FormationControlV2 {

struct DubinsSegment {
    SegmentType Type{SegmentType::Straight};
    double LengthM{};
    double StartS{};
    Pose2 StartPose{};
};

struct DubinsCandidate {
    DubinsType Type{DubinsType::Invalid};
    std::array<double, 3> NormalizedLengths{};
    double LengthM{std::numeric_limits<double>::infinity()};
    bool bValid{};
};

struct ProjectionWindowConfig {
    double AdvanceFactor{2.0};
    double ForwardMarginM{50.0};
    double MinimumForwardWindowM{100.0};
    double MaximumForwardWindowM{2000.0};
    double MinimumDtS{0.0001};
    double MaximumDtS{0.25};
};

class DubinsPath {
public:
    static std::vector<DubinsCandidate> BuildCandidates(const Pose2 &start, const Pose2 &terminal, double radiusM);
    static double ComputeForwardSearchWindow(double groundSpeedMps, double dtS,
                                             const ProjectionWindowConfig &config, bool &valid);
    bool Build(const Pose2 &start, const Pose2 &terminal, double radiusM);
    bool BuildType(const Pose2 &start, const Pose2 &terminal, double radiusM, DubinsType type);
    PathSample Sample(double s) const;
    ProjectionResult Project(const Vec2 &position, double previousS, double backtrackToleranceM,
                             double forwardWindowM, bool enforceMonotonic = true,
                             double maximumDistanceM = std::numeric_limits<double>::infinity()) const;
    double GetLength() const { return LengthM; }
    double GetRadius() const { return RadiusM; }
    DubinsType GetType() const { return Type; }
    const std::array<DubinsSegment, 3> &GetSegments() const { return Segments; }
    std::array<double, 2> GetSegmentJunctions() const;
    bool IsValid() const { return bValid; }

private:
    bool BuildCandidate(const Pose2 &start, const Pose2 &terminal, double radiusM, const DubinsCandidate &candidate);
    static Pose2 Advance(const Pose2 &pose, SegmentType type, double distanceM, double radiusM);
    ProjectionResult ProjectSegment(const Vec2 &position, const DubinsSegment &segment,
                                    double localMin, double localMax) const;

    Pose2 Start{};
    Pose2 Terminal{};
    std::array<DubinsSegment, 3> Segments{};
    double RadiusM{};
    double LengthM{};
    DubinsType Type{DubinsType::Invalid};
    bool bValid{};
};

// Path-selection definitions reserved for the future stateful planner:
// HeadingError = abs(WrapPi(TerminalCourse - StartCourse)).
// TerminalRange = norm(TerminalPosition - StartPosition).
// ActiveEquivalent means a newly generated candidate of ActiveDubinsType for the
// same current start, terminal and Rmin; it is not the old path's remaining cost.

} // namespace FormationControlV2
