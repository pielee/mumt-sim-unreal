#pragma once

#include "FormationControlV2/FormationNavigationTypes.h"
#include "FormationControlV2/MissionNavigationFrameV2.h"

namespace FormationControlV2 {

struct FNavigationRawSnapshotV2 {
    Vec3dV2 VehicleCgEcefFt{}, EcefVelocityFps{};
    double GeodeticLatitudeRad{}, LongitudeRad{}, GeodeticAltitudeFt{};
    double EquivalentAirspeedKts{}, TrueAirspeedFps{};
    Vec2 WindNEDFps{};
    double AltitudeAslFt{}, ClimbRateFps{}, SimulationTimeS{};
    bool bHolding{}, bValidFrame{};
};

struct FCanonicalNavigationAdapterV2Config {
    double MinimumGroundSpeedMps{1.0};
    double MinimumEasMps{1.0};
    double MinimumDerivativeDtS{1e-4};
    double MaximumDerivativeDtS{0.25};
};

struct FCanonicalNavigationTrackerV2 {
    double PreviousCourseRad{}, PreviousSimulationTimeS{};
    std::uint32_t PreviousResetGeneration{}, PreviousOriginGeneration{};
    bool bPreviousCourseValid{};
    void Reset() { *this = {}; }
};

class CanonicalNavigationAdapterV2 {
public:
    static FCanonicalNavigationStateV2 Convert(const FNavigationRawSnapshotV2 &raw,
        const MissionNavigationFrameV2 &frame, std::uint32_t resetGeneration,
        FCanonicalNavigationTrackerV2 &tracker,
        const FCanonicalNavigationAdapterV2Config &config = {});
};

} // namespace FormationControlV2
