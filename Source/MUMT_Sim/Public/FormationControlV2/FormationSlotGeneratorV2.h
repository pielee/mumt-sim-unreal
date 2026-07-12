#pragma once

#include "FormationControlV2/FormationNavigationTypes.h"

namespace FormationControlV2 {

struct FFormationSlotGeneratorV2Config {
    double MaxLeaderStateAgeS{0.5};
    double MaxCommandAgeS{1.0};
    double TimestampToleranceS{1e-9};
    double MinSlotCourseSpeedMps{1.0};
    double MinSlotCurvatureSpeedMps{1.0};
};

class FormationSlotGeneratorV2 {
public:
    static FFormationSlotStateV2 Calculate(
        const FCanonicalNavigationStateV2 &leader,
        const FFormationSlotCommandV2 &command,
        double currentSimulationTimeS,
        const FFormationSlotGeneratorV2Config &config = {});
};

} // namespace FormationControlV2
