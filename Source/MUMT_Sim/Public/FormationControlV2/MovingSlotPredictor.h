#pragma once
#include "PlannerV2Types.h"

namespace FormationControlV2 {
class MovingSlotPredictor {
public:
    static PredictionResult PredictAtTime(const MovingSlotState &slot, double timeS, const PredictionConfig &config);
    static PredictionResult SolveCaptureTime(const MovingSlotState &slot, const Vec2 &followerPosition,
                                              double initialTimeS, double captureGroundSpeedMps,
                                              const PredictionConfig &config);
};
}
