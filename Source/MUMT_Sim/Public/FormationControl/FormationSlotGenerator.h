#pragma once

#include "FormationControlTypes.h"

namespace FormationControl
{
class FFormationSlotGenerator
{
public:
    double MaxLeaderStateAgeS = 0.5;
    double MinCourseSpeedMps = 10.0;
    double MaxDtS = 0.25;

    void Reset();
    FMovingSlotState Update(const FFormationLeaderState& Leader, const FFormationOffset& Offset, double DtS);

private:
    FVector2 LastTangent{1.0, 0.0};
    bool bHasLastTangent = false;
};
}
