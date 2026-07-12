#pragma once

#include "FormationControlV2/FormationNavigationTypes.h"
#include "FormationControlV2/PlannerV2Types.h"

namespace FormationControlV2 {

struct FPlannerV2InputAdapterConfig { double MaxFollowerAgeS{0.5}, MaxSlotAgeS{0.5}; };
struct FPlannerV2InputAdapterRequest {
    FCanonicalNavigationStateV2 Follower{};
    FFormationSlotStateV2 Slot{};
    double CurrentSimulationTimeS{}, DtS{};
};
struct FPlannerV2InputAdapterResult { FormationPlannerV2Input Input{}; bool bValid{}; };

struct FNpfgPathInputV2 {
    Vec2 PathPositionNE_m{}, PathUnitTangentNE{};
    double PathCurvature_per_m{};
    bool bValid{};
};
struct FTecsSetpointInputV2 {
    double TargetEasMps{}, TargetAltitudeAslM{}, TargetClimbRateMps{};
    bool bTargetEasValid{}, bTargetAltitudeValid{}, bTargetClimbRateValid{}, bCommandReady{};
};
struct FPlannerV2OutputAdapterResult { FNpfgPathInputV2 Npfg{}; FTecsSetpointInputV2 Tecs{}; };

class PlannerV2InputAdapter {
public: static FPlannerV2InputAdapterResult Build(const FPlannerV2InputAdapterRequest&, const FPlannerV2InputAdapterConfig& = {});
};
class PlannerV2OutputAdapter {
public: static FPlannerV2OutputAdapterResult Build(const FormationPlannerV2Output&, const FFormationSlotStateV2&,
                                                    const FCanonicalNavigationStateV2&);
};
} // namespace FormationControlV2
