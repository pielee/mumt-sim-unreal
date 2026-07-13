#pragma once

#include "FormationControl/Px4NpfgAdapter.h"
#include "FormationControl/Px4TecsAdapter.h"
#include "FormationControlV2/FormationNavigationTypes.h"
#include "FormationControlV2/PlannerV2Adapters.h"

#include <cstdint>
#include <memory>

namespace FormationControlV2 {

enum class EGuidanceFailureV2 : std::uint8_t {
    None, Disabled, ShadowDisabled, Paused, ResetFrame, InvalidFollower,
    InvalidPlannerDto, InvalidWind, InvalidTime, OriginMismatch,
    ResetMismatch, ZeroTangent, NonFiniteInput, NonFiniteOutput,
    // Appended LAST so every existing index stays stable for observers.
    InvalidConfig
};

// TECS vertical acceleration limit, from pinned PX4 v1.17.0
// (d6f12ad1c4f70ad3230afd7d86e971421e02fef4), parameter FW_T_VERT_ACC:
//   @unit m/s^2  @min 1.0  @max 10.0  default 7.0f
// TECS itself defaults this member to 0.0f, and TECSControl::_calcPitchControl derives the pitch
// slew rate from it (pitch_increment = dt * vert_accel_limit / max(tas, eps)). At 0 the slew bound
// collapses to [_pitch_setpoint, _pitch_setpoint], so the pitch setpoint can never leave its
// initial value -- which is why an unconfigured TECS reports PitchReference == 0 forever. The
// caller MUST set it; there is no safe silent fallback, so an out-of-range value is rejected.
inline constexpr double kTecsVerticalAccelLimitMinMps2 = 1.0;   // FW_T_VERT_ACC @min
inline constexpr double kTecsVerticalAccelLimitMaxMps2 = 10.0;  // FW_T_VERT_ACC @max

struct FGuidanceConfigV2 {
    double RollLimitRad{0.7853981633974483};
    double PitchMinRad{-0.5}, PitchMaxRad{0.5};
    double ThrottleMin{0.0}, ThrottleMax{1.0}, ThrottleTrim{0.5};
    double EasMinMps{10.0}, EasMaxMps{80.0};
    double MinimumGroundSpeedMps{0.0};
    double TargetClimbRateMps{10.0}, TargetSinkRateMps{10.0};
    double FastDescendAltitudeErrorM{100.0};
    double TecsVerticalAccelLimitMps2{7.0};   // FW_T_VERT_ACC default
    bool bDetectUnderspeed{true};
};

// Pure: the config contract the coordinator enforces before it will build a controller.
// Exposed so tests grade against the same predicate instead of re-deriving the bounds.
bool IsGuidanceConfigValid(const FGuidanceConfigV2 &config);

struct FGuidanceCoordinatorInputV2 {
    FCanonicalNavigationStateV2 Follower{};
    FFormationSlotStateV2 Slot{};
    FPlannerV2OutputAdapterResult PlannerDto{};
    double CurrentPitchRad{};
    bool bCurrentPitchValid{};
    double SimulationTimeS{}, DtS{};
    std::uint32_t ResetGeneration{}, OriginGeneration{};
    bool bControllerEnabled{true}, bShadowEnabled{true}, bPaused{};
};

struct FGuidanceCoordinatorOutputV2 {
    double RollReferenceRad{}, PitchReferenceRad{}, ThrottleReferenceNorm{};
    double LateralAccelerationFeedforwardMps2{}, LateralAccelerationFeedbackMps2{};
    double LateralAccelerationTotalMps2{}, CourseSetpointRad{}, WindFeasibility{};
    double UnderspeedRatio{}, FastDescendRatio{};
    double TotalEnergyRateSetpoint{}, TotalEnergyRateEstimate{};
    double EnergyBalanceRateSetpoint{}, EnergyBalanceRateEstimate{};
    double PitchIntegrator{}, ThrottleIntegrator{};
    double TimestampS{};
    std::uint32_t ResetGeneration{};
    bool bNpfgValid{}, bTecsValid{}, bCommandReady{};
    EGuidanceFailureV2 FailureReason{EGuidanceFailureV2::None};
};

class FormationGuidanceCoordinatorV2 {
public:
    FormationGuidanceCoordinatorV2();
    ~FormationGuidanceCoordinatorV2();
    FormationGuidanceCoordinatorV2(const FormationGuidanceCoordinatorV2 &) = delete;
    FormationGuidanceCoordinatorV2 &operator=(const FormationGuidanceCoordinatorV2 &) = delete;

    FGuidanceCoordinatorOutputV2 Update(const FGuidanceCoordinatorInputV2 &, const FGuidanceConfigV2 & = {});
    void Reset(std::uint32_t resetGeneration = 0);

private:
    void RecreateControllers(const FGuidanceConfigV2 &);
    std::unique_ptr<MumtPx4::FPx4NpfgAdapter> Npfg;
    std::unique_ptr<MumtPx4::FPx4TecsAdapter> Tecs;
    std::uint32_t LastResetGeneration{};
    double LastSimulationTimeS{};
    bool bInitialized{}, bWasPaused{};
};

} // namespace FormationControlV2
