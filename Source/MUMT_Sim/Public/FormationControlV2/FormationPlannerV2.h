#pragma once
#include "DubinsPath.h"
#include "MovingSlotPredictor.h"
#include "CaptureSpeedPlanner.h"

namespace FormationControlV2 {
class FormationPlannerV2 {
public:
    FormationPlannerV2Config Config{};
    FormationPlannerV2Output Update(const FormationPlannerV2Input &input, FormationPlannerV2Diagnostics &diagnostics);
    void Reset(std::uint32_t resetGeneration=0);
    PlannerMode GetMode() const{return Mode;} double GetProgressS()const{return ProgressS;}
    std::uint64_t GetPathGeneration()const{return PathGeneration;} DubinsType GetActiveType()const{return ActiveDubinsType;}
private:
    bool ValidateCritical(const FormationPlannerV2Input&)const;
    bool Replan(const FormationPlannerV2Input&,const PredictionResult&,double rmin,double along,FormationPlannerV2Diagnostics&);
    void UpdateGuards(const FormationPlannerV2Input&,double dt,const PathSample&,FormationPlannerV2Diagnostics&);
    void Transition(PlannerMode,double now);
    FormationPlannerV2Output FreshInvalid(PlannerFailure failure)const;
    static bool IsCsc(DubinsType t){return t==DubinsType::LSL||t==DubinsType::RSR||t==DubinsType::LSR||t==DubinsType::RSL;}

    DubinsPath ActivePath{}; DubinsType ActiveDubinsType{DubinsType::Invalid}; double ProgressS{};
    Pose2 LastTerminalPose{}; double PathBuildTime{},LastReplanTime{-1e9},LastModeTransitionTime{};
    std::uint32_t LastResetGeneration{}; std::uint64_t PathGeneration{}; PlannerMode Mode{PlannerMode::Rejoin};
    std::array<double,(std::size_t)GuardTransition::Count> GuardDwell{}; double StateAge{},HeldPathAge{};
    bool bUsingHeldPath{},bInitialized{},bWasPaused{},bModeChangedPending{}; int ReplanCount{};
};
}
