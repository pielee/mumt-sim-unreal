#pragma once
#include "DubinsPath.h"
#include "MovingSlotPredictor.h"
#include "CaptureSpeedPlanner.h"
#include "SlotLocalPathPrimitiveV2.h"

namespace FormationControlV2 {
class FormationPlannerV2 {
public:
    FormationPlannerV2Config Config{};
    FormationPlannerV2Output Update(const FormationPlannerV2Input &input, FormationPlannerV2Diagnostics &diagnostics);
    void Reset(std::uint32_t resetGeneration=0);
    PlannerMode GetMode() const{return Mode;} double GetProgressS()const{return ProgressS;}
    std::uint64_t GetPathGeneration()const{return PathGeneration;} DubinsType GetActiveType()const{return ActiveDubinsType;}

    // Single source of truth for candidate build + cost + production filter.
    // Used by Replan() and by the near-field audit (the audit must NOT duplicate these
    // expressions). Pure: no planner state is read or written.
    static FCandidateEvaluationSetV2 EvaluateCandidates(const Pose2 &start, const Pose2 &terminal,
                                                        double rmin, double refSpeedMps, double alongM,
                                                        const FormationPlannerV2Config &config,
                                                        std::array<DubinsPath, 6> *outPaths = nullptr);
private:
    bool ValidateCritical(const FormationPlannerV2Input&)const;
    bool Replan(const FormationPlannerV2Input&,const PredictionResult&,double rmin,double along,FormationPlannerV2Diagnostics&);
    // sample==nullptr => no active path: only the path-independent geometry guards are evaluated.
    void UpdateGuards(const FormationPlannerV2Input&,double dt,const PathSample*,double rmin,FormationPlannerV2Diagnostics&);
    bool ShouldEvaluateCandidates(const FormationPlannerV2Input&,const Pose2&terminal,double speedMps)const;
    SlotLocalPathProjectionV2 BuildSlotLocalPath(const FormationPlannerV2Input&,double rmin)const;
    void Transition(PlannerMode,double now);
    FormationPlannerV2Output FreshInvalid(PlannerFailure failure)const;
    static bool IsCsc(DubinsType t){return t==DubinsType::LSL||t==DubinsType::RSR||t==DubinsType::LSR||t==DubinsType::RSL;}

    DubinsPath ActivePath{}; DubinsType ActiveDubinsType{DubinsType::Invalid}; double ProgressS{};
    Pose2 LastTerminalPose{}; double PathBuildTime{},LastReplanTime{-1e9},LastModeTransitionTime{};
    std::uint32_t LastResetGeneration{}; std::uint64_t PathGeneration{}; PlannerMode Mode{PlannerMode::Rejoin};
    std::array<double,(std::size_t)GuardTransition::Count> GuardDwell{}; double StateAge{},HeldPathAge{};
    bool bUsingHeldPath{},bInitialized{},bWasPaused{},bModeChangedPending{}; int ReplanCount{};
    // candidate evaluation / retry tracker
    int CandidateEvaluationCount{},ConsecutiveCandidateFailures{};
    bool bCandidateFeasible{},bHasLastEval{};
    double LastCandidateEvalTimeS{-1e9},LastEvalSpeedMps{-1e9},LastInputTimeS{};
    Pose2 LastEvalTerminal{}; PlannerMode LastEvalMode{PlannerMode::Rejoin}; std::uint32_t LastRejectMask{};
    // near-field Slot-local state (no Dubins projector/progress state is reused)
    double NearFieldTargetGroundSpeedMps{},NearFieldTargetEasMps{};
    std::uint64_t SlotPathGeneration{};
};
}
