#include "FormationControlV2/FormationPlannerV2.h"
#include <algorithm>

namespace FormationControlV2 {
void FormationPlannerV2::Reset(std::uint32_t gen){ActivePath={};ActiveDubinsType=DubinsType::Invalid;ProgressS=0;LastTerminalPose={};PathBuildTime=0;LastReplanTime=-1e9;LastModeTransitionTime=0;LastResetGeneration=gen;PathGeneration=0;Mode=PlannerMode::Rejoin;GuardDwell={};StateAge=0;HeldPathAge=0;bUsingHeldPath=false;bInitialized=true;bWasPaused=false;bModeChangedPending=false;ReplanCount=0;
 // candidate retry tracker + near-field deceleration state
 CandidateEvaluationCount=0;ConsecutiveCandidateFailures=0;bCandidateFeasible=false;bHasLastEval=false;
 LastCandidateEvalTimeS=-1e9;LastEvalSpeedMps=-1e9;LastInputTimeS=0;LastEvalTerminal={};LastEvalMode=PlannerMode::Rejoin;LastRejectMask=0;
 NearFieldTargetGroundSpeedMps=0;NearFieldTargetEasMps=0;SlotPathGeneration=0;}
FormationPlannerV2Output FormationPlannerV2::FreshInvalid(PlannerFailure f)const{FormationPlannerV2Output o;o.Mode=Mode;o.Failure=f;o.PathGeneration=PathGeneration;return o;}
bool FormationPlannerV2::ValidateCritical(const FormationPlannerV2Input&i)const{
 return i.bFollowerValid&&i.bCourseValid&&i.FollowerPositionNE.IsFinite()&&i.FollowerGroundVelocityNE.IsFinite()&&Finite(i.FollowerCourseRad)&&
 i.Slot.bValid&&i.Slot.Pose.IsFinite()&&i.Slot.GroundVelocityNE.IsFinite()&&Finite(i.Slot.GroundSpeedMps)&&i.Slot.GroundSpeedMps>=0&&
 Finite(i.SimulationTimeS)&&Finite(Config.PlannerBankLimitRad)&&Config.PlannerBankLimitRad>1e-6&&Config.PlannerBankLimitRad<Pi/2&&
 Finite(Config.MinimumTurnRadiusM)&&Config.MinimumTurnRadiusM>0&&Finite(Config.TurnRadiusSafetyFactor)&&Config.TurnRadiusSafetyFactor>=1&&
 Finite(Config.PlanningSpeedFloorMps)&&Config.PlanningSpeedFloorMps>0;
}
void FormationPlannerV2::Transition(PlannerMode next,double now){if(next!=Mode){Mode=next;StateAge=0;LastModeTransitionTime=now;GuardDwell={};bModeChangedPending=true;
 // Slot-local modes never reuse a finite Dubins path/projector.
 if(next==PlannerMode::NearFieldSlotTrack||next==PlannerMode::SlotHold||next==PlannerMode::ClosureTaper){
  ActivePath={};ActiveDubinsType=DubinsType::Invalid;ProgressS=0;bUsingHeldPath=false;HeldPathAge=0;PathGeneration++;SlotPathGeneration++;}
 if(next!=PlannerMode::NearFieldSlotTrack&&next!=PlannerMode::SlotHold&&next!=PlannerMode::ClosureTaper){NearFieldTargetGroundSpeedMps=0;NearFieldTargetEasMps=0;}}}

SlotLocalPathProjectionV2 FormationPlannerV2::BuildSlotLocalPath(const FormationPlannerV2Input&i,double rmin)const{
 SlotLocalPathInputV2 in{};in.SlotPose=i.Slot.Pose;in.SlotCurvaturePerM=i.Slot.CurvaturePerM;
 in.bSlotCurvatureValid=i.Slot.bCurvatureValid;in.FollowerPositionNE=i.FollowerPositionNE;
 in.FollowerTurnRadiusBoundM=rmin;in.bAllowStraightAssumption=Config.bAllowNearFieldStraightAssumption;
 in.Generation=SlotPathGeneration;in.SimulationTimeS=i.SimulationTimeS;return SlotLocalPathPrimitiveV2::Project(in);
}

// Throttle candidate re-evaluation once it has failed: identical inputs + identical failure must
// not be re-evaluated every frame. Any real change (mode, speed, slot terminal, retry interval)
// re-opens it immediately.
bool FormationPlannerV2::ShouldEvaluateCandidates(const FormationPlannerV2Input&i,const Pose2&terminal,double speedMps)const{
 if(!bHasLastEval||bCandidateFeasible)return true;                       // never throttle the healthy path
 if(Mode!=LastEvalMode)return true;
 if(std::abs(speedMps-LastEvalSpeedMps)>=Config.CandidateRetrySpeedDeltaMps)return true;
 if((terminal.Position-LastEvalTerminal.Position).Norm()>=Config.CandidateRetryTerminalDeltaM)return true;
 if(std::abs(WrapPi(terminal.CourseRad-LastEvalTerminal.CourseRad))>=Config.CandidateRetryTerminalHeadingRad)return true;
 if(i.SimulationTimeS-LastCandidateEvalTimeS>=Config.CandidateRetryIntervalS)return true;
 return false;
}

// Pure candidate evaluation: build each family, cost it, and apply the production filter.
// Both Replan() and the near-field audit go through this; the expressions live only here.
FCandidateEvaluationSetV2 FormationPlannerV2::EvaluateCandidates(const Pose2&start,const Pose2&terminal,
  double rmin,double refSpeedMps,double alongM,const FormationPlannerV2Config&config,std::array<DubinsPath,6>*outPaths){
 FCandidateEvaluationSetV2 set{};
 for(int x=0;x<6;x++){set.Evaluations[x].Type=(DubinsType)x;set.Evaluations[x].RejectReason=CandidateRejectReason::BuildFailed;}
 const double headingError=std::abs(WrapPi(terminal.CourseRad-start.CourseRad));
 const double refSpeed=std::max(config.ReferenceGroundSpeedFloorMps,refSpeedMps);
 auto raw=DubinsPath::BuildCandidates(start,terminal,rmin);
 for(const auto&c:raw){const int idx=(int)c.Type;DubinsPath p;if(!p.BuildType(start,terminal,rmin,c.Type))continue;
  auto &e=set.Evaluations[idx];e.bBuildSucceeded=true;
  const double transit=p.GetLength()/refSpeed;const double closureDistance=std::max(0.0,-alongM-config.SafetyDistanceM);
  const double totalClosure=closureDistance/std::max(config.DesiredClosureMps,1.0);const double closure=std::max(0.0,totalClosure-transit);
  const double excursion=std::max(0.0,(p.GetSegments()[0].LengthM+p.GetSegments()[2].LengthM)/rmin-headingError);
  const double headingPenalty=std::max(0.0,excursion-Pi)/(Pi/4)*2.0;
  const double terminalPenalty=config.bDecelerationEnvelopeValid?0.0:std::numeric_limits<double>::infinity();
  auto &cost=set.Costs[idx];cost.Type=c.Type;cost.TransitTimeS=transit;cost.ClosureTimeS=closure;
  cost.CccPenaltyS=IsCsc(c.Type)?0:config.CccPenaltyS;cost.HeadingExcursionPenaltyS=headingPenalty;cost.TerminalClosurePenaltyS=terminalPenalty;
  cost.TotalS=transit+config.CaptureTimeWeight*closure+cost.CccPenaltyS+headingPenalty+terminalPenalty;cost.bCsc=IsCsc(c.Type);
  cost.bValid=Finite(cost.TotalS)&&p.GetLength()<=config.MaxPathLengthM&&transit+closure<=config.MaxCaptureTimeS;
  e.PathLengthM=p.GetLength();e.TransitTimeS=transit;e.ClosureTimeS=closure;e.TotalTimeS=transit+closure;e.TotalCostS=cost.TotalS;e.bValid=cost.bValid;
  // exact reject reason (priority: envelope -> non-finite -> length -> capture time)
  if(cost.bValid)e.RejectReason=CandidateRejectReason::None;
  else if(!config.bDecelerationEnvelopeValid)e.RejectReason=CandidateRejectReason::DecelerationEnvelopeInvalid;
  else if(!Finite(cost.TotalS))e.RejectReason=CandidateRejectReason::NonFiniteCost;
  else if(p.GetLength()>config.MaxPathLengthM)e.RejectReason=CandidateRejectReason::PathLengthExceeded;
  else if(transit+closure>config.MaxCaptureTimeS)e.RejectReason=CandidateRejectReason::CaptureTimeExceeded;
  else e.RejectReason=CandidateRejectReason::NonFiniteCost;
  if(outPaths)(*outPaths)[idx]=p;}
 for(int x=0;x<6;x++){const auto&e=set.Evaluations[x];if(!e.bBuildSucceeded)set.BuildFailedCount++;
  if(e.bValid)set.ValidCount++;else set.RejectMask|=(1u<<x);}
 return set;
}

bool FormationPlannerV2::Replan(const FormationPlannerV2Input&i,const PredictionResult&pred,double rmin,double along,FormationPlannerV2Diagnostics&d){
 LastReplanTime=i.SimulationTimeS;ReplanCount++;const Pose2 start{i.FollowerPositionNE,i.FollowerCourseRad};const Pose2 terminal=pred.TerminalPose;
 const double refSpeed=std::max(Config.ReferenceGroundSpeedFloorMps,i.FollowerGroundVelocityNE.Norm());const double headingError=std::abs(WrapPi(terminal.CourseRad-start.CourseRad));const double range=(terminal.Position-start.Position).Norm();
 std::array<DubinsPath,6> paths{};
 const FCandidateEvaluationSetV2 set=EvaluateCandidates(start,terminal,rmin,i.FollowerGroundVelocityNE.Norm(),along,Config,&paths);
 d.CandidateCosts=set.Costs;d.CandidateEvaluationCount=++CandidateEvaluationCount;d.LastCandidateRejectMask=set.RejectMask;
 // retry tracker: remember exactly what was evaluated, so an identical retry can be suppressed.
 bHasLastEval=true;LastCandidateEvalTimeS=i.SimulationTimeS;LastEvalSpeedMps=i.FollowerGroundVelocityNE.Norm();
 LastEvalTerminal=terminal;LastEvalMode=Mode;LastRejectMask=set.RejectMask;bCandidateFeasible=set.ValidCount>0;
 ConsecutiveCandidateFailures=bCandidateFeasible?0:ConsecutiveCandidateFailures+1;
 std::array<bool,6> valid{};for(int x=0;x<6;x++)valid[x]=set.Evaluations[x].bValid;
 (void)refSpeed;(void)headingError;
 int bestCsc=-1,bestCcc=-1;for(int x=0;x<6;x++)if(valid[x]){if(IsCsc((DubinsType)x)){if(bestCsc<0||d.CandidateCosts[x].TotalS<d.CandidateCosts[bestCsc].TotalS)bestCsc=x;}else if(bestCcc<0||d.CandidateCosts[x].TotalS<d.CandidateCosts[bestCcc].TotalS)bestCcc=x;}
 d.bCccEligible=bestCcc>=0&&(bestCsc<0||(headingError>=Config.CccMinimumHeadingErrorRad&&range<=Config.CccMaximumRangeRadiusFactor*rmin&&d.CandidateCosts[bestCcc].TotalS<d.CandidateCosts[bestCsc].TotalS-std::max(Config.CccAbsoluteAdvantageS,Config.CccRelativeAdvantage*d.CandidateCosts[bestCsc].TotalS)));
 int chosen=d.bCccEligible?bestCcc:bestCsc;if(chosen<0&&bestCcc>=0)chosen=bestCcc;if(chosen<0){bCandidateFeasible=false;return false;}
 const int active=(int)ActiveDubinsType;if(active>=0&&active<6&&valid[active]&&chosen!=active){d.RegeneratedActiveTypeCostS=d.CandidateCosts[active].TotalS;const double advantage=std::max(Config.TypeSwitchAbsoluteAdvantageS,Config.TypeSwitchRelativeAdvantage*d.RegeneratedActiveTypeCostS);if(!(d.CandidateCosts[chosen].TotalS<d.RegeneratedActiveTypeCostS-advantage))chosen=active;else d.bTypeSwitched=true;}
 ActivePath=paths[chosen];ActiveDubinsType=(DubinsType)chosen;ProgressS=0;LastTerminalPose=terminal;PathBuildTime=i.SimulationTimeS;PathGeneration++;HeldPathAge=0;bUsingHeldPath=false;d.SelectedType=ActiveDubinsType;return true;
}

void FormationPlannerV2::UpdateGuards(const FormationPlannerV2Input&i,double dt,const PathSample*sample,double rmin,FormationPlannerV2Diagnostics&d){
 // (A) Geometry — needs the slot and the follower only, NEVER an active path.
 const Vec2 t=FromCourse(i.Slot.Pose.CourseRad),r=RightNormal(i.Slot.Pose.CourseRad),delta=i.FollowerPositionNE-i.Slot.Pose.Position;d.AlongErrorM=delta.Dot(t);d.CrossErrorM=delta.Dot(r);d.RangeM=delta.Norm();d.RelativeAlongSpeedMps=(i.FollowerGroundVelocityNE-i.Slot.GroundVelocityNE).Dot(t);d.SlotHeadingErrorRad=std::abs(WrapPi(i.FollowerCourseRad-i.Slot.Pose.CourseRad));
 // (B) Path heading — the ONLY guard quantity that needs a path. Undefined without one.
 const bool bHasPath=sample!=nullptr&&sample->bValid;
 d.PathHeadingErrorRad=bHasPath?std::abs(WrapPi(i.FollowerCourseRad-std::atan2(sample->UnitTangent.E,sample->UnitTangent.N))):std::numeric_limits<double>::infinity();
 std::array<bool,(std::size_t)GuardTransition::Count> g{};const double a=d.AlongErrorM,c=std::abs(d.CrossErrorM),range=d.RangeM,v=d.RelativeAlongSpeedMps,ph=d.PathHeadingErrorRad,sh=d.SlotHeadingErrorRad;
 const bool strictHold=std::abs(a)<=Config.DirectHoldAlongM&&c<=Config.DirectHoldCrossM&&range<=Config.DirectHoldRangeM&&
                       sh<=Config.DirectHoldHeadingRad&&std::abs(v)<=Config.DirectHoldRelSpeedMps;
 // path-dependent guards (unchanged windows; simply not evaluated when there is no path)
 g[(int)GuardTransition::RejoinToCapture]=bHasPath&&Mode==PlannerMode::Rejoin&&a>=-6000&&a<=-600&&c<=400&&range>=600&&range<=8000&&ph<=45*Pi/180&&v>=-10&&v<=80;
 g[(int)GuardTransition::CaptureToTaper]=bHasPath&&Mode==PlannerMode::CaptureEntry&&a>=-1000&&a<=-250&&c<=180&&range>=250&&range<=1200&&ph<=20*Pi/180&&v>=0&&v<=35;
 g[(int)GuardTransition::CaptureToRejoin]=bHasPath&&Mode==PlannerMode::CaptureEntry&&(a<-1400||a>250||c>=300||range>1800||ph>=35*Pi/180||v<-15||v>50);
 g[(int)GuardTransition::TaperToHold]=bHasPath&&Mode==PlannerMode::ClosureTaper&&a>=-120&&a<=120&&c<=80&&range<=160&&sh<=15*Pi/180&&v>=-12&&v<=12;
 const bool outer=a<-1400||a>250||c>300||range>1800||ph>35*Pi/180||v<-15||v>50;
 const bool nearField=range<=std::max(Config.NearFieldRangeM,Config.NearFieldRangeRadiusFactor*rmin);
 g[(int)GuardTransition::TaperToRejoin]=bHasPath&&Mode==PlannerMode::ClosureTaper&&outer&&!nearField;
 g[(int)GuardTransition::TaperToNearField]=bHasPath&&Mode==PlannerMode::ClosureTaper&&!strictHold&&
  ((nearField&&outer)||(!outer&&(a<-220||a>180||c>140||range>260||ph>25*Pi/180||v<-12||v>20)));
 g[(int)GuardTransition::HoldToRejoin]=Mode==PlannerMode::SlotHold&&(a<-250||a>250||c>=160||range>=300||sh>=30*Pi/180||v<-22||v>22);

 // ---- geometry-only guards (evaluable with no path at all) ----
 // Strict direct hold: every bound is tighter than TaperToHold, so this can never be the loose
 // door into SlotHold. The observed near-field state (along +107..+213, cross <=121) fails it.
 d.bStrictHoldGeometry=strictHold;
 // "Near field" = the slot sits inside (or barely outside) the turn circle the CURRENT speed forces.
 const bool infeasible=bHasLastEval&&!bCandidateFeasible;   // last evaluation rejected every family
 g[(int)GuardTransition::RejoinToHold]=Mode==PlannerMode::Rejoin&&strictHold;
 g[(int)GuardTransition::NearFieldToHold]=Mode==PlannerMode::NearFieldSlotTrack&&strictHold;
 // A production-filter rejection is itself the explicit boundary: never emit CandidateSelectionFailed.
 // The far-field planner gets one real evaluation; if every exact-pose family is rejected, SlotTrack
 // takes over without retrying Dubins inside that mode.
 g[(int)GuardTransition::RejoinToNearField]=(Mode==PlannerMode::Rejoin||Mode==PlannerMode::CaptureEntry)&&!strictHold&&infeasible;
 g[(int)GuardTransition::HoldToNearField]=Mode==PlannerMode::SlotHold&&nearField&&g[(int)GuardTransition::HoldToRejoin];
 const bool taperGeometry=Mode==PlannerMode::NearFieldSlotTrack&&StateAge>=Config.NearFieldMinDwellS&&
  a>=-300&&a<=180&&c<=140&&range<=360&&sh<=25*Pi/180&&v>=-15&&v<=20;
 g[(int)GuardTransition::NearFieldToTaper]=taperGeometry&&!strictHold;
 g[(int)GuardTransition::NearFieldToRejoin]=Mode==PlannerMode::NearFieldSlotTrack&&!nearField&&bCandidateFeasible&&StateAge>=Config.NearFieldMinDwellS;

 for(std::size_t x=0;x<g.size();x++){GuardDwell[x]=g[x]?GuardDwell[x]+dt:0;}
 d.GuardDwellS=GuardDwell;
 auto Fire=[&](GuardTransition x,double dwell){return g[(int)x]&&GuardDwell[(int)x]>=dwell;};
 // The seven original transitions keep their relative priority; the new ones are interleaved so that
 // a strict hold always beats a fallback, and a Rejoin handback is the last near-field resort.
 if(Fire(GuardTransition::TaperToRejoin,.5))Transition(PlannerMode::Rejoin,i.SimulationTimeS);
 else if(Fire(GuardTransition::CaptureToRejoin,.5))Transition(PlannerMode::Rejoin,i.SimulationTimeS);
 else if(Fire(GuardTransition::HoldToNearField,Config.NearFieldEntryDwellS))Transition(PlannerMode::NearFieldSlotTrack,i.SimulationTimeS);
 else if(Fire(GuardTransition::HoldToRejoin,1.))Transition(PlannerMode::Rejoin,i.SimulationTimeS);
 else if(Fire(GuardTransition::TaperToNearField,.75))Transition(PlannerMode::NearFieldSlotTrack,i.SimulationTimeS);
 else if(Fire(GuardTransition::TaperToHold,1.))Transition(PlannerMode::SlotHold,i.SimulationTimeS);
 else if(Fire(GuardTransition::RejoinToHold,Config.DirectHoldDwellS))Transition(PlannerMode::SlotHold,i.SimulationTimeS);
 else if(Fire(GuardTransition::NearFieldToHold,Config.DirectHoldDwellS))Transition(PlannerMode::SlotHold,i.SimulationTimeS);
 else if(Fire(GuardTransition::NearFieldToTaper,Config.NearFieldExitDwellS))Transition(PlannerMode::ClosureTaper,i.SimulationTimeS);
 else if(Fire(GuardTransition::NearFieldToRejoin,Config.NearFieldExitDwellS))Transition(PlannerMode::Rejoin,i.SimulationTimeS);
 else if(Fire(GuardTransition::CaptureToTaper,.75))Transition(PlannerMode::ClosureTaper,i.SimulationTimeS);
 else if(Fire(GuardTransition::RejoinToCapture,.5))Transition(PlannerMode::CaptureEntry,i.SimulationTimeS);
 // Entry dwell exists to stop chattering while a usable path is still being flown. With no path at
 // all there is nothing to chatter against and nothing else to emit, so entry is immediate.
 else if(Fire(GuardTransition::RejoinToNearField,bHasPath?Config.NearFieldEntryDwellS:0.0))Transition(PlannerMode::NearFieldSlotTrack,i.SimulationTimeS);
}

FormationPlannerV2Output FormationPlannerV2::Update(const FormationPlannerV2Input&i,FormationPlannerV2Diagnostics&d){
 d={};if(!bInitialized)Reset(i.ResetGeneration);if(i.ResetGeneration!=LastResetGeneration)Reset(i.ResetGeneration);
 // Pause freezes dwell, the candidate retry timer and the speed state. The pause interval must never
 // be consumed as dt, nor be counted toward the retry interval, on the resume frame.
 if(i.bPaused){bWasPaused=true;return FreshInvalid(PlannerFailure::Paused);}double dt=i.DtS;const bool resumed=bWasPaused;
 if(resumed){dt=0;bWasPaused=false;const double frozen=i.SimulationTimeS-LastInputTimeS;
  if(Finite(frozen)&&frozen>0){LastCandidateEvalTimeS+=frozen;LastReplanTime+=frozen;LastModeTransitionTime+=frozen;}}
 if(!ValidateCritical(i)){Reset(i.ResetGeneration);return FreshInvalid(PlannerFailure::CriticalInputInvalid);}if(!Finite(dt)||dt<0||(!resumed&&dt<=0)||dt>Config.Projection.MaximumDtS){Reset(i.ResetGeneration);return FreshInvalid(PlannerFailure::AbnormalDt);}
 LastInputTimeS=i.SimulationTimeS;
 // Wind=0 validated bound: only the follower's current ground speed determines its turn capability.
 const double followerSpeed=i.FollowerGroundVelocityNE.Norm();const double speed=std::max(Config.PlanningSpeedFloorMps,followerSpeed);
 const double tanBank=std::tan(Config.PlannerBankLimitRad);const double rmin=std::max(Config.MinimumTurnRadiusM,Config.TurnRadiusSafetyFactor*speed*speed/(GravityMps2*tanBank));
 d.RminM=rmin;d.TurnBoundSpeedMps=speed;d.FollowerGroundSpeedMps=followerSpeed;d.SlotGroundSpeedMps=i.Slot.GroundSpeedMps;
 d.TurnBoundSource=TurnBoundSourceV2::FollowerGroundSpeedZeroWind;d.bWindAwareTurnBound=false;d.bZeroWindAssumption=true;
 d.bTurnBoundValid=Finite(rmin)&&rmin>0;
 // Turn-bound feasibility, published EVERY frame from here on (including the frames this planner
 // then rejects). Observers classify their populations off this instead of re-deriving Rmin.
 {const SlotCurvatureFeasibilityV2 f=ClassifySlotCurvature(i.Slot.CurvaturePerM,i.Slot.bCurvatureValid,rmin);
  d.SlotCurvaturePerM=f.SlotCurvaturePerM;d.bSlotCurvatureValid=f.bCurvatureValid;
  d.MaxFeasibleSlotCurvaturePerM=f.MaxFeasibleCurvaturePerM;d.SlotCurvatureClass=(int)f.Class;
  d.bSlotCurvatureFeasible=!f.bRejectedByTurnBound;}
 if(!Finite(rmin)){Reset(i.ResetGeneration);return FreshInvalid(PlannerFailure::TurnBoundInvalid);}
 const double slotRange=(i.FollowerPositionNE-i.Slot.Pose.Position).Norm();
 if(Mode==PlannerMode::Rejoin&&slotRange<=std::max(Config.NearFieldRangeM,Config.NearFieldRangeRadiusFactor*rmin))
  Transition(PlannerMode::NearFieldSlotTrack,i.SimulationTimeS);
 auto pred=MovingSlotPredictor::PredictAtTime(i.Slot,Config.PredictionTimeS,Config.Prediction);bool predictionOk=pred.bValid;std::uint32_t reasons=0;
 bool windowValid=false;double fw=DubinsPath::ComputeForwardSearchWindow(i.FollowerGroundVelocityNE.Norm(),std::max(dt,Config.Projection.MinimumDtS),Config.Projection,windowValid);ProjectionResult projection;
 if(ActivePath.IsValid()&&windowValid){projection=ActivePath.Project(i.FollowerPositionNE,ProgressS,Config.ProjectionBacktrackM,fw,true,Config.ProjectionFailureDistanceM);if(projection.bValid){ProgressS=projection.S;d.ProjectionDistanceM=projection.DistanceM;}else reasons|=ReplanReason::ProjectionFailed;}
 if(!predictionOk){reasons|=ReplanReason::PredictionRefreshFailed;if(ActivePath.IsValid()&&projection.bValid&&ActivePath.GetLength()-ProgressS>Config.HeldPathMinimumRemainingM&&HeldPathAge<=Config.MaxHeldPathAgeS){bUsingHeldPath=true;HeldPathAge+=dt;}else{ActivePath={};return FreshInvalid(HeldPathAge>Config.MaxHeldPathAgeS?PlannerFailure::HeldPathExpired:PlannerFailure::PredictionRefreshFailed);}}

 const bool slotLocalMode=Mode==PlannerMode::NearFieldSlotTrack||Mode==PlannerMode::ClosureTaper||Mode==PlannerMode::SlotHold;
 SlotLocalPathProjectionV2 slotPath=slotLocalMode?BuildSlotLocalPath(i,rmin):SlotLocalPathProjectionV2{};

 // Exact-terminal Dubins belongs only to far-field Rejoin/CaptureEntry.
 bool candidateFailedThisFrame=false;
 if(predictionOk&&!slotLocalMode){const double posDelta=ActivePath.IsValid()?(pred.TerminalPose.Position-LastTerminalPose.Position).Norm():INFINITY;const double hdgDelta=ActivePath.IsValid()?std::abs(WrapPi(pred.TerminalPose.CourseRad-LastTerminalPose.CourseRad)):INFINITY;if(!ActivePath.IsValid())reasons|=ReplanReason::NoActivePath;if(posDelta>Config.TerminalPositionReplanM)reasons|=ReplanReason::TerminalPositionChanged;if(hdgDelta>Config.TerminalHeadingReplanRad)reasons|=ReplanReason::TerminalHeadingChanged;if(ActivePath.IsValid()&&i.SimulationTimeS-PathBuildTime>Config.PathExpirationS)reasons|=ReplanReason::PathExpired;if(ActivePath.IsValid()&&ActivePath.GetLength()-ProgressS<Config.RemainingPathReplanM)reasons|=ReplanReason::RemainingPathExhausted;if(bModeChangedPending)reasons|=ReplanReason::ModeChanged;
  const bool hardInvalid=ActivePath.IsValid()&&(!Config.bDecelerationEnvelopeValid||ActivePath.GetLength()>Config.MaxPathLengthM||ActivePath.GetRadius()+1e-9<rmin||(ActivePath.GetLength()-ProgressS)/std::max(Config.ReferenceGroundSpeedFloorMps,i.FollowerGroundVelocityNE.Norm())>Config.MaxCaptureTimeS);if(hardInvalid)reasons|=ReplanReason::HardCandidateInvalid;
  const bool mandatory=reasons&(ReplanReason::NoActivePath|ReplanReason::ProjectionFailed|ReplanReason::TerminalPositionChanged|ReplanReason::TerminalHeadingChanged|ReplanReason::PathExpired|ReplanReason::RemainingPathExhausted|ReplanReason::ModeChanged|ReplanReason::HardCandidateInvalid);
  const bool wanted=mandatory&&(i.SimulationTimeS-LastReplanTime>=Config.MinimumReplanIntervalS||reasons&(ReplanReason::NoActivePath|ReplanReason::ProjectionFailed|ReplanReason::ModeChanged|ReplanReason::HardCandidateInvalid));
  // Storm guard: once every family has been rejected, an identical retry (same mode, same speed,
  // same slot terminal, within the retry interval) is suppressed instead of re-run at 60 Hz.
  if(wanted&&ShouldEvaluateCandidates(i,pred.TerminalPose,i.FollowerGroundVelocityNE.Norm())){const Vec2 st=FromCourse(i.Slot.Pose.CourseRad);const double along=(i.FollowerPositionNE-i.Slot.Pose.Position).Dot(st);
   if(!Replan(i,pred,rmin,along,d))candidateFailedThisFrame=true;
   else{projection=ActivePath.Project(i.FollowerPositionNE,0,0,std::max(100.,fw),true,Config.ProjectionFailureDistanceM);if(projection.bValid)ProgressS=projection.S;bModeChangedPending=false;}}
  if(!candidateFailedThisFrame&&ActivePath.IsValid()){bUsingHeldPath=false;HeldPathAge=0;}
 }
 (void)candidateFailedThisFrame;

 // Guards run on the post-replan sample (unchanged ordering), but are now reachable even when the
 // candidate set was rejected: the geometry guards below need no path at all.
 const PathSample guardSample=slotLocalMode&&slotPath.bValid?slotPath.Path:(ActivePath.IsValid()?ActivePath.Sample(ProgressS):PathSample{});
 if(dt>0)StateAge+=dt;
 UpdateGuards(i,dt,guardSample.bValid?&guardSample:nullptr,rmin,d);
 if(Mode==PlannerMode::CaptureEntry&&StateAge>=Config.CaptureEntryTimeoutS){Transition(PlannerMode::Rejoin,i.SimulationTimeS);return FreshInvalid(PlannerFailure::CaptureTimeout);}
 if(Mode==PlannerMode::ClosureTaper&&StateAge>=Config.ClosureTaperTimeoutS){Transition(PlannerMode::Rejoin,i.SimulationTimeS);return FreshInvalid(PlannerFailure::TaperTimeout);}
 if(Mode==PlannerMode::Rejoin&&Config.RejoinTimeoutS>Config.MaxCaptureTimeS&&StateAge>=Config.RejoinTimeoutS){return FreshInvalid(PlannerFailure::RejoinTimeout);}
 // A completed candidate evaluation with no retained far-field path falls through to the explicit
 // SlotTrack primitive, never to the generic CandidateSelectionFailed output.
 if((Mode==PlannerMode::Rejoin||Mode==PlannerMode::CaptureEntry)&&!ActivePath.IsValid()&&bHasLastEval&&!bCandidateFeasible)
  Transition(PlannerMode::NearFieldSlotTrack,i.SimulationTimeS);

 auto FillCommon=[&](FormationPlannerV2Output&o){o.Mode=Mode;o.PathGeneration=PathGeneration;d.ReplanReasons=reasons;d.ReplanCount=ReplanCount;
  d.HeldPathAgeS=HeldPathAge;d.bUsingHeldPath=bUsingHeldPath;d.StateAgeS=StateAge;
  d.CandidateEvaluationCount=CandidateEvaluationCount;d.ConsecutiveCandidateFailures=ConsecutiveCandidateFailures;
  d.LastCandidateRejectMask=LastRejectMask;d.TimeSinceLastCandidateEvaluationS=bHasLastEval?i.SimulationTimeS-LastCandidateEvalTimeS:std::numeric_limits<double>::infinity();};

 // NearFieldSlotTrack, ClosureTaper and SlotHold all follow the same Slot-local line/circle.
 const bool outputSlotLocal=Mode==PlannerMode::NearFieldSlotTrack||Mode==PlannerMode::ClosureTaper||Mode==PlannerMode::SlotHold;
 if(outputSlotLocal){
  slotPath=BuildSlotLocalPath(i,rmin);d.bSlotLocalPath=slotPath.bValid;d.SlotLocalPrimitiveFailure=(int)slotPath.Failure;
  d.SlotLocalSignedCrossErrorM=slotPath.SignedCrossTrackErrorM;d.ProjectionDistanceM=slotPath.ProjectionDistanceM;
  if(!slotPath.bValid){PlannerFailure failure=PlannerFailure::SlotPathInvalid;
   if(slotPath.Failure==SlotLocalPathFailureV2::CurvatureUnavailable)failure=PlannerFailure::SlotCurvatureUnavailable;
   if(slotPath.Failure==SlotLocalPathFailureV2::CurvatureInfeasible)failure=PlannerFailure::SlotCurvatureInfeasible;
   FormationPlannerV2Output o=FreshInvalid(failure);FillCommon(o);return o;}
  NearFieldSpeedInputV2 si{};si.SlotGroundVelocityNE=i.Slot.GroundVelocityNE;si.SlotUnitTangentNE=slotPath.Path.UnitTangent;
  si.FollowerGroundVelocityNE=i.FollowerGroundVelocityNE;si.WindVelocityNE=i.WindVelocityNE;
  si.AlongErrorM=Mode==PlannerMode::SlotHold?0.0:d.AlongErrorM;si.EasToTasRatio=i.EasToTasRatio;si.DtS=dt;
  si.ClosureTimeConstantS=Config.NearFieldClosureTimeConstantS;si.HoldTaperDistanceM=Config.NearFieldHoldTaperDistanceM;
  si.MaxClosureMps=Config.MaxClosureMps;si.AccelerationMps2=Config.NearFieldAccelerationMps2;si.DecelerationMps2=Config.ConfiguredDecelerationMps2;
  si.MinTargetEasMps=Config.MinTargetEasMps;si.MaxTargetEasMps=Config.MaxTargetEasMps;
  si.bWindValid=i.bWindValid;si.bRatioValid=i.bRatioValid;si.bEnvelopeValid=Config.bDecelerationEnvelopeValid;
  const auto speedOut=NearFieldSpeedPlannerV2::Compute(si);FormationPlannerV2Output o{};o.Path=slotPath.Path;o.ProgressS=slotPath.LocalCoordinate;
  o.bPathValid=true;o.TargetEasMps=speedOut.TargetEasMps;o.bTargetEasValid=speedOut.bValid;o.bValid=speedOut.bValid;
  o.Failure=speedOut.Failure==NearFieldSpeedFailureV2::InvalidWind?PlannerFailure::InvalidWind:
   (speedOut.Failure==NearFieldSpeedFailureV2::InvalidRatio?PlannerFailure::InvalidRatio:
   (speedOut.Failure==NearFieldSpeedFailureV2::InvalidEnvelope?PlannerFailure::InvalidDeceleration:PlannerFailure::None));
  if(!o.bValid){o.Path={};o.bPathValid=false;o.TargetEasMps=0;}
  NearFieldTargetGroundSpeedMps=speedOut.TargetGroundSpeedMps;NearFieldTargetEasMps=speedOut.TargetEasMps;
  d.NearFieldTargetGroundSpeedMps=NearFieldTargetGroundSpeedMps;d.NearFieldTargetEasMps=NearFieldTargetEasMps;
  d.SelectedType=DubinsType::Invalid;d.ActivePathRemainingCostS=0;FillCommon(o);return o;
 }

 if(!ActivePath.IsValid()){FormationPlannerV2Output o=FreshInvalid(PlannerFailure::CandidateSelectionFailed);FillCommon(o);return o;}
 const PathSample sample=ActivePath.Sample(ProgressS);
 if(!sample.bValid){FormationPlannerV2Output o=FreshInvalid(PlannerFailure::ProjectionFailed);FillCommon(o);return o;}
 CaptureSpeedInput si;si.PredictedSlotGroundVelocityNE=predictionOk?pred.TerminalGroundVelocityNE:i.Slot.GroundVelocityNE;si.PathUnitTangentNE=sample.UnitTangent;si.WindVelocityNE=i.WindVelocityNE;si.EasToTasRatio=i.EasToTasRatio;si.DesiredCaptureClosureMps=Config.DesiredClosureMps;si.RemainingAlongDistanceM=std::max(0.,-d.AlongErrorM);si.SafetyDistanceM=Config.SafetyDistanceM;si.ConfiguredDecelerationMps2=Config.ConfiguredDecelerationMps2;si.MaxClosureSpeedMps=Config.MaxClosureMps;si.MinTargetEasMps=Config.MinTargetEasMps;si.MaxTargetEasMps=Config.MaxTargetEasMps;si.bWindValid=i.bWindValid;si.bRatioValid=i.bRatioValid;si.bDecelerationEnvelopeValid=Config.bDecelerationEnvelopeValid;auto speedOut=CaptureSpeedPlanner::Compute(si);
 FormationPlannerV2Output o;o.Path=sample;o.ProgressS=ProgressS;o.bPathValid=true;o.TargetEasMps=speedOut.TargetEasMps;o.bTargetEasValid=speedOut.bTargetEasValid;o.bValid=o.bPathValid&&o.bTargetEasValid;o.Failure=speedOut.Failure==SpeedPlanFailure::InvalidWind?PlannerFailure::InvalidWind:(speedOut.Failure==SpeedPlanFailure::InvalidEasToTasRatio?PlannerFailure::InvalidRatio:(speedOut.Failure==SpeedPlanFailure::InvalidDecelerationEnvelope?PlannerFailure::InvalidDeceleration:PlannerFailure::None));
 if(!o.bValid){o.Path={};o.bPathValid=false;o.TargetEasMps=0;}     // never emit a stale/partial output
 d.SelectedType=ActiveDubinsType;d.ActivePathRemainingCostS=(ActivePath.GetLength()-ProgressS)/std::max(Config.ReferenceGroundSpeedFloorMps,i.FollowerGroundVelocityNE.Norm());
 FillCommon(o);return o;
}
}
