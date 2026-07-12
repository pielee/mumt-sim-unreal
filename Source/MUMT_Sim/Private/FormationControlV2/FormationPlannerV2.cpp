#include "FormationControlV2/FormationPlannerV2.h"
#include <algorithm>

namespace FormationControlV2 {
void FormationPlannerV2::Reset(std::uint32_t gen){ActivePath={};ActiveDubinsType=DubinsType::Invalid;ProgressS=0;LastTerminalPose={};PathBuildTime=0;LastReplanTime=-1e9;LastModeTransitionTime=0;LastResetGeneration=gen;PathGeneration=0;Mode=PlannerMode::Rejoin;GuardDwell={};StateAge=0;HeldPathAge=0;bUsingHeldPath=false;bInitialized=true;bWasPaused=false;bModeChangedPending=false;ReplanCount=0;}
FormationPlannerV2Output FormationPlannerV2::FreshInvalid(PlannerFailure f)const{FormationPlannerV2Output o;o.Mode=Mode;o.Failure=f;o.PathGeneration=PathGeneration;return o;}
bool FormationPlannerV2::ValidateCritical(const FormationPlannerV2Input&i)const{
 return i.bFollowerValid&&i.bCourseValid&&i.FollowerPositionNE.IsFinite()&&i.FollowerGroundVelocityNE.IsFinite()&&Finite(i.FollowerCourseRad)&&
 i.Slot.bValid&&i.Slot.Pose.IsFinite()&&i.Slot.GroundVelocityNE.IsFinite()&&Finite(i.Slot.GroundSpeedMps)&&i.Slot.GroundSpeedMps>=0&&
 Finite(i.SimulationTimeS)&&Finite(Config.PlannerBankLimitRad)&&Config.PlannerBankLimitRad>1e-6&&Config.PlannerBankLimitRad<Pi/2&&
 Finite(Config.MinimumTurnRadiusM)&&Config.MinimumTurnRadiusM>0&&Finite(Config.TurnRadiusSafetyFactor)&&Config.TurnRadiusSafetyFactor>=1&&
 Finite(Config.PlanningSpeedFloorMps)&&Config.PlanningSpeedFloorMps>0;
}
void FormationPlannerV2::Transition(PlannerMode next,double now){if(next!=Mode){Mode=next;StateAge=0;LastModeTransitionTime=now;GuardDwell={};bModeChangedPending=true;}}

bool FormationPlannerV2::Replan(const FormationPlannerV2Input&i,const PredictionResult&pred,double rmin,double along,FormationPlannerV2Diagnostics&d){
 LastReplanTime=i.SimulationTimeS;ReplanCount++;const Pose2 start{i.FollowerPositionNE,i.FollowerCourseRad};const Pose2 terminal=pred.TerminalPose;
 const double refSpeed=std::max(Config.ReferenceGroundSpeedFloorMps,i.FollowerGroundVelocityNE.Norm());const double headingError=std::abs(WrapPi(terminal.CourseRad-start.CourseRad));const double range=(terminal.Position-start.Position).Norm();
 auto raw=DubinsPath::BuildCandidates(start,terminal,rmin);std::array<DubinsPath,6> paths{};std::array<bool,6> valid{};
 for(auto &c:d.CandidateCosts)c={};
 for(const auto&c:raw){const int idx=(int)c.Type;DubinsPath p;if(!p.BuildType(start,terminal,rmin,c.Type))continue;const double transit=p.GetLength()/refSpeed;const double closureDistance=std::max(0.0,-along-Config.SafetyDistanceM);const double totalClosure=closureDistance/std::max(Config.DesiredClosureMps,1.0);const double closure=std::max(0.0,totalClosure-transit);
  const double excursion=std::max(0.0,(p.GetSegments()[0].LengthM+p.GetSegments()[2].LengthM)/rmin-headingError);const double headingPenalty=std::max(0.0,excursion-Pi)/(Pi/4)*2.0;const double terminalPenalty=Config.bDecelerationEnvelopeValid?0.0:std::numeric_limits<double>::infinity();
  auto &cost=d.CandidateCosts[idx];cost.Type=c.Type;cost.TransitTimeS=transit;cost.ClosureTimeS=closure;cost.CccPenaltyS=IsCsc(c.Type)?0:Config.CccPenaltyS;cost.HeadingExcursionPenaltyS=headingPenalty;cost.TerminalClosurePenaltyS=terminalPenalty;cost.TotalS=transit+Config.CaptureTimeWeight*closure+cost.CccPenaltyS+headingPenalty+terminalPenalty;cost.bCsc=IsCsc(c.Type);cost.bValid=Finite(cost.TotalS)&&p.GetLength()<=Config.MaxPathLengthM&&transit+closure<=Config.MaxCaptureTimeS;paths[idx]=p;valid[idx]=cost.bValid;}
 int bestCsc=-1,bestCcc=-1;for(int x=0;x<6;x++)if(valid[x]){if(IsCsc((DubinsType)x)){if(bestCsc<0||d.CandidateCosts[x].TotalS<d.CandidateCosts[bestCsc].TotalS)bestCsc=x;}else if(bestCcc<0||d.CandidateCosts[x].TotalS<d.CandidateCosts[bestCcc].TotalS)bestCcc=x;}
 d.bCccEligible=bestCcc>=0&&(bestCsc<0||(headingError>=Config.CccMinimumHeadingErrorRad&&range<=Config.CccMaximumRangeRadiusFactor*rmin&&d.CandidateCosts[bestCcc].TotalS<d.CandidateCosts[bestCsc].TotalS-std::max(Config.CccAbsoluteAdvantageS,Config.CccRelativeAdvantage*d.CandidateCosts[bestCsc].TotalS)));
 int chosen=d.bCccEligible?bestCcc:bestCsc;if(chosen<0&&bestCcc>=0)chosen=bestCcc;if(chosen<0)return false;
 const int active=(int)ActiveDubinsType;if(active>=0&&active<6&&valid[active]&&chosen!=active){d.RegeneratedActiveTypeCostS=d.CandidateCosts[active].TotalS;const double advantage=std::max(Config.TypeSwitchAbsoluteAdvantageS,Config.TypeSwitchRelativeAdvantage*d.RegeneratedActiveTypeCostS);if(!(d.CandidateCosts[chosen].TotalS<d.RegeneratedActiveTypeCostS-advantage))chosen=active;else d.bTypeSwitched=true;}
 ActivePath=paths[chosen];ActiveDubinsType=(DubinsType)chosen;ProgressS=0;LastTerminalPose=terminal;PathBuildTime=i.SimulationTimeS;PathGeneration++;HeldPathAge=0;bUsingHeldPath=false;d.SelectedType=ActiveDubinsType;return true;
}

void FormationPlannerV2::UpdateGuards(const FormationPlannerV2Input&i,double dt,const PathSample&sample,FormationPlannerV2Diagnostics&d){
 const Vec2 t=FromCourse(i.Slot.Pose.CourseRad),r=RightNormal(i.Slot.Pose.CourseRad),delta=i.FollowerPositionNE-i.Slot.Pose.Position;d.AlongErrorM=delta.Dot(t);d.CrossErrorM=delta.Dot(r);d.RangeM=delta.Norm();d.RelativeAlongSpeedMps=(i.FollowerGroundVelocityNE-i.Slot.GroundVelocityNE).Dot(t);d.SlotHeadingErrorRad=std::abs(WrapPi(i.FollowerCourseRad-i.Slot.Pose.CourseRad));const double pathCourse=sample.bValid?std::atan2(sample.UnitTangent.E,sample.UnitTangent.N):i.FollowerCourseRad;d.PathHeadingErrorRad=std::abs(WrapPi(i.FollowerCourseRad-pathCourse));
 std::array<bool,(std::size_t)GuardTransition::Count> g{};const double a=d.AlongErrorM,c=std::abs(d.CrossErrorM),range=d.RangeM,v=d.RelativeAlongSpeedMps,ph=d.PathHeadingErrorRad,sh=d.SlotHeadingErrorRad;
 g[(int)GuardTransition::RejoinToCapture]=Mode==PlannerMode::Rejoin&&a>=-6000&&a<=-600&&c<=400&&range>=600&&range<=8000&&ph<=45*Pi/180&&v>=-10&&v<=80;
 g[(int)GuardTransition::CaptureToTaper]=Mode==PlannerMode::CaptureEntry&&a>=-1000&&a<=-250&&c<=180&&range>=250&&range<=1200&&ph<=20*Pi/180&&v>=0&&v<=35;
 g[(int)GuardTransition::CaptureToRejoin]=Mode==PlannerMode::CaptureEntry&&(a<-1400||a>250||c>=300||range>1800||ph>=35*Pi/180||v<-15||v>50);
 g[(int)GuardTransition::TaperToHold]=Mode==PlannerMode::ClosureTaper&&a>=-120&&a<=120&&c<=80&&range<=160&&sh<=15*Pi/180&&v>=-12&&v<=12;
 const bool outer=a<-1400||a>250||c>300||range>1800||ph>35*Pi/180||v<-15||v>50;
 g[(int)GuardTransition::TaperToRejoin]=Mode==PlannerMode::ClosureTaper&&outer;
 g[(int)GuardTransition::TaperToCapture]=Mode==PlannerMode::ClosureTaper&&!outer&&(a<-220||a>180||c>140||range>260||ph>25*Pi/180||v<-12||v>20);
 g[(int)GuardTransition::HoldToRejoin]=Mode==PlannerMode::SlotHold&&(a<-250||a>250||c>=160||range>=300||sh>=30*Pi/180||v<-22||v>22);
 for(std::size_t x=0;x<g.size();x++){GuardDwell[x]=g[x]?GuardDwell[x]+dt:0;}
 d.GuardDwellS=GuardDwell;
 if(g[(int)GuardTransition::TaperToRejoin]&&GuardDwell[(int)GuardTransition::TaperToRejoin]>=.5)Transition(PlannerMode::Rejoin,i.SimulationTimeS);
 else if(g[(int)GuardTransition::CaptureToRejoin]&&GuardDwell[(int)GuardTransition::CaptureToRejoin]>=.5)Transition(PlannerMode::Rejoin,i.SimulationTimeS);
 else if(g[(int)GuardTransition::HoldToRejoin]&&GuardDwell[(int)GuardTransition::HoldToRejoin]>=1.)Transition(PlannerMode::Rejoin,i.SimulationTimeS);
 else if(g[(int)GuardTransition::TaperToCapture]&&GuardDwell[(int)GuardTransition::TaperToCapture]>=.75)Transition(PlannerMode::CaptureEntry,i.SimulationTimeS);
 else if(g[(int)GuardTransition::TaperToHold]&&GuardDwell[(int)GuardTransition::TaperToHold]>=1.)Transition(PlannerMode::SlotHold,i.SimulationTimeS);
 else if(g[(int)GuardTransition::CaptureToTaper]&&GuardDwell[(int)GuardTransition::CaptureToTaper]>=.75)Transition(PlannerMode::ClosureTaper,i.SimulationTimeS);
 else if(g[(int)GuardTransition::RejoinToCapture]&&GuardDwell[(int)GuardTransition::RejoinToCapture]>=.5)Transition(PlannerMode::CaptureEntry,i.SimulationTimeS);
}

FormationPlannerV2Output FormationPlannerV2::Update(const FormationPlannerV2Input&i,FormationPlannerV2Diagnostics&d){
 d={};if(!bInitialized)Reset(i.ResetGeneration);if(i.ResetGeneration!=LastResetGeneration)Reset(i.ResetGeneration);
 if(i.bPaused){bWasPaused=true;return FreshInvalid(PlannerFailure::Paused);}double dt=i.DtS;const bool resumed=bWasPaused;if(resumed){dt=0;bWasPaused=false;}
 if(!ValidateCritical(i)){Reset(i.ResetGeneration);return FreshInvalid(PlannerFailure::CriticalInputInvalid);}if(!Finite(dt)||dt<0||(!resumed&&dt<=0)||dt>Config.Projection.MaximumDtS){Reset(i.ResetGeneration);return FreshInvalid(PlannerFailure::AbnormalDt);}
 const double speed=std::max({Config.PlanningSpeedFloorMps,i.FollowerGroundVelocityNE.Norm(),i.Slot.GroundSpeedMps});const double tanBank=std::tan(Config.PlannerBankLimitRad);const double rmin=std::max(Config.MinimumTurnRadiusM,Config.TurnRadiusSafetyFactor*speed*speed/(GravityMps2*tanBank));d.RminM=rmin;if(!Finite(rmin)){Reset(i.ResetGeneration);return FreshInvalid(PlannerFailure::TurnBoundInvalid);}
 auto pred=MovingSlotPredictor::PredictAtTime(i.Slot,Config.PredictionTimeS,Config.Prediction);bool predictionOk=pred.bValid;std::uint32_t reasons=0;
 bool windowValid=false;double fw=DubinsPath::ComputeForwardSearchWindow(i.FollowerGroundVelocityNE.Norm(),std::max(dt,Config.Projection.MinimumDtS),Config.Projection,windowValid);ProjectionResult projection;
 if(ActivePath.IsValid()&&windowValid){projection=ActivePath.Project(i.FollowerPositionNE,ProgressS,Config.ProjectionBacktrackM,fw,true,Config.ProjectionFailureDistanceM);if(projection.bValid){ProgressS=projection.S;d.ProjectionDistanceM=projection.DistanceM;}else reasons|=ReplanReason::ProjectionFailed;}
 if(!predictionOk){reasons|=ReplanReason::PredictionRefreshFailed;if(ActivePath.IsValid()&&projection.bValid&&ActivePath.GetLength()-ProgressS>Config.HeldPathMinimumRemainingM&&HeldPathAge<=Config.MaxHeldPathAgeS){bUsingHeldPath=true;HeldPathAge+=dt;}else{ActivePath={};return FreshInvalid(HeldPathAge>Config.MaxHeldPathAgeS?PlannerFailure::HeldPathExpired:PlannerFailure::PredictionRefreshFailed);}}
 if(predictionOk){const double posDelta=ActivePath.IsValid()?(pred.TerminalPose.Position-LastTerminalPose.Position).Norm():INFINITY;const double hdgDelta=ActivePath.IsValid()?std::abs(WrapPi(pred.TerminalPose.CourseRad-LastTerminalPose.CourseRad)):INFINITY;if(!ActivePath.IsValid())reasons|=ReplanReason::NoActivePath;if(posDelta>Config.TerminalPositionReplanM)reasons|=ReplanReason::TerminalPositionChanged;if(hdgDelta>Config.TerminalHeadingReplanRad)reasons|=ReplanReason::TerminalHeadingChanged;if(ActivePath.IsValid()&&i.SimulationTimeS-PathBuildTime>Config.PathExpirationS)reasons|=ReplanReason::PathExpired;if(ActivePath.IsValid()&&ActivePath.GetLength()-ProgressS<Config.RemainingPathReplanM&&Mode!=PlannerMode::SlotHold)reasons|=ReplanReason::RemainingPathExhausted;if(bModeChangedPending)reasons|=ReplanReason::ModeChanged;
  const bool hardInvalid=ActivePath.IsValid()&&(!Config.bDecelerationEnvelopeValid||ActivePath.GetLength()>Config.MaxPathLengthM||ActivePath.GetRadius()+1e-9<rmin||(ActivePath.GetLength()-ProgressS)/std::max(Config.ReferenceGroundSpeedFloorMps,i.FollowerGroundVelocityNE.Norm())>Config.MaxCaptureTimeS);if(hardInvalid)reasons|=ReplanReason::HardCandidateInvalid;
  const bool mandatory=reasons&(ReplanReason::NoActivePath|ReplanReason::ProjectionFailed|ReplanReason::TerminalPositionChanged|ReplanReason::TerminalHeadingChanged|ReplanReason::PathExpired|ReplanReason::RemainingPathExhausted|ReplanReason::ModeChanged|ReplanReason::HardCandidateInvalid);if(mandatory&&(i.SimulationTimeS-LastReplanTime>=Config.MinimumReplanIntervalS||reasons&(ReplanReason::NoActivePath|ReplanReason::ProjectionFailed|ReplanReason::ModeChanged|ReplanReason::HardCandidateInvalid))){const Vec2 st=FromCourse(i.Slot.Pose.CourseRad);const double along=(i.FollowerPositionNE-i.Slot.Pose.Position).Dot(st);if(!Replan(i,pred,rmin,along,d))return FreshInvalid(PlannerFailure::CandidateSelectionFailed);projection=ActivePath.Project(i.FollowerPositionNE,0,0,std::max(100.,fw),true,Config.ProjectionFailureDistanceM);if(projection.bValid)ProgressS=projection.S;bModeChangedPending=false;}
  bUsingHeldPath=false;HeldPathAge=0;
 }
 if(!ActivePath.IsValid())return FreshInvalid(PlannerFailure::CandidateSelectionFailed);
 const PathSample sample=ActivePath.Sample(ProgressS);if(!sample.bValid)return FreshInvalid(PlannerFailure::ProjectionFailed);
 if(dt>0){StateAge+=dt;UpdateGuards(i,dt,sample,d);}if(Mode==PlannerMode::CaptureEntry&&StateAge>=Config.CaptureEntryTimeoutS){Transition(PlannerMode::Rejoin,i.SimulationTimeS);return FreshInvalid(PlannerFailure::CaptureTimeout);}if(Mode==PlannerMode::ClosureTaper&&StateAge>=Config.ClosureTaperTimeoutS){Transition(PlannerMode::Rejoin,i.SimulationTimeS);return FreshInvalid(PlannerFailure::TaperTimeout);}if(Mode==PlannerMode::Rejoin&&Config.RejoinTimeoutS>Config.MaxCaptureTimeS&&StateAge>=Config.RejoinTimeoutS){return FreshInvalid(PlannerFailure::RejoinTimeout);}
 CaptureSpeedInput si;si.PredictedSlotGroundVelocityNE=predictionOk?pred.TerminalGroundVelocityNE:i.Slot.GroundVelocityNE;si.PathUnitTangentNE=sample.UnitTangent;si.WindVelocityNE=i.WindVelocityNE;si.EasToTasRatio=i.EasToTasRatio;si.DesiredCaptureClosureMps=Config.DesiredClosureMps;si.RemainingAlongDistanceM=std::max(0.,-d.AlongErrorM);si.SafetyDistanceM=Config.SafetyDistanceM;si.ConfiguredDecelerationMps2=Config.ConfiguredDecelerationMps2;si.MaxClosureSpeedMps=Config.MaxClosureMps;si.MinTargetEasMps=Config.MinTargetEasMps;si.MaxTargetEasMps=Config.MaxTargetEasMps;si.bWindValid=i.bWindValid;si.bRatioValid=i.bRatioValid;si.bDecelerationEnvelopeValid=Config.bDecelerationEnvelopeValid;auto speedOut=CaptureSpeedPlanner::Compute(si);
 FormationPlannerV2Output o;o.Path=sample;o.Mode=Mode;o.PathGeneration=PathGeneration;o.ProgressS=ProgressS;o.bPathValid=true;o.TargetEasMps=speedOut.TargetEasMps;o.bTargetEasValid=speedOut.bTargetEasValid;o.bValid=o.bPathValid&&o.bTargetEasValid;o.Failure=speedOut.Failure==SpeedPlanFailure::InvalidWind?PlannerFailure::InvalidWind:(speedOut.Failure==SpeedPlanFailure::InvalidEasToTasRatio?PlannerFailure::InvalidRatio:(speedOut.Failure==SpeedPlanFailure::InvalidDecelerationEnvelope?PlannerFailure::InvalidDeceleration:PlannerFailure::None));d.ReplanReasons=reasons;d.ReplanCount=ReplanCount;d.SelectedType=ActiveDubinsType;d.HeldPathAgeS=HeldPathAge;d.bUsingHeldPath=bUsingHeldPath;d.StateAgeS=StateAge;d.ActivePathRemainingCostS=(ActivePath.GetLength()-ProgressS)/std::max(Config.ReferenceGroundSpeedFloorMps,i.FollowerGroundVelocityNE.Norm());return o;
}
}
