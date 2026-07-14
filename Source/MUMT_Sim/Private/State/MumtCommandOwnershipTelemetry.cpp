#include "State/MumtCommandOwnershipTelemetry.h"

#include "JSBSimMovementComponent.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Misc/App.h"

DEFINE_LOG_CATEGORY_STATIC(LogMumtCmdOwn, Display, All);

namespace MumtCommandOwnership
{
namespace
{

// The subset of the command block the registered writers actually touch. Everything else (trims,
// brakes, flaps, gear) is written by nobody in this project, so it is not tracked -- adding it would
// only add noise. If a future writer touches them this must grow with it.
struct FSnapshot
{
	double Aileron = 0.0, Elevator = 0.0, Rudder = 0.0, SpeedBrake = 0.0;
	double Throttle = 0.0;
	bool bCutOff = false;
	bool bValid = false;

	bool operator==(const FSnapshot &O) const
	{
		return Aileron == O.Aileron && Elevator == O.Elevator && Rudder == O.Rudder
		    && SpeedBrake == O.SpeedBrake && Throttle == O.Throttle && bCutOff == O.bCutOff;
	}
	bool operator!=(const FSnapshot &O) const { return !(*this == O); }

	FString ToString() const
	{
		return FString::Printf(TEXT("ail=%.6f elv=%.6f rud=%.6f spb=%.6f thr=%.6f cutoff=%d"),
			Aileron, Elevator, Rudder, SpeedBrake, Throttle, bCutOff ? 1 : 0);
	}
};

FSnapshot Capture(const FFlightControlCommands &C, const TArray<FEngineCommand> &E)
{
	FSnapshot S;
	S.Aileron = C.Aileron;
	S.Elevator = C.Elevator;
	S.Rudder = C.Rudder;
	S.SpeedBrake = C.SpeedBrake;
	if (E.Num() > 0)
	{
		S.Throttle = E[0].Throttle;
		S.bCutOff = E[0].CutOff;
	}
	S.bValid = true;
	return S;
}

struct FWriteRecord
{
	EWriterId Writer = EWriterId::None;
	uint8 AxisMask = 0;
	uint64 Frame = 0;
	double WorldTimeS = 0.0;
	bool bGameThread = true;
	FSnapshot After;
};

// One of these per JSBSim component, i.e. per aircraft. Aircraft are independent by construction:
// nothing here is shared between components except the aggregate counters.
struct FPerComponent
{
	FString ActorName;
	int64 ConsumeSeq = 0;
	TArray<FWriteRecord> WritesSinceConsume;
	FSnapshot LastNotifiedSnapshot;   // what the last REGISTERED writer left behind
	FSnapshot LastConsumed;
	EWriterId LastWriterOfPrevConsume = EWriterId::None;
	uint64 LastConsumeFrame = 0;
	bool bHaveConsumed = false;

	// per-aircraft tallies, so the report can show that two aircraft did not interfere
	int64 Consumes = 0;
	int64 Writes = 0;
	int64 MultiWriterConsumes = 0;
	int64 NoWriterConsumes = 0;
	int64 Unattributed = 0;
	int64 WritesByWriter[static_cast<int32>(EWriterId::Count)] = {};
};

bool GEnabled = false;
FString GScenario;
FCounters GCounters;
TMap<const UJSBSimMovementComponent *, FPerComponent> GPerComponent;
TArray<FString> GEventLog;          // bounded; the sequence evidence
constexpr int32 kMaxEventLines = 4000;

void LogEvent(const FString &Line)
{
	if (GEventLog.Num() < kMaxEventLines)
	{
		GEventLog.Add(Line);
	}
}

bool IsFinite(const FSnapshot &S)
{
	return FMath::IsFinite(S.Aileron) && FMath::IsFinite(S.Elevator) && FMath::IsFinite(S.Rudder)
	    && FMath::IsFinite(S.SpeedBrake) && FMath::IsFinite(S.Throttle);
}

// Observed, never enforced. The point is to find out whether anything already writes out of range,
// not to start clamping (that would change what JSBSim consumes, which this task forbids).
bool IsOutOfRange(const FSnapshot &S)
{
	auto Bad = [](double V) { return V < -1.0 || V > 1.0; };
	return Bad(S.Aileron) || Bad(S.Elevator) || Bad(S.Rudder)
	    || S.SpeedBrake < 0.0 || S.SpeedBrake > 1.0
	    || S.Throttle < 0.0 || S.Throttle > 1.0;
}

FString AxisMaskToString(uint8 M)
{
	FString R;
	if (M & Axis_Aileron)    R += TEXT("A");
	if (M & Axis_Elevator)   R += TEXT("E");
	if (M & Axis_Rudder)     R += TEXT("R");
	if (M & Axis_Throttle)   R += TEXT("T");
	if (M & Axis_SpeedBrake) R += TEXT("S");
	if (M & Axis_CutOff)     R += TEXT("C");
	return R.IsEmpty() ? TEXT("-") : R;
}

} // namespace

const TCHAR *WriterName(EWriterId Id)
{
	switch (Id)
	{
	case EWriterId::HealthHardover:     return TEXT("HealthHardover");
	case EWriterId::InnerLoopAutopilot: return TEXT("InnerLoopAutopilot");
	case EWriterId::ManualUdp:          return TEXT("ManualUdp");
	default:                            return TEXT("None");
	}
}

bool IsEnabled() { return GEnabled; }

void SetEnabled(bool bEnable)
{
	GEnabled = bEnable;
	if (bEnable)
	{
		UJSBSimMovementComponent::CommandConsumeObserver.BindStatic(&OnConsume);
	}
	else
	{
		UJSBSimMovementComponent::CommandConsumeObserver.Unbind();
	}
}

void ResetSession(const FString &ScenarioLabel)
{
	GScenario = ScenarioLabel;
	GCounters = FCounters{};
	GPerComponent.Reset();
	GEventLog.Reset();
}

void NotifyWrite(EWriterId Writer, const UJSBSimMovementComponent *Component, uint8 AxisMask)
{
	if (!GEnabled || Component == nullptr)
	{
		return;
	}

	FPerComponent &S = GPerComponent.FindOrAdd(Component);
	if (S.ActorName.IsEmpty())
	{
		S.ActorName = Component->GetOwner() ? Component->GetOwner()->GetActorNameOrLabel() : TEXT("<no-owner>");
	}

	FWriteRecord R;
	R.Writer = Writer;
	R.AxisMask = AxisMask;
	R.Frame = GFrameCounter;
	R.WorldTimeS = FApp::GetCurrentTime();
	R.bGameThread = IsInGameThread();
	// Read the values back OUT of the component: this records what the writer actually left in the
	// block, which is the only thing that can reach the FCS. It is a read; nothing is written back.
	R.After = Capture(Component->Commands, Component->EngineCommands);

	// A write that lands in the same frame as, but after, this component's consume is a one-frame
	// delay: it cannot reach the FDM until the NEXT step. This is the measurement that decides the
	// ordering question the source could not answer.
	if (S.bHaveConsumed && R.Frame == S.LastConsumeFrame)
	{
		++GCounters.WritesAfterConsumeSameFrame;
	}

	for (const FWriteRecord &Prev : S.WritesSinceConsume)
	{
		if (Prev.Writer == Writer)
		{
			++GCounters.DuplicateWriteCount;
			break;
		}
	}

	S.WritesSinceConsume.Add(R);
	S.LastNotifiedSnapshot = R.After;

	++GCounters.TotalWriteEvents;
	++GCounters.WritesByWriter[static_cast<int32>(Writer)];
	++S.Writes;
	++S.WritesByWriter[static_cast<int32>(Writer)];

	LogEvent(FString::Printf(
		TEXT("[CMDOWN] WRITE seq=%lld frame=%llu t=%.4f actor=%s writer=%s axes=%s gameThread=%d %s"),
		GCounters.TotalWriteEvents, R.Frame, R.WorldTimeS, *S.ActorName, WriterName(Writer),
		*AxisMaskToString(AxisMask), R.bGameThread ? 1 : 0, *R.After.ToString()));
}

void OnConsume(const UJSBSimMovementComponent *Component, const FFlightControlCommands &Commands,
               const TArray<FEngineCommand> &EngineCommands)
{
	if (!GEnabled || Component == nullptr)
	{
		return;
	}

	FPerComponent &S = GPerComponent.FindOrAdd(Component);
	if (S.ActorName.IsEmpty())
	{
		S.ActorName = Component->GetOwner() ? Component->GetOwner()->GetActorNameOrLabel() : TEXT("<no-owner>");
	}

	const FSnapshot Consumed = Capture(Commands, EngineCommands);
	const int32 WriteCount = S.WritesSinceConsume.Num();

	// distinct writers between the previous consume and this one
	bool bSeen[static_cast<int32>(EWriterId::Count)] = {};
	int32 Unique = 0;
	FString WriterIds, WriterSeq;
	for (const FWriteRecord &W : S.WritesSinceConsume)
	{
		const int32 Idx = static_cast<int32>(W.Writer);
		if (!bSeen[Idx]) { bSeen[Idx] = true; ++Unique; WriterIds += FString::Printf(TEXT("%s|"), WriterName(W.Writer)); }
		WriterSeq += FString::Printf(TEXT("%s>"), WriterName(W.Writer));
	}
	const EWriterId LastWriter = WriteCount > 0 ? S.WritesSinceConsume.Last().Writer : EWriterId::None;

	// UNATTRIBUTED CHANGE: the block JSBSim is about to consume differs from what the last registered
	// writer left behind. Something that does not report itself moved it -- a Blueprint node, or any
	// other unregistered path. Only meaningful once a registered writer has actually written.
	bool bUnattributed = false;
	if (S.LastNotifiedSnapshot.bValid && Consumed != S.LastNotifiedSnapshot)
	{
		bUnattributed = true;
		++GCounters.UnattributedChangeCount;
		++S.Unattributed;

		FString Changed;
		if (Consumed.Aileron    != S.LastNotifiedSnapshot.Aileron)    Changed += TEXT("aileron ");
		if (Consumed.Elevator   != S.LastNotifiedSnapshot.Elevator)   Changed += TEXT("elevator ");
		if (Consumed.Rudder     != S.LastNotifiedSnapshot.Rudder)     Changed += TEXT("rudder ");
		if (Consumed.SpeedBrake != S.LastNotifiedSnapshot.SpeedBrake) Changed += TEXT("speedbrake ");
		if (Consumed.Throttle   != S.LastNotifiedSnapshot.Throttle)   Changed += TEXT("throttle ");
		if (Consumed.bCutOff    != S.LastNotifiedSnapshot.bCutOff)    Changed += TEXT("cutoff ");

		LogEvent(FString::Printf(
			TEXT("[CMDOWN] UNATTRIBUTED_CHANGE frame=%llu actor=%s component=%s changed=[%s] "
			     "lastNotifiedWriter=%s before={%s} atConsume={%s}"),
			GFrameCounter, *S.ActorName, *Component->GetName(), *Changed.TrimEnd(),
			WriterName(LastWriter != EWriterId::None ? LastWriter : S.LastWriterOfPrevConsume),
			*S.LastNotifiedSnapshot.ToString(), *Consumed.ToString()));
	}

	if (WriteCount == 0) { ++GCounters.NoWriterConsumeCount; ++S.NoWriterConsumes; }
	if (Unique > 1)      { ++GCounters.MultiWriterConsumeCount; ++S.MultiWriterConsumes; }
	if (WriteCount > GCounters.MaxWritesPerConsume) { GCounters.MaxWritesPerConsume = WriteCount; }
	if (!IsFinite(Consumed))    { ++GCounters.NonFiniteObservationCount; }
	if (IsOutOfRange(Consumed)) { ++GCounters.RangeViolationObservationCount; }
	if (LastWriter != EWriterId::None && S.bHaveConsumed && LastWriter != S.LastWriterOfPrevConsume)
	{
		++GCounters.OwnershipTransitionCount;
	}

	++GCounters.Consumes;
	++S.Consumes;
	++S.ConsumeSeq;

	LogEvent(FString::Printf(
		TEXT("[CMDOWN] CONSUME seq=%lld frame=%llu t=%.4f actor=%s writes=%d unique=%d writers=[%s] "
		     "order=%s last=%s unattributed=%d %s"),
		S.ConsumeSeq, GFrameCounter, FApp::GetCurrentTime(), *S.ActorName, WriteCount, Unique,
		*WriterIds, WriterSeq.IsEmpty() ? TEXT("-") : *WriterSeq, WriterName(LastWriter),
		bUnattributed ? 1 : 0, *Consumed.ToString()));

	if (LastWriter != EWriterId::None)
	{
		S.LastWriterOfPrevConsume = LastWriter;
	}
	S.LastConsumed = Consumed;
	S.LastConsumeFrame = GFrameCounter;
	S.bHaveConsumed = true;
	S.WritesSinceConsume.Reset();
}

TArray<FString> BuildReport()
{
	TArray<FString> Out;
	const FCounters &C = GCounters;

	Out.Add(FString::Printf(TEXT("[CMDOWN] SCENARIO=%s"), *GScenario));
	Out.Add(FString::Printf(
		TEXT("[CMDOWN] TOTALS total_write_events=%lld consumes=%lld no_writer_consume_count=%lld "
		     "duplicate_write_count=%lld multi_writer_consume_count=%lld max_writes_per_consume=%d"),
		C.TotalWriteEvents, C.Consumes, C.NoWriterConsumeCount, C.DuplicateWriteCount,
		C.MultiWriterConsumeCount, C.MaxWritesPerConsume));
	Out.Add(FString::Printf(
		TEXT("[CMDOWN] WRITERS HealthHardover=%lld InnerLoopAutopilot=%lld ManualUdp=%lld"),
		C.WritesByWriter[static_cast<int32>(EWriterId::HealthHardover)],
		C.WritesByWriter[static_cast<int32>(EWriterId::InnerLoopAutopilot)],
		C.WritesByWriter[static_cast<int32>(EWriterId::ManualUdp)]));
	Out.Add(FString::Printf(
		TEXT("[CMDOWN] OBSERVATIONS unattributed_command_change_count=%lld "
		     "writes_after_consume_same_frame=%lld ownership_transition_observation_count=%lld "
		     "non_finite_observation_count=%lld range_violation_observation_count=%lld"),
		C.UnattributedChangeCount, C.WritesAfterConsumeSameFrame, C.OwnershipTransitionCount,
		C.NonFiniteObservationCount, C.RangeViolationObservationCount));

	for (const TPair<const UJSBSimMovementComponent *, FPerComponent> &P : GPerComponent)
	{
		const FPerComponent &S = P.Value;
		Out.Add(FString::Printf(
			TEXT("[CMDOWN] AIRCRAFT actor=%s consumes=%lld writes=%lld health=%lld autopilot=%lld "
			     "manual=%lld multi_writer_consumes=%lld no_writer_consumes=%lld unattributed=%lld "
			     "last_consumed={%s}"),
			*S.ActorName, S.Consumes, S.Writes,
			S.WritesByWriter[static_cast<int32>(EWriterId::HealthHardover)],
			S.WritesByWriter[static_cast<int32>(EWriterId::InnerLoopAutopilot)],
			S.WritesByWriter[static_cast<int32>(EWriterId::ManualUdp)],
			S.MultiWriterConsumes, S.NoWriterConsumes, S.Unattributed, *S.LastConsumed.ToString()));
	}

	Out.Add(FString::Printf(TEXT("[CMDOWN] EVENT_LOG lines=%d (capped at %d)"), GEventLog.Num(), kMaxEventLines));
	Out.Append(GEventLog);
	return Out;
}

const FCounters &GetCounters() { return GCounters; }

bool GetWritesForActor(const FString &ActorName, EWriterId Writer, int64 &OutWrites)
{
	for (const TPair<const UJSBSimMovementComponent *, FPerComponent> &P : GPerComponent)
	{
		if (P.Value.ActorName == ActorName)
		{
			OutWrites = P.Value.WritesByWriter[static_cast<int32>(Writer)];
			return true;
		}
	}
	return false;
}

} // namespace MumtCommandOwnership
