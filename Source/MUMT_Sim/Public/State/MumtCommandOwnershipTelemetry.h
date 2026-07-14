// MumtCommandOwnershipTelemetry.h — who writes the JSBSim command block, and what JSBSim consumed.
//
// The read-only audit established that three C++ writers set UJSBSimMovementComponent::Commands
// (Falling hardover, the InnerLoop autopilot, and the manual/UDP path), that they run in three
// different execution contexts (Component Tick / 60 Hz timer / Actor Tick) with NO TickGroup and NO
// tick prerequisite anywhere, and that `Commands` is BlueprintReadWrite so a Blueprint could write it
// too. What it could NOT establish from source alone was the actual ordering, whether two writers hit
// the same aircraft between two FDM steps, and whether anything unregistered mutates the block.
//
// This facility answers those questions by MEASUREMENT. It observes; it never arbitrates:
//   * each C++ writer reports itself immediately AFTER it has written (the write already happened),
//   * UJSBSimMovementComponent::CopyToJSBSim reports the block it is about to hand to the FCS,
//     by const reference, before the first FCS setter and before Exec->Run().
//
// Nothing here modifies a command, and the whole facility is INERT unless a test enables it, so the
// commands JSBSim consumes are bit-for-bit the same with telemetry off and on.
//
// LIMIT, stated up front: an unattributed change is detected by comparing the block at consume time
// against the last snapshot a registered writer reported. A Blueprint (or any unregistered path) that
// writes the SAME values a registered writer already wrote is therefore INVISIBLE to this method --
// it changes nothing to detect. This tells us "something unregistered changed the commands", never
// "nothing unregistered touched the commands".
#pragma once

#include "CoreMinimal.h"

class UJSBSimMovementComponent;
struct FFlightControlCommands;
struct FEngineCommand;

namespace MumtCommandOwnership
{
// The registered writers. Anything that moves the command block without reporting itself as one of
// these shows up as an unattributed change.
enum class EWriterId : uint8
{
	None = 0,
	HealthHardover,       // UHealthComponent::TickComponent   (Component Tick, Falling only)
	InnerLoopAutopilot,   // AUDPControlReceiver::ApplyAutopilotToPawn (60 Hz FTimerHandle)
	ManualUdp,            // AUDPControlReceiver::ApplyControlCommandToPawn (Actor Tick)
	Count
};

const TCHAR *WriterName(EWriterId Id);

// Which axes a writer actually touched. Recorded from the writer's own code path, not inferred.
enum EAxisMask : uint8
{
	Axis_None       = 0,
	Axis_Aileron    = 1 << 0,
	Axis_Elevator   = 1 << 1,
	Axis_Rudder     = 1 << 2,
	Axis_Throttle   = 1 << 3,
	Axis_SpeedBrake = 1 << 4,
	Axis_CutOff     = 1 << 5,
};

// Off by default. While disabled every entry point below is a single bool test and returns, so the
// command stream is untouched. The tests turn it on; nothing in production ever does.
bool IsEnabled();
void SetEnabled(bool bEnable);

// Clears all counters and per-component state, and stamps the scenario label used in the report.
void ResetSession(const FString &ScenarioLabel);

// Called by a writer immediately AFTER it has written to Commands/EngineCommands. It passes the
// component it wrote to; the telemetry reads the CURRENT values back from that component, so what is
// recorded is what the writer actually left behind, not what it intended.
void NotifyWrite(EWriterId Writer, const UJSBSimMovementComponent *Component, uint8 AxisMask);

// Registered as UJSBSimMovementComponent::CommandConsumeObserver while enabled.
void OnConsume(const UJSBSimMovementComponent *Component, const FFlightControlCommands &Commands,
               const TArray<FEngineCommand> &EngineCommands);

// Human-readable report, one [CMDOWN] line per fact, for the Automation log.
TArray<FString> BuildReport();

// Aggregate counters, exposed so an Automation test can assert on them.
struct FCounters
{
	int64 TotalWriteEvents = 0;
	int64 WritesByWriter[static_cast<int32>(EWriterId::Count)] = {};
	int64 Consumes = 0;
	int64 NoWriterConsumeCount = 0;          // a consume with zero writes since the previous consume
	int64 DuplicateWriteCount = 0;           // same writer wrote twice between two consumes
	int64 MultiWriterConsumeCount = 0;       // >1 DISTINCT writer between two consumes (the overlap case)
	int64 UnattributedChangeCount = 0;       // block differs from the last writer-reported snapshot
	int64 NonFiniteObservationCount = 0;     // observed only; never rejected or clamped here
	int64 RangeViolationObservationCount = 0;// observed only; never clamped here
	int64 OwnershipTransitionCount = 0;      // last-writer identity changed between consumes
	int64 WritesAfterConsumeSameFrame = 0;   // a write landed in the same frame AFTER the consume ran
	int32 MaxWritesPerConsume = 0;
};
const FCounters &GetCounters();

// Per-aircraft write counts, so a test can ask "did THIS aircraft stop being written by the autopilot
// after it started falling" rather than only reasoning about fleet-wide totals. Returns false if the
// aircraft was never seen.
bool GetWritesForActor(const FString &ActorName, EWriterId Writer, int64 &OutWrites);

} // namespace MumtCommandOwnership
