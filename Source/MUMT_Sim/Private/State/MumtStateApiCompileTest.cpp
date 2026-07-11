// MumtStateApiCompileTest.cpp — dev-only compile/automation coverage for the read-only State
// API adapter. Forces UBT to compile State/MumtControlState.h and to instantiate the templated
// MumtState::ConvertJsbToControlState with the REAL plugin FJsbFlightSnapshot type, producing
// FMumtControlState. No runtime Tick, no Actor wiring, no logging and no control output. The
// whole translation unit is empty outside dev/automation builds (WITH_DEV_AUTOMATION_TESTS), so
// it adds no production runtime API.
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "JSBSimMovementComponent.h" // FJsbFlightSnapshot (plugin public header, raw snapshot POD)
#include "State/MumtControlState.h"  // MumtState::ConvertJsbToControlState, FMumtControlState

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtStateApiAdapterCompileTest,
	"MUMT.StateApi.AdapterCompiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMumtStateApiAdapterCompileTest::RunTest(const FString& /*Parameters*/)
{
	// Instantiate the templated adapter with the real plugin snapshot type. This is the point
	// where FJsbFlightSnapshot -> MumtState::ConvertJsbToControlState -> FMumtControlState is
	// compiled by UBT. No JSBSim objects, no Actor, no side effects.
	//
	// Explicit value-initialization: no unset field of the snapshot or tracker may reach the
	// adapter uninitialized, even though both structs already carry default member initializers.
	FJsbFlightSnapshot Snapshot{};
	Snapshot.bValidFrame    = true;
	Snapshot.VequivalentKTS = 100.0; // EAS -> 51.4444 m/s
	Snapshot.VtFps          = 300.0; // TAS -> 91.44   m/s

	MumtState::FMumtStateTracker Tracker{};
	const MumtState::FMumtControlState State = MumtState::ConvertJsbToControlState(Snapshot, Tracker);

	// Meaningful assertions on the conversion result (also exercised when the test is run).
	TestTrue(TEXT("EAS valid"), State.bEasValid);
	TestTrue(TEXT("eas_to_tas valid"), State.bRatioValid);
	// eas_to_tas = TAS/EAS = 91.44 / 51.4444 ~ 1.7774
	TestTrue(TEXT("eas_to_tas = TAS/EAS ~ 1.7774"),
	         FMath::IsNearlyEqual(State.EasToTasRatio, 1.7774, 1.0e-3));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
