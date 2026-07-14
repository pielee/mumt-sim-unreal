// MumtCommandOwnershipTest.cpp — MEASURES who owns the JSBSim command block. Changes nothing.
//
// The read-only audit could not answer three questions from source alone:
//   1. In what order do the writers and the FDM consume actually run? There is no TickGroup and no
//      tick prerequisite anywhere, and the writers live in three different contexts (Component Tick,
//      Actor Tick, 60 Hz timer), so the source simply does not say.
//   2. Can two writers hit the SAME aircraft between two FDM steps?
//   3. `Commands` is BlueprintReadWrite -- does anything unregistered actually move it?
//
// These tests answer them by running the real thing and recording the real event sequence. They add
// no arbitration, no clamp, no fallback: the command stream is identical with telemetry off and on.
//
// Scenarios (one Automation test each, one editor process each -- see the runner):
//   A  AutopilotOnly   -FormationTest, no manual command      -> expect InnerLoopAutopilot only
//   B  ManualOnly      no -FormationTest, UDP command sent    -> expect ManualUdp only
//   C  Overlap         -FormationTest AND a UDP command for the SAME pawn (M_F16 is both the
//                      formation leader and a controlled pawn) -> expect BOTH writers on one aircraft
//   D  Falling         -FormationTest, then real damage        -> expect HealthHardover to take over
//   E  BlueprintAudit  no PIE; walks the Blueprint graphs looking for command-write nodes
#include "Misc/AutomationTest.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

#include "State/MumtCommandOwnershipTelemetry.h"
#include "HealthComponent.h"
#include "UDPControlReceiver.h"
#include "JSBSimMovementComponent.h"
#include "Tests/AutomationEditorCommon.h"
#include "Editor.h"
#include "Engine/Blueprint.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EdGraph/EdGraph.h"
#include "EdGraph/EdGraphNode.h"
#include "EdGraph/EdGraphPin.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Common/UdpSocketBuilder.h"
#include "IPAddress.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

DEFINE_LOG_CATEGORY_STATIC(LogMumtCmdOwnTest, Display, All);

namespace
{
// TU-distinct names: unity builds concatenate this with the other State/ tests.
const TCHAR *kOwnMap = TEXT("/Game/RL_2");
constexpr double kOwnMaxWallSeconds = 300.0;
constexpr double kOwnRunSeconds = 30.0;    // observation window, simulated seconds
constexpr double kOwnDamageAtS = 15.0;     // scenario D: damage the follower mid-flight
constexpr int32  kOwnUdpPort = 5005;       // AUDPControlReceiver::ListenPort default

enum class EOwnScenario : uint8 { AutopilotOnly, ManualOnly, Overlap, Falling };

const TCHAR *ScenarioName(EOwnScenario S)
{
	switch (S)
	{
	case EOwnScenario::AutopilotOnly: return TEXT("A_AutopilotOnly");
	case EOwnScenario::ManualOnly:    return TEXT("B_ManualOnly");
	case EOwnScenario::Overlap:       return TEXT("C_Overlap");
	default:                          return TEXT("D_Falling");
	}
}

// The manual/UDP path is driven through the REAL socket the joystick uses, not by reaching into the
// receiver's private state. That keeps the production code untouched and exercises the actual parse
// -> NamedControlCommands -> ApplyControlCommandToPawn chain.
struct FOwnUdpSender
{
	FSocket *Socket = nullptr;
	TSharedPtr<FInternetAddr> Addr;

	bool Init()
	{
		ISocketSubsystem *SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
		if (!SS) return false;
		Socket = SS->CreateSocket(NAME_DGram, TEXT("MumtCmdOwnTestSender"), false);
		if (!Socket) return false;
		Addr = SS->CreateInternetAddr();
		bool bValid = false;
		Addr->SetIp(TEXT("127.0.0.1"), bValid);
		Addr->SetPort(kOwnUdpPort);
		return bValid;
	}

	void Send(const FString &AircraftName, double Roll, double Pitch, double Yaw, double Throttle)
	{
		if (!Socket || !Addr.IsValid()) return;
		const FString Msg = FString::Printf(
			TEXT("{\"commands\":[{\"aircraft_name\":\"%s\",\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f,\"throttle\":%.4f}]}"),
			*AircraftName, Roll, Pitch, Yaw, Throttle);
		FTCHARToUTF8 Utf8(*Msg);
		int32 Sent = 0;
		Socket->SendTo(reinterpret_cast<const uint8 *>(Utf8.Get()), Utf8.Length(), Sent, *Addr);
	}

	void Close()
	{
		if (Socket)
		{
			ISocketSubsystem *SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
			Socket->Close();
			if (SS) SS->DestroySocket(Socket);
			Socket = nullptr;
		}
	}
};

struct FOwnState
{
	double FirstWall = 0.0;
	double FirstSim = -1.0;
	bool bStarted = false;
	bool bDamaged = false;
	int32 Ticks = 0;
	FOwnUdpSender Udp;
	FString TargetName;      // the aircraft the manual command is aimed at
	FString FallingActor;         // scenario D: the aircraft actually damaged
	int64 AutoWritesAtDamage = 0; // its autopilot write count at the instant of damage
	bool bSawAnyComponent = false;
};

// The simulation clock comes from the aircraft itself, not from wall time -- the same atomic snapshot
// getter the shadow test uses, so both tests agree on what "sim time" means.
double GetSimTime(UWorld *World)
{
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (UJSBSimMovementComponent *C = It->FindComponentByClass<UJSBSimMovementComponent>())
		{
			FJsbFlightSnapshot S{};
			if (C->GetJsbFlightSnapshot(S) && S.bValidFrame)
			{
				return S.SimTimeSec;
			}
		}
	}
	return -1.0;
}

class FMumtOwnSampleCommand : public IAutomationLatentCommand
{
public:
	FMumtOwnSampleCommand(FAutomationTestBase *T, TSharedPtr<FOwnState> S, EOwnScenario InScenario)
		: Test(T), St(S), Scenario(InScenario) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		UWorld *World = nullptr;
		if (GEditor && GEditor->PlayWorld) World = GEditor->PlayWorld;
		if (!World) return false;

		if (!St->bStarted)
		{
			St->bStarted = true;
			St->FirstWall = NowWall;
			MumtCommandOwnership::SetEnabled(true);
			MumtCommandOwnership::ResetSession(ScenarioName(Scenario));
			St->Udp.Init();

			// The manual command targets the FORMATION LEADER (M_F16). It is in
			// ControlledPawnNamePatterns AND it is the aircraft FormationTest drives with a setpoint,
			// which is exactly what makes the Overlap scenario a real overlap rather than a contrived
			// one: two production writers, same aircraft, same frame budget.
			St->TargetName = TEXT("M_F16");
		}

		++St->Ticks;

		// Manual scenarios: keep a live named command in flight. The receiver drains the socket in its
		// Tick and applies it in the same Tick.
		if (Scenario == EOwnScenario::ManualOnly || Scenario == EOwnScenario::Overlap)
		{
			St->Udp.Send(St->TargetName, 0.15, -0.05, 0.0, 0.7);
		}

		const double SimT = GetSimTime(World);
		if (SimT >= 0.0)
		{
			St->bSawAnyComponent = true;
			if (St->FirstSim < 0.0) St->FirstSim = SimT;
		}
		const double Elapsed = (St->FirstSim >= 0.0 && SimT >= 0.0) ? (SimT - St->FirstSim) : 0.0;

		// Scenario D: take the aircraft out for real, through the production damage API.
		//
		// The aircraft MUST be one the autopilot is actually flying, otherwise the scenario proves
		// nothing: a hardover on an idle spare shows a hardover, but not a hand-over. So the target is
		// selected by evidence -- the telemetry itself is asked which aircraft the autopilot has been
		// writing to -- and the count at that moment is remembered, so Finalize can prove the autopilot
		// STOPPED writing to it afterwards.
		//
		// (Naming, learned the hard way: GetName() yields "F16_UAV_C_0" while the label is "F16_UAV1".
		//  The telemetry keys on the label, which is what the rest of the project uses, so match that.)
		if (Scenario == EOwnScenario::Falling && !St->bDamaged && Elapsed >= kOwnDamageAtS)
		{
			for (TActorIterator<AActor> It(World); It; ++It)
			{
				UHealthComponent *H = It->FindComponentByClass<UHealthComponent>();
				const UJSBSimMovementComponent *J = It->FindComponentByClass<UJSBSimMovementComponent>();
				if (!H || !J || !H->IsAlive()) continue;

				const FString Label = It->GetActorNameOrLabel();
				if (!Label.Contains(TEXT("UAV"))) continue;   // leave the manned leader flying

				int64 AutoWrites = 0;
				if (!MumtCommandOwnership::GetWritesForActor(
						Label, MumtCommandOwnership::EWriterId::InnerLoopAutopilot, AutoWrites)
					|| AutoWrites <= 0)
				{
					continue;   // this UAV is not being flown -- damaging it would prove nothing
				}

				H->ApplyDamage(1.0e6f, nullptr);
				St->bDamaged = true;
				St->FallingActor = Label;
				St->AutoWritesAtDamage = AutoWrites;
				UE_LOG(LogMumtCmdOwnTest, Display,
					TEXT("[CMDOWN] scenario D: damaged %s (an aircraft the autopilot HAD been flying: "
					     "%lld autopilot writes so far) at sim t=%.2f"),
					*Label, AutoWrites, SimT);
				break;
			}
			if (!St->bDamaged)
			{
				Test->AddError(TEXT("[CMDOWN] scenario D: no autopilot-flown UAV was available to damage"));
				return Finalize();
			}
		}

		const bool bDone = (Elapsed >= kOwnRunSeconds) || ((NowWall - St->FirstWall) >= kOwnMaxWallSeconds);
		if (!bDone) return false;

		return Finalize();
	}

private:
	bool Finalize()
	{
		St->Udp.Close();

		for (const FString &Line : MumtCommandOwnership::BuildReport())
		{
			UE_LOG(LogMumtCmdOwnTest, Display, TEXT("%s"), *Line);
		}
		const MumtCommandOwnership::FCounters C = MumtCommandOwnership::GetCounters();
		MumtCommandOwnership::SetEnabled(false);

		using W = MumtCommandOwnership::EWriterId;
		const int64 Health   = C.WritesByWriter[static_cast<int32>(W::HealthHardover)];
		const int64 Auto     = C.WritesByWriter[static_cast<int32>(W::InnerLoopAutopilot)];
		const int64 Manual   = C.WritesByWriter[static_cast<int32>(W::ManualUdp)];

		Test->TestTrue(TEXT("the JSBSim command block was observed being consumed"), C.Consumes > 0);
		Test->TestTrue(TEXT("telemetry saw at least one aircraft"), St->bSawAnyComponent);

		// These are OBSERVATIONS, not policy. They are asserted because if they ever became non-zero
		// the Active Writer design would have to account for it, and silence would hide that.
		Test->TestEqual(TEXT("no non-finite command was consumed"), C.NonFiniteObservationCount, (int64)0);

		switch (Scenario)
		{
		case EOwnScenario::AutopilotOnly:
			Test->TestTrue(TEXT("A: the autopilot wrote commands"), Auto > 0);
			Test->TestEqual(TEXT("A: the manual/UDP writer never ran"), Manual, (int64)0);
			Test->TestEqual(TEXT("A: the Falling hardover never ran"), Health, (int64)0);
			break;

		case EOwnScenario::ManualOnly:
			Test->TestTrue(TEXT("B: the manual/UDP writer wrote commands"), Manual > 0);
			Test->TestEqual(TEXT("B: the autopilot never ran (no setpoints exist)"), Auto, (int64)0);
			Test->TestEqual(TEXT("B: the Falling hardover never ran"), Health, (int64)0);
			break;

		case EOwnScenario::Overlap:
			// The point of the scenario: prove two production writers really can own one aircraft.
			Test->TestTrue(TEXT("C: the autopilot wrote commands"), Auto > 0);
			Test->TestTrue(TEXT("C: the manual/UDP writer wrote commands"), Manual > 0);
			Test->TestTrue(TEXT("C: both writers hit the SAME aircraft between two FDM consumes"),
				C.MultiWriterConsumeCount > 0);
			break;

		case EOwnScenario::Falling:
		{
			Test->TestTrue(TEXT("D: an aircraft was actually damaged"), St->bDamaged);
			Test->TestTrue(TEXT("D: the damaged aircraft HAD been flown by the autopilot"),
				St->AutoWritesAtDamage > 0);

			// The ownership question, on the ONE aircraft that changed hands: after it started falling,
			// the hardover must be writing it and the autopilot must have stopped. Fleet-wide totals
			// cannot show this -- the other aircraft keep flying -- so it is asked per aircraft.
			int64 AutoAfter = 0, HealthOnTarget = 0;
			const bool bFoundAuto = MumtCommandOwnership::GetWritesForActor(
				St->FallingActor, W::InnerLoopAutopilot, AutoAfter);
			const bool bFoundHealth = MumtCommandOwnership::GetWritesForActor(
				St->FallingActor, W::HealthHardover, HealthOnTarget);
			Test->TestTrue(TEXT("D: the damaged aircraft is known to the telemetry"), bFoundAuto && bFoundHealth);
			Test->TestTrue(TEXT("D: the Falling hardover took over the damaged aircraft"), HealthOnTarget > 0);
			Test->TestEqual(TEXT("D: the autopilot stopped writing the damaged aircraft (early return)"),
				AutoAfter, St->AutoWritesAtDamage);

			UE_LOG(LogMumtCmdOwnTest, Display,
				TEXT("[CMDOWN] D_FALLING_TARGET actor=%s autopilot_writes_at_damage=%lld "
				     "autopilot_writes_at_end=%lld hardover_writes=%lld"),
				*St->FallingActor, St->AutoWritesAtDamage, AutoAfter, HealthOnTarget);
			Test->TestTrue(TEXT("D: the fleet-wide hardover writer ran"), Health > 0);
			break;
		}
		}
		return true;
	}

	FAutomationTestBase *Test;
	TSharedPtr<FOwnState> St;
	EOwnScenario Scenario{EOwnScenario::AutopilotOnly};
};

void RunOwnershipScenario(FAutomationTestBase *T, EOwnScenario S)
{
	// Pre-existing, unrelated Blueprint Construction Script error in F16_UAV / M_F16. Content is out
	// of scope; matched narrowly so it cannot hide a command-ownership error.
	T->AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);

	TSharedPtr<FOwnState> State = MakeShared<FOwnState>();
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kOwnMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtOwnSampleCommand(T, State, S));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
}

// ---- Blueprint graph audit (no PIE) ---------------------------------------------------------------
// The binary asset strings prove only that the NAMES appear. They cannot tell a read from a write.
// This walks the actual graphs and reports every node that could write the command block, by node
// class and title. It opens the assets read-only: nothing is saved and nothing is compiled.
bool AuditBlueprintGraphs(FAutomationTestBase *T, const TCHAR *AssetPath, int32 &OutWriteNodes)
{
	OutWriteNodes = 0;
	UBlueprint *BP = LoadObject<UBlueprint>(nullptr, AssetPath);
	if (!BP)
	{
		UE_LOG(LogMumtCmdOwnTest, Display, TEXT("[CMDOWN] BP_AUDIT asset=%s NOT_LOADABLE"), AssetPath);
		return false;
	}

	TArray<UEdGraph *> Graphs;
	BP->GetAllGraphs(Graphs);

	int32 Inspected = 0;
	for (const UEdGraph *G : Graphs)
	{
		if (!G) continue;
		for (const UEdGraphNode *N : G->Nodes)
		{
			if (!N) continue;
			++Inspected;
			const FString NodeClass = N->GetClass()->GetName();
			const FString Title = N->GetNodeTitle(ENodeTitleType::ListView).ToString();

			// Any node class that can WRITE: a variable set, a struct member set, a struct make fed
			// into one. Reads (VariableGet / BreakStruct) are deliberately not counted.
			const bool bWriteClass =
				NodeClass.Contains(TEXT("K2Node_VariableSet")) ||
				NodeClass.Contains(TEXT("K2Node_SetFieldsInStruct")) ||
				NodeClass.Contains(TEXT("K2Node_StructMemberSet")) ||
				NodeClass.Contains(TEXT("K2Node_MakeStruct"));

			FString PinNames;
			bool bTouchesCommand = Title.Contains(TEXT("Commands"));
			for (const UEdGraphPin *P : N->Pins)
			{
				if (!P) continue;
				const FString PinName = P->PinName.ToString();
				PinNames += PinName + TEXT(",");
				if (PinName.Equals(TEXT("Commands")) || PinName.Equals(TEXT("EngineCommands"))
					|| PinName.Equals(TEXT("Aileron")) || PinName.Equals(TEXT("Elevator"))
					|| PinName.Equals(TEXT("Rudder")) || PinName.Equals(TEXT("Throttle")))
				{
					bTouchesCommand = true;
				}
			}

			if (bWriteClass && bTouchesCommand)
			{
				++OutWriteNodes;
				UE_LOG(LogMumtCmdOwnTest, Display,
					TEXT("[CMDOWN] BP_AUDIT asset=%s graph=%s WRITE_NODE class=%s title=\"%s\" pins=[%s]"),
					AssetPath, *G->GetName(), *NodeClass, *Title, *PinNames);
			}
		}
	}

	UE_LOG(LogMumtCmdOwnTest, Display,
		TEXT("[CMDOWN] BP_AUDIT asset=%s graphs=%d nodes_inspected=%d command_write_nodes=%d"),
		AssetPath, Graphs.Num(), Inspected, OutWriteNodes);
	return true;
}

} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtCommandOwnershipAutopilotOnlyTest,
	"MUMT.ControlV2.CommandOwnershipAutopilotOnly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtCommandOwnershipAutopilotOnlyTest::RunTest(const FString &)
{ RunOwnershipScenario(this, EOwnScenario::AutopilotOnly); return true; }

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtCommandOwnershipManualOnlyTest,
	"MUMT.ControlV2.CommandOwnershipManualOnly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtCommandOwnershipManualOnlyTest::RunTest(const FString &)
{ RunOwnershipScenario(this, EOwnScenario::ManualOnly); return true; }

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtCommandOwnershipOverlapTest,
	"MUMT.ControlV2.CommandOwnershipOverlap",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtCommandOwnershipOverlapTest::RunTest(const FString &)
{ RunOwnershipScenario(this, EOwnScenario::Overlap); return true; }

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtCommandOwnershipFallingTest,
	"MUMT.ControlV2.CommandOwnershipFalling",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtCommandOwnershipFallingTest::RunTest(const FString &)
{ RunOwnershipScenario(this, EOwnScenario::Falling); return true; }

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtCommandOwnershipBlueprintAuditTest,
	"MUMT.ControlV2.CommandOwnershipBlueprintAudit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtCommandOwnershipBlueprintAuditTest::RunTest(const FString &)
{
	int32 MannedWrites = 0, UavWrites = 0;
	const bool bMannedOk = AuditBlueprintGraphs(this, TEXT("/Game/Blueprints/M_F16.M_F16"), MannedWrites);
	const bool bUavOk = AuditBlueprintGraphs(this, TEXT("/Game/Blueprints/F16_UAV.F16_UAV"), UavWrites);

	TestTrue(TEXT("M_F16 Blueprint is loadable for inspection"), bMannedOk);
	TestTrue(TEXT("F16_UAV Blueprint is loadable for inspection"), bUavOk);

	// NOT asserted to be zero. The purpose is to REPORT what is there; if a command-write node exists,
	// the Active Writer must treat Blueprint as a Legacy input rather than pretend it does not exist.
	UE_LOG(LogMumtCmdOwnTest, Display,
		TEXT("[CMDOWN] BP_AUDIT_SUMMARY manned_command_write_nodes=%d uav_command_write_nodes=%d"),
		MannedWrites, UavWrites);
	return true;
}

#endif // WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
