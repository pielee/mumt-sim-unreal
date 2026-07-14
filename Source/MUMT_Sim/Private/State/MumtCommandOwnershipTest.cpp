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
#include "State/MumtCommandArbiterV2.h"
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

	// ---- scenario D: the evidence, captured as VALUE COPIES before the FDM is stopped ----------------
	//
	// Why copies: the telemetry and the arbiter keep counting for as long as anything ticks. An assertion
	// that reads them later is asserting about a different moment than the one it claims to be about, and
	// the numbers can move underneath it. These are the numbers AT THE INSTANT the evidence was complete,
	// and they are what the test actually proves.
	bool bEvidenceComplete = false;      // every piece below was obtained
	bool bTickDisabled = false;          // ...and ONLY then was the aircraft's FDM stopped
	bool bFallingEntered = false;        // the health component really says Falling (not merely damaged)
	int32 FallingLifeState = -1;
	int64 FallingResolutionsAtEvidence = 0;    // a Falling-resolved command was actually CONSUMED
	int64 FormationResolutionsAtEvidence = 0;  // ...and Formation never was
	int64 HardoverWritesAtEvidence = 0;        // the hardover owns the aircraft
	int64 AutoWritesAtEvidence = 0;            // ...and the autopilot has stopped writing it
	int64 ManualWritesAtEvidence = 0;          // (the manual writer never targets the UAV at all)
	double FinalThrottle = -1.0;               // engine 0 in the final resolved block
	bool bFinalCutOff = false;
	uint64 EvidenceConsumeSequence = 0;
	int64 ConsumesAtEvidence = 0;              // resolved consumes on this aircraft when the evidence closed

	// ---- two-phase stop -------------------------------------------------------------------------------
	// Disabling the tick and finalizing in the same latent Update would assert that the aircraft stopped
	// without ever letting a frame pass to find out. The claim "it stopped" is only checkable on the NEXT
	// tick, so the stop is verified there, against the numbers captured before it.
	bool bAwaitingPostStopVerification = false;
	bool bPostStopVerified = false;
	bool bPostStopComponentAlive = false;
	bool bPostStopTickDisabled = false;
	int64 PostStopFallingResolutions = -1;
	int64 PostStopConsumes = -1;
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

		// PHASE 2 of the stop. A frame has now passed since the tick was disabled, so "the aircraft
		// stopped" is finally a checkable claim rather than an assumption made in the same breath as the
		// action. If anything moved after the stop, this is where it shows.
		if (Scenario == EOwnScenario::Falling && St->bAwaitingPostStopVerification && !St->bPostStopVerified)
		{
			VerifyFallingAircraftStopped(World);
			return Finalize();
		}

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

		// Scenario D ends WHEN IT HAS ITS EVIDENCE, not when a timer runs out.
		//
		// The aircraft under test is in a hardover dive. What this scenario has to establish is that the
		// HealthHardover takes ownership of the damaged aircraft's command block, and that the final
		// resolved block at the FDM consume boundary carries throttle 0 and cutoff. Once that is in hand,
		// everything after it is the aircraft falling further towards the ground: it establishes nothing
		// new, and it eventually drives JSBSim's aerodynamic tables outside their domain (an abort inside
		// FGTable::GetValue). So once the evidence is complete -- and not one tick before -- the aircraft's
		// FDM is stopped and the test finishes.
		//
		// This is not a shortened timeout and not a retry: no expectation is weakened, and every assertion
		// below is made on evidence collected while the aircraft was still being flown by the hardover.
		if (Scenario == EOwnScenario::Falling && St->bDamaged && !St->bEvidenceComplete)
		{
			if (TryCollectFallingEvidence(World))
			{
				StopFallingAircraftFdm(World);   // ONLY the damaged aircraft, ONLY now
				St->bAwaitingPostStopVerification = true;
				return false;                    // end THIS update; the stop is checked on the next one
			}
			const bool bOutOfTime = (Elapsed >= kOwnRunSeconds)
				|| ((NowWall - St->FirstWall) >= kOwnMaxWallSeconds);
			if (bOutOfTime)
			{
				// The hardover never reached the FDM. That is a real failure and must be reported as one,
				// not quietly finalized as though the scenario had been satisfied.
				Test->AddError(TEXT("[CMDOWN] D: the Falling evidence never completed -- the hardover "
				                    "never reached the FDM within the run budget"));
				return Finalize();
			}
			return false;
		}

		const bool bDone = (Elapsed >= kOwnRunSeconds) || ((NowWall - St->FirstWall) >= kOwnMaxWallSeconds);
		if (!bDone) return false;

		return Finalize();
	}

private:
	// Returns true ONLY when every required piece of evidence exists AND has been copied out. Any missing
	// piece means "not yet" -- never "close enough".
	bool TryCollectFallingEvidence(UWorld *World)
	{
		namespace Arb = MumtCommandArbiterV2;
		using W = MumtCommandOwnership::EWriterId;

		AActor *Actor = nullptr;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (It->GetActorNameOrLabel() == St->FallingActor) { Actor = *It; break; }
		}
		if (!Actor) return false;

		UJSBSimMovementComponent *Comp = Actor->FindComponentByClass<UJSBSimMovementComponent>();
		UHealthComponent *Health = Actor->FindComponentByClass<UHealthComponent>();
		if (!Comp || !Health) return false;

		// (1) It is FALLING -- the production life state says so. "Damaged" is not the same claim.
		if (Health->IsAlive()) return false;
		if (Health->LifeState != EAircraftLifeState::Falling) return false;

		// (2) A Falling-resolved block was actually produced AT THE FDM CONSUME BOUNDARY. The arbiter
		//     records the resolution there, so a non-zero count means the block was passed into the FCS
		//     setter path -- not merely that some writer wrote one.
		Arb::FAircraftCounters AC{};
		if (!Arb::GetAircraftCounters(St->FallingActor, AC)) return false;
		if (AC.FallingResolutions <= 0) return false;

		// (3) And the final resolved block at that boundary is the hardover's: engine 0 at throttle 0 with
		//     cutoff set. This reads the RESOLVED block passed into the FCS setter path, not any writer's
		//     intent.
		Arb::FResolvedCommandSnapshot Snap;
		if (!Arb::GetLastResolvedSnapshot(Comp, Snap)) return false;
		if (Snap.EngineCommands.Num() <= 0) return false;
		if (Snap.EngineCommands[0].Throttle != 0.0) return false;
		if (!Snap.EngineCommands[0].CutOff) return false;

		// (4) OWNERSHIP, per aircraft: HealthHardover is writing this aircraft, the autopilot has stopped
		//     writing it since the Falling transition, and the manual writer never participates on it at
		//     all (it aims at the manned leader).
		//
		//     This is an ownership observation, NOT a priority proof. That the arbiter would prefer Falling
		//     over a live Formation candidate is a different claim, established by
		//     MUMT.ControlV2.ArbiterFallingOverCandidate -- not by these write counts.
		int64 HardWrites = 0, AutoNow = 0, ManualNow = 0;
		if (!MumtCommandOwnership::GetWritesForActor(St->FallingActor, W::HealthHardover, HardWrites)) return false;
		if (!MumtCommandOwnership::GetWritesForActor(St->FallingActor, W::InnerLoopAutopilot, AutoNow)) return false;
		MumtCommandOwnership::GetWritesForActor(St->FallingActor, W::ManualUdp, ManualNow);
		if (HardWrites <= 0) return false;
		if (AutoNow != St->AutoWritesAtDamage) return false;   // still being flown: the hand-over is not done

		// Everything is in hand. COPY IT OUT before anything else can move.
		St->bFallingEntered = true;
		St->FallingLifeState = static_cast<int32>(Health->LifeState);
		St->FallingResolutionsAtEvidence = AC.FallingResolutions;
		St->FormationResolutionsAtEvidence = AC.FormationResolutions;
		St->HardoverWritesAtEvidence = HardWrites;
		St->AutoWritesAtEvidence = AutoNow;
		St->ManualWritesAtEvidence = ManualNow;
		St->FinalThrottle = Snap.EngineCommands[0].Throttle;
		St->bFinalCutOff = Snap.EngineCommands[0].CutOff;
		St->EvidenceConsumeSequence = Snap.ConsumeSequence;
		St->ConsumesAtEvidence = AC.Consumes;
		St->bEvidenceComplete = true;
		return true;
	}

	// Stops ONLY the damaged aircraft's flight dynamics, and only after the evidence is complete. Nothing
	// about production Falling behaviour, the health component, JSBSim or the arbiter changes: this is the
	// automation declaring it has seen what it came to see.
	void StopFallingAircraftFdm(UWorld *World)
	{
		if (!St->bEvidenceComplete) return;   // belt and braces: never before the evidence
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (It->GetActorNameOrLabel() != St->FallingActor) continue;
			if (UJSBSimMovementComponent *C = It->FindComponentByClass<UJSBSimMovementComponent>())
			{
				C->SetComponentTickEnabled(false);
				St->bTickDisabled = true;
			}
			break;
		}
	}

	// One tick later: did it actually stop? Asked of the engine, not of our own intention -- the component
	// is re-found, its tick state is read back, and the counters are compared with the values captured
	// while it was still flying. bPostStopVerified is set ONLY if every one of those holds.
	void VerifyFallingAircraftStopped(UWorld *World)
	{
		namespace Arb = MumtCommandArbiterV2;

		UJSBSimMovementComponent *Comp = nullptr;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (It->GetActorNameOrLabel() != St->FallingActor) continue;
			Comp = It->FindComponentByClass<UJSBSimMovementComponent>();
			break;
		}
		St->bPostStopComponentAlive = (Comp != nullptr);
		if (!Comp) return;   // the aircraft vanished: the stop cannot be verified, and Finalize will say so

		St->bPostStopTickDisabled = !Comp->IsComponentTickEnabled();

		Arb::FAircraftCounters After{};
		if (!Arb::GetAircraftCounters(St->FallingActor, After)) return;
		St->PostStopFallingResolutions = After.FallingResolutions;
		St->PostStopConsumes = After.Consumes;

		St->bPostStopVerified =
			St->bPostStopComponentAlive
			&& St->bPostStopTickDisabled
			&& St->PostStopFallingResolutions == St->FallingResolutionsAtEvidence
			&& St->PostStopConsumes == St->ConsumesAtEvidence;
	}

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

			// ---- the evidence, asserted on the VALUES CAPTURED while the aircraft was still flying -----
			//
			// Order matters here as much as content: the test may only have stopped the aircraft AFTER it
			// had all of this. A run that stopped the FDM first and then went looking for evidence would be
			// asserting about a corpse.
			//
			// SCOPE: this scenario establishes OWNERSHIP of the damaged aircraft's command block, not the
			// arbiter's Falling PRIORITY. Priority over a live Formation candidate is established by
			// MUMT.ControlV2.ArbiterFallingOverCandidate.
			Test->TestTrue(TEXT("D: the evidence was COMPLETE"), St->bEvidenceComplete);
			Test->TestTrue(TEXT("D: the aircraft really entered Falling (life state, not just damage)"),
				St->bFallingEntered);
			Test->TestEqual(TEXT("D: ...and the life state was Falling"),
				St->FallingLifeState, static_cast<int32>(EAircraftLifeState::Falling));

			// A Falling-resolved block was produced at the FDM consume boundary.
			Test->TestTrue(TEXT("D: falling_resolution_count > 0 (a Falling-resolved block reached the "
			                    "FDM consume boundary)"),
				St->FallingResolutionsAtEvidence > 0);
			Test->TestEqual(TEXT("D: formation_resolution_count == 0"),
				St->FormationResolutionsAtEvidence, (int64)0);

			// The final resolved block passed into the FCS setter path.
			Test->TestEqual(TEXT("D: final_throttle == 0"), St->FinalThrottle, 0.0);
			Test->TestTrue(TEXT("D: final_cutoff == true"), St->bFinalCutOff);

			// OWNERSHIP of the damaged aircraft -- stated as what was actually measured, and no more.
			Test->TestTrue(TEXT("D: HealthHardover took ownership of the damaged aircraft"),
				St->HardoverWritesAtEvidence > 0);
			Test->TestEqual(TEXT("D: the autopilot stopped writing the damaged aircraft after the Falling "
			                     "transition"),
				St->AutoWritesAtEvidence, St->AutoWritesAtDamage);
			Test->TestEqual(TEXT("D: the manual writer did not participate on the damaged UAV"),
				St->ManualWritesAtEvidence, (int64)0);

			// THE STOP, verified a tick later rather than assumed in the same breath as the action.
			Test->TestTrue(TEXT("D: the FDM tick was disabled ONLY after the evidence was complete"),
				St->bEvidenceComplete && St->bTickDisabled);
			Test->TestTrue(TEXT("D: the stop was VERIFIED on a later tick (post_stop_verified)"),
				St->bPostStopVerified);
			Test->TestTrue(TEXT("D: ...the aircraft still existed when the stop was verified"),
				St->bPostStopComponentAlive);
			Test->TestTrue(TEXT("D: ...its FDM tick really is disabled (asked of the component)"),
				St->bPostStopTickDisabled);
			Test->TestEqual(TEXT("D: ...and no further Falling resolution occurred after the stop"),
				St->PostStopFallingResolutions, St->FallingResolutionsAtEvidence);
			Test->TestEqual(TEXT("D: ...nor any further resolved consume at all"),
				St->PostStopConsumes, St->ConsumesAtEvidence);

			// One stable line, identical run to run: the invariants this scenario exists to establish.
			UE_LOG(LogMumtCmdOwnTest, Display,
				TEXT("[CMDOWN] D_EVIDENCE evidence_complete=%d falling_entered=%d falling_resolved_at_boundary=%d "
				     "final_throttle=%.6f final_cutoff=%d formation_resolutions=%lld autopilot_stopped=%d "
				     "manual_writes=%lld hardover_owns=%d tick_disabled_after_evidence=%d post_stop_verified=%d"),
				St->bEvidenceComplete ? 1 : 0, St->bFallingEntered ? 1 : 0,
				St->FallingResolutionsAtEvidence > 0 ? 1 : 0,
				St->FinalThrottle, St->bFinalCutOff ? 1 : 0, St->FormationResolutionsAtEvidence,
				St->AutoWritesAtEvidence == St->AutoWritesAtDamage ? 1 : 0, St->ManualWritesAtEvidence,
				St->HardoverWritesAtEvidence > 0 ? 1 : 0,
				(St->bEvidenceComplete && St->bTickDisabled) ? 1 : 0,
				St->bPostStopVerified ? 1 : 0);
			UE_LOG(LogMumtCmdOwnTest, Display,
				TEXT("[CMDOWN] D_EVIDENCE_DETAIL actor=%s falling_resolutions=%lld hardover_writes=%lld "
				     "autopilot_writes_at_damage=%lld consume_sequence=%llu consumes_at_evidence=%lld "
				     "consumes_after_stop=%lld"),
				*St->FallingActor, St->FallingResolutionsAtEvidence, St->HardoverWritesAtEvidence,
				St->AutoWritesAtDamage, St->EvidenceConsumeSequence, St->ConsumesAtEvidence,
				St->PostStopConsumes);
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
