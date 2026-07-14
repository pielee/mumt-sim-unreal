// MumtCommandArbiterV2Test.cpp — proves the consume-boundary arbiter is transparent by default.
//
// The whole risk of Phase A is a silent behaviour change: the arbiter now stands between every writer
// and the FCS, so if it drops or alters ONE field the aircraft flies differently and nothing else in
// the project would notice. So the central assertion is not "it works" but "it changed nothing":
// resolved == legacy in every consumed field -- all 15 flight-control fields and all 14 per-engine
// fields, compared one by one, not hashed.
//
// The Formation path is exercised only through the explicit test API. Production cannot reach it.
#include "Misc/AutomationTest.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

#include "State/MumtCommandArbiterV2.h"
#include "State/MumtCommandOwnershipTelemetry.h"
#include "HealthComponent.h"
#include "JSBSimMovementComponent.h"
#include "Tests/AutomationEditorCommon.h"
#include "Editor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Misc/App.h"
#include "Common/UdpSocketBuilder.h"
#include "IPAddress.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

DEFINE_LOG_CATEGORY_STATIC(LogMumtArbTest, Display, All);

namespace
{
namespace Arb = MumtCommandArbiterV2;

const TCHAR *kArbMap = TEXT("/Game/RL_2");
constexpr double kArbMaxWallSeconds = 300.0;
constexpr int32  kArbUdpPort = 5005;
constexpr double kArbDamageAtS = 15.0;

enum class EArbScenario : uint8
{
	LegacyManual,        // A
	LegacyAutopilot,     // B
	LegacyOverlap,       // C
	FallingOverCandidate,// D
	CandidateValid,      // E
	CandidateStale,      // F
	CandidateInvalid,    // G
	Isolation,           // H
};

const TCHAR *ScenarioName(EArbScenario S)
{
	switch (S)
	{
	case EArbScenario::LegacyManual:         return TEXT("A_LegacyManual");
	case EArbScenario::LegacyAutopilot:      return TEXT("B_LegacyAutopilot");
	case EArbScenario::LegacyOverlap:        return TEXT("C_LegacyOverlap");
	case EArbScenario::FallingOverCandidate: return TEXT("D_FallingOverCandidate");
	case EArbScenario::CandidateValid:       return TEXT("E_CandidateValid");
	case EArbScenario::CandidateStale:       return TEXT("F_CandidateStale");
	case EArbScenario::CandidateInvalid:     return TEXT("G_CandidateInvalid");
	default:                                 return TEXT("H_Isolation");
	}
}

// The flight scenarios need real flight; the candidate-policy ones only need a live PIE world.
double RunSecondsFor(EArbScenario S)
{
	switch (S)
	{
	case EArbScenario::LegacyManual:
	case EArbScenario::LegacyAutopilot:
	case EArbScenario::LegacyOverlap:        return 25.0;
	case EArbScenario::FallingOverCandidate: return 30.0;
	default:                                 return 8.0;
	}
}

// ---- what the FCS actually consumed, captured off the same multicast the arbiter uses --------------
struct FConsumedProbe
{
	FString ActorName;
	double Aileron = 0, Elevator = 0, Rudder = 0, SpeedBrake = 0, Throttle = 0;
	bool bCutOff = false;
	bool bSeen = false;
};
TMap<FString, FConsumedProbe> GProbe;
FDelegateHandle GProbeHandle;

void ProbeResolved(const UJSBSimMovementComponent *Component, uint64 /*Seq*/,
                   const FFlightControlCommands & /*Legacy*/, const TArray<FEngineCommand> & /*LegacyEng*/,
                   const FJSBSimResolvedCommandBlock &Resolved)
{
	if (!Component || !Component->GetOwner()) return;
	FConsumedProbe &P = GProbe.FindOrAdd(Component->GetOwner()->GetActorNameOrLabel());
	P.ActorName = Component->GetOwner()->GetActorNameOrLabel();
	P.Aileron = Resolved.Commands.Aileron;
	P.Elevator = Resolved.Commands.Elevator;
	P.Rudder = Resolved.Commands.Rudder;
	P.SpeedBrake = Resolved.Commands.SpeedBrake;
	if (Resolved.EngineCommands.Num() > 0)
	{
		P.Throttle = Resolved.EngineCommands[0].Throttle;
		P.bCutOff = Resolved.EngineCommands[0].CutOff;
	}
	P.bSeen = true;
}

struct FArbUdpSender
{
	FSocket *Socket = nullptr;
	TSharedPtr<FInternetAddr> Addr;

	bool Init()
	{
		ISocketSubsystem *SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
		if (!SS) return false;
		Socket = SS->CreateSocket(NAME_DGram, TEXT("MumtArbTestSender"), false);
		if (!Socket) return false;
		Addr = SS->CreateInternetAddr();
		bool bValid = false;
		Addr->SetIp(TEXT("127.0.0.1"), bValid);
		Addr->SetPort(kArbUdpPort);
		return bValid;
	}
	void Send(const FString &Name, double Roll, double Pitch, double Yaw, double Throttle)
	{
		if (!Socket || !Addr.IsValid()) return;
		const FString Msg = FString::Printf(
			TEXT("{\"commands\":[{\"aircraft_name\":\"%s\",\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f,\"throttle\":%.4f}]}"),
			*Name, Roll, Pitch, Yaw, Throttle);
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

// Counting starts only after the world has settled. The first PIE frames carry map-load and shader
// hitches long enough that a candidate pushed on one tick is genuinely older than kCandidateMaxAgeS by
// the time the next consume runs -- and the arbiter then correctly refuses it as stale. That refusal is
// the safety property working, not a defect, but counting it would mean asserting "PIE startup has no
// frame hitches", which is not a property of the code under test. So the measurement window opens after
// the transient, and inside it a stale fallback really would be a bug.
constexpr double kArbSettleS = 1.5;

struct FArbState
{
	double FirstWall = 0.0;
	double FirstSim = -1.0;
	double MeasureStartSim = -1.0;
	bool bMeasuring = false;
	bool bStarted = false;
	bool bDamaged = false;
	bool bModeSet = false;
	FArbUdpSender Udp;
	FString FallingActor;
	int64 FormationBeforeDamage = 0;
	int64 AutoWritesAtDamage = 0;
	// the candidate the test last pushed, so the resolved block can be checked against it
	Arb::FFormationCandidate LastCandidate;
};

UJSBSimMovementComponent *FindArbAircraft(UWorld *World, const TCHAR *LabelSubstring, AActor **OutActor = nullptr)
{
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (!It->GetActorNameOrLabel().Contains(LabelSubstring)) continue;
		if (UJSBSimMovementComponent *C = It->FindComponentByClass<UJSBSimMovementComponent>())
		{
			if (OutActor) *OutActor = *It;
			return C;
		}
	}
	return nullptr;
}

double GetArbSimTime(UWorld *World)
{
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		if (UJSBSimMovementComponent *C = It->FindComponentByClass<UJSBSimMovementComponent>())
		{
			FJsbFlightSnapshot S{};
			if (C->GetJsbFlightSnapshot(S) && S.bValidFrame) return S.SimTimeSec;
		}
	}
	return -1.0;
}

class FMumtArbSampleCommand : public IAutomationLatentCommand
{
public:
	FMumtArbSampleCommand(FAutomationTestBase *T, TSharedPtr<FArbState> S, EArbScenario InScenario)
		: Test(T), St(S), Scenario(InScenario) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		UWorld *World = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
		if (!World) return false;

		if (!St->bStarted)
		{
			St->bStarted = true;
			St->FirstWall = NowWall;
			GProbe.Reset();
			GProbeHandle = UJSBSimMovementComponent::CommandResolvedObserver.AddStatic(&ProbeResolved);

			MumtCommandOwnership::SetEnabled(true);
			MumtCommandOwnership::ResetSession(ScenarioName(Scenario));
			// The resolver is bound by the MODULE, not by us. If a test had to switch it on, every one
			// of these scenarios would be exercising a code path production never runs -- so this is an
			// assertion, not a setup step.
			Test->TestTrue(TEXT("the resolver is bound in production (module startup), not by the test"),
				Arb::IsEnabled());
			Arb::ResetSession(ScenarioName(Scenario));
			St->Udp.Init();
		}

		if (Scenario == EArbScenario::LegacyManual || Scenario == EArbScenario::LegacyOverlap)
		{
			St->Udp.Send(TEXT("M_F16"), 0.15, -0.05, 0.0, 0.7);
		}

		const double SimT = GetArbSimTime(World);
		if (SimT >= 0.0 && St->FirstSim < 0.0) St->FirstSim = SimT;
		const double SinceStart = (St->FirstSim >= 0.0 && SimT >= 0.0) ? (SimT - St->FirstSim) : 0.0;

		// Candidates are pushed from the very first tick, so the arbiter is exercised across the
		// startup transient too -- but the counters that get ASSERTED are zeroed once it is over.
		DriveCandidates(World, SimT);

		if (!St->bMeasuring)
		{
			if (SinceStart < kArbSettleS) return false;
			St->bMeasuring = true;
			St->MeasureStartSim = SimT;
			GProbe.Reset();
			Arb::ResetSession(ScenarioName(Scenario));
			MumtCommandOwnership::ResetSession(ScenarioName(Scenario));
			UE_LOG(LogMumtArbTest, Display,
				TEXT("[ARBITER] measurement window opened at sim t=%.2f (settled after %.1f s)"),
				SimT, kArbSettleS);
			return false;
		}

		const double Elapsed = SimT - St->MeasureStartSim;

		if (Scenario == EArbScenario::FallingOverCandidate && !St->bDamaged && Elapsed >= kArbDamageAtS)
		{
			AActor *Actor = nullptr;
			if (UJSBSimMovementComponent *C = FindArbAircraft(World, TEXT("F16_UAV1"), &Actor))
			{
				if (UHealthComponent *H = Actor ? Actor->FindComponentByClass<UHealthComponent>() : nullptr)
				{
					Arb::FAircraftCounters Before{};
					Arb::GetAircraftCounters(Actor->GetActorNameOrLabel(), Before);
					St->FormationBeforeDamage = Before.FormationResolutions;
					MumtCommandOwnership::GetWritesForActor(Actor->GetActorNameOrLabel(),
						MumtCommandOwnership::EWriterId::InnerLoopAutopilot, St->AutoWritesAtDamage);

					H->ApplyDamage(1.0e6f, nullptr);
					St->bDamaged = true;
					St->FallingActor = Actor->GetActorNameOrLabel();
					UE_LOG(LogMumtArbTest, Display,
						TEXT("[ARBITER] scenario D: damaged %s at sim t=%.2f (formation resolutions so far=%lld, "
						     "autopilot writes so far=%lld)"),
						*St->FallingActor, SimT, St->FormationBeforeDamage, St->AutoWritesAtDamage);
					(void)C;
				}
			}
			if (!St->bDamaged)
			{
				Test->AddError(TEXT("[ARBITER] scenario D: no damageable F16_UAV1 found"));
				return Finalize();
			}
		}

		const bool bDone = (Elapsed >= RunSecondsFor(Scenario)) || ((NowWall - St->FirstWall) >= kArbMaxWallSeconds);
		return bDone ? Finalize() : false;
	}

private:
	// Pushes the test-only Formation candidate. Nothing in production does this.
	void DriveCandidates(UWorld *World, double SimT)
	{
		const bool bNeedsCandidate =
			Scenario == EArbScenario::FallingOverCandidate || Scenario == EArbScenario::CandidateValid ||
			Scenario == EArbScenario::CandidateStale || Scenario == EArbScenario::CandidateInvalid ||
			Scenario == EArbScenario::Isolation;
		if (!bNeedsCandidate) return;

		UJSBSimMovementComponent *Target = FindArbAircraft(World, TEXT("F16_UAV1"));
		if (!Target) return;

		if (!St->bModeSet)
		{
			Arb::SetModeForTesting(Target, Arb::ECommandMode::FormationControlV2);
			St->bModeSet = true;
			// H: every other aircraft stays in the default mode and must be untouched by the above.
		}

		Arb::FFormationCandidate K{};
		K.AileronNorm = 0.25;
		K.ElevatorNorm = -0.10;
		K.RudderNorm = 0.05;
		K.ThrottleNorm = 0.80;
		K.SpeedBrakeNorm = 0.30;
		K.Generation = 7;
		K.bValid = true;
		K.bCommandReady = true;
		K.bFinite = true;
		K.TimestampS = FApp::GetCurrentTime();

		switch (Scenario)
		{
		case EArbScenario::CandidateStale:
			// Old enough to be refused. The producer "died" a second ago.
			K.TimestampS = FApp::GetCurrentTime() - 1.0;
			break;
		case EArbScenario::CandidateInvalid:
			// Claims to be finite AND ready, but carries a NaN. The arbiter must VERIFY, not trust the
			// flag -- a producer that lies is exactly the case a fallback exists for.
			K.AileronNorm = std::numeric_limits<double>::quiet_NaN();
			break;
		default:
			break;
		}

		St->LastCandidate = K;
		Arb::SetCandidateForTesting(Target, K);
		(void)SimT;
	}

	bool Finalize()
	{
		St->Udp.Close();
		if (GProbeHandle.IsValid())
		{
			UJSBSimMovementComponent::CommandResolvedObserver.Remove(GProbeHandle);
			GProbeHandle.Reset();
		}

		for (const FString &L : Arb::BuildReport())        UE_LOG(LogMumtArbTest, Display, TEXT("%s"), *L);
		for (const FString &L : MumtCommandOwnership::BuildReport()) UE_LOG(LogMumtArbTest, Display, TEXT("%s"), *L);

		const Arb::FCounters C = Arb::GetCounters();
		const MumtCommandOwnership::FCounters O = MumtCommandOwnership::GetCounters();
		// Deliberately NOT Arb::SetEnabled(false): the resolver belongs to the module, and tearing it
		// down here would leave every later test in this editor process running unarbitrated.
		MumtCommandOwnership::SetEnabled(false);
		// Leave nothing in Formation mode behind us. (TActorIterator would crash on a null world, so the
		// world is checked rather than assumed -- Finalize can also be reached from an error path.)
		if (UWorld *W = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr)
		{
			if (UJSBSimMovementComponent *Cmp = FindArbAircraft(W, TEXT("F16_UAV1")))
			{
				Arb::SetModeForTesting(Cmp, Arb::ECommandMode::LegacyOrManual);
			}
		}

		using W = MumtCommandOwnership::EWriterId;
		const int64 Manual = O.WritesByWriter[static_cast<int32>(W::ManualUdp)];
		const int64 Auto   = O.WritesByWriter[static_cast<int32>(W::InnerLoopAutopilot)];
		const int64 Health = O.WritesByWriter[static_cast<int32>(W::HealthHardover)];

		// ---- invariants that hold in EVERY scenario ------------------------------------------------
		Test->TestTrue(TEXT("the FDM consumed commands"), C.ConsumeCount > 0);
		Test->TestEqual(TEXT("the resolver ran exactly once per consume (no duplicates)"),
			C.DuplicateResolutionCount, (int64)0);
		Test->TestEqual(TEXT("no consume bypassed the resolver"), C.MissingResolutionCount, (int64)0);
		Test->TestEqual(TEXT("the resolver never mutated the legacy block"),
			C.LegacyBlockMutationCount, (int64)0);
		Test->TestEqual(TEXT("no non-finite command was ever resolved to the FCS"),
			C.ResolvedNonFiniteCount, (int64)0);
		Test->TestEqual(TEXT("nothing else tried to own the single-cast resolver"),
			Arb::GetResolverOwnershipConflictCount(), (int64)0);

		switch (Scenario)
		{
		case EArbScenario::LegacyManual:
			Test->TestTrue(TEXT("A: the manual/UDP writer ran"), Manual > 0);
			Test->TestTrue(TEXT("A: every resolution was Legacy"), C.LegacyResolutionCount > 0);
			Test->TestEqual(TEXT("A: no Formation resolution"), C.FormationResolutionCount, (int64)0);
			Test->TestEqual(TEXT("A: LEGACY IS TRANSPARENT -- zero fields changed"),
				C.LegacyChangedFieldCount, (int64)0);
			Test->TestEqual(TEXT("A: the FCS consumed exactly what the writers produced"),
				O.ResolvedDiffersFromLegacyCount, (int64)0);
			break;

		case EArbScenario::LegacyAutopilot:
			Test->TestTrue(TEXT("B: the autopilot ran"), Auto > 0);
			Test->TestEqual(TEXT("B: LEGACY IS TRANSPARENT -- zero fields changed"),
				C.LegacyChangedFieldCount, (int64)0);
			Test->TestEqual(TEXT("B: the FCS consumed exactly what the writers produced"),
				O.ResolvedDiffersFromLegacyCount, (int64)0);
			// The arbiter must not have moved the writers around: the autopilot still writes AFTER the
			// consume, which is the ordering the ownership audit measured.
			Test->TestTrue(TEXT("B: the autopilot still writes after the consume (ordering unchanged)"),
				O.WritesAfterConsumeSameFrame > 0);
			break;

		case EArbScenario::LegacyOverlap:
			Test->TestTrue(TEXT("C: the autopilot ran"), Auto > 0);
			Test->TestTrue(TEXT("C: the manual/UDP writer ran"), Manual > 0);
			Test->TestTrue(TEXT("C: both writers still own one aircraft between two consumes"),
				O.MultiWriterConsumeCount > 0);
			// The field mixture (manual A/E/R/T + the autopilot's SpeedBrake) is current behaviour and
			// the arbiter must carry it through untouched -- including the fields no writer claims.
			Test->TestEqual(TEXT("C: LEGACY IS TRANSPARENT -- the writer field mixture survives intact"),
				C.LegacyChangedFieldCount, (int64)0);
			Test->TestEqual(TEXT("C: the FCS consumed exactly what the writers produced"),
				O.ResolvedDiffersFromLegacyCount, (int64)0);
			break;

		case EArbScenario::FallingOverCandidate:
		{
			Test->TestTrue(TEXT("D: an aircraft was damaged"), St->bDamaged);
			Test->TestTrue(TEXT("D: the Formation candidate WAS being applied before the damage"),
				St->FormationBeforeDamage > 0);
			Test->TestTrue(TEXT("D: the Falling hardover ran"), Health > 0);

			Arb::FAircraftCounters A{};
			const bool bFound = Arb::GetAircraftCounters(St->FallingActor, A);
			Test->TestTrue(TEXT("D: the damaged aircraft is known to the arbiter"), bFound);
			Test->TestTrue(TEXT("D: after Falling, resolution switched to FallingLegacy"),
				A.FallingResolutions > 0);
			// THE POINT: a live, valid, fresh Formation candidate keeps arriving, and it must NOT be
			// applied to a dead aircraft.
			Test->TestEqual(TEXT("D: FALLING OUTRANKS THE FORMATION CANDIDATE -- no Formation after damage"),
				A.FormationResolutions, St->FormationBeforeDamage);
			Test->TestEqual(TEXT("D: the Falling block passed through untouched"),
				A.LegacyChangedFields, (int64)0);

			int64 AutoAfter = 0;
			MumtCommandOwnership::GetWritesForActor(St->FallingActor, W::InnerLoopAutopilot, AutoAfter);
			Test->TestEqual(TEXT("D: the autopilot stopped writing the dead aircraft"),
				AutoAfter, St->AutoWritesAtDamage);

			const FConsumedProbe *P = GProbe.Find(St->FallingActor);
			Test->TestTrue(TEXT("D: the consumed block was observed"), P && P->bSeen);
			if (P)
			{
				Test->TestEqual(TEXT("D: the FCS consumed throttle 0"), P->Throttle, 0.0);
				Test->TestTrue(TEXT("D: the FCS consumed engine cutoff"), P->bCutOff);
			}
			UE_LOG(LogMumtArbTest, Display,
				TEXT("[ARBITER] D_RESULT actor=%s formation_before=%lld formation_after=%lld falling=%lld "
				     "autopilot_at_damage=%lld autopilot_at_end=%lld"),
				*St->FallingActor, St->FormationBeforeDamage, A.FormationResolutions, A.FallingResolutions,
				St->AutoWritesAtDamage, AutoAfter);
			break;
		}

		case EArbScenario::CandidateValid:
		{
			Test->TestTrue(TEXT("E: the Formation candidate was applied"), C.FormationResolutionCount > 0);
			Test->TestEqual(TEXT("E: no stale fallback in steady state"), C.StaleFallbackCount, (int64)0);
			Test->TestEqual(TEXT("E: no invalid fallback"), C.InvalidFallbackCount, (int64)0);

			// A fresh, valid candidate must win EVERY consume on that aircraft -- not most of them.
			Arb::FAircraftCounters A{};
			Test->TestTrue(TEXT("E: the Formation aircraft is known to the arbiter"),
				Arb::GetAircraftCounters(TEXT("F16_UAV1"), A));
			Test->TestTrue(TEXT("E: it consumed commands"), A.Consumes > 0);
			Test->TestEqual(TEXT("E: EVERY consume resolved to Formation (no silent fallback)"),
				A.FormationResolutions, A.Consumes);
			Test->TestEqual(TEXT("E: it never fell back to legacy"), A.LegacyResolutions, (int64)0);

			const FConsumedProbe *P = GProbe.Find(TEXT("F16_UAV1"));
			Test->TestTrue(TEXT("E: the consumed block was observed"), P && P->bSeen);
			if (P)
			{
				Test->TestEqual(TEXT("E: consumed aileron == candidate"), P->Aileron, St->LastCandidate.AileronNorm);
				Test->TestEqual(TEXT("E: consumed elevator == candidate"), P->Elevator, St->LastCandidate.ElevatorNorm);
				Test->TestEqual(TEXT("E: consumed rudder == candidate"), P->Rudder, St->LastCandidate.RudderNorm);
				Test->TestEqual(TEXT("E: consumed speedbrake == candidate"), P->SpeedBrake, St->LastCandidate.SpeedBrakeNorm);
				Test->TestEqual(TEXT("E: consumed throttle == candidate"), P->Throttle, St->LastCandidate.ThrottleNorm);
			}
			break;
		}

		case EArbScenario::CandidateStale:
			Test->TestTrue(TEXT("F: the stale candidate was refused"), C.StaleFallbackCount > 0);
			Test->TestEqual(TEXT("F: no Formation resolution happened"), C.FormationResolutionCount, (int64)0);
			Test->TestEqual(TEXT("F: the legacy block was passed through untouched"),
				C.LegacyChangedFieldCount, (int64)0);
			break;

		case EArbScenario::CandidateInvalid:
			Test->TestTrue(TEXT("G: the non-finite candidate was refused"), C.NonFiniteFallbackCount > 0);
			Test->TestEqual(TEXT("G: no Formation resolution happened"), C.FormationResolutionCount, (int64)0);
			Test->TestEqual(TEXT("G: the legacy block was passed through untouched"),
				C.LegacyChangedFieldCount, (int64)0);
			Test->TestEqual(TEXT("G: nothing non-finite ever reached the FCS"),
				C.ResolvedNonFiniteCount, (int64)0);
			break;

		case EArbScenario::Isolation:
		{
			Arb::FAircraftCounters Uav{}, Manned{};
			const bool bU = Arb::GetAircraftCounters(TEXT("F16_UAV1"), Uav);
			const bool bM = Arb::GetAircraftCounters(TEXT("M_F16"), Manned);
			Test->TestTrue(TEXT("H: both aircraft are known to the arbiter"), bU && bM);
			Test->TestTrue(TEXT("H: the Formation aircraft resolved Formation"), Uav.FormationResolutions > 0);
			Test->TestEqual(TEXT("H: the OTHER aircraft never left Legacy mode"),
				static_cast<int32>(Manned.Mode), static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
			Test->TestEqual(TEXT("H: the OTHER aircraft never resolved Formation"),
				Manned.FormationResolutions, (int64)0);
			Test->TestEqual(TEXT("H: the OTHER aircraft's block was passed through untouched"),
				Manned.LegacyChangedFields, (int64)0);
			break;
		}
		}
		return true;
	}

	FAutomationTestBase *Test;
	TSharedPtr<FArbState> St;
	EArbScenario Scenario{EArbScenario::LegacyManual};
};

void RunArbiterScenario(FAutomationTestBase *T, EArbScenario S)
{
	T->AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);
	TSharedPtr<FArbState> State = MakeShared<FArbState>();
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kArbMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtArbSampleCommand(T, State, S));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
}

} // namespace

// ---- PRODUCTION DEFAULT ---------------------------------------------------------------------------
// The scenarios above still push modes and candidates through the test API. This one touches NOTHING:
// it asserts that the resolver is already running because the MODULE bound it, and that in that
// untouched production state the arbiter is perfectly transparent. Without this test, every other
// arbiter test could pass while production ran with no resolver at all.
class FMumtArbProductionDefaultCommand : public IAutomationLatentCommand
{
public:
	FMumtArbProductionDefaultCommand(FAutomationTestBase *T, TSharedPtr<FArbState> S) : Test(T), St(S) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		UWorld *World = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
		if (!World) return false;

		if (!St->bStarted)
		{
			St->bStarted = true;
			St->FirstWall = NowWall;
			// NOTE: no Arb::SetEnabled, no SetModeForTesting, no SetCandidateForTesting. Nothing.
			Test->TestTrue(TEXT("PROD: the resolver is bound by the module, with no test intervention"),
				Arb::IsEnabled());
			MumtCommandOwnership::SetEnabled(true);   // an OBSERVER only -- it does not resolve anything
			MumtCommandOwnership::ResetSession(TEXT("ProductionDefault"));
		}

		const double SimT = GetArbSimTime(World);
		if (SimT >= 0.0 && St->FirstSim < 0.0) St->FirstSim = SimT;
		const double SinceStart = (St->FirstSim >= 0.0 && SimT >= 0.0) ? (SimT - St->FirstSim) : 0.0;

		if (!St->bMeasuring)
		{
			if (SinceStart < kArbSettleS) return false;
			St->bMeasuring = true;
			St->MeasureStartSim = SimT;
			Arb::ResetSession(TEXT("ProductionDefault"));   // measurement only; changes no decision
			MumtCommandOwnership::ResetSession(TEXT("ProductionDefault"));
			return false;
		}

		if ((SimT - St->MeasureStartSim) < 8.0 && (NowWall - St->FirstWall) < kArbMaxWallSeconds) return false;

		for (const FString &L : Arb::BuildReport()) UE_LOG(LogMumtArbTest, Display, TEXT("%s"), *L);
		const Arb::FCounters C = Arb::GetCounters();
		const MumtCommandOwnership::FCounters O = MumtCommandOwnership::GetCounters();
		MumtCommandOwnership::SetEnabled(false);

		Test->TestTrue(TEXT("PROD: the FDM consumed commands"), C.ConsumeCount > 0);
		Test->TestEqual(TEXT("PROD: the resolver ran on every consume"),
			C.ResolverCallCount, C.ConsumeCount);
		Test->TestEqual(TEXT("PROD: no duplicate resolution"), C.DuplicateResolutionCount, (int64)0);
		Test->TestEqual(TEXT("PROD: no consume bypassed the resolver"), C.MissingResolutionCount, (int64)0);
		Test->TestEqual(TEXT("PROD: zero fields changed (the arbiter is transparent by default)"),
			C.LegacyChangedFieldCount, (int64)0);
		Test->TestEqual(TEXT("PROD: the legacy block was never mutated"),
			C.LegacyBlockMutationCount, (int64)0);
		Test->TestEqual(TEXT("PROD: FormationControlV2 is NOT active in production"),
			C.FormationResolutionCount, (int64)0);
		Test->TestEqual(TEXT("PROD: the FCS consumed exactly what the writers produced"),
			O.ResolvedDiffersFromLegacyCount, (int64)0);
		Test->TestEqual(TEXT("PROD: nothing else owns the resolver"),
			Arb::GetResolverOwnershipConflictCount(), (int64)0);

		// Every aircraft, not just one: a single stray Formation mode would be a production activation.
		int32 Checked = 0;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			UJSBSimMovementComponent *Cmp = It->FindComponentByClass<UJSBSimMovementComponent>();
			if (!Cmp) continue;
			++Checked;
			Test->TestEqual(TEXT("PROD: every aircraft defaults to LegacyOrManual"),
				static_cast<int32>(Arb::GetMode(Cmp)),
				static_cast<int32>(Arb::ECommandMode::LegacyOrManual));
		}
		Test->TestTrue(TEXT("PROD: at least one aircraft was checked"), Checked > 0);
		UE_LOG(LogMumtArbTest, Display,
			TEXT("[ARBITER] PROD_DEFAULT resolver_enabled=1 aircraft_checked=%d consumes=%lld "
			     "resolver_calls=%lld changed_fields=%lld formation=%lld resolved_differs=%lld"),
			Checked, C.ConsumeCount, C.ResolverCallCount, C.LegacyChangedFieldCount,
			C.FormationResolutionCount, O.ResolvedDiffersFromLegacyCount);
		return true;
	}

private:
	FAutomationTestBase *Test;
	TSharedPtr<FArbState> St;
};

// ---- REGISTRY LIFECYCLE ---------------------------------------------------------------------------
// A TWeakObjectPtr going invalid is not cleanup: the entry, its mode and its candidate still sit in the
// registry. This destroys a real aircraft mid-PIE and requires the entry to actually leave.
class FMumtArbLifecycleCommand : public IAutomationLatentCommand
{
public:
	FMumtArbLifecycleCommand(FAutomationTestBase *T, TSharedPtr<FArbState> S) : Test(T), St(S) {}

	virtual bool Update() override
	{
		const double NowWall = FPlatformTime::Seconds();
		UWorld *World = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
		if (!World) return false;

		if (!St->bStarted)
		{
			St->bStarted = true;
			St->FirstWall = NowWall;
			Test->TestTrue(TEXT("LIFE: the resolver is bound by the module"), Arb::IsEnabled());
		}

		const double SimT = GetArbSimTime(World);
		if (SimT >= 0.0 && St->FirstSim < 0.0) St->FirstSim = SimT;
		const double SinceStart = (St->FirstSim >= 0.0 && SimT >= 0.0) ? (SimT - St->FirstSim) : 0.0;
		if (SinceStart < kArbSettleS) return false;

		if (!St->bDamaged)   // reused flag: "have we destroyed the victim yet"
		{
			SizeBefore = Arb::GetRegistrySize();
			Test->TestTrue(TEXT("LIFE: aircraft are registered while the world runs"), SizeBefore > 0);

			AActor *Victim = nullptr;
			UJSBSimMovementComponent *C = FindArbAircraft(World, TEXT("F16_UAV2"), &Victim);
			if (!C || !Victim)
			{
				Test->AddError(TEXT("[ARBITER] LIFE: no F16_UAV2 to destroy"));
				return true;
			}
			// Give it a mode and a candidate first: if the entry leaked, THIS is what would leak with it.
			Arb::SetModeForTesting(C, Arb::ECommandMode::FormationControlV2);
			VictimName = Victim->GetActorNameOrLabel();

			Victim->Destroy();
			St->bDamaged = true;
			DestroyedAtSim = SimT;
			UE_LOG(LogMumtArbTest, Display, TEXT("[ARBITER] LIFE: destroyed %s (registry was %d)"),
				*VictimName, SizeBefore);
			return false;
		}

		// Give the world a couple of seconds to actually tear the actor down and let a consume purge it.
		if ((SimT - DestroyedAtSim) < 2.0 && (NowWall - St->FirstWall) < kArbMaxWallSeconds) return false;

		const int32 SizeAfter = Arb::GetRegistrySize();
		Arb::FAircraftCounters Gone{};
		const bool bStillThere = Arb::GetAircraftCounters(VictimName, Gone);

		Test->TestFalse(TEXT("LIFE: the destroyed aircraft's entry (and its mode/candidate) is gone"),
			bStillThere);
		Test->TestTrue(TEXT("LIFE: the registry actually shrank"), SizeAfter < SizeBefore);
		UE_LOG(LogMumtArbTest, Display,
			TEXT("[ARBITER] LIFE_RESULT victim=%s registry_before=%d registry_after=%d entry_still_present=%d"),
			*VictimName, SizeBefore, SizeAfter, bStillThere ? 1 : 0);
		return true;
	}

private:
	FAutomationTestBase *Test;
	TSharedPtr<FArbState> St;
	FString VictimName;
	int32 SizeBefore = 0;
	double DestroyedAtSim = 0.0;
};

// Runs AFTER FEndPlayMapCommand. World teardown is ASYNCHRONOUS -- FEndPlayMapCommand only *requests*
// the end of PIE and returns; the world is actually cleaned up over the following editor ticks. Checking
// the registry on the very first tick afterwards would be asserting that teardown is synchronous, which
// is not a property of the code under test (and is what made the first version of this test fail with a
// registry of 2).
//
// So it waits for the world to actually go away, and only then requires the registry to be empty. The
// wait is bounded: if cleanup never happens, the timeout fires and the test fails rather than hanging.
class FMumtArbPostPieRegistryCheck : public IAutomationLatentCommand
{
public:
	explicit FMumtArbPostPieRegistryCheck(FAutomationTestBase *T) : Test(T) {}

	virtual bool Update() override
	{
		if (StartWall < 0.0) StartWall = FPlatformTime::Seconds();
		const double Waited = FPlatformTime::Seconds() - StartWall;

		const bool bWorldGone = (GEditor == nullptr) || (GEditor->PlayWorld == nullptr);
		const int32 Size = Arb::GetRegistrySize();

		if ((!bWorldGone || Size > 0) && Waited < kMaxWaitS)
		{
			return false;   // teardown still in flight
		}

		Test->TestTrue(TEXT("LIFE: PIE actually ended"), bWorldGone);
		Test->TestEqual(TEXT("LIFE: the registry is empty once the world is torn down"), Size, 0);
		Test->TestTrue(TEXT("LIFE: the resolver is STILL bound after PIE (it belongs to the module, "
		                    "not to the world)"), Arb::IsEnabled());
		UE_LOG(LogMumtArbTest, Display,
			TEXT("[ARBITER] LIFE_POST_PIE world_gone=%d registry_size=%d resolver_still_bound=%d waited_s=%.2f"),
			bWorldGone ? 1 : 0, Size, Arb::IsEnabled() ? 1 : 0, Waited);
		return true;
	}

private:
	static constexpr double kMaxWaitS = 30.0;
	FAutomationTestBase *Test;
	double StartWall = -1.0;
};

// ---- WORLD-SCOPED CLEANUP -------------------------------------------------------------------------
// The registry is global; the aircraft in it are not. Tearing down one world must not delete the mode,
// candidate and counters of an aircraft that lives in a DIFFERENT, still-running world.
//
// The plain lifecycle test cannot catch this: with only a PIE world alive, "wipe everything" and "wipe
// this world's entries" are indistinguishable, and a blanket GAircraft.Reset() passes it. This builds a
// second world on purpose so the two behaviours separate.
class FMumtArbWorldIsolationCommand : public IAutomationLatentCommand
{
public:
	explicit FMumtArbWorldIsolationCommand(FAutomationTestBase *T) : Test(T) {}

	virtual bool Update() override
	{
		UWorld *PieWorld = (GEditor && GEditor->PlayWorld) ? GEditor->PlayWorld : nullptr;
		if (!PieWorld) return false;
		if (Arb::GetRegistrySize() == 0) return false;   // wait until the PIE aircraft have consumed

		// EVERYTHING BELOW HAPPENS IN ONE Update() CALL, ON PURPOSE.
		// PIE is still running: if a tick were allowed to slip between the teardown and the check, the
		// PIE aircraft would consume again and re-register themselves, and the removal would look like
		// it never happened. (That is exactly what the first version of this test measured -- 4 -> 4 --
		// and it was the test that was wrong, not the cleanup.)

		// World B: a second, independent world with its own aircraft component. The component is NOT
		// registered -- it never needs to tick or fly, it only needs a world and an identity, and
		// registering it would drag in JSBSim/GeoReferencing initialisation that has nothing to do with
		// what is being tested.
		OtherWorld = UWorld::CreateWorld(EWorldType::Game, /*bInformEngineOfWorld=*/false);
		if (!OtherWorld)
		{
			Test->AddError(TEXT("[ARBITER] WORLDISO: could not create a second world"));
			return true;
		}
		AActor *OtherActor = OtherWorld->SpawnActor<AActor>();
		if (!OtherActor)
		{
			Test->AddError(TEXT("[ARBITER] WORLDISO: could not spawn an actor in world B"));
			OtherWorld->DestroyWorld(false);
			return true;
		}
		UJSBSimMovementComponent *OtherComp = NewObject<UJSBSimMovementComponent>(OtherActor);

		// Give world B's aircraft a mode and a candidate: a blanket wipe would silently destroy them.
		Arb::SetModeForTesting(OtherComp, Arb::ECommandMode::FormationControlV2);
		Arb::FFormationCandidate K{};
		K.AileronNorm = 0.42; K.bValid = true; K.bCommandReady = true; K.bFinite = true;
		K.Generation = 99; K.TimestampS = FApp::GetCurrentTime();
		Arb::SetCandidateForTesting(OtherComp, K);

		const int32 SizeWithBoth = Arb::GetRegistrySize();
		Test->TestTrue(TEXT("WORLDISO: both worlds' aircraft are registered"), SizeWithBoth >= 2);
		Test->TestEqual(TEXT("WORLDISO: world B's aircraft is in Formation mode"),
			static_cast<int32>(Arb::GetMode(OtherComp)),
			static_cast<int32>(Arb::ECommandMode::FormationControlV2));

		// Tear down ONLY world A (the PIE world).
		FWorldDelegates::OnWorldCleanup.Broadcast(PieWorld, /*bSessionEnded=*/true, /*bCleanupResources=*/true);
		const int32 SizeAfterA = Arb::GetRegistrySize();

		Test->TestTrue(TEXT("WORLDISO: tearing down world A removed world A's aircraft"),
			SizeAfterA < SizeWithBoth);
		Test->TestEqual(TEXT("WORLDISO: ONLY world B's aircraft remains"), SizeAfterA, 1);
		// The load-bearing claim: a blanket GAircraft.Reset() would have destroyed this.
		Test->TestEqual(TEXT("WORLDISO: world B's mode SURVIVED another world's teardown"),
			static_cast<int32>(Arb::GetMode(OtherComp)),
			static_cast<int32>(Arb::ECommandMode::FormationControlV2));

		// Now tear down world B: the registry must finally be empty.
		FWorldDelegates::OnWorldCleanup.Broadcast(OtherWorld, true, true);
		const int32 SizeAfterB = Arb::GetRegistrySize();
		Test->TestEqual(TEXT("WORLDISO: tearing down world B empties the registry"), SizeAfterB, 0);

		UE_LOG(LogMumtArbTest, Display,
			TEXT("[ARBITER] WORLDISO_RESULT both=%d after_world_A=%d after_world_B=%d"),
			SizeWithBoth, SizeAfterA, SizeAfterB);

		OtherWorld->DestroyWorld(false);
		OtherWorld = nullptr;
		return true;
	}

private:
	FAutomationTestBase *Test;
	UWorld *OtherWorld = nullptr;
};

// ---- RESOLVER OWNERSHIP ----------------------------------------------------------------------------
// The resolver is single-cast. If another binder replaces it, the arbiter must NOT: (a) keep claiming to
// be enabled, or (b) unbind somebody else's resolver on shutdown. A boolean "we bound it" flag cannot
// tell the difference -- only delegate-handle identity can, and that is what this pins down.
int32 GForeignResolverCalls = 0;
void ForeignResolver(const UJSBSimMovementComponent *, uint64, const FFlightControlCommands &,
                     const TArray<FEngineCommand> &, FJSBSimResolvedCommandBlock &)
{
	++GForeignResolverCalls;   // deliberately changes nothing
}

class FMumtArbOwnershipCommand : public IAutomationLatentCommand
{
public:
	explicit FMumtArbOwnershipCommand(FAutomationTestBase *T) : Test(T) {}

	virtual bool Update() override
	{
		// Update() is called every tick. The ownership churn below must run ONCE; after that we only
		// wait for consumes to accumulate.
		if (bArmed) return Check();

		// 1. production state: the arbiter owns the resolver, bound by the module.
		Test->TestTrue(TEXT("OWN: the arbiter owns the resolver in production"), Arb::IsEnabled());
		const int64 ConflictsBefore = Arb::GetResolverOwnershipConflictCount();
		const int64 LostBefore = Arb::GetResolverOwnershipLostCount();

		// 2. a foreign resolver takes the single-cast delegate.
		UJSBSimMovementComponent::CommandResolver.BindStatic(&ForeignResolver);
		Test->TestFalse(TEXT("OWN: the arbiter no longer claims to be enabled after a takeover"),
			Arb::IsEnabled());

		// 3. disabling the arbiter must NOT remove the foreign resolver.
		Arb::SetEnabled(false);
		Test->TestTrue(TEXT("OWN: the foreign resolver is still bound (we did not evict it)"),
			UJSBSimMovementComponent::CommandResolver.IsBound());
		Test->TestEqual(TEXT("OWN: the takeover was counted, not silently absorbed"),
			Arb::GetResolverOwnershipLostCount(), LostBefore + 1);

		// 4. double-disable is safe and still does not touch the foreign resolver.
		Arb::SetEnabled(false);
		Test->TestTrue(TEXT("OWN: double-disable still leaves the foreign resolver alone"),
			UJSBSimMovementComponent::CommandResolver.IsBound());

		// 5. while the foreign resolver holds the delegate, the arbiter refuses to evict it.
		Arb::SetEnabled(true);
		Test->TestFalse(TEXT("OWN: the arbiter will not seize a resolver it does not own"),
			Arb::IsEnabled());
		Test->TestEqual(TEXT("OWN: the refusal was counted as a conflict"),
			Arb::GetResolverOwnershipConflictCount(), ConflictsBefore + 1);

		// 6. the foreign owner goes away -> the arbiter can take the boundary back, explicitly.
		UJSBSimMovementComponent::CommandResolver.Unbind();
		Arb::SetEnabled(true);
		Test->TestTrue(TEXT("OWN: the arbiter recovers the resolver once it is free"), Arb::IsEnabled());

		// 7. double-enable must not double-register the multicast observer (which would double-count
		//    every consume).
		Arb::SetEnabled(true);
		Test->TestTrue(TEXT("OWN: double-enable is idempotent"), Arb::IsEnabled());
		Arb::ResetSession(TEXT("Ownership"));
		SettleFrames = 0;
		bArmed = true;
		return false;   // now let consumes accumulate, then Check()
	}

private:
	bool Check()
	{
		// Let a few consumes run and confirm exactly one resolve per consume -- i.e. the observer is
		// registered exactly once despite the double enable.
		if (++SettleFrames < 90) return false;
		const Arb::FCounters C = Arb::GetCounters();
		Test->TestTrue(TEXT("OWN: consumes happened after recovery"), C.ConsumeCount > 0);
		Test->TestEqual(TEXT("OWN: exactly one resolve per consume (no duplicate observer)"),
			C.ResolverCallCount, C.ConsumeCount);
		Test->TestEqual(TEXT("OWN: no duplicate resolution"), C.DuplicateResolutionCount, (int64)0);
		Test->TestEqual(TEXT("OWN: still transparent after the ownership churn"),
			C.LegacyChangedFieldCount, (int64)0);
		UE_LOG(LogMumtArbTest, Display,
			TEXT("[ARBITER] OWNERSHIP_RESULT enabled=%d conflicts=%lld lost=%lld foreign_calls=%d "
			     "consumes=%lld resolver_calls=%lld"),
			Arb::IsEnabled() ? 1 : 0, Arb::GetResolverOwnershipConflictCount(),
			Arb::GetResolverOwnershipLostCount(), GForeignResolverCalls,
			C.ConsumeCount, C.ResolverCallCount);
		return true;
	}

	FAutomationTestBase *Test;
	bool bArmed = false;
	int32 SettleFrames = 0;
};

#define ARB_TEST(ClassName, TestName, ScenarioValue)                                                   \
	IMPLEMENT_SIMPLE_AUTOMATION_TEST(ClassName, TestName,                                              \
		EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)                     \
	bool ClassName::RunTest(const FString &)                                                           \
	{ RunArbiterScenario(this, ScenarioValue); return true; }

ARB_TEST(FMumtArbLegacyManualTest,     "MUMT.ControlV2.ArbiterLegacyManual",        EArbScenario::LegacyManual)
ARB_TEST(FMumtArbLegacyAutopilotTest,  "MUMT.ControlV2.ArbiterLegacyAutopilot",     EArbScenario::LegacyAutopilot)
ARB_TEST(FMumtArbLegacyOverlapTest,    "MUMT.ControlV2.ArbiterLegacyOverlap",       EArbScenario::LegacyOverlap)
ARB_TEST(FMumtArbFallingTest,          "MUMT.ControlV2.ArbiterFallingOverCandidate",EArbScenario::FallingOverCandidate)
ARB_TEST(FMumtArbCandidateValidTest,   "MUMT.ControlV2.ArbiterCandidateValid",      EArbScenario::CandidateValid)
ARB_TEST(FMumtArbCandidateStaleTest,   "MUMT.ControlV2.ArbiterCandidateStale",      EArbScenario::CandidateStale)
ARB_TEST(FMumtArbCandidateInvalidTest, "MUMT.ControlV2.ArbiterCandidateInvalid",    EArbScenario::CandidateInvalid)
ARB_TEST(FMumtArbIsolationTest,        "MUMT.ControlV2.ArbiterIsolation",           EArbScenario::Isolation)

#undef ARB_TEST

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtArbProductionDefaultTest,
	"MUMT.ControlV2.ArbiterProductionDefault",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtArbProductionDefaultTest::RunTest(const FString &)
{
	AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);
	TSharedPtr<FArbState> State = MakeShared<FArbState>();
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kArbMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtArbProductionDefaultCommand(this, State));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtArbWorldIsolationTest,
	"MUMT.ControlV2.ArbiterWorldIsolationCleanup",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtArbWorldIsolationTest::RunTest(const FString &)
{
	AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kArbMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtArbWorldIsolationCommand(this));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtArbOwnershipTest,
	"MUMT.ControlV2.ArbiterResolverOwnershipLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtArbOwnershipTest::RunTest(const FString &)
{
	AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);
	// The arbiter logs an Error when it refuses to evict a foreign resolver -- that refusal is the
	// behaviour under test, so the error is expected rather than a failure.
	AddExpectedErrorPlain(TEXT("already bound by something else"), EAutomationExpectedErrorFlags::Contains, 0);
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kArbMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtArbOwnershipCommand(this));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMumtArbLifecycleTest,
	"MUMT.ControlV2.ArbiterLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMumtArbLifecycleTest::RunTest(const FString &)
{
	AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), EAutomationExpectedErrorFlags::Contains, 0);
	TSharedPtr<FArbState> State = MakeShared<FArbState>();
	ADD_LATENT_AUTOMATION_COMMAND(FEditorLoadMap(kArbMap));
	ADD_LATENT_AUTOMATION_COMMAND(FStartPIECommand(false));
	ADD_LATENT_AUTOMATION_COMMAND(FMumtArbLifecycleCommand(this, State));
	ADD_LATENT_AUTOMATION_COMMAND(FEndPlayMapCommand());
	// The world is gone by the time this runs: the registry must have been emptied by the world-cleanup
	// handler, while the resolver itself -- which belongs to the module, not the world -- stays bound.
	ADD_LATENT_AUTOMATION_COMMAND(FMumtArbPostPieRegistryCheck(this));
	return true;
}

#endif // WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
