// probe_jsbsim_getters.cpp — compile-only check (no Unreal, no linking) that:
//   (1) the read-only snapshot getter's JSBSim usage matches the real pinned JSBSim API
//       (names, signatures, enum indices), and
//   (2) a snapshot POD with the plugin's FJsbFlightSnapshot field layout feeds the templated
//       MumtState::ConvertJsbToControlState adapter.
// Build: g++ -std=c++17 -fsyntax-only -I <plugin>/Source/ThirdParty/JSBSim/Include \
//            -I Source/MUMT_Sim/Public  probe_jsbsim_getters.cpp
// The real FJsbFlightSnapshot lives in the UE plugin header; this host mirror keeps the field
// contract auditable without Unreal. The full UE compile is deferred (see docs/STATE_API.md).
#include "FGFDMExec.h"
#include "models/FGAuxiliary.h"
#include "models/FGPropagate.h"
#include "models/atmosphere/FGWinds.h"
#include "State/MumtControlState.h"

using namespace JSBSim;

// Host mirror of UJSBSimMovementComponent::FJsbFlightSnapshot (identical field names + units).
struct FJsbFlightSnapshotMirror {
	bool   bValidFrame = false;
	double VequivalentKTS = 0.0, VtFps = 0.0, VcalibratedKTS = 0.0;
	double WindNorthFps = 0.0, WindEastFps = 0.0, AltAslFt = 0.0, HdotFps = 0.0;
	double PitchRad = 0.0, RollRad = 0.0, SimTimeSec = 0.0;
	bool   bHolding = false;
};

// Mirrors exactly the calls in UJSBSimMovementComponent::GetJsbFlightSnapshot(), then feeds the
// resulting snapshot through the MUMT state adapter (proving the whole chain compiles on host).
void probe(FGFDMExec *Exec, FGAuxiliary *Auxiliary, FGWinds *Winds, FGPropagate *Propagate)
{
	FJsbFlightSnapshotMirror snap;
	snap.bValidFrame    = true;
	snap.VequivalentKTS = Auxiliary->GetVequivalentKTS();            // EAS  [knots]
	snap.VtFps          = Auxiliary->GetVt();                        // TAS  [ft/s]
	snap.VcalibratedKTS = Auxiliary->GetVcalibratedKTS();            // CAS  [knots]
	snap.WindNorthFps   = Winds->GetTotalWindNED(FGJSBBase::eNorth); // wind N [ft/s]
	snap.WindEastFps    = Winds->GetTotalWindNED(FGJSBBase::eEast);  // wind E [ft/s]
	snap.AltAslFt       = Propagate->GetAltitudeASL();              // ASL  [ft]
	snap.HdotFps        = Propagate->Gethdot();                     // climb [ft/s]
	snap.PitchRad       = Propagate->GetEuler(FGJSBBase::eTht);     // pitch [rad]
	snap.RollRad        = Propagate->GetEuler(FGJSBBase::ePhi);     // roll  [rad]
	snap.SimTimeSec     = Exec->GetSimTime();                       // sim time [s]
	snap.bHolding       = Exec->Holding();                          // paused

	// The templated adapter must accept the snapshot layout directly (getter -> adapter).
	MumtState::FMumtStateTracker tracker;
	MumtState::FMumtControlState state = MumtState::ConvertJsbToControlState(snap, tracker);
	(void)state;
}
