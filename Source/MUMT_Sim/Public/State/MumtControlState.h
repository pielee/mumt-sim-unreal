// =====================================================================
// MumtControlState.h  —  read-only control-state boundary (pure C++)
// ---------------------------------------------------------------------
// SI units, local NED frame. Contains NO Unreal and NO JSBSim types, so it
// compiles on the host and is unit-testable in isolation (Tools/state_api).
//
// Purpose: the NPFG/TECS guidance controllers must receive aircraft state
// WITHOUT touching the protected JSBSim objects (FGFDMExec / FGAuxiliary /
// FGWinds / FGAccelerations / FGPropagate) owned by UJSBSimMovementComponent.
// The live actor holds a base UJSBSimMovementComponent (obtained via
// FindComponentByClass), so a subclass cannot read its state. Instead a minimal
// read-only raw-snapshot getter on that existing component fills FJsbRawState,
// and ConvertJsbToControlState() here turns it into the SI/NED FMumtControlState
// the controller consumes. This file is the single place unit/frame/validity
// conversions live (see docs/STATE_API.md for the ownership analysis).
//
// Boundary rules honoured here:
//   * EAS comes from GetVequivalentKTS (see FJsbRawState.VequivalentKTS); CAS
//     (VcalibratedKTS) is carried only for reference and is NEVER used as EAS.
//   * Wind is the actual JSBSim total wind (GetTotalWindNED); it is never
//     replaced by zero. A zero wind only appears if JSBSim itself reports zero.
//   * eas_to_tas = TAS/EAS is computed from the real EAS and TAS with an EAS floor
//     to protect the division; below the floor it is marked invalid.
//   * Simulation time is JSBSim GetSimTime (monotonic sim time), not wall clock.
//   * Pause/Resume/Reset are surfaced as explicit events + a generation counter.
// =====================================================================
#pragma once
#include <cstdint>
#include <cmath>

namespace MumtState {

// ---- unit constants ----
constexpr double KNOT_TO_MPS = 0.514444444444444; // international knot -> m/s
constexpr double FT_TO_M     = 0.3048;            // foot -> metre
constexpr double MIN_EAS_MPS = 1.0;               // EAS floor for the eas_to_tas (=TAS/EAS) ratio guard

// Raw JSBSim-native snapshot (units exactly as JSBSim returns them). This mirrors the plugin POD
// UJSBSimMovementComponent::FJsbFlightSnapshot field-for-field, so GetJsbFlightSnapshot()'s result
// can be passed straight into ConvertJsbToControlState() below (templated on the snapshot type).
// FJsbRawState is the host-testable reference layout for that contract.
struct FJsbRawState {
	double VequivalentKTS = 0.0; // EAS  : FGAuxiliary::GetVequivalentKTS   [knots]
	double VtFps          = 0.0; // TAS  : FGAuxiliary::GetVt               [ft/s]
	double VcalibratedKTS = 0.0; // CAS  : reference only, NEVER used as EAS [knots]
	double WindNorthFps   = 0.0; // wind : FGWinds::GetTotalWindNED(eNorth) [ft/s]
	double WindEastFps    = 0.0; // wind : FGWinds::GetTotalWindNED(eEast)  [ft/s]
	double AltAslFt       = 0.0; // alt  : FGPropagate::GetAltitudeASL      [ft, +up]
	double HdotFps        = 0.0; // climb: FGPropagate::Gethdot             [ft/s, +up]
	double PitchRad       = 0.0; // pitch: FGPropagate::GetEuler(eTht)      [rad]
	double RollRad        = 0.0; // roll : FGPropagate::GetEuler(ePhi)      [rad]
	double SimTimeSec     = 0.0; // time : FGFDMExec::GetSimTime            [s, monotonic]
	bool   bHolding       = false; //      FGFDMExec::Holding (sim time frozen = paused)
	bool   bValidFrame    = true;  // false when the JSBSim objects are unavailable
	double VehicleCgEcefXFt = 0.0, VehicleCgEcefYFt = 0.0, VehicleCgEcefZFt = 0.0;
	double EcefVelocityXFps = 0.0, EcefVelocityYFps = 0.0, EcefVelocityZFps = 0.0;
	double GeodeticLatitudeRad = 0.0, LongitudeRad = 0.0, GeodeticAltitudeFt = 0.0;
};

// SI / local-NED read-only snapshot consumed by the guidance/energy controllers.
struct FMumtControlState {
	double   EquivalentAirspeed_mps = 0.0; // EAS  (+)
	double   TrueAirspeed_mps       = 0.0; // TAS  (+)
	double   EasToTasRatio          = 1.0; // eas_to_tas = TAS/EAS (PX4 TECS input; guarded, 1.0 fallback)
	double   WindNorth_mps          = 0.0; // total wind, local N (+N)
	double   WindEast_mps           = 0.0; // total wind, local E (+E)
	double   AltitudeAsl_m          = 0.0; // ASL (+up)
	double   ClimbRate_mps          = 0.0; // (+up)
	double   Pitch_rad              = 0.0;
	double   Roll_rad               = 0.0;
	double   TASRateMps2            = 0.0; // d(TAS)/dt (information only; NOT PX4 speed_deriv_forward)
	double   ForwardAccelerationMps2 = 0.0; // body-X forward acceleration — NOT supplied this stage
	double   SimTime_s              = 0.0;
	uint64_t SimTimeMicros          = 0;   // SimTime_s in microseconds (TECS clock input)

	// events
	bool     bPaused        = false; // sim time frozen this sample
	bool     bResumeEvent   = false; // pause -> run transition on this sample
	bool     bResetEvent    = false; // sim time went backwards (JSBSim reset)
	uint32_t ResetGeneration = 0;    // increments once per detected reset

	// per-field validity (finite + physically-usable + available)
	bool bEasValid = false, bTasValid = false, bRatioValid = false, bWindValid = false;
	bool bAltValid = false, bClimbValid = false, bAttitudeValid = false;
	bool bTASRateValid = false;             // d(TAS)/dt available this sample
	bool bForwardAccelerationValid = false; // body-X forward accel — not supplied this stage
	bool bTimeValid = false;
};

// Cross-tick state needed for dt, event detection and the airspeed derivative.
struct FMumtStateTracker {
	bool     bInitialized    = false;
	double   PrevSimTimeSec  = 0.0;
	double   PrevTasMps      = 0.0;
	bool     bPrevHolding    = false;
	uint32_t ResetGeneration = 0;
};

inline bool IsFiniteD(double x) { return std::isfinite(x); }

// Pure conversion. Updates `tracker`. Call once per simulation tick so the
// ForwardAccel finite difference sees a genuine dt.
//
// Forward-acceleration semantics (verified against the PX4 v1.17.0 call site):
//   TECS `speed_deriv_forward` is used ONLY as the airspeed-filter rate
//   measurement (equivalent_airspeed_rate = speed_deriv_forward / eas_to_tas,
//   TECS.cpp:743). In v1.17.0 the caller passes it as 0: airspeed_rate_estimate
//   is hard-set to 0 by a HOTFIX because the body-forward-acceleration estimate
//   "has shown to lead to high biases" (FwLateralLongitudinalControl.cpp:383-385).
//   Therefore this layer claims NO value as speed_deriv_forward:
//     * TASRateMps2 = unfiltered d(TAS)/dt, exposed as information only.
//     * ForwardAccelerationMps2 = body-X forward acceleration, NOT supplied here
//       (kept invalid); supplying it would reintroduce the bias PX4 removed.
//   A v1.17.0-faithful TECS integration passes 0 for speed_deriv_forward.
template <class TSnap>
inline FMumtControlState ConvertJsbToControlState(const TSnap &r, FMumtStateTracker &tracker)
{
	FMumtControlState s;

	if (!r.bValidFrame) {
		// JSBSim not available: everything invalid, tracker untouched except gen echo.
		s.ResetGeneration = tracker.ResetGeneration;
		return s;
	}

	// --- airspeeds ---
	s.EquivalentAirspeed_mps = r.VequivalentKTS * KNOT_TO_MPS; // EAS (NOT CAS)
	s.TrueAirspeed_mps       = r.VtFps * FT_TO_M;
	s.bEasValid = IsFiniteD(s.EquivalentAirspeed_mps) && s.EquivalentAirspeed_mps >= 0.0;
	s.bTasValid = IsFiniteD(s.TrueAirspeed_mps) && s.TrueAirspeed_mps >= 0.0;

	// --- eas_to_tas = TAS / EAS  (PX4 TECS convention: TAS = EAS * eas_to_tas; this is the
	//     eas_to_tas input to TECS). Guard against EAS ~ 0, and reject a non-finite or
	//     non-positive ratio (e.g. TAS = 0 while EAS > 0) as invalid. ---
	s.EasToTasRatio = 1.0; // default = safe fallback; overwritten only when valid
	s.bRatioValid   = false;
	if (s.bEasValid && s.bTasValid && s.EquivalentAirspeed_mps >= MIN_EAS_MPS) {
		const double ratio = s.TrueAirspeed_mps / s.EquivalentAirspeed_mps;
		if (IsFiniteD(ratio) && ratio > 0.0) {
			s.EasToTasRatio = ratio;
			s.bRatioValid   = true;
		}
	}

	// --- wind (JSBSim total wind NED velocity vector) -> m/s, never zeroed here ---
	s.WindNorth_mps = r.WindNorthFps * FT_TO_M;
	s.WindEast_mps  = r.WindEastFps * FT_TO_M;
	s.bWindValid    = IsFiniteD(s.WindNorth_mps) && IsFiniteD(s.WindEast_mps);

	// --- altitude / climb (+up) ---
	s.AltitudeAsl_m = r.AltAslFt * FT_TO_M;
	s.ClimbRate_mps = r.HdotFps * FT_TO_M;
	s.bAltValid     = IsFiniteD(s.AltitudeAsl_m);
	s.bClimbValid   = IsFiniteD(s.ClimbRate_mps);

	// --- attitude (radians passthrough) ---
	s.Pitch_rad      = r.PitchRad;
	s.Roll_rad       = r.RollRad;
	s.bAttitudeValid = IsFiniteD(s.Pitch_rad) && IsFiniteD(s.Roll_rad);

	// --- simulation time ---
	s.SimTime_s     = r.SimTimeSec;
	s.bTimeValid    = IsFiniteD(s.SimTime_s) && s.SimTime_s >= 0.0;
	s.SimTimeMicros = s.bTimeValid ? (uint64_t)std::llround(s.SimTime_s * 1.0e6) : 0;

	// --- events / dt ---
	s.bPaused = r.bHolding;
	double dt = 0.0;
	bool dtUsable = false;
	if (tracker.bInitialized) {
		dt = r.SimTimeSec - tracker.PrevSimTimeSec;
		if (dt < 0.0) {                       // time went backwards -> reset
			s.bResetEvent = true;
			tracker.ResetGeneration++;
		} else if (dt > 0.0 && !r.bHolding) { // genuine forward step while running
			dtUsable = true;
		}
		if (tracker.bPrevHolding && !r.bHolding) s.bResumeEvent = true;
	}
	s.ResetGeneration = tracker.ResetGeneration;

	// --- TAS rate = d(TAS)/dt (unfiltered numerical derivative, information only) ---
	if (dtUsable && s.bTasValid && !s.bResetEvent && !s.bResumeEvent) {
		s.TASRateMps2   = (s.TrueAirspeed_mps - tracker.PrevTasMps) / dt;
		s.bTASRateValid = IsFiniteD(s.TASRateMps2);
	} else {
		s.TASRateMps2   = 0.0;
		s.bTASRateValid = false;
	}
	// --- body-X forward acceleration: NOT supplied this stage (see note above) ---
	s.ForwardAccelerationMps2   = 0.0;
	s.bForwardAccelerationValid = false;

	// --- advance tracker ---
	tracker.PrevSimTimeSec = r.SimTimeSec;
	tracker.PrevTasMps     = s.TrueAirspeed_mps;
	tracker.bPrevHolding   = r.bHolding;
	tracker.bInitialized   = true;
	return s;
}

} // namespace MumtState
