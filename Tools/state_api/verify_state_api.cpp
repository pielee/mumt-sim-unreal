// verify_state_api.cpp — host unit tests for the read-only State API conversion layer.
// Exercises MumtState::ConvertJsbToControlState (dependency-free) with no Unreal/JSBSim.
// Covers unit conversions, coordinate signs, validity, simulation time, pause/resume/reset
// events, the eas_to_tas (TAS/EAS) guard, CAS-is-not-EAS, and wind-is-not-zeroed. Exit 0 iff all pass.
#include "State/MumtControlState.h"
#include <cstdio>
#include <cmath>
#include <limits>
#include <string>

using namespace MumtState;

static int g_fail = 0, g_total = 0;

static void ok(bool cond, const std::string &name, const std::string &detail = "")
{
	g_total++;
	if (!cond) { g_fail++; std::printf("  [FAIL] %-42s %s\n", name.c_str(), detail.c_str()); }
	else       {           std::printf("  [ ok ] %-42s %s\n", name.c_str(), detail.c_str()); }
}
static bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
static std::string s(double v) { char b[64]; std::snprintf(b, sizeof b, "%.6g", v); return b; }

int main()
{
	const double NaN = std::numeric_limits<double>::quiet_NaN();
	const double Inf = std::numeric_limits<double>::infinity();

	std::printf("=== State API conversion tests ===\n");

	// ---- 1. EAS unit conversion (knot -> m/s), from GetVequivalentKTS ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VequivalentKTS = 100.0; r.VtFps = 300.0;
		auto o = ConvertJsbToControlState(r, t);
		ok(near(o.EquivalentAirspeed_mps, 100.0 * KNOT_TO_MPS, 1e-6) && o.bEasValid,
		   "EAS 100 kt -> 51.4444 m/s", s(o.EquivalentAirspeed_mps));
	}
	// ---- 2. CAS is NOT substituted for EAS ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VequivalentKTS = 100.0; r.VcalibratedKTS = 200.0; r.VtFps = 300.0;
		auto o = ConvertJsbToControlState(r, t);
		ok(near(o.EquivalentAirspeed_mps, 100.0 * KNOT_TO_MPS, 1e-6),
		   "EAS uses VequivalentKTS, ignores CAS(200)", s(o.EquivalentAirspeed_mps));
	}
	// ---- 3. TAS unit conversion (ft/s -> m/s) ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VtFps = 300.0; r.VequivalentKTS = 100.0;
		auto o = ConvertJsbToControlState(r, t);
		ok(near(o.TrueAirspeed_mps, 300.0 * FT_TO_M, 1e-9) && o.bTasValid,
		   "TAS 300 ft/s -> 91.44 m/s", s(o.TrueAirspeed_mps));
	}
	// ---- 4. eas_to_tas = TAS/EAS (PX4: TAS = EAS * eas_to_tas); EAS 51.4444, TAS 91.44 ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VequivalentKTS = 100.0; r.VtFps = 300.0;
		auto o = ConvertJsbToControlState(r, t);
		double expect = (300.0 * FT_TO_M) / (100.0 * KNOT_TO_MPS); // TAS/EAS
		ok(o.bRatioValid && near(o.EasToTasRatio, expect, 1e-9) && near(o.EasToTasRatio, 1.7774, 1e-3),
		   "eas_to_tas = TAS/EAS ~ 1.7774 (EAS 51.4444, TAS 91.44)", s(o.EasToTasRatio));
	}
	// ---- 4b. EAS == TAS -> ratio == 1 ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VtFps = 300.0; r.VequivalentKTS = (300.0 * FT_TO_M) / KNOT_TO_MPS;
		auto o = ConvertJsbToControlState(r, t);
		ok(o.bRatioValid && near(o.EasToTasRatio, 1.0, 1e-6), "EAS == TAS -> ratio ~ 1", s(o.EasToTasRatio));
	}
	// ---- 4c. TAS > EAS -> ratio > 1 ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VequivalentKTS = 100.0; r.VtFps = 400.0; // TAS 121.9 > EAS 51.4
		auto o = ConvertJsbToControlState(r, t);
		ok(o.bRatioValid && o.EasToTasRatio > 1.0, "TAS > EAS -> ratio > 1", s(o.EasToTasRatio));
	}
	// ---- 5. EAS ~ 0 guard: ratio invalid, fallback 1.0 ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VequivalentKTS = 0.0; r.VtFps = 300.0;
		auto o = ConvertJsbToControlState(r, t);
		ok(!o.bRatioValid && near(o.EasToTasRatio, 1.0), "EAS~0 -> ratio invalid, fallback 1.0", s(o.EasToTasRatio));
	}
	// ---- 5b. EAS > 0, TAS = 0 -> ratio 0 is abnormal -> invalid, fallback 1.0 ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VequivalentKTS = 100.0; r.VtFps = 0.0; // EAS>0, TAS=0
		auto o = ConvertJsbToControlState(r, t);
		ok(!o.bRatioValid && near(o.EasToTasRatio, 1.0), "EAS>0, TAS=0 -> ratio invalid, fallback 1.0", s(o.EasToTasRatio));
	}
	// ---- 6. Wind sign + scale, not zeroed ----
	{
		FMumtStateTracker t; FJsbRawState r; r.WindNorthFps = 32.8084; r.WindEastFps = -16.4042;
		auto o = ConvertJsbToControlState(r, t);
		ok(near(o.WindNorth_mps, 10.0, 1e-3) && near(o.WindEast_mps, -5.0, 1e-3) && o.bWindValid,
		   "Wind N=+10 E=-5 m/s (sign+scale, not zeroed)", s(o.WindNorth_mps) + "," + s(o.WindEast_mps));
	}
	// ---- 7. Altitude ft -> m ----
	{
		FMumtStateTracker t; FJsbRawState r; r.AltAslFt = 3280.8399;
		auto o = ConvertJsbToControlState(r, t);
		ok(near(o.AltitudeAsl_m, 1000.0, 1e-3) && o.bAltValid, "Altitude 3280.84 ft -> 1000 m", s(o.AltitudeAsl_m));
	}
	// ---- 8. Climb-rate sign (+up when hdot>0, -down when hdot<0) ----
	{
		FMumtStateTracker t1, t2; FJsbRawState up, dn; up.HdotFps = 32.8084; dn.HdotFps = -32.8084;
		auto ou = ConvertJsbToControlState(up, t1); auto od = ConvertJsbToControlState(dn, t2);
		ok(near(ou.ClimbRate_mps, 10.0, 1e-3) && ou.ClimbRate_mps > 0 && near(od.ClimbRate_mps, -10.0, 1e-3),
		   "Climb +10 (up>0) / -10 (down<0) m/s", s(ou.ClimbRate_mps) + "," + s(od.ClimbRate_mps));
	}
	// ---- 9. Pitch/Roll radians passthrough ----
	{
		FMumtStateTracker t; FJsbRawState r; r.PitchRad = 0.1745; r.RollRad = -0.2618;
		auto o = ConvertJsbToControlState(r, t);
		ok(near(o.Pitch_rad, 0.1745) && near(o.Roll_rad, -0.2618) && o.bAttitudeValid,
		   "Pitch +0.1745 / Roll -0.2618 rad (passthrough)", s(o.Pitch_rad) + "," + s(o.Roll_rad));
	}
	// ---- 10. Simulation time monotonic -> microseconds ----
	{
		FMumtStateTracker t; FJsbRawState a, b; a.SimTimeSec = 1.5; b.SimTimeSec = 1.52;
		auto oa = ConvertJsbToControlState(a, t); auto ob = ConvertJsbToControlState(b, t);
		ok(oa.SimTimeMicros == 1500000ULL && ob.SimTimeMicros == 1520000ULL && ob.SimTimeMicros > oa.SimTimeMicros && oa.bTimeValid,
		   "SimTime 1.5->1.52 s monotonic, micros exact", std::to_string((unsigned long long)ob.SimTimeMicros));
	}
	// ---- 11. Pause (Holding): bPaused set, forward accel invalid, sim time frozen ----
	{
		FMumtStateTracker t; FJsbRawState a, b; a.SimTimeSec = 1.0; a.VtFps = 300;
		b.SimTimeSec = 1.0; b.VtFps = 300; b.bHolding = true; // held: sim time frozen
		ConvertJsbToControlState(a, t); auto o = ConvertJsbToControlState(b, t);
		ok(o.bPaused && !o.bTASRateValid, "Pause -> bPaused, TAS rate invalid", "");
	}
	// ---- 12. Resume event (holding true -> false) ----
	{
		FMumtStateTracker t; FJsbRawState a, b, c;
		a.SimTimeSec = 1.0; b.SimTimeSec = 1.0; b.bHolding = true; c.SimTimeSec = 1.02; c.bHolding = false;
		ConvertJsbToControlState(a, t); ConvertJsbToControlState(b, t); auto o = ConvertJsbToControlState(c, t);
		ok(o.bResumeEvent && !o.bPaused, "Resume event on hold->run transition", "");
	}
	// ---- 13. Reset event (sim time backwards -> generation++) ----
	{
		FMumtStateTracker t; FJsbRawState a, b; a.SimTimeSec = 5.0; b.SimTimeSec = 0.0;
		auto oa = ConvertJsbToControlState(a, t); auto ob = ConvertJsbToControlState(b, t);
		ok(ob.bResetEvent && ob.ResetGeneration == oa.ResetGeneration + 1,
		   "Reset event on time-backwards, generation++", "gen " + std::to_string(ob.ResetGeneration));
	}
	// ---- 14. TAS rate = d(TAS)/dt (information only, NOT speed_deriv_forward) ----
	{
		FMumtStateTracker t; FJsbRawState a, b;
		a.SimTimeSec = 1.0; a.VtFps = 300.0;      // TAS = 91.44 m/s
		b.SimTimeSec = 1.1; b.VtFps = 303.28084;  // TAS = 92.44 m/s, dt=0.1 -> +10 m/s^2
		auto oa = ConvertJsbToControlState(a, t); auto ob = ConvertJsbToControlState(b, t);
		ok(!oa.bTASRateValid && ob.bTASRateValid && near(ob.TASRateMps2, 10.0, 1e-3),
		   "TASRate d(TAS)/dt = +10 m/s^2 (first sample invalid)", s(ob.TASRateMps2));
	}
	// ---- 14b. ForwardAcceleration (body-X) is NEVER supplied this stage ----
	{
		FMumtStateTracker t; FJsbRawState a, b;
		a.SimTimeSec = 1.0; a.VtFps = 300.0; b.SimTimeSec = 1.1; b.VtFps = 303.28084;
		ConvertJsbToControlState(a, t); auto ob = ConvertJsbToControlState(b, t);
		ok(!ob.bForwardAccelerationValid && near(ob.ForwardAccelerationMps2, 0.0),
		   "ForwardAcceleration invalid+0 even when TASRate valid (PX4 v1.17 passes 0)", "");
	}
	// ---- 15. NaN / Inf validity ----
	{
		FMumtStateTracker t; FJsbRawState r; r.VtFps = NaN; r.VequivalentKTS = Inf; r.AltAslFt = NaN;
		auto o = ConvertJsbToControlState(r, t);
		ok(!o.bTasValid && !o.bEasValid && !o.bAltValid, "NaN/Inf inputs -> fields invalid", "");
	}
	// ---- 16. Unavailable JSBSim frame -> everything invalid ----
	{
		FMumtStateTracker t; FJsbRawState r; r.bValidFrame = false;
		auto o = ConvertJsbToControlState(r, t);
		ok(!o.bEasValid && !o.bTasValid && !o.bWindValid && !o.bAltValid && !o.bClimbValid &&
		   !o.bAttitudeValid && !o.bTimeValid && !o.bTASRateValid && !o.bForwardAccelerationValid,
		   "Invalid frame -> all fields invalid", "");
	}

	std::printf("\nState API tests: %d/%d passed, %d failed\n", g_total - g_fail, g_total, g_fail);
	return g_fail ? 1 : 0;
}
