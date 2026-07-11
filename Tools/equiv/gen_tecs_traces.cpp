// gen_tecs_traces.cpp — emits the shared TECS input CSV (data only, no control law).
// Usage: gen_tecs_traces <out.csv>
// Rows 1..216 reproduce the exact reported baseline sequence; appended segments add
// explicit airspeed-setpoint step and pitch/throttle saturation for Section 2 coverage.
#include "trace_io.h"
#include <limits>

int main(int argc, char **argv)
{
	if (argc < 2) { std::fprintf(stderr, "usage: %s <out.csv>\n", argv[0]); return 1; }
	const float N = std::numeric_limits<float>::quiet_NaN();
	using equiv::TecsTrace;
	std::vector<TecsTrace> tr;
	uint64_t t = 1000000;

	// Mirror of the reported harness `add`: fixed pitch .03, eas_sp 35, ratio 1.12,
	// throttle [0,1] trim .5, pitch [-.45,.45], climb 8, sink 5.
	auto add = [&](const char *name, float alt, float asp, float eas, float hr,
	               float accel = 0, float hsp = std::numeric_limits<float>::quiet_NaN(),
	               int action = 0, uint64_t step = 20000) {
		t += step;
		tr.push_back(TecsTrace{name, t, 0.03f, alt, asp, 35.f, eas, 1.12f, 0.f, 1.f, 0.5f,
		                       -0.45f, 0.45f, 8.f, 5.f, accel, hr, hsp, action});
	};
	// Full-control variant for saturation coverage (explicit eas_sp + limits).
	auto addF = [&](const char *name, float alt, float asp, float eas_sp, float eas, float hr,
	                float tmin, float tmax, float pmin, float pmax, int action = 0, uint64_t step = 20000) {
		t += step;
		tr.push_back(TecsTrace{name, t, 0.03f, alt, asp, eas_sp, eas, 1.12f, tmin, tmax, 0.5f,
		                       pmin, pmax, 8.f, 5.f, 0.f, hr, N, action});
	};

	// ---------- reported 216-sample baseline (unchanged behavior) ----------
	add("hold", 1000, 1000, 35, 0, 0, N, 1, 0);
	for (int i = 0; i < 20; i++) add("hold", 1000, 1000, 35, 0);
	add("alt_step", 1000, 1080, 35, 0);
	for (int i = 0; i < 20; i++) add("climb", 1000 + i, 1080, 35, 2);
	add("sink", 1030, 980, 35, -3);
	for (int i = 0; i < 15; i++) add("fast_descend", 1030 - i, 980, 35, -3);
	for (int i = 0; i < 150; i++) add("underspeed", 1000, 1000, 0, 0);
	add("speed_step", 1000, 1000, 55, 0);
	add("invalid_eas", 1000, 1100, N, 0);
	add("reset", 1000, 1100, 25, 0, 0, N, 2);
	add("estimator_step", 950, 1100, 25, 0, 0, N, 4);
	add("dt_sub_ms", 950, 1100, 25, 0, 0, N, 0, 500);
	add("dt_over_1s", 950, 1100, 25, 0, 0, N, 0, 1500000);
	add("pause", 950, 1100, 25, 0, 0, N, 0, 0);
	add("restart", 950, 1100, 25, 0, 0, N, 1, 20000);

	// ---------- appended coverage segments (Section 2) ----------
	// Each segment starts with action=1 (fresh controller) and holds the measured
	// altitude constant while the reference model ramps, so the demand is sustained.

	// airspeed setpoint step: eas_sp 45 -> 55 (measured eas held at 45)
	addF("asp_pre", 1000, 1000, 45, 45, 0, 0.f, 1.f, -0.45f, 0.45f, 1);
	for (int i = 0; i < 5; i++) addF("asp_pre", 1000, 1000, 45, 45, 0, 0.f, 1.f, -0.45f, 0.45f);
	for (int i = 0; i < 8; i++) addF("asp_step", 1000, 1000, 55, 45, 0, 0.f, 1.f, -0.45f, 0.45f);

	// throttle UPPER saturation is proven by the underspeed segment (throttle pinned to
	// throttle_max=1.0); this climb-band segment adds equivalence coverage under climb.
	addF("thr_climb", 1000, 1400, 45, 45, 0, 0.4f, 0.6f, -0.45f, 0.45f, 1);
	for (int i = 0; i < 30; i++) addF("thr_climb", 1000, 1400, 45, 45, 0, 0.4f, 0.6f, -0.45f, 0.45f);
	// throttle LOWER saturation: fully-engaged fast descend (large descending altitude
	// error, eas above the underspeed threshold so no underspeed override) forces throttle
	// to throttle_min per TECSControl::_calcThrottleControl. Needs ~2s for fast_descend->1.
	addF("thr_sat_lo", 2000, 1000, 50, 50, 0, 0.3f, 0.7f, -0.45f, 0.45f, 1);
	for (int i = 0; i < 130; i++) addF("thr_sat_lo", 2000, 1000, 50, 50, 0, 0.3f, 0.7f, -0.45f, 0.45f);
	// pitch UPPER saturation: hold altitude, far-above setpoint, narrow band [-0.05,0.05]
	addF("pitch_sat_hi", 1000, 2000, 45, 45, 0, 0.f, 1.f, -0.05f, 0.05f, 1);
	for (int i = 0; i < 100; i++) addF("pitch_sat_hi", 1000, 2000, 45, 45, 0, 0.f, 1.f, -0.05f, 0.05f);
	// pitch LOWER saturation: hold altitude, far-below setpoint, narrow band [-0.05,0.05]
	addF("pitch_sat_lo", 2000, 1000, 45, 45, 0, 0.f, 1.f, -0.05f, 0.05f, 1);
	for (int i = 0; i < 100; i++) addF("pitch_sat_lo", 2000, 1000, 45, 45, 0, 0.f, 1.f, -0.05f, 0.05f);

	FILE *f = std::fopen(argv[1], "w");
	if (!f) { std::perror(argv[1]); return 2; }
	std::fprintf(f, "%s\n", equiv::TECS_IN_HEADER);
	for (auto &x : tr) equiv::writeTecsRow(f, x);
	std::fclose(f);
	std::fprintf(stderr, "wrote %zu TECS traces to %s\n", tr.size(), argv[1]);
	return 0;
}
