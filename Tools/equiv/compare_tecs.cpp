// compare_tecs.cpp — Comparator ONLY. Links no controller (neither PX4 nor MUMT).
// Reads the shared input CSV + both output CSVs, checks per-field equivalence
// tolerances (+ exact timestamp), and proves Section 2 guard coverage from the
// PX4-original output using behavioral signatures (not "merely executed").
// Exit 0 iff every field within tolerance AND every required guard actually engaged.
#include "trace_io.h"
#include <cmath>
#include <string>
#include <vector>
using namespace equiv;

static float ferr(float a, float b)
{
	if (std::isnan(a) && std::isnan(b)) return 0.f;
	if (!std::isfinite(a) || !std::isfinite(b)) return (a == b) ? 0.f : INFINITY;
	return std::fabs(a - b);
}

int main(int argc, char **argv)
{
	if (argc < 4) { std::fprintf(stderr, "usage: %s <in.csv> <orig.csv> <port.csv>\n", argv[0]); return 1; }
	auto IN = readTecs(argv[1]);
	auto O = readCells(argv[2]);
	auto P = readCells(argv[3]);
	if (O.size() != P.size() || O.size() != IN.size()) {
		std::fprintf(stderr, "row count mismatch: in=%zu orig=%zu port=%zu\n", IN.size(), O.size(), P.size());
		return 3;
	}
	const char *names[TECS_OUT_N] = {"pitch_sp", "throttle_sp", "underspeed", "filtered_tas", "tas_rate",
	                                 "te_rate_sp", "te_rate_est", "eb_rate_sp", "eb_rate_est",
	                                 "pitch_i", "throttle_i", "fast_descend", "alt_ref"};
	const float tol[TECS_OUT_N] = {1e-5f, 1e-5f, 1e-5f, 1e-5f, 1e-5f, 1e-4f, 1e-4f, 1e-4f, 1e-4f, 1e-6f, 1e-6f, 0.f, 1e-4f};
	const int TS = TECS_OUT_N + 1; // timestamp cell index (name + 13 floats)

	float mx[TECS_OUT_N] = {};
	int fails = 0, ts_fails = 0;
	for (size_t i = 0; i < O.size(); i++) {
		for (int j = 0; j < TECS_OUT_N; j++) {
			float a = pf(O[i][j + 1]), b = pf(P[i][j + 1]);
			float e = ferr(a, b);
			if (e > mx[j]) mx[j] = e;
			if (e > tol[j]) { fails++; std::printf("FAIL row=%zu %-13s orig=%g port=%g err=%g\n", i, names[j], a, b, e); }
		}
		if (pu(O[i][TS]) != pu(P[i][TS])) { ts_fails++; std::printf("FAIL row=%zu timestamp orig=%s port=%s\n", i, O[i][TS].c_str(), P[i][TS].c_str()); }
	}

	std::printf("\n=== TECS equivalence (separate original vs port binaries) ===\n");
	for (int j = 0; j < TECS_OUT_N; j++) std::printf("MAX %-14s %.9g  (tol %.0e)\n", names[j], mx[j], tol[j]);
	std::printf("timestamp exact mismatches=%d\n", ts_fails);
	std::printf("rows=%zu equivalence_failures=%d\n", O.size(), fails + ts_fails);

	// ---------- Section 2 guard coverage (behavioral, from PX4-original) ----------
	auto oc = [&](size_t i, int j) { return pf(O[i][j + 1]); };
	auto ts = [&](size_t i) { return pu(O[i][TS]); };
	bool underspeed = false, fastdesc = false, thi = false, tlo = false, phi = false, plo = false;
	bool altstep = false, aspstep = false, reset = false, estim = false, dtsub = false, dtover = false, pause = false, restart = false;
	float maxund = 0.f, maxfd = 0.f, minreset_i = 1e9f;
	for (size_t i = 0; i < IN.size(); i++) {
		const auto &t = IN[i];
		if (oc(i, 2) > maxund) maxund = oc(i, 2);
		if (oc(i, 11) > maxfd) maxfd = oc(i, 11);
		if (oc(i, 2) > 0.f) underspeed = true;
		if (oc(i, 11) > 0.f) fastdesc = true;
		// saturation: output pinned exactly to a limit that differs from trim/other bound
		if (std::fabs(oc(i, 1) - t.tmax) < 1e-6f && t.tmax != t.ttrim && t.tmax != t.tmin) thi = true;
		if (std::fabs(oc(i, 1) - t.tmin) < 1e-6f && t.tmin != t.ttrim) tlo = true;
		if (std::fabs(oc(i, 0) - t.pmax) < 1e-6f && t.pmax != 0.f) phi = true;
		if (std::fabs(oc(i, 0) - t.pmin) < 1e-6f && t.pmin != 0.f) plo = true;
		// altitude/airspeed setpoint step
		if (i > 0 && std::fabs(t.alt_sp - IN[i - 1].alt_sp) > 50.f) altstep = true;
		if (i > 0 && std::fabs(t.eas_sp - IN[i - 1].eas_sp) > 5.f) aspstep = true;
		// integrator reset: action bit 2 zeroed both integrators (near-zero after one step)
		if (t.action & 2) {
			float s = std::fabs(oc(i, 9)) + std::fabs(oc(i, 10));
			if (s < minreset_i) minreset_i = s;
			if (std::fabs(oc(i, 9)) < 1e-3f && std::fabs(oc(i, 10)) < 1e-3f) reset = true;
		}
		// estimator step: reference snapped to stepped altitude (distinct from setpoint)
		if ((t.action & 4) && std::fabs(oc(i, 12) - t.alt) < 2.0f && std::fabs(t.alt - t.alt_sp) > 50.f) estim = true;
		// dt guards (behavioral): hold freezes timestamp+outputs; reinit advances timestamp
		if (i > 0) {
			double dt = (double)((long long)t.us - (long long)IN[i - 1].us) / 1e6;
			bool frozen_ts = (ts(i) == ts(i - 1));
			bool frozen_out = true;
			for (int k = 1; k <= TECS_OUT_N; k++) if (O[i][k] != O[i - 1][k]) frozen_out = false;
			if (dt > 0 && dt < 0.001 && frozen_ts && frozen_out) dtsub = true;
			if ((t.us == IN[i - 1].us) && frozen_ts && frozen_out) pause = true;
			if (dt > 1.0 && ts(i) == t.us) dtover = true;
		}
		if (i > 0 && (t.action & 1) && ts(i) == t.us) restart = true;
	}

	struct Cov { const char *name; bool ok; std::string detail; };
	std::vector<Cov> cov = {
		{"pitch saturation (upper)",   phi,        "pitch_sp pinned to pitch_max"},
		{"pitch saturation (lower)",   plo,        "pitch_sp pinned to pitch_min"},
		{"throttle saturation (upper)", thi,       "throttle_sp pinned to throttle_max"},
		{"throttle saturation (lower)", tlo,       "throttle_sp pinned to throttle_min"},
		{"underspeed ratio > 0",       underspeed, "max underspeed_ratio=" + std::to_string(maxund)},
		{"fast descend active",        fastdesc,   "max fast_descend=" + std::to_string(maxfd)},
		{"altitude setpoint step",     altstep,    "|d alt_sp|>50 present"},
		{"airspeed setpoint step",     aspstep,    "|d eas_sp|>5 present"},
		{"integrator reset",           reset,      "min |pitch_i|+|throttle_i| at reset row=" + std::to_string(minreset_i)},
		{"altitude estimator step",    estim,      "alt_ref snapped to estimator altitude"},
		{"dt < 0.001 hold",            dtsub,      "timestamp+outputs frozen"},
		{"dt > 1.0 reinitialize",      dtover,     "timestamp advanced through reinit branch"},
		{"pause (dt == 0) hold",       pause,      "timestamp+outputs frozen"},
		{"restart (fresh controller)", restart,    "controller recreated + reinitialized"},
	};
	int missing = 0;
	std::printf("\n=== TECS guard coverage (behavioral, from PX4-original) ===\n");
	for (auto &c : cov) { std::printf("  [%s] %-28s %s\n", c.ok ? "x" : " ", c.name, c.detail.c_str()); if (!c.ok) missing++; }
	std::printf("coverage_missing=%d\n", missing);

	bool pass = (fails == 0 && ts_fails == 0 && missing == 0);
	std::printf("\nTECS RESULT: %s\n", pass ? "PASS" : "FAIL");
	return pass ? 0 : 1;
}
