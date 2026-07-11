// compare_npfg.cpp — Comparator ONLY. Links no controller (neither PX4 nor MUMT).
// Reads the shared input CSV + both output CSVs, checks per-field equivalence
// tolerances, and proves Section 2 trace coverage from the PX4-original output.
// Exit code 0 iff every field is within tolerance AND every required condition
// was actually observed.
#include "trace_io.h"
#include <cmath>
#include <vector>
using namespace equiv;

static float adiff(float a, float b, bool ang)
{
	if (std::isnan(a) && std::isnan(b)) return 0.f;
	if (!std::isfinite(a) || !std::isfinite(b)) return (a == b) ? 0.f : INFINITY;
	float d = a - b;
	if (ang) { while (d > (float)M_PI) d -= 2.f * (float)M_PI; while (d < -(float)M_PI) d += 2.f * (float)M_PI; }
	return std::fabs(d);
}

int main(int argc, char **argv)
{
	if (argc < 4) { std::fprintf(stderr, "usage: %s <in.csv> <orig.csv> <port.csv>\n", argv[0]); return 1; }
	auto IN = readNpfg(argv[1]);
	auto O = readCells(argv[2]);
	auto P = readCells(argv[3]);
	if (O.size() != P.size() || O.size() != IN.size()) {
		std::fprintf(stderr, "row count mismatch: in=%zu orig=%zu port=%zu\n", IN.size(), O.size(), P.size());
		return 3;
	}
	const char *names[NPFG_OUT_N] = {"course", "lat_ff", "lat_fb", "lat_total", "track_error",
	                                 "track_bound", "feasibility", "period", "air_direction", "min_airspeed"};
	const float tol[NPFG_OUT_N] = {1e-6f, 1e-5f, 1e-5f, 1e-5f, 1e-5f, 1e-5f, 1e-6f, 1e-6f, 1e-6f, 1e-5f};
	const bool  ang[NPFG_OUT_N] = {true, false, false, false, false, false, false, false, true, false};

	float mx[NPFG_OUT_N] = {};
	int fails = 0;
	for (size_t i = 0; i < O.size(); i++)
		for (int j = 0; j < NPFG_OUT_N; j++) {
			float a = pf(O[i][j + 1]), b = pf(P[i][j + 1]);
			float e = adiff(a, b, ang[j]);
			if (e > mx[j]) mx[j] = e;
			if (e > tol[j]) { fails++; std::printf("FAIL %-16s %-13s orig=%g port=%g err=%g\n", O[i][0].c_str(), names[j], a, b, e); }
		}

	std::printf("\n=== NPFG equivalence (separate original vs port binaries) ===\n");
	for (int j = 0; j < NPFG_OUT_N; j++) std::printf("MAX %-14s %.9g  (tol %.0e)\n", names[j], mx[j], tol[j]);
	std::printf("rows=%zu equivalence_failures=%d\n", O.size(), fails);

	// ---------- Section 2 coverage from PX4-original output + input geometry ----------
	auto col = [&](size_t i, int j) { return pf(O[i][j + 1]); };
	bool straight = false, rc = false, lc = false, lowgs = false, higs = false;
	bool head = false, tail = false, cross = false, feas = false, degen = false, nan_in = false, inf_in = false;
	float minfeas = 1e9f, maxte = -1e9f, minte = 1e9f, maxcourse = 0.f;
	bool degen_finite = true;
	for (size_t i = 0; i < IN.size(); i++) {
		const auto &t = IN[i];
		float gs = std::sqrt(t.gx * t.gx + t.gy * t.gy);
		float wt = t.wx * t.tx + t.wy * t.ty;      // wind along path tangent
		float wc = t.wx * t.ty - t.wy * t.tx;      // wind across path tangent
		if (std::fabs(t.k) < 1e-9f) straight = true;
		if (t.k > 1e-6f) rc = true;
		if (t.k < -1e-6f) lc = true;
		if (gs < 1.0f) lowgs = true;
		if (gs > 50.f) higs = true;
		if (wt < -5.f && std::fabs(wc) < 5.f) head = true;
		if (wt > 5.f) tail = true;
		if (std::fabs(wc) > 20.f && std::fabs(wt) < 5.f) cross = true;
		if (std::isnan(t.px) || std::isnan(t.py)) nan_in = true;
		if (std::isinf(t.gx) || std::isinf(t.gy)) inf_in = true;
		if (t.tx == 0.f && t.ty == 0.f) {
			degen = true;
			for (int j = 0; j < NPFG_OUT_N; j++) if (!std::isfinite(col(i, j))) degen_finite = false;
		}
		float te = col(i, 4), fe = col(i, 6), cs = col(i, 0);
		if (std::isfinite(te)) { if (te > maxte) maxte = te; if (te < minte) minte = te; }
		if (std::isfinite(fe)) { if (fe < minfeas) minfeas = fe; if (fe > 0.99f) feas = true; }
		if (std::isfinite(cs) && std::fabs(cs) > maxcourse) maxcourse = std::fabs(cs);
	}
	bool ter = (maxte > 0.5f), tel = (minte < -0.5f), infeas = (minfeas < 0.9f), wrap = (maxcourse > 3.0f);

	struct Cov { const char *name; bool ok; std::string detail; };
	std::vector<Cov> cov = {
		{"straight path",        straight, "curvature==0 row present"},
		{"right curve",          rc,       "k>0 present"},
		{"left curve",           lc,       "k<0 present"},
		{"right track error",    ter,      "max signed track_error=" + std::to_string(maxte)},
		{"left track error",     tel,      "min signed track_error=" + std::to_string(minte)},
		{"low ground speed",     lowgs,    "|ground_vel|<1 present"},
		{"high ground speed",    higs,     "|ground_vel|>50 present"},
		{"headwind",             head,     "wind.tangent<-5 present"},
		{"tailwind",             tail,     "wind.tangent>5 present"},
		{"crosswind",            cross,    "|wind x tangent|>20 present"},
		{"feasible wind",        feas,     "feasibility>0.99 present"},
		{"infeasible wind",      infeas,   "min feasibility=" + std::to_string(minfeas)},
		{"course wrap",          wrap,     "max |course|=" + std::to_string(maxcourse)},
		{"degenerate tangent",   degen,    std::string("zero-tangent row present, outputs ") + (degen_finite ? "finite" : "non-finite(matched)")},
		{"NaN input",            nan_in,   "NaN input row present"},
		{"Inf input",            inf_in,   "Inf input row present"},
	};
	int missing = 0;
	std::printf("\n=== NPFG trace coverage (proved from PX4-original output) ===\n");
	for (auto &c : cov) { std::printf("  [%s] %-20s %s\n", c.ok ? "x" : " ", c.name, c.detail.c_str()); if (!c.ok) missing++; }
	std::printf("coverage_missing=%d\n", missing);

	bool pass = (fails == 0 && missing == 0);
	std::printf("\nNPFG RESULT: %s\n", pass ? "PASS" : "FAIL");
	return pass ? 0 : 1;
}
