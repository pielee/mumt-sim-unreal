// gen_npfg_traces.cpp — emits the shared NPFG input CSV (data only, no control law).
// Usage: gen_npfg_traces <out.csv>
// Coverage target (Section 2): straight, right/left curve, right/left track error,
// low/high ground speed, headwind, tailwind, crosswind, feasible & infeasible wind,
// course wrap, degenerate tangent, NaN, Inf.
#include "trace_io.h"
#include <limits>

int main(int argc, char **argv)
{
	if (argc < 2) { std::fprintf(stderr, "usage: %s <out.csv>\n", argv[0]); return 1; }
	const float N = std::numeric_limits<float>::quiet_NaN();
	const float I = std::numeric_limits<float>::infinity();
	using equiv::NpfgTrace;

	// name           px  py    gx    gy    wx   wy    tx    ty        qx qy  k       sp  tas min_gs
	std::vector<NpfgTrace> v = {
		{"straight",         0,10,  40,   0,    0,   0,    1,   0,       0, 0,  0.0f,   40, 40, 5},
		{"right_curve",      0,50,  45,   2,    4,  -3,    1,   0,       0, 0,  0.003f, 45, 41, 5},
		{"left_curve",       0,-50, 45,  -2,   -4,   3,    1,   0,       0, 0, -0.003f, 45, 41, 5},
		{"track_err_right",  0,5,   40,   0,    0,   0,    1,   0,       0, 0,  0.0f,   40, 40, 5}, // 5 E of N path
		{"track_err_left",   0,-5,  40,   0,    0,   0,    1,   0,       0, 0,  0.0f,   40, 40, 5}, // 5 W of N path
		{"low_speed",        20,5,  0.01f,0.01f,8,   0,    0,   1,       0, 0,  0.0f,   25, 8,  5},
		{"high_speed",       0,10,  80,   0,    0,   0,    1,   0,       0, 0,  0.0f,   60, 80, 5},
		{"headwind",         5,20,  30,   0,   -28,  0,    1,   0,       0, 0,  0.0f,   35, 58, 5},
		{"tailwind",        -5,-20, 30,   0,    28,  0,    1,   0,       0, 0,  0.0f,   35, 2,  5},
		{"crosswind",        30,5,  20,   0,    0,   35,   1,   0,       0, 0,  0.0f,   25, 40, 5},
		{"feasible_wind",    0,10,  40,   0,    2,   1,    1,   0,       0, 0,  0.0f,   40, 40, 5}, // |w|~2 << tas
		{"infeasible_wind",  0,8,   15,   3,    0,   40,   1,   0,       0, 0,  0.0f,   20, 15, 5}, // |w|40 > tas15
		{"course_wrap",      10,1, -30,  0.01f, 0,   0,   -1,  -1e-5f,   0, 0,  0.0f,   30, 30, 5}, // target ~behind
		{"degenerate",       0,0,   0,    0,    0,   0,    0,   0,       0, 0,  0.0f,   20, 0,  0}, // zero tangent
		{"nan_input",        N,0,   20,   0,    0,   0,    1,   0,       0, 0,  0.0f,   20, 20, 5},
		{"inf_input",        0,0,   I,    0,    0,   0,    1,   0,       0, 0,  0.0f,   20, 20, 5},
	};

	FILE *f = std::fopen(argv[1], "w");
	if (!f) { std::perror(argv[1]); return 2; }
	std::fprintf(f, "%s\n", equiv::NPFG_IN_HEADER);
	for (const auto &t : v) equiv::writeNpfgRow(f, t);
	std::fclose(f);
	std::fprintf(stderr, "wrote %zu NPFG traces to %s\n", v.size(), argv[1]);
	return 0;
}
