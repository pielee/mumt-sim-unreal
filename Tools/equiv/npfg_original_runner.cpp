// npfg_original_runner.cpp — PX4-ORIGINAL side of the NPFG equivalence test.
// Links ONLY the pinned PX4 npfg sources (see build script). No MUMT adapter code.
// Reads the shared input CSV, writes the original-output CSV.
#include <lib/npfg/DirectionalGuidance.hpp>
#include <lib/npfg/AirspeedDirectionController.hpp>
#include <lib/npfg/CourseToAirspeedRefMapper.hpp>
#include "trace_io.h"
#include <cmath>

int main(int argc, char **argv)
{
	if (argc < 3) { std::fprintf(stderr, "usage: %s <in.csv> <out.csv>\n", argv[0]); return 1; }
	auto traces = equiv::readNpfg(argv[1]);
	FILE *f = std::fopen(argv[2], "w");
	if (!f) { std::perror(argv[2]); return 2; }
	std::fprintf(f, "%s\n", equiv::NPFG_OUT_HEADER);

	for (const auto &x : traces) {
		matrix::Vector2f p(x.px, x.py), g(x.gx, x.gy), w(x.wx, x.wy), t(x.tx, x.ty), q(x.qx, x.qy);
		// Fresh guidance objects per trace — identical to the reported reference harness.
		DirectionalGuidance gd; AirspeedDirectionController hc; CourseToAirspeedRefMapper mp;
		const auto d = gd.guideToPath(p, g, w, t, q, x.k);
		const float hdg = mp.mapCourseSetpointToHeadingSetpoint(d.course_setpoint, w, x.sp);
		const float current = atan2f(g(1) - w(1), g(0) - w(0));
		const float fb = hc.controlHeading(hdg, current, x.tas);
		float out[equiv::NPFG_OUT_N] = {
			d.course_setpoint,
			d.lateral_acceleration_feedforward,
			fb,
			fb + d.lateral_acceleration_feedforward,
			gd.getSignedTrackError(),
			gd.getTrackErrorBound(),
			gd.getBearingFeasibility(),
			gd.getAdaptedPeriod(),
			hdg,
			mp.getMinAirspeedForCurrentBearing(d.course_setpoint, w, x.sp, x.min_gs),
		};
		equiv::writeOutRow(f, x.name, out, equiv::NPFG_OUT_N);
		std::fprintf(f, "\n");
	}
	std::fclose(f);
	return 0;
}
