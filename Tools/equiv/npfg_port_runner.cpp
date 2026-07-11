// npfg_port_runner.cpp — MUMT-PORT side of the NPFG equivalence test.
// Links ONLY the MUMT adapter + compatibility layer (see build script). No PX4 source.
// Reads the shared input CSV, writes the port-output CSV.
#include "FormationControl/Px4NpfgAdapter.h"
#include "trace_io.h"

int main(int argc, char **argv)
{
	if (argc < 3) { std::fprintf(stderr, "usage: %s <in.csv> <out.csv>\n", argv[0]); return 1; }
	auto traces = equiv::readNpfg(argv[1]);
	FILE *f = std::fopen(argv[2], "w");
	if (!f) { std::perror(argv[2]); return 2; }
	std::fprintf(f, "%s\n", equiv::NPFG_OUT_HEADER);

	for (const auto &x : traces) {
		// Fresh adapter per trace — identical to the reported port harness.
		MumtPx4::FPx4NpfgAdapter port;
		MumtPx4::NpfgInput in{
			matrix::Vector2f{x.px, x.py}, matrix::Vector2f{x.gx, x.gy}, matrix::Vector2f{x.wx, x.wy},
			matrix::Vector2f{x.tx, x.ty}, matrix::Vector2f{x.qx, x.qy},
			x.k, x.sp, x.tas, x.min_gs
		};
		const auto o = port.update(in);
		float out[equiv::NPFG_OUT_N] = {
			o.course_setpoint,
			o.lateral_acceleration_feedforward,
			o.lateral_acceleration_feedback,
			o.lateral_acceleration_total,
			o.track_error,
			o.track_error_bound,
			o.wind_feasibility,
			o.adapted_period,
			o.airspeed_direction,
			o.minimum_required_airspeed,
		};
		equiv::writeOutRow(f, x.name, out, equiv::NPFG_OUT_N);
		std::fprintf(f, "\n");
	}
	std::fclose(f);
	return 0;
}
