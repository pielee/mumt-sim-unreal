// tecs_port_runner.cpp — MUMT-PORT side of the TECS equivalence test.
// Links ONLY the MUMT adapter + compatibility layer (see build script). No PX4 source.
// Time is injected through the port's own Px4MonotonicClock via TecsInput.timestamp_us.
// Reads the shared input CSV, writes the port-output CSV. Stateful across rows.
#include "FormationControl/Px4TecsAdapter.h"
#include "trace_io.h"
#include <memory>

static std::unique_ptr<MumtPx4::FPx4TecsAdapter> makePort()
{
	auto p = std::make_unique<MumtPx4::FPx4TecsAdapter>();
	p->controller().enable_airspeed(true);
	p->controller().set_equivalent_airspeed_min(40.f);
	p->controller().set_equivalent_airspeed_max(60.f);
	p->controller().set_fast_descend_altitude_error(20.f);
	p->controller().set_detect_underspeed_enabled(true);
	// Altitude/pitch loop configuration (identical to the original-side runner).
	p->controller().set_vertical_accel_limit(7.f);
	p->controller().set_max_climb_rate(8.f);
	p->controller().set_max_sink_rate(8.f);
	p->controller().set_min_sink_rate(2.f);
	p->controller().set_altitude_error_time_constant(2.f);
	p->controller().set_integrator_gain_pitch(0.15f);
	p->controller().set_pitch_damping(0.1f);
	p->controller().set_speed_weight(1.f);
	return p;
}

int main(int argc, char **argv)
{
	if (argc < 3) { std::fprintf(stderr, "usage: %s <in.csv> <out.csv>\n", argv[0]); return 1; }
	auto traces = equiv::readTecs(argv[1]);
	FILE *f = std::fopen(argv[2], "w");
	if (!f) { std::perror(argv[2]); return 2; }
	std::fprintf(f, "%s\n", equiv::TECS_OUT_HEADER);

	auto port = makePort();
	for (const auto &x : traces) {
		if (x.action & 1) port = makePort();
		if (x.action & 2) port->controller().resetIntegrals();
		if (x.action & 4) port->controller().handle_alt_step(x.alt, x.hgt_rate);
		MumtPx4::TecsInput in{
			x.us, x.pitch, x.alt, x.alt_sp, x.eas_sp, x.eas, x.ratio, x.tmin, x.tmax, x.ttrim,
			x.pmin, x.pmax, x.climb, x.sink, x.accel, x.hgt_rate, x.hgt_rate_sp
		};
		const auto o = port->update(in);
		float out[equiv::TECS_OUT_N] = {
			o.pitch_setpoint, o.throttle_setpoint, o.underspeed_ratio,
			o.filtered_tas, o.filtered_tas_rate,
			o.total_energy_rate_sp, o.total_energy_rate_estimate,
			o.energy_balance_rate_sp, o.energy_balance_rate_estimate,
			o.pitch_integrator, o.throttle_integrator, o.fast_descend,
			port->controller().getStatus().altitude_reference,
		};
		equiv::writeOutRow(f, x.name, out, equiv::TECS_OUT_N);
		std::fprintf(f, ",%llu\n", (unsigned long long)o.state_timestamp_us);
	}
	std::fclose(f);
	return 0;
}
