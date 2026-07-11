// tecs_original_runner.cpp — PX4-ORIGINAL side of the TECS equivalence test.
// Links ONLY the pinned PX4 TECS + motion_planning sources (see build script).
// Time is injected through the px4_shims drv_hrt.h shim (px4_equivalence_time_us).
// Reads the shared input CSV, writes the original-output CSV. Stateful across rows.
#include <lib/tecs/TECS.hpp>
#include "trace_io.h"
#include <cstdint>

// Definition consumed by Tools/px4_shims/drivers/drv_hrt.h (hrt_absolute_time()).
uint64_t px4_equivalence_time_us = 0;

int main(int argc, char **argv)
{
	if (argc < 3) { std::fprintf(stderr, "usage: %s <in.csv> <out.csv>\n", argv[0]); return 1; }
	auto traces = equiv::readTecs(argv[1]);
	FILE *f = std::fopen(argv[2], "w");
	if (!f) { std::perror(argv[2]); return 2; }
	std::fprintf(f, "%s\n", equiv::TECS_OUT_HEADER);

	TECS *c = nullptr;
	for (const auto &x : traces) {
		if (!c || (x.action & 1)) {
			delete c;
			c = new TECS();
			c->enable_airspeed(true);
			c->set_equivalent_airspeed_min(40.f);
			c->set_equivalent_airspeed_max(60.f);
			c->set_fast_descend_altitude_error(20.f);
			c->set_detect_underspeed_enabled(true);
			// Altitude/pitch loop configuration (identical on both sides). Without a
			// non-zero vertical accel limit + altitude error gain the reference model is
			// frozen and pitch demand is trivially 0, so these exercise the pitch law.
			c->set_vertical_accel_limit(7.f);
			c->set_max_climb_rate(8.f);
			c->set_max_sink_rate(8.f);
			c->set_min_sink_rate(2.f);
			c->set_altitude_error_time_constant(2.f);
			c->set_integrator_gain_pitch(0.15f);
			c->set_pitch_damping(0.1f);
			c->set_speed_weight(1.f);
		}
		if (x.action & 2) c->resetIntegrals();
		if (x.action & 4) c->handle_alt_step(x.alt, x.hgt_rate);
		px4_equivalence_time_us = x.us;
		c->update(x.pitch, x.alt, x.alt_sp, x.eas_sp, x.eas, x.ratio, x.tmin, x.tmax, x.ttrim,
		          x.pmin, x.pmax, x.climb, x.sink, x.accel, x.hgt_rate, x.hgt_rate_sp);
		const auto &d = c->getStatus();
		float out[equiv::TECS_OUT_N] = {
			c->get_pitch_setpoint(), c->get_throttle_setpoint(), c->get_underspeed_ratio(),
			d.true_airspeed_filtered, d.true_airspeed_derivative,
			d.control.total_energy_rate_sp, d.control.total_energy_rate_estimate,
			d.control.energy_balance_rate_sp, d.control.energy_balance_rate_estimate,
			d.control.pitch_integrator, d.control.throttle_integrator, d.fast_descend,
			d.altitude_reference,
		};
		equiv::writeOutRow(f, x.name, out, equiv::TECS_OUT_N);
		std::fprintf(f, ",%llu\n", (unsigned long long)c->timestamp());
	}
	delete c;
	std::fclose(f);
	return 0;
}
