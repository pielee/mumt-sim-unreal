// =====================================================================
// trace_io.h  —  Phase 2A-R independent equivalence harness (shared IO)
// ---------------------------------------------------------------------
// SHARED CODE SURFACE.  This header is deliberately the *only* source
// file shared between the PX4-original runner and the MUMT-port runner.
// It contains:
//   * plain-float trace record layouts (NO matrix/vector types)
//   * CSV read/write helpers (pure text <-> float)
// It contains NO control law, NO PX4 code, NO MUMT adapter code, and no
// math library.  Each runner converts these plain floats into its own
// vector type and calls its own, independently-compiled controller.
// The two controllers never share an object, a translation unit, or a
// linked binary.
// =====================================================================
#pragma once
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace equiv {

// ---------------------------------------------------------------- NPFG
// One NPFG guidance evaluation.  Fields are plain scalars; the (x,y) pairs
// are North/East components that each runner packs into its own Vector2.
struct NpfgTrace {
	std::string name;
	float px, py;      // position_ne
	float gx, gy;      // ground_velocity_ne
	float wx, wy;      // wind_velocity_ne
	float tx, ty;      // path_tangent_ne
	float qx, qy;      // path_position_ne
	float k;           // path_curvature   (+ = right turn)
	float sp;          // true_airspeed_setpoint
	float tas;         // true_airspeed
	float min_gs;      // minimum_ground_speed
};

static constexpr const char *NPFG_IN_HEADER =
	"name,px,py,gx,gy,wx,wy,tx,ty,qx,qy,k,sp,tas,min_gs";
static constexpr const char *NPFG_OUT_HEADER =
	"name,course,lat_ff,lat_fb,lat_total,track_error,track_bound,feasibility,period,air_direction,min_airspeed";
static constexpr int NPFG_OUT_N = 10;

// ---------------------------------------------------------------- TECS
struct TecsTrace {
	std::string name;
	uint64_t us;       // injected monotonic timestamp (microseconds)
	float pitch;
	float alt;
	float alt_sp;
	float eas_sp;
	float eas;
	float ratio;       // eas_to_tas
	float tmin, tmax, ttrim;
	float pmin, pmax;
	float climb, sink; // target climb / sink rate
	float accel;       // forward_airspeed_acceleration
	float hgt_rate;
	float hgt_rate_sp;
	int   action;      // bit0=recreate/restart, bit1=resetIntegrals, bit2=handle_alt_step
};

static constexpr const char *TECS_IN_HEADER =
	"name,us,pitch,alt,alt_sp,eas_sp,eas,ratio,tmin,tmax,ttrim,pmin,pmax,climb,sink,accel,hgt_rate,hgt_rate_sp,action";
// 13 float columns then one integer timestamp column (compared exactly).
static constexpr const char *TECS_OUT_HEADER =
	"name,pitch_sp,throttle_sp,underspeed,filtered_tas,tas_rate,te_rate_sp,te_rate_est,eb_rate_sp,eb_rate_est,pitch_i,throttle_i,fast_descend,alt_ref,timestamp_us";
static constexpr int TECS_OUT_N = 13; // float columns (timestamp handled separately)

// -------------------------------------------------------- text parsing
inline std::vector<std::string> splitCsv(const std::string &line)
{
	std::vector<std::string> out;
	std::string cur;
	for (char c : line) {
		if (c == ',') { out.push_back(cur); cur.clear(); }
		else if (c != '\r' && c != '\n') { cur.push_back(c); }
	}
	out.push_back(cur);
	return out;
}
inline float    pf(const std::string &s) { return std::strtof(s.c_str(), nullptr); }  // handles nan/inf/-inf
inline uint64_t pu(const std::string &s) { return std::strtoull(s.c_str(), nullptr, 10); }
inline int      pi(const std::string &s) { return (int)std::strtol(s.c_str(), nullptr, 10); }

// -------------------------------------------------------- NPFG file IO
inline void writeNpfgRow(FILE *f, const NpfgTrace &t)
{
	std::fprintf(f, "%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
	             t.name.c_str(), t.px, t.py, t.gx, t.gy, t.wx, t.wy, t.tx, t.ty,
	             t.qx, t.qy, t.k, t.sp, t.tas, t.min_gs);
}
inline std::vector<NpfgTrace> readNpfg(const char *path)
{
	std::vector<NpfgTrace> v;
	FILE *f = std::fopen(path, "r");
	if (!f) { std::perror(path); std::exit(2); }
	char *line = nullptr; size_t n = 0; bool header = true;
	while (getline(&line, &n, f) != -1) {
		std::string L(line);
		if (header) { header = false; continue; }
		auto c = splitCsv(L);
		if (c.size() < 15) continue;
		NpfgTrace t; t.name = c[0];
		t.px = pf(c[1]);  t.py = pf(c[2]);  t.gx = pf(c[3]);  t.gy = pf(c[4]);
		t.wx = pf(c[5]);  t.wy = pf(c[6]);  t.tx = pf(c[7]);  t.ty = pf(c[8]);
		t.qx = pf(c[9]);  t.qy = pf(c[10]); t.k  = pf(c[11]); t.sp = pf(c[12]);
		t.tas = pf(c[13]); t.min_gs = pf(c[14]);
		v.push_back(t);
	}
	std::free(line); std::fclose(f);
	return v;
}

// -------------------------------------------------------- TECS file IO
inline void writeTecsRow(FILE *f, const TecsTrace &t)
{
	std::fprintf(f, "%s,%llu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d\n",
	             t.name.c_str(), (unsigned long long)t.us, t.pitch, t.alt, t.alt_sp, t.eas_sp, t.eas, t.ratio,
	             t.tmin, t.tmax, t.ttrim, t.pmin, t.pmax, t.climb, t.sink, t.accel, t.hgt_rate, t.hgt_rate_sp, t.action);
}
inline std::vector<TecsTrace> readTecs(const char *path)
{
	std::vector<TecsTrace> v;
	FILE *f = std::fopen(path, "r");
	if (!f) { std::perror(path); std::exit(2); }
	char *line = nullptr; size_t n = 0; bool header = true;
	while (getline(&line, &n, f) != -1) {
		std::string L(line);
		if (header) { header = false; continue; }
		auto c = splitCsv(L);
		if (c.size() < 19) continue;
		TecsTrace t; t.name = c[0]; t.us = pu(c[1]);
		t.pitch = pf(c[2]);  t.alt = pf(c[3]);   t.alt_sp = pf(c[4]);  t.eas_sp = pf(c[5]);
		t.eas = pf(c[6]);    t.ratio = pf(c[7]); t.tmin = pf(c[8]);    t.tmax = pf(c[9]);
		t.ttrim = pf(c[10]); t.pmin = pf(c[11]); t.pmax = pf(c[12]);   t.climb = pf(c[13]);
		t.sink = pf(c[14]);  t.accel = pf(c[15]); t.hgt_rate = pf(c[16]); t.hgt_rate_sp = pf(c[17]);
		t.action = pi(c[18]);
		v.push_back(t);
	}
	std::free(line); std::fclose(f);
	return v;
}

// generic output-CSV reader: returns raw cells per data row (header skipped).
inline std::vector<std::vector<std::string>> readCells(const char *path)
{
	std::vector<std::vector<std::string>> rows;
	FILE *f = std::fopen(path, "r");
	if (!f) { std::perror(path); std::exit(2); }
	char *line = nullptr; size_t n = 0; bool header = true;
	while (getline(&line, &n, f) != -1) {
		std::string L(line);
		if (header) { header = false; continue; }
		if (L.size() < 2) continue;
		rows.push_back(splitCsv(L));
	}
	std::free(line); std::fclose(f);
	return rows;
}

// -------------------------------------------------------- output rows
// name + nc float columns, %.9g gives exact round-trip for float32.
inline void writeOutRow(FILE *f, const std::string &name, const float *vals, int nc)
{
	std::fprintf(f, "%s", name.c_str());
	for (int i = 0; i < nc; i++) std::fprintf(f, ",%.9g", vals[i]);
}

} // namespace equiv
