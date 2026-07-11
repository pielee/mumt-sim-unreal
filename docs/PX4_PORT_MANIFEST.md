# PX4 v1.17.0 Port Manifest — Phase 2A-R

Repository: `https://github.com/PX4/PX4-Autopilot`  
Tag: `v1.17.0`  
Commit: `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`  
License: BSD 3-Clause (`ThirdPartyNotices/PX4-BSD-3-Clause.txt`)

The port is isolated from the Unreal runtime and is not connected to UDP, JSBSim commands, an Actor, Shadow, or Active control. All control equations, branches, default parameters, and state transitions below are copied from the pinned source. Changes are limited to namespaces, deterministic time injection, host logging removal, and the compatibility types listed here.

## Source correspondence

| PX4 original | MUMT artifact | Modification |
| --- | --- | --- |
| `src/lib/npfg/DirectionalGuidance.hpp/.cpp` | `Px4NpfgAdapter.h/.cpp` | `MumtPx4` namespace and compatible Vector2/math |
| `src/lib/npfg/AirspeedDirectionController.hpp/.cpp` | `Px4NpfgAdapter.h/.cpp` | namespace only; feedback remains separate from curvature feedforward |
| `src/lib/npfg/CourseToAirspeedRefMapper.hpp/.cpp` | `Px4NpfgAdapter.h/.cpp` | namespace only |
| `src/lib/tecs/TECS.hpp/.cpp` | `Px4TecsAdapter.h/.cpp` | namespace; `hrt_absolute_time()` replaced by injected monotonic clock |
| `src/lib/mathlib/math/filter/AlphaFilter.hpp` | `Px4AlphaFilter.h` | scalar specialization required by TECS |
| `src/lib/motion_planning/VelocitySmoothing.hpp/.cpp` | `Px4VelocitySmoothing.h`, `Px4TecsAdapter.cpp` | namespace and math compatibility |
| `src/lib/motion_planning/ManualVelocitySmoothingZ.hpp/.cpp` | `Px4VelocitySmoothing.h`, `Px4TecsAdapter.cpp` | namespace and math compatibility |
| `matrix::Vector2f`, 2x2 `matrix::Matrix`, `mathlib` subset | `Px4MathAdapter.h` | small host compatibility surface |
| `hrt_abstime`, `hrt_absolute_time()` | `Px4TimeAdapter.h` | deterministic microsecond clock supplied by the caller |

`DirectionalGuidance` returns curvature feedforward only. `FPx4NpfgAdapter` then performs course-to-wind-aware-air-direction mapping, air-direction feedback, summation, and `roll = atan(lateral_acceleration / g)`. Coordinates are North/East, angles radians, positive curvature is right turn, and positive lateral acceleration/roll is right bank. Wind is the inertial wind vector used by `air_velocity = ground_velocity - wind_velocity`; it is never replaced by zero.

TECS retains the original airspeed filter, altitude reference model, energy controller, integrators, limits, underspeed ratio, fast-descend state, and time behavior. It does not add `TECSMode`, explicit UCD state, or explicit climbout state. First update and `dt > 1.0 s` initialize; `dt < 0.001 s` holds.

## Equivalence harness authority

**Authoritative independent harness: `Tools/equiv/**`.** It builds three *separate* executables —
`{npfg,tecs}_original` (pinned PX4 control sources only), `{npfg,tecs}_port` (MUMT adapter only),
and a `compare_*` comparator that links neither — driven by a shared input-trace CSV emitted by
`gen_*_traces`. Each controller runs as its own process and writes its own output CSV; the
comparator reads only those CSVs. The original and port binaries carry **disjoint control-law
symbol sets** (verified with `nm`: e.g. `npfg_original` has PX4 `DirectionalGuidance::` symbols and
0 `MumtPx4::`; `npfg_port` has `MumtPx4::` symbols and 0 global `DirectionalGuidance::`) and
**distinct SHA-256 digests**, so there is no shared control-law object and no COMDAT
symbol-folding path between them.

Results (separate original vs port binaries):
- NPFG — 16 traces, max error 0 across all 10 outputs (tol 1e-6..1e-5); 16/16 coverage.
- TECS — 594 rows, max error 0 across all 13 outputs + exact timestamp (tol 1e-6..1e-4); 14/14
  guard coverage, with the altitude/pitch control loop active.

**TECS altitude/pitch activation (test-only).** The harness sets a non-zero altitude/pitch
configuration (`vertical_accel_limit`, `max_climb/sink_rate`, `min_sink_rate`,
`altitude_error_time_constant`, `integrator_gain_pitch`, `pitch_damping`, `speed_weight`)
*identically* on the original and port sides. These values only exist to drive the pitch law —
PX4 defaults leave the altitude reference model frozen (`vert_accel_limit = 0`) and pitch demand
at 0, so without them the pitch path is never exercised. They are **not** F-16 production tuning
and are never used at runtime.

**Zero/degenerate tangent.** The NPFG `degenerate` trace supplies an actual zero-length
`unit_path_tangent` `(0,0)`, violating PX4's unit-length precondition. `guideToPath` performs no
internal normalization/guard and the MUMT adapter does not normalize either; both sides produce
finite, bit-identical output (`course=lat_ff=lat_fb=lat_total=track_error=feasibility=air_direction
=min_airspeed=0`, `track_error_bound=3.5355`, `adapted_period=10`). The trace proves port≡original
on a precondition-violating input, **not** the presence of a validity guard.

**Deprecated / non-authoritative.** The earlier single-combined-binary harness
(`Tools/verify_{npfg,tecs}_equivalence.cpp`, `Tools/verify_{npfg,tecs}_reference.cpp`,
`Tools/build_verify_{npfg,tecs}_equivalence.sh`) links the PX4 reference and the MUMT port into
one executable. Because both define same-named `matrix::`/`math::` inline/template symbols, that
layout carries a COMDAT symbol-folding risk; it is retained only for historical comparison and is
not the equivalence proof.

**Test-only shims.** `Tools/px4_shims/**` are host-build-only declarations (deterministic
`drv_hrt` time injection, uORB/logging stubs) with no control law. They are compiled into the
equivalence harness only and are never part of the Unreal/production build.

## MUMT state supply audit

| State | Actual source | Source unit | Port unit / frame | Status |
| --- | --- | --- | --- | --- |
| Equivalent airspeed | JSBSim `FGAuxiliary::GetVequivalentKTS()` in `FGAuxiliary.h`; not copied to `FAircraftState` | kt | m/s | read-only API addition required; CAS is not a substitute |
| True airspeed | `FAircraftState::TotalVelocityKts`, assigned from `FGAuxiliary::GetVt()` | kt | m/s | currently present |
| EAS-to-TAS ratio | derivable only when both actual EAS and TAS are valid | ratio | ratio | unavailable until EAS exposure; no substitution |
| Wind North/East | `FGFDMExec::GetWinds()` / `FGWinds`; not copied to `FAircraftState` | ft/s internally | m/s N/E | read-only API addition required; zero wind is not a substitute |
| Altitude ASL | `FAircraftState::AltitudeASLFt` | ft | m | currently present |
| Climb rate | `FAircraftState::AltitudeRateFtps`, from `Propagate::Gethdot()` | ft/s, positive up | m/s, positive up | currently present |
| Pitch / roll | `FAircraftState::LocalEulerAngles.Pitch/Roll` | deg | rad, aircraft attitude | currently present |
| Forward airspeed acceleration | not exposed in `FAircraftState`; TECS caller currently has no validated longitudinal acceleration | — | m/s² forward | currently unavailable; read-only API/design needed |
| Simulation monotonic time | JSBSim `FGFDMExec::GetSimTime()` exists but is not exposed through `FAircraftState` | s | monotonic µs | read-only API addition required |
| Pause / restart | JSBSim `Hold/Resume/ResetToInitialConditions` exists; no formation-state event API | discrete | explicit event plus timestamp discontinuity | read-only/event API addition required |
| Ground velocity N/E | `FAircraftState::VelocityNEDfps.X/Y` | ft/s | m/s N/E | currently present |

The independent adapters accept these values explicitly and the harness verifies supply boundaries without modifying `UDPControlReceiver` or the JSBSim movement component. Runtime wiring is intentionally absent.

## Planner baseline retained

`FFormationCapturePlanner` remains an independent Cubic Hermite draft with clamped reported curvature. Clamping the reported sample does not change the Hermite geometry, so Position/Tangent/Curvature are not guaranteed to describe one curvature-limited curve. `Tools/audit_formation_phase2b_geometry.cpp` remains the regression baseline: 72/72 turning traces fail, the worst observed curvature ratio is 2567.256, minimum turn radius is not met, and Active use is prohibited.
