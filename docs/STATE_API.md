# Read-only JSBSim State API (Phase 2A-R, Section 4–6)

Supplies the NPFG/TECS controllers with aircraft state in SI units / local NED frame **without
letting a controller touch the protected JSBSim objects** (`FGFDMExec`, `FGAuxiliary`,
`FGWinds`, `FGPropagate`, `FGAccelerations`) owned by `UJSBSimMovementComponent`.

## Runtime ownership analysis (why a subclass does NOT work)

Investigated against the actual code:

| Question | Finding |
| -- | -- |
| Class of the F-16's JSBSim component | F-16 is **Blueprint-only** (no C++ actor in `Source/`); the Blueprint adds the plugin's spawnable **base `UJSBSimMovementComponent`**. |
| How consumers obtain it | Always `FindComponentByClass<UJSBSimMovementComponent>()` (`HealthComponent.cpp:15`, `UDPControlReceiver.cpp:1504`), cached as the **base** type (`HealthComponent.h:84`). |
| Subclass ⇒ asset change? | **Yes** — the Blueprint would have to add the subclass (Actor Asset change), not allowed. |
| Two FDM instances | Each component builds its own `FGFDMExec` in `InitializeJSBSim()`; a second component ⇒ a second FDM (forbidden). |
| `Cast<subclass>(existing)` | **Fails (nullptr)** — the live instance is the base class. |

**Conclusion: the subclass design was discarded** (its files, never committed, were removed). The
CommandConsole string-property path was also rejected (string round-trip, name fragility).

## Architecture (implemented)

```
existing UJSBSimMovementComponent  (found via FindComponentByClass — base class, no cast)
  → bool GetJsbFlightSnapshot(FJsbFlightSnapshot& Out) const   [minimal read-only plugin getter]
  → MumtState::ConvertJsbToControlState(snapshot, tracker)     [pure templated adapter, host-tested]
  → FMumtControlState                                          (SI/NED, consumed by the controller)
```

- **Plugin getter (2 files changed):** `FJsbFlightSnapshot` (plain POD, raw JSBSim units, no
  Unreal reflection, no control law) + `bool GetJsbFlightSnapshot(FJsbFlightSnapshot&) const`.
  It reads the protected `Auxiliary/Winds/Propagate/Exec` and returns `false` (leaving
  `bValidFrame=false`) if the FDM is not initialized. No new component and no new `FGFDMExec` are
  created; the existing component and its single FDM are used.
- **Adapter:** `MumtState::ConvertJsbToControlState` is **templated on the snapshot type**, so the
  plugin snapshot feeds it directly. All unit/frame/validity/event logic lives in the
  dependency-free `Source/MUMT_Sim/Public/State/MumtControlState.h` (no Unreal, no JSBSim), which is
  host-compilable and unit-tested. `MumtState::FJsbRawState` is the host-testable reference layout
  that `FJsbFlightSnapshot` mirrors field-for-field.
- A separate read-only struct is used **instead of extending `FAircraftState`**, so there is zero
  ABI/Blueprint-serialization impact on the existing USTRUCT.

**Thread affinity:** `GetJsbFlightSnapshot` must be called on the **Game Thread in sync with the
JSBSim tick** (the thread that runs `TickComponent`/`CopyFromJSBSim`). It reads live FDM state and
is not thread-safe against a concurrent `Run()`/reset. Documented on the getter and here.

## Snapshot fields (raw JSBSim units, no conversion)

| `FJsbFlightSnapshot` field | JSBSim source | Unit |
| -- | -- | -- |
| `VequivalentKTS` | `Auxiliary->GetVequivalentKTS()` | knots (EAS, **not** CAS) |
| `VtFps` | `Auxiliary->GetVt()` | ft/s (TAS) |
| `VcalibratedKTS` | `Auxiliary->GetVcalibratedKTS()` | knots (CAS, reference only) |
| `WindNorthFps` / `WindEastFps` | `Winds->GetTotalWindNED(eNorth/eEast)` | ft/s, local NED |
| `AltAslFt` | `Propagate->GetAltitudeASL()` | ft, +up |
| `HdotFps` | `Propagate->Gethdot()` | ft/s, +up |
| `PitchRad` / `RollRad` | `Propagate->GetEuler(eTht/ePhi)` | rad |
| `SimTimeSec` | `Exec->GetSimTime()` | s, monotonic |
| `bHolding` | `Exec->Holding()` | bool (paused) |
| `bValidFrame` | (getter sets it) | true iff FDM objects ready |

## Adapter output: SI/NED conversion, coordinate frame, validity

| State | From | API unit / frame | Validity rule |
| -- | -- | -- | -- |
| Equivalent airspeed | `VequivalentKTS` × knot→m/s | m/s | finite & ≥0 |
| True airspeed | `VtFps` × ft→m | m/s | finite & ≥0 |
| eas_to_tas (=TAS/EAS) | TAS / EAS | ratio | EAS&TAS valid, **EAS ≥ 1 m/s**, ratio finite & >0; else invalid, fallback 1.0 |
| Wind North / East | `Wind*Fps` × ft→m | m/s, +N / +E (local NED) | finite |
| Altitude ASL | `AltAslFt` × ft→m | m, +up | finite |
| Climb rate | `HdotFps` × ft→m | m/s, +up | finite |
| Pitch / Roll | passthrough | rad | finite |
| **TAS rate** | `d(TAS)/dt` | m/s² | dt>0, running, not first/pause/reset |
| **Forward acceleration** (body-X) | — | m/s² | **not supplied — always invalid** |
| Simulation time | `SimTimeSec` | s + µs (`SimTimeMicros`) | finite & ≥0 |
| Pause | `bHolding` | `bPaused` | — |
| Resume event | `Holding` true→false | `bResumeEvent` | — |
| Reset event | sim time decreases | `bResetEvent` + `ResetGeneration` | heuristic only (see caveats) |

Constraints honoured: **EAS is `GetVequivalentKTS`, never CAS**; **wind is the JSBSim total wind,
never forced to zero**; the `eas_to_tas` (= TAS/EAS) division is guarded on EAS; sim time is JSBSim
sim time, not wall clock; the controller reads only the snapshot/`FMumtControlState`, never the
protected objects.

`EasToTasRatio` is the **`eas_to_tas` input to TECS**: PX4 uses `TAS = EAS × eas_to_tas`, so this
field is `TAS / EAS` (≈ 1.777 at EAS 51.4444 / TAS 91.44), **not** EAS/TAS. When the TECS adapter is
wired, this value connects directly to the TECS `eas_to_tas` argument.

## Forward acceleration / PX4 `speed_deriv_forward` (verified against v1.17.0)

- Inside TECS, `speed_deriv_forward` is used **only** as the airspeed-filter rate measurement:
  `equivalent_airspeed_rate = speed_deriv_forward / eas_to_tas` (`TECS.cpp:743`).
- The v1.17.0 caller `FwLateralLongitudinalControl::tecs_update_pitch_throttle` passes **0**:
  `const float airspeed_rate_estimate = 0.f;`, guarded by a HOTFIX — *"the airspeed rate estimate
  using acceleration in body-forward direction has shown to lead to high biases"*
  (`FwLateralLongitudinalControl.cpp:383–387`).
- Therefore the API **claims no value as `speed_deriv_forward`**:
  - `TASRateMps2` / `bTASRateValid` — unfiltered `d(TAS)/dt`, **diagnostic only**.
  - `ForwardAccelerationMps2` / `bForwardAccelerationValid` — body-X forward acceleration,
    **not supplied** (kept invalid); supplying it would reintroduce the bias PX4 removed.
- A v1.17.0-faithful TECS integration passes **0** for `speed_deriv_forward`.

## Zero-length path tangent — Invalid handling (DESIGN ONLY, not connected)

PX4 `DirectionalGuidance::guideToPath` takes `unit_path_tangent` as a unit-length precondition with
no internal guard (established in the NPFG degenerate audit). The **upper guidance call boundary**
must reject `‖tangent‖ < eps` as Invalid (skip/hold the guidance update; do not call `guideToPath`)
and otherwise normalise. Not connected to the planner or NPFG this stage.

## Compile boundary (this stage)

| Item | Status |
| -- | -- |
| Pure adapter + 17 unit tests | **host-compiled & passing** (`Tools/state_api/build_verify_state_api.sh`) |
| Getter JSBSim usage + snapshot→adapter chain | **host `-fsyntax-only` OK** against pinned JSBSim headers (`probe_jsbsim_getters.cpp`) |
| Plugin getter (`UJSBSimMovementComponent`) full compile/link | **not built here** — no Unreal Engine in this environment |

Probe target = the single canonical JSBSim header set the plugin compiles against
(`.../ThirdParty/JSBSim/Include`, exposed via `PublicSystemIncludePaths`; only one `FGAuxiliary.h`).
The numeric JSBSim version is `FGJSBBase::JSBSim_version` in the linked library
(`Lib/JSBSim.lib` Win64 + `Lib/Linux/libJSBSim.a` Linux both shipped), not a header literal.
Full editor build (engine-installed machine):

```
"<UE_5.4>/Engine/Build/BatchFiles/Linux/Build.sh" MUMT_SimEditor Linux Development \
    -project="<abs>/MUMT_Sim/MUMT_Sim.uproject" -waitmutex
```

The adapter/getter need no `MUMT_Sim.Build.cs` change (JSBSimFlightDynamicsModel is already a public
dependency). The dev/editor-only live-snapshot PIE test (`MumtLiveSnapshotTest.cpp`) adds an
editor-only `if (Target.bBuildEditor) UnrealEd` dependency for the PIE latent commands; it is
excluded from non-editor (game/shipping) builds.

## Verification status

**Verified:**
- **Full UE 5.4.4 (Linux, Development editor) build** — plugin getter + module compile & link, UHT OK.
- **Adapter UBT compile** — `MumtControlState.h` compiled and `FJsbFlightSnapshot -> ConvertJsbToControlState
  -> FMumtControlState` instantiated in-module (automation test `MUMT.StateApi.AdapterCompiles`, Result=Success).
- **Live PIE (RL_2, existing F-16, post-FDM-init success path)** — existing-component reuse (1 per
  actor, no new component/FGFDMExec), `GetJsbFlightSnapshot` on the Game Thread, snapshot -> adapter ->
  `FMumtControlState`, property-tree cross-check (EAS/TAS/Alt/Climb/Pitch/Roll/SimTime raw == property to
  ~1e-10; SI == independent calc), SimTime monotonic + microseconds, TASRate (first invalid then valid),
  ForwardAcceleration invalid/0, Game-Thread affinity, `eas_to_tas = TAS/EAS` valid path (962 samples).

Automation verdict vs process exit (recorded separately): the automation **Result = Success**
(1 test performed, *Automation Test Queue Empty*); the `UnrealEditor-Cmd` **process exit code was 1** —
this is not asserted to be normal and is reported independently of the Success verdict.

**Not verified (kept explicit):**
- **Pre-init / null-object getter FAILURE path** — did not occur live (the FDM was valid by the first
  sample); covered only by the getter's `Out = FJsbFlightSnapshot{}` reset + host tests.
- **Stale-clear on a live failure** — proven by code + host tests, not by a live failure path.
- **Pause / Resume events** — not exercised (the read-only scenario cannot call `Exec->Hold()`;
  world-pause would not set `bHolding`; forcing it is out of scope).
- **Reset lifecycle / `ResetGeneration`** — the sim-time-backwards heuristic is not authoritative; an
  explicit RunIC/Respawn/Reset-generation hook is still required as follow-up. Not verified.
- **Non-zero Wind blowing-toward sign** — the live scenario is zero-wind; sign remains unverified
  (wind was not forced).
- **High-speed flight dynamics** — the live scenario was near-stationary (ground); high-dynamic values
  were not reached.
