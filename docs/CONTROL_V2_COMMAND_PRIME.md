# Prime — the bumpless Legacy → Formation handoff (Phase B)

Turning a controller on is easy. Turning it on **without the aircraft twitching** is the hard part, and
it is the only thing this phase is about.

## What the handoff must be continuous with

Not the legacy writer block. That block is an *input*: the [ownership
measurements](CONTROL_V2_COMMAND_OWNERSHIP.md) showed it can be a field-wise **mixture** of several
writers (aileron/elevator/throttle from the manual path while `SpeedBrake` still carries the autopilot's
value), and it keeps changing underneath you while you look at it.

The only thing the aircraft is genuinely flying is the **final resolved block the FDM last consumed** —
the output of the [Phase A arbiter](CONTROL_V2_COMMAND_ARBITER.md). That is the baseline, and everything
here is anchored to it.

```
Legacy writers -> legacy block -> resolver -> RESOLVED BLOCK
                                                    |
                                                    v
                                            prime snapshot (29 fields)
                                                    |
                                    prime the stateful controllers
                                                    |
                                            first candidate
                                                    |
                                      validate against the ticket
                                                    |
                                    only then: activate Formation
```

## The snapshot is all 29 fields

Formation overwrites five fields today (aileron, elevator, rudder, throttle, speedbrake). The snapshot
keeps **all 15 flight-control fields and all 14 per-engine fields** anyway.

The Blueprints' trims, flaps and brakes and the engine state are part of what the aircraft is flying. A
snapshot that dropped them would become a *second source of truth* the moment anything else started
using it — which is precisely the failure this whole effort exists to remove.

**`FuelFreeze` is deliberately NOT in the snapshot.** It is a global on the movement component
(`Propulsion->SetFuelFreeze`) and has never travelled through the command block. Including it would
invent a command path that does not exist.

## Whose command is it? The snapshot carries its own identity

A consume sequence is **not an identity**. Two aircraft can sit on the same one, and a snapshot that knows
only "sequence 4242" would happily prime an aircraft against a block a *different* aircraft, in a
*different* world, actually flew.

So every snapshot carries the component and the world it came from — taken from the component the
**resolver was invoked with**, never from a caller's claim — and `RequestPrime` refuses
(`IdentityMismatch`) unless both match the aircraft asking. The test-only injection seam cannot forge an
owner either: it overwrites the identity from the target component, so it can supply values but never
authorship.

## The ticket: generation + baseline consume sequence

A candidate alone proves nothing. A candidate from a *superseded* prime — anchored to a command the
aircraft stopped flying seconds ago — looks identical to a fresh one, and activating on it steps.

So every candidate must carry:

- **`PrimeGeneration`** — strictly monotonic **per aircraft**, never reused. Issuing a new prime
  immediately invalidates the previous ticket *and any candidate riding on it*.
- **`BaselineConsumeSequence`** — which FDM consume the baseline came from.

Both must match the current ticket, or the candidate is refused and the aircraft stays on Legacy.

## Freshness is asked twice, and the second answer is the one that counts

A candidate is checked for staleness when it is submitted. That check is about the **past**. The handoff
happens later, at a consume boundary, and by then the producer may have died, stalled, or simply been
overtaken by the aircraft's own motion.

So the boundary asks again, and the two refusals it can give are **different failures** and are counted
separately — a gate that accepted either would not know which one it had proved:

| At the consume boundary | Means | Counter |
| --- | --- | --- |
| **`InterveningConsume`** | a Legacy consume slipped in after the prime: the *baseline* moved on | `intervening_consume_rejected` |
| **`StaleCandidate`** | the *candidate* rotted in place between submission and the handoff | `activation_stale_rejected` |

Either way the ticket and its candidate are destroyed, the mode stays `LegacyOrManual`, and only a **new**
prime — against the command the aircraft is actually flying now — may proceed.

`ActivateFormationForTesting()` therefore only **asks**. It sets `ActivationPending`; the switch itself
happens in `Resolve()`, at the consume boundary, because that is the only place either failure is visible.
Flipping the mode inside `Activate()` could not see an intervening consume at all: it happens after the
call returns.

## Game thread only

The registry, the resolver binding and the prime state have **no synchronisation whatsoever**, and none is
claimed. Every mutating entry point — `Resolve`, `OnResolved`, `RequestPrime`, `SubmitPrimedCandidate`,
`ActivateFormationForTesting`, `ResetSession`, `SetEnabled`, world cleanup, the dead-component purge and
every test-only seam — asserts `IsInGameThread()`. A contract that is only written in a comment is not a
contract; this one fails loudly the first time it is broken.

## Priming stateful controllers — and only stateful ones

The audit drove this, not a guess.

| Component | State | Prime API | Why |
| --- | --- | --- | --- |
| **`F16StickAdapterV2`** | `PitchIntegrator`, `PrevAileron/Elevator/Rudder/Throttle`, `bHavePrevCommands` | **`PrimeFromResolvedCommand()`** | has both a slew memory and an integrator |
| **TECS** | `_pitch_integ_state`, `_throttle_integ_state`, previous setpoints, filter | **none added** | PX4 already self-initialises — see below |
| **NPFG** | `feas_`, `track_proximity_`, `course_sp_`, `lateral_accel_ff_` | **none added** | stateless: every field is recomputed from the current inputs each update. No integrator, no previous-output dependency. |
| **`FormationPlannerV2`** | `ActivePath`, `ProgressS`, `GuardDwell`, held-path age | **none added** | path state, not *command* state — it holds nothing the command must be continuous with |

### The stick: slew memory alone is not enough

`Reset()` zeroes everything, so the first command after a reset starts at 0 and the slew limiter drags it
toward the target — a visible step, merely made gradual.

`PrimeFromResolvedCommand()` **validates first and mutates second**, and returns `false` having changed
*nothing* if the prime is not usable — a half-applied prime is worse than none, because the slew anchors
would move to a command nobody validated and the latch would promise a baseline nobody checked. Refused:
generation or consume sequence 0, a non-finite or out-of-range baseline, a non-finite body pitch rate, and
an unusable **config** (non-finite damping gain or integrator limit, a negative integrator limit, an
inverted elevator/throttle range). The config is not decoration: the integrator seed is computed *from* it
and the baseline is range-checked *against* it, so validating the baseline against an unchecked config
would be validating nothing.

Otherwise it seeds both halves:

```
PrevAileron/Elevator/Rudder/Throttle  <- the resolved command   (the slew limiter cannot jump)
PitchIntegrator                       <- D*q - elevator         (the loop REPRODUCES that elevator)
```

The second line matters. Update computes `elevator = -(Kp*err + I) + D*q`; at the handoff the reference
is the current attitude, so `err = 0` and therefore `I = D*q - elevator`. Without it the integrator would
start at zero, the *target* would differ from the baseline, and the slew limiter would only smear the
step out over time instead of preventing it.

This is **state initialisation, not tuning**: no gain, limit or rate is touched.

### TECS: NOT primed, and not claimed to be

**TECS is not primed in Phase B.** No prime API was added to it, and no continuity claim is made about it.

The pinned PX4 TECS performs its normal first-update **state initialization** when it has no history:

```cpp
if (dt > DT_MAX || _update_timestamp == 0UL) {
    initialize(altitude, hgt_rate, equivalent_airspeed, eas_to_tas);
}
```

That initializes the altitude reference model and the airspeed filter from the actual aircraft state. It
is **not** a bumpless prime, and it must not be described as one:

- **Its integrators are reset to zero and are NOT seeded from the resolved command**
  (`TECSControl::initialize -> resetIntegrals`).
- It therefore **does not reproduce the throttle or elevator currently being consumed**.
- If the real chain runs TECS continuously in shadow before the handoff, `_update_timestamp != 0` and this
  path does not even fire — the integrators carry the shadow history, not the legacy baseline.

**Exact TECS throttle/elevator continuity is NOT proven in Phase B.** It is also not needed here: the
candidate producer is not connected, so no TECS output reaches the command block. The zero-step handoff
this phase proves comes from the arbiter's prime snapshot and the stick's baseline latch.

**Phase C must measure and decide the TECS handoff policy.** The options are a `Coordinator::Reset()`
immediately before handoff (forcing the first-update initialization, available today with no PX4 change)
or seeding the integrator (which would require modifying the pinned v1.17.0 reference this project
protects with byte-equivalence audits). That decision belongs to Phase C, on evidence — not to a comment
written now.

## Safety outranks bumpless — always

If the aircraft stops being `Alive` while a prime is pending or a candidate is ready:

- the prime is **cancelled** and the ticket destroyed (it cannot be replayed on revival),
- the candidate is invalidated,
- the mode returns to `LegacyOrManual` — recovering from a hardover must never silently resume Formation,
- `FallingLegacy` applies **immediately**: no blend, no delay.

An aircraft whose health cannot be established is refused too (`UnknownHealth`) — **not** assumed healthy.
"No health component" is not the same as "alive", and a prime is a promise of continuity for an aircraft
that is actually flying.

A perfectly valid, fresh candidate is refused for a dead aircraft. The hardover is what puts it into the
ground and nothing may fly it out.

## The reverse direction is NOT bumpless, and is not claimed to be

Formation → Legacy is an **immediate safety fallback**. A controller that has gone stale, invalid or
non-finite is abandoned *instantly*, snapping back to whatever the legacy writers are producing right
now. Holding or blending a command from a producer we have just declared untrustworthy would be exactly
backwards.

The continuity Phase B proves is the **first `LegacyOrManual -> FormationControlV2` handoff**. That is
the whole claim.

## Reaching Formation

Two doors, both test-only, and they are not the same:

- **`SetModeForTesting()`** — the *raw* seam from Phase A. Flips the mode with **no prime**. It exists so
  the resolver's own policy (falling priority, staleness, finiteness, isolation) can be exercised
  directly. It is **not** the handoff path.
- **`RequestPrime()` → `SubmitPrimedCandidate()` → `ActivateFormationForTesting()`** — the handoff path.
  Activation is **refused** unless the aircraft is in `PrimedCandidateReady`. A valid, fresh candidate is
  not enough: with no prime behind it, nothing anchors it to what the aircraft is currently flying.

**Production activation remains zero.** No config value, Blueprint default, or UDP packet reaches either
door. The resolver stays bound for the module's lifetime; the default mode stays `LegacyOrManual`.

## What is proven, and what is NOT

Phase B produces **two independent pieces of evidence**. They are not yet joined:

- **The arbiter's handoff continuity** — proven at the consume boundary, through the **test-only**
  activation door. The first Formation command steps by exactly zero from the immutable ticket baseline
  (`PrimeExactHandoff`), and a deliberately offset candidate is *seen* to step
  (`PrimeHandoffDeltaNegativeControl`), so the zero is a measurement and not a tautology.
- **The stick's first-compute continuity** — proven at the **controller** level, by calling
  `F16StickAdapterV2::Update()` directly: the first valid frame after a prime emits the baseline exactly,
  and advances no controller state at all.

**They are not connected to a real candidate producer.** Nothing in the Formation chain feeds the arbiter
today, and the stick's primed output does not reach the command block. Wiring them together is Phase C.

Also **not** claimed here:

- **TECS continuity is NOT proven.** TECS is not primed at all (see above), and its integrators start at
  zero rather than reproducing the consumed throttle/elevator.
- **Formation → Legacy is NOT bumpless**, by design: it is an immediate safety fallback.
- **The real integrated handoff is Phase C**, and it must re-prove continuity end to end with the actual
  producer in the loop — not inherit this proof.

## Run it

```bash
bash Tools/planner_v2/run_command_prime_v2.sh     # COMMAND_PRIME_V2_RESULT=PASS
```

Twenty scenarios, one editor process each, run **exclusively** (the other gates bind UDP 5005).

| | Scenario | Establishes |
| --- | --- | --- |
| A | `PrimeNoResolvedSnapshot` | no consumed command ⇒ no baseline ⇒ prime refused |
| B | `PrimeSnapshotExact` | the snapshot equals the last resolved block in all 29 fields, and mutating the legacy block afterwards does **not** move it |
| C | `PrimeGeneration` | generations are strictly monotonic; a candidate from the superseded one is refused |
| D | `PrimeExactHandoff` | **the first Formation consume steps by exactly 0 on all five controlled fields** |
| E | `PrimeUnprimedRejected` | a valid, fresh candidate with **no prime** cannot activate |
| F | `PrimeWrongGeneration` | a generation that was never issued is refused; mode unchanged |
| G | `PrimeStale` | stale candidate ⇒ Legacy |
| H | `PrimeNonFinite` | a candidate that *claims* finite but carries a NaN is caught by **verification**, not trusted |
| I | `PrimeFallingPreemption` | damage during a pending handoff ⇒ prime cancelled, Formation never resolves, throttle 0 / cutoff 1 |
| J | `PrimePerAircraftIsolation` | priming one aircraft leaves the others untouched |
| K | `PrimeWorldCleanup` | tearing down one world removes only its aircraft; another world's primed state, generation, candidate and all 29 snapshot fields survive **field by field** |
| L | `PrimeResetSessionSafety` | with a candidate accepted and waiting, a session reset clears counters but preserves the prime state, generation, candidate and all 29 baseline fields |
| M | `PrimeResolverOwnershipLost` | if a foreign resolver owns the consume boundary, priming and activation are refused — and we never unbind somebody else's resolver |
| N | `PrimeInterveningConsumeRejected` | a Legacy consume between the prime and the activation ⇒ refused **at the boundary**; Formation never resolves |
| O | `PrimeStickExactFirstCompute` | the stick's own `Update()` reproduces the baseline **exactly** on its first frame |
| P | `PrimeHandoffDeltaNegativeControl` | a deliberately offset candidate **is** measured as a step ⇒ D's zero is a measurement, not a tautology |
| Q | `PrimeStickWrongIdentityDoesNotConsume` | a latch armed for another prime is **not** spent: no baseline, no state change |
| R | `PrimeStickLatchedFrameNoStateAdvance` | the latched frame advances **nothing** — proven by a normal-path counter, not by watching an integrator that anti-windup can legitimately freeze |
| S | `PrimeStickInvalidPrimeRejected` | 15 unusable primes (bad identity, bad baseline, bad config) each change **no** part of the controller state |
| T | `PrimeActivationBoundaryStaleRejected` | a candidate that was fresh at submission but rotted before the handoff is refused **as stale**, not as an intervening consume |

### One subtlety the tests had to respect

`RequestPrime` → `SubmitPrimedCandidate` → `ActivateFormationForTesting` all happen inside **one**
latent-command `Update()`. Latent commands run on the editor tick while the PIE world consumes *between*
them, so splitting them across ticks would let the baseline go stale under the ticket — and the handoff
would step through no fault of the arbiter.
