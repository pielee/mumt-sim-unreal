# The command arbiter — Phase A

There is now exactly one place in the program where the question *"what will actually fly this aircraft
this frame?"* has an answer: `UJSBSimMovementComponent::CopyToJSBSim`, immediately before the first FCS
setter. Everything upstream of that is an **input**; everything downstream consumes a single **resolved**
block.

**Phase A changes no behaviour.** The default mode is `LegacyOrManual` and it is transparent: the resolved
block equals the legacy block in every consumed field. Production ControlV2 is *not* activated.

## Why arbitration cannot live in a writer

The [ownership measurements](CONTROL_V2_COMMAND_OWNERSHIP.md) ruled that out:

- **Four** writers touch the block — the Falling hardover, the InnerLoop autopilot, the manual/UDP path,
  and the aircraft **Blueprints** (five command-write nodes each, confirmed by graph audit). None can
  simply be deleted.
- They run in three different execution contexts with **no `TickGroup` and no tick prerequisite anywhere**,
  so no writer can know whether it is last.
- The block the FDM consumes is therefore a **field-wise mixture**: aileron/elevator/throttle from the
  manual writer while `SpeedBrake` still carries the autopilot's value, because the manual writer never
  touches that field. That mixture is current behaviour.

A writer cannot arbitrate what it cannot see. The consume boundary can.

## The shape

```
Blueprint / ManualUdp / InnerLoopAutopilot / HealthHardover
        ↓  (unchanged — nobody was removed, disabled, or reordered)
Commands + EngineCommands            ← the LEGACY INPUT BLOCK (const from here on)
        ↓
CopyToJSBSim: FJSBSimResolvedCommandBlock Resolved = { Commands, EngineCommands }   (full copy)
        ↓
CommandResolver.ExecuteIfBound(...)  ← THE single arbitration point
        ↓
Resolved.Commands / Resolved.EngineCommands
        ↓
FCS setters + ApplyEnginesCommands(Resolved.EngineCommands)
        ↓
Exec->Run()
```

The legacy members are **never written** by this path — only copied. The resolver receives them as
`const&` and can only write the copy, so writer-ordering measurements remain exactly as valid as before.
`ApplyEnginesCommands` now takes the block explicitly instead of reading the member; passing the member
would have quietly reintroduced a second source of truth for the engines.

With **no resolver bound the copy is the legacy block**, byte for byte, and the aircraft flies as before.

## Module direction

`MUMT_Sim` depends on the JSBSim plugin; the plugin depends on nothing of ours. So the plugin exposes a
**generic** delegate (component, consume sequence, const legacy commands, const legacy engine commands,
mutable resolved block) and the game module registers the policy. The plugin does not know what
FormationControlV2 is, and no circular dependency is introduced.

`CommandResolvedObserver` is **multicast**: the arbiter audits its own decision and the ownership
telemetry records the same event independently. A single-cast delegate would let whichever bound last
silently displace the other — precisely the failure mode this work exists to eliminate.

## Transparency is proven field by field, not hashed

A hash or `memcmp` over the structs would lie in both directions — padding differs, and unconsumed members
(`Steer`, mixture, magnetos…) would create false alarms or mask real ones. So every field the FDM actually
consumes is compared individually:

- **15 flight-control fields**: `Aileron Elevator Rudder YawTrim PitchTrim RollTrim Steer LeftBrake
  RightBrake CenterBrake ParkingBrake GearDown Flap SpeedBrake Spoiler`
- **14 per-engine fields**: `Throttle Mixture Starter Running PropellerAdvance PropellerFeather Magnetos
  Augmentation Injection Ignition Reverse CutOff GeneratorPower Condition`

`legacy_changed_field_count` must be **0** in `LegacyOrManual`, and `legacy_block_mutation_count` must be
**0** always (the resolver is handed the legacy block as const; this checks that claim against reality
rather than trusting the type system).

This deliberately covers the trims, flaps and brakes the **Blueprints** set, and the `SpeedBrake` the
autopilot sets — not just the four obvious stick axes. Comparing only aileron/elevator/rudder/throttle
would have "proven" transparency while silently dropping every Blueprint field.

## Priority order

1. **Falling outranks everything.** If the aircraft's `UHealthComponent` is not `Alive`, the legacy block
   passes through untouched — even if a perfectly valid, fresh Formation candidate is arriving. The
   hardover is what puts a dead aircraft into the ground and nothing may fly it out.
2. **No health component ⇒ Legacy.** We do *not* assume such an actor is alive. We cannot prove it is
   flyable, so we refuse to override, which is exactly what that aircraft does today.
3. `LegacyOrManual` (**the default**) ⇒ legacy passes through.
4. `FormationControlV2` ⇒ the candidate must earn it: present, `bValid`, `bCommandReady`, finite (verified,
   not merely flagged), and no older than `kCandidateMaxAgeS = 0.1 s`. Any doubt falls back to legacy, with
   the reason counted.

An accepted candidate overwrites **only the axes it owns** (aileron, elevator, rudder, throttle,
speedbrake). Trims, flaps, brakes, gear and every Blueprint field keep their legacy values — Formation does
not get to silently zero a field merely because it has no opinion about it.

## Two switches, and they are NOT the same thing

This distinction is the whole of Phase A, and conflating them would make every test here meaningless:

| | State | Where |
| --- | --- | --- |
| **The resolver** | **ENABLED in production** | `FMumtSimModule::StartupModule()` → `MumtCommandArbiterV2::SetEnabled(true)` |
| **FormationControlV2** | **DISABLED in production** | reachable only via `SetModeForTesting()` |
| **Default mode** | `LegacyOrManual` | every aircraft, always, at registration |
| **Formation candidate** | **not wired** | NPFG/TECS/Stick feed nothing in this phase |

The **resolver is a production facility**, bound for the module's lifetime. If it were switched on only by
tests, then production would still be running the old *"whichever writer happened to go last wins"*
behaviour, and every arbiter test would be proving something about a code path that never runs. That is
why `MUMT.ControlV2.ArbiterProductionDefault` exists: it enables **nothing**, sets **no** mode and **no**
candidate, and asserts that the resolver is already running because the module bound it — and that in that
untouched state `resolver_call_count == consume_count`, `legacy_changed_field_count == 0`,
`formation_resolution_count == 0`, and every aircraft is in `LegacyOrManual`.

**FormationControlV2 is not activated.** The test API is for supplying a mode and a candidate — it is *not*
what turns the resolver on. No config value, Blueprint default, or UDP packet can move an aircraft out of
`LegacyOrManual`.

`SetEnabled` is **idempotent in both directions**: enabling twice does not double-register the multicast
observer, disabling twice removes only the handle *we* stored, and because the resolver is single-cast, it
**refuses to evict** a resolver owned by anything else (counted, logged as an error, never silent).

## Lifecycle

State is **per aircraft**, keyed by `TWeakObjectPtr<const UJSBSimMovementComponent>` — there is no global
single mode.

A weak pointer going invalid is **not cleanup**: the entry, its mode and its candidate all still sit in the
registry. So entries are purged on every consume, and `FWorldDelegates::OnWorldCleanup` clears the registry
at world teardown. `MUMT.ControlV2.ArbiterLifecycle` proves it by destroying a real aircraft mid-PIE — one
that has been given a Formation mode first, so a leak would be visible — and requiring the entry to be
gone; then, after PIE ends, requiring the registry to be **empty** while the resolver **stays bound**
(it belongs to the module, not to the world).

## Run it

```bash
bash Tools/planner_v2/run_command_arbiter_v2.sh     # COMMAND_ARBITER_V2_RESULT=PASS
```

Eight scenarios, one editor process each. Scenarios A and C send a real datagram to `127.0.0.1:5005`, so
the gate runs **exclusively** and refuses to start if another editor is up (a second editor binds the same
port and can swallow the datagram, producing a failure that has nothing to do with the code).

| | Scenario | Establishes |
| --- | --- | --- |
| A | `ArbiterLegacyManual` | manual writer alone → resolved == legacy |
| B | `ArbiterLegacyAutopilot` | autopilot alone → resolved == legacy, **and the writer ordering is unchanged** |
| C | `ArbiterLegacyOverlap` | both writers on one aircraft → the **field mixture survives intact** |
| D | `ArbiterFallingOverCandidate` | a live valid candidate is **refused** once the aircraft is dead; throttle 0, cutoff true |
| E | `ArbiterCandidateValid` | the consumed block equals the candidate on its five axes |
| F | `ArbiterCandidateStale` | stale → legacy fallback |
| G | `ArbiterCandidateInvalid` | a candidate that *claims* finite but carries a NaN → legacy fallback; nothing non-finite ever reaches the FCS |
| H | `ArbiterIsolation` | one aircraft in Formation mode leaves the other in Legacy, untouched |

## What Phase A does NOT do

No production ControlV2 activation. No Prime/bumpless-transfer API. No clamping, no slew, no range
enforcement — range and non-finite conditions are **observed and counted**, never corrected, because
correcting them would change what the FDM consumes. No writer disabled, no tick order changed, no tuning
touched.
