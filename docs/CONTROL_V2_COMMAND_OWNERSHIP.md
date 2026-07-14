# Command ownership — who actually flies the aircraft

Before an Active Single Command Writer can exist, one question has to be answered honestly: **who writes
the JSBSim command block today, in what order, and can anything write it that we don't know about?**

The read-only audit got as far as source could take it and then stopped, because two things genuinely
could not be determined by reading:

1. **Ordering.** There is no `TickGroup` and no `AddTickPrerequisite` *anywhere* in the project or the
   plugin. The three writers run in three different contexts — the Falling hardover on a Component Tick,
   the manual/UDP path on the Actor Tick, and the InnerLoop autopilot on a **60 Hz `FTimerHandle`** — and
   the FDM step runs on the JSBSim component's tick. Nothing in the source says which happens first.
2. **Blueprint.** `UJSBSimMovementComponent::Commands` is `BlueprintReadWrite`, and both aircraft
   Blueprints reference `Commands` / `Aileron` / `UDP_Roll`. A C++ comment in the manual path even says
   raw joystick control works *"even when the pawn's Blueprint doesn't forward UDP_\* into Commands"* —
   which only makes sense if some Blueprint does.

This instrumentation answers both by **measurement**. It observes and never arbitrates.

## Run it

```bash
bash Tools/planner_v2/run_command_ownership_telemetry.sh
```

Expected final line: `COMMAND_OWNERSHIP_RESULT=PASS`.

Seven editor processes: four scenarios, one Blueprint graph audit, and a telemetry off/on equivalence
pair. Artifacts land in `/tmp/mumt_command_ownership` (override with `MUMT_OWNERSHIP_OUT`).

## Where it measures

`UJSBSimMovementComponent::CopyToJSBSim()` is the **one** place a command block reaches the FCS:

```
CopyToJSBSim()
  ├─ CommandConsumeObserver.ExecuteIfBound(this, Commands, EngineCommands)   ← observation point
  ├─ FCS->SetDaCmd(Commands.Aileron) ...                                     ← first FCS setter
  └─ Exec->Run()                                                             ← FDM step
```

Whatever sits in `Commands` at that instant is what flies the aircraft — no matter who put it there, C++
or Blueprint or anything else. The observer is handed the block **by const reference**, before the first
setter, so it cannot alter what is consumed. It is unbound unless a test enables it.

The hook lives in the plugin because the module dependency only runs one way (`MUMT_Sim` → the JSBSim
plugin). The game module registers an observer; the plugin never calls into the game module.

Each of the three C++ writers reports itself immediately **after** it has written. The telemetry then
reads the values back out of the component, so what gets recorded is what the writer actually left
behind — not what it intended to leave.

## What "unattributed change" means, and what it cannot mean

At consume time the block is compared against the last snapshot a **registered** writer reported. If they
differ, something that does not report itself moved it, and that is counted and logged with the changed
axes.

**The limit, stated plainly:** a Blueprint (or any unregistered path) that writes *the same values* a
registered writer already wrote is **invisible** to this method — there is nothing to detect. So a zero
count means *"nothing unregistered changed the commands during this run"*. It does **not** mean
*"nothing unregistered touched the commands."* Those are different claims and only the first is
supported. This is why the Blueprint graph audit exists as a separate, independent check.

## The Blueprint graph audit

`MUMT.ControlV2.CommandOwnershipBlueprintAudit` loads `M_F16` and `F16_UAV` **read-only** (no save, no
compile), walks every graph via `UBlueprint::GetAllGraphs`, and reports every node that could *write* the
command block — variable sets, struct-member sets, struct makes — whose title or pins reference
`Commands` / `EngineCommands` / `Aileron` / `Elevator` / `Rudder` / `Throttle`. Reads are deliberately
not counted.

Binary string scans cannot distinguish a read from a write; this can. The result is **reported, not
asserted to be zero** — if a Blueprint write node exists, the Active Writer must treat Blueprint as a
Legacy *input* rather than pretend it isn't there.

## Scenarios

| | Scenario | Switches | Isolates |
| --- | --- | --- | --- |
| A | `CommandOwnershipAutopilotOnly` | `-FormationTest` | the InnerLoop autopilot alone |
| B | `CommandOwnershipManualOnly` | *(none)* | the manual/UDP writer alone — **no** `-FormationTest`, so no setpoints exist and the autopilot cannot run |
| C | `CommandOwnershipOverlap` | `-FormationTest` | **both** writers on one aircraft |
| D | `CommandOwnershipFalling` | `-FormationTest` | the Falling hardover taking ownership |
| E | `CommandOwnershipBlueprintAudit` | *(none, no PIE)* | Blueprint command-write nodes |

Scenario C is a real overlap, not a contrived one: `M_F16` is simultaneously the FormationTest **leader**
(so it gets a setpoint, driving the autopilot writer) and a member of `ControlledPawnNamePatterns` (so a
named UDP command reaches it, driving the manual writer). The manual command is delivered through the
**real UDP socket** on port 5005 — the same path a joystick uses — so no production code is touched to
make the scenario happen.

Scenario D damages the aircraft through the production `UHealthComponent::ApplyDamage` API.

## Proving the instrumentation is inert

"The code only reads" is an argument. This is a measurement: the **same** airborne scenario is run twice,
identical in every way except `-CommandOwnershipTelemetry`, and the resulting flight must be identical —
same sample count, same airborne samples, same guidance/stick/planner populations. Those numbers come out
of the actual flown trajectory, so if the telemetry had perturbed a single command they would diverge.

The switch is opt-in only; nothing in production passes it, and with it absent the consume delegate is
unbound and every writer-side call is a single bool test that returns.

## What the measurements found

### Blueprint IS a writer — confirmed, not inferred

The graph audit found **five command-write nodes in each aircraft Blueprint**:

| Graph | Node | Writes |
| --- | --- | --- |
| `EventGraph` | `Set Commands` (`K2Node_VariableSet`) ×2 | the whole `Commands` struct |
| `EventGraph` | `Set members in FlightControlCommands` | `Aileron`, `Elevator`, `Rudder`, `YawTrim`, `PitchTrim`, `RollTrim`, `Flap` |
| `EventGraph` | `Set members in EngineCommand` | `Throttle` |
| `BrakingLogic` | `Set members in FlightControlCommands` | `LeftBrake`, `RightBrake`, `CenterBrake` |

Both `M_F16` and `F16_UAV`. So the Active Writer **cannot** treat the C++ writers as the complete set:
Blueprint is a Legacy command input and has to be designed for, not assumed away.

(`unattributed_command_change_count` was 0 in every scenario. That does **not** contradict this — see the
limit above. These nodes either did not execute in these runs or wrote values identical to what a
registered writer had already written, and the second case is undetectable by construction.)

### The writers do not run where you would guess

Measured, per frame, on one aircraft:

```
WRITE ManualUdp  →  CONSUME (CopyToJSBSim → FCS → Exec->Run)  →  WRITE InnerLoopAutopilot ×2
```

`writes_after_consume_same_frame` was **0 / 1801** for the manual writer and **3600 / 3600** for the
autopilot. The manual/UDP path (Actor Tick) writes *before* the FDM consumes; the autopilot (60 Hz
`FTimerHandle`) writes *after* it. Two consequences fall straight out:

- **Autopilot commands reach the FDM one frame late.**
- **About half of them never reach it at all.** The timer is not in lockstep with the consume:
  `max_writes_per_consume = 2` with `duplicate_write_count = 1800` out of 3600 writes, and roughly half
  the consumes saw no write at all. Where two autopilot writes land between two consumes, the first is
  overwritten before JSBSim ever sees it; where none land, the FDM re-consumes the previous command.

### Two writers really do own one aircraft

In the overlap scenario `multi_writer_consume_count = 900`, and **every** such consume ended with
`last = ManualUdp`. The manual command wins — not because of any priority rule (there is none), but
because of the tick order above: the autopilot writes after the consume, and the manual writer overwrites
it before the next one. **The autopilot's output is silently discarded whenever a manual command is
present.** That is an accident of registration order, not a design.

### Falling ownership is exclusive — and that part is sound

`F16_UAV1` was shot down mid-flight while the autopilot was actively flying it. On that one aircraft:

```
autopilot writes at the moment of damage : 900
autopilot writes at the end of the run   : 900     <- it stopped, completely
Falling hardover writes                  : 901
command finally consumed  ail=0.682  elv=-0.204  rud=0.363  thr=0.000  cutoff=1
```

The autopilot did not write it again — the `IsAlive()` early-return in both `ApplyAutopilotToPawn` and
`ApplyControlCommandToPawn` holds — while the leader kept flying normally. Falling ownership is exclusive
and nothing overwrites the hardover. **The Active Writer must preserve this: the hardover has to outrank
every other source, including FormationControlV2.**

(The first attempt at this scenario damaged an *idle spare* UAV and still "passed", because the fleet-wide
totals showed a hardover somewhere. It proved nothing about hand-over. The target is now chosen by asking
the telemetry which aircraft the autopilot is actually writing, and the before/after counts are asserted
per aircraft, because fleet-wide totals cannot answer a per-aircraft question.)

## What this does NOT do

No arbitration. No priority. No clamp, no slew, no fallback, no rejection. Nothing is disabled and no
writer's order is changed. Range and non-finite conditions are **observed and counted**, never enforced —
enforcing would change what JSBSim consumes, which is exactly what this task must not do.
