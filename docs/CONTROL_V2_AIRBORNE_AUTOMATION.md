# ControlV2 Airborne shadow — runtime gate

Every other ControlV2 verification in this repo is a **host harness**: it links the production
controllers straight against JSBSim and never loads Unreal. Those harnesses are where the numbers come
from (NPFG caller contract, the F-16 lateral/longitudinal closed loops, the tuning sweeps). What they
cannot prove is that the same chain still behaves **inside a real Unreal world**, on a real map, with
the real command writer flying the aircraft.

These three Automation tests are the only coverage of that, and until this runner existed there was no
committed way to invoke them — nothing in the repo referenced them, so in practice they never ran.

## Why the gate had to be *restored*, not just added

The first time these tests were actually executed, all three failed. Not flakily — every frame, for a
concrete reason.

`ab8d6be` (*supply TECS turn load factor*) made `CurrentRollRad` / `bCurrentRollValid` a **required**
input on `FGuidanceCoordinatorInputV2`: the coordinator feeds roll to TECS as the turn load factor, and
rejects the frame with `InvalidFollower` when the validity flag is false. That commit updated the host
harnesses, but not this Automation fixture — because nothing ran it. So the fixture kept building its
guidance input with pitch only, `bCurrentRollValid` defaulted to `false`, and **every** guidance frame
was rejected:

```
planner valid=13816   guidance valid=0   stick valid=0   leaderStickValid=0
guidanceFailures ... InvFollower=17995     (of 18,002 samples)
```

`airborneSamples(WOW=false)=0` was a *consequence*, not a second bug: a frame with a valid planner but
no command-ready guidance is classified `Invalid`, so the `Airborne` phase could never be reached.

The fixture now supplies roll from the same authoritative snapshot and the same attitude-valid flag as
pitch. That is the whole fix: an Automation fixture meeting the current production caller contract. No
production algorithm, tuning, acceptance threshold, or command ownership changed.

The lesson is the point of this file: **a test that nothing runs is not coverage.** This regression sat
latent for several commits purely because the gate had no runner.

## Why each test needs its OWN editor process

This is the second thing the first real execution exposed, and it is not optional.

Each test calls `AddExpectedErrorPlain(TEXT("bUseAttachParentBound"), ...)` with an occurrence count of
`0`, which in UE means **"must occur at least once"**. The matched message is a pre-existing Blueprint
construction-script error in `F16_UAV` / `M_F16` (an array-get on `None` feeding
`Set bUseAttachParentBound`), and it is emitted **only on the first PIE session of an editor process** —
the Blueprint is compiled once and then stays compiled.

So when all three tests share one editor process, the first test consumes the one and only occurrence
and the other two fail with *"Expected suppressed ... did not occur"* — regardless of the control chain,
and with only the execution order deciding which test survives. Measured directly: all 30 occurrences of
the warning fell inside the first test's log window, and exactly the two later tests failed, each with
that single error and no control-assertion error at all.

The gate therefore runs **one test per `UnrealEditor-Cmd` process**. Each test becomes its own first
PIE, observes the expected warning, and the expectation is deterministic again.

The Blueprint warning itself is **not** fixed, suppressed, or deleted here, and the fixture's
`AddExpectedErrorPlain` is left exactly as it is. Cleaning up that Blueprint is separate work.

**Full gate = 3 tests × 2 repetitions = 6 editor processes** (plus one discovery process).

## Run it

```bash
bash Tools/planner_v2/run_airborne_control_v2_automation.sh
```

Expected final line:

```
AIRBORNE_CONTROL_V2_AUTOMATION_RESULT=PASS
```

This launches **7 editor processes** — one discovery pass, then 3 tests × 2 repetitions — and takes on
the order of an hour. Each test is a full 300 s of simulated flight.

`UE_ROOT` defaults to `$HOME/unreal`; override it if the engine lives elsewhere. Artifacts are written
to `/tmp/mumt_control_v2_airborne_automation` (override with `MUMT_AUTOMATION_OUT`). The runner writes
nothing into the repository and never calls `git clean`.

## The tests

Registered in [`Source/MUMT_Sim/Private/State/MumtAirborneShadowTest.cpp`](../Source/MUMT_Sim/Private/State/MumtAirborneShadowTest.cpp)
via `IMPLEMENT_SIMPLE_AUTOMATION_TEST`, all three with
`EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter`, inside
`#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS`:

| Test | Graded population |
| --- | --- |
| `MUMT.ControlV2.AirborneNearFieldPlannerShadow` | planner-feasible frames |
| `MUMT.ControlV2.AirborneNearFieldEnvelopeRejectionShadow` | envelope-rejected frames |
| `MUMT.ControlV2.AirborneFullControlShadow` | full guidance + stick chain |

All three observe the *identical* shadow run and log the identical populations; they differ only in
which population they grade. Because the flag is `EditorContext`, they need a real editor process —
`UnrealEditor-Cmd`, not a standalone game binary.

**Required world:** `/Game/RL_2` (`Content/RL_2.umap`), loaded by `FEditorLoadMap`, then `FStartPIECommand`.
The tests therefore require PIE. They run headless under `-nullrhi`.

**Required switch:** `-FormationTest`. The aircraft are flown by the existing writer
(`AUDPControlReceiver` → `InnerLoopAutopilot` → `UJSBSimMovementComponent`), and its scripted formation
profile (takeoff → straight 220 → 3°/s turn → 4°/s turn + climb + decel → rollout → breakaway → rejoin)
is armed **only** by that command-line switch ([`UDPControlReceiver.cpp:169`](../Source/MUMT_Sim/Private/UDPControlReceiver.cpp#L169)).
Without it the aircraft never fly the profile and the tests fail their airborne-sample assertions.

`-FormationTestExit` is deliberately **not** passed: it makes the writer exit the process when the
profile finishes, which would kill the editor mid-Automation and destroy the result report.

**Termination:** each test self-terminates — `kMaxSimSeconds = 300`, hard wall cap
`kMaxWallSeconds = 420`. No manual editor interaction is ever required.

## The exact invocation

One process per test — `<TEST>` is a single exact test name, never a `+`-joined filter:

```bash
$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd "$REPO_ROOT/MUMT_Sim.uproject" \
  -ExecCmds="Automation RunTests <TEST>; Quit" \
  -ReportExportPath="<out>/repetition_<N>/<slug>/report" \
  -FormationTest \
  -unattended -nopause -nosplash -nullrhi \
  -abslog="<out>/repetition_<N>/<slug>/automation.log"
```

Artifacts, one directory per test per repetition:

```
/tmp/mumt_control_v2_airborne_automation/
  discover/                       automation test list (registration check)
  repetition_1/{planner,envelope,full_control}/
  repetition_2/{planner,envelope,full_control}/
      automation.log  report/index.json  invocation.txt
      summary.txt     durations.txt      airshadow.txt
```

`invocation.txt` records the exact command, engine root, project path, start/end times and process exit.
`airshadow.txt` carries the test's own `[AIRSHADOW]` runtime counters.

The trailing `; Quit` is load-bearing. The Automation command handler only sets `GIsCriticalError`
(and therefore a non-zero exit status) on the queued *Quit*; without it the editor lingers and the
process exit code says nothing about the tests.

## Pass/fail is decided by the report, not the exit code

`docs/STATE_API.md` records a prior Automation run of this project where the **verdict was Success but
the `UnrealEditor-Cmd` process still exited 1**. An exit-code-only gate is therefore not trustworthy
here, and a filter that matches nothing does not fail — it runs zero tests and exits cleanly, which is
the most dangerous failure mode a gate like this has: green, having proved nothing.

So the runner takes three independent signals per process and requires all of them to agree:

1. **Registration** — the three names are confirmed against the live engine's own `Automation List`
   output *before* anything runs. A missing test is a hard failure, never a silent skip.
2. **The Automation report** (`-ReportExportPath` → `index.json`) — the authoritative verdict. Each
   single-test process must report exactly one test, whose `fullTestPath` is the exact name requested
   and whose `state` is `Success`, with `failed == 0`, `notRun == 0`, `inProcess == 0`.
   (The engine writes `index.json` with a UTF-8 BOM, so it is parsed with `encoding="utf-8-sig"`.)
3. **The process** — exit code 0, the engine's own `**** TEST COMPLETE. EXIT CODE: 0 ****` marker, no
   timeout, and no `Fatal error` / `Assertion failed` / `Ensure condition failed` /
   `=== Critical error ===` / `SIGSEGV` marker in the log. An *absent* exit marker also fails: it means
   the editor never reached the queued `Quit` — it hung or died — which must never read as a pass.

### `succeededWithWarnings` is a normal pass here

The engine does **not** count a passing-but-warning test under `succeeded`. It has a separate bucket:

```
passed_count = succeeded + succeededWithWarnings      # must be exactly 1 per single-test process
```

This scenario *always* trips the pre-existing `F16_UAV` Blueprint warning, so a genuinely passing test
lands in `succeededWithWarnings` and `succeeded` reads `0`. Asserting `succeeded == 1` would therefore
reject a healthy run — an early version of this runner did exactly that. The gate sums the two buckets
and still independently requires `state == Success`, so a *failing* test can never be laundered into a
pass. The warning is neither suppressed nor deleted; it is simply counted where the engine puts it.

The crash markers are matched narrowly on purpose. A bare `Error` grep would trip on that same
Blueprint construction-script error, which the test itself declares expected via `AddExpectedErrorPlain`.

### Repetition

The whole three-test set is run **twice**, in two independent sets of processes (6 in total), in the
fixed order planner → envelope-rejection → full-control. The normalized summary of each test — requested
and executed name, `state`, `succeeded` / `succeededWithWarnings` / `failed` / `notRun` / `inProcess`,
process exit, engine exit code, crash count — must be identical across both repetitions.

**Not** compared: durations, timestamps, absolute log paths, PIE instance numbers, and UE log bytes.
They vary between processes for reasons that say nothing about the control chain. Per-test durations are
recorded separately in `durations.txt`.

The gate is **fail-fast** at every level: the first failing test ends the run. Once it is red, spending
another 20–30 minutes of editor time to confirm it only delays the report.

## The shadow write contract — stated exactly

Be precise about what this does and does not establish:

- The **existing `UDPControlReceiver` / `InnerLoopAutopilot` writer flies the actual aircraft.** It is
  the only thing commanding them, exactly as in normal operation.
- The **ControlV2 shadow computes a command DTO only**, on its own separate
  `FormationGuidanceCoordinatorV2` / `F16StickAdapterV2` instances (one set per observed aircraft).
- `MumtAirborneShadowTest.cpp` contains **zero command-write call sites** — no `Commands`,
  `EngineCommands`, or `fcs/*-cmd-norm` assignment anywhere in the file.
- Therefore: **that the ControlV2 shadow writes no additional JSBSim command is established
  structurally** — by the absence of any write path in the translation unit.

What is **not** claimed:

- **Production writer invocation = 0 is NOT asserted.** The production writer is *supposed* to be
  running; it is flying the aircraft. Nothing here counts its invocations.
- There is **no runtime command-ownership counter** yet. The shadow-only property rests on a static
  argument about the source, which is strong, but it is not a runtime assertion.

Verified per frame by the tests themselves:

- all valid shadow outputs finite; stick commands within `[-1,1]`, throttle within `[0,1]`
- no configured slew-limit violation
- no stale command on an invalid frame; guidance/stick reset generations consistent
- planner-invalid ⇒ stick not command-ready; no partial guidance/stick output
- frame and open-loop-phase accounting closes (no unclassified frames)
- authoritative JSBSim weight-on-wheels observes a genuine airborne phase (> 100 samples)
- roll sign contract holds; all shadow calls on the game thread; sim time monotonic
- curvature never exceeds the planner's own turn bound; no false accept/reject; no rejection latch

### Not verified — recorded, not papered over

The tests emit **no runtime counter** for command ownership. The shadow-only property is guaranteed by
construction (no write call site exists in the test TU), which is strong, but it is a *static* argument
and not a runtime assertion. Specifically, there is no runtime check that:

- production writer invocations are 0 during the shadow run
- the Legacy writer is unchanged
- normalized-command / surface-position / aerodynamic-property direct writes are 0
- UDP/BT command ownership is unchanged

These were **not** added here: extending the Automation test source was out of scope for this task. They
are the next minimal test seam, and they are the natural companion to the Active Single Command Writer
audit — that audit is what would need them.

## Production tuning

The tests do not print the guidance config, and neither the production source nor the test source was
modified to make them. The shipped NPFG policy (`NpfgPeriodS = 25.0`, `NpfgDamping = 0.7`,
`NpfgRollTimeConstantS = 0.0`, `RollLimitRad` unchanged) is evidenced by the host caller contract:

```bash
bash Tools/planner_v2/build_verify_npfg_caller_contract_v2.sh   # 109 checks / 0 failures
```
