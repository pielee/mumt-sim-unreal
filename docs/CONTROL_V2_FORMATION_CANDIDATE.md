# Connecting the real Formation candidate producer (Phase C)

Phase B proved two things in isolation: the arbiter's Prime → Submit → consume-boundary activation path,
and the stick's `PrimeFromResolvedCommand()` baseline latch. Neither was connected to a real controller.

Phase C connects them. `FFormationCandidateProducerV2` runs the **actual** Formation chain —
`FormationPlannerV2 → NPFG → TECS → F16StickAdapterV2` — and feeds its output through the Phase B contract
into the command arbiter. Nothing about the arbiter's guarantees changed; the producer is a *caller* of the
existing contract.

## The chain is two stages, and only the second is an FCS command

```
CanonicalNavigationAdapterV2::Convert → FNav
FormationSlotGeneratorV2::Calculate   → Slot            (needs the leader's canonical nav)
PlannerV2InputAdapter::Build → FormationPlannerV2::Update → PlannerV2OutputAdapter::Build → Dto
FormationGuidanceCoordinatorV2::Update → {RollRef, PitchRef, ThrottleRef}   (NPFG roll + TECS pitch/throttle)
F16StickAdapterV2::Update              → {aileron, elevator, rudder, throttle} norm
```

The coordinator produces *references*, not surface commands. The stick turns them into normalized FCS
commands. **Speed-brake is never produced** by the chain, so the producer owns exactly four fields —
aileron, elevator, rudder, throttle — and leaves speed-brake at the baseline.

There is no direct JSBSim surface (`fcs/*-pos-rad`) or aerodynamic write anywhere: the only route into the
FDM is the normalized command block through `CopyToJSBSim`, exactly as in Phase A/B.

## The candidate is a full 29-field block

Every candidate the producer submits is the **whole** resolved command block: it copies the immutable prime
baseline and overwrites only the four fields the chain owns. Every other field — trims, flaps, brakes,
gear, and every engine field but throttle — carries the **baseline** value, so the block is continuous with
what the aircraft was actually flying, not a mixture of producer axes and one-more-frame of legacy writers.

The arbiter applies this full block wholesale in Formation mode (`bHasFullBlock`). The Phase B five-field
overlay path is untouched and still used by the Phase B tests — the extension is additive.

## The prime order (one game-thread call)

`BeginHandoff()` = `PrepareHandoff()` (steps 1–6) then `RequestActivation()` (step 7), all in one call so no
Legacy consume can slip between the prime and the activation:

1. read the last resolved snapshot the FDM consumed — the baseline
2. `RequestPrime`
3. keep the generation + baseline consume sequence + immutable baseline
4. `PrimeFromResolvedCommand` — arms the stick's one-shot baseline latch, keyed to this generation/sequence
5. compute the first candidate for the **same** aircraft/state — the primed stick returns the baseline
   **exactly** on this first compute (that is the zero-step)
6. `SubmitPrimedCandidate` as a full 29-field block
7. `RequestFormationActivation`

The switch is confirmed later, at the next FDM consume boundary, where the arbiter re-checks that the
baseline is still current (no intervening consume) and the candidate is still fresh. Until then the mode is
Legacy.

Before a handoff, `WarmChain()` runs the chain in shadow (no prime, no submit) for a short window. The
canonical nav adapter needs consecutive samples for a ground course and the coordinator skips its first
frame, so a cold chain would produce nothing for a few frames and the baseline candidate would go stale
waiting. Warming first — exactly what the airborne shadow test does — means the frame after the handoff is
already a valid real command.

## ActiveFormation and the safety fallback

Once active, `Update()` computes a fresh real candidate each frame and resubmits it against the same ticket
(new timestamp). If the producer stops submitting, the last candidate goes stale within
`kCandidateMaxAgeS` and the arbiter falls back to Legacy on its own — no new code, the Phase B safety
contract. `RequestLegacyFallback()` is the **immediate** Formation → Legacy fallback: the mode drops now,
the next consume resolves Legacy, and all prime state is discarded. There is no blend, and the reverse
direction is not claimed to be bumpless.

Falling always outranks a handoff: damage to an aircraft with a ready candidate cancels the prime before it
ever resolves Formation, and the hardover's throttle-0 / cutoff block reaches the FDM.

## What is proven, and what is NOT

Proven, by `Tools/planner_v2/run_formation_candidate_integration_v2.sh` (8 scenarios, real producer, live
leader + follower):

- **RealProducerHandoffExactFirstConsume** — the first Formation consume steps by exactly 0 on all five
  controlled fields, with the candidate computed by the real chain.
- **RealProducerNegativeControl** — the first frame is exactly the baseline (latch), and from the second
  frame the real output moves the controls (measured throttle up to 1.0, elevator up to ~0.56): proof it is
  a live connection, not a fake that re-emits the baseline.
- **InterveningLegacyConsumeRejected**, **StaleRealCandidateRejected** — the two boundary refusals, kept
  distinct, now with a real candidate.
- **FallingPreemptsRealCandidate** — safety cancels a ready candidate before Formation ever resolves.
- **PerAircraftAndWorldIsolation**, **ActiveFormationUpdates**, **ImmediateFormationToLegacyFallback**.

**NOT** proven, and not claimed:

- **TECS bumpless continuity.** The exact-zero first consume is the stick latch's guarantee. From the second
  frame on, whatever transient the real controllers (including TECS) produce is present; the tests **log**
  the second-frame deltas but never require them to be zero. Seeding TECS for a bumpless second frame is a
  separate follow-up.
- **No production trigger.** `BeginHandoff` is an explicit game-thread call; nothing auto-activates
  Formation, and there is no UDP/BT/Blueprint path to it in this phase. The default mode stays Legacy.

## Run it

```bash
bash Tools/planner_v2/run_formation_candidate_integration_v2.sh   # COMMAND_FORMATION_CANDIDATE_V2_RESULT=PASS
```
