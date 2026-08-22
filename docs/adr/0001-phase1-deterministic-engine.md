# ADR 0001: Phase 1 deterministic engine ordering and random streams

- Status: Accepted
- Date: 2026-08-22
- Scope: Phase 1 single-robot standalone engine

## Decision

The authoritative engine is owned by one `SimulationEngine` value. State can be committed only through
mutable engine access, once per fixed 100 ms control tick. Operational monotonic time is supplied through
the `MonotonicClock` port but is excluded from simulation ordering, physics, records, and state digests.

Each tick uses this total phase order:

| Priority | Event/phase | Stable tie-break |
|---:|---|---|
| 10 | emergency-stop fault | source ID bytes, source sequence |
| 20 | actuator-stuck fault | source ID bytes, source sequence |
| 30 | actuator-slowdown fault | source ID bytes, source sequence |
| 40 | sensor-dropout fault | source ID bytes, source sequence |
| 100 | scenario action candidate | single robot, one candidate per tick |
| 110 | safety decision and motion preview | fixed safety pipeline order |
| 120 | invariant check and world commit | single commit |

Fault specs are kept in stable source order and evaluated only for the current tick. Events produced at
that tick are ordered by `(simulationTimeMs, priority, stableSourceId UTF-8 bytes, sourceSequence)`.
Ranges are never expanded into a future event list. The emergency-stop state is latched. Invalid actions
become `WAIT`; unsafe requested motion is replaced by a safe `WAIT`, a controlled stop, or an emergency
stop. Both stop outcomes integrate velocity and position using their configured deceleration instead of
setting velocity to zero immediately.

Motion and safety use one rate-selection rule. Reversal first decelerates to zero, and slowdown uses the
controlled deceleration limit. Safety accepts a next state only when its one-tick swept path and its
conservative emergency stopping segment `v²/(2a)` clear static obstacles. If controlled braking is unsafe,
the decision escalates to emergency braking. If even that transition is unsafe, the tick fails with a
collision invariant and state, time, emergency latch, and persistent fault state are not committed.

Random values use the checked-in SplitMix64-based algorithm and stateless `(stream seed, tick, lane)`
sampling. The master seed is derived into disjoint streams:

| Stream label | Scope | Lanes |
|---|---|---|
| `sensor` | robot position sensor | 0 dropout, 1 x noise, 2 y noise |
| `fault` | each stable fault source | 0 activation trial |

Stream derivation includes the master seed, subsystem label, robot ID, and stable source ID. Changing
the number of sensor draws therefore cannot change fault activation, and vice versa. OS randomness,
wall-clock time, hash-map order, and thread completion order are not simulation inputs.

## Consequences

The same validated scenario and master seed produce the same canonical SHA-256 state digest. Scenario
records include the contract version, map/config/action/fault-spec identities, actually applied fault
events, ordered step records, sensor output, safety outcome, and final state. They do not include a
materialized future fault schedule, so memory use does not scale with a fault range's length.
Cross-platform equality assumes Rust's IEEE-754 `f64` behavior for the fixed arithmetic sequence used by
this phase.
