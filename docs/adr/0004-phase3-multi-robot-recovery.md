# ADR 0004: Phase 3 multi-robot plan and recovery boundary

- Status: accepted
- Date: 2026-08-23

## Context

Phase 3 must prevent partial `PlanRevision` delivery from authorizing motion, keep unreleased robots as
stationary reservations, commit multi-robot ticks without partial world updates, bound report memory, and
restore observed safety facts after a process restart without treating local state as Core authority.

## Decision

- `PlanCoordinator` owns an immutable revision, the 15-second prepare deadline, per-robot hold/prepared/active
  state, and Planner's total activation order. The full prepare barrier must complete before the first release;
  each later release additionally requires the prior activation's runtime-safe confirmation.
- `MultiRobotEngine` stores robots in `BTreeMap<RobotId, _>`. It previews every transition on cloned engines,
  checks the current swept transition and the contract's two-second constant-velocity separation horizon, and
  swaps in the complete next fleet only after all checks pass.
- An unreleased, failed, or recovery-held robot receives `WAIT` and remains a stationary reservation. A predicted
  crossing or corridor conflict yields deterministically, holds the revision, and requests replan. Bounded
  no-progress ticks trigger deadlock recovery. A robot failure holds every target instead of releasing a
  downstream dependency.
- Pre-spool reports use a bounded priority queue. State projections may coalesce and diagnostics may drop;
  critical/control/durable overflow latches backpressure for stop and reconciliation. The durable spool and its
  dead-letter audit are bounded independently.
- Fleet pose, velocity, acceleration, tick/time, emergency latch, no-progress counter, and plan state use a
  checksum-protected atomic checkpoint. Loading verifies simulator/map/robot identities and fences all motion
  until Core reconciliation. A restored checkpoint never resumes an active plan by itself.

## Consequences

- Fleet scheduling is conservative: a two-second prediction may hold a robot earlier than strictly necessary.
  This is acceptable because local safety may slow or stop but may not manufacture an unsafe release.
- Checkpoint filesystem work is synchronous and must run during startup or through a blocking worker boundary.
- Core remains authoritative for Order and plan versions; checkpoint mismatch produces `Not Ready` recovery,
  not local overwrite.
