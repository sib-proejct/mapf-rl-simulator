# ADR 0002: Ruckig jerk-limited cardinal motion

- Status: Accepted
- Date: 2026-08-22
- Scope: Phase 1 single-robot motion and stopping envelope
- Supersedes: ADR 0001 motion integration and `v²/(2a)` stopping-distance decisions

## Decision

Pin the official `ruckig` crate at `0.19.4`. Its upstream C++ implementation is compiled through the
crate's Rust bridge, so supported build environments require a C++20 compiler in addition to Rust.

Each `WAIT/NORTH/EAST/SOUTH/WEST` candidate remains a world-frame target velocity. The current world
velocity and collinear acceleration are projected onto one scalar path DoF, passed to Ruckig's velocity
interface with the fixed 100 ms control cycle, and projected back to world position, velocity, and
acceleration. A direction change targets zero velocity and zero acceleration on the existing path; the new
direction can accelerate only on a later tick after the stop is complete. Yaw remains unchanged.

Motion limits include explicit normal and emergency jerk. Controlled and emergency stops both use Ruckig,
with emergency-specific deceleration and jerk. A calculation error, non-finite output, invalid dimension,
or worsening out-of-limit acceleration fails closed.

Safety checks the sampled one-tick trajectory and the complete Ruckig emergency-stop trajectory. Each
sample chord increases the required clearance by the bounded interpolation deviation
`maxAcceleration × sampleDuration² / 8`. State and scenario digests include acceleration, both jerk limits,
the motion-model identity, and the pinned Ruckig version.

## Consequences

Acceleration is authoritative state and must be supplied for initial scenarios. A stopped initial state
has zero acceleration; a moving initial state's acceleration must be collinear with velocity and within
the configured normal capability. Jerk-limited stops can initially keep increasing speed while an existing
positive acceleration is reduced, and their stopping distance replaces the former analytic `v²/(2a)`
estimate. Existing Phase 1 golden digests are intentionally incompatible and use digest schema v2.
