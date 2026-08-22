# MAPF-RL Simulator Instructions

## Responsibility

- This repository owns the Rust/Tokio virtual-robot runtime: Core communication, order interpretation, local policy inference, motion and sensor simulation, fault injection, and deterministic safety enforcement.
- Communicate only with Core's versioned REST and WebSocket APIs. Never access Redis or PostgreSQL directly.
- The simulator has final authority to reject unsafe policy output, slow down, or stop even when Core or a policy requests motion.

## Engineering rules

- Prefer the simplest design that satisfies the current requirements. Do not add speculative abstractions, extension points, configuration, or layers without a concrete use case.
- Handle errors at meaningful boundaries and where recovery or useful context is possible. Avoid broad, redundant, or defensive exception handling that hides programming errors or complicates the normal control flow.
- Optimize code for readability and concision. Use direct control flow, clear names, and small focused units; avoid cleverness, unnecessary indirection, and duplicated ceremony.
- Keep protocol handling, simulation state, policy inference, safety checks, and time progression in distinct modules.
- Preserve determinism for the same scenario, seed, configuration, model artifact, and event sequence. Avoid wall-clock dependence in simulation logic; inject time and randomness.
- Do not block Tokio executors with ONNX inference, heavy collision checks, filesystem access, or CPU-bound simulation work. Isolate blocking work explicitly.
- Treat all network messages and model artifacts as untrusted input. Validate schema versions, dimensions, numeric ranges, checksums, and compatibility before entering Ready state.
- Handle disconnects, timeouts, malformed messages, stale `orderUpdateId`, and duplicate commands without panicking.
- Safety checks are deterministic and independent of policy confidence: permitted path, obstacle clearance, minimum separation, speed/acceleration bounds, emergency stop, and communication timeout.
- Use explicit coordinate frames and physical units. Centralize conversions and reject invalid or non-finite floating-point values.
- Keep hot loops allocation-aware, but optimize only with measurements and preserve readable safety logic.

## Policy package contract

- Load only approved immutable packages containing `model.onnx`, `manifest.json`, observation/action schemas, normalization data, and checksums.
- Verify the policy ID/version, contract versions, input/output shapes, normalization parameters, checksum, and minimum simulator version before use.
- Record the active policy version in reported state and fail safe when loading or inference validation fails.
- Never apply raw model output directly to simulated motion; convert it to a candidate action and pass it through deterministic safety checks.

## Testing and validation

- Use seeded tests for kinematics, sensing, inference adapters, safety rejection, and time progression.
- Cover crossings, narrow corridors, deadlocks, dynamic obstacles, packet delay/loss/duplication, reconnects, invalid artifacts, NaN/Infinity output, and inference timeout.
- Add contract tests against Core schemas and regression scenarios with stable expected metrics.
- Discover build, format, lint, and test commands from `Cargo.toml` and checked-in tooling. Once scaffolded, use `cargo fmt --check`, `cargo clippy` with the repository's flags, and focused `cargo test` runs as applicable.
- Until `Cargo.toml` exists, do not pretend Rust validation ran; report the missing scaffold explicitly.

## Commit workflow

- Before staging a Rust change, run `cargo fmt --check`; use `cargo fmt` to apply formatting, then re-run the check. Run `cargo clippy` with the repository's configured flags when practical.
- Check the changes before proposing a commit, and split unrelated changes into separate commits.
- Run relevant validation when practical, beginning with the narrowest checks.
- Propose the exact files and commit message to the user, then obtain confirmation before committing.
- Commit only the files explicitly approved by the user.

Use this subject format:

```text
type(scope): concise English summary
```

Use this body format, with a Korean translation of the subject and English and Korean details:

```text
- type(scope): 간결한 영어 요약.
- English detail 1.
- Korean detail 2.
```

## Code review rules

- Flag any path that bypasses deterministic safety checks or treats policy output as trusted.
- Flag nondeterministic simulation behavior without a recorded seed and configuration.
- Flag blocking work on async executors, panic paths reachable from external input, and reconnect logic that can replay stale commands.

## Recommended Codex profile

- Default: `gpt-5.6-sol` with `high` reasoning.
- Use `xhigh` for concurrency, determinism, policy-runtime compatibility, and safety-critical reviews.
