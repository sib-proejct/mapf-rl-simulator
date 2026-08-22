# ADR 0003: Phase 2 Core session and report spool

- Status: Accepted
- Date: 2026-08-22

## Context

The Simulator must consume Core contract `1.0.0` over authenticated REST and WebSocket,
fence an older session, apply one Order idempotently, and retain reports until Core has
durably classified them. A WebSocket write only proves transport handoff. Process restart
must not erase an unacknowledged report or reuse its stable identity.

## Decision

- `CoreSession` owns the active `sessionEpoch`, reconciliation gate, Core stream cursor,
  single-Order checkpoint, and explicit 100 ms / 5 second schedules.
- `simulatorBootId` is UUIDv4 process identity. `reportSequence` starts at zero for each
  new boot and is allocated from one simulator-wide spool, not per robot or report class.
- The spool is a versioned JSON document with a SHA-256 checksum. Updates use a temporary
  file, file sync, atomic rename, and parent-directory sync. Filesystem calls run through
  the runtime's blocking boundary.
- Reconnect rebinds only reports created by the current process boot to the new epoch.
  Reports recovered from an earlier process retain their original epoch, boot, sequence,
  message ID, request ID, and payload for historical recovery submission.
- `ACCEPTED` and `DUPLICATE` report outcomes remove an entry. Retryable `REJECTED` retains
  it. Non-retryable `REJECTED` moves it to the local dead-letter audit. WebSocket write
  success never changes the spool.
- The application Order checkpoint is persisted before the coordinator returns an
  apply decision. A matching redelivery returns `DUPLICATE` and never applies the Order
  to the robot a second time. The command ack and later execution state are separate
  reports with separate process-global sequences.
- The async transport uses `X-API-Key` for both REST and the `mapf.v1` WebSocket handshake.
  Redirects are disabled. Insecure HTTP/WS is accepted only for loopback `local`; `dev`
  and `production` require HTTPS/WSS. The API key has a redacted debug representation.
- WebSocket is the 10 Hz state fast path. When disconnected, authenticated REST snapshot
  and bounded report batches are attempted on the explicit five-second fallback schedule.

## Consequences

The local checkpoint accelerates recovery but never becomes Core authority. A recovered
old-epoch report can be fenced by Core and moved to dead-letter; it cannot update current
projection state. Spool exhaustion or persistence failure is an explicit runtime error and
therefore cannot be treated as successful telemetry or command application.
