# MAPF-RL Simulator

MAPF-RL의 Rust/Tokio virtual-robot runtime이다. Core contract를 사용하고 local inference,
deterministic simulation과 최종 safety authority를 소유한다.

Phase 1은 Core 없이 실행 가능한 deterministic single-robot engine을 제공한다. Engine은 100 ms
fixed tick, typed simulation time/coordinate boundary, Ruckig 기반 jerk-limited holonomic cardinal
motion, circle-footprint safety, 실제 비상 정지 궤적까지 포함한 static clearance, current-tick lazy
fault 평가, seeded sensor/fault stream과 canonical state digest를 포함한다. ONNX runtime은 후속
vertical slice 범위다. 같은 tick의 비상 transition도 안전하지 않으면 부분 state를 남기지 않고
scenario를 `CollisionInvariant`로 종료한다.

Phase 2는 Core contract `1.0.0`의 authenticated REST/WS client, durable session fencing과
process-global report spool, single Order 멱등 적용, application ack/execution report 분리,
10 Hz state fast path, reconnect/replay와 5초 REST recovery를 제공한다. Baseline controller는
`local`/`dev`에서만 명시적으로 선택되며 identity와 digest가 모든 state report에 포함된다.
WS write만으로 report를 제거하지 않고 matching `report.ack` 또는 REST batch outcome의
`ACCEPTED`/`DUPLICATE`를 확인한 뒤 제거한다.

공식 `ruckig` Rust crate는 upstream C++ 구현을 빌드하므로 Rust toolchain과 함께 C++20 compiler가
필요하다. Motion capability는 speed, acceleration/deceleration과 정상·비상 jerk를 모두 명시한다.

Standalone scenario harness:

```bash
cargo run
```

동일 scenario와 seed는 wall clock이나 실행 thread와 무관하게 같은 SHA-256 state digest를 출력한다.

```bash
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
```

계약 변경은 먼저 sibling Core 저장소의 `scripts/contracts.py generate`로 생성물을 갱신한다.
