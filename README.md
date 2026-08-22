# MAPF-RL Simulator

MAPF-RL의 Rust/Tokio virtual-robot runtime이다. Core contract를 사용하고 local inference,
deterministic simulation과 최종 safety authority를 소유한다.

Phase 1은 Core 없이 실행 가능한 deterministic single-robot engine을 제공한다. Engine은 100 ms
fixed tick, typed simulation time/coordinate boundary, holonomic cardinal motion, circle-footprint safety,
제어·비상 감속의 tick 적분, 비상 정지거리까지 포함한 static clearance, current-tick lazy fault 평가,
seeded sensor/fault stream과 canonical state digest를 포함한다. Network와 ONNX runtime은 후속
vertical slice 범위다. 같은 tick의 비상 transition도 안전하지 않으면 부분 state를 남기지 않고
scenario를 `CollisionInvariant`로 종료한다.

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
