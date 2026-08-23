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

Operational mode는 Core snapshot의 authoritative map을 Phase 1 engine에 적용하고, Order goal까지
deterministic safety boundary를 거쳐 이동한 뒤 `robot.event.report`의 `ORDER_COMPLETED`를 durable
spool에 기록한다. Core가 completion을 수용한 ack를 보낸 경우에만 local applied-Order checkpoint를
해제한다.

공식 `ruckig` Rust crate는 upstream C++ 구현을 빌드하므로 Rust toolchain과 함께 C++20 compiler가
필요하다. Motion capability는 speed, acceleration/deceleration과 정상·비상 jerk를 모두 명시한다.

Standalone scenario harness:

```bash
cargo run
```

Core-connected operational binary:

```bash
MAPF_SIMULATOR_MODE=core \
MAPF_PROFILE=local \
MAPF_SIMULATOR_ID=sim-01 \
MAPF_SIMULATOR_ROBOT_ID=robot-01 \
MAPF_SIMULATOR_CORE_REST_URL=http://127.0.0.1:8000/ \
MAPF_SIMULATOR_CORE_WS_URL=ws://127.0.0.1:8000/ws/v1 \
MAPF_SIMULATOR_API_KEY='injected-secret' \
MAPF_SIMULATOR_SPOOL_PATH=/var/lib/mapf-rl-simulator/spool.json \
MAPF_SIMULATOR_MAP_ID=00000000-0000-4000-8000-000000000001 \
MAPF_SIMULATOR_MAP_REVISION=1 \
MAPF_SIMULATOR_MAP_DIGEST_SHA256='64-hex-digest' \
cargo run
```

필수 identity/map 값과 Core endpoint/API key가 누락되거나 Core snapshot의 map
identity/content가 일치하지 않으면 motion을 시작하기 전에 실패한다. 테스트용 시작 cell과 seed는
각각 `MAPF_SIMULATOR_START_COLUMN`, `MAPF_SIMULATOR_START_ROW`,
`MAPF_SIMULATOR_MASTER_SEED`로 지정할 수 있다.

동일 scenario와 seed는 wall clock이나 실행 thread와 무관하게 같은 SHA-256 state digest를 출력한다.

```bash
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
```

계약 변경은 먼저 sibling Core 저장소의 `scripts/contracts.py generate`로 생성물을 갱신한다.
