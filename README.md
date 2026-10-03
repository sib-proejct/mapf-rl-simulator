# MAPF-RL Simulator

## 전체 로컬 환경 실행 (Docker Compose)

Sibling 저장소가 같은 parent 아래 있고 Infra `.env`의 DB/Redis 설정이 준비되어 있으면:

```sh
cd ../mapf-rl-infra
docker compose up -d
scripts/smoke-test.sh
```

최초 실행은 이미지를 빌드하고 PostgreSQL/Redis health → Core migration·seed → API/Planner →
FE/Simulator 순서로 시작한다. 초기 빌드에는 네트워크 접근과 시간이 필요하다.
`http://127.0.0.1:8000/local/login`을 열고 FE에서 **Live WS**를 선택한다.
`r-001`이 준비되면 목표 **column 14, row 8**로 Order를 만든다.
시작은 **column 4, row 2**이며 재시작 후에는 저장된 checkpoint를 복원한다.

맵은 기본 Fixture 10Hz와 같은 **32×20, 1m/cell 창고**다. Core의
`scripts/warehouse-map.json`이 원본이며 `scripts/contracts.py generate`/`check`가 FE JSON 복사본을
동기화/검사한다. Fixture의 가상 로봇·Order·이벤트는 Live 환경에 주입하지 않는다.
실제 Core Planner와 Simulator가 로봇 한 대를 운용한다.

코드 변경 후에는 `docker compose up -d --build`, 로그는
`docker compose logs --tail=100 core planner simulator fe`로 확인한다.
`docker compose stop` 또는 `docker compose down`은 데이터를 유지한다.
**`down --volumes`를 실행하지 않는다.** 기존 7×5 데모 상태도 그대로 보존한다.

DB/Redis는 Core 전용 네트워크에 있고 FE/Simulator는 별도 client 네트워크에서 Core에만 연결한다.
모든 host 포트는 `127.0.0.1`에 바인딩한다. Simulator의 `MAPF_SIMULATOR_LOCAL_COMPOSE=true`는 `local` profile에서
`http://core:8000`/`ws://core:8000`만 추가로 허용한다.
컨테이너용 local 인증도 Host/Origin/session/CSRF를
검증하며 `local` profile에서만 허용된다. 기본 client subnet은 `172.30.50.0/24`이며 충돌 시
Infra `.env`의 `MAPF_DEMO_SUBNET`을 변경한다. 이 구성은 운영 배포용이 아니다.

Pepper, Simulator credential, spool/checkpoint는 서로 구분된 named volume에 보존된다.
DB와 credential/pepper를 함께 유지해야 한다. 기존 파일을 지우거나 checkpoint를 삭제해 복구를
우회하지 않는다. API를 재시작하면 로컬 browser session이 바뀌므로 login URL을 다시 연다.


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

Operational mode는 Core snapshot의 authoritative map을 Phase 1 engine에 적용한다. Plan command의
`PREPARE`에서 goal과 time-indexed route를 durable spool에 저장하고 safe hold를 확인하며,
`ACTIVATE`에서는 command payload로 실행 정보를 다시 만들지 않고 저장된 prepared state 전체를
`AppliedOrder`로 원자 승격한다. 운영 loop는 route follower, `PlanCoordinator`, `MultiRobotEngine`과
fenced safety checkpoint를 조합해 deterministic safety boundary를 통과한 action만 적용한다. Route
deviation, collision/corridor/deadlock recovery는 local hold와 durable critical incident를 만든다.
Core가 `ORDER_COMPLETED`를 수용한 ack를 보낸 경우에만 local applied-Order checkpoint를 해제한다.
프로세스 재시작에서 active checkpoint가 복원되면 motion authority는 계속 fenced되고, 복원된 state를
먼저 보고한 뒤 stable `SAFETY_RESTART_RECONCILIATION_REQUIRED` incident로 Core replan을 요청한다.

각 Core session credential은 한 operational robot을 bind하고 live API도 Order당 한 assignment로
제한된다. 로컬 Compose의 `MAPF_SIMULATOR_MODE=fleet`은 로봇별 session과 독립 plan을 하나의
`MultiRobotEngine`에서 함께 실행한다. FE에서 생성한 로봇은 tick 경계에서 시작 위치와 정지 궤적을
검사한 뒤 추가한다. 기존 `MAPF_SIMULATOR_MODE=core` 단일 로봇 실행 모드도 유지한다.

Fleet 모드는 `MAPF_SIMULATOR_RUNTIME_KEY_PATH`의 별도 로컬 runtime credential을 사용한다.
robot credential은 Core의 `robot-provisioning 1.0.0` claim 응답으로 받으며 FE에 노출하지 않는다.
`fleet-checkpoint.json`은 모든 robot의 membership·물리 상태·plan·station 상태를 원자적으로 보존한다.
WS 재접속은 물리 tick 밖에서 실행하며 로봇별 분산 지수 백오프(1초 이상, 최대 30초)를 적용한다.
handshake·welcome 수신·socket 쓰기는 각각 3초 timeout을 사용한다. runtime lease 갱신은
tick·초기화와 독립적으로 실행하며 마지막 성공 요청 시작부터 10초간 갱신되지 않으면 종료한다.
기존 `checkpoint.json`과 `spool.json`은 삭제하지 않고 최초 fleet 전환에서 사용한다. Core migration
`20261003_0007`을 먼저 적용해야 하며 운영 환경 자동 provisioning은 지원하지 않는다.

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
MAPF_SIMULATOR_CHECKPOINT_PATH=/var/lib/mapf-rl-simulator/safety-checkpoint.json \
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

### Arrival actions

The runtime advertises station actions 1.1.0 and executes PICK/PLACE/CHARGE only
after a safe, stopped arrival. Load, charge and progress are checkpointed before
reporting completion. Disconnects and safety stops pause simulation-time progress.
Defaults: `MAPF_SIMULATOR_PICK_DURATION_MS=2000`,
`MAPF_SIMULATOR_PLACE_DURATION_MS=2000`,
`MAPF_SIMULATOR_CHARGE_PERCENT_PER_SECOND=1` (percentage points/s),
`MAPF_SIMULATOR_INITIAL_BATTERY_PERCENT=100` (used only without a restored checkpoint).
Roll out compatible Core/migration first, then Simulator, then FE.


## 배터리 및 자동 충전

배터리 소모와 20% 이하 자동 충전을 지원한다. 진행 중인 작업은 완료한 뒤 충전하며 새 일반 작업은 제한한다. Core가 사용 가능한 충전소에 기존 CHARGE Order를 배정하고 Simulator가 100%까지 충전한다. 0%에서는 안전 정지하며 운영자 복구가 필요하다. 충전소가 없거나 점유 중이면 기다리고 재평가한다. 자세한 계약과 제한은 [Live WS 기능 현황](../mapf-rl-docs/LIVE-WS-CAPABILITIES.md#3-배터리-로직)을 참고한다.

| 환경변수 | 기본값 | 단위 |
|---|---|---|
| `MAPF_SIMULATOR_DRIVE_PERCENT_PER_METER` | 0.1 | %p/m |
| `MAPF_SIMULATOR_IDLE_PERCENT_PER_SECOND` | 0.001 | %p/s |
| `MAPF_SIMULATOR_LOADED_BATTERY_MULTIPLIER` | 1.5 | 적재 주행 거리 소모 배율 |

소모율은 유한한 0 이상 값, 적재 배율은 유한한 1 이상 값이어야 한다. 안전하게 충전하는 tick에는 소모하지 않는다. 초기 잔량·충전 속도 설정과 기존 checkpoint 우선 복원은 유지한다. 고갈 incident metadata가 있는 새 checkpoint는 이전 버전으로 downgrade할 때 호환되지 않으므로 원본을 보존한다.
