# MAPF-RL Simulator

MAPF-RL의 Rust/Tokio virtual-robot runtime이다. Core contract를 사용하고 local inference,
deterministic simulation과 최종 safety authority를 소유한다.

현재 Phase 0 scaffold는 dependency-free generated contract representation과 drift/compile test만
포함한다. Network, motion, safety와 ONNX runtime은 확정 schema/fixture를 바탕으로 후속 vertical
slice에서 추가한다.

```bash
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
```

계약 변경은 먼저 sibling Core 저장소의 `scripts/contracts.py generate`로 생성물을 갱신한다.
