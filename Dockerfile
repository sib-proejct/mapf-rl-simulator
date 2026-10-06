FROM rust:1.90.0-bookworm AS build
RUN apt-get update && apt-get install -y --no-install-recommends clang && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY . .
RUN CARGO_BUILD_JOBS=1 CXX=clang++ cargo build --locked --release
FROM python:3.12.13-slim-bookworm
RUN apt-get update && apt-get install -y --no-install-recommends libstdc++6 ca-certificates && rm -rf /var/lib/apt/lists/*
COPY --from=build /app/target/release/mapf-rl-simulator /usr/local/bin/mapf-rl-simulator
COPY scripts/container.py scripts/map_worker.py /app/
ENTRYPOINT ["python", "/app/container.py"]
