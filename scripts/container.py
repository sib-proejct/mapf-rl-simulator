"""Load only Core-issued demo credentials; preserve safety recovery state."""
import json
import os
from pathlib import Path

info = json.loads(Path("/run/mapf-demo/connection.json").read_text())
os.environ.update({
    "MAPF_SIMULATOR_LOCAL_COMPOSE": "true",
    "MAPF_SIMULATOR_RUNTIME_KEY_PATH": "/run/mapf-demo/runtime-key",
    "MAPF_SIMULATOR_MODE": "fleet", "MAPF_PROFILE": "local",
    "MAPF_SIMULATOR_ID": info["simulatorId"],
    "MAPF_SIMULATOR_ROBOT_ID": info["robotId"],
    "MAPF_SIMULATOR_API_KEY": info["apiKey"],
    "MAPF_SIMULATOR_CORE_REST_URL": "http://core:8000/",
    "MAPF_SIMULATOR_CORE_WS_URL": "ws://core:8000/ws/v1",
    "MAPF_SIMULATOR_MAP_ID": info["mapId"],
    "MAPF_SIMULATOR_MAP_REVISION": "1",
    "MAPF_SIMULATOR_MAP_DIGEST_SHA256": info["mapDigestSha256"],
    "MAPF_SIMULATOR_SPOOL_PATH": "/var/lib/mapf-simulator/spool.json",
    "MAPF_SIMULATOR_CHECKPOINT_PATH": "/var/lib/mapf-simulator/checkpoint.json",
    "MAPF_SIMULATOR_START_COLUMN": "4", "MAPF_SIMULATOR_START_ROW": "2",
    "MAPF_SIMULATOR_EXIT_AFTER_COMPLETION": "false",
})
os.execvp("mapf-rl-simulator", ["mapf-rl-simulator"])
