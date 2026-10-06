"""Generated consumers use this Core-owned local runtime supervisor protocol."""

import json
import os
import signal
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path


def run_worker(role, command, environment, key_path):
    key = Path(key_path).read_text().strip()
    base = os.environ.get("MAPF_MAP_CONTROL_URL", "http://core:8000/api/v1/local-maps/runtime")
    child = None
    token = None
    stopped = False
    frozen = False
    last_ack = None

    def shutdown(*_):
        nonlocal stopped
        stopped = True

    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, shutdown)

    def request(body=None):
        headers = {"X-Runtime-Key": key, "Content-Type": "application/json"}
        req = urllib.request.Request(
            base + ("/ack" if body else ""),
            data=json.dumps(body).encode() if body else None,
            headers=headers,
        )
        with urllib.request.urlopen(req, timeout=4) as response:
            return json.load(response)

    def terminate():
        nonlocal child, frozen, token
        if child is not None:
            if frozen:
                child.send_signal(signal.SIGCONT)
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        child, token, frozen = None, None, False

    try:
        while not stopped:
            try:
                state = request()
                current = (state["bootId"], state["generation"], state["mapType"])
                phase = state["phase"]
                should_run = phase == "ACTIVE" or phase == "PREPARING" and role == "simulator"
                if child is not None and (token != current or not should_run):
                    terminate()
                if phase == "STOPPING":
                    ack = (*current, "STOPPED")
                    if last_ack != ack:
                        request(
                            dict(
                                contractVersion="1.0.0",
                                role=role,
                                generation=state["generation"],
                                bootId=state["bootId"],
                                state="STOPPED",
                            )
                        )
                        last_ack = ack
                elif should_run:
                    if child is None:
                        env = environment(state)
                        env["MAPF_MAP_GENERATION"] = str(state["generation"])
                        child = subprocess.Popen(command, env=env)
                        token = current
                    if child.poll() is not None:
                        raise RuntimeError("runtime child exited")
                    if phase == "PREPARING" and state["fleetReady"]:
                        if not frozen:
                            child.send_signal(signal.SIGSTOP)
                            frozen = True
                        ack = (*current, "READY")
                        if last_ack != ack:
                            request(
                                dict(
                                    contractVersion="1.0.0",
                                    role=role,
                                    generation=state["generation"],
                                    bootId=state["bootId"],
                                    state="READY",
                                )
                            )
                            last_ack = ack
                    elif phase == "ACTIVE" and frozen:
                        child.send_signal(signal.SIGCONT)
                        frozen = False
            except (OSError, ValueError, RuntimeError, urllib.error.URLError):
                # Loss of control authority stops the virtual runtime, never runs both maps.
                terminate()
                last_ack = None
            time.sleep(0.5)
    finally:
        terminate()
