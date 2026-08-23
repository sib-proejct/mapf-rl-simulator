use reqwest::StatusCode;
use serde::Deserialize;
use serde_json::json;
use std::fs;
use std::net::TcpListener;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};
use tempfile::TempDir;
use tokio::time::sleep;
use uuid::Uuid;

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ConnectionInfo {
    api_key: String,
    map_id: String,
    map_digest_sha256: String,
}

struct ChildGuard(Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[tokio::test]
#[ignore = "requires localhost process networking"]
async fn core_and_operational_simulator_complete_one_order_as_separate_processes() {
    let simulator_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let core_root = simulator_root.parent().unwrap().join("mapf-rl-core");
    let python = core_root.join(".venv/bin/python");
    assert!(
        python.is_file(),
        "Core test virtual environment is required"
    );
    let temp = TempDir::new().unwrap();
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    drop(listener);

    let core = Command::new(python)
        .arg("tests/e2e_core_process.py")
        .arg(port.to_string())
        .arg(temp.path())
        .current_dir(&core_root)
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut core = ChildGuard(core);
    let info_path = temp.path().join("connection.json");
    wait_until(Duration::from_secs(10), || info_path.is_file()).await;
    let info: ConnectionInfo = serde_json::from_slice(&fs::read(&info_path).unwrap()).unwrap();

    let rest_base = format!("http://127.0.0.1:{port}/");
    let client = reqwest::Client::new();
    wait_for_core(&client, &rest_base).await;

    let simulator = Command::new(env!("CARGO_BIN_EXE_mapf-rl-simulator"))
        .env("MAPF_SIMULATOR_MODE", "core")
        .env("MAPF_PROFILE", "local")
        .env("MAPF_SIMULATOR_ID", "sim-1")
        .env("MAPF_SIMULATOR_ROBOT_ID", "robot-1")
        .env("MAPF_SIMULATOR_CORE_REST_URL", &rest_base)
        .env(
            "MAPF_SIMULATOR_CORE_WS_URL",
            format!("ws://127.0.0.1:{port}/ws/v1"),
        )
        .env("MAPF_SIMULATOR_API_KEY", &info.api_key)
        .env("MAPF_SIMULATOR_SPOOL_PATH", temp.path().join("spool.json"))
        .env("MAPF_SIMULATOR_MAP_ID", &info.map_id)
        .env("MAPF_SIMULATOR_MAP_REVISION", "1")
        .env("MAPF_SIMULATOR_MAP_DIGEST_SHA256", &info.map_digest_sha256)
        .env("MAPF_SIMULATOR_START_COLUMN", "1")
        .env("MAPF_SIMULATOR_START_ROW", "2")
        .env("MAPF_SIMULATOR_EXIT_AFTER_COMPLETION", "true")
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut simulator = ChildGuard(simulator);
    wait_for_simulator_session(&client, &rest_base, &info.api_key).await;

    let request_id = Uuid::new_v4();
    let response = client
        .post(format!("{rest_base}api/v1/orders"))
        .header("Cookie", "__Host-mapf_session=operator-session")
        .header("X-CSRF-Token", "csrf-token")
        .json(&json!({
            "requestId": request_id,
            "mapId": info.map_id,
            "mapRevision": 1,
            "assignments": [{"robotId": "robot-1", "goalColumn": 3, "goalRow": 2}]
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(response.status(), StatusCode::ACCEPTED);
    let order_id = response.json::<serde_json::Value>().await.unwrap()["orderId"]
        .as_str()
        .unwrap()
        .to_owned();

    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let snapshot = client
            .get(format!("{rest_base}api/v1/operations/snapshot"))
            .header("Cookie", "__Host-mapf_session=operator-session")
            .send()
            .await
            .unwrap()
            .json::<serde_json::Value>()
            .await
            .unwrap();
        let completed = snapshot["entities"]
            .as_array()
            .unwrap()
            .iter()
            .any(|entity| {
                entity["entityType"] == "ORDER"
                    && entity["entityId"] == order_id
                    && entity["data"]["state"] == "Completed"
            });
        if completed {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "Order did not complete before timeout"
        );
        sleep(Duration::from_millis(100)).await;
    }

    wait_until(Duration::from_secs(5), || {
        simulator.0.try_wait().unwrap().is_some()
    })
    .await;
    assert!(simulator.0.try_wait().unwrap().unwrap().success());
    core.0.kill().unwrap();
}

async fn wait_for_core(client: &reqwest::Client, rest_base: &str) {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if client
            .get(format!("{rest_base}api/v1/operations/snapshot"))
            .header("Cookie", "__Host-mapf_session=operator-session")
            .send()
            .await
            .is_ok_and(|response| response.status().is_success())
        {
            return;
        }
        assert!(Instant::now() < deadline, "Core process did not start");
        sleep(Duration::from_millis(50)).await;
    }
}

async fn wait_for_simulator_session(client: &reqwest::Client, rest_base: &str, api_key: &str) {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        if client
            .get(format!("{rest_base}api/v1/simulators/sim-1/snapshot"))
            .header("X-API-Key", api_key)
            .send()
            .await
            .is_ok_and(|response| response.status().is_success())
        {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "Simulator did not establish a Core session"
        );
        sleep(Duration::from_millis(50)).await;
    }
}

async fn wait_until(mut timeout: Duration, mut condition: impl FnMut() -> bool) {
    while !condition() {
        assert!(!timeout.is_zero(), "condition timed out");
        let step = timeout.min(Duration::from_millis(25));
        sleep(step).await;
        timeout = timeout.saturating_sub(step);
    }
}
