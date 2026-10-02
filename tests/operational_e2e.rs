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
async fn core_and_wave3_operational_simulator_complete_one_planned_route() {
    run_scenario(false, false).await;
}
#[tokio::test]
async fn station_pick_place_and_charge_complete_after_arrival() {
    run_scenario(true, false).await;
}
#[tokio::test]
async fn live_cancellation_stops_a_moving_robot() {
    run_scenario(false, true).await;
}
// Each scenario drives wall-clock processes against simulation-time route windows.
// Run them sequentially so competing local servers do not consume the prepare window.
static OPERATIONAL_E2E_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

async fn run_scenario(station_actions: bool, cancellation: bool) {
    let _guard = OPERATIONAL_E2E_LOCK.lock().await;
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
        .arg(if station_actions { "stations" } else { "move" })
        .current_dir(&core_root)
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
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
        .env(
            "MAPF_SIMULATOR_CHECKPOINT_PATH",
            temp.path().join("checkpoint.json"),
        )
        .env("MAPF_SIMULATOR_MAP_ID", &info.map_id)
        .env("MAPF_SIMULATOR_MAP_REVISION", "1")
        .env("MAPF_SIMULATOR_MAP_DIGEST_SHA256", &info.map_digest_sha256)
        .env("MAPF_SIMULATOR_START_COLUMN", "1")
        .env("MAPF_SIMULATOR_START_ROW", "2")
        .env(
            "MAPF_SIMULATOR_EXIT_AFTER_COMPLETION",
            if station_actions || cancellation {
                "false"
            } else {
                "true"
            },
        )
        .env("MAPF_SIMULATOR_INITIAL_BATTERY_PERCENT", "99")
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .unwrap();
    let mut simulator = ChildGuard(simulator);
    wait_for_simulator_session(&client, &rest_base, &info.api_key).await;

    let steps = if station_actions {
        vec![(3, Some("PICK")), (4, Some("PLACE")), (5, Some("CHARGE"))]
    } else {
        vec![(3, None)]
    };
    for (column, arrival_action) in steps {
        let idle_deadline = Instant::now() + Duration::from_secs(5);
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
            if snapshot["entities"]
                .as_array()
                .unwrap()
                .iter()
                .any(|entity| {
                    entity["entityType"] == "ROBOT"
                        && entity["data"]["operationalState"] == "IDLE"
                        && entity["data"]["safety"] == "NORMAL"
                })
            {
                break;
            }
            assert!(
                Instant::now() < idle_deadline,
                "robot did not become idle: {snapshot}"
            );
            sleep(Duration::from_millis(100)).await;
        }
        let request_id = Uuid::new_v4();
        let mut assignment = json!({"robotId": "robot-1", "goalColumn": column, "goalRow": 2});
        if let Some(action) = arrival_action {
            assignment["arrivalAction"] = json!(action);
        }
        let response = client
            .post(format!("{rest_base}api/v1/orders"))
            .header("Cookie", "__Host-mapf_session=operator-session")
            .header("X-CSRF-Token", "csrf-token")
            .json(&json!({
                "requestId": request_id,
                "mapId": info.map_id,
                "mapRevision": 1,
                "assignments": [assignment]
            }))
            .send()
            .await
            .unwrap();
        let status = response.status();
        if status != StatusCode::ACCEPTED {
            panic!(
                "Order {arrival_action:?} was rejected: {status} {}",
                response.text().await.unwrap()
            );
        }
        let order_id = response.json::<serde_json::Value>().await.unwrap()["orderId"]
            .as_str()
            .unwrap()
            .to_owned();

        let deadline = Instant::now() + Duration::from_secs(20);
        let mut cancel_requested = false;
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
            if cancellation {
                let order = snapshot["entities"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .find(|e| e["entityType"] == "ORDER" && e["entityId"] == order_id)
                    .unwrap();
                if !cancel_requested && order["data"]["state"] == "Executing" {
                    let response = client.post(format!("{rest_base}api/v1/orders/{order_id}/cancel"))
                        .header("Cookie", "__Host-mapf_session=operator-session")
                        .header("X-CSRF-Token", "csrf-token")
                        .json(&json!({"requestId": Uuid::new_v4(), "orderUpdateId": order["data"]["orderUpdateId"], "reason": "E2E cancellation"}))
                        .send().await.unwrap();
                    assert_eq!(
                        response.status(),
                        StatusCode::ACCEPTED,
                        "{}",
                        response.text().await.unwrap()
                    );
                    cancel_requested = true;
                }
                if order["data"]["state"] == "Cancelled" {
                    assert!(cancel_requested);
                    let checkpoint: serde_json::Value = serde_json::from_slice(
                        &fs::read(temp.path().join("checkpoint.json")).unwrap(),
                    )
                    .unwrap();
                    let robot = &checkpoint["body"]["checkpoint"]["robots"][0];
                    for field in [
                        "velocityXMps",
                        "velocityYMps",
                        "accelerationXMps2",
                        "accelerationYMps2",
                    ] {
                        assert!(
                            robot[field].as_f64().unwrap().abs() < 1.0e-6,
                            "{checkpoint}"
                        );
                    }
                    simulator.0.kill().unwrap();
                    return;
                }
                assert!(
                    Instant::now() < deadline,
                    "Cancellation timed out: {snapshot}"
                );
                sleep(Duration::from_millis(100)).await;
                continue;
            }
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
                let robot = snapshot["entities"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .find(|entity| {
                        entity["entityType"] == "ROBOT" && entity["entityId"] == "robot-1"
                    })
                    .expect("completed order must include robot state");
                let pose = &robot["data"]["pose"];
                assert!(
                    (pose["xMeters"].as_f64().unwrap() - (f64::from(column) + 0.5)).abs() < 1.0e-6,
                    "{robot}"
                );
                assert!(
                    (pose["yMeters"].as_f64().unwrap() - 2.5).abs() < 1.0e-6,
                    "{robot}"
                );
                if let Some(action) = arrival_action {
                    let state = &robot["data"]["stationState"];
                    assert_eq!(state["action"], action);
                    assert_eq!(state["phase"], "COMPLETED");
                    assert_eq!(state["loaded"], action == "PICK");
                    if action == "CHARGE" {
                        assert_eq!(state["batteryPercent"], 100.0);
                    }
                }
                break;
            }
            assert!(
                Instant::now() < deadline,
                "Order did not complete before timeout: {snapshot}"
            );
            sleep(Duration::from_millis(100)).await;
        }
    }
    if station_actions {
        simulator.0.kill().unwrap();
        return;
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
        let response = client
            .get(format!("{rest_base}api/v1/simulators/sim-1/snapshot"))
            .header("X-API-Key", api_key)
            .send()
            .await;
        if let Ok(response) = response
            && response.status().is_success()
            && let Ok(snapshot) = response.json::<serde_json::Value>().await
            && snapshot["robots"]
                .as_array()
                .is_some_and(|robots| robots.iter().any(|robot| robot["ready"] == true))
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
