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
    run_scenario(false, false, BatteryScenario::Normal, false, false).await;
}
#[tokio::test]
async fn station_pick_place_and_charge_complete_after_arrival() {
    run_scenario(true, false, BatteryScenario::Normal, false, false).await;
}
#[tokio::test]
async fn live_cancellation_stops_a_moving_robot() {
    run_scenario(false, true, BatteryScenario::Normal, false, false).await;
}
#[tokio::test]
async fn queued_wave_runs_pick_place_pairs_on_the_same_robot() {
    run_scenario(true, false, BatteryScenario::Normal, false, true).await;
}

// Each scenario drives wall-clock processes against simulation-time route windows.
// Run them sequentially so competing local servers do not consume the prepare window.
static OPERATIONAL_E2E_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

#[derive(Clone, Copy, PartialEq)]
enum BatteryScenario {
    Normal,
    AutoCharge,
    Deplete,
}

#[tokio::test]
async fn low_battery_finishes_work_then_automatically_travels_and_charges() {
    run_scenario(true, false, BatteryScenario::AutoCharge, false, false).await;
}

#[tokio::test]
async fn battery_depletion_stops_motion_and_reports_one_incident() {
    run_scenario(true, false, BatteryScenario::Deplete, false, false).await;
}

#[tokio::test]
async fn fleet_keeps_renewing_lease_and_provisions_a_robot() {
    run_scenario(false, false, BatteryScenario::Normal, true, false).await;
}

async fn run_scenario(
    station_actions: bool,
    cancellation: bool,
    battery: BatteryScenario,
    fleet_mode: bool,
    queue_mode: bool,
) {
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
        .env(
            "MAPF_SIMULATOR_MODE",
            if fleet_mode { "fleet" } else { "core" },
        )
        .env(
            "MAPF_SIMULATOR_RUNTIME_KEY_PATH",
            temp.path().join("runtime-key"),
        )
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
        .env(
            "MAPF_SIMULATOR_INITIAL_BATTERY_PERCENT",
            if battery == BatteryScenario::Normal {
                "99"
            } else {
                "21"
            },
        )
        .env(
            "MAPF_SIMULATOR_DRIVE_PERCENT_PER_METER",
            match battery {
                BatteryScenario::Normal => "0.1",
                BatteryScenario::AutoCharge => "1",
                BatteryScenario::Deplete => "100",
            },
        )
        .env(
            "MAPF_SIMULATOR_CHARGE_PERCENT_PER_SECOND",
            if battery == BatteryScenario::AutoCharge {
                "20"
            } else {
                "1"
            },
        )
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .unwrap();
    let mut simulator = ChildGuard(simulator);
    wait_for_simulator_session(&client, &rest_base, &info.api_key).await;

    if queue_mode {
        let command = json!({
            "requestId": Uuid::new_v4(), "mapId": info.map_id, "mapRevision": 1,
            "tasks": [
                {"robotId": "robot-1", "steps": [
                    {"goalColumn": 3, "goalRow": 2, "arrivalAction": "PICK"},
                    {"goalColumn": 4, "goalRow": 2, "arrivalAction": "PLACE"}]},
                {"steps": [
                    {"goalColumn": 3, "goalRow": 2, "arrivalAction": "PICK"},
                    {"goalColumn": 4, "goalRow": 2, "arrivalAction": "PLACE"}]}
            ]
        });
        let mut accepted = None;
        for _ in 0..2 {
            let response = client
                .post(format!("{rest_base}api/v1/waves"))
                .header("Cookie", "__Host-mapf_session=operator-session")
                .header("X-CSRF-Token", "csrf-token")
                .json(&command)
                .send()
                .await
                .unwrap();
            assert_eq!(response.status(), StatusCode::ACCEPTED);
            let outcome = response.json::<serde_json::Value>().await.unwrap();
            if let Some(previous) = &accepted {
                assert_eq!(previous, &outcome);
            }
            accepted = Some(outcome);
        }
        let deadline = Instant::now() + Duration::from_secs(60);
        loop {
            let response = client
                .get(format!("{rest_base}api/v1/queue/tasks"))
                .header("Cookie", "__Host-mapf_session=operator-session")
                .send()
                .await
                .unwrap()
                .json::<serde_json::Value>()
                .await
                .unwrap();
            let tasks = response["tasks"].as_array().unwrap();
            assert_eq!(tasks.len(), 2);
            if tasks.iter().all(|task| task["state"] == "Completed") {
                for task in tasks {
                    assert_eq!(task["robotId"], "robot-1");
                    assert_eq!(task["stage"], 1);
                    assert_eq!(task["orderIds"].as_array().unwrap().len(), 2);
                }
                assert_eq!(response["waves"][0]["counts"]["Completed"], 2);
                break;
            }
            assert!(
                Instant::now() < deadline,
                "Queued wave timed out: {response}"
            );
            sleep(Duration::from_millis(100)).await;
        }
        wait_for_buffer_waiting(&client, &rest_base).await;
        simulator.0.kill().unwrap();
        core.0.kill().unwrap();
        return;
    }

    let steps = if station_actions && battery == BatteryScenario::Normal {
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
            if battery == BatteryScenario::Deplete {
                let incidents: Vec<_> = snapshot["entities"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .filter(|entity| {
                        entity["entityType"] == "INCIDENT"
                            && entity["data"]["code"] == "BATTERY_DEPLETED"
                    })
                    .collect();
                if !incidents.is_empty() {
                    assert_eq!(incidents.len(), 1);
                    let checkpoint: serde_json::Value = serde_json::from_slice(
                        &fs::read(temp.path().join("checkpoint.json")).unwrap(),
                    )
                    .unwrap();
                    let robot = &checkpoint["body"]["checkpoint"]["robots"][0];
                    if robot["velocityXMps"].as_f64().unwrap().abs() < 1e-6
                        && robot["accelerationXMps2"].as_f64().unwrap().abs() < 1e-6
                    {
                        assert_eq!(
                            checkpoint["body"]["checkpoint"]["stationStates"]["robot-1"]["batteryPercent"],
                            0.0
                        );
                        assert!(robot["xMeters"].as_f64().unwrap() < 3.5);
                        simulator.0.kill().unwrap();
                        return;
                    }
                }
                assert!(
                    Instant::now() < deadline,
                    "Depletion did not stop: {snapshot}"
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
                // Low-battery recovery may already be driving to the charger
                // when the next snapshot observes this completed Order.
                if battery != BatteryScenario::AutoCharge {
                    let pose = &robot["data"]["pose"];
                    assert!(
                        (pose["xMeters"].as_f64().unwrap() - (f64::from(column) + 0.5)).abs()
                            < 1.0e-6,
                        "{robot}"
                    );
                    assert!(
                        (pose["yMeters"].as_f64().unwrap() - 2.5).abs() < 1.0e-6,
                        "{robot}"
                    );
                }
                if let Some(action) = arrival_action {
                    let state = &robot["data"]["stationState"];
                    assert_eq!(state["action"], action);
                    assert_eq!(state["phase"], "COMPLETED");
                    assert_eq!(state["loaded"], action == "PICK");
                    if action == "CHARGE" {
                        // Core verifies exactly 100% in completion evidence. A later
                        // idle state report may already include a tick of idle drain.
                        let percent = state["batteryPercent"].as_f64().unwrap();
                        assert!((99.99..=100.0).contains(&percent));
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
        if arrival_action == Some("PLACE") {
            wait_for_buffer_waiting(&client, &rest_base).await;
        }
    }
    if battery == BatteryScenario::AutoCharge {
        let deadline = Instant::now() + Duration::from_secs(30);
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
            let charge_complete = snapshot["entities"]
                .as_array()
                .unwrap()
                .iter()
                .any(|entity| {
                    entity["entityType"] == "ORDER"
                        && entity["data"]["state"] == "Completed"
                        && entity["data"]["assignments"][0]["arrivalAction"] == "CHARGE"
                });
            if charge_complete {
                let checkpoint: serde_json::Value =
                    serde_json::from_slice(&fs::read(temp.path().join("checkpoint.json")).unwrap())
                        .unwrap();
                let station = &checkpoint["body"]["checkpoint"]["stationStates"]["robot-1"];
                assert!(station["batteryPercent"].as_f64().unwrap() > 99.99);
                assert_eq!(station["phase"], "COMPLETED");
                let orders = snapshot["entities"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .filter(|e| e["entityType"] == "ORDER")
                    .count();
                assert_eq!(orders, 2);
                break;
            }
            assert!(
                Instant::now() < deadline,
                "Auto charge did not complete: {snapshot}"
            );
            sleep(Duration::from_millis(100)).await;
        }
    }
    if fleet_mode {
        let created = client
            .post(format!("{rest_base}api/v1/robots"))
            .header("Cookie", "__Host-mapf_session=operator-session")
            .header("X-CSRF-Token", "csrf-token")
            .json(
                &json!({"contractVersion": "1.0.0", "requestId": Uuid::new_v4(),
                "map": {"mapId": info.map_id, "revision": 1,
                    "contentDigestSha256": info.map_digest_sha256},
                "start": {"column": 0, "row": 0}}),
            )
            .send()
            .await
            .unwrap();
        assert_eq!(created.status(), StatusCode::ACCEPTED);
        let created: serde_json::Value = created.json().await.unwrap();
        let robot_id = created["robotId"].as_str().unwrap();
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            let status: serde_json::Value = client
                .get(format!("{rest_base}api/v1/robots/{robot_id}/provisioning"))
                .header("Cookie", "__Host-mapf_session=operator-session")
                .send()
                .await
                .unwrap()
                .json()
                .await
                .unwrap();
            if status["state"] == "READY" {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "fleet robot never became READY: {status}"
            );
            sleep(Duration::from_millis(100)).await;
        }
        // Stay alive past the original lease/watchdog window.
        sleep(Duration::from_secs(11)).await;
        assert!(simulator.0.try_wait().unwrap().is_none());
        let checkpoint: serde_json::Value =
            serde_json::from_slice(&fs::read(temp.path().join("fleet-checkpoint.json")).unwrap())
                .unwrap();
        assert_eq!(
            checkpoint["body"]["checkpoint"]["robots"]
                .as_array()
                .unwrap()
                .len(),
            2
        );
    }
    if station_actions || fleet_mode {
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

async fn wait_for_buffer_waiting(client: &reqwest::Client, rest_base: &str) {
    let deadline = Instant::now() + Duration::from_secs(25);
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
        let entities = snapshot["entities"].as_array().unwrap();
        if let Some(state) = entities
            .iter()
            .find(|e| e["entityType"] == "BUFFER_STATE" && e["entityId"] == "robot-1")
        {
            assert_ne!(
                state["data"]["phase"], "HELD",
                "Buffer departure failed: {snapshot}"
            );
            if state["data"]["phase"] == "WAITING" {
                assert_eq!(state["data"]["bufferOccupied"], true);
                assert_eq!(state["data"]["stationClear"], true);
                let robot = entities
                    .iter()
                    .find(|e| e["entityType"] == "ROBOT" && e["entityId"] == "robot-1")
                    .unwrap();
                assert!((robot["data"]["pose"]["xMeters"].as_f64().unwrap() - 2.5).abs() < 0.1);
                assert!((robot["data"]["pose"]["yMeters"].as_f64().unwrap() - 3.5).abs() < 0.1);
                assert_eq!(robot["data"]["stationState"]["loaded"], false);
                return;
            }
        }
        assert!(
            Instant::now() < deadline,
            "Post-PLACE buffer arrival timed out: {snapshot}"
        );
        sleep(Duration::from_millis(100)).await;
    }
}
