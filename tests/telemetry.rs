use futures_util::{SinkExt, StreamExt};
use mapf_rl_simulator::controller::RuntimeProfile;
use mapf_rl_simulator::core_client::{ApiKey, CoreClient, CoreClientConfig};
use mapf_rl_simulator::telemetry::{TelemetryTransport, durable_fingerprint};
use serde_json::{Value, json};
use std::time::{Duration, Instant};
use tokio::net::TcpListener;
use tokio_tungstenite::tungstenite::{
    Message,
    handshake::server::{Request, Response},
};
use url::Url;

fn client(port: u16) -> CoreClient {
    CoreClient::new(
        CoreClientConfig::new(
            RuntimeProfile::Local,
            "sim-1".into(),
            Url::parse(&format!("http://127.0.0.1:{port}/")).unwrap(),
            Url::parse(&format!("ws://127.0.0.1:{port}/ws/v1")).unwrap(),
            ApiKey::new("test-key").unwrap(),
        )
        .unwrap(),
    )
    .unwrap()
}
fn frame(sequence: u64) -> Value {
    json!({"telemetryVersion":"1.0.0", "messageType":"telemetry.report", "sessionEpoch":1,
        "simulatorBootId":"123e4567-e89b-42d3-a456-426614174000", "simulatorId":"sim-1",
        "robotId":"robot-1", "telemetrySequence":sequence})
}
fn ack(mut value: Value) -> Message {
    value["messageType"] = json!("telemetry.ack");
    value["durability"] = json!("PROJECTION");
    value["disposition"] = json!("ACCEPTED");
    Message::Text(value.to_string().into())
}
async fn ready(transport: &TelemetryTransport) {
    let until = Instant::now() + Duration::from_secs(3);
    while !transport.ready() && Instant::now() < until {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert!(transport.ready());
}
#[tokio::test]
#[allow(clippy::result_large_err)] // tungstenite handshake callback has a fixed error type.
async fn latest_only_ack_timeout_hold_and_reconnect() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move {
        for expected in [99, 100] {
            let (stream, _) = listener.accept().await.unwrap();
            let mut socket = tokio_tungstenite::accept_hdr_async(
                stream,
                |request: &Request, mut response: Response| {
                    assert_eq!(request.uri().path(), "/ws/v1/telemetry");
                    response.headers_mut().insert(
                        "Sec-WebSocket-Protocol",
                        "mapf.telemetry.v1".parse().unwrap(),
                    );
                    Ok(response)
                },
            )
            .await
            .unwrap();
            let message = socket.next().await.unwrap().unwrap();
            let value: Value = serde_json::from_str(message.to_text().unwrap()).unwrap();
            assert_eq!(value["telemetrySequence"], expected);
            socket.send(ack(value)).await.unwrap();
            if expected == 99 {
                tokio::time::sleep(Duration::from_millis(3300)).await;
            } else {
                tokio::time::sleep(Duration::from_millis(500)).await;
            }
        }
    });
    let transport = TelemetryTransport::start(client(port));
    for sequence in 0..100 {
        transport.publish(frame(sequence));
    }
    ready(&transport).await;
    tokio::time::sleep(Duration::from_millis(3100)).await;
    assert!(
        !transport.ready(),
        "motion must hold after three seconds without ACK"
    );
    transport.publish(frame(100));
    ready(&transport).await;
    server.await.unwrap();
}
#[test]
fn ordinary_pose_battery_progress_are_volatile_but_transitions_are_durable() {
    let initial = json!({"pose":{"xMeters":1},"operationalState":"IDLE","safety":"NORMAL", "stationState":{"batteryPercent":75,"phase":"IDLE","elapsedMs":0}});
    let mut moved = initial.clone();
    moved["pose"]["xMeters"] = json!(2);
    moved["stationState"]["batteryPercent"] = json!(74);
    moved["stationState"]["elapsedMs"] = json!(200);
    assert_eq!(durable_fingerprint(&initial), durable_fingerprint(&moved));
    for (key, value) in [
        ("operationalState", "EXECUTING"),
        ("safety", "CONTROLLED_STOP"),
        ("orderId", "new-order"),
    ] {
        let mut critical = moved.clone();
        critical[key] = json!(value);
        assert_ne!(durable_fingerprint(&moved), durable_fingerprint(&critical));
    }
    moved["stationState"]["phase"] = json!("COMPLETED");
    assert_ne!(durable_fingerprint(&initial), durable_fingerprint(&moved));
}

#[tokio::test]
#[allow(clippy::result_large_err)]
async fn ping_during_publication_and_invalid_ack_fail_closed() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let (ping_tx, ping_rx) = tokio::sync::oneshot::channel();
    let server = tokio::spawn(async move {
        let (stream, _) = listener.accept().await.unwrap();
        let mut socket =
            tokio_tungstenite::accept_hdr_async(stream, |_: &Request, mut response: Response| {
                response.headers_mut().insert(
                    "Sec-WebSocket-Protocol",
                    "mapf.telemetry.v1".parse().unwrap(),
                );
                Ok(response)
            })
            .await
            .unwrap();
        let first = socket.next().await.unwrap().unwrap();
        let value: Value = serde_json::from_str(first.to_text().unwrap()).unwrap();
        assert_eq!(value["telemetrySequence"], 1);
        socket
            .send(Message::Ping(vec![1, 2, 3].into()))
            .await
            .unwrap();
        ping_tx.send(()).unwrap();
        let mut pong = false;
        let mut report = false;
        while !pong || !report {
            match socket.next().await.unwrap().unwrap() {
                Message::Pong(bytes) => {
                    assert_eq!(bytes.as_ref(), &[1, 2, 3]);
                    pong = true;
                }
                Message::Text(text) => {
                    let value: Value = serde_json::from_str(&text).unwrap();
                    assert_eq!(value["telemetrySequence"], 2);
                    report = true;
                }
                other => panic!("unexpected frame: {other:?}"),
            }
        }
        let mut invalid = frame(2);
        invalid["sessionEpoch"] = json!(999);
        socket.send(ack(invalid)).await.unwrap();
        assert!(
            tokio::time::timeout(Duration::from_secs(1), socket.next())
                .await
                .is_ok()
        );
    });
    let transport = TelemetryTransport::start(client(port));
    transport.publish(frame(1));
    ping_rx.await.unwrap();
    transport.publish(frame(2));
    server.await.unwrap();
    assert!(!transport.ready());
}
