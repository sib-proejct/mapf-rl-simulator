use mapf_rl_simulator::controller::{BaselineController, RuntimeProfile};
use mapf_rl_simulator::core_client::{ApiKey, CoreClient, CoreClientConfig, CoreClientError};
use mapf_rl_simulator::protocol::MapIdentity;
use mapf_rl_simulator::runtime::{Phase2Runtime, RuntimeError};
use mapf_rl_simulator::session::CoreSession;
use mapf_rl_simulator::spool::DurableSpool;
use std::time::Duration;
use tempfile::TempDir;
use tokio::net::TcpListener;
use url::Url;
use uuid::Uuid;

fn client(port: u16) -> CoreClient {
    CoreClient::new(
        CoreClientConfig::new(
            RuntimeProfile::Local,
            "sim-1".to_owned(),
            Url::parse(&format!("http://127.0.0.1:{port}/")).unwrap(),
            Url::parse(&format!("ws://127.0.0.1:{port}/ws/v1")).unwrap(),
            ApiKey::new("test-key").unwrap(),
        )
        .unwrap(),
    )
    .unwrap()
}

#[tokio::test]
async fn stalled_reconnect_runs_off_tick_and_times_out() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move {
        let (_socket, _) = listener.accept().await.unwrap();
        tokio::time::sleep(Duration::from_secs(10)).await;
    });
    let temp = TempDir::new().unwrap();
    let session = CoreSession::new(
        "sim-1".to_owned(),
        "robot-1".to_owned(),
        MapIdentity {
            map_id: Uuid::new_v4(),
            revision: 1,
            content_digest_sha256: "a".repeat(64),
        },
        BaselineController::explicit(RuntimeProfile::Local)
            .unwrap()
            .report()
            .clone(),
        DurableSpool::open(temp.path().join("spool.json"), "sim-1").unwrap(),
    )
    .unwrap();
    let runtime = Phase2Runtime::new(client(port), session);
    let reconnect = runtime.reconnect_task(0);
    assert!(
        !tokio::time::timeout(Duration::from_millis(250), runtime.synchronized())
            .await
            .unwrap()
            .unwrap()
    );
    assert!(!reconnect.is_finished());
    let result = tokio::time::timeout(Duration::from_secs(5), reconnect)
        .await
        .unwrap()
        .unwrap();
    assert!(matches!(
        result,
        Err(RuntimeError::Core(CoreClientError::Timeout))
    ));
    server.abort();
}

#[tokio::test]
// tungstenite's server callback requires its concrete HTTP error response type.
#[allow(clippy::result_large_err)]
async fn missing_welcome_times_out_after_successful_handshake() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = tokio::spawn(async move {
        let (socket, _) = listener.accept().await.unwrap();
        let _websocket = tokio_tungstenite::accept_hdr_async(socket,
            |_: &tokio_tungstenite::tungstenite::handshake::server::Request,
             mut response: tokio_tungstenite::tungstenite::handshake::server::Response| {
                response.headers_mut().insert("Sec-WebSocket-Protocol", "mapf.v1".parse().unwrap());
                Ok(response)
            }).await.unwrap();
        tokio::time::sleep(Duration::from_secs(10)).await;
    });
    let mut websocket = client(port).connect_websocket(None).await.unwrap();
    let result = tokio::time::timeout(Duration::from_secs(5), websocket.next_json())
        .await
        .unwrap();
    assert!(matches!(result, Err(CoreClientError::Timeout)));
    server.abort();
}
