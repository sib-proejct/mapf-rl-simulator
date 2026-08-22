use mapf_rl_simulator::contracts::generated::{
    ActionCandidate, CONTRACT_TREE_SHA256, CONTRACT_VERSION, WsMessageType,
};
use mapf_rl_simulator::protocol::{OrderCommand, ReportAck, ReportEnvelope};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::PathBuf;

const CONSUMER_LOCK: &str = include_str!("../src/contracts/contracts.lock.json");

#[test]
fn generated_identity_matches_consumer_lock() {
    assert_eq!(CONTRACT_VERSION, "1.0.0");
    assert!(CONSUMER_LOCK.contains(CONTRACT_TREE_SHA256));
}

#[test]
fn core_fixture_bytes_match_the_consumer_lock_and_parse_as_json() {
    let lock: Value = serde_json::from_str(CONSUMER_LOCK).expect("consumer lock must be JSON");
    let core_contracts =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../mapf-rl-core/packages/contracts");
    let files = lock["files"]
        .as_object()
        .expect("lock files must be an object");

    for (relative_path, expected_digest) in files {
        if !relative_path.starts_with("fixtures/") || !relative_path.ends_with(".json") {
            continue;
        }
        let bytes = fs::read(core_contracts.join(relative_path)).expect("fixture must be readable");
        let actual_digest = format!("{:x}", Sha256::digest(&bytes));
        assert_eq!(actual_digest, expected_digest.as_str().unwrap());
        serde_json::from_slice::<Value>(&bytes).expect("fixture must parse as JSON");
    }
}

#[test]
fn action_indices_and_message_names_are_stable() {
    assert_eq!(ActionCandidate::Wait as u8, 0);
    assert_eq!(ActionCandidate::North as u8, 1);
    assert_eq!(ActionCandidate::East as u8, 2);
    assert_eq!(ActionCandidate::South as u8, 3);
    assert_eq!(ActionCandidate::West as u8, 4);
    assert_eq!(WsMessageType::ReportAck.as_str(), "report.ack");
    assert_eq!(
        WsMessageType::RobotStateReport.as_str(),
        "robot.state.report"
    );
}

#[test]
fn phase2_core_fixtures_map_to_the_typed_consumer_boundary() {
    let fixtures = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../mapf-rl-core/packages/contracts/fixtures/valid");

    let order: OrderCommand = serde_json::from_slice(
        &fs::read(fixtures.join("ws-order-command.json")).expect("fixture must be readable"),
    )
    .expect("Order fixture must deserialize");
    order.validate().expect("Order fixture must validate");

    for name in ["ws-command-ack.json", "ws-robot-state-report.json"] {
        let report: ReportEnvelope = serde_json::from_slice(
            &fs::read(fixtures.join(name)).expect("fixture must be readable"),
        )
        .expect("report fixture must deserialize");
        report.validate().expect("report fixture must validate");
    }

    let acknowledgement: ReportAck = serde_json::from_slice(
        &fs::read(fixtures.join("ws-report-ack.json")).expect("fixture must be readable"),
    )
    .expect("report ack fixture must deserialize");
    acknowledgement
        .validate()
        .expect("report ack fixture must validate");
}
