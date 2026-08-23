use mapf_rl_simulator::controller::{BaselineController, RuntimeProfile};
use mapf_rl_simulator::core_client::{ApiKey, CoreClientConfig};
use mapf_rl_simulator::protocol::{
    MapIdentity, OrderCommand, OrderCommandPayload, OrderPhase, PoseReport, Producer, ProducerKind,
    ReportAck, ReportAckPayload, ReportDisposition, ReportDurability, ReportPayload,
    SimulatorRobotSnapshot, SimulatorSnapshot, StreamCursor, StreamWelcome, StreamWelcomePayload,
};
use mapf_rl_simulator::session::{
    CoreSession, REST_FALLBACK_INTERVAL_MS, STATE_REPORT_INTERVAL_MS, SessionError, SessionState,
};
use mapf_rl_simulator::spool::{AckResult, DurableSpool};
use tempfile::TempDir;
use url::Url;
use uuid::Uuid;

const NOW: &str = "2026-08-22T01:00:00.000Z";

fn map() -> MapIdentity {
    MapIdentity {
        map_id: Uuid::new_v4(),
        revision: 1,
        content_digest_sha256: "a".repeat(64),
    }
}

fn welcome(epoch: u64, stream_id: &str) -> StreamWelcome {
    StreamWelcome {
        contract_version: "1.0.0".to_owned(),
        message_id: Uuid::new_v4(),
        message_type: "stream.welcome".to_owned(),
        producer: Producer {
            kind: ProducerKind::Core,
            id: "core-api".to_owned(),
        },
        occurred_at: NOW.to_owned(),
        correlation_id: Uuid::new_v4(),
        stream_id: stream_id.to_owned(),
        payload: StreamWelcomePayload {
            resume_retention_seconds: 900,
            resume_max_events: 10_000,
            heartbeat_interval_seconds: 5,
            session_epoch: epoch,
        },
    }
}

fn snapshot(
    epoch: u64,
    stream_id: &str,
    map: MapIdentity,
    controller: &str,
    order: Option<(&str, u64)>,
) -> SimulatorSnapshot {
    SimulatorSnapshot {
        contract_version: "1.0.0".to_owned(),
        simulator_id: "sim-1".to_owned(),
        session_epoch: epoch,
        stream_cursor: StreamCursor {
            stream_id: stream_id.to_owned(),
            event_sequence: 17,
        },
        robots: vec![SimulatorRobotSnapshot {
            robot_id: "robot-1".to_owned(),
            ready: false,
            map,
            map_content: None,
            order_id: order.map(|(id, _)| id.to_owned()),
            order_update_id: order.map(|(_, update)| update),
            active_controller: controller.to_owned(),
        }],
    }
}

fn command(epoch: u64, map: MapIdentity, command_id: Uuid, order_id: &str) -> OrderCommand {
    OrderCommand {
        contract_version: "1.0.0".to_owned(),
        message_id: Uuid::new_v4(),
        message_type: "order.command".to_owned(),
        producer: Producer {
            kind: ProducerKind::Core,
            id: "core-api".to_owned(),
        },
        occurred_at: NOW.to_owned(),
        correlation_id: Uuid::new_v4(),
        stream_id: "simulator:sim-1".to_owned(),
        event_sequence: 18,
        session_epoch: epoch,
        robot_id: "robot-1".to_owned(),
        payload: OrderCommandPayload {
            command_id,
            order_id: order_id.to_owned(),
            order_update_id: 0,
            content_digest_sha256: "b".repeat(64),
            plan_revision_id: None,
            phase: OrderPhase::Activate,
            map,
            goal: None,
        },
    }
}

fn session(temp: &TempDir, map: MapIdentity, boot: Uuid) -> CoreSession {
    let spool =
        DurableSpool::open_with_boot(temp.path().join("spool.json"), "sim-1", boot, 100).unwrap();
    CoreSession::new(
        "sim-1".to_owned(),
        "robot-1".to_owned(),
        map,
        BaselineController::explicit(RuntimeProfile::Local)
            .unwrap()
            .report()
            .clone(),
        spool,
    )
    .unwrap()
}

fn synchronize(session: &mut CoreSession, epoch: u64, map: MapIdentity) {
    session
        .accept_welcome(welcome(epoch, "simulator:sim-1"))
        .unwrap();
    session
        .reconcile(&snapshot(epoch, "simulator:sim-1", map, "unknown", None))
        .unwrap();
}

fn report_ack(session: &CoreSession, disposition: ReportDisposition) -> ReportAck {
    let report = &session.spool().pending()[0];
    ReportAck {
        contract_version: "1.0.0".to_owned(),
        message_id: Uuid::new_v4(),
        message_type: "report.ack".to_owned(),
        producer: Producer {
            kind: ProducerKind::Core,
            id: "core-api".to_owned(),
        },
        occurred_at: NOW.to_owned(),
        correlation_id: report.correlation_id,
        stream_id: "simulator:sim-1".to_owned(),
        event_sequence: 19,
        session_epoch: session.session_epoch().unwrap(),
        simulator_id: "sim-1".to_owned(),
        payload: ReportAckPayload {
            report_message_id: report.message_id,
            simulator_boot_id: report.simulator_boot_id,
            report_sequence: report.report_sequence,
            disposition,
            durability: ReportDurability::Durable,
            retryable: false,
            code: match disposition {
                ReportDisposition::Accepted => "REPORT_ACCEPTED",
                ReportDisposition::Duplicate => "REPORT_DUPLICATE",
                ReportDisposition::Rejected => "REPORT_REJECTED",
            }
            .to_owned(),
        },
    }
}

#[test]
fn websocket_handoff_does_not_remove_report_before_core_acceptance() {
    let temp = TempDir::new().unwrap();
    let map = map();
    let mut session = session(&temp, map.clone(), Uuid::new_v4());
    synchronize(&mut session, 1, map);
    session
        .queue_state_report(
            0,
            0,
            PoseReport {
                x_meters: 1.0,
                y_meters: 2.0,
                yaw_radians: 0.0,
            },
            NOW.to_owned(),
        )
        .unwrap();

    // Reading/replaying the outbound bytes represents any number of successful WS writes.
    let sent = serde_json::to_vec(&session.spool().pending()[0]).unwrap();
    assert!(!sent.is_empty());
    assert_eq!(session.spool().pending().len(), 1);
    let ReportPayload::State(payload) = &session.spool().pending()[0].payload else {
        panic!("expected periodic state report");
    };
    assert_eq!(
        payload.active_controller.identity,
        "cardinal-baseline/1.0.0"
    );

    let acknowledgement = report_ack(&session, ReportDisposition::Accepted);
    assert_eq!(
        session.accept_report_ack(&acknowledgement).unwrap(),
        AckResult::Removed
    );
    assert!(session.spool().pending().is_empty());
}

#[test]
fn retryable_rejection_stays_spooled_and_duplicate_acceptance_removes_it() {
    let temp = TempDir::new().unwrap();
    let map = map();
    let mut session = session(&temp, map.clone(), Uuid::new_v4());
    synchronize(&mut session, 1, map);
    session
        .queue_state_report(
            0,
            0,
            PoseReport {
                x_meters: 1.0,
                y_meters: 2.0,
                yaw_radians: 0.0,
            },
            NOW.to_owned(),
        )
        .unwrap();

    let mut rejected = report_ack(&session, ReportDisposition::Rejected);
    rejected.payload.retryable = true;
    rejected.payload.code = "REPORT_SEQUENCE_GAP".to_owned();
    assert_eq!(
        session.accept_report_ack(&rejected).unwrap(),
        AckResult::Retained
    );
    assert_eq!(session.spool().pending().len(), 1);

    let duplicate = report_ack(&session, ReportDisposition::Duplicate);
    assert_eq!(
        session.accept_report_ack(&duplicate).unwrap(),
        AckResult::Removed
    );
    assert!(session.spool().pending().is_empty());
}

#[test]
fn duplicate_command_is_acknowledged_without_second_robot_application() {
    let temp = TempDir::new().unwrap();
    let map = map();
    let mut session = session(&temp, map.clone(), Uuid::new_v4());
    synchronize(&mut session, 3, map.clone());
    let command = command(3, map, Uuid::new_v4(), "order-1");

    let first = session.accept_order(&command, NOW.to_owned()).unwrap();
    let duplicate = session.accept_order(&command, NOW.to_owned()).unwrap();

    assert!(first.apply_to_robot);
    assert!(!duplicate.apply_to_robot);
    assert_eq!(duplicate.code, "COMMAND_DUPLICATE");
    assert_eq!(session.spool().pending().len(), 2);
    assert_eq!(session.spool().pending()[0].report_sequence, 0);
    assert_eq!(session.spool().pending()[1].report_sequence, 1);
}

#[test]
fn application_checkpoint_and_ack_are_one_spool_transaction() {
    let temp = TempDir::new().unwrap();
    let map = map();
    let spool =
        DurableSpool::open_with_boot(temp.path().join("spool.json"), "sim-1", Uuid::new_v4(), 1)
            .unwrap();
    let mut session = CoreSession::new(
        "sim-1".to_owned(),
        "robot-1".to_owned(),
        map.clone(),
        BaselineController::explicit(RuntimeProfile::Local)
            .unwrap()
            .report()
            .clone(),
        spool,
    )
    .unwrap();
    synchronize(&mut session, 1, map.clone());
    session
        .queue_state_report(
            0,
            0,
            PoseReport {
                x_meters: 1.0,
                y_meters: 2.0,
                yaw_radians: 0.0,
            },
            NOW.to_owned(),
        )
        .unwrap();

    assert!(
        session
            .accept_order(&command(1, map, Uuid::new_v4(), "order-1"), NOW.to_owned())
            .is_err()
    );
    assert!(session.spool().applied_order().is_none());
}

#[test]
fn accepted_completion_ack_clears_the_applied_order_checkpoint() {
    let temp = TempDir::new().unwrap();
    let map = map();
    let mut session = session(&temp, map.clone(), Uuid::new_v4());
    synchronize(&mut session, 1, map.clone());
    session
        .accept_order(&command(1, map, Uuid::new_v4(), "order-1"), NOW.to_owned())
        .unwrap();
    session.queue_order_completed(500, NOW.to_owned()).unwrap();
    let completion = session.spool().pending().last().unwrap();
    let acknowledgement = ReportAck {
        contract_version: "1.0.0".to_owned(),
        message_id: Uuid::new_v4(),
        message_type: "report.ack".to_owned(),
        producer: Producer {
            kind: ProducerKind::Core,
            id: "core-api".to_owned(),
        },
        occurred_at: NOW.to_owned(),
        correlation_id: completion.correlation_id,
        stream_id: "simulator:sim-1".to_owned(),
        event_sequence: 19,
        session_epoch: 1,
        simulator_id: "sim-1".to_owned(),
        payload: ReportAckPayload {
            report_message_id: completion.message_id,
            simulator_boot_id: completion.simulator_boot_id,
            report_sequence: completion.report_sequence,
            disposition: ReportDisposition::Accepted,
            durability: ReportDurability::Durable,
            retryable: false,
            code: "REPORT_ACCEPTED".to_owned(),
        },
    };

    assert!(session.spool().applied_order().is_some());
    assert_eq!(
        session.accept_report_ack(&acknowledgement).unwrap(),
        AckResult::Removed
    );
    assert!(session.spool().applied_order().is_none());
}

#[test]
fn restart_preserves_unacknowledged_report_with_original_identity() {
    let temp = TempDir::new().unwrap();
    let map = map();
    let old_boot = Uuid::new_v4();
    let identity = {
        let mut first = session(&temp, map.clone(), old_boot);
        synchronize(&mut first, 4, map.clone());
        first
            .queue_state_report(
                7,
                700,
                PoseReport {
                    x_meters: 1.0,
                    y_meters: 2.0,
                    yaw_radians: 0.0,
                },
                NOW.to_owned(),
            )
            .unwrap();
        let report = &first.spool().pending()[0];
        (
            report.message_id,
            report.session_epoch,
            report.report_sequence,
        )
    };

    let new_boot = Uuid::new_v4();
    let mut restarted = session(&temp, map, new_boot);
    assert_eq!(restarted.spool().pending().len(), 1);
    let recovered = &restarted.spool().pending()[0];
    assert_eq!(
        (
            recovered.message_id,
            recovered.session_epoch,
            recovered.report_sequence
        ),
        identity
    );
    assert_eq!(recovered.simulator_boot_id, old_boot);

    restarted
        .accept_welcome(welcome(5, "simulator:sim-1"))
        .unwrap();
    assert_eq!(restarted.spool().pending()[0].session_epoch, 4);
}

#[test]
fn reconnect_fences_old_epoch_and_converges_with_authoritative_snapshot() {
    let temp = TempDir::new().unwrap();
    let map = map();
    let mut session = session(&temp, map.clone(), Uuid::new_v4());
    synchronize(&mut session, 8, map.clone());
    let mut original = command(8, map.clone(), Uuid::new_v4(), "order-1");
    assert!(
        session
            .accept_order(&original, NOW.to_owned())
            .unwrap()
            .apply_to_robot
    );
    session.mark_disconnected(1_000);
    assert_eq!(session.state(), SessionState::Degraded);
    assert!(!session.rest_fallback_due(5_999));
    assert!(session.rest_fallback_due(6_000));

    session
        .accept_welcome(welcome(9, "simulator:sim-1"))
        .unwrap();
    assert_eq!(session.spool().pending()[0].session_epoch, 9);
    session
        .reconcile(&snapshot(
            9,
            "simulator:sim-1",
            map.clone(),
            session.controller().identity.as_str(),
            Some(("order-1", 0)),
        ))
        .unwrap();
    assert_eq!(session.state(), SessionState::Synchronized);

    assert!(matches!(
        session.accept_order(&original, NOW.to_owned()),
        Err(SessionError::OldSession)
    ));
    original.session_epoch = 9;
    assert!(
        !session
            .accept_order(&original, NOW.to_owned())
            .unwrap()
            .apply_to_robot
    );
}

#[test]
fn report_clock_is_ten_hz_and_rest_fallback_is_five_seconds() {
    let temp = TempDir::new().unwrap();
    let map = map();
    let mut session = session(&temp, map.clone(), Uuid::new_v4());
    synchronize(&mut session, 1, map);
    assert!(session.state_report_due(0));
    assert!(!session.state_report_due(STATE_REPORT_INTERVAL_MS - 1));
    assert!(session.state_report_due(STATE_REPORT_INTERVAL_MS));

    session.mark_disconnected(10);
    assert!(!session.rest_fallback_due(10 + REST_FALLBACK_INTERVAL_MS - 1));
    assert!(session.rest_fallback_due(10 + REST_FALLBACK_INTERVAL_MS));
    assert!(!session.rest_fallback_due(10 + REST_FALLBACK_INTERVAL_MS + 1));
}

#[test]
fn api_key_is_redacted_and_insecure_transport_is_loopback_local_only() {
    let key = ApiKey::new("01234567890123456789012345678901").unwrap();
    assert_eq!(format!("{key:?}"), "ApiKey([REDACTED])");
    assert!(
        CoreClientConfig::new(
            RuntimeProfile::Local,
            "sim-1".to_owned(),
            Url::parse("http://127.0.0.1:8000/").unwrap(),
            Url::parse("ws://127.0.0.1:8000/ws/v1").unwrap(),
            key.clone(),
        )
        .is_ok()
    );
    assert!(
        CoreClientConfig::new(
            RuntimeProfile::Dev,
            "sim-1".to_owned(),
            Url::parse("http://core.internal/").unwrap(),
            Url::parse("ws://core.internal/ws/v1").unwrap(),
            key,
        )
        .is_err()
    );
}
