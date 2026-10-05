use mapf_rl_simulator::checkpoint::CheckpointStore;
use mapf_rl_simulator::controller::{BaselineController, RuntimeProfile};
use mapf_rl_simulator::fault::{FaultKind, FaultSpec};
use mapf_rl_simulator::fleet::{FleetConfig, MultiRobotEngine, RecoveryReason};
use mapf_rl_simulator::motion::MotionLimits;
use mapf_rl_simulator::plan::{
    PREPARE_BARRIER_TIMEOUT_MS, PlanCoordinator, PlanError, PlanRevision, PlanTarget,
    RevisionState, RobotPlanState,
};
use mapf_rl_simulator::protocol::{
    ActiveController, CommandDisposition, ControllerMode, EventSeverity, MapIdentity, OrderCommand,
    OrderCommandPayload, OrderGoal, OrderPhase, OrderRoute, OrderRouteWaypoint, PoseReport,
    Producer, ProducerKind, ReportEnvelope, ReportPayload, RobotEventPayload, RobotStatePayload,
    SimulatorRobotSnapshot, SimulatorSnapshot, StreamCursor, StreamWelcome, StreamWelcomePayload,
};
use mapf_rl_simulator::report_queue::{
    BoundedReportQueue, EnqueueOutcome, ReportClass, ReportQueueError,
};
use mapf_rl_simulator::route::action_for_route;
use mapf_rl_simulator::safety::SafetyConfig;
use mapf_rl_simulator::sensing::SensorConfig;
use mapf_rl_simulator::session::{CoreSession, SessionState};
use mapf_rl_simulator::simulation::{EngineConfig, ManualMonotonicClock, SimulationEngine};
use mapf_rl_simulator::spool::{AppliedOrder, DurableSpool};
use mapf_rl_simulator::types::{
    Acceleration, RobotId, RobotState, SimulationTimeMs, Velocity, WorldPosition,
};
use mapf_rl_simulator::world::{GridCell, GridMap};
use std::collections::BTreeMap;
use tempfile::TempDir;
use uuid::Uuid;

const NOW: &str = "2026-08-23T01:00:00.000Z";

fn target(robot_id: &str, update: u64) -> PlanTarget {
    PlanTarget {
        robot_id: robot_id.to_owned(),
        order_id: format!("order-{robot_id}"),
        order_update_id: update,
        content_digest_sha256: format!("{:064x}", update + 1),
    }
}

fn revision(robot_ids: &[&str]) -> PlanRevision {
    PlanRevision {
        plan_revision_id: Uuid::new_v4(),
        planning_snapshot_digest_sha256: "a".repeat(64),
        targets: robot_ids
            .iter()
            .enumerate()
            .map(|(index, id)| target(id, index as u64))
            .collect(),
        activation_order: robot_ids.iter().map(|id| (*id).to_owned()).collect(),
    }
}

fn prepared(revision: &PlanRevision) -> PlanCoordinator {
    let mut plans = PlanCoordinator::new();
    plans.begin_prepare(revision.clone(), 0).unwrap();
    for target in &revision.targets {
        plans
            .mark_safe_hold(revision.plan_revision_id, &target.robot_id, true, 0)
            .unwrap();
    }
    plans
}

#[test]
fn incomplete_prepare_barrier_never_authorizes_motion_and_times_out_at_15_seconds() {
    let revision = revision(&["r1", "r2"]);
    let mut plans = PlanCoordinator::new();
    plans.begin_prepare(revision.clone(), 10).unwrap();
    plans
        .mark_safe_hold(revision.plan_revision_id, "r1", true, 20)
        .unwrap();

    assert_eq!(plans.state(), Some(RevisionState::Preparing));
    assert!(!plans.motion_authorized("r1"));
    assert!(matches!(
        plans.activate(revision.plan_revision_id, &revision.targets[0], 30),
        Err(PlanError::PrepareBarrierIncomplete)
    ));
    assert!(!plans.poll_timeout(10 + PREPARE_BARRIER_TIMEOUT_MS - 1));
    assert!(plans.poll_timeout(10 + PREPARE_BARRIER_TIMEOUT_MS));
    assert_eq!(plans.state(), Some(RevisionState::Aborted));
    assert_eq!(plans.robot_state("r1"), Some(RobotPlanState::Aborted));
    assert!(!plans.motion_authorized("r2"));
}

#[test]
fn dependency_release_requires_prior_activation_runtime_confirmation() {
    let revision = revision(&["r1", "r2"]);
    let mut plans = prepared(&revision);
    plans
        .activate(revision.plan_revision_id, &revision.targets[0], 1)
        .unwrap();
    assert!(plans.motion_authorized("r1"));
    assert!(!plans.motion_authorized("r2"));
    assert!(matches!(
        plans.activate(revision.plan_revision_id, &revision.targets[1], 2),
        Err(PlanError::ActivationDependencyIncomplete)
    ));
    plans
        .confirm_runtime_safe(revision.plan_revision_id, "r1")
        .unwrap();
    plans
        .activate(revision.plan_revision_id, &revision.targets[1], 3)
        .unwrap();
    assert!(plans.motion_authorized("r2"));
}

#[test]
fn core_prepare_ack_and_activation_checkpoint_are_separate_and_timeout_is_fail_closed() {
    let temp = TempDir::new().unwrap();
    let map = MapIdentity {
        map_id: Uuid::new_v4(),
        revision: 1,
        content_digest_sha256: "d".repeat(64),
    };
    let mut session = synchronized_session(&temp, map.clone());
    let plan_revision_id = Uuid::new_v4();
    let mut command = plan_command(map, plan_revision_id, OrderPhase::Prepare);

    let prepared = session
        .accept_order_at(&command, NOW.to_owned(), 100, true)
        .unwrap();
    assert_eq!(prepared.disposition, CommandDisposition::Prepared);
    assert!(!prepared.apply_to_robot);
    assert!(session.applied_order().is_none());
    assert_eq!(
        session.prepared_order().unwrap().plan_revision_id,
        plan_revision_id
    );

    command.payload.phase = OrderPhase::Activate;
    command.payload.command_id = Uuid::new_v4();
    command.payload.goal = None;
    command.payload.route = None;
    let activated = session
        .accept_order_at(&command, NOW.to_owned(), 101, true)
        .unwrap();
    assert_eq!(activated.disposition, CommandDisposition::Applied);
    assert!(activated.apply_to_robot);
    assert!(session.prepared_order().is_none());
    assert_eq!(
        session.applied_order().unwrap().plan_revision_id,
        Some(plan_revision_id)
    );
    assert_eq!(
        session
            .applied_order()
            .unwrap()
            .route
            .as_ref()
            .unwrap()
            .waypoints
            .len(),
        2
    );

    command.payload.phase = OrderPhase::Abort;
    command.payload.command_id = Uuid::new_v4();
    let aborted = session
        .accept_order_at(&command, NOW.to_owned(), 102, true)
        .unwrap();
    assert_eq!(aborted.code, "PLAN_ABORTED_STOPPED");
    assert!(session.applied_order().is_none());

    let timeout_temp = TempDir::new().unwrap();
    let map = command.payload.map.clone();
    let mut timeout_session = synchronized_session(&timeout_temp, map.clone());
    timeout_session
        .accept_order_at(
            &plan_command(map, Uuid::new_v4(), OrderPhase::Prepare),
            NOW.to_owned(),
            1_000,
            true,
        )
        .unwrap();
    assert!(
        !timeout_session
            .poll_prepare_barrier(1_000 + PREPARE_BARRIER_TIMEOUT_MS - 1)
            .unwrap()
    );
    assert!(
        timeout_session
            .poll_prepare_barrier(1_000 + PREPARE_BARRIER_TIMEOUT_MS)
            .unwrap()
    );
    assert_eq!(timeout_session.state(), SessionState::Degraded);
    assert!(timeout_session.prepared_order().is_none());
}

#[test]
fn prepared_route_follower_waits_for_timestamp_then_moves_to_next_waypoint() {
    let map = GridMap::new(3, 1, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let route = OrderRoute {
        execution_control: None,
        robot_id: "r1".to_owned(),
        order_id: "order-r1".to_owned(),
        release_after_robot_id: None,
        plan_digest_sha256: "f".repeat(64),
        motion: None,
        waypoints: vec![
            OrderRouteWaypoint {
                column: 0,
                row: 0,
                start_simulation_time_ms: 0,
                end_simulation_time_ms: 1_000,
            },
            OrderRouteWaypoint {
                column: 1,
                row: 0,
                start_simulation_time_ms: 1_000,
                end_simulation_time_ms: 2_000,
            },
        ],
    };
    let position = map.grid_to_world(GridCell::new(0, 0)).unwrap();
    assert_eq!(
        action_for_route(&map, position, SimulationTimeMs::ZERO, &route).unwrap(),
        0
    );
    assert_eq!(
        action_for_route(
            &map,
            position,
            SimulationTimeMs::new(1_000).unwrap(),
            &route,
        )
        .unwrap(),
        2
    );
}

fn synchronized_session(temp: &TempDir, map: MapIdentity) -> CoreSession {
    let spool =
        DurableSpool::open_with_boot(temp.path().join("spool.json"), "sim-1", Uuid::new_v4(), 100)
            .unwrap();
    let mut session = CoreSession::new(
        "sim-1".to_owned(),
        "r1".to_owned(),
        map.clone(),
        controller(),
        spool,
    )
    .unwrap();
    session
        .accept_welcome(StreamWelcome {
            contract_version: "1.0.0".to_owned(),
            message_id: Uuid::new_v4(),
            message_type: "stream.welcome".to_owned(),
            producer: Producer {
                kind: ProducerKind::Core,
                id: "core-api".to_owned(),
            },
            occurred_at: NOW.to_owned(),
            correlation_id: Uuid::new_v4(),
            stream_id: "simulator:sim-1".to_owned(),
            payload: StreamWelcomePayload {
                resume_retention_seconds: 900,
                resume_max_events: 10_000,
                heartbeat_interval_seconds: 5,
                session_epoch: 1,
            },
        })
        .unwrap();
    session
        .reconcile(&SimulatorSnapshot {
            contract_version: "1.0.0".to_owned(),
            simulator_id: "sim-1".to_owned(),
            session_epoch: 1,
            stream_cursor: StreamCursor {
                stream_id: "simulator:sim-1".to_owned(),
                event_sequence: 0,
            },
            robots: vec![SimulatorRobotSnapshot {
                robot_id: "r1".to_owned(),
                ready: false,
                map,
                map_content: None,
                order_id: None,
                order_update_id: None,
                active_controller: "unknown".to_owned(),
            }],
        })
        .unwrap();
    session
}

fn plan_command(map: MapIdentity, plan_revision_id: Uuid, phase: OrderPhase) -> OrderCommand {
    let route = OrderRoute {
        execution_control: None,
        robot_id: "r1".to_owned(),
        order_id: "order-r1".to_owned(),
        release_after_robot_id: None,
        plan_digest_sha256: "f".repeat(64),
        motion: None,
        waypoints: vec![
            OrderRouteWaypoint {
                column: 0,
                row: 0,
                start_simulation_time_ms: 0,
                end_simulation_time_ms: 1_000,
            },
            OrderRouteWaypoint {
                column: 1,
                row: 0,
                start_simulation_time_ms: 1_000,
                end_simulation_time_ms: 2_000,
            },
        ],
    };
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
        event_sequence: 1,
        session_epoch: 1,
        robot_id: "r1".to_owned(),
        payload: OrderCommandPayload {
            command_id: Uuid::new_v4(),
            order_id: "order-r1".to_owned(),
            order_update_id: 1,
            content_digest_sha256: "e".repeat(64),
            plan_revision_id: Some(plan_revision_id),
            phase,
            map,
            goal: Some(OrderGoal { column: 1, row: 0 }),
            route: Some(route),
            arrival_action: None,
        },
    }
}

fn engine(
    id: &str,
    map: GridMap,
    position: WorldPosition,
    faults: &[FaultSpec],
) -> SimulationEngine<ManualMonotonicClock> {
    SimulationEngine::new(
        ManualMonotonicClock::default(),
        RobotId::new(id).unwrap(),
        map,
        RobotState::new(position, Velocity::ZERO, Acceleration::ZERO, 0.0).unwrap(),
        EngineConfig {
            motion_limits: MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        },
        7,
        faults,
    )
    .unwrap()
}

fn open_map() -> GridMap {
    GridMap::new(9, 5, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap()
}

#[test]
fn unreleased_robot_remains_a_stationary_reservation_during_collision_recovery() {
    let map = open_map();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(2.5, 2.5).unwrap(),
                &[],
            ),
            engine("r2", map, WorldPosition::new(3.5, 2.5).unwrap(), &[]),
        ],
        FleetConfig::new(0.6, 50).unwrap(),
    )
    .unwrap();
    let revision = revision(&["r1", "r2"]);
    let mut plans = prepared(&revision);
    plans
        .activate(revision.plan_revision_id, &revision.targets[0], 0)
        .unwrap();
    let r2_start = fleet.robots()[&RobotId::new("r2").unwrap()].state();

    let actions = BTreeMap::from([
        (RobotId::new("r1").unwrap(), 2),
        (RobotId::new("r2").unwrap(), 4),
    ]);
    let mut recovery = None;
    for _ in 0..30 {
        let step = fleet.step(&actions, &mut plans).unwrap();
        if step.recovery.is_some() {
            recovery = step.recovery;
            break;
        }
    }

    let recovery = recovery.expect("released robot must stop before stationary reservation");
    assert!(matches!(
        recovery.reason,
        RecoveryReason::Collision | RecoveryReason::CorridorConflict
    ));
    assert!(recovery.replan_required);
    assert_eq!(
        fleet.robots()[&RobotId::new("r2").unwrap()]
            .state()
            .position(),
        r2_start.position()
    );
    assert_eq!(plans.state(), Some(RevisionState::HeldForRecovery));
    assert!(!plans.motion_authorized("r1"));
    assert!(!plans.motion_authorized("r2"));
}

#[test]
fn one_robot_failure_holds_every_target_without_unsafe_release() {
    let map = open_map();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(2.5, 2.5).unwrap(),
                &[],
            ),
            engine("r2", map, WorldPosition::new(5.5, 2.5).unwrap(), &[]),
        ],
        FleetConfig::new(0.6, 20).unwrap(),
    )
    .unwrap();
    let revision = revision(&["r1", "r2"]);
    let mut plans = prepared(&revision);
    plans
        .activate(revision.plan_revision_id, &revision.targets[0], 0)
        .unwrap();

    let event = fleet
        .fail_robot(&RobotId::new("r1").unwrap(), &mut plans)
        .unwrap();
    assert_eq!(event.reason, RecoveryReason::RobotFailure);
    assert_eq!(plans.robot_state("r1"), Some(RobotPlanState::Failed));
    assert_eq!(plans.robot_state("r2"), Some(RobotPlanState::Holding));
    assert!(!plans.motion_authorized("r2"));
}

#[test]
fn crossing_conflict_is_stopped_and_requests_replan() {
    let map = open_map();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(4.5, 2.5).unwrap(),
                &[],
            ),
            engine("r2", map, WorldPosition::new(3.5, 3.5).unwrap(), &[]),
        ],
        FleetConfig::new(0.6, 50).unwrap(),
    )
    .unwrap();
    let revision = revision(&["r1", "r2"]);
    let mut plans = prepared(&revision);
    plans
        .activate(revision.plan_revision_id, &revision.targets[0], 0)
        .unwrap();
    plans
        .confirm_runtime_safe(revision.plan_revision_id, "r1")
        .unwrap();
    plans
        .activate(revision.plan_revision_id, &revision.targets[1], 0)
        .unwrap();
    let actions = BTreeMap::from([
        (RobotId::new("r1").unwrap(), 1),
        (RobotId::new("r2").unwrap(), 2),
    ]);

    let recovery = (0..30)
        .find_map(|_| fleet.step(&actions, &mut plans).unwrap().recovery)
        .expect("crossing paths must be recovered before collision");
    assert_eq!(recovery.reason, RecoveryReason::Collision);
    assert_eq!(plans.state(), Some(RevisionState::HeldForRecovery));
}

#[test]
fn opposite_narrow_corridor_entry_is_classified_and_held() {
    let blocked = (0..9).flat_map(|column| [GridCell::new(column, 0), GridCell::new(column, 2)]);
    let map = GridMap::new(9, 3, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, blocked).unwrap();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(3.5, 1.5).unwrap(),
                &[],
            ),
            engine("r2", map, WorldPosition::new(5.5, 1.5).unwrap(), &[]),
        ],
        FleetConfig::new(0.6, 50).unwrap(),
    )
    .unwrap();
    let revision = revision(&["r1", "r2"]);
    let mut plans = prepared(&revision);
    plans
        .activate(revision.plan_revision_id, &revision.targets[0], 0)
        .unwrap();
    plans
        .confirm_runtime_safe(revision.plan_revision_id, "r1")
        .unwrap();
    plans
        .activate(revision.plan_revision_id, &revision.targets[1], 0)
        .unwrap();
    let actions = BTreeMap::from([
        (RobotId::new("r1").unwrap(), 2),
        (RobotId::new("r2").unwrap(), 4),
    ]);

    let recovery = (0..40)
        .find_map(|_| fleet.step(&actions, &mut plans).unwrap().recovery)
        .expect("opposite corridor entry must be held");
    assert_eq!(recovery.reason, RecoveryReason::CorridorConflict);
    assert!(recovery.replan_required);
}

#[test]
fn sustained_no_progress_triggers_deterministic_deadlock_recovery() {
    let map = GridMap::new(
        6,
        5,
        WorldPosition::new(0.0, 0.0).unwrap(),
        1.0,
        [GridCell::new(3, 2)],
    )
    .unwrap();
    let mut fleet = MultiRobotEngine::new(
        [engine(
            "r1",
            map,
            WorldPosition::new(2.5, 2.5).unwrap(),
            &[],
        )],
        FleetConfig::new(0.6, 2).unwrap(),
    )
    .unwrap();
    let revision = revision(&["r1"]);
    let mut plans = prepared(&revision);
    plans
        .activate(revision.plan_revision_id, &revision.targets[0], 0)
        .unwrap();
    let actions = BTreeMap::from([(RobotId::new("r1").unwrap(), 2)]);

    let recovery = (0..40)
        .find_map(|_| fleet.step(&actions, &mut plans).unwrap().recovery)
        .expect("blocked robot must reach the bounded deadlock threshold");
    assert_eq!(recovery.reason, RecoveryReason::Deadlock);
    assert_eq!(plans.state(), Some(RevisionState::HeldForRecovery));
}

#[test]
fn bounded_report_queue_coalesces_projection_and_fails_closed_for_control_overload() {
    let mut queue = BoundedReportQueue::new(2).unwrap();
    assert_eq!(
        queue.enqueue(state_report("r1", 0), ReportClass::StateProjection),
        Ok(EnqueueOutcome::Queued)
    );
    assert_eq!(
        queue.enqueue(state_report("r2", 0), ReportClass::StateProjection),
        Ok(EnqueueOutcome::Queued)
    );
    assert_eq!(
        queue.enqueue(state_report("r1", 1), ReportClass::StateProjection),
        Ok(EnqueueOutcome::CoalescedProjection)
    );
    assert_eq!(queue.len(), 2);
    assert_eq!(queue.dropped_projections(), 1);
    assert_eq!(
        queue.enqueue(critical_event("r1"), ReportClass::Critical),
        Ok(EnqueueOutcome::Queued)
    );
    assert_eq!(
        queue.enqueue(critical_event("r2"), ReportClass::Critical),
        Ok(EnqueueOutcome::Queued)
    );
    assert_eq!(queue.len(), 2);
    assert_eq!(
        queue.enqueue(critical_event("r3"), ReportClass::Critical),
        Err(ReportQueueError::CriticalBackpressure)
    );
    assert!(queue.critical_backpressure());
    assert_eq!(queue.len(), 2);
}

#[test]
fn restart_restores_pose_emergency_latch_and_fences_motion_authority() {
    let temp = TempDir::new().unwrap();
    let map = open_map();
    let mut fleet = MultiRobotEngine::new(
        [engine(
            "r1",
            map.clone(),
            WorldPosition::new(2.5, 2.5).unwrap(),
            &[FaultSpec::once(0, "stop", FaultKind::EmergencyStop)],
        )],
        FleetConfig::new(0.6, 20).unwrap(),
    )
    .unwrap();
    let revision = revision(&["r1"]);
    let mut plans = prepared(&revision);
    plans
        .activate(revision.plan_revision_id, &revision.targets[0], 0)
        .unwrap();
    fleet
        .step(
            &BTreeMap::from([(RobotId::new("r1").unwrap(), 2)]),
            &mut plans,
        )
        .unwrap();
    let before = fleet.robots()[&RobotId::new("r1").unwrap()].state();
    assert!(fleet.robots()[&RobotId::new("r1").unwrap()].emergency_stop_latched());

    let store = CheckpointStore::new(temp.path().join("safety.json")).unwrap();
    store
        .save(&fleet.checkpoint("sim-1".to_owned(), "b".repeat(64), plans))
        .unwrap();
    let recovered = store
        .load_for_restart("sim-1", &"b".repeat(64))
        .unwrap()
        .unwrap();
    let mut restarted = MultiRobotEngine::new(
        [engine(
            "r1",
            map,
            WorldPosition::new(2.5, 2.5).unwrap(),
            &[],
        )],
        FleetConfig::new(0.6, 20).unwrap(),
    )
    .unwrap();
    let recovered_plans = restarted.restore_checkpoint(&recovered).unwrap();

    assert_eq!(
        restarted.robots()[&RobotId::new("r1").unwrap()].state(),
        before
    );
    assert!(restarted.robots()[&RobotId::new("r1").unwrap()].emergency_stop_latched());
    assert!(!recovered_plans.motion_authorized("r1"));
    assert!(recovered.requires_core_reconciliation);
}

#[test]
fn restart_does_not_reuse_a_spooled_plan_activation_as_motion_authority() {
    let temp = TempDir::new().unwrap();
    let path = temp.path().join("reports.json");
    let plan_revision_id = Uuid::new_v4();
    let old_boot = Uuid::new_v4();
    {
        let mut spool = DurableSpool::open_with_boot(&path, "sim-1", old_boot, 10).unwrap();
        let mut report = state_report("r1", 0);
        report.simulator_boot_id = old_boot;
        spool
            .enqueue_with_order(
                report,
                Some(AppliedOrder {
                    command_id: Uuid::new_v4(),
                    order_id: "order-r1".to_owned(),
                    order_update_id: 1,
                    content_digest_sha256: "c".repeat(64),
                    plan_revision_id: Some(plan_revision_id),
                    goal: None,
                    route: None,
                    arrival_action: None,
                }),
            )
            .unwrap();
        assert_eq!(
            spool.applied_order().unwrap().plan_revision_id,
            Some(plan_revision_id)
        );
    }

    let restarted = DurableSpool::open_with_boot(path, "sim-1", Uuid::new_v4(), 10).unwrap();
    assert!(restarted.applied_order().is_none());
    assert_eq!(restarted.pending().len(), 1);
    assert_eq!(restarted.pending()[0].simulator_boot_id, old_boot);
}

fn state_report(robot_id: &str, version: u64) -> ReportEnvelope {
    report(
        robot_id,
        "robot.state.report",
        None,
        ReportPayload::State(RobotStatePayload {
            traffic_wait: None,
            state_version: version,
            simulation_time_ms: version as i64 * 100,
            pose: PoseReport {
                x_meters: 1.0,
                y_meters: 1.0,
                yaw_radians: 0.0,
            },
            active_controller: controller(),
            order_id: None,
            order_update_id: None,
            station_state: None,
            station_actions_version: None,
            operational_state: None,
            safety: None,
        }),
    )
}

fn critical_event(robot_id: &str) -> ReportEnvelope {
    report(
        robot_id,
        "robot.event.report",
        Some(Uuid::new_v4()),
        ReportPayload::RobotEvent(RobotEventPayload {
            event_id: Uuid::new_v4(),
            severity: EventSeverity::Critical,
            code: "COLLISION_INVARIANT".to_owned(),
            simulation_time_ms: 0,
            evidence: serde_json::Map::new(),
        }),
    )
}

fn report(
    robot_id: &str,
    message_type: &str,
    request_id: Option<Uuid>,
    payload: ReportPayload,
) -> ReportEnvelope {
    ReportEnvelope {
        contract_version: "1.0.0".to_owned(),
        message_id: Uuid::new_v4(),
        message_type: message_type.to_owned(),
        producer: Producer {
            kind: ProducerKind::Simulator,
            id: "sim-1".to_owned(),
        },
        occurred_at: NOW.to_owned(),
        correlation_id: Uuid::new_v4(),
        request_id,
        session_epoch: 1,
        simulator_id: "sim-1".to_owned(),
        simulator_boot_id: Uuid::new_v4(),
        report_sequence: 0,
        robot_id: robot_id.to_owned(),
        payload,
    }
}

fn controller() -> ActiveController {
    BaselineController::explicit(RuntimeProfile::Local)
        .unwrap()
        .report()
        .clone()
}

#[test]
fn report_test_controller_contract_remains_baseline() {
    assert_eq!(controller().mode, ControllerMode::Baseline);
}

#[test]
fn fractional_motion_checkpoint_roundtrip_preserves_checksum_and_pose() {
    use mapf_rl_simulator::checkpoint::{RecoveryCheckpoint, RobotSafetyCheckpoint};
    let temp = TempDir::new().unwrap();
    let store = CheckpointStore::new(temp.path().join("safety.json")).unwrap();
    let checkpoint = RecoveryCheckpoint {
        independent_plans: BTreeMap::new(),
        simulator_id: "sim-1".to_owned(),
        map_content_digest_sha256: "b".repeat(64),
        robots: vec![RobotSafetyCheckpoint {
            robot_id: "r1".to_owned(),
            tick: 1,
            simulation_time_ms: 100,
            x_meters: 14.233333333333313,
            y_meters: 8.299999999999992,
            velocity_x_mps: 0.0,
            velocity_y_mps: 0.0,
            acceleration_x_mps2: 0.0,
            acceleration_y_mps2: 0.0,
            yaw_radians: 0.0,
            emergency_stop_latched: false,
        }],
        plans: Default::default(),
        no_progress_ticks: 0,
        station_states: Default::default(),
        battery_depletion_ids: Default::default(),
        motion_profiles: Default::default(),
        requires_core_reconciliation: false,
    };
    use mapf_rl_simulator::contracts::generated::StationAction;
    use mapf_rl_simulator::station::{StationConfig, StationState};
    let mut station = StationState::default();
    station.advance(
        "pick-1",
        StationAction::Pick,
        true,
        1000,
        StationConfig::default(),
    );
    let checkpoint = checkpoint.with_station_state("r1".to_owned(), station.clone());
    store.save(&checkpoint).unwrap();
    let recovered = store
        .load_for_restart("sim-1", &"b".repeat(64))
        .unwrap()
        .unwrap();
    assert_eq!(recovered.robots, checkpoint.robots);
    let mut restored = recovered.station_states["r1"].clone();
    assert_eq!(restored, station);
    restored.advance(
        "pick-1",
        StationAction::Pick,
        false,
        5000,
        StationConfig::default(),
    );
    assert!(!restored.loaded);
    assert_eq!(restored.elapsed_ms, 1000);
    restored.advance(
        "pick-1",
        StationAction::Pick,
        true,
        1000,
        StationConfig::default(),
    );
    assert!(restored.loaded);
    let completed = restored.clone();
    restored.advance(
        "pick-1",
        StationAction::Pick,
        true,
        1000,
        StationConfig::default(),
    );
    assert_eq!(restored, completed);
    assert!(recovered.requires_core_reconciliation);
}

#[test]
fn prepared_station_order_accepts_cancellation_and_fences_late_activation() {
    use mapf_rl_simulator::contracts::generated::StationAction;

    for action in [
        StationAction::Pick,
        StationAction::Place,
        StationAction::Charge,
    ] {
        let temp = TempDir::new().unwrap();
        let map = MapIdentity {
            map_id: Uuid::new_v4(),
            revision: 1,
            content_digest_sha256: "d".repeat(64),
        };
        let mut session = synchronized_session(&temp, map.clone());
        let mut command = plan_command(map, Uuid::new_v4(), OrderPhase::Prepare);
        command.contract_version = "1.1.0".to_owned();
        command.payload.arrival_action = Some(action);
        assert_eq!(
            session
                .accept_order_at(&command, NOW.to_owned(), 100, true)
                .unwrap()
                .disposition,
            CommandDisposition::Prepared
        );
        command.payload.goal = None;
        command.payload.route = None;
        command.payload.phase = OrderPhase::Abort;
        command.payload.command_id = Uuid::new_v4();
        let abort = session
            .accept_order_at(&command, NOW.to_owned(), 101, true)
            .unwrap();
        assert_eq!(abort.disposition, CommandDisposition::Applied);
        assert_eq!(abort.code, "PLAN_ABORTED_STOPPED");
        assert!(session.prepared_order().is_none());
        command.payload.phase = OrderPhase::Activate;
        command.payload.command_id = Uuid::new_v4();
        assert_eq!(
            session
                .accept_order_at(&command, NOW.to_owned(), 102, true)
                .unwrap()
                .code,
            "ORDER_ABORTED"
        );
    }
}

#[test]
fn cancellation_waits_for_stop_and_fences_late_activation() {
    let temp = TempDir::new().unwrap();
    let map = MapIdentity {
        map_id: Uuid::new_v4(),
        revision: 1,
        content_digest_sha256: "d".repeat(64),
    };
    let mut session = synchronized_session(&temp, map.clone());
    let mut command = plan_command(map, Uuid::new_v4(), OrderPhase::Prepare);
    session
        .accept_order_at(&command, NOW.to_owned(), 100, true)
        .unwrap();
    command.payload.phase = OrderPhase::Activate;
    command.payload.command_id = Uuid::new_v4();
    command.payload.goal = None;
    command.payload.route = None;
    session
        .accept_order_at(&command, NOW.to_owned(), 101, true)
        .unwrap();
    let activation = command.clone();
    command.payload.phase = OrderPhase::Abort;
    command.payload.command_id = Uuid::new_v4();
    let before = session.spool().pending().len();
    let stopping = session
        .accept_order_at(&command, NOW.to_owned(), 102, false)
        .unwrap();
    assert_eq!(stopping.code, "PLAN_ABORT_STOPPING");
    assert_eq!(session.spool().pending().len(), before);
    assert!(session.spool().pending_abort().is_some());
    assert!(session.applied_order().is_some());
    let stopped = session
        .accept_order_at(&command, NOW.to_owned(), 103, true)
        .unwrap();
    assert_eq!(stopped.code, "PLAN_ABORTED_STOPPED");
    assert!(session.spool().pending_abort().is_none());
    assert!(session.applied_order().is_none());
    assert_eq!(
        session
            .accept_order_at(&activation, NOW.to_owned(), 104, true)
            .unwrap()
            .code,
        "ORDER_ABORTED"
    );
    assert_eq!(
        session
            .accept_order_at(&command, NOW.to_owned(), 105, true)
            .unwrap()
            .code,
        "PLAN_ABORT_NOOP_STOPPED"
    );
}

fn independent_plan(id: &str) -> PlanCoordinator {
    let revision = revision(&[id]);
    let mut plans = prepared(&revision);
    plans
        .activate(revision.plan_revision_id, &revision.targets[0], 0)
        .unwrap();
    plans
}

#[test]
fn dynamically_inserted_robot_shares_tick_safety_and_failed_insert_is_atomic() {
    let map = open_map();
    let r1 = RobotId::new("r1").unwrap();
    let r2 = RobotId::new("r2").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [engine(
            "r1",
            map.clone(),
            WorldPosition::new(1.5, 1.5).unwrap(),
            &[],
        )],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([(r1.clone(), independent_plan("r1"))]);
    fleet
        .step_with_independent_plans(
            &BTreeMap::from([(r1.clone(), 2)]),
            &BTreeMap::new(),
            &mut plans,
        )
        .unwrap();
    let position = fleet.robots()[&r1].state().position();
    assert!(
        fleet
            .insert(engine("bad", map.clone(), position, &[]))
            .is_err()
    );
    assert_eq!(fleet.robots().len(), 1);
    fleet
        .insert(engine(
            "r2",
            map,
            WorldPosition::new(1.5, 3.5).unwrap(),
            &[],
        ))
        .unwrap();
    plans.insert(r2.clone(), independent_plan("r2"));
    let step = fleet
        .step_with_independent_plans(
            &BTreeMap::from([(r1.clone(), 2), (r2.clone(), 2)]),
            &BTreeMap::new(),
            &mut plans,
        )
        .unwrap();
    assert_eq!(step.records.len(), 2);
    assert!(step.records[&r1].state.position().x_meters() > position.x_meters());
    assert!(step.records[&r2].state.position().x_meters() > 1.5);
}

#[test]
fn independent_orders_crossing_are_held_before_collision_and_idle_robot_is_reserved() {
    let map = open_map();
    let r1 = RobotId::new("r1").unwrap();
    let r2 = RobotId::new("r2").unwrap();
    let r3 = RobotId::new("r3").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(4.5, 2.5).unwrap(),
                &[],
            ),
            engine(
                "r2",
                map.clone(),
                WorldPosition::new(3.5, 3.5).unwrap(),
                &[],
            ),
            engine("r3", map, WorldPosition::new(7.5, 3.5).unwrap(), &[]),
        ],
        FleetConfig::new(0.6, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([
        (r1.clone(), independent_plan("r1")),
        (r2.clone(), independent_plan("r2")),
        (r3.clone(), PlanCoordinator::new()),
    ]);
    let actions = BTreeMap::from([(r1.clone(), 1), (r2.clone(), 2), (r3.clone(), 1)]);
    let recovery = (0..30)
        .find_map(|_| {
            let step = fleet
                .step_with_independent_plans(&actions, &BTreeMap::new(), &mut plans)
                .unwrap();
            assert!(step.stationary_reservations.contains("r3"));
            step.recovery
        })
        .expect("crossing independently authorized robots must be held");
    assert_eq!(recovery.reason, RecoveryReason::Collision);
    for id in [&r1, &r2] {
        assert_eq!(
            plans[id].motion_authorized(id.as_str()),
            !recovery.held_robots.iter().any(|held| held == id.as_str())
        );
    }
    assert_eq!(fleet.robots()[&r3].state().position().x_meters(), 7.5);
}

#[test]
fn fleet_checkpoint_preserves_membership_station_states_and_fences_all_plans() {
    let map = open_map();
    let fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(1.5, 1.5).unwrap(),
                &[],
            ),
            engine("r2", map, WorldPosition::new(1.5, 3.5).unwrap(), &[]),
        ],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let directory = TempDir::new().unwrap();
    let store = CheckpointStore::new(directory.path().join("fleet.json")).unwrap();
    let mut checkpoint =
        fleet.checkpoint("local-fleet".into(), "b".repeat(64), PlanCoordinator::new());
    checkpoint
        .independent_plans
        .insert("r1".into(), independent_plan("r1"));
    checkpoint
        .independent_plans
        .insert("r2".into(), independent_plan("r2"));
    checkpoint.station_states.insert(
        "r2".into(),
        mapf_rl_simulator::station::StationState {
            battery_percent: 42.0,
            loaded: true,
            ..Default::default()
        },
    );
    store.save(&checkpoint).unwrap();
    let restored = store
        .load_for_restart("local-fleet", &"b".repeat(64))
        .unwrap()
        .unwrap();
    assert_eq!(restored.robots.len(), 2);
    assert_eq!(restored.station_states["r2"].battery_percent, 42.0);
    assert!(restored.station_states["r2"].loaded);
    assert!(!restored.independent_plans["r1"].motion_authorized("r1"));
    assert!(!restored.independent_plans["r2"].motion_authorized("r2"));
}

#[test]
fn removed_robot_must_stop_and_stays_out_of_checkpoint() {
    let map = open_map();
    let base = RobotId::new("r1").unwrap();
    let added = RobotId::new("r2").unwrap();
    let moving = SimulationEngine::new(
        ManualMonotonicClock::default(),
        added.clone(),
        map.clone(),
        RobotState::new(
            WorldPosition::new(1.5, 3.5).unwrap(),
            Velocity::new(0.2, 0.0).unwrap(),
            Acceleration::ZERO,
            0.0,
        )
        .unwrap(),
        EngineConfig {
            motion_limits: MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        },
        7,
        &[],
    )
    .unwrap();
    let mut fleet = MultiRobotEngine::new(
        [
            engine("r1", map, WorldPosition::new(1.5, 1.5).unwrap(), &[]),
            moving,
        ],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    assert!(!fleet.remove_stationary(&added).unwrap());
    assert_eq!(fleet.robots().len(), 2);
    fleet.latch_robot_emergency_stop(&added).unwrap();
    let mut plans = BTreeMap::from([
        (base.clone(), PlanCoordinator::new()),
        (added.clone(), PlanCoordinator::new()),
    ]);
    let actions = BTreeMap::from([(base.clone(), 0), (added.clone(), 0)]);
    for _ in 0..100 {
        fleet
            .step_with_independent_plans(&actions, &BTreeMap::new(), &mut plans)
            .unwrap();
        if fleet.remove_stationary(&added).unwrap() {
            break;
        }
    }
    assert!(!fleet.robots().contains_key(&added));
    assert!(fleet.remove_stationary(&base).is_err());
    let checkpoint = fleet.checkpoint("local-fleet".into(), "b".repeat(64), PlanCoordinator::new());
    assert_eq!(checkpoint.robots.len(), 1);
    assert_eq!(checkpoint.robots[0].robot_id, "r1");
    let directory = TempDir::new().unwrap();
    let store = CheckpointStore::new(directory.path().join("fleet.json")).unwrap();
    store.save(&checkpoint).unwrap();
    let restored = store
        .load_for_restart("local-fleet", &"b".repeat(64))
        .unwrap()
        .unwrap();
    assert_eq!(restored.robots.len(), 1);
    assert_eq!(restored.robots[0].robot_id, "r1");
}

#[test]
fn depleted_robot_brakes_safely_and_checkpoint_preserves_incident_identity() {
    let id = RobotId::new("r1").unwrap();
    let robot = engine("r1", open_map(), WorldPosition::new(2.5, 2.5).unwrap(), &[]);
    let mut fleet = MultiRobotEngine::new([robot], FleetConfig::new(0.5, 50).unwrap()).unwrap();
    let rev = revision(&["r1"]);
    let mut plans = prepared(&rev);
    plans
        .activate(rev.plan_revision_id, &rev.targets[0], 0)
        .unwrap();
    for _ in 0..10 {
        fleet
            .step(&BTreeMap::from([(id.clone(), 1)]), &mut plans)
            .unwrap();
    }
    fleet.latch_robot_emergency_stop(&id).unwrap();
    for _ in 0..30 {
        fleet
            .step(&BTreeMap::from([(id.clone(), 1)]), &mut plans)
            .unwrap();
    }
    assert!(fleet.robots()[&id].emergency_stop_latched());
    assert!(fleet.robots()[&id].state().velocity().magnitude() < 1e-6);
    let event_id = Uuid::new_v4();
    let checkpoint = fleet
        .checkpoint("sim-1".into(), "a".repeat(64), plans)
        .with_station_state(
            "r1".into(),
            mapf_rl_simulator::station::StationState {
                battery_percent: 0.0,
                ..Default::default()
            },
        )
        .with_battery_depletion("r1".into(), Some(event_id));
    let temp = TempDir::new().unwrap();
    let store = CheckpointStore::new(temp.path().join("battery.json")).unwrap();
    store.save(&checkpoint).unwrap();
    let restored = store
        .load_for_restart("sim-1", &"a".repeat(64))
        .unwrap()
        .unwrap();
    assert_eq!(restored.station_states["r1"].battery_percent, 0.0);
    assert_eq!(restored.battery_depletion_ids["r1"], event_id);
}

#[test]
fn collision_recovery_does_not_fence_an_unrelated_independent_order() {
    let map = open_map();
    let r1 = RobotId::new("r1").unwrap();
    let r2 = RobotId::new("r2").unwrap();
    let r3 = RobotId::new("r3").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(1.5, 1.5).unwrap(),
                &[],
            ),
            engine(
                "r2",
                map.clone(),
                WorldPosition::new(3.5, 1.5).unwrap(),
                &[],
            ),
            engine("r3", map, WorldPosition::new(1.5, 3.5).unwrap(), &[]),
        ],
        FleetConfig::new(0.6, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([
        (r1.clone(), independent_plan("r1")),
        (r2.clone(), PlanCoordinator::new()),
        (r3.clone(), independent_plan("r3")),
    ]);
    let actions = BTreeMap::from([(r1.clone(), 2), (r3.clone(), 2)]);
    let step = (0..30)
        .find_map(|_| {
            let step = fleet
                .step_with_independent_plans(&actions, &BTreeMap::new(), &mut plans)
                .unwrap();
            step.recovery.is_some().then_some(step)
        })
        .expect("moving robot must stop before the parked robot");
    assert_eq!(step.recovery.unwrap().held_robots, vec!["r1"]);
    assert!(!plans[&r1].motion_authorized("r1"));
    assert!(plans[&r3].motion_authorized("r3"));
    let before = fleet.robots()[&r3].state().position().x_meters();
    fleet
        .step_with_independent_plans(&actions, &BTreeMap::new(), &mut plans)
        .unwrap();
    assert!(fleet.robots()[&r3].state().position().x_meters() > before);
}

#[test]
fn waypoint_stop_and_turn_do_not_predict_motion_through_a_parked_robot() {
    let map = open_map();
    let r1 = RobotId::new("r1").unwrap();
    let r2 = RobotId::new("r2").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(2.5, 2.5).unwrap(),
                &[],
            ),
            engine("r2", map, WorldPosition::new(8.5, 2.5).unwrap(), &[]),
        ],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([
        (r1.clone(), independent_plan("r1")),
        (r2.clone(), PlanCoordinator::new()),
    ]);
    for target in [
        WorldPosition::new(7.5, 2.5).unwrap(),
        WorldPosition::new(7.5, 3.5).unwrap(),
    ] {
        let mut arrived = false;
        for _ in 0..150 {
            let state = fleet.robots()[&r1].state();
            let action = mapf_rl_simulator::route::action_toward_position(state.position(), target);
            let step = fleet
                .step_with_independent_plans(
                    &BTreeMap::from([(r1.clone(), action)]),
                    &BTreeMap::from([(r1.clone(), target)]),
                    &mut plans,
                )
                .unwrap();
            assert!(
                step.recovery.is_none(),
                "a safe waypoint stop must not be extrapolated past its target"
            );
            let state = step.records[&r1].state;
            if (state.position().x_meters() - target.x_meters()).abs() < 1e-6
                && (state.position().y_meters() - target.y_meters()).abs() < 1e-6
                && state.velocity().magnitude() < 1e-6
            {
                arrived = true;
                break;
            }
        }
        assert!(arrived);
    }
}

#[test]
fn target_on_a_parked_robot_still_triggers_collision_recovery() {
    let map = open_map();
    let r1 = RobotId::new("r1").unwrap();
    let r2 = RobotId::new("r2").unwrap();
    let target = WorldPosition::new(4.5, 2.5).unwrap();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(
                "r1",
                map.clone(),
                WorldPosition::new(1.5, 2.5).unwrap(),
                &[],
            ),
            engine("r2", map, target, &[]),
        ],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([
        (r1.clone(), independent_plan("r1")),
        (r2.clone(), PlanCoordinator::new()),
    ]);
    let recovery = (0..100)
        .find_map(|_| {
            fleet
                .step_with_independent_plans(
                    &BTreeMap::from([(r1.clone(), 2)]),
                    &BTreeMap::from([(r1.clone(), target)]),
                    &mut plans,
                )
                .unwrap()
                .recovery
        })
        .expect("target control must not disable collision prediction");
    assert!(recovery.held_robots.contains(&"r1".into()));
    assert!(!plans[&r1].motion_authorized("r1"));
    assert!(fleet.robots()[&r1].state().position().x_meters() < 4.0);
}
