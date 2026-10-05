//! Production composition of Core transport, durable plan state, and fleet safety.

use crate::checkpoint::CheckpointStore;
use crate::controller::{BaselineController, RuntimeProfile};
use crate::core_client::{ApiKey, CoreClient, CoreClientConfig};
use crate::fleet::{FleetConfig, MultiRobotEngine, RecoveryReason};
use crate::motion::MotionLimits;
use crate::plan::{PlanCoordinator, PlanRevision, PlanTarget};
use crate::protocol::{
    CommandDisposition, EventSeverity, MapIdentity, OrderCommand, OrderGoal, OrderPhase,
    PoseReport, RasterMapContent,
};
use crate::route::{NODE_CENTER_TOLERANCE_METERS, action_toward_position, motion_target_for_route};
use crate::runtime::{IncidentReport, Phase2Runtime, RuntimeEvent};
use crate::safety::SafetyConfig;
use crate::sensing::SensorConfig;
use crate::session::CoreSession;
use crate::simulation::{EngineConfig, ManualMonotonicClock, SimulationEngine};
use crate::spool::DurableSpool;
use crate::types::{Acceleration, RobotId, RobotState, Velocity, WorldPosition};
use crate::world::{GridCell, GridMap};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::env;
use std::error::Error;
use std::io;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use url::Url;
use uuid::Uuid;

#[derive(Clone)]
pub(super) struct OperationalConfig {
    pub(super) profile: RuntimeProfile,
    pub(super) simulator_id: String,
    pub(super) robot_id: String,
    pub(super) rest_base_url: Url,
    pub(super) websocket_url: Url,
    pub(super) api_key: ApiKey,
    pub(super) spool_path: PathBuf,
    pub(super) checkpoint_path: PathBuf,
    pub(super) map: MapIdentity,
    pub(super) start: GridCell,
    pub(super) master_seed: u64,
    pub(super) exit_after_completion: bool,
    pub(super) station_config: crate::station::StationConfig,
    pub(super) initial_battery_percent: f64,
    pub(super) battery_config: crate::station::BatteryConfig,
}

impl OperationalConfig {
    pub(super) fn from_environment() -> Result<Self, Box<dyn Error>> {
        let profile = RuntimeProfile::parse(&required("MAPF_PROFILE")?)?;
        let simulator_id = required("MAPF_SIMULATOR_ID")?;
        let robot_id = required("MAPF_SIMULATOR_ROBOT_ID")?;
        let rest_base_url = Url::parse(&required("MAPF_SIMULATOR_CORE_REST_URL")?)?;
        let websocket_url = Url::parse(&required("MAPF_SIMULATOR_CORE_WS_URL")?)?;
        let api_key = ApiKey::new(&required("MAPF_SIMULATOR_API_KEY")?)?;
        let spool_path = PathBuf::from(required("MAPF_SIMULATOR_SPOOL_PATH")?);
        let checkpoint_path = env::var("MAPF_SIMULATOR_CHECKPOINT_PATH")
            .map(PathBuf::from)
            .unwrap_or_else(|_| spool_path.with_extension("safety.json"));
        let map = MapIdentity {
            map_id: Uuid::parse_str(&required("MAPF_SIMULATOR_MAP_ID")?)?,
            revision: parse("MAPF_SIMULATOR_MAP_REVISION")?,
            content_digest_sha256: required("MAPF_SIMULATOR_MAP_DIGEST_SHA256")?,
        };
        map.validate()?;
        Ok(Self {
            profile,
            simulator_id,
            robot_id,
            rest_base_url,
            websocket_url,
            api_key,
            spool_path,
            checkpoint_path,
            map,
            start: GridCell::new(
                parse_or("MAPF_SIMULATOR_START_COLUMN", 0)?,
                parse_or("MAPF_SIMULATOR_START_ROW", 0)?,
            ),
            master_seed: parse_or("MAPF_SIMULATOR_MASTER_SEED", 0x5eed_2026)?,
            station_config: crate::station::StationConfig {
                pick_duration_ms: parse_or("MAPF_SIMULATOR_PICK_DURATION_MS", 2000)?,
                place_duration_ms: parse_or("MAPF_SIMULATOR_PLACE_DURATION_MS", 2000)?,
                charge_percent_per_second: parse_or(
                    "MAPF_SIMULATOR_CHARGE_PERCENT_PER_SECOND",
                    1.0,
                )?,
            },
            battery_config: crate::station::BatteryConfig {
                drive_percent_per_meter: parse_or("MAPF_SIMULATOR_DRIVE_PERCENT_PER_METER", 0.1)?,
                idle_percent_per_second: parse_or("MAPF_SIMULATOR_IDLE_PERCENT_PER_SECOND", 0.001)?,
                loaded_multiplier: parse_or("MAPF_SIMULATOR_LOADED_BATTERY_MULTIPLIER", 1.5)?,
            },
            initial_battery_percent: parse_or("MAPF_SIMULATOR_INITIAL_BATTERY_PERCENT", 100.0)?,
            exit_after_completion: parse_bool_or("MAPF_SIMULATOR_EXIT_AFTER_COMPLETION", false)?,
        })
    }
    pub(super) fn validate(&self) -> Result<(), io::Error> {
        if !self.battery_config.valid()
            || self.station_config.pick_duration_ms == 0
            || self.station_config.place_duration_ms == 0
            || !self.station_config.charge_percent_per_second.is_finite()
            || self.station_config.charge_percent_per_second <= 0.0
            || !self.initial_battery_percent.is_finite()
            || !(0.0..=100.0).contains(&self.initial_battery_percent)
        {
            return Err(invalid("invalid station simulation configuration"));
        }
        Ok(())
    }
}

pub async fn run() -> Result<(), Box<dyn Error>> {
    let config = OperationalConfig::from_environment()?;
    config.validate()?;
    let controller = BaselineController::explicit(config.profile)?;
    let spool = DurableSpool::open(&config.spool_path, &config.simulator_id)?;
    let session = CoreSession::new(
        config.simulator_id.clone(),
        config.robot_id.clone(),
        config.map.clone(),
        controller.report().clone(),
        spool,
    )?;
    let client = CoreClient::new(CoreClientConfig::new_with_local_compose(
        config.profile,
        config.simulator_id.clone(),
        config.rest_base_url,
        config.websocket_url,
        config.api_key,
        parse_bool_or("MAPF_SIMULATOR_LOCAL_COMPOSE", false)?,
    )?)?;
    let mut runtime = Phase2Runtime::new(client, session);
    let started = Instant::now();
    let snapshot = runtime.connect_and_replay(0).await?;
    let robot = snapshot
        .robots
        .iter()
        .find(|robot| robot.robot_id == config.robot_id)
        .ok_or_else(|| invalid("Core snapshot omits the configured robot"))?;
    let map_content = robot
        .map_content
        .as_ref()
        .ok_or_else(|| invalid("Core snapshot omits authoritative map content"))?;
    let map = build_map(map_content)?;
    let initial_position = map.grid_to_world(config.start)?;
    if map.is_blocked(config.start)? {
        return Err(invalid("configured start cell is blocked").into());
    }
    let engine = SimulationEngine::new(
        ManualMonotonicClock::default(),
        RobotId::new(config.robot_id.clone())?,
        map,
        RobotState::new(initial_position, Velocity::ZERO, Acceleration::ZERO, 0.0)?,
        EngineConfig {
            motion_limits: MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0)?,
            safety: SafetyConfig::new(0.2, 0.05)?,
            sensor: SensorConfig::new(0.0, 0)?,
        },
        config.master_seed,
        &[],
    )?;
    let robot_id = RobotId::new(config.robot_id.clone())?;
    let mut fleet = MultiRobotEngine::new([engine], FleetConfig::new(0.5, 50)?)?;
    let checkpoint_store = CheckpointStore::new(&config.checkpoint_path)?;
    let mut station = crate::station::StationState {
        battery_percent: config.initial_battery_percent,
        ..Default::default()
    };
    let mut plans = PlanCoordinator::new();
    let mut restart_plan_revision_id = None;
    let mut battery_depletion_id = None;
    if let Some(checkpoint) = checkpoint_store
        .load_for_restart(&config.simulator_id, &config.map.content_digest_sha256)?
    {
        if let Some(state) = checkpoint.station_states.get(&config.robot_id) {
            station = state.clone();
        }
        plans = fleet.restore_checkpoint(&checkpoint)?;
        restart_plan_revision_id = plans.active_revision_id();
        battery_depletion_id = checkpoint
            .battery_depletion_ids
            .get(&config.robot_id)
            .copied();
    }
    let mut completion_queued = false;
    let mut recovery_reported = false;
    let mut station_failure_reported = false;
    let mut battery_depletion_reported = false;
    let mut interval = tokio::time::interval(Duration::from_millis(100));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    loop {
        interval.tick().await;
        let monotonic_ms = elapsed_milliseconds(started);
        {
            let applied = runtime.applied_order().await?;
            let preparing = runtime.prepared_order().await?.is_some();
            if station.battery_percent <= 0.0 {
                fleet.latch_robot_emergency_stop(&robot_id)?;
            }
            let engine = &fleet.robots()[&robot_id];
            let previous_position = engine.state().position();
            let stopped_at_goal_before_tick = applied
                .as_ref()
                .and_then(|order| order.goal)
                .is_some_and(|goal| goal_reached_and_stopped(engine, goal));
            let mut route_deviation = false;
            let abort_pending = runtime.abort_pending().await?;
            let target = if preparing
                || abort_pending
                || station.battery_percent <= 0.0
                || !runtime.synchronized().await?
            {
                None
            } else if let Some(order) = &applied {
                if let Some(route) = &order.route {
                    if route.execution_control.as_deref() != Some("occupancy-rights-v1") {
                        route_deviation = true;
                        None
                    } else {
                        match motion_target_for_route(
                            engine.map(),
                            engine.state(),
                            engine.simulation_time(),
                            route,
                        ) {
                            Ok(target) => Some(target),
                            Err(_) => {
                                route_deviation = true;
                                None
                            }
                        }
                    }
                } else {
                    order.goal.and_then(|goal| {
                        engine
                            .map()
                            .grid_to_world(GridCell::new(goal.column, goal.row))
                            .ok()
                            .map(Into::into)
                    })
                }
            } else {
                None
            };
            let action = target.map_or(0, |target| {
                action_toward_position(engine.state().position(), target.position)
            });
            let targets = target
                .map(|target| BTreeMap::from([(robot_id.clone(), target)]))
                .unwrap_or_default();
            let mut fleet_plans = BTreeMap::from([(robot_id.clone(), plans.clone())]);
            let step = fleet.step_with_passage_rights(
                &BTreeMap::from([(robot_id.clone(), action)]),
                &targets,
                &mut fleet_plans,
            )?;
            plans = fleet_plans
                .remove(&robot_id)
                .ok_or_else(|| invalid("fleet omitted plan"))?;
            let record = step
                .records
                .get(&robot_id)
                .ok_or_else(|| invalid("fleet step omitted the configured robot"))?;
            if let Some(plan_revision_id) =
                applied.as_ref().and_then(|order| order.plan_revision_id)
                && plans.motion_authorized(&config.robot_id)
            {
                plans.confirm_runtime_safe(plan_revision_id, &config.robot_id)?;
            }
            let charging = station.battery_percent > 0.0
                && !preparing
                && !abort_pending
                && runtime.synchronized().await?
                && plans.motion_authorized(&config.robot_id)
                && !fleet.robots()[&robot_id].emergency_stop_latched()
                && !route_deviation
                && step.recovery.is_none()
                && record.safety_outcome == crate::safety::SafetyOutcome::Accept
                && stopped_at_goal_before_tick
                && applied.as_ref().is_some_and(|order| {
                    order.arrival_action == Some(crate::contracts::generated::StationAction::Charge)
                        && order.goal.is_some_and(|goal| {
                            goal_reached_and_stopped(&fleet.robots()[&robot_id], goal)
                        })
                });
            let distance = distance_meters(previous_position, record.state.position());
            station.consume_battery(distance, 100, charging, config.battery_config);
            if station.battery_percent <= 0.0 {
                fleet.latch_robot_emergency_stop(&robot_id)?;
                battery_depletion_id
                    .get_or_insert_with(|| battery_event_id(&config.robot_id, record.tick.get()));
            }
            let engine = &fleet.robots()[&robot_id];
            if route_deviation && !recovery_reported {
                plans.hold_for_recovery(None)?;
            }
            if !recovery_reported && (route_deviation || step.recovery.is_some()) {
                let (code, held_robots) = if route_deviation {
                    ("SAFETY_ROUTE_DEVIATION", vec![config.robot_id.clone()])
                } else {
                    let recovery = step.recovery.as_ref().expect("checked recovery");
                    let code = match recovery.reason {
                        RecoveryReason::Collision => "COLLISION_RISK",
                        RecoveryReason::CorridorConflict => "CORRIDOR_CONFLICT",
                        RecoveryReason::Deadlock => "DEADLOCK_DETECTED",
                        RecoveryReason::RobotFailure => "ROBOT_FAILURE",
                    };
                    (code, recovery.held_robots.clone())
                };
                let mut evidence = serde_json::Map::new();
                evidence.insert("heldRobots".to_owned(), serde_json::json!(held_robots));
                if let Some(order) = &applied {
                    evidence.insert("orderId".to_owned(), order.order_id.clone().into());
                    evidence.insert(
                        "orderUpdateId".to_owned(),
                        serde_json::Value::from(order.order_update_id),
                    );
                }
                runtime
                    .publish_incident(
                        IncidentReport {
                            event_id: Uuid::new_v4(),
                            severity: EventSeverity::Critical,
                            code: code.to_owned(),
                            simulation_time_ms: engine.simulation_time().get(),
                            evidence,
                            occurred_at: utc_now_milliseconds()?,
                        },
                        monotonic_ms,
                    )
                    .await?;
                recovery_reported = true;
            }
            let station_safe = station.battery_percent > 0.0
                && !abort_pending
                && runtime.synchronized().await?
                && plans.motion_authorized(&config.robot_id)
                && !engine.emergency_stop_latched()
                && !route_deviation
                && step.recovery.is_none()
                && record.safety_outcome == crate::safety::SafetyOutcome::Accept;
            if let Some(order) = &applied
                && let Some(action) = order.arrival_action
            {
                let arrived = order
                    .goal
                    .is_some_and(|goal| goal_reached_and_stopped(engine, goal));
                station.advance(
                    &order.order_id,
                    action,
                    station_safe && arrived && stopped_at_goal_before_tick && !preparing,
                    100,
                    config.station_config,
                );
            }
            runtime
                .set_station_state(
                    station.clone(),
                    station_safe
                        || applied.is_none()
                            && !abort_pending
                            && !engine.emergency_stop_latched()
                            && !route_deviation
                            && step.recovery.is_none()
                            && record.safety_outcome == crate::safety::SafetyOutcome::Accept,
                )
                .await?;
            save_checkpoint(
                &checkpoint_store,
                fleet
                    .checkpoint(
                        config.simulator_id.clone(),
                        config.map.content_digest_sha256.clone(),
                        plans.clone(),
                    )
                    .with_station_state(config.robot_id.clone(), station.clone())
                    .with_battery_depletion(config.robot_id.clone(), battery_depletion_id),
            )
            .await?;
            if let Some(event_id) = battery_depletion_id
                && !battery_depletion_reported
            {
                runtime
                    .publish_incident(
                        battery_incident(
                            event_id,
                            engine.simulation_time().get(),
                            utc_now_milliseconds()?,
                        ),
                        monotonic_ms,
                    )
                    .await?;
                battery_depletion_reported = true;
            }
            if station.phase == crate::station::StationPhase::Failed && !station_failure_reported {
                let mut evidence = serde_json::Map::new();
                evidence.insert("stationState".to_owned(), serde_json::to_value(&station)?);
                runtime
                    .publish_incident(
                        IncidentReport {
                            event_id: Uuid::new_v4(),
                            severity: EventSeverity::Critical,
                            code: "STATION_ACTION_FAILED".to_owned(),
                            simulation_time_ms: engine.simulation_time().get(),
                            evidence,
                            occurred_at: utc_now_milliseconds()?,
                        },
                        monotonic_ms,
                    )
                    .await?;
                station_failure_reported = true;
            }
            if abort_pending
                && engine.state().velocity().magnitude() < 1.0e-6
                && engine.state().acceleration().magnitude() < 1.0e-6
            {
                runtime
                    .publish_state_now(
                        monotonic_ms,
                        engine.tick().get(),
                        engine.simulation_time().get(),
                        PoseReport {
                            x_meters: record.state.position().x_meters(),
                            y_meters: record.state.position().y_meters(),
                            yaw_radians: record.state.yaw_radians(),
                        },
                        utc_now_milliseconds()?,
                    )
                    .await?;
                runtime
                    .finish_abort_if_stopped(utc_now_milliseconds()?)
                    .await?;
            }
            let occurred_at = utc_now_milliseconds()?;
            runtime
                .publish_state_if_due(
                    monotonic_ms,
                    engine.tick().get(),
                    engine.simulation_time().get(),
                    PoseReport {
                        x_meters: record.state.position().x_meters(),
                        y_meters: record.state.position().y_meters(),
                        yaw_radians: record.state.yaw_radians(),
                    },
                    occurred_at,
                )
                .await?;
            if let Some(plan_revision_id) = restart_plan_revision_id {
                let mut evidence = serde_json::Map::new();
                evidence.insert(
                    "planRevisionId".to_owned(),
                    plan_revision_id.to_string().into(),
                );
                runtime
                    .publish_incident(
                        IncidentReport {
                            event_id: stable_restart_event_id(plan_revision_id),
                            severity: EventSeverity::Critical,
                            code: "SAFETY_RESTART_RECONCILIATION_REQUIRED".to_owned(),
                            simulation_time_ms: engine.simulation_time().get(),
                            evidence,
                            occurred_at: utc_now_milliseconds()?,
                        },
                        monotonic_ms,
                    )
                    .await?;
                restart_plan_revision_id = None;
            }
            if !completion_queued
                && !abort_pending
                && applied.as_ref().is_some_and(|order| {
                    station_safe
                        && order
                            .arrival_action
                            .is_none_or(|action| station.completed(&order.order_id, action))
                        && order
                            .goal
                            .is_some_and(|goal| goal_reached_and_stopped(engine, goal))
                        && order.route.as_ref().is_none_or(|route| {
                            route.waypoints.last().is_some_and(|waypoint| {
                                engine.simulation_time().get()
                                    >= i64::try_from(waypoint.start_simulation_time_ms)
                                        .unwrap_or(i64::MAX)
                            })
                        })
                })
            {
                runtime
                    .publish_state_now(
                        monotonic_ms,
                        engine.tick().get(),
                        engine.simulation_time().get(),
                        PoseReport {
                            x_meters: record.state.position().x_meters(),
                            y_meters: record.state.position().y_meters(),
                            yaw_radians: record.state.yaw_radians(),
                        },
                        utc_now_milliseconds()?,
                    )
                    .await?;
                runtime
                    .publish_order_completed(
                        engine.simulation_time().get(),
                        utc_now_milliseconds()?,
                        monotonic_ms,
                    )
                    .await?;
                completion_queued = true;
            }
        }
        // Only the socket read has a timeout; durable ACK/command handling
        // must finish before another physics tick can start.
        while let Some(event) = runtime
            .poll_one_with_motion(
                monotonic_ms,
                utc_now_milliseconds()?,
                fleet.robots()[&robot_id].state().velocity().magnitude() < 1.0e-6
                    && fleet.robots()[&robot_id].state().acceleration().magnitude() < 1.0e-6,
            )
            .await?
        {
            match event {
                RuntimeEvent::Order { decision, command } => {
                    if command.payload.phase == OrderPhase::Prepare
                        && decision.disposition == CommandDisposition::Prepared
                    {
                        let revision = local_revision(&command)?;
                        plans.begin_prepare(revision.clone(), monotonic_ms)?;
                        plans.mark_safe_hold(
                            revision.plan_revision_id,
                            &config.robot_id,
                            true,
                            monotonic_ms,
                        )?;
                    } else if command.payload.phase == OrderPhase::Abort
                        && decision.disposition != CommandDisposition::Rejected
                    {
                        if let Some(plan_revision_id) = command.payload.plan_revision_id
                            && plans.active_revision_id() == Some(plan_revision_id)
                        {
                            plans.abort(plan_revision_id)?;
                        }
                    } else if decision.apply_to_robot {
                        let applied = runtime
                            .applied_order()
                            .await?
                            .ok_or_else(|| invalid("applied Order checkpoint is missing"))?;
                        let goal = applied
                            .goal
                            .ok_or_else(|| invalid("prepared Order omits its goal"))?;
                        validate_goal(fleet.robots()[&robot_id].map(), goal)?;
                        if let Some(plan_revision_id) = applied.plan_revision_id {
                            plans.activate(
                                plan_revision_id,
                                &PlanTarget {
                                    robot_id: config.robot_id.clone(),
                                    order_id: applied.order_id,
                                    order_update_id: applied.order_update_id,
                                    content_digest_sha256: applied.content_digest_sha256,
                                },
                                monotonic_ms,
                            )?;
                        } else {
                            let revision = PlanRevision {
                                plan_revision_id: command.payload.command_id,
                                planning_snapshot_digest_sha256: applied
                                    .content_digest_sha256
                                    .clone(),
                                targets: vec![PlanTarget {
                                    robot_id: config.robot_id.clone(),
                                    order_id: applied.order_id,
                                    order_update_id: applied.order_update_id,
                                    content_digest_sha256: applied.content_digest_sha256,
                                }],
                                activation_order: vec![config.robot_id.clone()],
                            };
                            plans.begin_prepare(revision.clone(), monotonic_ms)?;
                            plans.mark_safe_hold(
                                revision.plan_revision_id,
                                &config.robot_id,
                                true,
                                monotonic_ms,
                            )?;
                            plans.activate(
                                revision.plan_revision_id,
                                &revision.targets[0],
                                monotonic_ms,
                            )?;
                        }
                        station_failure_reported = false;
                        completion_queued = false;
                        recovery_reported = false;
                    }
                    save_checkpoint(
                        &checkpoint_store,
                        fleet
                            .checkpoint(
                                config.simulator_id.clone(),
                                config.map.content_digest_sha256.clone(),
                                plans.clone(),
                            )
                            .with_station_state(config.robot_id.clone(), station.clone())
                            .with_battery_depletion(config.robot_id.clone(), battery_depletion_id),
                    )
                    .await?;
                }
                RuntimeEvent::ReportAcknowledged(_) => {
                    if completion_queued && runtime.applied_order().await?.is_none() {
                        plans = PlanCoordinator::new();
                        save_checkpoint(
                            &checkpoint_store,
                            fleet
                                .checkpoint(
                                    config.simulator_id.clone(),
                                    config.map.content_digest_sha256.clone(),
                                    plans.clone(),
                                )
                                .with_station_state(config.robot_id.clone(), station.clone())
                                .with_battery_depletion(
                                    config.robot_id.clone(),
                                    battery_depletion_id,
                                ),
                        )
                        .await?;
                        if config.exit_after_completion {
                            return Ok(());
                        }
                        station_failure_reported = false;
                        completion_queued = false;
                    }
                }
            }
        }
    }
}

pub(super) fn local_revision(command: &OrderCommand) -> Result<PlanRevision, Box<dyn Error>> {
    let plan_revision_id = command
        .payload
        .plan_revision_id
        .ok_or_else(|| invalid("PREPARE omits planRevisionId"))?;
    let route = command
        .payload
        .route
        .as_ref()
        .ok_or_else(|| invalid("PREPARE omits its time-indexed route"))?;
    let target = PlanTarget {
        robot_id: command.robot_id.clone(),
        order_id: command.payload.order_id.clone(),
        order_update_id: command.payload.order_update_id,
        content_digest_sha256: command.payload.content_digest_sha256.clone(),
    };
    Ok(PlanRevision {
        plan_revision_id,
        planning_snapshot_digest_sha256: route.plan_digest_sha256.clone(),
        targets: vec![target],
        activation_order: vec![command.robot_id.clone()],
    })
}

pub(super) fn distance_meters(
    a: crate::types::WorldPosition,
    b: crate::types::WorldPosition,
) -> f64 {
    (a.x_meters() - b.x_meters()).hypot(a.y_meters() - b.y_meters())
}

pub(super) fn battery_event_id(robot_id: &str, tick: u64) -> Uuid {
    let mut hash = Sha256::new();
    hash.update(b"battery-depleted:");
    hash.update(robot_id.as_bytes());
    hash.update(tick.to_be_bytes());
    let digest = hash.finalize();
    let mut bytes = [0; 16];
    bytes.copy_from_slice(&digest[..16]);
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    Uuid::from_bytes(bytes)
}

pub(super) fn battery_incident(
    event_id: Uuid,
    simulation_time_ms: i64,
    occurred_at: String,
) -> IncidentReport {
    IncidentReport {
        event_id,
        severity: EventSeverity::Critical,
        code: "BATTERY_DEPLETED".to_owned(),
        simulation_time_ms,
        evidence: serde_json::Map::from_iter([(
            "batteryPercent".to_owned(),
            serde_json::json!(0.0),
        )]),
        occurred_at,
    }
}

pub(super) fn stable_restart_event_id(plan_revision_id: Uuid) -> Uuid {
    let mut hasher = Sha256::new();
    hasher.update(plan_revision_id.as_bytes());
    hasher.update(b"wave3-restart-reconciliation");
    let digest = hasher.finalize();
    let mut bytes = [0_u8; 16];
    bytes.copy_from_slice(&digest[..16]);
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    Uuid::from_bytes(bytes)
}

pub(super) async fn save_checkpoint(
    store: &CheckpointStore,
    checkpoint: crate::checkpoint::RecoveryCheckpoint,
) -> Result<(), crate::checkpoint::CheckpointError> {
    let store = store.clone();
    tokio::task::spawn_blocking(move || store.save(&checkpoint))
        .await
        .map_err(|_| crate::checkpoint::CheckpointError::Corrupt)?
}

pub(super) fn build_map(content: &RasterMapContent) -> Result<GridMap, Box<dyn Error>> {
    content.validate()?;
    let blocked = content
        .cells
        .iter()
        .copied()
        .enumerate()
        .filter(|(_, cell)| *cell == 1)
        .map(|(index, _)| {
            GridCell::new(
                (index % content.width_cells as usize) as u32,
                (index / content.width_cells as usize) as u32,
            )
        });
    Ok(GridMap::new(
        content.width_cells,
        content.height_cells,
        WorldPosition::new(content.origin.x_meters, content.origin.y_meters)?,
        content.resolution_meters,
        blocked,
    )?)
}

pub(super) fn validate_goal(map: &GridMap, goal: OrderGoal) -> Result<(), Box<dyn Error>> {
    map.grid_to_world(GridCell::new(goal.column, goal.row))?;
    if map.is_blocked(GridCell::new(goal.column, goal.row))? {
        return Err(invalid("Core Order goal is blocked").into());
    }
    Ok(())
}

pub(super) fn goal_reached_and_stopped(
    engine: &SimulationEngine<ManualMonotonicClock>,
    goal: OrderGoal,
) -> bool {
    engine
        .map()
        .grid_to_world(GridCell::new(goal.column, goal.row))
        .is_ok_and(|center| {
            (engine.state().position().x_meters() - center.x_meters())
                .hypot(engine.state().position().y_meters() - center.y_meters())
                <= NODE_CENTER_TOLERANCE_METERS
        })
        && engine.state().velocity().magnitude() < 1.0e-6
        && engine.state().acceleration().magnitude() < 1.0e-6
}

fn required(name: &'static str) -> Result<String, io::Error> {
    env::var(name).map_err(|_| invalid(&format!("{name} is required")))
}

fn parse<T>(name: &'static str) -> Result<T, io::Error>
where
    T: std::str::FromStr,
{
    required(name)?
        .parse()
        .map_err(|_| invalid(&format!("{name} is invalid")))
}

fn parse_or<T>(name: &'static str, fallback: T) -> Result<T, io::Error>
where
    T: std::str::FromStr,
{
    match env::var(name) {
        Ok(value) => value
            .parse()
            .map_err(|_| invalid(&format!("{name} is invalid"))),
        Err(env::VarError::NotPresent) => Ok(fallback),
        Err(_) => Err(invalid(&format!("{name} is invalid"))),
    }
}

pub(super) fn parse_bool_or(name: &'static str, fallback: bool) -> Result<bool, io::Error> {
    match env::var(name) {
        Ok(value) if value == "true" => Ok(true),
        Ok(value) if value == "false" => Ok(false),
        Ok(_) => Err(invalid(&format!("{name} must be true or false"))),
        Err(env::VarError::NotPresent) => Ok(fallback),
        Err(_) => Err(invalid(&format!("{name} is invalid"))),
    }
}

pub(super) fn elapsed_milliseconds(started: Instant) -> u64 {
    u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX)
}

pub(super) fn utc_now_milliseconds() -> Result<String, io::Error> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| invalid("system clock predates the Unix epoch"))?;
    let seconds = i64::try_from(duration.as_secs())
        .map_err(|_| invalid("system clock exceeds supported UTC range"))?;
    let days = seconds / 86_400;
    let seconds_of_day = seconds % 86_400;
    let (year, month, day) = civil_from_days(days);
    let hour = seconds_of_day / 3_600;
    let minute = seconds_of_day % 3_600 / 60;
    let second = seconds_of_day % 60;
    Ok(format!(
        "{year:04}-{month:02}-{day:02}T{hour:02}:{minute:02}:{second:02}.{:03}Z",
        duration.subsec_millis()
    ))
}

fn civil_from_days(days_since_epoch: i64) -> (i64, i64, i64) {
    let shifted = days_since_epoch + 719_468;
    let era = shifted / 146_097;
    let day_of_era = shifted - era * 146_097;
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let mut year = year_of_era + era * 400;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_prime = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * month_prime + 2) / 5 + 1;
    let month = month_prime + if month_prime < 10 { 3 } else { -9 };
    year += i64::from(month <= 2);
    (year, month, day)
}

pub(super) fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unix_epoch_conversion_uses_utc_calendar() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(20_688), (2026, 8, 23));
    }

    #[test]
    fn restart_incident_identity_is_stable_uuid_v4() {
        let revision = Uuid::parse_str("123e4567-e89b-42d3-a456-426614174000").unwrap();
        let identity = stable_restart_event_id(revision);
        assert_eq!(identity, stable_restart_event_id(revision));
        assert_eq!(identity.get_version_num(), 4);
    }
}
