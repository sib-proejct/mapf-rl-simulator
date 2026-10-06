//! Local dynamic fleet: per-robot sessions with one atomic deterministic physics tick.
use crate::checkpoint::CheckpointStore;
use crate::contracts::provisioning_generated::{
    AcknowledgeMotionProfilesRequest, MotionProfile, MotionProfileTarget, MotionProfilesOutcome,
    RobotRemovalOutcome, RuntimeClaimOutcome, RuntimeClaimRequest, RuntimeRemovalOutcome,
    RuntimeRemovalRequest, RuntimeResultRequest, RuntimeRobotClaim,
};
use crate::controller::BaselineController;
use crate::core_client::{ApiKey, CoreClient, CoreClientConfig};
use crate::fleet::{FleetConfig, FleetStep, MultiRobotEngine, RecoveryReason};
use crate::motion::MotionLimits;
use crate::operational::*;
use crate::plan::{PlanCoordinator, PlanRevision, PlanTarget};
use crate::protocol::{CommandDisposition, EventSeverity, OrderPhase, PoseReport};
use crate::route::{action_toward_position, motion_target_for_route};
use crate::runtime::{IncidentReport, Phase2Runtime, RuntimeEvent};
use crate::safety::SafetyConfig;
use crate::sensing::SensorConfig;
use crate::session::CoreSession;
use crate::simulation::{EngineConfig, ManualMonotonicClock, SimulationEngine};
use crate::spool::{AppliedOrder, DurableSpool};
use crate::types::{Acceleration, RobotId, RobotState, Velocity, WorldPosition};
use crate::world::GridCell;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};
use uuid::Uuid;

type Fleet = MultiRobotEngine<ManualMonotonicClock>;

// Fleet physics remains atomic; only robots held by this recovery lose authority.
fn recovery_affects(step: &FleetStep, robot_id: &str) -> bool {
    step.recovery
        .as_ref()
        .is_some_and(|recovery| recovery.held_robots.iter().any(|held| held == robot_id))
}

struct FleetStartup {
    recovered: Option<crate::checkpoint::RecoveryCheckpoint>,
    initial_claims: Vec<RuntimeRobotClaim>,
    initial_removals: Vec<RobotRemovalOutcome>,
}

struct RobotRuntime {
    config: OperationalConfig,
    runtime: Phase2Runtime,
    plans: PlanCoordinator,
    station: crate::station::StationState,
    restart_plan_revision_id: Option<Uuid>,
    completion_queued: bool,
    recovery_reported: bool,
    station_failure_reported: bool,
    connected: bool,
    battery_depletion_id: Option<Uuid>,
    battery_depletion_reported: bool,
    reconnect: Option<tokio::task::JoinHandle<Result<Phase2Runtime, crate::runtime::RuntimeError>>>,
    retry_attempt: u32,
    next_retry_ms: u64,
}

impl Drop for RobotRuntime {
    fn drop(&mut self) {
        if let Some(task) = &self.reconnect {
            task.abort();
        }
    }
}

fn retry_delay_ms(robot_id: &str, attempt: u32) -> u64 {
    let digest = Sha256::digest(robot_id.as_bytes());
    let jitter = u64::from(u16::from_be_bytes([digest[0], digest[1]])) % 1000;
    ((1000_u64 << attempt.min(5)) + jitter).min(30_000)
}

async fn lease_expired(started: Instant, renewed_ms: &AtomicU64) {
    loop {
        let deadline = renewed_ms.load(Ordering::Relaxed).saturating_add(10_000);
        let now = elapsed_milliseconds(started);
        if now >= deadline {
            return;
        }
        tokio::time::sleep(Duration::from_millis(deadline - now)).await;
    }
}
struct Intent {
    applied: Option<AppliedOrder>,
    preparing: bool,
    abort_pending: bool,
    route_deviation: bool,
    stopped_at_goal_before_tick: bool,
    previous_position: WorldPosition,
    action: i32,
    target: Option<crate::motion::MotionTarget>,
}

impl RobotRuntime {
    async fn initialize(
        config: OperationalConfig,
        recovered: Option<&crate::checkpoint::RecoveryCheckpoint>,
    ) -> Result<(Self, SimulationEngine<ManualMonotonicClock>), Box<dyn Error>> {
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
            config.rest_base_url.clone(),
            config.websocket_url.clone(),
            config.api_key.clone(),
            parse_bool_or("MAPF_SIMULATOR_LOCAL_COMPOSE", false)?,
        )?)?;
        let mut runtime = Phase2Runtime::new(client, session);

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
                motion_limits: recovered
                    .and_then(|checkpoint| checkpoint.motion_profiles.get(&config.robot_id))
                    .map(|profile| profile.motion_limits())
                    .transpose()?
                    .unwrap_or(MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0)?),
                safety: SafetyConfig::new(0.2, 0.05)?,
                sensor: SensorConfig::new(0.0, 0)?,
            },
            config.master_seed,
            &[],
        )?;

        let robot_id = RobotId::new(config.robot_id.clone())?;
        let mut fleet = Fleet::new([engine], FleetConfig::new(0.5, 50)?)?;
        let store = CheckpointStore::new(&config.checkpoint_path)?;
        let mut station = crate::station::StationState {
            battery_percent: config.initial_battery_percent,
            ..Default::default()
        };
        let mut plans = PlanCoordinator::new();
        let mut battery_depletion_id = None;
        if let Some(checkpoint) =
            store.load_for_restart(&config.simulator_id, &config.map.content_digest_sha256)?
        {
            if let Some(saved) = checkpoint.station_states.get(&config.robot_id) {
                station = saved.clone();
            }
            plans = fleet.restore_checkpoint(&checkpoint)?;
            battery_depletion_id = checkpoint
                .battery_depletion_ids
                .get(&config.robot_id)
                .copied();
        }
        let mut engine = fleet.robots()[&robot_id].clone();
        if let Some(checkpoint) = recovered
            && let Some(saved) = checkpoint
                .robots
                .iter()
                .find(|saved| saved.robot_id == config.robot_id)
        {
            engine.restore_safety_state(
                RobotState::new(
                    WorldPosition::new(saved.x_meters, saved.y_meters)?,
                    Velocity::new(saved.velocity_x_mps, saved.velocity_y_mps)?,
                    Acceleration::new(saved.acceleration_x_mps2, saved.acceleration_y_mps2)?,
                    saved.yaw_radians,
                )?,
                crate::types::ControlTick::new(saved.tick),
                crate::types::SimulationTimeMs::new(saved.simulation_time_ms)?,
                saved.emergency_stop_latched,
            )?;
            plans = checkpoint
                .independent_plans
                .get(&config.robot_id)
                .cloned()
                .ok_or_else(|| invalid("fleet checkpoint omits independent plan"))?;
            battery_depletion_id = checkpoint
                .battery_depletion_ids
                .get(&config.robot_id)
                .copied();
            if let Some(saved_station) = checkpoint.station_states.get(&config.robot_id) {
                station = saved_station.clone();
            }
        }
        let restart_plan_revision_id = plans.active_revision_id().or(runtime
            .applied_order()
            .await?
            .and_then(|order| order.plan_revision_id));
        Ok((
            Self {
                config,
                runtime,
                plans,
                station,
                restart_plan_revision_id,
                completion_queued: false,
                recovery_reported: false,
                station_failure_reported: false,
                connected: true,
                battery_depletion_id,
                battery_depletion_reported: false,
                reconnect: None,
                retry_attempt: 0,
                next_retry_ms: 0,
            },
            engine,
        ))
    }
    async fn intent(&mut self, fleet: &Fleet) -> Result<Intent, Box<dyn Error>> {
        let config = &self.config;
        let runtime = &mut self.runtime;
        let robot_id = RobotId::new(config.robot_id.clone())?;
        let applied = runtime.applied_order().await?;
        let preparing = runtime.prepared_order().await?.is_some();
        let engine = &fleet.robots()[&robot_id];
        let stopped_at_goal_before_tick = applied
            .as_ref()
            .and_then(|order| order.goal)
            .is_some_and(|goal| goal_reached_and_stopped(engine, goal));
        let mut route_deviation = false;
        let abort_pending = runtime.abort_pending().await?;
        let target = if !self.connected
            || self.station.battery_percent <= 0.0
            || preparing
            || abort_pending
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

        Ok(Intent {
            applied,
            preparing,
            abort_pending,
            route_deviation,
            stopped_at_goal_before_tick,
            previous_position: engine.state().position(),
            action,
            target,
        })
    }
    async fn advance_physics_state(
        &mut self,
        fleet: &mut Fleet,
        step: &FleetStep,
        intent: &Intent,
    ) -> Result<(), Box<dyn Error>> {
        let id = RobotId::new(self.config.robot_id.clone())?;
        let engine = &fleet.robots()[&id];
        let record = &step.records[&id];
        let safe = !intent.abort_pending
            && self.runtime.synchronized().await?
            && self.plans.motion_authorized(&self.config.robot_id)
            && !engine.emergency_stop_latched()
            && !intent.route_deviation
            && !recovery_affects(step, &self.config.robot_id)
            && record.safety_outcome == crate::safety::SafetyOutcome::Accept;
        let arrived = intent
            .applied
            .as_ref()
            .and_then(|order| order.goal)
            .is_some_and(|goal| goal_reached_and_stopped(engine, goal));
        let charging = safe
            && arrived
            && intent.stopped_at_goal_before_tick
            && !intent.preparing
            && intent.applied.as_ref().is_some_and(|order| {
                order.arrival_action == Some(crate::contracts::generated::StationAction::Charge)
            });
        self.station.consume_battery(
            distance_meters(intent.previous_position, record.state.position()),
            100,
            charging,
            self.config.battery_config,
        );
        if self.station.battery_percent <= 0.0 {
            self.battery_depletion_id
                .get_or_insert_with(|| battery_event_id(&self.config.robot_id, record.tick.get()));
            fleet.latch_robot_emergency_stop(&id)?;
        } else if let Some(order) = &intent.applied
            && let Some(action) = order.arrival_action
        {
            self.station.advance(
                &order.order_id,
                action,
                safe && arrived && intent.stopped_at_goal_before_tick && !intent.preparing,
                100,
                self.config.station_config,
            );
        }
        Ok(())
    }

    async fn finish_tick(
        &mut self,
        fleet: &Fleet,
        step: &FleetStep,
        intent: Intent,
        monotonic_ms: u64,
    ) -> Result<(), Box<dyn Error>> {
        let Intent {
            applied,
            abort_pending,
            route_deviation,
            ..
        } = intent;
        let config = &self.config;
        let runtime = &mut self.runtime;
        let plans = &mut self.plans;
        let station = &mut self.station;
        let robot_id = RobotId::new(config.robot_id.clone())?;
        let mut completion_queued = self.completion_queued;
        let mut recovery_reported = self.recovery_reported;
        let mut station_failure_reported = self.station_failure_reported;
        let mut restart_plan_revision_id = self.restart_plan_revision_id;
        let record = step
            .records
            .get(&robot_id)
            .ok_or_else(|| invalid("fleet step omitted the configured robot"))?;
        if let Some(plan_revision_id) = applied.as_ref().and_then(|order| order.plan_revision_id)
            && plans.motion_authorized(&config.robot_id)
        {
            plans.confirm_runtime_safe(plan_revision_id, &config.robot_id)?;
        }
        let engine = &fleet.robots()[&robot_id];
        if route_deviation && !recovery_reported {
            plans.hold_for_recovery(None)?;
        }
        if !recovery_reported && (route_deviation || recovery_affects(step, &config.robot_id)) {
            let (code, held_robots) = if route_deviation {
                ("SAFETY_ROUTE_DEVIATION", vec![config.robot_id.clone()])
            } else {
                let recovery = step.recovery.as_ref().expect("checked recovery");
                let code = match recovery.reason {
                    RecoveryReason::Collision => "COLLISION_RISK",
                    RecoveryReason::CorridorConflict => "CORRIDOR_CONFLICT",
                    RecoveryReason::Deadlock => "DEADLOCK_DETECTED",
                    RecoveryReason::RobotFailure => "ROBOT_FAILURE",
                    RecoveryReason::StationaryBlocked => "SAFETY_STATIONARY_BLOCKED",
                };
                (code, recovery.held_robots.clone())
            };
            let mut evidence = serde_json::Map::new();
            evidence.insert("heldRobots".to_owned(), serde_json::json!(held_robots));
            if code == "SAFETY_STATIONARY_BLOCKED" {
                evidence.insert(
                    "blockingRobotIds".to_owned(),
                    serde_json::json!(
                        step.traffic_wait
                            .get(&config.robot_id)
                            .cloned()
                            .unwrap_or_default()
                    ),
                );
            }
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
            && !recovery_affects(step, &self.config.robot_id)
            && record.safety_outcome == crate::safety::SafetyOutcome::Accept;
        runtime
            .set_station_state(
                station.clone(),
                station_safe
                    || applied.is_none()
                        && !abort_pending
                        && !engine.emergency_stop_latched()
                        && !route_deviation
                        && !recovery_affects(step, &config.robot_id)
                        && record.safety_outcome == crate::safety::SafetyOutcome::Accept,
            )
            .await?;

        if let Some(event_id) = self.battery_depletion_id
            && !self.battery_depletion_reported
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
            self.battery_depletion_reported = true;
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
            .set_traffic_wait(step.traffic_wait.get(&config.robot_id).map(|blockers| {
                crate::contracts::provisioning_generated::TrafficWait {
                    contract_version: "1.0.0".into(),
                    reason: "PASSAGE_RIGHT_UNAVAILABLE".into(),
                    blocking_robot_ids: blockers.iter().cloned().collect(),
                }
            }))
            .await?;
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
        self.completion_queued = completion_queued;
        self.recovery_reported = recovery_reported;
        self.station_failure_reported = station_failure_reported;
        self.restart_plan_revision_id = restart_plan_revision_id;
        Ok(())
    }
    async fn event(
        &mut self,
        fleet: &Fleet,
        event: RuntimeEvent,
        monotonic_ms: u64,
    ) -> Result<(), Box<dyn Error>> {
        let config = &self.config;
        let runtime = &mut self.runtime;
        let plans = &mut self.plans;
        let robot_id = RobotId::new(config.robot_id.clone())?;
        let mut completion_queued = self.completion_queued;
        let mut recovery_reported = self.recovery_reported;
        let mut station_failure_reported = self.station_failure_reported;
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
                            planning_snapshot_digest_sha256: applied.content_digest_sha256.clone(),
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
            }
            RuntimeEvent::ReportAcknowledged(_) => {
                if completion_queued && runtime.applied_order().await?.is_none() {
                    *plans = PlanCoordinator::new();

                    if config.exit_after_completion {
                        return Ok(());
                    }
                    station_failure_reported = false;
                    completion_queued = false;
                }
            }
        }
        self.completion_queued = completion_queued;
        self.recovery_reported = recovery_reported;
        self.station_failure_reported = station_failure_reported;
        Ok(())
    }
    async fn disconnect(&mut self, monotonic_ms: u64) -> Result<(), Box<dyn Error>> {
        self.connected = false;
        self.next_retry_ms =
            monotonic_ms.saturating_add(retry_delay_ms(&self.config.robot_id, self.retry_attempt));
        self.runtime.mark_disconnected(monotonic_ms).await?;
        if self.plans.active_revision_id().is_some() {
            self.plans.hold_for_recovery(None)?;
            self.restart_plan_revision_id = self.plans.active_revision_id();
        }
        Ok(())
    }

    async fn reconnect_if_due(&mut self, monotonic_ms: u64) -> Result<(), Box<dyn Error>> {
        if self
            .reconnect
            .as_ref()
            .is_some_and(|task| task.is_finished())
        {
            let result = self
                .reconnect
                .take()
                .expect("checked reconnect task")
                .await?;
            match result {
                Ok(runtime) => {
                    self.runtime = runtime;
                    self.connected = true;
                    self.retry_attempt = 0;
                }
                Err(error) => {
                    self.retry_attempt = self.retry_attempt.saturating_add(1);
                    self.next_retry_ms = monotonic_ms
                        .saturating_add(retry_delay_ms(&self.config.robot_id, self.retry_attempt));
                    eprintln!(
                        "fleet robot {} reconnect deferred: {error}",
                        self.config.robot_id
                    );
                }
            }
        }
        if !self.connected && self.reconnect.is_none() && monotonic_ms >= self.next_retry_ms {
            self.reconnect = Some(self.runtime.reconnect_task(monotonic_ms));
        }
        Ok(())
    }
}

struct ProfileSync {
    profiles_tx: tokio::sync::mpsc::UnboundedSender<Vec<MotionProfile>>,
    applied_rx: tokio::sync::mpsc::UnboundedReceiver<Vec<MotionProfileTarget>>,
}

struct FleetUpdates {
    claims_rx: tokio::sync::mpsc::UnboundedReceiver<Vec<RuntimeRobotClaim>>,
    removals_rx: tokio::sync::mpsc::UnboundedReceiver<Vec<RobotRemovalOutcome>>,
    profiles_rx: tokio::sync::mpsc::UnboundedReceiver<Vec<MotionProfile>>,
    applied_tx: tokio::sync::mpsc::UnboundedSender<Vec<MotionProfileTarget>>,
}

#[derive(Clone)]
struct Provisioner {
    http: reqwest::Client,
    base: url::Url,
    key: http::HeaderValue,
    boot_id: Uuid,
}
impl Provisioner {
    async fn renew_lease(
        &self,
        mut known: BTreeSet<String>,
        claims: tokio::sync::mpsc::UnboundedSender<Vec<RuntimeRobotClaim>>,
        started: Instant,
        renewed_ms: &AtomicU64,
    ) -> Result<(), Box<dyn Error>> {
        let mut interval = tokio::time::interval_at(
            tokio::time::Instant::now() + Duration::from_secs(3),
            Duration::from_secs(3),
        );
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            interval.tick().await;
            let request_started = elapsed_milliseconds(started);
            match self.claim(known.iter().cloned().collect()).await {
                Ok(robots) => {
                    renewed_ms.store(request_started, Ordering::Relaxed);
                    known.extend(robots.iter().map(|robot| robot.robot_id.clone()));
                    if !robots.is_empty() {
                        claims
                            .send(robots)
                            .map_err(|_| invalid("fleet claim receiver closed"))?;
                    }
                }
                Err(error) => eprintln!("fleet lease renewal failed: {error}"),
            }
        }
    }

    async fn sync_configuration(
        &self,
        removals: tokio::sync::mpsc::UnboundedSender<Vec<RobotRemovalOutcome>>,
        mut sync: ProfileSync,
    ) -> Result<(), Box<dyn Error>> {
        loop {
            tokio::time::sleep(Duration::from_secs(3)).await;
            while let Ok(targets) = sync.applied_rx.try_recv() {
                if let Err(error) = self.acknowledge_profiles(targets).await {
                    eprintln!("motion profile acknowledgement failed: {error}");
                }
            }
            match self.motion_profiles().await {
                Ok(profiles) => sync
                    .profiles_tx
                    .send(profiles)
                    .map_err(|_| invalid("motion profile receiver closed"))?,
                Err(error) => eprintln!("motion profile sync failed: {error}"),
            }
            match self.removals().await {
                Ok(value) => removals
                    .send(value)
                    .map_err(|_| invalid("fleet removal receiver closed"))?,
                Err(error) => eprintln!("fleet removal sync failed: {error}"),
            }
        }
    }

    async fn motion_profiles(&self) -> Result<Vec<MotionProfile>, Box<dyn Error>> {
        let response = self
            .http
            .get(self.base.join("api/v1/runtime/motion-profiles")?)
            .query(&[("bootId", self.boot_id.to_string())])
            .header("X-Runtime-Key", self.key.clone())
            .send()
            .await?;
        if response.status() == reqwest::StatusCode::NOT_FOUND {
            return Ok(Vec::new()); // Backward-compatible older Core.
        }
        let outcome = response
            .error_for_status()?
            .json::<MotionProfilesOutcome>()
            .await?;
        if outcome.contract_version != "1.0.0"
            || outcome.profiles.iter().any(|profile| {
                profile.applied_version > profile.version || profile.limits.motion_limits().is_err()
            })
        {
            return Err(invalid("unsupported or invalid motion profile").into());
        }
        Ok(outcome.profiles)
    }

    async fn acknowledge_profiles(
        &self,
        targets: Vec<MotionProfileTarget>,
    ) -> Result<(), Box<dyn Error>> {
        self.http
            .post(self.base.join("api/v1/runtime/motion-profiles/ack")?)
            .header("X-Runtime-Key", self.key.clone())
            .json(&AcknowledgeMotionProfilesRequest {
                contract_version: "1.0.0".into(),
                request_id: Uuid::new_v4().to_string(),
                boot_id: self.boot_id.to_string(),
                targets,
            })
            .send()
            .await?
            .error_for_status()?;
        Ok(())
    }

    fn new(config: &OperationalConfig) -> Result<Self, Box<dyn Error>> {
        if config.profile != crate::controller::RuntimeProfile::Local {
            return Err(invalid("dynamic fleet requires the local profile").into());
        }
        let key_path = std::env::var("MAPF_SIMULATOR_RUNTIME_KEY_PATH")?;
        let mut key = http::HeaderValue::from_str(std::fs::read_to_string(key_path)?.trim())?;
        key.set_sensitive(true);
        Ok(Self {
            http: reqwest::Client::builder()
                .timeout(Duration::from_secs(3))
                .redirect(reqwest::redirect::Policy::none())
                .build()?,
            base: config.rest_base_url.clone(),
            key,
            boot_id: Uuid::new_v4(),
        })
    }
    async fn claim(&self, running: Vec<String>) -> Result<Vec<RuntimeRobotClaim>, Box<dyn Error>> {
        let response = self
            .http
            .post(self.base.join("api/v1/runtime/provisioning/claim")?)
            .header("X-Runtime-Key", self.key.clone())
            .json(&RuntimeClaimRequest {
                contract_version: "1.0.0".to_owned(),
                boot_id: self.boot_id.to_string(),
                running_robot_ids: running,
            })
            .send()
            .await?;
        if !response.status().is_success() {
            return Err(invalid("runtime claim rejected").into());
        }
        let outcome = response.json::<RuntimeClaimOutcome>().await?;
        if outcome.contract_version != "1.0.0" {
            return Err(invalid("unsupported provisioning contract").into());
        }
        Ok(outcome.robots)
    }
    async fn fail(&self, claim: &RuntimeRobotClaim, code: &str) -> Result<(), Box<dyn Error>> {
        let response = self
            .http
            .post(self.base.join(&format!(
                "api/v1/runtime/robots/{}/provisioning/failure",
                claim.robot_id
            ))?)
            .header("X-Runtime-Key", self.key.clone())
            .json(&RuntimeResultRequest {
                contract_version: "1.0.0".to_owned(),
                boot_id: self.boot_id.to_string(),
                lease_epoch: claim.lease_epoch,
                failure_code: code.to_owned(),
            })
            .send()
            .await?;
        if !response.status().is_success() {
            return Err(invalid("runtime failure result rejected").into());
        }
        Ok(())
    }

    async fn removals(&self) -> Result<Vec<RobotRemovalOutcome>, Box<dyn Error>> {
        let response = self
            .http
            .post(self.base.join("api/v1/runtime/robot-removals/sync")?)
            .header("X-Runtime-Key", self.key.clone())
            .json(&RuntimeRemovalRequest {
                contract_version: "1.0.0".into(),
                boot_id: self.boot_id.to_string(),
            })
            .send()
            .await?;
        if !response.status().is_success() {
            return Err(invalid("robot removal sync rejected").into());
        }
        let result = response.json::<RuntimeRemovalOutcome>().await?;
        if result.contract_version != "1.0.0"
            || result.removals.iter().any(|row| {
                row.contract_version != "1.0.0"
                    || !matches!(row.state.as_str(), "PENDING" | "REMOVED")
            })
        {
            return Err(invalid("invalid robot removal contract").into());
        }
        Ok(result.removals)
    }

    async fn complete_removal(&self, id: &str) -> Result<(), Box<dyn Error>> {
        let response = self
            .http
            .post(
                self.base
                    .join(&format!("api/v1/runtime/robots/{id}/removal/complete"))?,
            )
            .header("X-Runtime-Key", self.key.clone())
            .json(&RuntimeRemovalRequest {
                contract_version: "1.0.0".into(),
                boot_id: self.boot_id.to_string(),
            })
            .send()
            .await?;
        if !response.status().is_success() {
            return Err(invalid("robot removal completion rejected").into());
        }
        let outcome = response.json::<RobotRemovalOutcome>().await?;
        if outcome.contract_version != "1.0.0"
            || outcome.robot_id != id
            || outcome.state != "REMOVED"
        {
            return Err(invalid("invalid robot removal completion").into());
        }
        Ok(())
    }
}

fn claimed_config(
    base: &OperationalConfig,
    claim: &RuntimeRobotClaim,
) -> Result<OperationalConfig, Box<dyn Error>> {
    RobotId::new(claim.robot_id.clone())?;
    if claim.robot_id.contains('/') || claim.robot_id.contains('\\') {
        return Err(invalid("invalid provisioned identity").into());
    }
    let mut config = base.clone();
    config.robot_id = claim.robot_id.clone();
    config.simulator_id = claim.simulator_id.clone();
    config.api_key = ApiKey::new(&claim.api_key)?;
    config.map = crate::protocol::MapIdentity {
        map_id: Uuid::parse_str(&claim.map.map_id)?,
        revision: claim.map.revision,
        content_digest_sha256: claim.map.content_digest_sha256.clone(),
    };
    config.map.validate()?;
    if config.map != base.map {
        return Err(invalid("provisioned map differs from fleet map").into());
    }
    config.start = GridCell::new(
        u32::try_from(claim.start.column)?,
        u32::try_from(claim.start.row)?,
    );
    let directory = base
        .spool_path
        .parent()
        .ok_or_else(|| invalid("spool directory is missing"))?
        .join(&claim.robot_id);
    config.spool_path = directory.join("spool.json");
    config.checkpoint_path = directory.join("checkpoint.json");
    Ok(config)
}

async fn add_claims(
    base: &OperationalConfig,
    provisioner: &Provisioner,
    claims: Vec<RuntimeRobotClaim>,
    fleet: &mut Fleet,
    robots: &mut BTreeMap<RobotId, RobotRuntime>,
    recovered: Option<&crate::checkpoint::RecoveryCheckpoint>,
) -> Result<(), Box<dyn Error>> {
    for claim in claims {
        if robots.keys().any(|id| id.as_str() == claim.robot_id) {
            return Err(invalid("Core reclaimed a running robot").into());
        }
        let config = claimed_config(base, &claim)?;
        let initialized = RobotRuntime::initialize(config, recovered).await;
        let (context, mut engine) = match initialized {
            Ok(value) => value,
            Err(_) => {
                provisioner.fail(&claim, "START_FAILED").await?;
                continue;
            }
        };
        if engine.tick().get() == 0 {
            // New robot joins the shared fleet clock at this tick boundary.
            if let Some(reference) = fleet.robots().values().next() {
                engine.restore_safety_state(
                    engine.state(),
                    reference.tick(),
                    reference.simulation_time(),
                    false,
                )?;
            }
        }
        let id = RobotId::new(claim.robot_id.clone())?;
        if fleet.insert(engine).is_err() {
            provisioner.fail(&claim, "UNSAFE_START").await?;
            continue;
        }
        robots.insert(id, context);
    }
    Ok(())
}

async fn persist(
    fleet: &Fleet,
    robots: &BTreeMap<RobotId, RobotRuntime>,
    store: &CheckpointStore,
    base: &OperationalConfig,
) -> Result<(), Box<dyn Error>> {
    let mut checkpoint = fleet.checkpoint(
        "local-fleet".to_owned(),
        base.map.content_digest_sha256.clone(),
        PlanCoordinator::new(),
    );
    for (id, robot) in robots {
        checkpoint
            .independent_plans
            .insert(id.as_str().to_owned(), robot.plans.clone());
        if let Some(event_id) = robot.battery_depletion_id {
            checkpoint
                .battery_depletion_ids
                .insert(id.as_str().to_owned(), event_id);
        }
        checkpoint
            .station_states
            .insert(id.as_str().to_owned(), robot.station.clone());
    }
    // One atomic document contains fleet membership, physics and every independent plan.
    save_checkpoint(store, checkpoint).await?;
    Ok(())
}

pub async fn run() -> Result<(), Box<dyn Error>> {
    let base = OperationalConfig::from_environment()?;
    base.validate()?;
    let provisioner = Provisioner::new(&base)?;
    let store = CheckpointStore::new(base.checkpoint_path.with_file_name("fleet-checkpoint.json"))?;
    let mut recovered = store.load_for_restart("local-fleet", &base.map.content_digest_sha256)?;
    let started = Instant::now();
    // Acquire the process lease before fencing any robot session.
    let (initial_claims, acquired_ms) = loop {
        let request_started = elapsed_milliseconds(started);
        match provisioner.claim(Vec::new()).await {
            Ok(claims) => break (claims, request_started),
            Err(_) => tokio::time::sleep(Duration::from_secs(1)).await,
        }
    };
    // Initializing/restoring robots can also block. Never reset the lease clock
    // after initialization or wait for the next failed claim to enforce expiry.
    let renewed_ms = AtomicU64::new(acquired_ms);
    let initial_removals = provisioner.removals().await?;
    if let Some(checkpoint) = &mut recovered {
        let removed: BTreeSet<_> = initial_removals
            .iter()
            .map(|row| row.robot_id.as_str())
            .collect();
        checkpoint
            .robots
            .retain(|robot| !removed.contains(robot.robot_id.as_str()));
        checkpoint
            .independent_plans
            .retain(|id, _| !removed.contains(id.as_str()));
        checkpoint
            .station_states
            .retain(|id, _| !removed.contains(id.as_str()));
        checkpoint
            .battery_depletion_ids
            .retain(|id, _| !removed.contains(id.as_str()));
        checkpoint
            .motion_profiles
            .retain(|id, _| !removed.contains(id.as_str()));
    }
    let known = initial_claims
        .iter()
        .map(|claim| claim.robot_id.clone())
        .collect();
    let (claims_tx, claims_rx) = tokio::sync::mpsc::unbounded_channel();
    let (removals_tx, removals_rx) = tokio::sync::mpsc::unbounded_channel();
    let (profiles_tx, profiles_rx) = tokio::sync::mpsc::unbounded_channel();
    let (applied_tx, applied_rx) = tokio::sync::mpsc::unbounded_channel();
    let heartbeat = provisioner.clone();
    tokio::select! {
        biased;
        _ = lease_expired(started, &renewed_ms) => Err(invalid("local fleet runtime lease was lost").into()),
        result = heartbeat.renew_lease(known, claims_tx, started, &renewed_ms) => result,
        result = heartbeat.sync_configuration(removals_tx, ProfileSync { profiles_tx, applied_rx }) => result,
        result = run_fleet(
            base,
            provisioner,
            store,
            FleetStartup { recovered, initial_claims, initial_removals },
            started,
            FleetUpdates { claims_rx, removals_rx, profiles_rx, applied_tx },
        ) => result,
    }
}

async fn run_fleet(
    base: OperationalConfig,
    provisioner: Provisioner,
    store: CheckpointStore,
    startup: FleetStartup,
    started: Instant,
    mut updates: FleetUpdates,
) -> Result<(), Box<dyn Error>> {
    let FleetStartup {
        recovered,
        initial_claims,
        initial_removals,
    } = startup;
    let (robot, engine) = RobotRuntime::initialize(base.clone(), recovered.as_ref()).await?;
    let mut fleet = Fleet::new([engine], FleetConfig::new(0.5, 50)?)?;
    let mut robots = BTreeMap::from([(RobotId::new(base.robot_id.clone())?, robot)]);
    add_claims(
        &base,
        &provisioner,
        initial_claims
            .into_iter()
            .filter(|claim| {
                !initial_removals
                    .iter()
                    .any(|removal| removal.robot_id == claim.robot_id)
            })
            .collect(),
        &mut fleet,
        &mut robots,
        recovered.as_ref(),
    )
    .await?;
    if let Some(checkpoint) = &recovered
        && checkpoint
            .robots
            .iter()
            .any(|saved| !robots.keys().any(|id| id.as_str() == saved.robot_id))
    {
        return Err(invalid("cannot resume with a missing checkpoint robot").into());
    }
    persist(&fleet, &robots, &store, &base).await?;
    let mut removals = initial_removals;
    let mut profiles = Vec::new();
    let mut interval = tokio::time::interval(Duration::from_millis(100));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        interval.tick().await;
        let monotonic_ms = elapsed_milliseconds(started);
        while let Ok(value) = updates.removals_rx.try_recv() {
            removals = value;
        }
        while let Ok(value) = updates.profiles_rx.try_recv() {
            profiles = value;
        }
        let mut applied_profiles = Vec::new();
        for profile in &profiles {
            let id = RobotId::new(profile.robot_id.clone())?;
            let Some(robot) = robots.get(&id) else {
                continue;
            };
            let Some(engine) = fleet.robots().get(&id) else {
                continue;
            };
            let limits = profile.limits.motion_limits()?;
            if engine.motion_limits() == limits && profile.applied_version == profile.version {
                continue;
            }
            // Do not alter a prepared or active route; wait for safe idle state.
            if !robot.connected
                || robot.runtime.applied_order().await?.is_some()
                || robot.runtime.prepared_order().await?.is_some()
                || robot.runtime.abort_pending().await?
                || !robot.runtime.synchronized().await?
            {
                continue;
            }
            if fleet.apply_motion_profile(&id, &profile.limits)? {
                applied_profiles.push(MotionProfileTarget {
                    robot_id: profile.robot_id.clone(),
                    version: profile.version,
                });
            }
        }
        if !applied_profiles.is_empty() {
            // Persist the actual limits before letting Core admit a new route.
            persist(&fleet, &robots, &store, &base).await?;
            updates
                .applied_tx
                .send(applied_profiles)
                .map_err(|_| invalid("motion profile ack receiver closed"))?;
            profiles.clear(); // The heartbeat retries if an ACK is lost.
        }
        for removal in &mut removals {
            let id = RobotId::new(removal.robot_id.clone())?;
            if id.as_str() == base.robot_id {
                return Err(invalid("base robot cannot be removed").into());
            }
            if fleet.robots().contains_key(&id) {
                fleet.latch_robot_emergency_stop(&id)?;
                if !fleet.remove_stationary(&id)? {
                    continue;
                }
                robots.remove(&id);
                persist(&fleet, &robots, &store, &base).await?;
            }
            if removal.state == "PENDING" {
                // Checkpoint exclusion must be durable before Core retires the robot.
                persist(&fleet, &robots, &store, &base).await?;
                match provisioner.complete_removal(id.as_str()).await {
                    Ok(()) => removal.state = "REMOVED".into(),
                    Err(error) => eprintln!("fleet removal completion failed: {error}"),
                }
            }
        }
        if let Ok(claims) = updates.claims_rx.try_recv() {
            let claims = claims
                .into_iter()
                .filter(|claim| {
                    !removals
                        .iter()
                        .any(|removal| removal.robot_id == claim.robot_id)
                })
                .collect();
            add_claims(&base, &provisioner, claims, &mut fleet, &mut robots, None).await?;
            persist(&fleet, &robots, &store, &base).await?;
        }
        let mut intents = BTreeMap::new();
        for (id, robot) in &mut robots {
            if !robot.connected {
                robot.reconnect_if_due(monotonic_ms).await?;
            } else {
                for _ in 0..16 {
                    let engine = &fleet.robots()[id];
                    let stationary = engine.state().velocity().magnitude() < 1e-6
                        && engine.state().acceleration().magnitude() < 1e-6;
                    match robot
                        .runtime
                        .poll_one_with_motion(monotonic_ms, utc_now_milliseconds()?, stationary)
                        .await
                    {
                        Ok(Some(event)) => {
                            if robot.event(&fleet, event, monotonic_ms).await.is_err() {
                                robot.disconnect(monotonic_ms).await?;
                                break;
                            }
                        }
                        Ok(None) => break,
                        Err(error) => {
                            eprintln!("fleet robot {} transport hold: {error}", id.as_str());
                            robot.disconnect(monotonic_ms).await?;
                            break;
                        }
                    }
                }
            }
            if robot.station.battery_percent <= 0.0 {
                fleet.latch_robot_emergency_stop(id)?;
            }
            intents.insert(id.clone(), robot.intent(&fleet).await?);
        }
        let actions = intents
            .iter()
            .map(|(id, intent)| (id.clone(), intent.action))
            .collect();
        let targets = intents
            .iter()
            .filter_map(|(id, intent)| intent.target.map(|target| (id.clone(), target)))
            .collect();
        let mut plans = robots
            .iter()
            .map(|(id, robot)| (id.clone(), robot.plans.clone()))
            .collect();
        let step = fleet.step_with_passage_rights(&actions, &targets, &mut plans)?;
        for (id, robot) in &mut robots {
            robot.plans = plans
                .remove(id)
                .ok_or_else(|| invalid("fleet omitted plan state"))?;
            robot
                .advance_physics_state(&mut fleet, &step, &intents[id])
                .await?;
        }
        // Commit all battery/station/physical progress before any robot reports it.
        persist(&fleet, &robots, &store, &base).await?;
        for (id, robot) in &mut robots {
            if !robot.connected {
                continue;
            }
            let intent = intents
                .remove(id)
                .ok_or_else(|| invalid("fleet omitted robot intent"))?;
            if let Err(error) = robot.finish_tick(&fleet, &step, intent, monotonic_ms).await {
                eprintln!("fleet robot {} report hold: {error}", id.as_str());
                robot.disconnect(monotonic_ms).await?;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovery_only_pauses_station_and_incident_processing_for_held_robots() {
        let mut step = FleetStep {
            traffic_wait: BTreeMap::new(),
            records: BTreeMap::new(),
            stationary_reservations: BTreeSet::new(),
            recovery: Some(crate::fleet::RecoveryEvent {
                reason: RecoveryReason::Collision,
                held_robots: vec!["r1".into()],
                replan_required: true,
            }),
        };
        assert!(recovery_affects(&step, "r1"));
        assert!(!recovery_affects(&step, "r2"));
        step.recovery
            .as_mut()
            .unwrap()
            .held_robots
            .push("r2".into());
        assert!(recovery_affects(&step, "r2"));
        step.recovery = None;
        assert!(!recovery_affects(&step, "r1"));
    }

    #[test]
    fn reconnect_delay_grows_caps_and_staggers_robots() {
        let delays: Vec<_> = (0..8)
            .map(|attempt| retry_delay_ms("robot-1", attempt))
            .collect();
        assert!(delays.windows(2).all(|p| p[0] <= p[1]));
        assert!((1000..2000).contains(&delays[0]));
        assert_eq!(delays[7], 30_000);
        assert_ne!(retry_delay_ms("robot-1", 0), retry_delay_ms("robot-2", 0));
    }

    #[tokio::test]
    async fn expired_lease_cancels_stalled_fleet_without_waiting_for_claim_failure() {
        let started = Instant::now() - Duration::from_secs(10);
        let renewed_ms = AtomicU64::new(0);
        let expired = tokio::select! {
            _ = lease_expired(started, &renewed_ms) => true,
            _ = std::future::pending::<()>() => false,
        };
        assert!(expired);
    }
}
