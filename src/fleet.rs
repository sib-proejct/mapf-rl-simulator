//! Atomic, deterministic multi-robot tick scheduling and local recovery.

use crate::checkpoint::{RecoveryCheckpoint, RobotSafetyCheckpoint};
use crate::plan::{PlanCoordinator, PlanError};
use crate::simulation::{EngineError, MonotonicClock, SimulationEngine, StepRecord};
use crate::types::{
    RobotId, RobotState, SAFETY_EPSILON_METERS, ValidationError, WorldPosition, ensure_positive,
};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RecoveryReason {
    Collision,
    CorridorConflict,
    Deadlock,
    RobotFailure,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RecoveryEvent {
    pub reason: RecoveryReason,
    pub held_robots: Vec<String>,
    pub replan_required: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FleetStep {
    pub records: BTreeMap<RobotId, StepRecord>,
    pub stationary_reservations: BTreeSet<String>,
    pub recovery: Option<RecoveryEvent>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FleetConfig {
    minimum_center_separation_meters: f64,
    deadlock_ticks: u32,
}

impl FleetConfig {
    pub fn new(
        minimum_center_separation_meters: f64,
        deadlock_ticks: u32,
    ) -> Result<Self, ValidationError> {
        ensure_positive(
            minimum_center_separation_meters,
            "fleet.minimum_center_separation",
        )?;
        if deadlock_ticks == 0 {
            return Err(ValidationError::OutOfRange("fleet.deadlock_ticks"));
        }
        Ok(Self {
            minimum_center_separation_meters,
            deadlock_ticks,
        })
    }

    pub const fn minimum_center_separation_meters(self) -> f64 {
        self.minimum_center_separation_meters
    }

    pub const fn deadlock_ticks(self) -> u32 {
        self.deadlock_ticks
    }
}

/// The BTreeMap order is the single deterministic fleet commit order.
pub struct MultiRobotEngine<C> {
    robots: BTreeMap<RobotId, SimulationEngine<C>>,
    config: FleetConfig,
    no_progress_ticks: u32,
}

impl<C: MonotonicClock + Clone> MultiRobotEngine<C> {
    pub fn new(
        robots: impl IntoIterator<Item = SimulationEngine<C>>,
        config: FleetConfig,
    ) -> Result<Self, FleetError> {
        let mut ordered = BTreeMap::new();
        for engine in robots {
            let id = engine.robot_id().clone();
            if ordered.insert(id, engine).is_some() {
                return Err(FleetError::DuplicateRobot);
            }
        }
        if ordered.is_empty() {
            return Err(FleetError::EmptyFleet);
        }
        validate_pairwise_states(
            ordered.iter().map(|(id, engine)| (id, engine.state())),
            config.minimum_center_separation_meters,
        )?;
        Ok(Self {
            robots: ordered,
            config,
            no_progress_ticks: 0,
        })
    }

    pub fn robots(&self) -> &BTreeMap<RobotId, SimulationEngine<C>> {
        &self.robots
    }

    pub const fn no_progress_ticks(&self) -> u32 {
        self.no_progress_ticks
    }

    pub fn checkpoint(
        &self,
        simulator_id: String,
        map_content_digest_sha256: String,
        plans: PlanCoordinator,
    ) -> RecoveryCheckpoint {
        RecoveryCheckpoint {
            simulator_id,
            map_content_digest_sha256,
            robots: self
                .robots
                .iter()
                .map(|(id, engine)| RobotSafetyCheckpoint {
                    robot_id: id.as_str().to_owned(),
                    tick: engine.tick().get(),
                    simulation_time_ms: engine.simulation_time().get(),
                    x_meters: engine.state().position().x_meters(),
                    y_meters: engine.state().position().y_meters(),
                    velocity_x_mps: engine.state().velocity().x_mps(),
                    velocity_y_mps: engine.state().velocity().y_mps(),
                    acceleration_x_mps2: engine.state().acceleration().x_mps2(),
                    acceleration_y_mps2: engine.state().acceleration().y_mps2(),
                    yaw_radians: engine.state().yaw_radians(),
                    emergency_stop_latched: engine.emergency_stop_latched(),
                })
                .collect(),
            plans,
            no_progress_ticks: self.no_progress_ticks,
            station_states: BTreeMap::new(),
            requires_core_reconciliation: false,
        }
    }

    /// Restores observed safety facts while leaving plan motion fenced by the
    /// checkpoint's recovered coordinator.
    pub fn restore_checkpoint(
        &mut self,
        checkpoint: &RecoveryCheckpoint,
    ) -> Result<PlanCoordinator, FleetError> {
        checkpoint
            .validate()
            .map_err(|_| FleetError::InvalidCheckpoint)?;
        if !checkpoint.requires_core_reconciliation || checkpoint.robots.len() != self.robots.len()
        {
            return Err(FleetError::InvalidCheckpoint);
        }
        let mut restored = self
            .robots
            .iter()
            .map(|(id, engine)| (id.clone(), engine.clone()))
            .collect::<BTreeMap<_, _>>();
        for robot in &checkpoint.robots {
            let id =
                RobotId::new(robot.robot_id.clone()).map_err(|_| FleetError::InvalidCheckpoint)?;
            let engine = restored.get_mut(&id).ok_or(FleetError::InvalidCheckpoint)?;
            let state = RobotState::new(
                crate::types::WorldPosition::new(robot.x_meters, robot.y_meters)
                    .map_err(|_| FleetError::InvalidCheckpoint)?,
                crate::types::Velocity::new(robot.velocity_x_mps, robot.velocity_y_mps)
                    .map_err(|_| FleetError::InvalidCheckpoint)?,
                crate::types::Acceleration::new(
                    robot.acceleration_x_mps2,
                    robot.acceleration_y_mps2,
                )
                .map_err(|_| FleetError::InvalidCheckpoint)?,
                robot.yaw_radians,
            )
            .map_err(|_| FleetError::InvalidCheckpoint)?;
            engine.restore_safety_state(
                state,
                crate::types::ControlTick::new(robot.tick),
                crate::types::SimulationTimeMs::new(robot.simulation_time_ms)
                    .map_err(|_| FleetError::InvalidCheckpoint)?,
                robot.emergency_stop_latched,
            )?;
        }
        validate_pairwise_states(
            restored.iter().map(|(id, engine)| (id, engine.state())),
            self.config.minimum_center_separation_meters,
        )?;
        self.robots = restored;
        self.no_progress_ticks = checkpoint.no_progress_ticks;
        Ok(checkpoint.plans.clone())
    }

    /// Previews every robot first, resolves conflicts, and swaps in the complete
    /// next fleet only after every static and pairwise invariant succeeds.
    pub fn step(
        &mut self,
        requested_actions: &BTreeMap<RobotId, i32>,
        plans: &mut PlanCoordinator,
    ) -> Result<FleetStep, FleetError> {
        self.step_with_targets(requested_actions, &BTreeMap::new(), plans)
    }

    pub fn step_with_targets(
        &mut self,
        requested_actions: &BTreeMap<RobotId, i32>,
        targets: &BTreeMap<RobotId, WorldPosition>,
        plans: &mut PlanCoordinator,
    ) -> Result<FleetStep, FleetError> {
        let reservations = plans.stationary_reservations();
        let mut actions = BTreeMap::new();
        for id in self.robots.keys() {
            let action = requested_actions.get(id).copied().unwrap_or(0);
            actions.insert(
                id.clone(),
                if plans.motion_authorized(id.as_str()) {
                    action
                } else {
                    0
                },
            );
        }

        let initial = self.preview(&actions, targets, false)?;
        let conflicts = conflicting_pairs(
            &self.robots,
            &initial,
            self.config.minimum_center_separation_meters,
            true,
        );
        let mut recovery = None;
        if !conflicts.is_empty() {
            let mut held = BTreeSet::new();
            let corridor = conflicts.iter().any(|(left, right)| {
                corridor_conflict(
                    &self.robots[left],
                    &self.robots[right],
                    actions[left],
                    actions[right],
                )
            });
            for (left, right) in &conflicts {
                // An unreleased robot is an immutable stationary reservation.
                // Otherwise the lexicographically later robot yields.
                let yielding = if reservations.contains(left.as_str()) {
                    right
                } else if reservations.contains(right.as_str()) {
                    left
                } else {
                    right
                };
                actions.insert(yielding.clone(), 0);
                held.insert(yielding.as_str().to_owned());
            }
            recovery = Some(RecoveryEvent {
                reason: if corridor {
                    RecoveryReason::CorridorConflict
                } else {
                    RecoveryReason::Collision
                },
                held_robots: held.into_iter().collect(),
                replan_required: true,
            });
        }

        let mut next = self.preview(&actions, targets, false)?;
        let remaining = conflicting_pairs(
            &self.robots,
            &next,
            self.config.minimum_center_separation_meters,
            false,
        );
        if !remaining.is_empty() {
            let involved: BTreeSet<_> = remaining
                .iter()
                .flat_map(|(left, right)| [(*left).clone(), (*right).clone()])
                .collect();
            let emergency_actions = self
                .robots
                .keys()
                .map(|id| (id.clone(), 0))
                .collect::<BTreeMap<_, _>>();
            next =
                self.preview_selected_emergency(&emergency_actions, &BTreeMap::new(), &involved)?;
            if !conflicting_pairs(
                &self.robots,
                &next,
                self.config.minimum_center_separation_meters,
                false,
            )
            .is_empty()
            {
                return Err(FleetError::CollisionInvariant);
            }
            recovery = Some(RecoveryEvent {
                reason: RecoveryReason::Collision,
                held_robots: involved.iter().map(|id| id.as_str().to_owned()).collect(),
                replan_required: true,
            });
        }

        let any_motion_requested = actions
            .iter()
            .any(|(id, action)| *action != 0 && plans.motion_authorized(id.as_str()));
        let made_progress = next.iter().any(|(id, (_, record))| {
            let previous = self.robots[id].state().position();
            let current = record.state.position();
            (current.x_meters() - previous.x_meters())
                .hypot(current.y_meters() - previous.y_meters())
                > SAFETY_EPSILON_METERS
        });
        self.no_progress_ticks = if any_motion_requested && !made_progress {
            self.no_progress_ticks.saturating_add(1)
        } else {
            0
        };
        if self.no_progress_ticks >= self.config.deadlock_ticks {
            plans.hold_for_recovery(None)?;
            recovery = Some(RecoveryEvent {
                reason: RecoveryReason::Deadlock,
                held_robots: self
                    .robots
                    .keys()
                    .map(|id| id.as_str().to_owned())
                    .collect(),
                replan_required: true,
            });
        }
        if recovery
            .as_ref()
            .is_some_and(|event| event.replan_required && event.reason != RecoveryReason::Deadlock)
        {
            plans.hold_for_recovery(None)?;
        }

        let mut committed = BTreeMap::new();
        let mut records = BTreeMap::new();
        for (id, (engine, record)) in next {
            committed.insert(id.clone(), engine);
            records.insert(id, record);
        }
        self.robots = committed;
        Ok(FleetStep {
            records,
            stationary_reservations: reservations,
            recovery,
        })
    }

    pub fn fail_robot(
        &mut self,
        robot_id: &RobotId,
        plans: &mut PlanCoordinator,
    ) -> Result<RecoveryEvent, FleetError> {
        if !self.robots.contains_key(robot_id) {
            return Err(FleetError::UnknownRobot);
        }
        plans.hold_for_recovery(Some(robot_id.as_str()))?;
        Ok(RecoveryEvent {
            reason: RecoveryReason::RobotFailure,
            held_robots: self
                .robots
                .keys()
                .map(|id| id.as_str().to_owned())
                .collect(),
            replan_required: true,
        })
    }

    fn preview(
        &self,
        actions: &BTreeMap<RobotId, i32>,
        targets: &BTreeMap<RobotId, WorldPosition>,
        emergency: bool,
    ) -> Result<BTreeMap<RobotId, (SimulationEngine<C>, StepRecord)>, FleetError> {
        let selected = if emergency {
            self.robots.keys().cloned().collect()
        } else {
            BTreeSet::new()
        };
        self.preview_selected_emergency(actions, targets, &selected)
    }

    fn preview_selected_emergency(
        &self,
        actions: &BTreeMap<RobotId, i32>,
        targets: &BTreeMap<RobotId, WorldPosition>,
        emergency: &BTreeSet<RobotId>,
    ) -> Result<BTreeMap<RobotId, (SimulationEngine<C>, StepRecord)>, FleetError> {
        let mut previews = BTreeMap::new();
        for (id, engine) in &self.robots {
            let mut clone = engine.clone();
            if emergency.contains(id) {
                clone.latch_emergency_stop();
            }
            let record = clone.step_with_target(
                actions.get(id).copied().unwrap_or(0),
                targets.get(id).copied(),
            )?;
            previews.insert(id.clone(), (clone, record));
        }
        Ok(previews)
    }
}

fn conflicting_pairs<C: MonotonicClock>(
    current: &BTreeMap<RobotId, SimulationEngine<C>>,
    next: &BTreeMap<RobotId, (SimulationEngine<C>, StepRecord)>,
    required: f64,
    include_two_second_prediction: bool,
) -> Vec<(RobotId, RobotId)> {
    let ids: Vec<_> = current.keys().collect();
    let mut conflicts = Vec::new();
    for (index, left) in ids.iter().enumerate() {
        for right in ids.iter().skip(index + 1) {
            let transition_distance = simultaneous_segment_distance_squared(
                current[*left].state(),
                next[*left].1.state,
                current[*right].state(),
                next[*right].1.state,
            );
            let predicted_distance = if include_two_second_prediction {
                constant_velocity_distance_squared(next[*left].1.state, next[*right].1.state, 2.0)
            } else {
                f64::INFINITY
            };
            if transition_distance.min(predicted_distance)
                < (required + SAFETY_EPSILON_METERS).powi(2)
            {
                conflicts.push(((*left).clone(), (*right).clone()));
            }
        }
    }
    conflicts
}

fn constant_velocity_distance_squared(
    left: RobotState,
    right: RobotState,
    horizon_seconds: f64,
) -> f64 {
    let relative_x = left.position().x_meters() - right.position().x_meters();
    let relative_y = left.position().y_meters() - right.position().y_meters();
    let velocity_x = left.velocity().x_mps() - right.velocity().x_mps();
    let velocity_y = left.velocity().y_mps() - right.velocity().y_mps();
    let speed_squared = velocity_x.powi(2) + velocity_y.powi(2);
    let time = if speed_squared == 0.0 {
        0.0
    } else {
        (-(relative_x * velocity_x + relative_y * velocity_y) / speed_squared)
            .clamp(0.0, horizon_seconds)
    };
    (relative_x + velocity_x * time).powi(2) + (relative_y + velocity_y * time).powi(2)
}

fn simultaneous_segment_distance_squared(
    left_start: RobotState,
    left_end: RobotState,
    right_start: RobotState,
    right_end: RobotState,
) -> f64 {
    let relative_start_x = left_start.position().x_meters() - right_start.position().x_meters();
    let relative_start_y = left_start.position().y_meters() - right_start.position().y_meters();
    let relative_delta_x = (left_end.position().x_meters() - left_start.position().x_meters())
        - (right_end.position().x_meters() - right_start.position().x_meters());
    let relative_delta_y = (left_end.position().y_meters() - left_start.position().y_meters())
        - (right_end.position().y_meters() - right_start.position().y_meters());
    let denominator = relative_delta_x.powi(2) + relative_delta_y.powi(2);
    let time = if denominator == 0.0 {
        0.0
    } else {
        (-(relative_start_x * relative_delta_x + relative_start_y * relative_delta_y) / denominator)
            .clamp(0.0, 1.0)
    };
    (relative_start_x + relative_delta_x * time).powi(2)
        + (relative_start_y + relative_delta_y * time).powi(2)
}

fn corridor_conflict<C: MonotonicClock>(
    left: &SimulationEngine<C>,
    right: &SimulationEngine<C>,
    left_action: i32,
    right_action: i32,
) -> bool {
    let opposite = matches!(
        (left_action, right_action),
        (1, 3) | (3, 1) | (2, 4) | (4, 2)
    );
    if !opposite {
        return false;
    }
    let Ok(left_cell) = left.map().world_to_grid(left.state().position()) else {
        return false;
    };
    let Ok(right_cell) = right.map().world_to_grid(right.state().position()) else {
        return false;
    };
    (left_cell.column() == right_cell.column() || left_cell.row() == right_cell.row())
        && traversable_neighbors(left.map(), left_cell) <= 2
        && traversable_neighbors(right.map(), right_cell) <= 2
}

fn traversable_neighbors(map: &crate::world::GridMap, cell: crate::world::GridCell) -> usize {
    let candidates = [
        cell.column()
            .checked_sub(1)
            .map(|column| crate::world::GridCell::new(column, cell.row())),
        cell.column()
            .checked_add(1)
            .map(|column| crate::world::GridCell::new(column, cell.row())),
        cell.row()
            .checked_sub(1)
            .map(|row| crate::world::GridCell::new(cell.column(), row)),
        cell.row()
            .checked_add(1)
            .map(|row| crate::world::GridCell::new(cell.column(), row)),
    ];
    candidates
        .into_iter()
        .flatten()
        .filter(|candidate| map.is_blocked(*candidate).is_ok_and(|blocked| !blocked))
        .count()
}

fn validate_pairwise_states<'a>(
    states: impl Iterator<Item = (&'a RobotId, RobotState)>,
    required: f64,
) -> Result<(), FleetError> {
    let states: Vec<_> = states.collect();
    for (index, (_, left)) in states.iter().enumerate() {
        for (_, right) in states.iter().skip(index + 1) {
            let distance = (left.position().x_meters() - right.position().x_meters())
                .hypot(left.position().y_meters() - right.position().y_meters());
            if distance < required + SAFETY_EPSILON_METERS {
                return Err(FleetError::UnsafeInitialSeparation);
            }
        }
    }
    Ok(())
}

#[derive(Debug)]
pub enum FleetError {
    EmptyFleet,
    DuplicateRobot,
    UnknownRobot,
    UnsafeInitialSeparation,
    CollisionInvariant,
    InvalidCheckpoint,
    Engine(EngineError),
    Plan(PlanError),
}

impl From<EngineError> for FleetError {
    fn from(value: EngineError) -> Self {
        Self::Engine(value)
    }
}

impl From<PlanError> for FleetError {
    fn from(value: PlanError) -> Self {
        Self::Plan(value)
    }
}

impl fmt::Display for FleetError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyFleet => {
                formatter.write_str("multi-robot engine requires at least one robot")
            }
            Self::DuplicateRobot => {
                formatter.write_str("multi-robot engine robot IDs must be unique")
            }
            Self::UnknownRobot => {
                formatter.write_str("robot is not owned by the multi-robot engine")
            }
            Self::UnsafeInitialSeparation => {
                formatter.write_str("initial robots violate minimum separation")
            }
            Self::CollisionInvariant => {
                formatter.write_str("no atomic collision-free fleet transition exists")
            }
            Self::InvalidCheckpoint => {
                formatter.write_str("fleet checkpoint does not match the configured robots")
            }
            Self::Engine(error) => error.fmt(formatter),
            Self::Plan(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for FleetError {}
