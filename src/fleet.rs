//! Atomic, deterministic multi-robot tick scheduling and local recovery.

use crate::checkpoint::{RecoveryCheckpoint, RobotSafetyCheckpoint};
use crate::motion::MotionTarget;
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
    StationaryBlocked,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RecoveryEvent {
    pub reason: RecoveryReason,
    pub held_robots: Vec<String>,
    pub replan_required: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FleetStep {
    pub traffic_wait: BTreeMap<String, BTreeSet<String>>,
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
    passage_rights: crate::traffic::PassageRights,
    passage_targets: BTreeMap<RobotId, MotionTarget>,
    stationary_waits: BTreeMap<String, (Option<uuid::Uuid>, BTreeSet<String>, u32)>,
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
            passage_rights: crate::traffic::PassageRights::default(),
            passage_targets: BTreeMap::new(),
            stationary_waits: BTreeMap::new(),
            robots: ordered,
            config,
            no_progress_ticks: 0,
        })
    }

    pub fn robots(&self) -> &BTreeMap<RobotId, SimulationEngine<C>> {
        &self.robots
    }

    /// Virtual membership removal is allowed only between ticks at rest.
    pub fn remove_stationary(&mut self, id: &RobotId) -> Result<bool, FleetError> {
        let Some(robot) = self.robots.get(id) else {
            return Ok(true);
        };
        if self.robots.len() == 1 {
            return Err(FleetError::EmptyFleet);
        }
        if robot.state().velocity().magnitude() >= 1e-6
            || robot.state().acceleration().magnitude() >= 1e-6
        {
            return Ok(false);
        }
        self.robots.remove(id);
        self.passage_targets.remove(id);
        self.no_progress_ticks = 0;
        Ok(true)
    }

    pub fn apply_motion_profile(
        &mut self,
        id: &RobotId,
        profile: &crate::contracts::provisioning_generated::MotionProfileLimits,
    ) -> Result<bool, FleetError> {
        let limits = profile.motion_limits().map_err(EngineError::from)?;
        Ok(self
            .robots
            .get_mut(id)
            .ok_or(FleetError::UnknownRobot)?
            .apply_motion_limits(limits)?)
    }

    pub const fn no_progress_ticks(&self) -> u32 {
        self.no_progress_ticks
    }

    /// The next atomic fleet step validates emergency braking against every robot.
    pub fn latch_robot_emergency_stop(&mut self, id: &RobotId) -> Result<(), FleetError> {
        let robot = self.robots.get_mut(id).ok_or(FleetError::UnknownRobot)?;
        robot.latch_emergency_stop();
        Ok(())
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
            independent_plans: BTreeMap::new(),
            motion_profiles: self
                .robots
                .iter()
                .map(|(id, engine)| {
                    (
                        id.as_str().to_owned(),
                        crate::checkpoint::CheckpointMotionLimits::from_motion_limits(
                            engine.motion_limits(),
                        ),
                    )
                })
                .collect(),
            no_progress_ticks: self.no_progress_ticks,
            station_states: BTreeMap::new(),
            battery_depletion_ids: BTreeMap::new(),
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
            if let Some(profile) = checkpoint.motion_profiles.get(&robot.robot_id) {
                let limits = profile
                    .motion_limits()
                    .map_err(|_| FleetError::InvalidCheckpoint)?;
                if !engine.apply_motion_limits(limits)? {
                    return Err(FleetError::InvalidCheckpoint);
                }
            }
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
        self.passage_rights = crate::traffic::PassageRights::default();
        self.passage_targets.clear();
        self.stationary_waits.clear();
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
        let targets = targets
            .iter()
            .map(|(id, target)| (id.clone(), (*target).into()))
            .collect();
        self.step_with_motion_targets(requested_actions, &targets, plans)
    }

    pub fn step_with_motion_targets(
        &mut self,
        requested_actions: &BTreeMap<RobotId, i32>,
        targets: &BTreeMap<RobotId, MotionTarget>,
        plans: &mut PlanCoordinator,
    ) -> Result<FleetStep, FleetError> {
        let authorized = self
            .robots
            .keys()
            .filter(|id| plans.motion_authorized(id.as_str()))
            .cloned()
            .collect();
        let step = self.step_authorized(requested_actions, targets, &authorized)?;
        if step
            .recovery
            .as_ref()
            .is_some_and(|event| event.replan_required)
        {
            plans.hold_for_recovery(None)?;
        }
        Ok(step)
    }

    /// Independent Core Orders share one atomic physical safety decision.
    pub fn step_with_independent_plans(
        &mut self,
        requested_actions: &BTreeMap<RobotId, i32>,
        targets: &BTreeMap<RobotId, WorldPosition>,
        plans: &mut BTreeMap<RobotId, PlanCoordinator>,
    ) -> Result<FleetStep, FleetError> {
        let targets = targets
            .iter()
            .map(|(id, target)| (id.clone(), (*target).into()))
            .collect();
        self.step_with_independent_motion_targets(requested_actions, &targets, plans)
    }

    pub fn step_with_independent_motion_targets(
        &mut self,
        requested_actions: &BTreeMap<RobotId, i32>,
        targets: &BTreeMap<RobotId, MotionTarget>,
        plans: &mut BTreeMap<RobotId, PlanCoordinator>,
    ) -> Result<FleetStep, FleetError> {
        let authorized = plans
            .iter()
            .filter(|(id, plan)| plan.motion_authorized(id.as_str()))
            .map(|(id, _)| id.clone())
            .collect();
        let step = self.step_authorized(requested_actions, targets, &authorized)?;
        if step
            .recovery
            .as_ref()
            .is_some_and(|event| event.replan_required)
        {
            let recovery = step.recovery.as_ref().expect("checked recovery");
            for (id, plan) in plans.iter_mut() {
                if recovery.held_robots.iter().any(|held| held == id.as_str())
                    && plan.active_revision_id().is_some()
                {
                    plan.hold_for_recovery(None)?;
                }
            }
        }
        Ok(step)
    }

    /// Occupancy control is used only by the default operational controller.
    /// The existing physical prediction, collision and braking checks still run.
    pub fn step_with_passage_rights(
        &mut self,
        requested_actions: &BTreeMap<RobotId, i32>,
        targets: &BTreeMap<RobotId, MotionTarget>,
        plans: &mut BTreeMap<RobotId, PlanCoordinator>,
    ) -> Result<FleetStep, FleetError> {
        use crate::traffic::{Request, corridor_resources};
        let mut occupied = BTreeMap::new();
        let mut retained = BTreeMap::new();
        let mut requests = Vec::new();
        for (id, engine) in &self.robots {
            let map = engine.map();
            let state = engine.state();
            let current = map
                .world_to_grid(state.position())
                .map_err(FleetError::Map)?;
            let radius = engine
                .footprint_radius_meters()
                .max(self.config.minimum_center_separation_meters / 2.0);
            occupied.insert(
                id.as_str().to_owned(),
                passage_segment_resources(map, state.position(), state.position(), radius),
            );
            // Clone previews leave faults, time and emergency latches unchanged.
            retained.insert(
                id.as_str().to_owned(),
                passage_motion_resources(engine, 0, None, radius)?.0,
            );
            let moving =
                state.velocity().magnitude() >= 1e-6 || state.acceleration().magnitude() >= 1e-6;
            if !moving {
                self.passage_targets.remove(id);
            }
            if !plans
                .get(id)
                .is_some_and(|p| p.motion_authorized(id.as_str()))
            {
                continue;
            }
            let Some(target) = targets.get(id) else {
                continue;
            };
            if requested_actions.get(id).copied().unwrap_or(0) == 0 {
                continue;
            }
            let end = map
                .world_to_grid(target.position)
                .map_err(FleetError::Map)?;
            // A changed target must wait for a stop on the old axis.
            if moving
                && self
                    .passage_targets
                    .get(id)
                    .is_some_and(|old| old != target)
            {
                continue;
            }
            if current.column() != end.column() && current.row() != end.row() {
                return Err(FleetError::CollisionInvariant);
            }
            let (mut required, emergency) = passage_motion_resources(
                engine,
                requested_actions.get(id).copied().unwrap_or(0),
                Some(*target),
                radius,
            )?;
            if emergency {
                continue;
            }
            // Narrow corridors remain exclusive, including both exits.
            for x in current.column().min(end.column())..=current.column().max(end.column()) {
                for y in current.row().min(end.row())..=current.row().max(end.row()) {
                    let corridor = corridor_resources(map, (x, y));
                    if corridor.len() > 1 {
                        required.extend(corridor);
                    }
                }
            }
            requests.push(Request {
                robot_id: id.as_str().to_owned(),
                resources: required,
            });
        }
        let mut allocation = self
            .passage_rights
            .allocate(&occupied, &retained, &requests);
        for (id, target) in targets {
            if allocation.granted.contains(id.as_str()) {
                self.passage_targets.insert(id.clone(), *target);
            }
        }
        let mut actions: BTreeMap<_, _> = requested_actions
            .iter()
            .map(|(id, action)| {
                (
                    id.clone(),
                    if allocation.granted.contains(id.as_str()) {
                        *action
                    } else {
                        0
                    },
                )
            })
            .collect();
        let mut allowed_targets: BTreeMap<_, _> = targets
            .iter()
            .filter(|(id, _)| allocation.granted.contains(id.as_str()))
            .map(|(id, target)| (id.clone(), *target))
            .collect();
        // The existing longer-horizon kernel predicts uninterrupted controller
        // motion. Yield before entering it so ordinary traffic contention does
        // not become a collision incident. Recheck after each WAIT decision.
        loop {
            let preview = self.preview(&actions, &allowed_targets, false)?;
            let conflicts = conflicting_pairs(
                &self.robots,
                &preview,
                self.config.minimum_center_separation_meters,
                Some(MotionPrediction {
                    actions: &actions,
                    targets: &allowed_targets,
                }),
            );
            let mut changed = false;
            for (left, right) in conflicts {
                let (yielding, blocker) = if !allocation.granted.contains(left.as_str()) {
                    (right, left)
                } else if !allocation.granted.contains(right.as_str()) {
                    (left, right)
                } else {
                    // Match the final collision kernel's stable tie-breaker;
                    // alternating forecast winners can strand a crossing robot.
                    (right, left)
                };
                if allocation.granted.remove(yielding.as_str()) {
                    actions.insert(yielding.clone(), 0);
                    allowed_targets.remove(&yielding);
                    allocation
                        .blockers
                        .entry(yielding.as_str().to_owned())
                        .or_default()
                        .insert(blocker.as_str().to_owned());
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        // Forecast yielding adds edges after physical resource allocation.
        allocation.refresh_cycle();
        self.passage_rights.confirm(&allocation.granted);
        let mut step =
            self.step_with_independent_motion_targets(&actions, &allowed_targets, plans)?;
        step.traffic_wait = allocation.blockers;
        // A parked blocker cannot release its resources by waiting. Count only
        // stopped requests against blockers without motion authority.
        let mut stationary_blocked = Vec::new();
        self.stationary_waits
            .retain(|id, _| step.traffic_wait.contains_key(id));
        for (id, blockers) in &step.traffic_wait {
            let robot_id = RobotId::new(id.clone()).map_err(|_| FleetError::CollisionInvariant)?;
            let stopped = |state: RobotState| {
                state.velocity().magnitude() < 1e-6 && state.acceleration().magnitude() < 1e-6
            };
            let parked: BTreeSet<_> = blockers
                .iter()
                .filter(|blocker| {
                    self.robots.iter().any(|(key, engine)| {
                        key.as_str() == blocker.as_str()
                            && stopped(engine.state())
                            && !plans
                                .get(key)
                                .is_some_and(|plan| plan.motion_authorized(blocker))
                    })
                })
                .cloned()
                .collect();
            if parked.is_empty()
                || !stopped(self.robots[&robot_id].state())
                || !plans
                    .get(&robot_id)
                    .is_some_and(|plan| plan.motion_authorized(id))
            {
                self.stationary_waits.remove(id);
                continue;
            }
            let revision = plans[&robot_id].active_revision_id();
            let wait =
                self.stationary_waits
                    .entry(id.clone())
                    .or_insert((revision, parked.clone(), 0));
            if wait.0 != revision || wait.1 != parked {
                *wait = (revision, parked, 0);
            }
            wait.2 = wait.2.saturating_add(1);
            if wait.2 >= self.config.deadlock_ticks {
                stationary_blocked.push(id.clone());
            }
        }
        if allocation.cycle.is_empty() && step.recovery.is_none() && !stationary_blocked.is_empty()
        {
            for id in &stationary_blocked {
                let key = RobotId::new(id.clone()).map_err(|_| FleetError::CollisionInvariant)?;
                plans
                    .get_mut(&key)
                    .expect("authorized plan")
                    .hold_for_recovery(None)?;
                self.stationary_waits.remove(id);
            }
            step.recovery = Some(RecoveryEvent {
                reason: RecoveryReason::StationaryBlocked,
                held_robots: stationary_blocked,
                replan_required: true,
            });
        }
        if !allocation.cycle.is_empty() {
            let held_robots: Vec<_> = allocation.cycle.into_iter().collect();
            for id in &held_robots {
                if let Some(plan) = plans
                    .iter_mut()
                    .find_map(|(key, plan)| (key.as_str() == id).then_some(plan))
                {
                    plan.hold_for_recovery(None)?;
                }
            }
            step.recovery = Some(RecoveryEvent {
                reason: RecoveryReason::Deadlock,
                held_robots,
                replan_required: true,
            });
        }
        Ok(step)
    }

    /// Called between fleet ticks, before the new robot receives motion authority.
    pub fn insert(&mut self, engine: SimulationEngine<C>) -> Result<(), FleetError> {
        let id = engine.robot_id().clone();
        if self.robots.contains_key(&id) {
            return Err(FleetError::DuplicateRobot);
        }
        validate_pairwise_states(
            self.robots
                .iter()
                .map(|(id, engine)| (id, engine.state()))
                .chain(std::iter::once((&id, engine.state()))),
            self.config.minimum_center_separation_meters,
        )?;
        let candidate = Self::new(
            self.robots
                .values()
                .cloned()
                .chain(std::iter::once(engine.clone())),
            self.config,
        )?;
        let actions = candidate.robots.keys().map(|id| (id.clone(), 0)).collect();
        let preview = candidate.preview(&actions, &BTreeMap::new(), false)?;
        if !conflicting_pairs(
            &candidate.robots,
            &preview,
            self.config.minimum_center_separation_meters,
            Some(MotionPrediction {
                actions: &actions,
                targets: &BTreeMap::new(),
            }),
        )
        .is_empty()
        {
            return Err(FleetError::CollisionInvariant);
        }
        self.robots.insert(id, engine);
        Ok(())
    }

    fn step_authorized(
        &mut self,
        requested_actions: &BTreeMap<RobotId, i32>,
        targets: &BTreeMap<RobotId, MotionTarget>,
        authorized: &BTreeSet<RobotId>,
    ) -> Result<FleetStep, FleetError> {
        let reservations: BTreeSet<String> = self
            .robots
            .keys()
            .filter(|id| !authorized.contains(*id))
            .map(|id| id.as_str().to_owned())
            .collect();
        let mut actions = BTreeMap::new();
        for id in self.robots.keys() {
            let action = requested_actions.get(id).copied().unwrap_or(0);
            actions.insert(id.clone(), if authorized.contains(id) { action } else { 0 });
        }

        let initial = self.preview(&actions, targets, false)?;
        let conflicts = conflicting_pairs(
            &self.robots,
            &initial,
            self.config.minimum_center_separation_meters,
            Some(MotionPrediction {
                actions: &actions,
                targets,
            }),
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
            None,
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
                None,
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
            .any(|(id, action)| *action != 0 && authorized.contains(id));
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

        let mut committed = BTreeMap::new();
        let mut records = BTreeMap::new();
        for (id, (engine, record)) in next {
            committed.insert(id.clone(), engine);
            records.insert(id, record);
        }
        self.robots = committed;
        Ok(FleetStep {
            traffic_wait: BTreeMap::new(),
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
        targets: &BTreeMap<RobotId, MotionTarget>,
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
        targets: &BTreeMap<RobotId, MotionTarget>,
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

struct MotionPrediction<'a> {
    actions: &'a BTreeMap<RobotId, i32>,
    targets: &'a BTreeMap<RobotId, MotionTarget>,
}

fn conflicting_pairs<C: MonotonicClock + Clone>(
    current: &BTreeMap<RobotId, SimulationEngine<C>>,
    next: &BTreeMap<RobotId, (SimulationEngine<C>, StepRecord)>,
    required: f64,
    prediction: Option<MotionPrediction<'_>>,
) -> Vec<(RobotId, RobotId)> {
    let ids: Vec<_> = current.keys().collect();
    let mut conflicts = Vec::new();
    let mut forecasts = BTreeMap::new();
    for (index, left) in ids.iter().enumerate() {
        for right in ids.iter().skip(index + 1) {
            let transition_distance = simultaneous_segment_distance_squared(
                current[*left].state(),
                next[*left].1.state,
                current[*right].state(),
                next[*right].1.state,
            );
            let predicted_distance = if let Some(MotionPrediction { actions, targets }) = prediction
            {
                let straight = constant_velocity_distance_squared(
                    next[*left].1.state,
                    next[*right].1.state,
                    2.0,
                );
                if targets.contains_key(*left) || targets.contains_key(*right) {
                    for id in [*left, *right] {
                        forecasts.entry(id).or_insert_with(|| {
                            targeted_prediction(
                                &next[id].0,
                                actions.get(id).copied().unwrap_or(0),
                                targets.get(id).copied(),
                            )
                        });
                    }
                    targeted_prediction_distance_squared(
                        forecasts[*left].as_deref(),
                        forecasts[*right].as_deref(),
                    )
                } else {
                    straight
                }
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

/// Conservative swept-cell envelope including footprint and interpolation error.
fn passage_segment_resources(
    map: &crate::world::GridMap,
    start: WorldPosition,
    end: WorldPosition,
    radius: f64,
) -> crate::traffic::Resources {
    let origin = map.origin();
    let resolution = map.resolution_meters();
    let radius = radius + SAFETY_EPSILON_METERS;
    let min_x = ((start.x_meters().min(end.x_meters()) - radius - origin.x_meters()) / resolution)
        .floor() as i64;
    let max_x = ((start.x_meters().max(end.x_meters()) + radius - origin.x_meters()) / resolution)
        .floor() as i64;
    let min_y = ((start.y_meters().min(end.y_meters()) - radius - origin.y_meters()) / resolution)
        .floor() as i64;
    let max_y = ((start.y_meters().max(end.y_meters()) + radius - origin.y_meters()) / resolution)
        .floor() as i64;
    let mut resources = crate::traffic::Resources::new();
    for x in min_x.max(0)..=max_x.min(i64::from(map.width()) - 1) {
        for y in min_y.max(0)..=max_y.min(i64::from(map.height()) - 1) {
            resources.extend(crate::traffic::corridor_resources(
                map,
                (x as u32, y as u32),
            ));
        }
    }
    resources
}

fn passage_motion_resources<C: MonotonicClock + Clone>(
    engine: &SimulationEngine<C>,
    action: i32,
    target: Option<MotionTarget>,
    radius: f64,
) -> Result<(crate::traffic::Resources, bool), FleetError> {
    let mut preview = engine.clone();
    let record = preview
        .step_with_target(action, target)
        .map_err(FleetError::Engine)?;
    let next = record.state;
    let limits = engine.motion_limits();
    // Bound interpolation between the endpoints of the actual next tick.
    let current = engine.state();
    let stationary = current.position() == next.position()
        && current.velocity().magnitude() == 0.0
        && current.acceleration().magnitude() == 0.0
        && next.velocity().magnitude() == 0.0
        && next.acceleration().magnitude() == 0.0;
    // A truly stationary WAIT has no swept-trajectory interpolation error.
    // Padding it with maximum emergency acceleration can claim an empty neighbor
    // cell forever and create a false wait cycle between safely separated robots.
    let margin = if stationary {
        0.0
    } else {
        limits
            .max_acceleration_mps2()
            .max(limits.max_deceleration_mps2())
            .max(limits.max_emergency_deceleration_mps2())
            * crate::types::CONTROL_TICK_SECONDS.powi(2)
            / 8.0
    };
    let mut resources = passage_segment_resources(
        engine.map(),
        engine.state().position(),
        next.position(),
        radius + margin,
    );
    // Reserve both ordinary and emergency stops, without assuming future grants
    // or continued motion by the leader.
    for (index, (deceleration, jerk)) in [
        (limits.max_deceleration_mps2(), limits.max_jerk_mps3()),
        (
            limits.max_emergency_deceleration_mps2(),
            limits.max_emergency_jerk_mps3(),
        ),
    ]
    .into_iter()
    .enumerate()
    {
        // A latched emergency continues the stronger braking profile; switching
        // back to ordinary limits can be kinematically impossible.
        if index == 0 && record.safety_outcome == crate::safety::SafetyOutcome::EmergencyStop {
            continue;
        }
        let segments = crate::motion::preview_stop_trajectory(next, deceleration, jerk, limits)
            .map_err(|_| FleetError::CollisionInvariant)?;
        for segment in segments {
            resources.extend(passage_segment_resources(
                engine.map(),
                segment.start,
                segment.end,
                radius + segment.curvature_margin_meters,
            ));
        }
    }
    Ok((
        resources,
        record.safety_outcome == crate::safety::SafetyOutcome::EmergencyStop,
    ))
}

// The endpoint is the next required stop, possibly beyond intermediate straight nodes.
// Replay the same deterministic target controller over the existing 2s horizon,
// checking every swept tick. A failed forecast is treated as a conflict.
fn targeted_prediction<C: MonotonicClock + Clone>(
    engine: &SimulationEngine<C>,
    action: i32,
    target: Option<MotionTarget>,
) -> Option<Vec<RobotState>> {
    let mut engine = engine.clone();
    let mut states = Vec::with_capacity(21);
    states.push(engine.state());
    for _ in 0..20 {
        states.push(engine.step_with_target(action, target).ok()?.state);
    }
    Some(states)
}

fn targeted_prediction_distance_squared(
    left: Option<&[RobotState]>,
    right: Option<&[RobotState]>,
) -> f64 {
    let (Some(left), Some(right)) = (left, right) else {
        return 0.0;
    };
    left.windows(2)
        .zip(right.windows(2))
        .map(|(left, right)| {
            simultaneous_segment_distance_squared(left[0], left[1], right[0], right[1])
        })
        .fold(f64::INFINITY, f64::min)
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
    Map(crate::world::MapError),
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
            Self::Map(error) => error.fmt(formatter),
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

#[cfg(test)]
mod passage_tests {
    use super::*;
    #[test]
    fn stationary_braking_resources_match_observed_footprint() {
        use crate::simulation::{EngineConfig, ManualMonotonicClock};
        use crate::types::{Acceleration, Velocity};
        let map =
            crate::world::GridMap::new(32, 20, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, [])
                .unwrap();
        let position = WorldPosition::new(13.5, 3.2551316701949484).unwrap();
        let engine = SimulationEngine::new(
            ManualMonotonicClock::default(),
            RobotId::new("r-010").unwrap(),
            map.clone(),
            RobotState::new(position, Velocity::ZERO, Acceleration::ZERO, 0.0).unwrap(),
            EngineConfig {
                motion_limits: crate::motion::MotionLimits::new(1.5, 1.0, 1.5, 6.0, 3.0, 60.0)
                    .unwrap(),
                safety: crate::safety::SafetyConfig::new(0.2, 0.05).unwrap(),
                sensor: crate::sensing::SensorConfig::new(0.0, 0).unwrap(),
            },
            7,
            &[],
        )
        .unwrap();
        let observed = passage_segment_resources(&map, position, position, 0.25);
        let (retained, emergency) = passage_motion_resources(&engine, 0, None, 0.25).unwrap();
        assert!(!emergency);
        assert!(!observed.contains(&(13, 2)));
        assert_eq!(retained, observed);
    }

    #[test]
    fn shared_forecasts_match_pairwise_replay() {
        use crate::simulation::{EngineConfig, ManualMonotonicClock};
        use crate::types::{Acceleration, Velocity};
        let mut robots = BTreeMap::new();
        let mut actions = BTreeMap::new();
        let mut targets = BTreeMap::new();
        for index in 0..8 {
            let id = RobotId::new(format!("r{index}")).unwrap();
            let y = 2.5 + f64::from(index);
            let engine = SimulationEngine::new(
                ManualMonotonicClock::default(),
                id.clone(),
                crate::world::GridMap::new(20, 12, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, [])
                    .unwrap(),
                RobotState::new(
                    WorldPosition::new(2.5, y).unwrap(),
                    Velocity::ZERO,
                    Acceleration::ZERO,
                    0.0,
                )
                .unwrap(),
                EngineConfig {
                    motion_limits: crate::motion::MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0)
                        .unwrap(),
                    safety: crate::safety::SafetyConfig::new(0.2, 0.05).unwrap(),
                    sensor: crate::sensing::SensorConfig::new(0.0, 0).unwrap(),
                },
                7,
                &[],
            )
            .unwrap();
            robots.insert(id.clone(), engine);
            actions.insert(id.clone(), 1);
            // Include mixed targeted and untargeted pairs.
            if index % 2 == 0 {
                targets.insert(id, WorldPosition::new(8.5, y).unwrap().into());
            }
        }
        let next: BTreeMap<_, _> = robots
            .iter()
            .map(|(id, engine)| {
                let mut engine = engine.clone();
                let record = engine
                    .step_with_target(actions[id], targets.get(id).copied())
                    .unwrap();
                (id.clone(), (engine, record))
            })
            .collect();
        let started = std::time::Instant::now();
        let mut expected = Vec::new();
        let ids: Vec<_> = robots.keys().collect();
        for (index, left) in ids.iter().enumerate() {
            for right in ids.iter().skip(index + 1) {
                let mut left_engine = next[*left].0.clone();
                let mut right_engine = next[*right].0.clone();
                let mut distance = constant_velocity_distance_squared(
                    next[*left].1.state,
                    next[*right].1.state,
                    2.0,
                );
                if targets.contains_key(*left) || targets.contains_key(*right) {
                    distance = f64::INFINITY;
                    for _ in 0..20 {
                        let left_start = left_engine.state();
                        let right_start = right_engine.state();
                        match (
                            left_engine
                                .step_with_target(actions[*left], targets.get(*left).copied()),
                            right_engine
                                .step_with_target(actions[*right], targets.get(*right).copied()),
                        ) {
                            (Ok(left_next), Ok(right_next)) => {
                                distance = distance.min(simultaneous_segment_distance_squared(
                                    left_start,
                                    left_next.state,
                                    right_start,
                                    right_next.state,
                                ))
                            }
                            _ => {
                                distance = 0.0;
                                break;
                            }
                        }
                    }
                }
                let transition = simultaneous_segment_distance_squared(
                    robots[*left].state(),
                    next[*left].1.state,
                    robots[*right].state(),
                    next[*right].1.state,
                );
                if transition.min(distance) < (1.1 + SAFETY_EPSILON_METERS).powi(2) {
                    expected.push(((*left).clone(), (*right).clone()));
                }
            }
        }
        let pairwise_elapsed = started.elapsed();
        let started = std::time::Instant::now();
        let actual = conflicting_pairs(
            &robots,
            &next,
            1.1,
            Some(MotionPrediction {
                actions: &actions,
                targets: &targets,
            }),
        );
        eprintln!(
            "pairwise={pairwise_elapsed:?}, shared={:?}",
            started.elapsed()
        );
        assert!(!expected.is_empty());
        assert_eq!(actual, expected);
        assert_eq!(targeted_prediction_distance_squared(None, None), 0.0);
    }

    #[test]
    fn rear_cell_stays_until_entire_safety_footprint_clears() {
        let map = crate::world::GridMap::new(9, 7, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, [])
            .unwrap();
        let touching = WorldPosition::new(2.25, 3.5).unwrap();
        assert!(passage_segment_resources(&map, touching, touching, 0.25).contains(&(1, 3)));
        let cleared = WorldPosition::new(2.26, 3.5).unwrap();
        assert!(!passage_segment_resources(&map, cleared, cleared, 0.25).contains(&(1, 3)));
        assert!(passage_segment_resources(&map, cleared, cleared, 0.25).contains(&(2, 3)));
    }
}
