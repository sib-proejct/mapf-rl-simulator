//! Deterministic `PlanRevision` prepare barrier and activation ordering.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use uuid::Uuid;

pub const PREPARE_BARRIER_TIMEOUT_MS: u64 = 15_000;

#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PlanTarget {
    pub robot_id: String,
    pub order_id: String,
    pub order_update_id: u64,
    pub content_digest_sha256: String,
}

impl PlanTarget {
    pub fn validate(&self) -> Result<(), PlanError> {
        validate_id(&self.robot_id, "robotId")?;
        validate_id(&self.order_id, "orderId")?;
        if !is_sha256(&self.content_digest_sha256) {
            return Err(PlanError::InvalidField("contentDigestSha256"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PlanRevision {
    pub plan_revision_id: Uuid,
    pub planning_snapshot_digest_sha256: String,
    pub targets: Vec<PlanTarget>,
    /// Every target exactly once, in the safe release order chosen by Planner.
    pub activation_order: Vec<String>,
}

impl PlanRevision {
    pub fn validate(&self) -> Result<(), PlanError> {
        if self.plan_revision_id.get_version_num() != 4 {
            return Err(PlanError::InvalidField("planRevisionId"));
        }
        if !is_sha256(&self.planning_snapshot_digest_sha256) {
            return Err(PlanError::InvalidField("planningSnapshotDigestSha256"));
        }
        if self.targets.is_empty() {
            return Err(PlanError::InvalidField("targets"));
        }
        let mut target_ids = BTreeSet::new();
        for target in &self.targets {
            target.validate()?;
            if !target_ids.insert(target.robot_id.as_str()) {
                return Err(PlanError::DuplicateTarget);
            }
        }
        let activation_ids: BTreeSet<_> =
            self.activation_order.iter().map(String::as_str).collect();
        if activation_ids.len() != self.activation_order.len()
            || activation_ids != target_ids
            || self.activation_order.len() != self.targets.len()
        {
            return Err(PlanError::InvalidActivationOrder);
        }
        Ok(())
    }

    fn target(&self, robot_id: &str) -> Option<&PlanTarget> {
        self.targets
            .iter()
            .find(|target| target.robot_id == robot_id)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RobotPlanState {
    Holding,
    Prepared,
    Active,
    Failed,
    Aborted,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RevisionState {
    Preparing,
    Ready,
    Activating,
    Active,
    HeldForRecovery,
    Aborted,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlanAckDisposition {
    Holding,
    Prepared,
    Activated,
    Aborted,
    Duplicate,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PlanAck {
    pub plan_revision_id: Uuid,
    pub robot_id: String,
    pub disposition: PlanAckDisposition,
    pub code: &'static str,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct ActiveRevision {
    revision: PlanRevision,
    prepare_started_ms: u64,
    states: BTreeMap<String, RobotPlanState>,
    activation_index: usize,
    runtime_confirmed: BTreeSet<String>,
    recovery_required: bool,
}

/// Owns only plan authorization. Motion remains subject to the fleet safety gate.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PlanCoordinator {
    active: Option<ActiveRevision>,
    reconciled: bool,
}

impl Default for PlanCoordinator {
    fn default() -> Self {
        Self::new()
    }
}

impl PlanCoordinator {
    pub const fn new() -> Self {
        Self {
            active: None,
            reconciled: true,
        }
    }

    /// Starts an immutable revision. Every target immediately enters a local hold.
    pub fn begin_prepare(
        &mut self,
        revision: PlanRevision,
        monotonic_ms: u64,
    ) -> Result<Vec<PlanAck>, PlanError> {
        revision.validate()?;
        if let Some(current) = &self.active {
            if current.revision.plan_revision_id == revision.plan_revision_id {
                if current.revision != revision {
                    return Err(PlanError::ConflictingRevision);
                }
                return Ok(current
                    .states
                    .keys()
                    .map(|robot_id| PlanAck {
                        plan_revision_id: revision.plan_revision_id,
                        robot_id: robot_id.clone(),
                        disposition: PlanAckDisposition::Duplicate,
                        code: "PLAN_PREPARE_DUPLICATE",
                    })
                    .collect());
            }
            if matches!(
                self.state(),
                Some(RevisionState::Activating | RevisionState::Active)
            ) {
                return Err(PlanError::RevisionAlreadyActive);
            }
        }
        let states = revision
            .targets
            .iter()
            .map(|target| (target.robot_id.clone(), RobotPlanState::Holding))
            .collect();
        let acknowledgements = revision
            .targets
            .iter()
            .map(|target| PlanAck {
                plan_revision_id: revision.plan_revision_id,
                robot_id: target.robot_id.clone(),
                disposition: PlanAckDisposition::Holding,
                code: "PLAN_SAFE_HOLD_REQUIRED",
            })
            .collect();
        self.active = Some(ActiveRevision {
            revision,
            prepare_started_ms: monotonic_ms,
            states,
            activation_index: 0,
            runtime_confirmed: BTreeSet::new(),
            recovery_required: false,
        });
        self.reconciled = true;
        Ok(acknowledgements)
    }

    /// A prepare ack is emitted only after the robot is actually stationary.
    pub fn mark_safe_hold(
        &mut self,
        plan_revision_id: Uuid,
        robot_id: &str,
        is_stationary: bool,
        monotonic_ms: u64,
    ) -> Result<PlanAck, PlanError> {
        self.expire_if_due(monotonic_ms);
        let active = self.active.as_mut().ok_or(PlanError::NoRevision)?;
        if active.revision.plan_revision_id != plan_revision_id {
            return Err(PlanError::RevisionMismatch);
        }
        let state = active
            .states
            .get_mut(robot_id)
            .ok_or(PlanError::UnknownTarget)?;
        if !is_stationary {
            *state = RobotPlanState::Holding;
            return Ok(PlanAck {
                plan_revision_id,
                robot_id: robot_id.to_owned(),
                disposition: PlanAckDisposition::Holding,
                code: "PLAN_HOLDING_UNTIL_STATIONARY",
            });
        }
        if *state == RobotPlanState::Prepared {
            return Ok(PlanAck {
                plan_revision_id,
                robot_id: robot_id.to_owned(),
                disposition: PlanAckDisposition::Duplicate,
                code: "PLAN_PREPARE_DUPLICATE",
            });
        }
        if *state != RobotPlanState::Holding {
            return Err(PlanError::InvalidTransition);
        }
        *state = RobotPlanState::Prepared;
        Ok(PlanAck {
            plan_revision_id,
            robot_id: robot_id.to_owned(),
            disposition: PlanAckDisposition::Prepared,
            code: "PLAN_PREPARED",
        })
    }

    /// Activates exactly the next dependency-safe target after the full barrier.
    pub fn activate(
        &mut self,
        plan_revision_id: Uuid,
        target: &PlanTarget,
        monotonic_ms: u64,
    ) -> Result<PlanAck, PlanError> {
        self.expire_if_due(monotonic_ms);
        let active = self.active.as_mut().ok_or(PlanError::NoRevision)?;
        if active.revision.plan_revision_id != plan_revision_id {
            return Err(PlanError::RevisionMismatch);
        }
        if active.recovery_required {
            return Err(PlanError::RecoveryRequired);
        }
        if active
            .states
            .values()
            .any(|state| *state != RobotPlanState::Prepared && *state != RobotPlanState::Active)
        {
            return Err(PlanError::PrepareBarrierIncomplete);
        }
        if active.revision.target(&target.robot_id) != Some(target) {
            return Err(PlanError::TargetIdentityMismatch);
        }
        if active.states.get(&target.robot_id) == Some(&RobotPlanState::Active) {
            return Ok(PlanAck {
                plan_revision_id,
                robot_id: target.robot_id.clone(),
                disposition: PlanAckDisposition::Duplicate,
                code: "PLAN_ACTIVATION_DUPLICATE",
            });
        }
        if active.activation_index > 0 {
            let dependency = &active.revision.activation_order[active.activation_index - 1];
            if !active.runtime_confirmed.contains(dependency) {
                return Err(PlanError::ActivationDependencyIncomplete);
            }
        }
        let expected = active
            .revision
            .activation_order
            .get(active.activation_index)
            .ok_or(PlanError::InvalidTransition)?;
        if expected != &target.robot_id {
            return Err(PlanError::UnsafeActivationOrder);
        }
        active
            .states
            .insert(target.robot_id.clone(), RobotPlanState::Active);
        active.activation_index += 1;
        Ok(PlanAck {
            plan_revision_id,
            robot_id: target.robot_id.clone(),
            disposition: PlanAckDisposition::Activated,
            code: "PLAN_ACTIVATED",
        })
    }

    /// Records the runtime-state confirmation required before the next release.
    pub fn confirm_runtime_safe(
        &mut self,
        plan_revision_id: Uuid,
        robot_id: &str,
    ) -> Result<(), PlanError> {
        let active = self.active.as_mut().ok_or(PlanError::NoRevision)?;
        if active.revision.plan_revision_id != plan_revision_id {
            return Err(PlanError::RevisionMismatch);
        }
        if active.states.get(robot_id) != Some(&RobotPlanState::Active) {
            return Err(PlanError::InvalidTransition);
        }
        active.runtime_confirmed.insert(robot_id.to_owned());
        Ok(())
    }

    pub fn abort(&mut self, plan_revision_id: Uuid) -> Result<Vec<PlanAck>, PlanError> {
        let active = self.active.as_mut().ok_or(PlanError::NoRevision)?;
        if active.revision.plan_revision_id != plan_revision_id {
            return Err(PlanError::RevisionMismatch);
        }
        active.recovery_required = true;
        Ok(active
            .states
            .iter_mut()
            .map(|(robot_id, state)| {
                *state = RobotPlanState::Aborted;
                PlanAck {
                    plan_revision_id,
                    robot_id: robot_id.clone(),
                    disposition: PlanAckDisposition::Aborted,
                    code: "PLAN_ABORTED",
                }
            })
            .collect())
    }

    /// A robot failure never releases another target; the whole revision is held.
    pub fn hold_for_recovery(&mut self, failed_robot_id: Option<&str>) -> Result<(), PlanError> {
        let active = self.active.as_mut().ok_or(PlanError::NoRevision)?;
        if let Some(robot_id) = failed_robot_id
            && !active.states.contains_key(robot_id)
        {
            return Err(PlanError::UnknownTarget);
        }
        active.recovery_required = true;
        for (robot_id, state) in &mut active.states {
            *state = if Some(robot_id.as_str()) == failed_robot_id {
                RobotPlanState::Failed
            } else {
                RobotPlanState::Holding
            };
        }
        Ok(())
    }

    pub fn poll_timeout(&mut self, monotonic_ms: u64) -> bool {
        self.expire_if_due(monotonic_ms)
    }

    pub fn state(&self) -> Option<RevisionState> {
        let active = self.active.as_ref()?;
        if active.recovery_required {
            if active
                .states
                .values()
                .all(|state| *state == RobotPlanState::Aborted)
            {
                return Some(RevisionState::Aborted);
            }
            return Some(RevisionState::HeldForRecovery);
        }
        if active.activation_index == active.revision.targets.len() {
            Some(RevisionState::Active)
        } else if active.activation_index > 0 {
            Some(RevisionState::Activating)
        } else if active
            .states
            .values()
            .all(|state| *state == RobotPlanState::Prepared)
        {
            Some(RevisionState::Ready)
        } else {
            Some(RevisionState::Preparing)
        }
    }

    pub fn robot_state(&self, robot_id: &str) -> Option<RobotPlanState> {
        self.active.as_ref()?.states.get(robot_id).copied()
    }

    pub fn motion_authorized(&self, robot_id: &str) -> bool {
        self.reconciled
            && self.robot_state(robot_id) == Some(RobotPlanState::Active)
            && self
                .active
                .as_ref()
                .is_some_and(|active| !active.recovery_required)
    }

    pub fn stationary_reservations(&self) -> BTreeSet<String> {
        self.active
            .as_ref()
            .map(|active| {
                active
                    .states
                    .iter()
                    .filter(|(robot_id, _)| !self.motion_authorized(robot_id))
                    .map(|(robot_id, _)| robot_id.clone())
                    .collect()
            })
            .unwrap_or_default()
    }

    pub fn active_revision_id(&self) -> Option<Uuid> {
        self.active
            .as_ref()
            .map(|active| active.revision.plan_revision_id)
    }

    /// Restored authorization is fenced until a matching Core snapshot is checked.
    pub fn fence_for_restart(&mut self) {
        self.reconciled = false;
        if let Some(active) = &mut self.active {
            active.recovery_required = true;
            for state in active.states.values_mut() {
                *state = RobotPlanState::Holding;
            }
        }
    }

    pub fn reconcile_after_restart(
        &mut self,
        revision: Option<&PlanRevision>,
    ) -> Result<(), PlanError> {
        match (&self.active, revision) {
            (None, None) => {}
            (Some(active), Some(revision)) if &active.revision == revision => {}
            _ => return Err(PlanError::RevisionMismatch),
        }
        self.reconciled = true;
        Ok(())
    }

    fn expire_if_due(&mut self, monotonic_ms: u64) -> bool {
        let Some(active) = &mut self.active else {
            return false;
        };
        if active.activation_index > 0
            || active
                .states
                .values()
                .all(|state| *state == RobotPlanState::Prepared)
            || monotonic_ms
                < active
                    .prepare_started_ms
                    .saturating_add(PREPARE_BARRIER_TIMEOUT_MS)
        {
            return false;
        }
        active.recovery_required = true;
        for state in active.states.values_mut() {
            *state = RobotPlanState::Aborted;
        }
        true
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlanError {
    InvalidField(&'static str),
    DuplicateTarget,
    InvalidActivationOrder,
    ConflictingRevision,
    RevisionAlreadyActive,
    NoRevision,
    RevisionMismatch,
    UnknownTarget,
    InvalidTransition,
    PrepareBarrierIncomplete,
    ActivationDependencyIncomplete,
    UnsafeActivationOrder,
    TargetIdentityMismatch,
    RecoveryRequired,
}

impl fmt::Display for PlanError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidField(field) => write!(formatter, "invalid plan field: {field}"),
            Self::DuplicateTarget => formatter.write_str("plan target robot is duplicated"),
            Self::InvalidActivationOrder => {
                formatter.write_str("activation order must contain every target exactly once")
            }
            Self::ConflictingRevision => {
                formatter.write_str("plan revision identity has conflicting content")
            }
            Self::RevisionAlreadyActive => {
                formatter.write_str("an active revision cannot be replaced by prepare")
            }
            Self::NoRevision => formatter.write_str("there is no current plan revision"),
            Self::RevisionMismatch => {
                formatter.write_str("plan revision does not match current state")
            }
            Self::UnknownTarget => formatter.write_str("robot is outside the plan revision"),
            Self::InvalidTransition => formatter.write_str("invalid plan state transition"),
            Self::PrepareBarrierIncomplete => formatter.write_str("prepare barrier is incomplete"),
            Self::ActivationDependencyIncomplete => {
                formatter.write_str("prior activation lacks runtime confirmation")
            }
            Self::UnsafeActivationOrder => {
                formatter.write_str("target activation violates dependency order")
            }
            Self::TargetIdentityMismatch => {
                formatter.write_str("activation target identity does not match prepared content")
            }
            Self::RecoveryRequired => {
                formatter.write_str("plan is held pending recovery and replan")
            }
        }
    }
}

impl std::error::Error for PlanError {}

fn validate_id(value: &str, field: &'static str) -> Result<(), PlanError> {
    if value.is_empty() || value.len() > 128 || value.chars().any(char::is_control) {
        return Err(PlanError::InvalidField(field));
    }
    Ok(())
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}
