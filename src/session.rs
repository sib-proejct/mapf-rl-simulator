//! Core session fencing, reconciliation, Order idempotency, and report scheduling.

use crate::protocol::{
    ActiveController, CommandAckPayload, CommandDisposition, EventSeverity, MapIdentity,
    OrderCommand, OrderPhase, PoseReport, Producer, ProducerKind, ReportAck, ReportBatchOutcome,
    ReportEnvelope, ReportPayload, RobotEventPayload, RobotStatePayload, SimulatorSnapshot,
    StreamWelcome,
};
use crate::spool::{AckResult, AppliedOrder, DurableSpool, SpoolError};
use std::fmt;
use uuid::Uuid;

pub const STATE_REPORT_INTERVAL_MS: u64 = 100;
pub const REST_FALLBACK_INTERVAL_MS: u64 = 5_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SessionState {
    NotReady,
    Reconciling,
    Synchronized,
    Degraded,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StreamSequenceDisposition {
    Accepted,
    DuplicateOrStale,
    Gap,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OrderDecision {
    pub apply_to_robot: bool,
    pub disposition: CommandDisposition,
    pub code: &'static str,
}

pub struct CoreSession {
    simulator_id: String,
    robot_id: String,
    map: MapIdentity,
    controller: ActiveController,
    spool: DurableSpool,
    state: SessionState,
    session_epoch: Option<u64>,
    stream_id: Option<String>,
    last_stream_sequence: Option<u64>,
    next_state_report_ms: u64,
    next_rest_fallback_ms: Option<u64>,
}

impl CoreSession {
    pub fn new(
        simulator_id: String,
        robot_id: String,
        map: MapIdentity,
        controller: ActiveController,
        spool: DurableSpool,
    ) -> Result<Self, SessionError> {
        validate_id(&simulator_id, "simulatorId")?;
        validate_id(&robot_id, "robotId")?;
        map.validate()?;
        controller.validate()?;
        if spool.simulator_id() != simulator_id {
            return Err(SessionError::InvalidIdentity("spool simulatorId"));
        }
        Ok(Self {
            simulator_id,
            robot_id,
            map,
            controller,
            spool,
            state: SessionState::NotReady,
            session_epoch: None,
            stream_id: None,
            last_stream_sequence: None,
            next_state_report_ms: 0,
            next_rest_fallback_ms: None,
        })
    }

    pub const fn state(&self) -> SessionState {
        self.state
    }

    pub const fn session_epoch(&self) -> Option<u64> {
        self.session_epoch
    }

    pub const fn simulator_boot_id(&self) -> Uuid {
        self.spool.current_boot_id()
    }

    pub const fn controller(&self) -> &ActiveController {
        &self.controller
    }

    pub fn spool(&self) -> &DurableSpool {
        &self.spool
    }

    pub fn applied_order(&self) -> Option<&AppliedOrder> {
        self.spool.applied_order()
    }

    pub fn accept_welcome(&mut self, welcome: StreamWelcome) -> Result<(), SessionError> {
        welcome.validate()?;
        if welcome.stream_id != format!("simulator:{}", self.simulator_id) {
            return Err(SessionError::SnapshotMismatch);
        }
        let next_epoch = welcome.payload.session_epoch;
        if self
            .session_epoch
            .is_some_and(|current| next_epoch <= current)
        {
            return Err(SessionError::OldSession);
        }
        self.spool.rebind_current_boot(next_epoch)?;
        self.session_epoch = Some(next_epoch);
        self.stream_id = Some(welcome.stream_id);
        self.state = SessionState::Reconciling;
        self.next_rest_fallback_ms = None;
        Ok(())
    }

    pub fn reconcile(&mut self, snapshot: &SimulatorSnapshot) -> Result<(), SessionError> {
        snapshot.validate()?;
        let epoch = self.session_epoch.ok_or(SessionError::NoSession)?;
        if snapshot.session_epoch != epoch || snapshot.simulator_id != self.simulator_id {
            self.state = SessionState::NotReady;
            return Err(SessionError::SnapshotMismatch);
        }
        let robot = snapshot
            .robots
            .iter()
            .find(|robot| robot.robot_id == self.robot_id)
            .ok_or(SessionError::RobotBinding)?;
        if robot.map != self.map
            || (robot.active_controller != "unknown"
                && robot.active_controller != self.controller.identity)
        {
            self.state = SessionState::NotReady;
            return Err(SessionError::SnapshotMismatch);
        }
        if let Some(applied) = self.spool.applied_order()
            && (robot.order_id.as_deref() != Some(&applied.order_id)
                || robot.order_update_id != Some(applied.order_update_id))
        {
            self.state = SessionState::NotReady;
            return Err(SessionError::SnapshotMismatch);
        }
        if snapshot.stream_cursor.stream_id != self.stream_id.as_deref().unwrap_or("") {
            self.state = SessionState::NotReady;
            return Err(SessionError::SnapshotMismatch);
        }
        self.last_stream_sequence = Some(snapshot.stream_cursor.event_sequence);
        self.state = SessionState::Synchronized;
        Ok(())
    }

    pub fn observe_stream_sequence(&mut self, sequence: u64) -> StreamSequenceDisposition {
        let Some(last) = self.last_stream_sequence else {
            self.last_stream_sequence = Some(sequence);
            return StreamSequenceDisposition::Accepted;
        };
        if sequence <= last {
            return StreamSequenceDisposition::DuplicateOrStale;
        }
        if sequence == last + 1 {
            self.last_stream_sequence = Some(sequence);
            return StreamSequenceDisposition::Accepted;
        }
        self.state = SessionState::Degraded;
        StreamSequenceDisposition::Gap
    }

    /// Persists the application outcome before returning `apply_to_robot=true`.
    /// Duplicate delivery therefore never asks the engine to apply the Order twice.
    pub fn accept_order(
        &mut self,
        command: &OrderCommand,
        occurred_at: String,
    ) -> Result<OrderDecision, SessionError> {
        command.validate()?;
        let epoch = self.session_epoch.ok_or(SessionError::NoSession)?;
        if command.session_epoch != epoch {
            return Err(SessionError::OldSession);
        }
        if self.state != SessionState::Synchronized {
            return Err(SessionError::NotSynchronized);
        }
        if command.robot_id != self.robot_id
            || command.stream_id != self.stream_id.as_deref().unwrap_or("")
            || command.payload.map != self.map
        {
            return Err(SessionError::RobotBinding);
        }

        let decision = self.classify_order(command);
        let applied_order = decision.apply_to_robot.then(|| AppliedOrder {
            command_id: command.payload.command_id,
            order_id: command.payload.order_id.clone(),
            order_update_id: command.payload.order_update_id,
            content_digest_sha256: command.payload.content_digest_sha256.clone(),
            goal: command.payload.goal,
        });
        self.queue_command_ack(command, &decision, occurred_at, applied_order)?;
        Ok(decision)
    }

    pub fn state_report_due(&mut self, monotonic_ms: u64) -> bool {
        if monotonic_ms < self.next_state_report_ms {
            return false;
        }
        self.next_state_report_ms = self
            .next_state_report_ms
            .saturating_add(STATE_REPORT_INTERVAL_MS);
        if self.next_state_report_ms <= monotonic_ms {
            self.next_state_report_ms = monotonic_ms.saturating_add(STATE_REPORT_INTERVAL_MS);
        }
        true
    }

    pub fn queue_state_report(
        &mut self,
        state_version: u64,
        simulation_time_ms: i64,
        pose: PoseReport,
        occurred_at: String,
    ) -> Result<Uuid, SessionError> {
        if self.state != SessionState::Synchronized {
            return Err(SessionError::NotSynchronized);
        }
        let epoch = self.session_epoch.ok_or(SessionError::NoSession)?;
        let applied = self.spool.applied_order();
        let payload = RobotStatePayload {
            state_version,
            simulation_time_ms,
            pose,
            active_controller: self.controller.clone(),
            order_id: applied.map(|order| order.order_id.clone()),
            order_update_id: applied.map(|order| order.order_update_id),
        };
        let message_id = Uuid::new_v4();
        let report = ReportEnvelope {
            contract_version: crate::contracts::generated::CONTRACT_VERSION.to_owned(),
            message_id,
            message_type: "robot.state.report".to_owned(),
            producer: self.producer(),
            occurred_at,
            correlation_id: Uuid::new_v4(),
            request_id: None,
            session_epoch: epoch,
            simulator_id: self.simulator_id.clone(),
            simulator_boot_id: self.spool.current_boot_id(),
            report_sequence: self.spool.next_report_sequence(),
            robot_id: self.robot_id.clone(),
            payload: ReportPayload::State(payload),
        };
        self.spool.enqueue(report)?;
        Ok(message_id)
    }

    pub fn queue_order_completed(
        &mut self,
        simulation_time_ms: i64,
        occurred_at: String,
    ) -> Result<Uuid, SessionError> {
        if self.state != SessionState::Synchronized {
            return Err(SessionError::NotSynchronized);
        }
        let epoch = self.session_epoch.ok_or(SessionError::NoSession)?;
        let applied = self
            .spool
            .applied_order()
            .cloned()
            .ok_or(SessionError::NoAppliedOrder)?;
        let mut evidence = serde_json::Map::new();
        evidence.insert("orderId".to_owned(), applied.order_id.into());
        evidence.insert(
            "orderUpdateId".to_owned(),
            serde_json::Value::from(applied.order_update_id),
        );
        let message_id = Uuid::new_v4();
        let report = ReportEnvelope {
            contract_version: crate::contracts::generated::CONTRACT_VERSION.to_owned(),
            message_id,
            message_type: "robot.event.report".to_owned(),
            producer: self.producer(),
            occurred_at,
            correlation_id: Uuid::new_v4(),
            request_id: Some(Uuid::new_v4()),
            session_epoch: epoch,
            simulator_id: self.simulator_id.clone(),
            simulator_boot_id: self.spool.current_boot_id(),
            report_sequence: self.spool.next_report_sequence(),
            robot_id: self.robot_id.clone(),
            payload: ReportPayload::RobotEvent(RobotEventPayload {
                event_id: Uuid::new_v4(),
                severity: EventSeverity::Info,
                code: "ORDER_COMPLETED".to_owned(),
                simulation_time_ms,
                evidence,
            }),
        };
        self.spool.enqueue(report)?;
        Ok(message_id)
    }

    pub fn accept_report_ack(
        &mut self,
        acknowledgement: &ReportAck,
    ) -> Result<AckResult, SessionError> {
        acknowledgement.validate()?;
        let epoch = self.session_epoch.ok_or(SessionError::NoSession)?;
        if acknowledgement.session_epoch != epoch
            || acknowledgement.simulator_id != self.simulator_id
            || acknowledgement.stream_id != self.stream_id.as_deref().unwrap_or("")
        {
            return Err(SessionError::OldSession);
        }
        Ok(self
            .spool
            .acknowledge_ws(acknowledgement.correlation_id, &acknowledgement.payload)?)
    }

    pub fn accept_report_batch(
        &mut self,
        outcome: &ReportBatchOutcome,
    ) -> Result<Vec<AckResult>, SessionError> {
        let mut results = Vec::with_capacity(outcome.outcomes.len());
        for item in &outcome.outcomes {
            results.push(self.spool.acknowledge(
                item.report_message_id,
                item.report_sequence,
                item.disposition,
                item.retryable,
                &item.code,
            )?);
        }
        Ok(results)
    }

    pub fn mark_disconnected(&mut self, monotonic_ms: u64) {
        self.state = SessionState::Degraded;
        if self.next_rest_fallback_ms.is_none() {
            self.next_rest_fallback_ms =
                Some(monotonic_ms.saturating_add(REST_FALLBACK_INTERVAL_MS));
        }
    }

    pub fn rest_fallback_due(&mut self, monotonic_ms: u64) -> bool {
        let Some(due) = self.next_rest_fallback_ms else {
            return false;
        };
        if monotonic_ms < due {
            return false;
        }
        self.next_rest_fallback_ms = Some(monotonic_ms.saturating_add(REST_FALLBACK_INTERVAL_MS));
        true
    }

    pub fn resume_after(&self) -> Option<u64> {
        self.last_stream_sequence
    }

    fn classify_order(&self, command: &OrderCommand) -> OrderDecision {
        if command.payload.phase != OrderPhase::Activate {
            return OrderDecision {
                apply_to_robot: false,
                disposition: CommandDisposition::Rejected,
                code: "COMMAND_PHASE_UNSUPPORTED",
            };
        }
        let Some(current) = self.spool.applied_order() else {
            return if command.payload.order_update_id == 0 {
                OrderDecision {
                    apply_to_robot: true,
                    disposition: CommandDisposition::Applied,
                    code: "COMMAND_APPLIED",
                }
            } else {
                OrderDecision {
                    apply_to_robot: false,
                    disposition: CommandDisposition::Rejected,
                    code: "ORDER_UPDATE_GAP",
                }
            };
        };
        if command.payload.order_id != current.order_id {
            return OrderDecision {
                apply_to_robot: false,
                disposition: CommandDisposition::Rejected,
                code: "ACTIVE_ORDER_CONFLICT",
            };
        }
        if command.payload.order_update_id == current.order_update_id {
            return if command.payload.content_digest_sha256 == current.content_digest_sha256 {
                OrderDecision {
                    apply_to_robot: false,
                    disposition: CommandDisposition::Duplicate,
                    code: "COMMAND_DUPLICATE",
                }
            } else {
                OrderDecision {
                    apply_to_robot: false,
                    disposition: CommandDisposition::Rejected,
                    code: "ORDER_UPDATE_CONFLICT",
                }
            };
        }
        if command.payload.order_update_id < current.order_update_id {
            return OrderDecision {
                apply_to_robot: false,
                disposition: CommandDisposition::Duplicate,
                code: "ORDER_UPDATE_STALE",
            };
        }
        if command.payload.order_update_id != current.order_update_id + 1 {
            return OrderDecision {
                apply_to_robot: false,
                disposition: CommandDisposition::Rejected,
                code: "ORDER_UPDATE_GAP",
            };
        }
        OrderDecision {
            apply_to_robot: true,
            disposition: CommandDisposition::Applied,
            code: "COMMAND_APPLIED",
        }
    }

    fn queue_command_ack(
        &mut self,
        command: &OrderCommand,
        decision: &OrderDecision,
        occurred_at: String,
        applied_order: Option<AppliedOrder>,
    ) -> Result<(), SessionError> {
        let epoch = self.session_epoch.ok_or(SessionError::NoSession)?;
        let report = ReportEnvelope {
            contract_version: crate::contracts::generated::CONTRACT_VERSION.to_owned(),
            message_id: Uuid::new_v4(),
            message_type: "command.ack".to_owned(),
            producer: self.producer(),
            occurred_at,
            correlation_id: command.correlation_id,
            request_id: Some(Uuid::new_v4()),
            session_epoch: epoch,
            simulator_id: self.simulator_id.clone(),
            simulator_boot_id: self.spool.current_boot_id(),
            report_sequence: self.spool.next_report_sequence(),
            robot_id: self.robot_id.clone(),
            payload: ReportPayload::CommandAck(CommandAckPayload {
                command_id: command.payload.command_id,
                disposition: decision.disposition,
                order_id: command.payload.order_id.clone(),
                order_update_id: command.payload.order_update_id,
                content_digest_sha256: command.payload.content_digest_sha256.clone(),
                code: Some(decision.code.to_owned()),
            }),
        };
        self.spool.enqueue_with_order(report, applied_order)?;
        Ok(())
    }

    fn producer(&self) -> Producer {
        Producer {
            kind: ProducerKind::Simulator,
            id: self.simulator_id.clone(),
        }
    }
}

#[derive(Debug)]
pub enum SessionError {
    Protocol(crate::protocol::ProtocolError),
    Spool(SpoolError),
    InvalidIdentity(&'static str),
    NoSession,
    NotSynchronized,
    OldSession,
    RobotBinding,
    SnapshotMismatch,
    NoAppliedOrder,
}

impl From<crate::protocol::ProtocolError> for SessionError {
    fn from(value: crate::protocol::ProtocolError) -> Self {
        Self::Protocol(value)
    }
}

impl From<SpoolError> for SessionError {
    fn from(value: SpoolError) -> Self {
        Self::Spool(value)
    }
}

impl fmt::Display for SessionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Protocol(error) => error.fmt(formatter),
            Self::Spool(error) => error.fmt(formatter),
            Self::InvalidIdentity(field) => write!(formatter, "invalid {field}"),
            Self::NoSession => formatter.write_str("Core session has no active epoch"),
            Self::NotSynchronized => formatter.write_str("Core session is not synchronized"),
            Self::OldSession => formatter.write_str("message belongs to a fenced session"),
            Self::RobotBinding => {
                formatter.write_str("message is outside the credential robot binding")
            }
            Self::SnapshotMismatch => {
                formatter.write_str("Core snapshot does not reconcile with local state")
            }
            Self::NoAppliedOrder => formatter.write_str("there is no applied Order to complete"),
        }
    }
}

impl std::error::Error for SessionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Protocol(error) => Some(error),
            Self::Spool(error) => Some(error),
            _ => None,
        }
    }
}

fn validate_id(value: &str, field: &'static str) -> Result<(), SessionError> {
    if value.is_empty() || value.len() > 128 || value.chars().any(char::is_control) {
        return Err(SessionError::InvalidIdentity(field));
    }
    Ok(())
}
