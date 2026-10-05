//! Async Phase 2 transport/session composition.

use crate::core_client::{CoreClient, CoreClientError, CoreWebSocket};
use crate::protocol::{
    EventSeverity, OrderCommand, PoseReport, ReportAck, ReportEnvelope, SimulatorSnapshot,
    StreamWelcome,
};
use crate::session::{
    CoreSession, OrderDecision, SessionError, SessionState, StreamSequenceDisposition,
};
use crate::spool::{AppliedOrder, PreparedOrder};
use std::fmt;
use std::sync::{Arc, Mutex};

pub struct Phase2Runtime {
    client: CoreClient,
    session: Arc<Mutex<CoreSession>>,
    websocket: Option<CoreWebSocket>,
    telemetry: Option<crate::telemetry::TelemetryTransport>,
    telemetry_sequence: u64,
    durable_fingerprint: Option<serde_json::Value>,
    durable_state_version: Option<u64>,
}

pub struct IncidentReport {
    pub event_id: uuid::Uuid,
    pub severity: EventSeverity,
    pub code: String,
    pub simulation_time_ms: i64,
    pub evidence: serde_json::Map<String, serde_json::Value>,
    pub occurred_at: String,
}

impl Phase2Runtime {
    /// Reconnect off the fleet tick. The shared durable session is held for motion
    /// until the caller adopts the completed worker and its new socket.
    pub fn reconnect_task(
        &self,
        monotonic_ms: u64,
    ) -> tokio::task::JoinHandle<Result<Self, RuntimeError>> {
        let mut worker = Self {
            client: self.client.clone(),
            session: Arc::clone(&self.session),
            websocket: None,
            telemetry: None,
            telemetry_sequence: 0,
            durable_fingerprint: None,
            durable_state_version: None,
        };
        tokio::spawn(async move {
            worker.connect_and_replay(monotonic_ms).await?;
            Ok(worker)
        })
    }

    pub fn new(client: CoreClient, session: CoreSession) -> Self {
        Self {
            client,
            session: Arc::new(Mutex::new(session)),
            websocket: None,
            telemetry: None,
            telemetry_sequence: 0,
            durable_fingerprint: None,
            durable_state_version: None,
        }
    }

    /// Authenticates the WebSocket, consumes the new epoch, reconciles through
    /// authenticated REST, then replays every report still in the local spool.
    pub async fn connect_and_replay(
        &mut self,
        monotonic_ms: u64,
    ) -> Result<SimulatorSnapshot, RuntimeError> {
        match self.connect_inner().await {
            Ok(snapshot) => Ok(snapshot),
            Err(error) => {
                self.websocket = None;
                self.mark_disconnected(monotonic_ms).await?;
                Err(error)
            }
        }
    }

    async fn connect_inner(&mut self) -> Result<SimulatorSnapshot, RuntimeError> {
        let pending = self
            .session_call(|session| Ok(session.spool().pending().to_vec()))
            .await?;
        if pending.len() > 100 {
            // Drain large recovery queues before opening WS, so recovery cannot
            // block WS acks or leave its ping/pong exchange unattended.
            for reports in pending.chunks(100) {
                let outcome = self.client.submit_report_batch(reports).await?;
                self.session_call(move |session| {
                    session.accept_report_batch(&outcome)?;
                    Ok(())
                })
                .await?;
            }
        }
        let resume_after = self
            .session_call(|session| Ok(session.resume_after()))
            .await?;
        self.client.require_occupancy_control().await?;
        let mut websocket = self.client.connect_websocket(resume_after).await?;
        let welcome_value = websocket.next_json().await?;
        let welcome: StreamWelcome = serde_json::from_value(welcome_value)?;
        self.session_call(move |session| Ok(session.accept_welcome(welcome)?))
            .await?;

        let snapshot = self.client.fetch_snapshot().await?;
        let reconciliation_snapshot = snapshot.clone();
        self.session_call(move |session| Ok(session.reconcile(&reconciliation_snapshot)?))
            .await?;
        let pending = self
            .session_call(|session| Ok(session.spool().pending().to_vec()))
            .await?;
        for report in &pending {
            websocket.write_report(report).await?;
        }
        self.websocket = Some(websocket);
        if self.client.telemetry_supported().await? {
            self.telemetry = Some(crate::telemetry::TelemetryTransport::start(
                self.client.clone(),
            ));
        }
        Ok(snapshot)
    }

    /// Queues one periodic state projection if the injected monotonic schedule is
    /// due, then writes it on the WS fast path. The durable copy remains pending.
    pub async fn publish_state_if_due(
        &mut self,
        monotonic_ms: u64,
        state_version: u64,
        simulation_time_ms: i64,
        pose: PoseReport,
        occurred_at: String,
    ) -> Result<bool, RuntimeError> {
        self.publish_state(
            monotonic_ms,
            state_version,
            simulation_time_ms,
            pose,
            occurred_at,
            false,
        )
        .await
    }

    /// Publish current physical evidence before a critical lifecycle transition.
    pub async fn publish_state_now(
        &mut self,
        monotonic_ms: u64,
        state_version: u64,
        simulation_time_ms: i64,
        pose: PoseReport,
        occurred_at: String,
    ) -> Result<bool, RuntimeError> {
        self.publish_state(
            monotonic_ms,
            state_version,
            simulation_time_ms,
            pose,
            occurred_at,
            true,
        )
        .await
    }

    async fn publish_state(
        &mut self,
        monotonic_ms: u64,
        state_version: u64,
        simulation_time_ms: i64,
        pose: PoseReport,
        occurred_at: String,
        force: bool,
    ) -> Result<bool, RuntimeError> {
        if force && self.durable_state_version == Some(state_version) {
            return Ok(false);
        }
        let report = self
            .session_call(move |session| {
                if !force && !session.state_report_due(monotonic_ms) {
                    return Ok(None);
                }
                Ok(Some(session.build_state_report(
                    state_version,
                    simulation_time_ms,
                    pose,
                    occurred_at,
                )?))
            })
            .await?;
        let Some(report) = report else {
            return Ok(false);
        };
        let payload = serde_json::to_value(&report.payload)?;
        let fingerprint = crate::telemetry::durable_fingerprint(&payload);
        if self.telemetry.is_none()
            || force
            || self.durable_fingerprint.as_ref() != Some(&fingerprint)
        {
            let queued = report.clone();
            self.session_call(move |session| Ok(session.enqueue_state_report(queued)?))
                .await?;
            let Some(socket) = self.websocket.as_mut() else {
                return Ok(true);
            };
            if let Err(error) = socket.write_report(&report).await {
                self.websocket = None;
                self.mark_disconnected(monotonic_ms).await?;
                return Err(error.into());
            }
            self.durable_state_version = Some(state_version);
            self.durable_fingerprint = Some(fingerprint);
        }
        if let Some(transport) = &self.telemetry {
            let frame = serde_json::json!({
                "telemetryVersion": crate::contracts::generated::TELEMETRY_VERSION,
                "messageType": "telemetry.report", "simulatorId": report.simulator_id,
                "robotId": report.robot_id, "sessionEpoch": report.session_epoch,
                "simulatorBootId": report.simulator_boot_id,
                "telemetrySequence": self.telemetry_sequence,
                "occurredAt": report.occurred_at, "payload": payload,
            });
            self.telemetry_sequence += 1;
            transport.publish(frame);
        }
        Ok(true)
    }

    pub async fn set_station_state(
        &mut self,
        state: crate::station::StationState,
        safe: bool,
    ) -> Result<(), RuntimeError> {
        self.session_call(move |session| {
            session.set_station_state(state, safe);
            Ok(())
        })
        .await
    }
    pub async fn set_traffic_wait(
        &mut self,
        wait: Option<crate::contracts::provisioning_generated::TrafficWait>,
    ) -> Result<(), RuntimeError> {
        self.session_call(move |session| {
            session.set_traffic_wait(wait);
            Ok(())
        })
        .await
    }
    pub async fn synchronized(&self) -> Result<bool, RuntimeError> {
        let synchronized = self
            .session_call(|session| {
                Ok(session.state() == crate::session::SessionState::Synchronized)
            })
            .await?;
        Ok(synchronized
            && self
                .telemetry
                .as_ref()
                .is_none_or(|transport| transport.ready()))
    }

    pub async fn publish_order_completed(
        &mut self,
        simulation_time_ms: i64,
        occurred_at: String,
        monotonic_ms: u64,
    ) -> Result<(), RuntimeError> {
        let report = self
            .session_call(move |session| {
                let message_id = session.queue_order_completed(simulation_time_ms, occurred_at)?;
                session
                    .spool()
                    .pending()
                    .iter()
                    .find(|report| report.message_id == message_id)
                    .cloned()
                    .ok_or(RuntimeError::MissingCompletionReport)
            })
            .await?;
        if let Err(error) = self.write_without_accepting(&report).await {
            self.websocket = None;
            self.mark_disconnected(monotonic_ms).await?;
            return Err(error);
        }
        Ok(())
    }

    pub async fn publish_incident(
        &mut self,
        incident: IncidentReport,
        monotonic_ms: u64,
    ) -> Result<(), RuntimeError> {
        let report = self
            .session_call(move |session| {
                let message_id = session.queue_incident(
                    incident.event_id,
                    incident.severity,
                    incident.code,
                    incident.simulation_time_ms,
                    incident.evidence,
                    incident.occurred_at,
                )?;
                session
                    .spool()
                    .pending()
                    .iter()
                    .find(|report| report.message_id == message_id)
                    .cloned()
                    .ok_or(RuntimeError::MissingCompletionReport)
            })
            .await?;
        if let Err(error) = self.write_without_accepting(&report).await {
            self.websocket = None;
            self.mark_disconnected(monotonic_ms).await?;
            return Err(error);
        }
        Ok(())
    }

    pub async fn applied_order(&self) -> Result<Option<AppliedOrder>, RuntimeError> {
        self.session_call(|session| Ok(session.applied_order().cloned()))
            .await
    }

    pub async fn prepared_order(&self) -> Result<Option<PreparedOrder>, RuntimeError> {
        self.session_call(|session| Ok(session.prepared_order().cloned()))
            .await
    }

    pub async fn abort_pending(&self) -> Result<bool, RuntimeError> {
        self.session_call(|session| Ok(session.spool().pending_abort().is_some()))
            .await
    }

    /// Called only after a stopped physical state has been checkpointed.
    pub async fn finish_abort_if_stopped(
        &mut self,
        occurred_at: String,
    ) -> Result<(), RuntimeError> {
        let report = self
            .session_call(move |session| {
                if session.state() != SessionState::Synchronized {
                    return Ok(None);
                }
                let Some(command) = session.spool().pending_abort().cloned() else {
                    return Ok(None);
                };
                // Reconnection must obtain a command rebound to the current epoch.
                if Some(command.session_epoch) != session.session_epoch() {
                    return Ok(None);
                }
                session.accept_order_at(&command, occurred_at, 0, true)?;
                Ok(session.spool().pending().last().cloned())
            })
            .await?;
        if let Some(report) = report {
            self.write_without_accepting(&report).await?;
        }
        Ok(())
    }

    /// Handles one Core frame. Command application and its durable application ack
    /// happen before `OrderApplied` is returned; execution is reported only by a
    /// later periodic state report.
    pub async fn receive_one(
        &mut self,
        monotonic_ms: u64,
        occurred_at: String,
    ) -> Result<RuntimeEvent, RuntimeError> {
        self.receive_one_with_motion(monotonic_ms, occurred_at, false)
            .await
    }

    pub async fn receive_one_with_motion(
        &mut self,
        monotonic_ms: u64,
        occurred_at: String,
        robot_is_stationary: bool,
    ) -> Result<RuntimeEvent, RuntimeError> {
        let next = self
            .websocket
            .as_mut()
            .ok_or(RuntimeError::Disconnected)?
            .next_json()
            .await;
        let value = match next {
            Ok(value) => value,
            Err(error) => {
                self.websocket = None;
                self.mark_disconnected(monotonic_ms).await?;
                return Err(error.into());
            }
        };
        self.accept_received(value, monotonic_ms, occurred_at, robot_is_stationary)
            .await
    }

    /// Timeout only the cancellation-safe socket read, never durable message handling.
    pub async fn poll_one_with_motion(
        &mut self,
        monotonic_ms: u64,
        occurred_at: String,
        robot_is_stationary: bool,
    ) -> Result<Option<RuntimeEvent>, RuntimeError> {
        let websocket = self.websocket.as_mut().ok_or(RuntimeError::Disconnected)?;
        match websocket.poll_json().await {
            Ok(None) => Ok(None),
            Ok(Some(value)) => self
                .accept_received(value, monotonic_ms, occurred_at, robot_is_stationary)
                .await
                .map(Some),
            Err(error) => {
                self.websocket = None;
                self.mark_disconnected(monotonic_ms).await?;
                Err(error.into())
            }
        }
    }

    async fn accept_received(
        &mut self,
        value: serde_json::Value,
        monotonic_ms: u64,
        occurred_at: String,
        robot_is_stationary: bool,
    ) -> Result<RuntimeEvent, RuntimeError> {
        let message_type = value
            .get("messageType")
            .and_then(serde_json::Value::as_str)
            .ok_or(RuntimeError::InvalidCoreMessage)?;
        match message_type {
            "order.command" => {
                let command: OrderCommand = serde_json::from_value(value)?;
                let event_command = command.clone();
                let sequence = command.event_sequence;
                let (stream, decision, acknowledgement) = self
                    .session_call(move |session| {
                        let stream = session.observe_stream_sequence(sequence);
                        if stream == StreamSequenceDisposition::Gap {
                            return Ok((stream, None, None));
                        }
                        let decision = session.accept_order_at(
                            &command,
                            occurred_at,
                            monotonic_ms,
                            robot_is_stationary,
                        )?;
                        if decision.code == "PLAN_ABORT_STOPPING" {
                            return Ok((stream, Some(decision), None));
                        }
                        let acknowledgement = session
                            .spool()
                            .pending()
                            .last()
                            .cloned()
                            .ok_or(RuntimeError::MissingApplicationAck)?;
                        Ok((stream, Some(decision), Some(acknowledgement)))
                    })
                    .await?;
                if stream == StreamSequenceDisposition::Gap {
                    return Err(RuntimeError::StreamGap);
                }
                let decision = decision.ok_or(RuntimeError::MissingApplicationAck)?;
                if let Some(acknowledgement) = acknowledgement
                    && let Err(error) = self.write_without_accepting(&acknowledgement).await
                {
                    self.websocket = None;
                    self.mark_disconnected(monotonic_ms).await?;
                    return Err(error);
                }
                Ok(RuntimeEvent::Order {
                    decision,
                    command: Box::new(event_command),
                })
            }
            "report.ack" => {
                let acknowledgement: ReportAck = serde_json::from_value(value)?;
                let sequence = acknowledgement.event_sequence;
                let (stream, result) = self
                    .session_call(move |session| {
                        let stream = session.observe_stream_sequence(sequence);
                        if stream == StreamSequenceDisposition::Gap {
                            return Ok((stream, None));
                        }
                        if session
                            .session_epoch()
                            .is_some_and(|epoch| acknowledgement.session_epoch < epoch)
                        {
                            // A resumed stream can replay acknowledgements from a fenced session.
                            // Their report remains spooled until a current-session ack or REST outcome.
                            return Ok((stream, Some(crate::spool::AckResult::Unknown)));
                        }
                        let result = session.accept_report_ack(&acknowledgement)?;
                        Ok((stream, Some(result)))
                    })
                    .await?;
                if stream == StreamSequenceDisposition::Gap {
                    return Err(RuntimeError::StreamGap);
                }
                Ok(RuntimeEvent::ReportAcknowledged(
                    result.ok_or(RuntimeError::InvalidCoreMessage)?,
                ))
            }
            "stream.welcome" => Err(RuntimeError::UnexpectedWelcome),
            _ => Err(RuntimeError::InvalidCoreMessage),
        }
    }

    pub async fn mark_disconnected(&self, monotonic_ms: u64) -> Result<(), RuntimeError> {
        self.session_call(move |session| {
            session.mark_disconnected(monotonic_ms);
            Ok(())
        })
        .await
    }

    /// Five-second REST recovery path. It validates the authenticated snapshot and
    /// submits bounded batches, applying the same Core acceptance rules as WS acks.
    /// It never substitutes REST for 5 Hz telemetry generation.
    pub async fn run_rest_fallback_if_due(&self, monotonic_ms: u64) -> Result<bool, RuntimeError> {
        let due = self
            .session_call(move |session| Ok(session.rest_fallback_due(monotonic_ms)))
            .await?;
        if !due {
            return Ok(false);
        }
        let snapshot = self.client.fetch_snapshot().await?;
        let (simulator_id, epoch) = self
            .session_call(|session| {
                Ok((
                    session
                        .spool()
                        .pending()
                        .first()
                        .map(|report| report.simulator_id.clone()),
                    session.session_epoch(),
                ))
            })
            .await?;
        if simulator_id
            .as_deref()
            .is_some_and(|id| id != snapshot.simulator_id)
            || epoch != Some(snapshot.session_epoch)
        {
            return Err(RuntimeError::SnapshotMismatch);
        }

        let pending = self
            .session_call(|session| Ok(session.spool().pending().to_vec()))
            .await?;
        for reports in pending.chunks(100) {
            let outcome = self.client.submit_report_batch(reports).await?;
            self.session_call(move |session| {
                session.accept_report_batch(&outcome)?;
                Ok(())
            })
            .await?;
        }
        Ok(true)
    }

    async fn write_without_accepting(
        &mut self,
        report: &ReportEnvelope,
    ) -> Result<(), RuntimeError> {
        let websocket = self.websocket.as_mut().ok_or(RuntimeError::Disconnected)?;
        websocket.write_report(report).await?;
        Ok(())
    }

    async fn session_call<T, F>(&self, operation: F) -> Result<T, RuntimeError>
    where
        T: Send + 'static,
        F: FnOnce(&mut CoreSession) -> Result<T, RuntimeError> + Send + 'static,
    {
        let session = Arc::clone(&self.session);
        tokio::task::spawn_blocking(move || {
            let mut session = session
                .lock()
                .map_err(|_| RuntimeError::SessionWorkerPanicked)?;
            operation(&mut session)
        })
        .await
        .map_err(|_| RuntimeError::SessionWorkerPanicked)?
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum RuntimeEvent {
    Order {
        decision: OrderDecision,
        command: Box<OrderCommand>,
    },
    ReportAcknowledged(crate::spool::AckResult),
}

#[derive(Debug)]
pub enum RuntimeError {
    Core(CoreClientError),
    Session(SessionError),
    Json(serde_json::Error),
    Disconnected,
    InvalidCoreMessage,
    MissingApplicationAck,
    MissingCompletionReport,
    SessionWorkerPanicked,
    SnapshotMismatch,
    StreamGap,
    UnexpectedWelcome,
}

impl From<CoreClientError> for RuntimeError {
    fn from(value: CoreClientError) -> Self {
        Self::Core(value)
    }
}

impl From<SessionError> for RuntimeError {
    fn from(value: SessionError) -> Self {
        Self::Session(value)
    }
}

impl From<serde_json::Error> for RuntimeError {
    fn from(value: serde_json::Error) -> Self {
        Self::Json(value)
    }
}

impl fmt::Display for RuntimeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Core(error) => error.fmt(formatter),
            Self::Session(error) => error.fmt(formatter),
            Self::Json(error) => write!(formatter, "Core message JSON failed: {error}"),
            Self::Disconnected => formatter.write_str("Core WebSocket is not connected"),
            Self::InvalidCoreMessage => formatter.write_str("Core sent an unsupported message"),
            Self::MissingApplicationAck => {
                formatter.write_str("application ack was not durably spooled")
            }
            Self::MissingCompletionReport => {
                formatter.write_str("completion report was not durably spooled")
            }
            Self::SessionWorkerPanicked => formatter.write_str("blocking session worker failed"),
            Self::SnapshotMismatch => {
                formatter.write_str("REST fallback snapshot does not match the active session")
            }
            Self::StreamGap => {
                formatter.write_str("Core stream sequence has a gap; reconciliation is required")
            }
            Self::UnexpectedWelcome => {
                formatter.write_str("Core sent a second welcome in one WebSocket session")
            }
        }
    }
}

impl std::error::Error for RuntimeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Core(error) => Some(error),
            Self::Session(error) => Some(error),
            Self::Json(error) => Some(error),
            _ => None,
        }
    }
}
