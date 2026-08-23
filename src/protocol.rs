//! Typed consumer boundary for the Core-owned contract `1.0.0`.

use crate::contracts::generated::CONTRACT_VERSION;
use serde::{Deserialize, Serialize};
use std::fmt;
use uuid::Uuid;

pub const MAX_WS_MESSAGE_BYTES: usize = 1024 * 1024;
pub const HEARTBEAT_INTERVAL_SECONDS: u64 = 5;
pub const RESUME_RETENTION_SECONDS: u64 = 900;
pub const RESUME_MAX_EVENTS: u64 = 10_000;

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Producer {
    pub kind: ProducerKind,
    pub id: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ProducerKind {
    Core,
    Simulator,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct MapIdentity {
    pub map_id: Uuid,
    pub revision: u64,
    pub content_digest_sha256: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct MapCoordinateFrame {
    pub name: String,
    pub handedness: String,
    pub x_axis: String,
    pub y_axis: String,
    pub z_axis: String,
    pub yaw: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct MapOrigin {
    pub x_meters: f64,
    pub y_meters: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct RasterMapContent {
    pub contract_version: String,
    pub map_id: Uuid,
    pub revision: u64,
    pub content_digest_sha256: String,
    pub coordinate_frame: MapCoordinateFrame,
    pub origin: MapOrigin,
    pub resolution_meters: f64,
    pub width_cells: u32,
    pub height_cells: u32,
    pub cells: Vec<u8>,
}

impl RasterMapContent {
    pub fn identity(&self) -> MapIdentity {
        MapIdentity {
            map_id: self.map_id,
            revision: self.revision,
            content_digest_sha256: self.content_digest_sha256.clone(),
        }
    }

    pub fn validate(&self) -> Result<(), ProtocolError> {
        self.identity().validate()?;
        if self.contract_version != CONTRACT_VERSION
            || self.coordinate_frame.name != "map"
            || self.coordinate_frame.handedness != "RIGHT_HANDED"
            || self.coordinate_frame.x_axis != "EAST"
            || self.coordinate_frame.y_axis != "NORTH"
            || self.coordinate_frame.z_axis != "UP"
            || self.coordinate_frame.yaw != "COUNTERCLOCKWISE_FROM_POSITIVE_X_RADIANS"
            || !self.origin.x_meters.is_finite()
            || !self.origin.y_meters.is_finite()
            || !self.resolution_meters.is_finite()
            || self.resolution_meters <= 0.0
            || self.width_cells == 0
            || self.height_cells == 0
            || self.cells.iter().any(|cell| *cell > 1)
            || usize::try_from(self.width_cells).ok().and_then(|width| {
                usize::try_from(self.height_cells)
                    .ok()
                    .and_then(|height| width.checked_mul(height))
            }) != Some(self.cells.len())
        {
            return Err(ProtocolError::InvalidField("mapContent"));
        }
        Ok(())
    }
}

impl MapIdentity {
    pub fn validate(&self) -> Result<(), ProtocolError> {
        ensure_uuid_v4(self.map_id, "mapId")?;
        ensure_sha256(&self.content_digest_sha256, "map content digest")
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct StreamWelcomePayload {
    pub resume_retention_seconds: u64,
    pub resume_max_events: u64,
    pub heartbeat_interval_seconds: u64,
    pub session_epoch: u64,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct StreamWelcome {
    pub contract_version: String,
    pub message_id: Uuid,
    pub message_type: String,
    pub producer: Producer,
    pub occurred_at: String,
    pub correlation_id: Uuid,
    pub stream_id: String,
    pub payload: StreamWelcomePayload,
}

impl StreamWelcome {
    pub fn validate(&self) -> Result<(), ProtocolError> {
        ensure_timestamp(&self.occurred_at)?;
        validate_core_envelope(
            &self.contract_version,
            self.message_id,
            &self.message_type,
            "stream.welcome",
            &self.producer,
            self.correlation_id,
        )?;
        ensure_id(&self.stream_id, 160, "streamId")?;
        if self.payload.session_epoch == 0 {
            return Err(ProtocolError::InvalidField("sessionEpoch"));
        }
        if self.payload.resume_retention_seconds != RESUME_RETENTION_SECONDS
            || self.payload.resume_max_events != RESUME_MAX_EVENTS
            || self.payload.heartbeat_interval_seconds != HEARTBEAT_INTERVAL_SECONDS
        {
            return Err(ProtocolError::IncompatibleCapability);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum OrderPhase {
    Prepare,
    Activate,
    Abort,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct OrderGoal {
    pub column: u32,
    pub row: u32,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct OrderCommandPayload {
    pub command_id: Uuid,
    pub order_id: String,
    pub order_update_id: u64,
    pub content_digest_sha256: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plan_revision_id: Option<Uuid>,
    pub phase: OrderPhase,
    pub map: MapIdentity,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub goal: Option<OrderGoal>,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct OrderCommand {
    pub contract_version: String,
    pub message_id: Uuid,
    pub message_type: String,
    pub producer: Producer,
    pub occurred_at: String,
    pub correlation_id: Uuid,
    pub stream_id: String,
    pub event_sequence: u64,
    pub session_epoch: u64,
    pub robot_id: String,
    pub payload: OrderCommandPayload,
}

impl OrderCommand {
    pub fn validate(&self) -> Result<(), ProtocolError> {
        ensure_timestamp(&self.occurred_at)?;
        validate_core_envelope(
            &self.contract_version,
            self.message_id,
            &self.message_type,
            "order.command",
            &self.producer,
            self.correlation_id,
        )?;
        ensure_id(&self.stream_id, 160, "streamId")?;
        ensure_id(&self.robot_id, 128, "robotId")?;
        if self.session_epoch == 0 {
            return Err(ProtocolError::InvalidField("sessionEpoch"));
        }
        ensure_uuid_v4(self.payload.command_id, "commandId")?;
        ensure_id(&self.payload.order_id, 128, "orderId")?;
        ensure_sha256(&self.payload.content_digest_sha256, "Order content digest")?;
        if let Some(plan_revision_id) = self.payload.plan_revision_id {
            ensure_uuid_v4(plan_revision_id, "planRevisionId")?;
        }
        self.payload.map.validate()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ControllerMode {
    Baseline,
    Policy,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ActiveController {
    pub mode: ControllerMode,
    pub identity: String,
    pub content_digest_sha256: String,
}

impl ActiveController {
    pub fn validate(&self) -> Result<(), ProtocolError> {
        ensure_id(&self.identity, 128, "controller identity")?;
        ensure_sha256(&self.content_digest_sha256, "controller content digest")
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PoseReport {
    pub x_meters: f64,
    pub y_meters: f64,
    pub yaw_radians: f64,
}

impl PoseReport {
    pub fn validate(self) -> Result<(), ProtocolError> {
        if !self.x_meters.is_finite()
            || !self.y_meters.is_finite()
            || !self.yaw_radians.is_finite()
            || !(-std::f64::consts::PI..=std::f64::consts::PI).contains(&self.yaw_radians)
        {
            return Err(ProtocolError::InvalidField("pose"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct RobotStatePayload {
    pub state_version: u64,
    pub simulation_time_ms: i64,
    pub pose: PoseReport,
    pub active_controller: ActiveController,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub order_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub order_update_id: Option<u64>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CommandDisposition {
    Prepared,
    Applied,
    Rejected,
    Duplicate,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct CommandAckPayload {
    pub command_id: Uuid,
    pub disposition: CommandDisposition,
    pub order_id: String,
    pub order_update_id: u64,
    pub content_digest_sha256: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EventSeverity {
    Info,
    Warning,
    Critical,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct RobotEventPayload {
    pub event_id: Uuid,
    pub severity: EventSeverity,
    pub code: String,
    pub simulation_time_ms: i64,
    pub evidence: serde_json::Map<String, serde_json::Value>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ReportPayload {
    State(RobotStatePayload),
    CommandAck(CommandAckPayload),
    RobotEvent(RobotEventPayload),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ReportEnvelope {
    pub contract_version: String,
    pub message_id: Uuid,
    pub message_type: String,
    pub producer: Producer,
    pub occurred_at: String,
    pub correlation_id: Uuid,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub request_id: Option<Uuid>,
    pub session_epoch: u64,
    pub simulator_id: String,
    pub simulator_boot_id: Uuid,
    pub report_sequence: u64,
    pub robot_id: String,
    pub payload: ReportPayload,
}

impl ReportEnvelope {
    pub fn validate(&self) -> Result<(), ProtocolError> {
        ensure_timestamp(&self.occurred_at)?;
        if self.contract_version != CONTRACT_VERSION {
            return Err(ProtocolError::ContractVersion);
        }
        ensure_uuid_v4(self.message_id, "messageId")?;
        ensure_uuid_v4(self.correlation_id, "correlationId")?;
        ensure_uuid_v4(self.simulator_boot_id, "simulatorBootId")?;
        ensure_id(&self.simulator_id, 128, "simulatorId")?;
        ensure_id(&self.robot_id, 128, "robotId")?;
        if self.session_epoch == 0
            || self.producer
                != (Producer {
                    kind: ProducerKind::Simulator,
                    id: self.simulator_id.clone(),
                })
        {
            return Err(ProtocolError::InvalidEnvelope);
        }
        match (&self.message_type[..], &self.payload, self.request_id) {
            ("robot.state.report", ReportPayload::State(payload), None) => {
                if payload.simulation_time_ms < 0
                    || payload.order_id.is_some() != payload.order_update_id.is_some()
                {
                    return Err(ProtocolError::InvalidField("state payload"));
                }
                if let Some(order_id) = &payload.order_id {
                    ensure_id(order_id, 128, "orderId")?;
                }
                payload.pose.validate()?;
                payload.active_controller.validate()
            }
            ("command.ack", ReportPayload::CommandAck(payload), Some(request_id)) => {
                ensure_uuid_v4(request_id, "requestId")?;
                ensure_uuid_v4(payload.command_id, "commandId")?;
                ensure_id(&payload.order_id, 128, "orderId")?;
                ensure_sha256(&payload.content_digest_sha256, "Order content digest")?;
                if let Some(code) = &payload.code {
                    ensure_code(code)?;
                }
                Ok(())
            }
            ("robot.event.report", ReportPayload::RobotEvent(payload), Some(request_id)) => {
                ensure_uuid_v4(request_id, "requestId")?;
                ensure_uuid_v4(payload.event_id, "eventId")?;
                ensure_code(&payload.code)?;
                if payload.simulation_time_ms < 0 {
                    return Err(ProtocolError::InvalidField("simulationTimeMs"));
                }
                Ok(())
            }
            _ => Err(ProtocolError::InvalidEnvelope),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ReportDisposition {
    Accepted,
    Duplicate,
    Rejected,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ReportDurability {
    Durable,
    Projection,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ReportAckPayload {
    pub report_message_id: Uuid,
    pub simulator_boot_id: Uuid,
    pub report_sequence: u64,
    pub disposition: ReportDisposition,
    pub durability: ReportDurability,
    pub retryable: bool,
    pub code: String,
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ReportAck {
    pub contract_version: String,
    pub message_id: Uuid,
    pub message_type: String,
    pub producer: Producer,
    pub occurred_at: String,
    pub correlation_id: Uuid,
    pub stream_id: String,
    pub event_sequence: u64,
    pub session_epoch: u64,
    pub simulator_id: String,
    pub payload: ReportAckPayload,
}

impl ReportAck {
    pub fn validate(&self) -> Result<(), ProtocolError> {
        ensure_timestamp(&self.occurred_at)?;
        validate_core_envelope(
            &self.contract_version,
            self.message_id,
            &self.message_type,
            "report.ack",
            &self.producer,
            self.correlation_id,
        )?;
        ensure_id(&self.stream_id, 160, "streamId")?;
        ensure_id(&self.simulator_id, 128, "simulatorId")?;
        if self.session_epoch == 0 {
            return Err(ProtocolError::InvalidField("sessionEpoch"));
        }
        ensure_uuid_v4(self.payload.report_message_id, "reportMessageId")?;
        ensure_uuid_v4(self.payload.simulator_boot_id, "simulatorBootId")?;
        ensure_code(&self.payload.code)
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct StreamCursor {
    pub stream_id: String,
    pub event_sequence: u64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SimulatorRobotSnapshot {
    pub robot_id: String,
    pub ready: bool,
    pub map: MapIdentity,
    #[serde(default)]
    pub map_content: Option<RasterMapContent>,
    #[serde(default)]
    pub order_id: Option<String>,
    #[serde(default)]
    pub order_update_id: Option<u64>,
    pub active_controller: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SimulatorSnapshot {
    pub contract_version: String,
    pub simulator_id: String,
    pub session_epoch: u64,
    pub stream_cursor: StreamCursor,
    pub robots: Vec<SimulatorRobotSnapshot>,
}

impl SimulatorSnapshot {
    pub fn validate(&self) -> Result<(), ProtocolError> {
        if self.contract_version != CONTRACT_VERSION || self.session_epoch == 0 {
            return Err(ProtocolError::ContractVersion);
        }
        ensure_id(&self.simulator_id, 128, "simulatorId")?;
        ensure_id(&self.stream_cursor.stream_id, 160, "streamId")?;
        if self.robots.is_empty() {
            return Err(ProtocolError::InvalidField("robots"));
        }
        for robot in &self.robots {
            ensure_id(&robot.robot_id, 128, "robotId")?;
            ensure_id(&robot.active_controller, 128, "activeController")?;
            robot.map.validate()?;
            if let Some(content) = &robot.map_content {
                content.validate()?;
                if content.identity() != robot.map {
                    return Err(ProtocolError::InvalidField("mapContent identity"));
                }
            }
            if robot.order_id.is_some() != robot.order_update_id.is_some() {
                return Err(ProtocolError::InvalidField("snapshot Order identity"));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ReportBatchRequest<'a> {
    pub request_id: Uuid,
    pub reports: &'a [ReportEnvelope],
}

#[derive(Clone, Debug, Eq, PartialEq, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ReportBatchOutcome {
    pub request_id: Uuid,
    pub outcomes: Vec<ReportBatchItemOutcome>,
}

#[derive(Clone, Debug, Eq, PartialEq, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ReportBatchItemOutcome {
    pub report_message_id: Uuid,
    pub report_sequence: u64,
    pub disposition: ReportDisposition,
    pub durability: ReportDurability,
    pub retryable: bool,
    pub code: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ProtocolError {
    ContractVersion,
    InvalidEnvelope,
    InvalidField(&'static str),
    IncompatibleCapability,
}

impl fmt::Display for ProtocolError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ContractVersion => formatter.write_str("incompatible Core contract version"),
            Self::InvalidEnvelope => formatter.write_str("invalid Core message envelope"),
            Self::InvalidField(field) => write!(formatter, "invalid protocol field: {field}"),
            Self::IncompatibleCapability => {
                formatter.write_str("Core stream capabilities are incompatible")
            }
        }
    }
}

impl std::error::Error for ProtocolError {}

fn validate_core_envelope(
    contract_version: &str,
    message_id: Uuid,
    actual_type: &str,
    expected_type: &'static str,
    producer: &Producer,
    correlation_id: Uuid,
) -> Result<(), ProtocolError> {
    if contract_version != CONTRACT_VERSION {
        return Err(ProtocolError::ContractVersion);
    }
    ensure_uuid_v4(message_id, "messageId")?;
    ensure_uuid_v4(correlation_id, "correlationId")?;
    if actual_type != expected_type
        || producer.kind != ProducerKind::Core
        || producer.id.is_empty()
        || producer.id.len() > 128
    {
        return Err(ProtocolError::InvalidEnvelope);
    }
    Ok(())
}

fn ensure_uuid_v4(value: Uuid, field: &'static str) -> Result<(), ProtocolError> {
    if value.get_version_num() != 4 || value.as_hyphenated().to_string() != value.to_string() {
        return Err(ProtocolError::InvalidField(field));
    }
    Ok(())
}

fn ensure_id(value: &str, max: usize, field: &'static str) -> Result<(), ProtocolError> {
    if value.is_empty() || value.len() > max || value.chars().any(char::is_control) {
        return Err(ProtocolError::InvalidField(field));
    }
    Ok(())
}

fn ensure_sha256(value: &str, field: &'static str) -> Result<(), ProtocolError> {
    if value.len() != 64
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(ProtocolError::InvalidField(field));
    }
    Ok(())
}

fn ensure_code(value: &str) -> Result<(), ProtocolError> {
    let mut bytes = value.bytes();
    if !(3..=64).contains(&value.len())
        || !bytes.next().is_some_and(|byte| byte.is_ascii_uppercase())
        || !bytes.all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit() || byte == b'_')
    {
        return Err(ProtocolError::InvalidField("code"));
    }
    Ok(())
}

fn ensure_timestamp(value: &str) -> Result<(), ProtocolError> {
    let bytes = value.as_bytes();
    if bytes.len() != 24
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || bytes[10] != b'T'
        || bytes[13] != b':'
        || bytes[16] != b':'
        || bytes[19] != b'.'
        || bytes[23] != b'Z'
        || bytes.iter().enumerate().any(|(index, byte)| {
            !matches!(index, 4 | 7 | 10 | 13 | 16 | 19 | 23) && !byte.is_ascii_digit()
        })
    {
        return Err(ProtocolError::InvalidField("occurredAt"));
    }
    let parse = |range: std::ops::Range<usize>| {
        value[range]
            .parse::<u32>()
            .map_err(|_| ProtocolError::InvalidField("occurredAt"))
    };
    let year = parse(0..4)?;
    let month = parse(5..7)?;
    let day = parse(8..10)?;
    let hour = parse(11..13)?;
    let minute = parse(14..16)?;
    let second = parse(17..19)?;
    let leap = year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400));
    let days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => 0,
    };
    if year == 0 || day == 0 || day > days || hour > 23 || minute > 59 || second > 59 {
        return Err(ProtocolError::InvalidField("occurredAt"));
    }
    Ok(())
}
