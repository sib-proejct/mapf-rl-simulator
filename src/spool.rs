//! Crash-safe local report spool.
//!
//! Filesystem methods are synchronous by design and must be called from startup or
//! a Tokio blocking boundary, never while occupying an async network task.

use crate::protocol::{
    OrderCommand, OrderGoal, OrderRoute, ReportAckPayload, ReportDisposition, ReportEnvelope,
    ReportPayload,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use uuid::Uuid;

const SPOOL_FORMAT_VERSION: u32 = 1;
pub const DEFAULT_REPORT_CAPACITY: usize = 10_000;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct AppliedOrder {
    pub command_id: Uuid,
    pub order_id: String,
    pub order_update_id: u64,
    pub content_digest_sha256: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plan_revision_id: Option<Uuid>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub goal: Option<OrderGoal>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub route: Option<OrderRoute>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arrival_action: Option<crate::contracts::generated::StationAction>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PreparedOrder {
    pub command_id: Uuid,
    pub order_id: String,
    pub order_update_id: u64,
    pub content_digest_sha256: String,
    pub plan_revision_id: Uuid,
    pub prepared_at_monotonic_ms: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub goal: Option<OrderGoal>,
    // Version 1 spool files written before Wave 3 did not contain a route.
    // They must remain readable so startup can fence and clear stale PREPARE state.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub route: Option<OrderRoute>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arrival_action: Option<crate::contracts::generated::StationAction>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct DeadLetter {
    pub code: String,
    pub report: ReportEnvelope,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct SpoolBody {
    format_version: u32,
    simulator_id: String,
    current_boot_id: Uuid,
    next_report_sequence: u64,
    pending: Vec<ReportEnvelope>,
    dead_letters: Vec<DeadLetter>,
    #[serde(default, skip_serializing_if = "is_zero")]
    dead_letters_dropped: u64,
    applied_order: Option<AppliedOrder>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    prepared_order: Option<PreparedOrder>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pending_abort: Option<OrderCommand>,
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    aborted_orders: std::collections::BTreeMap<String, u64>,
    #[serde(default, skip_serializing_if = "std::collections::BTreeSet::is_empty")]
    completed_orders: std::collections::BTreeSet<String>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct SpoolDocument {
    body: SpoolBody,
    checksum_sha256: String,
}

#[derive(Debug)]
pub struct DurableSpool {
    path: PathBuf,
    capacity: usize,
    body: SpoolBody,
}

impl DurableSpool {
    /// Opens durable state and starts a new process boot while retaining reports
    /// that were not accepted before the previous process stopped.
    pub fn open(path: impl Into<PathBuf>, simulator_id: &str) -> Result<Self, SpoolError> {
        Self::open_with_boot(path, simulator_id, Uuid::new_v4(), DEFAULT_REPORT_CAPACITY)
    }

    pub fn open_with_boot(
        path: impl Into<PathBuf>,
        simulator_id: &str,
        boot_id: Uuid,
        capacity: usize,
    ) -> Result<Self, SpoolError> {
        validate_id(simulator_id)?;
        validate_uuid_v4(boot_id)?;
        if capacity == 0 || capacity > DEFAULT_REPORT_CAPACITY {
            return Err(SpoolError::InvalidCapacity);
        }
        let path = path.into();
        let body = if path.exists() {
            let bytes = fs::read(&path)?;
            let document: SpoolDocument = serde_json::from_slice(&bytes)?;
            let checksum = checksum(&document.body)?;
            if checksum != document.checksum_sha256
                || document.body.format_version != SPOOL_FORMAT_VERSION
                || document.body.simulator_id != simulator_id
                || document.body.pending.len() > capacity
            {
                return Err(SpoolError::Corrupt);
            }
            for report in &document.body.pending {
                report.validate().map_err(|_| SpoolError::Corrupt)?;
            }
            let mut recovered = document.body;
            // PREPARE and plan activation authority never survive a process boot.
            // The safety checkpoint retains their identities for reconciliation,
            // but the command spool cannot use them to resume motion.
            recovered.prepared_order = None;
            if recovered
                .applied_order
                .as_ref()
                .is_some_and(|order| order.plan_revision_id.is_some())
            {
                recovered.applied_order = None;
            }
            SpoolBody {
                current_boot_id: boot_id,
                next_report_sequence: 0,
                ..recovered
            }
        } else {
            SpoolBody {
                format_version: SPOOL_FORMAT_VERSION,
                simulator_id: simulator_id.to_owned(),
                current_boot_id: boot_id,
                next_report_sequence: 0,
                pending: Vec::new(),
                dead_letters: Vec::new(),
                dead_letters_dropped: 0,
                applied_order: None,
                prepared_order: None,
                pending_abort: None,
                aborted_orders: std::collections::BTreeMap::new(),
                completed_orders: std::collections::BTreeSet::new(),
            }
        };
        let spool = Self {
            path,
            capacity,
            body,
        };
        spool.persist()?;
        Ok(spool)
    }

    pub const fn current_boot_id(&self) -> Uuid {
        self.body.current_boot_id
    }

    pub fn simulator_id(&self) -> &str {
        &self.body.simulator_id
    }

    pub const fn next_report_sequence(&self) -> u64 {
        self.body.next_report_sequence
    }

    pub fn pending(&self) -> &[ReportEnvelope] {
        &self.body.pending
    }

    pub fn dead_letters(&self) -> &[DeadLetter] {
        &self.body.dead_letters
    }

    pub const fn dead_letters_dropped(&self) -> u64 {
        self.body.dead_letters_dropped
    }

    pub const fn applied_order(&self) -> Option<&AppliedOrder> {
        self.body.applied_order.as_ref()
    }

    pub const fn prepared_order(&self) -> Option<&PreparedOrder> {
        self.body.prepared_order.as_ref()
    }

    pub fn pending_abort(&self) -> Option<&OrderCommand> {
        self.body.pending_abort.as_ref()
    }

    pub fn was_completed(&self, order_id: &str) -> bool {
        self.body.completed_orders.contains(order_id)
    }

    pub fn was_aborted(&self, order_id: &str, update_id: u64) -> bool {
        self.body
            .aborted_orders
            .get(order_id)
            .is_some_and(|version| update_id <= *version)
    }

    pub(crate) fn defer_abort(&mut self, command: OrderCommand) -> Result<(), SpoolError> {
        let previous = self.body.clone();
        self.body.pending_abort = Some(command);
        self.persist_or_rollback(previous)
    }

    pub fn enqueue(&mut self, report: ReportEnvelope) -> Result<(), SpoolError> {
        self.enqueue_with_order(report, None)
    }

    pub fn enqueue_with_order(
        &mut self,
        report: ReportEnvelope,
        applied_order: Option<AppliedOrder>,
    ) -> Result<(), SpoolError> {
        self.enqueue_with_command_checkpoint(
            report,
            applied_order.map_or(AppliedOrderUpdate::Keep, |order| {
                AppliedOrderUpdate::Set(Box::new(order))
            }),
            PreparedOrderUpdate::Keep,
        )
    }

    pub(crate) fn enqueue_with_prepared_order(
        &mut self,
        report: ReportEnvelope,
        prepared_order: PreparedOrder,
    ) -> Result<(), SpoolError> {
        self.enqueue_with_command_checkpoint(
            report,
            AppliedOrderUpdate::Keep,
            PreparedOrderUpdate::Set(Box::new(prepared_order)),
        )
    }

    pub(crate) fn enqueue_activating_prepared_order(
        &mut self,
        report: ReportEnvelope,
        applied_order: AppliedOrder,
    ) -> Result<(), SpoolError> {
        self.enqueue_with_command_checkpoint(
            report,
            AppliedOrderUpdate::Set(Box::new(applied_order)),
            PreparedOrderUpdate::Clear,
        )
    }

    pub(crate) fn enqueue_aborting_prepared_order(
        &mut self,
        report: ReportEnvelope,
    ) -> Result<(), SpoolError> {
        let ReportPayload::CommandAck(ack) = &report.payload else {
            return Err(SpoolError::InvalidReport);
        };
        let applied = if self
            .body
            .applied_order
            .as_ref()
            .is_some_and(|order| order.order_id == ack.order_id)
        {
            AppliedOrderUpdate::Clear
        } else {
            AppliedOrderUpdate::Keep
        };
        let prepared = if self
            .body
            .prepared_order
            .as_ref()
            .is_some_and(|order| order.order_id == ack.order_id)
        {
            PreparedOrderUpdate::Clear
        } else {
            PreparedOrderUpdate::Keep
        };
        self.enqueue_with_command_checkpoint(report, applied, prepared)
    }

    fn enqueue_with_command_checkpoint(
        &mut self,
        report: ReportEnvelope,
        applied_order: AppliedOrderUpdate,
        prepared_order: PreparedOrderUpdate,
    ) -> Result<(), SpoolError> {
        report.validate().map_err(|_| SpoolError::InvalidReport)?;
        if self.body.pending.len() >= self.capacity {
            return Err(SpoolError::Full);
        }
        if report.simulator_id != self.body.simulator_id
            || report.simulator_boot_id != self.body.current_boot_id
            || report.report_sequence != self.body.next_report_sequence
        {
            return Err(SpoolError::InvalidReportIdentity);
        }
        let previous = self.body.clone();
        self.body.next_report_sequence = self
            .body
            .next_report_sequence
            .checked_add(1)
            .ok_or(SpoolError::SequenceExhausted)?;
        if let ReportPayload::CommandAck(ack) = &report.payload
            && matches!(
                ack.code.as_deref(),
                Some("PLAN_ABORTED_STOPPED" | "PLAN_ABORT_NOOP_STOPPED")
            )
        {
            self.body
                .aborted_orders
                .entry(ack.order_id.clone())
                .and_modify(|version| *version = (*version).max(ack.order_update_id))
                .or_insert(ack.order_update_id);
            self.body.pending_abort = None;
        }
        self.body.pending.push(report);
        match applied_order {
            AppliedOrderUpdate::Keep => {}
            AppliedOrderUpdate::Set(order) => self.body.applied_order = Some(*order),
            AppliedOrderUpdate::Clear => self.body.applied_order = None,
        }
        match prepared_order {
            PreparedOrderUpdate::Keep => {}
            PreparedOrderUpdate::Set(order) => self.body.prepared_order = Some(*order),
            PreparedOrderUpdate::Clear => self.body.prepared_order = None,
        }
        self.persist_or_rollback(previous)
    }

    pub(crate) fn expire_prepared_order(&mut self, monotonic_ms: u64) -> Result<bool, SpoolError> {
        let Some(prepared) = &self.body.prepared_order else {
            return Ok(false);
        };
        if monotonic_ms
            < prepared
                .prepared_at_monotonic_ms
                .saturating_add(crate::plan::PREPARE_BARRIER_TIMEOUT_MS)
        {
            return Ok(false);
        }
        let previous = self.body.clone();
        self.body.prepared_order = None;
        self.persist_or_rollback(previous)?;
        Ok(true)
    }

    /// Rebinds only this process's pending reports. Reports recovered from an old
    /// boot retain their original epoch and are submitted as historical recovery.
    pub fn rebind_current_boot(&mut self, session_epoch: u64) -> Result<(), SpoolError> {
        if session_epoch == 0 {
            return Err(SpoolError::InvalidReportIdentity);
        }
        let previous = self.body.clone();
        for report in &mut self.body.pending {
            if report.simulator_boot_id == self.body.current_boot_id {
                report.session_epoch = session_epoch;
            }
        }
        self.persist_or_rollback(previous)
    }

    pub fn acknowledge_ws(
        &mut self,
        correlation_id: Uuid,
        acknowledgement: &ReportAckPayload,
    ) -> Result<AckResult, SpoolError> {
        let Some(report) = self.body.pending.iter().find(|report| {
            report.message_id == acknowledgement.report_message_id
                && report.simulator_boot_id == acknowledgement.simulator_boot_id
                && report.report_sequence == acknowledgement.report_sequence
                && report.correlation_id == correlation_id
        }) else {
            return Ok(AckResult::Unknown);
        };
        self.acknowledge(
            report.message_id,
            report.report_sequence,
            acknowledgement.disposition,
            acknowledgement.retryable,
            &acknowledgement.code,
        )
    }

    pub fn acknowledge(
        &mut self,
        message_id: Uuid,
        report_sequence: u64,
        disposition: ReportDisposition,
        retryable: bool,
        code: &str,
    ) -> Result<AckResult, SpoolError> {
        let Some(index) = self.body.pending.iter().position(|report| {
            report.message_id == message_id && report.report_sequence == report_sequence
        }) else {
            return Ok(AckResult::Unknown);
        };
        if disposition == ReportDisposition::Rejected && retryable {
            return Ok(AckResult::Retained);
        }

        let clears_applied_order = disposition != ReportDisposition::Rejected
            && completion_matches_applied_order(
                &self.body.pending[index],
                self.body.applied_order.as_ref(),
            );
        let previous = self.body.clone();
        let report = self.body.pending.remove(index);
        let result = if disposition == ReportDisposition::Rejected {
            if self.body.dead_letters.len() >= self.capacity {
                self.body.dead_letters.remove(0);
                self.body.dead_letters_dropped = self.body.dead_letters_dropped.saturating_add(1);
            }
            self.body.dead_letters.push(DeadLetter {
                code: code.to_owned(),
                report,
            });
            AckResult::DeadLettered
        } else {
            if let ReportPayload::RobotEvent(event) = &report.payload
                && event.code == "ORDER_COMPLETED"
                && let Some(order_id) = event
                    .evidence
                    .get("orderId")
                    .and_then(serde_json::Value::as_str)
            {
                self.body.completed_orders.insert(order_id.to_owned());
            }
            if clears_applied_order {
                self.body.applied_order = None;
            }
            AckResult::Removed
        };
        self.persist_or_rollback(previous)?;
        Ok(result)
    }

    pub fn pending_state_report_count(&self) -> usize {
        self.body
            .pending
            .iter()
            .filter(|report| matches!(report.payload, ReportPayload::State(_)))
            .count()
    }

    fn persist_or_rollback(&mut self, previous: SpoolBody) -> Result<(), SpoolError> {
        if let Err(error) = self.persist() {
            self.body = previous;
            return Err(error);
        }
        Ok(())
    }

    fn persist(&self) -> Result<(), SpoolError> {
        let parent = self.path.parent().ok_or(SpoolError::InvalidPath)?;
        fs::create_dir_all(parent)?;
        let document = SpoolDocument {
            checksum_sha256: checksum(&self.body)?,
            body: self.body.clone(),
        };
        let bytes = serde_json::to_vec(&document)?;
        let temporary = temporary_path(&self.path);
        let mut file = OpenOptions::new()
            .create(true)
            .truncate(true)
            .write(true)
            .open(&temporary)?;
        file.write_all(&bytes)?;
        file.sync_all()?;
        fs::rename(&temporary, &self.path)?;
        File::open(parent)?.sync_all()?;
        Ok(())
    }
}

fn completion_matches_applied_order(
    report: &ReportEnvelope,
    applied: Option<&AppliedOrder>,
) -> bool {
    let Some(applied) = applied else {
        return false;
    };
    let ReportPayload::RobotEvent(payload) = &report.payload else {
        return false;
    };
    payload.code == "ORDER_COMPLETED"
        && payload
            .evidence
            .get("orderId")
            .and_then(serde_json::Value::as_str)
            == Some(applied.order_id.as_str())
        && payload
            .evidence
            .get("orderUpdateId")
            .and_then(serde_json::Value::as_u64)
            == Some(applied.order_update_id)
}

enum PreparedOrderUpdate {
    Keep,
    Set(Box<PreparedOrder>),
    Clear,
}

enum AppliedOrderUpdate {
    Keep,
    Set(Box<AppliedOrder>),
    Clear,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AckResult {
    Removed,
    Retained,
    DeadLettered,
    Unknown,
}

#[derive(Debug)]
pub enum SpoolError {
    Io(io::Error),
    Json(serde_json::Error),
    Corrupt,
    Full,
    InvalidCapacity,
    InvalidPath,
    InvalidReport,
    InvalidReportIdentity,
    InvalidSimulatorId,
    InvalidUuid,
    SequenceExhausted,
}

impl From<io::Error> for SpoolError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<serde_json::Error> for SpoolError {
    fn from(value: serde_json::Error) -> Self {
        Self::Json(value)
    }
}

impl std::fmt::Display for SpoolError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "report spool I/O failed: {error}"),
            Self::Json(error) => write!(formatter, "report spool JSON failed: {error}"),
            Self::Corrupt => formatter.write_str("report spool checksum or contents are invalid"),
            Self::Full => formatter.write_str("report spool capacity is exhausted"),
            Self::InvalidCapacity => formatter.write_str("report spool capacity is invalid"),
            Self::InvalidPath => formatter.write_str("report spool path has no parent"),
            Self::InvalidReport => formatter.write_str("report does not satisfy the Core contract"),
            Self::InvalidReportIdentity => {
                formatter.write_str("report identity is not the next process-global sequence")
            }
            Self::InvalidSimulatorId => formatter.write_str("simulatorId is invalid"),
            Self::InvalidUuid => formatter.write_str("simulatorBootId must be UUIDv4"),
            Self::SequenceExhausted => formatter.write_str("reportSequence is exhausted"),
        }
    }
}

impl std::error::Error for SpoolError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Json(error) => Some(error),
            _ => None,
        }
    }
}

fn checksum(body: &SpoolBody) -> Result<String, serde_json::Error> {
    Ok(format!("{:x}", Sha256::digest(serde_json::to_vec(body)?)))
}

fn temporary_path(path: &Path) -> PathBuf {
    let mut extension = path.extension().unwrap_or_default().to_os_string();
    extension.push(".tmp");
    path.with_extension(extension)
}

fn validate_id(value: &str) -> Result<(), SpoolError> {
    if value.is_empty() || value.len() > 128 || value.chars().any(char::is_control) {
        return Err(SpoolError::InvalidSimulatorId);
    }
    Ok(())
}

fn validate_uuid_v4(value: Uuid) -> Result<(), SpoolError> {
    if value.get_version_num() != 4 {
        return Err(SpoolError::InvalidUuid);
    }
    Ok(())
}

const fn is_zero(value: &u64) -> bool {
    *value == 0
}
