//! Crash-safe local report spool.
//!
//! Filesystem methods are synchronous by design and must be called from startup or
//! a Tokio blocking boundary, never while occupying an async network task.

use crate::protocol::{ReportAckPayload, ReportDisposition, ReportEnvelope, ReportPayload};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use uuid::Uuid;

const SPOOL_FORMAT_VERSION: u32 = 1;
pub const DEFAULT_REPORT_CAPACITY: usize = 10_000;

#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct AppliedOrder {
    pub command_id: Uuid,
    pub order_id: String,
    pub order_update_id: u64,
    pub content_digest_sha256: String,
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
    applied_order: Option<AppliedOrder>,
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
            SpoolBody {
                current_boot_id: boot_id,
                next_report_sequence: 0,
                ..document.body
            }
        } else {
            SpoolBody {
                format_version: SPOOL_FORMAT_VERSION,
                simulator_id: simulator_id.to_owned(),
                current_boot_id: boot_id,
                next_report_sequence: 0,
                pending: Vec::new(),
                dead_letters: Vec::new(),
                applied_order: None,
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

    pub const fn applied_order(&self) -> Option<&AppliedOrder> {
        self.body.applied_order.as_ref()
    }

    pub fn enqueue(&mut self, report: ReportEnvelope) -> Result<(), SpoolError> {
        self.enqueue_with_order(report, None)
    }

    pub fn enqueue_with_order(
        &mut self,
        report: ReportEnvelope,
        applied_order: Option<AppliedOrder>,
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
        self.body.pending.push(report);
        if let Some(order) = applied_order {
            self.body.applied_order = Some(order);
        }
        self.persist_or_rollback(previous)
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

        let previous = self.body.clone();
        let report = self.body.pending.remove(index);
        let result = if disposition == ReportDisposition::Rejected {
            self.body.dead_letters.push(DeadLetter {
                code: code.to_owned(),
                report,
            });
            AckResult::DeadLettered
        } else {
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
