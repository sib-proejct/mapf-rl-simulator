//! Crash-safe fleet safety checkpoint.
//!
//! A recovered checkpoint is local evidence, not motion authority. Loading it
//! fences the plan coordinator until a matching Core snapshot is reconciled.

use crate::plan::PlanCoordinator;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};

const CHECKPOINT_FORMAT_VERSION: u32 = 1;
pub const MAX_CHECKPOINT_ROBOTS: usize = 10_000;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct RobotSafetyCheckpoint {
    pub robot_id: String,
    pub tick: u64,
    pub simulation_time_ms: i64,
    pub x_meters: f64,
    pub y_meters: f64,
    pub velocity_x_mps: f64,
    pub velocity_y_mps: f64,
    pub acceleration_x_mps2: f64,
    pub acceleration_y_mps2: f64,
    pub yaw_radians: f64,
    pub emergency_stop_latched: bool,
}

impl RobotSafetyCheckpoint {
    fn validate(&self) -> Result<(), CheckpointError> {
        if self.robot_id.is_empty()
            || self.robot_id.len() > 128
            || self.robot_id.chars().any(char::is_control)
            || self.simulation_time_ms < 0
            || self.simulation_time_ms
                != i64::try_from(self.tick)
                    .ok()
                    .and_then(|tick| tick.checked_mul(crate::types::CONTROL_TICK_MS))
                    .ok_or(CheckpointError::Corrupt)?
            || [
                self.x_meters,
                self.y_meters,
                self.velocity_x_mps,
                self.velocity_y_mps,
                self.acceleration_x_mps2,
                self.acceleration_y_mps2,
                self.yaw_radians,
            ]
            .iter()
            .any(|value| !value.is_finite())
        {
            return Err(CheckpointError::Corrupt);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct RecoveryCheckpoint {
    pub simulator_id: String,
    pub map_content_digest_sha256: String,
    pub robots: Vec<RobotSafetyCheckpoint>,
    pub plans: PlanCoordinator,
    pub no_progress_ticks: u32,
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub station_states: std::collections::BTreeMap<String, crate::station::StationState>,
    /// Always true after a persisted checkpoint is loaded in a new process.
    pub requires_core_reconciliation: bool,
}

impl RecoveryCheckpoint {
    pub fn with_station_state(
        mut self,
        robot_id: String,
        state: crate::station::StationState,
    ) -> Self {
        self.station_states.insert(robot_id, state);
        self
    }
    pub fn validate(&self) -> Result<(), CheckpointError> {
        if self.simulator_id.is_empty()
            || self.simulator_id.len() > 128
            || self.simulator_id.chars().any(char::is_control)
            || !is_sha256(&self.map_content_digest_sha256)
            || self.robots.is_empty()
            || self.robots.len() > MAX_CHECKPOINT_ROBOTS
        {
            return Err(CheckpointError::Corrupt);
        }
        if self.station_states.iter().any(|(id, state)| {
            !state.valid() || !self.robots.iter().any(|robot| &robot.robot_id == id)
        }) {
            return Err(CheckpointError::Corrupt);
        }
        let mut ids = std::collections::BTreeSet::new();
        for robot in &self.robots {
            robot.validate()?;
            if !ids.insert(robot.robot_id.as_str()) {
                return Err(CheckpointError::Corrupt);
            }
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct CheckpointBody {
    format_version: u32,
    checkpoint: RecoveryCheckpoint,
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct CheckpointDocument {
    body: CheckpointBody,
    checksum_sha256: String,
}

#[derive(Clone, Debug)]
pub struct CheckpointStore {
    path: PathBuf,
}

impl CheckpointStore {
    pub fn new(path: impl Into<PathBuf>) -> Result<Self, CheckpointError> {
        let path = path.into();
        if path.parent().is_none() {
            return Err(CheckpointError::InvalidPath);
        }
        Ok(Self { path })
    }

    pub fn save(&self, checkpoint: &RecoveryCheckpoint) -> Result<(), CheckpointError> {
        checkpoint.validate()?;
        let body = CheckpointBody {
            format_version: CHECKPOINT_FORMAT_VERSION,
            checkpoint: checkpoint.clone(),
        };
        let document = CheckpointDocument {
            checksum_sha256: checksum(&body)?,
            body,
        };
        let bytes = serde_json::to_vec(&document)?;
        let parent = self.path.parent().ok_or(CheckpointError::InvalidPath)?;
        fs::create_dir_all(parent)?;
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

    pub fn load_for_restart(
        &self,
        simulator_id: &str,
        map_content_digest_sha256: &str,
    ) -> Result<Option<RecoveryCheckpoint>, CheckpointError> {
        if !self.path.exists() {
            return Ok(None);
        }
        let bytes = fs::read(&self.path)?;
        let document: CheckpointDocument = serde_json::from_slice(&bytes)?;
        if document.body.format_version != CHECKPOINT_FORMAT_VERSION
            || checksum(&document.body)? != document.checksum_sha256
        {
            return Err(CheckpointError::Corrupt);
        }
        let mut checkpoint = document.body.checkpoint;
        checkpoint.validate()?;
        if checkpoint.simulator_id != simulator_id
            || checkpoint.map_content_digest_sha256 != map_content_digest_sha256
        {
            return Err(CheckpointError::IdentityMismatch);
        }
        checkpoint.requires_core_reconciliation = true;
        checkpoint.plans.fence_for_restart();
        Ok(Some(checkpoint))
    }
}

#[derive(Debug)]
pub enum CheckpointError {
    Io(io::Error),
    Json(serde_json::Error),
    InvalidPath,
    Corrupt,
    IdentityMismatch,
}

impl From<io::Error> for CheckpointError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<serde_json::Error> for CheckpointError {
    fn from(value: serde_json::Error) -> Self {
        Self::Json(value)
    }
}

impl std::fmt::Display for CheckpointError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "checkpoint I/O failed: {error}"),
            Self::Json(error) => write!(formatter, "checkpoint JSON failed: {error}"),
            Self::InvalidPath => formatter.write_str("checkpoint path has no parent"),
            Self::Corrupt => formatter.write_str("checkpoint checksum or safety state is invalid"),
            Self::IdentityMismatch => {
                formatter.write_str("checkpoint simulator or map identity does not match")
            }
        }
    }
}

impl std::error::Error for CheckpointError {}

fn checksum(body: &CheckpointBody) -> Result<String, serde_json::Error> {
    Ok(format!("{:x}", Sha256::digest(serde_json::to_vec(body)?)))
}

fn temporary_path(path: &Path) -> PathBuf {
    let mut extension = path.extension().unwrap_or_default().to_os_string();
    extension.push(".tmp");
    path.with_extension(extension)
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}
