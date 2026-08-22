//! Explicit controller selection for the Phase 2 baseline slice.

use crate::protocol::{ActiveController, ControllerMode};
use sha2::{Digest, Sha256};
use std::fmt;

pub const BASELINE_CONTROLLER_ID: &str = "cardinal-baseline/1.0.0";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RuntimeProfile {
    Local,
    Dev,
    Production,
}

impl RuntimeProfile {
    pub fn parse(value: &str) -> Result<Self, ControllerError> {
        match value {
            "local" => Ok(Self::Local),
            "dev" => Ok(Self::Dev),
            "production" => Ok(Self::Production),
            _ => Err(ControllerError::UnknownProfile),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BaselineController {
    report: ActiveController,
}

impl BaselineController {
    pub fn explicit(profile: RuntimeProfile) -> Result<Self, ControllerError> {
        if profile == RuntimeProfile::Production {
            return Err(ControllerError::BaselineNotAllowed);
        }
        let configuration = format!(
            "mapf-rl.baseline.v1\nidentity={BASELINE_CONTROLLER_ID}\naction-contract=1.0.0\nprofile={profile:?}\n"
        );
        Ok(Self {
            report: ActiveController {
                mode: ControllerMode::Baseline,
                identity: BASELINE_CONTROLLER_ID.to_owned(),
                content_digest_sha256: format!("{:x}", Sha256::digest(configuration.as_bytes())),
            },
        })
    }

    pub const fn report(&self) -> &ActiveController {
        &self.report
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControllerError {
    UnknownProfile,
    BaselineNotAllowed,
}

impl fmt::Display for ControllerError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownProfile => {
                formatter.write_str("MAPF_PROFILE must be local, dev, or production")
            }
            Self::BaselineNotAllowed => {
                formatter.write_str("baseline controller is not allowed in production")
            }
        }
    }
}

impl std::error::Error for ControllerError {}
