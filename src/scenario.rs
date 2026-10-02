use crate::contracts::generated::CONTRACT_VERSION;
use crate::fault::{FaultKind, FaultSpec};
use crate::motion::{MotionLimits, RUCKIG_VERSION};
use crate::safety::SafetyConfig;
use crate::sensing::SensorConfig;
use crate::simulation::{EngineConfig, EngineError, MonotonicClock, SimulationEngine, StepRecord};
use crate::types::{
    Acceleration, CONTROL_TICK_MS, RobotId, RobotState, ScenarioId, SimulationTimeMs,
    ValidationError, Velocity, WorldPosition,
};
use crate::world::{GridCell, GridMap, MapError};
use sha2::{Digest, Sha256};
use std::fmt;

#[derive(Clone, Debug)]
pub struct Scenario {
    pub id: ScenarioId,
    pub version: String,
    pub master_seed: u64,
    pub robot_id: RobotId,
    pub map: GridMap,
    pub initial_state: RobotState,
    pub engine_config: EngineConfig,
    pub action_indices: Vec<i32>,
    pub fault_specs: Vec<FaultSpec>,
}

impl Scenario {
    pub fn validate(&self) -> Result<(), ScenarioError> {
        if self.version.is_empty()
            || self.version.len() > 64
            || self.version.chars().any(char::is_control)
        {
            return Err(ScenarioError::Invalid(ValidationError::InvalidId(
                "scenario version",
            )));
        }
        if self.action_indices.is_empty() {
            return Err(ScenarioError::NoTicks);
        }
        Ok(())
    }

    pub fn run<C: MonotonicClock>(&self, clock: C) -> Result<ScenarioResult, ScenarioError> {
        self.validate()?;
        let mut engine = SimulationEngine::new(
            clock,
            self.robot_id.clone(),
            self.map.clone(),
            self.initial_state,
            self.engine_config,
            self.master_seed,
            &self.fault_specs,
        )?;
        let mut records = Vec::with_capacity(self.action_indices.len());
        for action in &self.action_indices {
            records.push(engine.step(*action)?);
        }
        let final_state = engine.state();
        let final_time = engine.simulation_time();
        let digest = state_digest(self, &records, final_state, final_time);
        Ok(ScenarioResult {
            scenario_id: self.id.clone(),
            scenario_version: self.version.clone(),
            master_seed: self.master_seed,
            control_tick_ms: CONTROL_TICK_MS,
            final_time,
            final_state,
            records,
            state_digest_sha256: digest,
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioResult {
    pub scenario_id: ScenarioId,
    pub scenario_version: String,
    pub master_seed: u64,
    pub control_tick_ms: i64,
    pub final_time: SimulationTimeMs,
    pub final_state: RobotState,
    pub records: Vec<StepRecord>,
    pub state_digest_sha256: String,
}

#[derive(Debug)]
pub enum ScenarioError {
    Invalid(ValidationError),
    Map(MapError),
    Engine(EngineError),
    NoTicks,
}

impl From<ValidationError> for ScenarioError {
    fn from(value: ValidationError) -> Self {
        Self::Invalid(value)
    }
}

impl From<MapError> for ScenarioError {
    fn from(value: MapError) -> Self {
        Self::Map(value)
    }
}

impl From<EngineError> for ScenarioError {
    fn from(value: EngineError) -> Self {
        Self::Engine(value)
    }
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid(error) => error.fmt(formatter),
            Self::Map(error) => error.fmt(formatter),
            Self::Engine(error) => error.fmt(formatter),
            Self::NoTicks => formatter.write_str("scenario must contain at least one control tick"),
        }
    }
}

impl std::error::Error for ScenarioError {}

pub fn demo_scenario() -> Result<Scenario, ScenarioError> {
    let map = GridMap::new(
        7,
        5,
        WorldPosition::new(0.0, 0.0)?,
        1.0,
        [GridCell::new(4, 2), GridCell::new(4, 3)],
    )?;
    Ok(Scenario {
        id: ScenarioId::new("phase1-demo")?,
        version: "2.0.0".to_owned(),
        master_seed: 0x5eed_2026,
        robot_id: RobotId::new("robot-001")?,
        map,
        initial_state: RobotState::new(
            WorldPosition::new(1.5, 2.5)?,
            Velocity::ZERO,
            Acceleration::ZERO,
            0.0,
        )?,
        engine_config: EngineConfig {
            motion_limits: MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0)?,
            safety: SafetyConfig::new(0.2, 0.05)?,
            sensor: SensorConfig::new(0.01, 20_000)?,
        },
        action_indices: vec![2, 2, 2, 2, 99, 1, 1, 1, 0, 4, 4, 4],
        fault_specs: vec![FaultSpec {
            start_tick: 2,
            end_tick_inclusive: 9,
            probability_parts_per_million: 180_000,
            stable_source_id: "seeded-sensor-drop".to_owned(),
            kind: FaultKind::SensorDropout,
        }],
    })
}

fn state_digest(
    scenario: &Scenario,
    records: &[StepRecord],
    final_state: RobotState,
    final_time: SimulationTimeMs,
) -> String {
    let mut canonical = CanonicalDigest::new();
    canonical.string("mapf-rl-simulator.phase1.state-digest.v2");
    canonical.string("ruckig.velocity.path-axis-1d");
    canonical.string(RUCKIG_VERSION);
    canonical.string(CONTRACT_VERSION);
    canonical.string(scenario.id.as_str());
    canonical.string(&scenario.version);
    canonical.u64(scenario.master_seed);
    canonical.string(scenario.robot_id.as_str());
    canonical.i64(CONTROL_TICK_MS);
    canonical.u32(scenario.map.width());
    canonical.u32(scenario.map.height());
    canonical.position(scenario.map.origin());
    canonical.f64(scenario.map.resolution_meters());
    canonical.u64(scenario.map.blocked_bits().len() as u64);
    for blocked in scenario.map.blocked_bits() {
        canonical.u8(u8::from(*blocked));
    }
    canonical.state(scenario.initial_state);
    canonical.f64(scenario.engine_config.motion_limits.max_linear_speed_mps());
    canonical.f64(scenario.engine_config.motion_limits.max_acceleration_mps2());
    canonical.f64(scenario.engine_config.motion_limits.max_deceleration_mps2());
    canonical.f64(
        scenario
            .engine_config
            .motion_limits
            .max_emergency_deceleration_mps2(),
    );
    canonical.f64(scenario.engine_config.motion_limits.max_jerk_mps3());
    canonical.f64(
        scenario
            .engine_config
            .motion_limits
            .max_emergency_jerk_mps3(),
    );
    canonical.f64(scenario.engine_config.safety.footprint_radius_meters());
    canonical.f64(
        scenario
            .engine_config
            .safety
            .minimum_obstacle_clearance_meters(),
    );
    canonical.f64(scenario.engine_config.sensor.position_noise_meters());
    canonical.u32(scenario.engine_config.sensor.dropout_parts_per_million());
    canonical.u64(scenario.action_indices.len() as u64);
    for action in &scenario.action_indices {
        canonical.i32(*action);
    }
    canonical.u64(scenario.fault_specs.len() as u64);
    for spec in &scenario.fault_specs {
        canonical.u64(spec.start_tick);
        canonical.u64(spec.end_tick_inclusive);
        canonical.u32(spec.probability_parts_per_million);
        canonical.string(&spec.stable_source_id);
        canonical.fault_kind(spec.kind);
    }
    canonical.u64(records.len() as u64);
    for record in records {
        canonical.u64(record.tick.get());
        canonical.i64(record.simulation_time.get());
        canonical.i32(record.raw_action_index);
        canonical.u8(record.requested_action as u8);
        canonical.u8(record.applied_action as u8);
        canonical.u8(u8::from(record.invalid_action));
        canonical.string(record.safety_outcome.code());
        canonical.string(record.safety_reason.code());
        canonical.i64(record.sensor.simulation_time.get());
        match record.sensor.observed_position {
            Some(position) => {
                canonical.u8(1);
                canonical.position(position);
            }
            None => canonical.u8(0),
        }
        canonical.state(record.state);
        canonical.u64(record.applied_faults.len() as u64);
        for fault in &record.applied_faults {
            canonical.string(&fault.stable_source_id);
            canonical.u64(fault.source_sequence);
            canonical.fault_kind(fault.kind);
        }
    }
    canonical.i64(final_time.get());
    canonical.state(final_state);
    canonical.finish()
}

struct CanonicalDigest(Sha256);

impl CanonicalDigest {
    fn new() -> Self {
        Self(Sha256::new())
    }

    fn u8(&mut self, value: u8) {
        self.0.update([value]);
    }

    fn u32(&mut self, value: u32) {
        self.0.update(value.to_be_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.0.update(value.to_be_bytes());
    }

    fn i32(&mut self, value: i32) {
        self.0.update(value.to_be_bytes());
    }

    fn i64(&mut self, value: i64) {
        self.0.update(value.to_be_bytes());
    }

    fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
    }

    fn string(&mut self, value: &str) {
        self.u64(value.len() as u64);
        self.0.update(value.as_bytes());
    }

    fn position(&mut self, value: WorldPosition) {
        self.f64(value.x_meters());
        self.f64(value.y_meters());
    }

    fn state(&mut self, value: RobotState) {
        self.position(value.position());
        self.f64(value.velocity().x_mps());
        self.f64(value.velocity().y_mps());
        self.f64(value.acceleration().x_mps2());
        self.f64(value.acceleration().y_mps2());
        self.f64(value.yaw_radians());
    }

    fn fault_kind(&mut self, value: FaultKind) {
        self.string(value.code());
        if let FaultKind::ActuatorSlowdownPermille(permille) = value {
            self.u32(u32::from(permille));
        }
    }

    fn finish(self) -> String {
        format!("{:x}", self.0.finalize())
    }
}
