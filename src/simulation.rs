use crate::action::ActionAdapter;
use crate::contracts::generated::ActionCandidate;
use crate::fault::{FaultInjector, FaultSpec, ScheduledFault};
use crate::motion::{MotionLimits, initial_kinematics_are_valid};
use crate::safety::{SafetyError, SafetyKernel, SafetyOutcome, SafetyReason};
use crate::sensing::{SeededSensor, SensorConfig, SensorReading};
use crate::types::{
    ControlTick, MonotonicInstantNs, RobotId, RobotState, SimulationTimeMs, ValidationError,
};
use crate::world::GridMap;
use std::fmt;
use std::sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
};

pub trait MonotonicClock {
    fn now(&self) -> MonotonicInstantNs;
}

#[derive(Clone, Debug, Default)]
pub struct ManualMonotonicClock {
    nanoseconds: Arc<AtomicU64>,
}

impl ManualMonotonicClock {
    pub fn new(nanoseconds: u64) -> Self {
        Self {
            nanoseconds: Arc::new(AtomicU64::new(nanoseconds)),
        }
    }

    pub fn set(&self, nanoseconds: u64) {
        self.nanoseconds.store(nanoseconds, Ordering::SeqCst);
    }

    pub fn advance(&self, nanoseconds: u64) -> Result<(), ValidationError> {
        self.nanoseconds
            .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |current| {
                current.checked_add(nanoseconds)
            })
            .map(|_| ())
            .map_err(|_| ValidationError::TimeOverflow)
    }
}

impl MonotonicClock for ManualMonotonicClock {
    fn now(&self) -> MonotonicInstantNs {
        MonotonicInstantNs::new(self.nanoseconds.load(Ordering::SeqCst))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngineConfig {
    pub motion_limits: MotionLimits,
    pub safety: crate::safety::SafetyConfig,
    pub sensor: SensorConfig,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StepRecord {
    pub tick: ControlTick,
    pub simulation_time: SimulationTimeMs,
    pub raw_action_index: i32,
    pub requested_action: ActionCandidate,
    pub applied_action: ActionCandidate,
    pub invalid_action: bool,
    pub safety_outcome: SafetyOutcome,
    pub safety_reason: SafetyReason,
    pub sensor: SensorReading,
    pub state: RobotState,
    pub applied_faults: Vec<ScheduledFault>,
}

pub struct SimulationEngine<C> {
    clock: C,
    robot_id: RobotId,
    map: GridMap,
    state: RobotState,
    tick: ControlTick,
    simulation_time: SimulationTimeMs,
    config: EngineConfig,
    safety: SafetyKernel,
    sensor: SeededSensor,
    faults: FaultInjector,
    emergency_stop_latched: bool,
}

impl<C: MonotonicClock> SimulationEngine<C> {
    pub fn new(
        clock: C,
        robot_id: RobotId,
        map: GridMap,
        initial_state: RobotState,
        config: EngineConfig,
        master_seed: u64,
        fault_specs: &[FaultSpec],
    ) -> Result<Self, EngineError> {
        let safety = SafetyKernel::new(config.safety);
        if !safety.state_is_clear(&map, initial_state) {
            return Err(EngineError::UnsafeInitialState);
        }
        if !initial_kinematics_are_valid(initial_state, config.motion_limits) {
            return Err(EngineError::InvalidInitialKinematics);
        }
        if !safety.state_has_safe_emergency_stop(&map, initial_state, config.motion_limits) {
            return Err(EngineError::UnsafeInitialStoppingEnvelope);
        }
        let sensor = SeededSensor::new(master_seed, &robot_id, config.sensor);
        let faults = FaultInjector::new(master_seed, &robot_id, fault_specs)?;
        Ok(Self {
            clock,
            robot_id,
            map,
            state: initial_state,
            tick: ControlTick::ZERO,
            simulation_time: SimulationTimeMs::ZERO,
            config,
            safety,
            sensor,
            faults,
            emergency_stop_latched: false,
        })
    }

    /// Returns operational monotonic time. Physics and event ordering never call
    /// this method and are based solely on integer simulation time.
    pub fn monotonic_now(&self) -> MonotonicInstantNs {
        self.clock.now()
    }

    pub const fn robot_id(&self) -> &RobotId {
        &self.robot_id
    }

    pub const fn tick(&self) -> ControlTick {
        self.tick
    }

    pub const fn simulation_time(&self) -> SimulationTimeMs {
        self.simulation_time
    }

    pub const fn state(&self) -> RobotState {
        self.state
    }

    pub const fn map(&self) -> &GridMap {
        &self.map
    }

    /// Executes the fixed Phase 1 tick ordering and atomically commits at most one
    /// next robot state. Mutable access is the engine's single-writer boundary.
    pub fn step(&mut self, raw_action_index: i32) -> Result<StepRecord, EngineError> {
        let tick = self.tick;
        let simulation_time = self.simulation_time;
        let next_tick = tick.checked_next()?;
        let next_simulation_time = simulation_time.checked_add_tick()?;

        // 1-2. Ordered faults and pre-motion guard.
        let applied_faults = self.faults.evaluate_at(tick, simulation_time);
        let emergency_stop_latched =
            self.emergency_stop_latched || applied_faults.active.emergency_stop();

        // 3. Sensor snapshot. A pre-motion emergency does not create an observation.
        let sensor = if emergency_stop_latched {
            SensorReading {
                simulation_time,
                observed_position: None,
            }
        } else {
            self.sensor.sense(
                tick,
                simulation_time,
                self.state,
                applied_faults.sensor_dropout,
            )
        };

        // 4-6. Candidate adaptation, deterministic safety, then motion preview.
        let adapted = ActionAdapter::from_index(raw_action_index);
        let decision = self.safety.decide(
            &self.map,
            self.state,
            adapted,
            self.config.motion_limits,
            applied_faults.active.actuator_effect(),
            emergency_stop_latched,
        )?;

        // 7. Recheck the candidate independently. An invariant failure leaves all
        // engine state, including the emergency latch and persistent faults, unchanged.
        if !self.safety.state_has_safe_emergency_stop(
            &self.map,
            decision.next_state,
            self.config.motion_limits,
        ) {
            return Err(EngineError::CollisionInvariant);
        }

        // 8-9. Build the transition record, then commit state and time together.
        let record = StepRecord {
            tick,
            simulation_time,
            raw_action_index,
            requested_action: decision.requested_action,
            applied_action: decision.applied_action,
            invalid_action: adapted.invalid_input(),
            safety_outcome: decision.outcome,
            safety_reason: decision.reason,
            sensor,
            state: decision.next_state,
            applied_faults: applied_faults.events,
        };
        self.faults.commit(applied_faults.active);
        self.emergency_stop_latched = emergency_stop_latched;
        self.state = decision.next_state;
        self.tick = next_tick;
        self.simulation_time = next_simulation_time;
        Ok(record)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EngineError {
    Invalid(ValidationError),
    UnsafeInitialState,
    InvalidInitialKinematics,
    UnsafeInitialStoppingEnvelope,
    CollisionInvariant,
}

impl From<ValidationError> for EngineError {
    fn from(value: ValidationError) -> Self {
        Self::Invalid(value)
    }
}

impl From<SafetyError> for EngineError {
    fn from(_: SafetyError) -> Self {
        Self::CollisionInvariant
    }
}

impl fmt::Display for EngineError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid(error) => error.fmt(formatter),
            Self::UnsafeInitialState => {
                formatter.write_str("initial circle footprint is not collision-free")
            }
            Self::InvalidInitialKinematics => {
                formatter.write_str("initial robot velocity or acceleration is invalid")
            }
            Self::UnsafeInitialStoppingEnvelope => {
                formatter.write_str("initial robot state has no collision-free emergency stop")
            }
            Self::CollisionInvariant => {
                formatter.write_str("no collision-free emergency transition can be committed")
            }
        }
    }
}

impl std::error::Error for EngineError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fault::{FaultKind, FaultSpec};
    use crate::motion::MotionLimits;
    use crate::safety::{SafetyConfig, SafetyOutcome};
    use crate::sensing::SensorConfig;
    use crate::types::{Acceleration, Velocity, WorldPosition};
    use crate::world::GridCell;

    #[test]
    fn clock_is_injected_but_does_not_advance_simulation_time() {
        let clock = ManualMonotonicClock::new(5);
        let map = GridMap::new(3, 3, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
        let state = RobotState::new(
            WorldPosition::new(1.5, 1.5).unwrap(),
            Velocity::ZERO,
            Acceleration::ZERO,
            0.0,
        )
        .unwrap();
        let config = EngineConfig {
            motion_limits: MotionLimits::new(1.0, 1.0, 1.0, 2.0, 10.0, 20.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        };
        let mut engine = SimulationEngine::new(
            clock.clone(),
            RobotId::new("r1").unwrap(),
            map,
            state,
            config,
            1,
            &[],
        )
        .unwrap();
        clock.set(u64::MAX);
        engine.step(0).unwrap();
        assert_eq!(engine.simulation_time().get(), 100);
        assert_eq!(engine.monotonic_now().get(), u64::MAX);
    }

    #[test]
    fn unsafe_emergency_transition_does_not_partially_commit_the_tick() {
        let map = GridMap::new(
            4,
            3,
            WorldPosition::new(0.0, 0.0).unwrap(),
            1.0,
            [GridCell::new(2, 1)],
        )
        .unwrap();
        let initial_state = RobotState::new(
            WorldPosition::new(0.5, 1.5).unwrap(),
            Velocity::ZERO,
            Acceleration::ZERO,
            0.0,
        )
        .unwrap();
        let config = EngineConfig {
            motion_limits: MotionLimits::new(2.0, 10.0, 10.0, 20.0, 100.0, 200.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        };
        let mut engine = SimulationEngine::new(
            ManualMonotonicClock::default(),
            RobotId::new("r1").unwrap(),
            map,
            initial_state,
            config,
            1,
            &[FaultSpec::once(0, "emergency", FaultKind::EmergencyStop)],
        )
        .unwrap();
        let unsafe_state = RobotState::new(
            WorldPosition::new(1.8, 1.5).unwrap(),
            Velocity::new(2.0, 0.0).unwrap(),
            Acceleration::ZERO,
            0.0,
        )
        .unwrap();
        engine.state = unsafe_state;

        assert_eq!(engine.step(2), Err(EngineError::CollisionInvariant));
        assert_eq!(engine.state(), unsafe_state);
        assert_eq!(engine.tick(), ControlTick::ZERO);
        assert_eq!(engine.simulation_time(), SimulationTimeMs::ZERO);
        assert!(!engine.emergency_stop_latched);

        engine.state = initial_state;
        let retry = engine.step(2).unwrap();
        assert_eq!(retry.safety_outcome, SafetyOutcome::EmergencyStop);
        assert_eq!(retry.applied_faults.len(), 1);
    }

    #[test]
    fn initial_state_must_have_a_clear_emergency_stopping_envelope() {
        let map = GridMap::new(
            4,
            3,
            WorldPosition::new(0.0, 0.0).unwrap(),
            1.0,
            [GridCell::new(2, 1)],
        )
        .unwrap();
        let state = RobotState::new(
            WorldPosition::new(1.5, 1.5).unwrap(),
            Velocity::new(1.0, 0.0).unwrap(),
            Acceleration::ZERO,
            0.0,
        )
        .unwrap();
        let result = SimulationEngine::new(
            ManualMonotonicClock::default(),
            RobotId::new("r1").unwrap(),
            map,
            state,
            EngineConfig {
                motion_limits: MotionLimits::new(2.0, 1.0, 1.0, 1.0, 10.0, 10.0).unwrap(),
                safety: SafetyConfig::new(0.2, 0.05).unwrap(),
                sensor: SensorConfig::new(0.0, 0).unwrap(),
            },
            1,
            &[],
        );

        assert!(matches!(
            result,
            Err(EngineError::UnsafeInitialStoppingEnvelope)
        ));
    }

    #[test]
    fn initial_acceleration_must_match_the_path_axis() {
        let map = GridMap::new(5, 5, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
        let config = EngineConfig {
            motion_limits: MotionLimits::new(2.0, 2.0, 3.0, 6.0, 30.0, 60.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        };
        for state in [
            RobotState::new(
                WorldPosition::new(2.5, 2.5).unwrap(),
                Velocity::ZERO,
                Acceleration::new(0.1, 0.0).unwrap(),
                0.0,
            )
            .unwrap(),
            RobotState::new(
                WorldPosition::new(2.5, 2.5).unwrap(),
                Velocity::new(1.0, 0.0).unwrap(),
                Acceleration::new(0.0, 0.1).unwrap(),
                0.0,
            )
            .unwrap(),
            RobotState::new(
                WorldPosition::new(2.5, 2.5).unwrap(),
                Velocity::new(1.0, 0.0).unwrap(),
                Acceleration::new(4.0, 0.0).unwrap(),
                0.0,
            )
            .unwrap(),
        ] {
            let result = SimulationEngine::new(
                ManualMonotonicClock::default(),
                RobotId::new("r1").unwrap(),
                map.clone(),
                state,
                config,
                1,
                &[],
            );
            assert!(matches!(result, Err(EngineError::InvalidInitialKinematics)));
        }
    }
}
