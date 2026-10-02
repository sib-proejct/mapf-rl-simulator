use crate::motion::ActuatorEffect;
use crate::random::{FAULT_STREAM, derive_seed, sample_for_tick};
use crate::types::{ControlTick, RobotId, SimulationTimeMs, ValidationError};
use std::collections::BTreeSet;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FaultKind {
    EmergencyStop,
    ActuatorStuck,
    ActuatorSlowdownPermille(u16),
    SensorDropout,
}

impl FaultKind {
    pub const fn priority(self) -> u8 {
        match self {
            Self::EmergencyStop => 10,
            Self::ActuatorStuck => 20,
            Self::ActuatorSlowdownPermille(_) => 30,
            Self::SensorDropout => 40,
        }
    }

    pub const fn code(self) -> &'static str {
        match self {
            Self::EmergencyStop => "emergency_stop",
            Self::ActuatorStuck => "actuator_stuck",
            Self::ActuatorSlowdownPermille(_) => "actuator_slowdown",
            Self::SensorDropout => "sensor_dropout",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FaultSpec {
    pub start_tick: u64,
    pub end_tick_inclusive: u64,
    pub probability_parts_per_million: u32,
    pub stable_source_id: String,
    pub kind: FaultKind,
}

impl FaultSpec {
    pub fn once(tick: u64, stable_source_id: impl Into<String>, kind: FaultKind) -> Self {
        Self {
            start_tick: tick,
            end_tick_inclusive: tick,
            probability_parts_per_million: 1_000_000,
            stable_source_id: stable_source_id.into(),
            kind,
        }
    }

    fn validate(&self) -> Result<(), ValidationError> {
        if self.start_tick > self.end_tick_inclusive {
            return Err(ValidationError::OutOfRange("fault.tick_range"));
        }
        if self.probability_parts_per_million > 1_000_000 {
            return Err(ValidationError::OutOfRange("fault.probability"));
        }
        if self.stable_source_id.is_empty()
            || self.stable_source_id.len() > 128
            || self.stable_source_id.chars().any(char::is_control)
        {
            return Err(ValidationError::InvalidId("fault source"));
        }
        if let FaultKind::ActuatorSlowdownPermille(value) = self.kind
            && value > 1_000
        {
            return Err(ValidationError::OutOfRange("fault.slowdown"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ScheduledFault {
    pub simulation_time: SimulationTimeMs,
    pub priority: u8,
    pub stable_source_id: String,
    pub source_sequence: u64,
    pub kind: FaultKind,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActiveFaults {
    emergency_stop: bool,
    actuator_stuck: bool,
    actuator_speed_scale: f64,
}

impl Default for ActiveFaults {
    fn default() -> Self {
        Self {
            emergency_stop: false,
            actuator_stuck: false,
            actuator_speed_scale: 1.0,
        }
    }
}

impl ActiveFaults {
    pub const fn emergency_stop(self) -> bool {
        self.emergency_stop
    }

    pub fn actuator_effect(self) -> ActuatorEffect {
        ActuatorEffect::new(self.actuator_speed_scale, self.actuator_stuck)
            .expect("validated fault state")
    }
}

#[derive(Clone, Debug)]
pub struct FaultInjector {
    sources: Vec<FaultSource>,
    active: ActiveFaults,
}

#[derive(Clone, Debug)]
struct FaultSource {
    spec: FaultSpec,
    stream_seed: u64,
}

impl FaultInjector {
    pub fn new(
        master_seed: u64,
        robot_id: &RobotId,
        specs: &[FaultSpec],
    ) -> Result<Self, ValidationError> {
        let mut sources = Vec::new();
        let mut source_ids = BTreeSet::new();
        for spec in specs {
            spec.validate()?;
            if !source_ids.insert(spec.stable_source_id.as_str()) {
                return Err(ValidationError::DuplicateId("fault source"));
            }
            sources.push(FaultSource {
                spec: spec.clone(),
                stream_seed: derive_seed(
                    master_seed,
                    FAULT_STREAM,
                    robot_id.as_str(),
                    &spec.stable_source_id,
                ),
            });
        }
        sources.sort_by(|left, right| {
            (
                left.spec.kind.priority(),
                left.spec.stable_source_id.as_bytes(),
            )
                .cmp(&(
                    right.spec.kind.priority(),
                    right.spec.stable_source_id.as_bytes(),
                ))
        });
        Ok(Self {
            sources,
            active: ActiveFaults::default(),
        })
    }

    pub fn evaluate_at(&self, tick: ControlTick, time: SimulationTimeMs) -> AppliedFaults {
        let mut applied = Vec::new();
        let mut sensor_dropout = false;
        let mut active = self.active;
        for source in &self.sources {
            let spec = &source.spec;
            if tick.get() < spec.start_tick || tick.get() > spec.end_tick_inclusive {
                continue;
            }
            let draw = sample_for_tick(source.stream_seed, tick.get(), 0) % 1_000_000;
            if draw >= u64::from(spec.probability_parts_per_million) {
                continue;
            }
            let event = ScheduledFault {
                simulation_time: time,
                priority: spec.kind.priority(),
                stable_source_id: spec.stable_source_id.clone(),
                source_sequence: tick.get() - spec.start_tick,
                kind: spec.kind,
            };
            match event.kind {
                FaultKind::EmergencyStop => active.emergency_stop = true,
                FaultKind::ActuatorStuck => active.actuator_stuck = true,
                FaultKind::ActuatorSlowdownPermille(value) => {
                    active.actuator_speed_scale = f64::from(value) / 1_000.0;
                }
                FaultKind::SensorDropout => sensor_dropout = true,
            }
            applied.push(event);
        }
        AppliedFaults {
            events: applied,
            sensor_dropout,
            active,
        }
    }

    pub fn commit(&mut self, active: ActiveFaults) {
        self.active = active;
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AppliedFaults {
    pub events: Vec<ScheduledFault>,
    pub sensor_dropout: bool,
    pub active: ActiveFaults,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_tick_faults_use_priority_then_source_order() {
        let robot_id = RobotId::new("r1").unwrap();
        let mut injector = FaultInjector::new(
            1,
            &robot_id,
            &[
                FaultSpec::once(0, "z-sensor", FaultKind::SensorDropout),
                FaultSpec::once(0, "a-actuator", FaultKind::ActuatorStuck),
                FaultSpec::once(0, "a-emergency", FaultKind::EmergencyStop),
            ],
        )
        .unwrap();
        let applied = injector.evaluate_at(ControlTick::ZERO, SimulationTimeMs::ZERO);
        let order: Vec<_> = applied
            .events
            .iter()
            .map(|event| (event.priority, event.stable_source_id.as_str()))
            .collect();
        assert_eq!(
            order,
            vec![(10, "a-emergency"), (20, "a-actuator"), (40, "z-sensor")]
        );
        injector.commit(applied.active);
    }

    #[test]
    fn large_fault_ranges_are_evaluated_lazily() {
        let robot_id = RobotId::new("r1").unwrap();
        let injector = FaultInjector::new(
            1,
            &robot_id,
            &[FaultSpec {
                start_tick: 0,
                end_tick_inclusive: u64::MAX,
                probability_parts_per_million: 0,
                stable_source_id: "long-range".to_owned(),
                kind: FaultKind::SensorDropout,
            }],
        )
        .unwrap();

        assert!(
            injector
                .evaluate_at(ControlTick::ZERO, SimulationTimeMs::ZERO)
                .events
                .is_empty()
        );
    }
}
