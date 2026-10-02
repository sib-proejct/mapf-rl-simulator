use crate::random::{SENSOR_STREAM, derive_seed, sample_for_tick, uniform_signed};
use crate::types::{
    ControlTick, Meters, RobotId, RobotState, SimulationTimeMs, ValidationError, WorldPosition,
    ensure_non_negative,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorConfig {
    position_noise: Meters,
    dropout_parts_per_million: u32,
}

impl SensorConfig {
    pub fn new(
        position_noise_meters: f64,
        dropout_parts_per_million: u32,
    ) -> Result<Self, ValidationError> {
        ensure_non_negative(position_noise_meters, "sensor.position_noise")?;
        if dropout_parts_per_million > 1_000_000 {
            return Err(ValidationError::OutOfRange("sensor.dropout"));
        }
        Ok(Self {
            position_noise: Meters::new(position_noise_meters)?,
            dropout_parts_per_million,
        })
    }

    pub const fn position_noise_meters(self) -> f64 {
        self.position_noise.get()
    }

    pub const fn dropout_parts_per_million(self) -> u32 {
        self.dropout_parts_per_million
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorReading {
    pub simulation_time: SimulationTimeMs,
    pub observed_position: Option<WorldPosition>,
}

#[derive(Clone, Debug)]
pub struct SeededSensor {
    stream_seed: u64,
    config: SensorConfig,
}

impl SeededSensor {
    pub fn new(master_seed: u64, robot_id: &RobotId, config: SensorConfig) -> Self {
        Self {
            stream_seed: derive_seed(master_seed, SENSOR_STREAM, robot_id.as_str(), "position"),
            config,
        }
    }

    pub fn sense(
        &self,
        tick: ControlTick,
        simulation_time: SimulationTimeMs,
        state: RobotState,
        forced_dropout: bool,
    ) -> SensorReading {
        let dropout_draw = sample_for_tick(self.stream_seed, tick.get(), 0) % 1_000_000;
        if forced_dropout || dropout_draw < u64::from(self.config.dropout_parts_per_million) {
            return SensorReading {
                simulation_time,
                observed_position: None,
            };
        }
        let noise_x = uniform_signed(sample_for_tick(self.stream_seed, tick.get(), 1))
            * self.config.position_noise.get();
        let noise_y = uniform_signed(sample_for_tick(self.stream_seed, tick.get(), 2))
            * self.config.position_noise.get();
        let position = state.position();
        SensorReading {
            simulation_time,
            observed_position: WorldPosition::new(
                position.x_meters() + noise_x,
                position.y_meters() + noise_y,
            )
            .ok(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Acceleration, Velocity};

    #[test]
    fn reading_is_a_pure_function_of_seed_robot_and_tick() {
        let robot_id = RobotId::new("robot-1").unwrap();
        let sensor = SeededSensor::new(19, &robot_id, SensorConfig::new(0.1, 0).unwrap());
        let state = RobotState::new(
            WorldPosition::new(1.0, 2.0).unwrap(),
            Velocity::ZERO,
            Acceleration::ZERO,
            0.0,
        )
        .unwrap();
        let first = sensor.sense(ControlTick::ZERO, SimulationTimeMs::ZERO, state, false);
        let second = sensor.sense(ControlTick::ZERO, SimulationTimeMs::ZERO, state, false);
        assert_eq!(first, second);
    }
}
