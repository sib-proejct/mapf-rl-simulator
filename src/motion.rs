use crate::action::action_vector;
use crate::contracts::generated::ActionCandidate;
use crate::types::{
    CONTROL_TICK_SECONDS, MetersPerSecond, MetersPerSecondSquared, RobotState, ValidationError,
    Velocity, ensure_non_negative, ensure_positive,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionLimits {
    max_linear_speed: MetersPerSecond,
    max_acceleration: MetersPerSecondSquared,
    max_deceleration: MetersPerSecondSquared,
    max_emergency_deceleration: MetersPerSecondSquared,
}

impl MotionLimits {
    pub fn new(
        max_linear_speed_mps: f64,
        max_acceleration_mps2: f64,
        max_deceleration_mps2: f64,
        max_emergency_deceleration_mps2: f64,
    ) -> Result<Self, ValidationError> {
        ensure_positive(max_linear_speed_mps, "motion.max_linear_speed")?;
        ensure_positive(max_acceleration_mps2, "motion.max_acceleration")?;
        ensure_positive(max_deceleration_mps2, "motion.max_deceleration")?;
        ensure_positive(
            max_emergency_deceleration_mps2,
            "motion.max_emergency_deceleration",
        )?;
        if max_emergency_deceleration_mps2 < max_deceleration_mps2 {
            return Err(ValidationError::OutOfRange(
                "motion.max_emergency_deceleration",
            ));
        }
        Ok(Self {
            max_linear_speed: MetersPerSecond::new(max_linear_speed_mps)?,
            max_acceleration: MetersPerSecondSquared::new(max_acceleration_mps2)?,
            max_deceleration: MetersPerSecondSquared::new(max_deceleration_mps2)?,
            max_emergency_deceleration: MetersPerSecondSquared::new(
                max_emergency_deceleration_mps2,
            )?,
        })
    }

    pub const fn max_linear_speed_mps(self) -> f64 {
        self.max_linear_speed.get()
    }

    pub const fn max_acceleration_mps2(self) -> f64 {
        self.max_acceleration.get()
    }

    pub const fn max_deceleration_mps2(self) -> f64 {
        self.max_deceleration.get()
    }

    pub const fn max_emergency_deceleration_mps2(self) -> f64 {
        self.max_emergency_deceleration.get()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorEffect {
    speed_scale: f64,
    stuck: bool,
}

impl ActuatorEffect {
    pub fn new(speed_scale: f64, stuck: bool) -> Result<Self, ValidationError> {
        ensure_non_negative(speed_scale, "fault.actuator_speed_scale")?;
        if speed_scale > 1.0 {
            return Err(ValidationError::OutOfRange("fault.actuator_speed_scale"));
        }
        Ok(Self { speed_scale, stuck })
    }

    pub const fn nominal() -> Self {
        Self {
            speed_scale: 1.0,
            stuck: false,
        }
    }

    pub const fn speed_scale(self) -> f64 {
        self.speed_scale
    }

    pub const fn stuck(self) -> bool {
        self.stuck
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionPreview {
    pub previous: RobotState,
    pub next: RobotState,
    pub velocity_delta_limit_mps: f64,
}

pub fn preview_holonomic_motion(
    state: RobotState,
    action: ActionCandidate,
    limits: MotionLimits,
    actuator: ActuatorEffect,
) -> Result<MotionPreview, ValidationError> {
    let (direction_x, direction_y) = action_vector(action);
    let desired_speed = if actuator.stuck {
        0.0
    } else {
        limits.max_linear_speed.get() * actuator.speed_scale
    };
    let desired_x = direction_x * desired_speed;
    let desired_y = direction_y * desired_speed;
    let current = state.velocity();
    let desired = Velocity::new(desired_x, desired_y)?;
    let current_speed = current.magnitude();
    let desired_speed = desired.magnitude();
    let opposite_or_perpendicular = current_speed > 0.0
        && current.x_mps() * desired.x_mps() + current.y_mps() * desired.y_mps() <= 0.0;
    let decelerating = opposite_or_perpendicular || desired_speed < current_speed;
    let target = if opposite_or_perpendicular {
        Velocity::ZERO
    } else {
        desired
    };
    let acceleration_limit = if decelerating {
        limits.max_deceleration.get()
    } else {
        limits.max_acceleration.get()
    };
    preview_toward_velocity(
        state,
        target,
        acceleration_limit,
        limits.max_linear_speed.get(),
    )
}

pub fn preview_stop(
    state: RobotState,
    deceleration_mps2: f64,
    limits: MotionLimits,
) -> Result<MotionPreview, ValidationError> {
    ensure_positive(deceleration_mps2, "motion.stop_deceleration")?;
    preview_toward_velocity(
        state,
        Velocity::ZERO,
        deceleration_mps2,
        limits.max_linear_speed.get(),
    )
}

fn preview_toward_velocity(
    state: RobotState,
    target: Velocity,
    acceleration_limit_mps2: f64,
    max_linear_speed_mps: f64,
) -> Result<MotionPreview, ValidationError> {
    let current = state.velocity();
    let delta_x = target.x_mps() - current.x_mps();
    let delta_y = target.y_mps() - current.y_mps();
    let delta_magnitude = delta_x.hypot(delta_y);
    let maximum_delta = acceleration_limit_mps2 * CONTROL_TICK_SECONDS;
    let scale = if delta_magnitude > maximum_delta && delta_magnitude > 0.0 {
        maximum_delta / delta_magnitude
    } else {
        1.0
    };
    let mut next_x = current.x_mps() + delta_x * scale;
    let mut next_y = current.y_mps() + delta_y * scale;
    let next_speed = next_x.hypot(next_y);
    if next_speed > max_linear_speed_mps {
        let clamp = max_linear_speed_mps / next_speed;
        next_x *= clamp;
        next_y *= clamp;
    }
    let velocity = Velocity::new(next_x, next_y)?;
    let position = state.position().translated(velocity, CONTROL_TICK_SECONDS);
    let next = state.transitioned(position, velocity);
    if !position.x_meters().is_finite() || !position.y_meters().is_finite() {
        return Err(ValidationError::NonFinite("motion.next_position"));
    }
    Ok(MotionPreview {
        previous: state,
        next,
        velocity_delta_limit_mps: maximum_delta,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::WorldPosition;

    #[test]
    fn semi_implicit_euler_uses_new_velocity_for_position() {
        let state =
            RobotState::new(WorldPosition::new(1.0, 1.0).unwrap(), Velocity::ZERO, 0.25).unwrap();
        let limits = MotionLimits::new(2.0, 1.0, 2.0, 4.0).unwrap();
        let next = preview_holonomic_motion(
            state,
            ActionCandidate::East,
            limits,
            ActuatorEffect::nominal(),
        )
        .unwrap()
        .next;
        assert_eq!(next.velocity().x_mps(), 0.1);
        assert_eq!(next.position().x_meters(), 1.01);
        assert_eq!(next.yaw_radians(), 0.25);
    }

    #[test]
    fn opposite_direction_brakes_before_accelerating() {
        let state = RobotState::new(
            WorldPosition::new(1.0, 1.0).unwrap(),
            Velocity::new(1.0, 0.0).unwrap(),
            0.0,
        )
        .unwrap();
        let limits = MotionLimits::new(2.0, 10.0, 1.0, 4.0).unwrap();
        let next = preview_holonomic_motion(
            state,
            ActionCandidate::West,
            limits,
            ActuatorEffect::nominal(),
        )
        .unwrap()
        .next;

        assert!((next.velocity().x_mps() - 0.9).abs() < 1.0e-12);
        assert_eq!(next.velocity().y_mps(), 0.0);
    }

    #[test]
    fn new_direction_accelerates_only_after_velocity_reaches_zero() {
        let state = RobotState::new(
            WorldPosition::new(1.0, 1.0).unwrap(),
            Velocity::new(0.1, 0.0).unwrap(),
            0.0,
        )
        .unwrap();
        let limits = MotionLimits::new(2.0, 2.0, 1.0, 4.0).unwrap();

        let stopped = preview_holonomic_motion(
            state,
            ActionCandidate::West,
            limits,
            ActuatorEffect::nominal(),
        )
        .unwrap()
        .next;
        assert_eq!(stopped.velocity(), Velocity::ZERO);

        let reversed = preview_holonomic_motion(
            stopped,
            ActionCandidate::West,
            limits,
            ActuatorEffect::nominal(),
        )
        .unwrap()
        .next;
        assert!((reversed.velocity().x_mps() + 0.2).abs() < 1.0e-12);
        assert_eq!(reversed.velocity().y_mps(), 0.0);
    }

    #[test]
    fn actuator_slowdown_uses_controlled_deceleration() {
        let state = RobotState::new(
            WorldPosition::new(1.0, 1.0).unwrap(),
            Velocity::new(1.0, 0.0).unwrap(),
            0.0,
        )
        .unwrap();
        let limits = MotionLimits::new(2.0, 10.0, 1.0, 4.0).unwrap();
        let next = preview_holonomic_motion(
            state,
            ActionCandidate::East,
            limits,
            ActuatorEffect::new(0.25, false).unwrap(),
        )
        .unwrap()
        .next;

        assert!((next.velocity().x_mps() - 0.9).abs() < 1.0e-12);
    }

    #[test]
    fn emergency_deceleration_cannot_be_weaker_than_controlled_deceleration() {
        assert_eq!(
            MotionLimits::new(2.0, 1.0, 2.0, 1.0),
            Err(ValidationError::OutOfRange(
                "motion.max_emergency_deceleration"
            ))
        );
    }
}
