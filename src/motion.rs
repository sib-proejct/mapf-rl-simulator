use crate::action::action_vector;
use crate::contracts::generated::ActionCandidate;
use crate::types::{
    Acceleration, CONTROL_TICK_SECONDS, MetersPerSecond, MetersPerSecondCubed,
    MetersPerSecondSquared, RobotState, SAFETY_EPSILON_METERS, ValidationError, Velocity,
    WorldPosition, ensure_non_negative, ensure_positive,
};
use ruckig::{ControlInterface, InputParameter, OutputParameter, Result as RuckigResult, Ruckig};
use std::fmt;

pub const RUCKIG_VERSION: &str = "0.19.4";
const SAFETY_SAMPLE_SECONDS: f64 = CONTROL_TICK_SECONDS;
const MAX_RUCKIG_TRAJECTORY_SECONDS: f64 = 7_000.0;
const MOTION_STATE_EPSILON: f64 = 1.0e-8;

/// A stop endpoint and an optional route-derived cruise speed (metres/second).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionTarget {
    pub position: WorldPosition,
    pub cruise_speed_mps: Option<f64>,
}

impl From<WorldPosition> for MotionTarget {
    fn from(position: WorldPosition) -> Self {
        Self {
            position,
            cruise_speed_mps: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionLimits {
    max_linear_speed: MetersPerSecond,
    max_acceleration: MetersPerSecondSquared,
    max_deceleration: MetersPerSecondSquared,
    max_emergency_deceleration: MetersPerSecondSquared,
    max_jerk: MetersPerSecondCubed,
    max_emergency_jerk: MetersPerSecondCubed,
}

impl MotionLimits {
    pub fn new(
        max_linear_speed_mps: f64,
        max_acceleration_mps2: f64,
        max_deceleration_mps2: f64,
        max_emergency_deceleration_mps2: f64,
        max_jerk_mps3: f64,
        max_emergency_jerk_mps3: f64,
    ) -> Result<Self, ValidationError> {
        ensure_positive(max_linear_speed_mps, "motion.max_linear_speed")?;
        ensure_positive(max_acceleration_mps2, "motion.max_acceleration")?;
        ensure_positive(max_deceleration_mps2, "motion.max_deceleration")?;
        ensure_positive(
            max_emergency_deceleration_mps2,
            "motion.max_emergency_deceleration",
        )?;
        ensure_positive(max_jerk_mps3, "motion.max_jerk")?;
        ensure_positive(max_emergency_jerk_mps3, "motion.max_emergency_jerk")?;
        if max_emergency_deceleration_mps2 < max_deceleration_mps2 {
            return Err(ValidationError::OutOfRange(
                "motion.max_emergency_deceleration",
            ));
        }
        if max_emergency_jerk_mps3 < max_jerk_mps3 {
            return Err(ValidationError::OutOfRange("motion.max_emergency_jerk"));
        }
        Ok(Self {
            max_linear_speed: MetersPerSecond::new(max_linear_speed_mps)?,
            max_acceleration: MetersPerSecondSquared::new(max_acceleration_mps2)?,
            max_deceleration: MetersPerSecondSquared::new(max_deceleration_mps2)?,
            max_emergency_deceleration: MetersPerSecondSquared::new(
                max_emergency_deceleration_mps2,
            )?,
            max_jerk: MetersPerSecondCubed::new(max_jerk_mps3)?,
            max_emergency_jerk: MetersPerSecondCubed::new(max_emergency_jerk_mps3)?,
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

    pub const fn max_jerk_mps3(self) -> f64 {
        self.max_jerk.get()
    }

    pub const fn max_emergency_jerk_mps3(self) -> f64 {
        self.max_emergency_jerk.get()
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
pub struct SweptSegment {
    pub start: WorldPosition,
    pub end: WorldPosition,
    pub curvature_margin_meters: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MotionPreview {
    pub previous: RobotState,
    pub next: RobotState,
    pub swept_segments: Vec<SweptSegment>,
    pub acceleration_limit_mps2: f64,
    pub jerk_limit_mps3: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionError {
    InvalidState,
    InvalidInput,
    TrajectoryDuration,
    CalculationFailed,
    InvalidOutput,
}

impl fmt::Display for MotionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::InvalidState => "motion state is incompatible with the cardinal path model",
            Self::InvalidInput => "Ruckig rejected the motion input",
            Self::TrajectoryDuration => "Ruckig trajectory duration is invalid",
            Self::CalculationFailed => "Ruckig could not calculate a trajectory",
            Self::InvalidOutput => "Ruckig returned invalid motion output",
        })
    }
}

impl std::error::Error for MotionError {}

#[derive(Clone, Copy)]
struct PathState {
    direction_x: f64,
    direction_y: f64,
    velocity_mps: f64,
    acceleration_mps2: f64,
}

pub fn initial_kinematics_are_valid(state: RobotState, limits: MotionLimits) -> bool {
    let acceleration_limit = limits
        .max_acceleration_mps2()
        .max(limits.max_deceleration_mps2());
    state.velocity().magnitude() <= limits.max_linear_speed_mps() + SAFETY_EPSILON_METERS
        && state.acceleration().magnitude() <= acceleration_limit + SAFETY_EPSILON_METERS
        && path_state(state, (0.0, 0.0)).is_ok()
}

pub fn preview_holonomic_motion(
    state: RobotState,
    action: ActionCandidate,
    limits: MotionLimits,
    actuator: ActuatorEffect,
) -> Result<MotionPreview, MotionError> {
    let desired_direction = action_vector(action);
    let current = path_state(state, desired_direction)?;
    let desired_speed = if actuator.stuck || desired_direction == (0.0, 0.0) {
        0.0
    } else {
        limits.max_linear_speed_mps() * actuator.speed_scale
    };
    let aligned = desired_direction != (0.0, 0.0)
        && direction_is_aligned(
            (current.direction_x, current.direction_y),
            desired_direction,
        );
    let target_velocity = if desired_direction == (0.0, 0.0) {
        0.0
    } else if aligned || current.velocity_mps == 0.0 {
        desired_speed
    } else {
        0.0
    };
    let decelerating = !aligned || target_velocity < current.velocity_mps;
    let acceleration_limit = if decelerating {
        limits.max_deceleration_mps2()
    } else {
        limits.max_acceleration_mps2()
    };
    preview_velocity_transition(
        state,
        current,
        target_velocity,
        acceleration_limit,
        limits.max_jerk_mps3(),
        limits.max_linear_speed_mps(),
    )
}

/// Position-controlled approach to one cardinal node center, ending at rest.
/// Direction changes still brake on the existing axis before turning.
pub fn preview_motion_to_target(
    state: RobotState,
    action: ActionCandidate,
    target: WorldPosition,
    limits: MotionLimits,
    actuator: ActuatorEffect,
) -> Result<MotionPreview, MotionError> {
    preview_motion_to_route_target(state, action, target.into(), limits, actuator)
}

pub fn preview_motion_to_route_target(
    state: RobotState,
    action: ActionCandidate,
    target: MotionTarget,
    limits: MotionLimits,
    actuator: ActuatorEffect,
) -> Result<MotionPreview, MotionError> {
    let cruise_speed = target
        .cruise_speed_mps
        .unwrap_or(limits.max_linear_speed_mps());
    ensure_positive(cruise_speed, "motion.cruise_speed").map_err(|_| MotionError::InvalidInput)?;
    let speed_limit = cruise_speed.min(limits.max_linear_speed_mps());
    let direction = action_vector(action);
    let current = path_state(state, direction)?;
    if action == ActionCandidate::Wait
        || actuator.stuck()
        || (current.velocity_mps > 0.0
            && !direction_is_aligned((current.direction_x, current.direction_y), direction))
    {
        return preview_holonomic_motion(state, action, limits, actuator);
    }
    let dx = target.position.x_meters() - state.position().x_meters();
    let dy = target.position.y_meters() - state.position().y_meters();
    let distance = dx * direction.0 + dy * direction.1;
    if distance < -SAFETY_EPSILON_METERS {
        return Err(MotionError::InvalidInput);
    }
    let acceleration_limit = limits
        .max_acceleration_mps2()
        .min(limits.max_deceleration_mps2());
    let mut input = ruckig_input(
        current,
        0.0,
        acceleration_limit,
        limits.max_jerk_mps3(),
        limits,
    )?;
    input.control_interface = ControlInterface::Position;
    input.target_position = vec![distance.max(0.0)];
    input.max_velocity = vec![speed_limit * actuator.speed_scale];
    let mut output = OutputParameter::new_direct(1);
    let mut ruckig = Ruckig::new_direct(1, CONTROL_TICK_SECONDS);
    ensure_ruckig_success(ruckig.update(&mut input, &mut output))?;
    preview_from_output(
        state,
        current,
        &output,
        acceleration_limit,
        limits.max_jerk_mps3(),
        limits.max_linear_speed_mps(),
    )
}

pub fn preview_stop(
    state: RobotState,
    deceleration_mps2: f64,
    jerk_mps3: f64,
    limits: MotionLimits,
) -> Result<MotionPreview, MotionError> {
    ensure_positive(deceleration_mps2, "motion.stop_deceleration")
        .map_err(|_| MotionError::InvalidInput)?;
    ensure_positive(jerk_mps3, "motion.stop_jerk").map_err(|_| MotionError::InvalidInput)?;
    let current = path_state(state, (0.0, 0.0))?;
    preview_velocity_transition(
        state,
        current,
        0.0,
        deceleration_mps2,
        jerk_mps3,
        limits.max_linear_speed_mps(),
    )
}

pub fn preview_stop_trajectory(
    state: RobotState,
    deceleration_mps2: f64,
    jerk_mps3: f64,
    limits: MotionLimits,
) -> Result<Vec<SweptSegment>, MotionError> {
    let current = path_state(state, (0.0, 0.0))?;
    if current.velocity_mps == 0.0 {
        return Ok(Vec::new());
    }
    let mut input = ruckig_input(current, 0.0, deceleration_mps2, jerk_mps3, limits)?;
    let mut trajectory = ruckig::Trajectory::new_direct(1);
    let mut ruckig = Ruckig::new_direct_and_offline(1);
    ensure_ruckig_success(ruckig.calculate(&mut input, &mut trajectory))?;
    let duration = trajectory.get_duration();
    if !duration.is_finite() || !(0.0..=MAX_RUCKIG_TRAJECTORY_SECONDS).contains(&duration) {
        return Err(MotionError::TrajectoryDuration);
    }

    let mut segments = Vec::with_capacity((duration / SAFETY_SAMPLE_SECONDS).ceil() as usize);
    let mut previous_time = 0.0;
    let mut previous_position = state.position();
    while previous_time < duration {
        let next_time = (previous_time + SAFETY_SAMPLE_SECONDS).min(duration);
        let (positions, velocities, accelerations) = trajectory.at_time(next_time);
        validate_sample(&positions, &velocities, &accelerations)?;
        if velocities[0] < -MOTION_STATE_EPSILON
            || velocities[0]
                > current.velocity_mps.max(limits.max_linear_speed_mps()) + SAFETY_EPSILON_METERS
        {
            return Err(MotionError::InvalidOutput);
        }
        let next_position = projected_position(state.position(), current, positions[0])?;
        segments.push(SweptSegment {
            start: previous_position,
            end: next_position,
            curvature_margin_meters: curvature_margin(
                current.acceleration_mps2,
                deceleration_mps2,
                next_time - previous_time,
            ),
        });
        previous_time = next_time;
        previous_position = next_position;
    }
    let (_, final_velocity, final_acceleration) = trajectory.at_time(duration);
    if final_velocity.len() != 1
        || final_acceleration.len() != 1
        || final_velocity[0].abs() > MOTION_STATE_EPSILON
        || final_acceleration[0].abs() > MOTION_STATE_EPSILON
    {
        return Err(MotionError::InvalidOutput);
    }
    Ok(segments)
}

fn preview_velocity_transition(
    state: RobotState,
    current: PathState,
    target_velocity_mps: f64,
    acceleration_limit_mps2: f64,
    jerk_limit_mps3: f64,
    max_linear_speed_mps: f64,
) -> Result<MotionPreview, MotionError> {
    let temporary_limits = MotionLimits::new(
        max_linear_speed_mps,
        acceleration_limit_mps2,
        acceleration_limit_mps2,
        acceleration_limit_mps2,
        jerk_limit_mps3,
        jerk_limit_mps3,
    )
    .map_err(|_| MotionError::InvalidInput)?;
    let mut input = ruckig_input(
        current,
        target_velocity_mps,
        acceleration_limit_mps2,
        jerk_limit_mps3,
        temporary_limits,
    )?;
    let mut output = OutputParameter::new_direct(1);
    let mut ruckig = Ruckig::new_direct(1, CONTROL_TICK_SECONDS);
    ensure_ruckig_success(ruckig.update(&mut input, &mut output))?;
    preview_from_output(
        state,
        current,
        &output,
        acceleration_limit_mps2,
        jerk_limit_mps3,
        max_linear_speed_mps,
    )
}

fn preview_from_output(
    state: RobotState,
    current: PathState,
    output: &OutputParameter,
    acceleration_limit_mps2: f64,
    jerk_limit_mps3: f64,
    max_linear_speed_mps: f64,
) -> Result<MotionPreview, MotionError> {
    validate_sample(
        &output.new_position,
        &output.new_velocity,
        &output.new_acceleration,
    )?;
    if output.new_velocity[0] < -MOTION_STATE_EPSILON
        || output.new_velocity[0]
            > current.velocity_mps.max(max_linear_speed_mps) + SAFETY_EPSILON_METERS
    {
        return Err(MotionError::InvalidOutput);
    }

    let position = projected_position(state.position(), current, output.new_position[0])?;
    let scalar_velocity = canonical_zero(output.new_velocity[0]);
    let scalar_acceleration = canonical_zero(output.new_acceleration[0]);
    let velocity = Velocity::new(
        current.direction_x * scalar_velocity,
        current.direction_y * scalar_velocity,
    )
    .map_err(|_| MotionError::InvalidOutput)?;
    let acceleration = Acceleration::new(
        current.direction_x * scalar_acceleration,
        current.direction_y * scalar_acceleration,
    )
    .map_err(|_| MotionError::InvalidOutput)?;
    let next = state.transitioned(position, velocity, acceleration);
    let swept_segments =
        sampled_tick_segments(state.position(), current, output, acceleration_limit_mps2)?;
    Ok(MotionPreview {
        previous: state,
        next,
        swept_segments,
        acceleration_limit_mps2,
        jerk_limit_mps3,
    })
}

fn ruckig_input(
    current: PathState,
    target_velocity_mps: f64,
    acceleration_limit_mps2: f64,
    jerk_limit_mps3: f64,
    limits: MotionLimits,
) -> Result<InputParameter, MotionError> {
    if !target_velocity_mps.is_finite()
        || target_velocity_mps < 0.0
        || target_velocity_mps > limits.max_linear_speed_mps() + SAFETY_EPSILON_METERS
    {
        return Err(MotionError::InvalidInput);
    }
    let mut input = InputParameter::new(1);
    input.control_interface = ControlInterface::Velocity;
    input.current_position = vec![0.0];
    input.current_velocity = vec![current.velocity_mps];
    input.current_acceleration = vec![current.acceleration_mps2];
    input.target_velocity = vec![target_velocity_mps];
    input.target_acceleration = vec![0.0];
    input.max_velocity = vec![limits.max_linear_speed_mps()];
    input.max_acceleration = vec![acceleration_limit_mps2];
    input.max_jerk = vec![jerk_limit_mps3];
    Ok(input)
}

fn sampled_tick_segments(
    origin: WorldPosition,
    current: PathState,
    output: &OutputParameter,
    acceleration_limit_mps2: f64,
) -> Result<Vec<SweptSegment>, MotionError> {
    const SAMPLE_COUNT: usize = 4;
    let mut segments = Vec::with_capacity(SAMPLE_COUNT);
    let mut previous_position = origin;
    for sample in 1..=SAMPLE_COUNT {
        let time = CONTROL_TICK_SECONDS * sample as f64 / SAMPLE_COUNT as f64;
        let local_position = if sample == SAMPLE_COUNT {
            output.new_position[0]
        } else {
            let (positions, velocities, accelerations) = output.trajectory().at_time(time);
            validate_sample(&positions, &velocities, &accelerations)?;
            positions[0]
        };
        let next_position = projected_position(origin, current, local_position)?;
        let interval = CONTROL_TICK_SECONDS / SAMPLE_COUNT as f64;
        segments.push(SweptSegment {
            start: previous_position,
            end: next_position,
            curvature_margin_meters: curvature_margin(
                current.acceleration_mps2,
                acceleration_limit_mps2,
                interval,
            ),
        });
        previous_position = next_position;
    }
    Ok(segments)
}

fn path_state(
    state: RobotState,
    preferred_direction: (f64, f64),
) -> Result<PathState, MotionError> {
    let velocity = state.velocity();
    let acceleration = state.acceleration();
    let speed = velocity.magnitude();
    if speed <= MOTION_STATE_EPSILON {
        if acceleration.magnitude() > MOTION_STATE_EPSILON {
            return Err(MotionError::InvalidState);
        }
        let direction = if preferred_direction == (0.0, 0.0) {
            (1.0, 0.0)
        } else {
            preferred_direction
        };
        return Ok(PathState {
            direction_x: direction.0,
            direction_y: direction.1,
            velocity_mps: 0.0,
            acceleration_mps2: 0.0,
        });
    }
    let direction_x = velocity.x_mps() / speed;
    let direction_y = velocity.y_mps() / speed;
    let scalar_acceleration =
        acceleration.x_mps2() * direction_x + acceleration.y_mps2() * direction_y;
    let perpendicular_acceleration =
        acceleration.x_mps2() * direction_y - acceleration.y_mps2() * direction_x;
    if perpendicular_acceleration.abs() > MOTION_STATE_EPSILON {
        return Err(MotionError::InvalidState);
    }
    Ok(PathState {
        direction_x,
        direction_y,
        velocity_mps: speed,
        acceleration_mps2: scalar_acceleration,
    })
}

fn direction_is_aligned(current: (f64, f64), desired: (f64, f64)) -> bool {
    current.0 * desired.0 + current.1 * desired.1 >= 1.0 - SAFETY_EPSILON_METERS
}

fn projected_position(
    origin: WorldPosition,
    path: PathState,
    displacement_meters: f64,
) -> Result<WorldPosition, MotionError> {
    WorldPosition::new(
        origin.x_meters() + path.direction_x * displacement_meters,
        origin.y_meters() + path.direction_y * displacement_meters,
    )
    .map_err(|_| MotionError::InvalidOutput)
}

fn curvature_margin(current_acceleration_mps2: f64, limit_mps2: f64, dt_seconds: f64) -> f64 {
    current_acceleration_mps2.abs().max(limit_mps2) * dt_seconds * dt_seconds / 8.0
}

fn canonical_zero(value: f64) -> f64 {
    if value.abs() <= MOTION_STATE_EPSILON {
        0.0
    } else {
        value
    }
}

fn validate_sample(
    position: &[f64],
    velocity: &[f64],
    acceleration: &[f64],
) -> Result<(), MotionError> {
    if position.len() != 1
        || velocity.len() != 1
        || acceleration.len() != 1
        || !position[0].is_finite()
        || !velocity[0].is_finite()
        || !acceleration[0].is_finite()
    {
        return Err(MotionError::InvalidOutput);
    }
    Ok(())
}

fn ensure_ruckig_success(result: RuckigResult) -> Result<(), MotionError> {
    match result {
        RuckigResult::Working | RuckigResult::Finished => Ok(()),
        RuckigResult::ErrorInvalidInput | RuckigResult::ErrorZeroLimits => {
            Err(MotionError::InvalidInput)
        }
        RuckigResult::ErrorTrajectoryDuration => Err(MotionError::TrajectoryDuration),
        RuckigResult::Error
        | RuckigResult::ErrorPositionalLimits
        | RuckigResult::ErrorExecutionTimeCalculation
        | RuckigResult::ErrorSynchronizationCalculation => Err(MotionError::CalculationFailed),
    }
}

impl crate::contracts::provisioning_generated::MotionProfileLimits {
    pub fn motion_limits(&self) -> Result<MotionLimits, ValidationError> {
        for (value, upper) in [
            (self.max_linear_speed_mps, 3.0),
            (self.max_linear_acceleration_mps2, 3.0),
            (self.max_linear_deceleration_mps2, 3.0),
            (self.max_linear_jerk_mps3, 30.0),
        ] {
            if !value.is_finite() || !(0.1..=upper).contains(&value) {
                return Err(ValidationError::OutOfRange("motion.profile"));
            }
        }
        MotionLimits::new(
            self.max_linear_speed_mps,
            self.max_linear_acceleration_mps2,
            self.max_linear_deceleration_mps2,
            6.0,
            self.max_linear_jerk_mps3,
            60.0,
        )
    }

    pub fn from_motion_limits(limits: MotionLimits) -> Self {
        Self {
            max_linear_speed_mps: limits.max_linear_speed_mps(),
            max_linear_acceleration_mps2: limits.max_acceleration_mps2(),
            max_linear_deceleration_mps2: limits.max_deceleration_mps2(),
            max_linear_jerk_mps3: limits.max_jerk_mps3(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(velocity: Velocity, acceleration: Acceleration) -> RobotState {
        RobotState::new(
            WorldPosition::new(1.0, 1.0).unwrap(),
            velocity,
            acceleration,
            0.25,
        )
        .unwrap()
    }

    fn limits() -> MotionLimits {
        MotionLimits::new(2.0, 2.0, 3.0, 6.0, 10.0, 30.0).unwrap()
    }

    #[test]
    fn heading_follows_cardinal_velocity_and_is_retained_at_rest() {
        for (x, y, heading) in [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, std::f64::consts::FRAC_PI_2),
            (-1.0, 0.0, std::f64::consts::PI),
            (0.0, -1.0, -std::f64::consts::FRAC_PI_2),
        ] {
            let initial = state(Velocity::ZERO, Acceleration::ZERO);
            let moving = initial.transitioned(
                initial.position(),
                Velocity::new(x, y).unwrap(),
                Acceleration::ZERO,
            );
            assert_eq!(moving.yaw_radians(), heading);
            let stopped =
                moving.transitioned(moving.position(), Velocity::ZERO, Acceleration::ZERO);
            assert_eq!(stopped.yaw_radians(), heading);
        }
    }

    #[test]
    fn position_control_reaches_and_stops_at_each_cardinal_center() {
        let origin = WorldPosition::new(3.25, -2.75).unwrap();
        for (dx, dy) in [(0.5, 0.0), (-0.5, 0.0), (0.0, 0.5), (0.0, -0.5)] {
            let target =
                WorldPosition::new(origin.x_meters() + dx, origin.y_meters() + dy).unwrap();
            let mut current =
                RobotState::new(origin, Velocity::ZERO, Acceleration::ZERO, 0.0).unwrap();
            for _ in 0..100 {
                let action = crate::action::ActionAdapter::from_index(
                    crate::route::action_toward_position(current.position(), target),
                )
                .candidate();
                let preview = preview_motion_to_target(
                    current,
                    action,
                    target,
                    limits(),
                    ActuatorEffect::nominal(),
                )
                .unwrap();
                assert!(
                    preview.next.velocity().magnitude() <= limits().max_linear_speed_mps() + 1.0e-8
                );
                assert!(
                    preview.next.acceleration().magnitude()
                        <= limits().max_acceleration_mps2() + 1.0e-8
                );
                let jerk = (preview.next.acceleration().x_mps2() - current.acceleration().x_mps2())
                    .hypot(preview.next.acceleration().y_mps2() - current.acceleration().y_mps2())
                    / CONTROL_TICK_SECONDS;
                assert!(jerk <= limits().max_jerk_mps3() + 1.0e-8);
                current = preview.next;
            }
            assert!((current.position().x_meters() - target.x_meters()).abs() < 1.0e-6);
            assert!((current.position().y_meters() - target.y_meters()).abs() < 1.0e-6);
            assert_eq!(current.velocity(), Velocity::ZERO);
            assert_eq!(current.acceleration(), Acceleration::ZERO);
        }
    }

    #[test]
    fn acceleration_is_jerk_limited() {
        let preview = preview_holonomic_motion(
            state(Velocity::ZERO, Acceleration::ZERO),
            ActionCandidate::East,
            limits(),
            ActuatorEffect::nominal(),
        )
        .unwrap();
        assert!(preview.next.velocity().x_mps() > 0.0);
        assert!(preview.next.acceleration().x_mps2() <= 1.0 + 1.0e-12);
        assert!(
            preview.next.acceleration().magnitude() / CONTROL_TICK_SECONDS
                <= limits().max_jerk_mps3() + 1.0e-12
        );
        assert_eq!(preview.next.yaw_radians(), 0.0);
    }

    #[test]
    fn wait_from_rest_keeps_the_robot_stopped() {
        let preview = preview_holonomic_motion(
            state(Velocity::ZERO, Acceleration::ZERO),
            ActionCandidate::Wait,
            limits(),
            ActuatorEffect::nominal(),
        )
        .unwrap();
        assert_eq!(preview.next.position(), preview.previous.position());
        assert_eq!(preview.next.velocity(), Velocity::ZERO);
        assert_eq!(preview.next.acceleration(), Acceleration::ZERO);
    }

    #[test]
    fn repeated_action_reaches_the_target_velocity() {
        let mut current = state(Velocity::ZERO, Acceleration::ZERO);
        for _ in 0..30 {
            current = preview_holonomic_motion(
                current,
                ActionCandidate::East,
                limits(),
                ActuatorEffect::nominal(),
            )
            .unwrap()
            .next;
        }
        assert!((current.velocity().x_mps() - limits().max_linear_speed_mps()).abs() < 1.0e-12);
        assert_eq!(current.acceleration(), Acceleration::ZERO);
        let cruising = preview_holonomic_motion(
            current,
            ActionCandidate::East,
            limits(),
            ActuatorEffect::nominal(),
        )
        .unwrap()
        .next;
        assert!(cruising.position().x_meters() > current.position().x_meters());
    }

    #[test]
    fn slowdown_and_emergency_stop_use_distinct_profiles() {
        let moving = state(Velocity::new(1.0, 0.0).unwrap(), Acceleration::ZERO);
        let slowed = preview_holonomic_motion(
            moving,
            ActionCandidate::East,
            limits(),
            ActuatorEffect::new(0.25, false).unwrap(),
        )
        .unwrap();
        let controlled = preview_stop(
            moving,
            limits().max_deceleration_mps2(),
            limits().max_jerk_mps3(),
            limits(),
        )
        .unwrap();
        let emergency = preview_stop(
            moving,
            limits().max_emergency_deceleration_mps2(),
            limits().max_emergency_jerk_mps3(),
            limits(),
        )
        .unwrap();
        assert!(slowed.next.velocity().x_mps() < moving.velocity().x_mps());
        assert!(emergency.next.velocity().x_mps() < controlled.next.velocity().x_mps());
        assert!(emergency.next.position().x_meters() < controlled.next.position().x_meters());
    }

    #[test]
    fn perpendicular_direction_brakes_before_accelerating() {
        let preview = preview_holonomic_motion(
            state(Velocity::new(1.0, 0.0).unwrap(), Acceleration::ZERO),
            ActionCandidate::North,
            limits(),
            ActuatorEffect::nominal(),
        )
        .unwrap();
        assert_eq!(preview.next.velocity().y_mps(), 0.0);
        assert!(preview.next.velocity().x_mps() < 1.0);
    }

    #[test]
    fn new_direction_accelerates_on_the_tick_after_a_full_stop() {
        let mut current = state(Velocity::new(1.0, 0.0).unwrap(), Acceleration::ZERO);
        for _ in 0..30 {
            let next = preview_holonomic_motion(
                current,
                ActionCandidate::North,
                limits(),
                ActuatorEffect::nominal(),
            )
            .unwrap()
            .next;
            assert_eq!(next.velocity().y_mps(), 0.0);
            current = next;
            if current.velocity() == Velocity::ZERO {
                break;
            }
        }
        assert_eq!(current.velocity(), Velocity::ZERO);
        let north = preview_holonomic_motion(
            current,
            ActionCandidate::North,
            limits(),
            ActuatorEffect::nominal(),
        )
        .unwrap()
        .next;
        assert!(north.velocity().y_mps() > 0.0);
        assert_eq!(north.velocity().x_mps(), 0.0);
    }

    #[test]
    fn non_parallel_acceleration_is_rejected() {
        let invalid = state(
            Velocity::new(1.0, 0.0).unwrap(),
            Acceleration::new(0.0, 0.1).unwrap(),
        );
        assert_eq!(
            preview_holonomic_motion(
                invalid,
                ActionCandidate::East,
                limits(),
                ActuatorEffect::nominal(),
            ),
            Err(MotionError::InvalidState)
        );
    }

    #[test]
    fn malformed_ruckig_output_is_rejected() {
        assert_eq!(
            validate_sample(&[], &[0.0], &[0.0]),
            Err(MotionError::InvalidOutput)
        );
        assert_eq!(
            validate_sample(&[f64::NAN], &[0.0], &[0.0]),
            Err(MotionError::InvalidOutput)
        );
        assert_eq!(
            ensure_ruckig_success(RuckigResult::ErrorInvalidInput),
            Err(MotionError::InvalidInput)
        );
    }

    #[test]
    fn emergency_limits_must_dominate_controlled_limits() {
        assert_eq!(
            MotionLimits::new(2.0, 1.0, 2.0, 1.0, 10.0, 20.0),
            Err(ValidationError::OutOfRange(
                "motion.max_emergency_deceleration"
            ))
        );
        assert_eq!(
            MotionLimits::new(2.0, 1.0, 2.0, 3.0, 20.0, 10.0),
            Err(ValidationError::OutOfRange("motion.max_emergency_jerk"))
        );
    }
}
