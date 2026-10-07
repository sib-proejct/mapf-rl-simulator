use crate::action::AdaptedAction;
use crate::contracts::generated::ActionCandidate;
use crate::motion::{
    ActuatorEffect, MotionLimits, MotionPreview, MotionTarget, SweptSegment,
    preview_holonomic_motion, preview_motion_to_route_target, preview_stop,
    preview_stop_trajectory,
};
use crate::types::{
    CONTROL_TICK_SECONDS, Meters, RobotState, SAFETY_EPSILON_METERS, ValidationError,
    WorldPosition, ensure_non_negative, ensure_positive,
};
use crate::world::GridMap;
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SafetyConfig {
    footprint_radius: Meters,
    minimum_obstacle_clearance: Meters,
}

impl SafetyConfig {
    pub fn new(
        footprint_radius_meters: f64,
        minimum_obstacle_clearance_meters: f64,
    ) -> Result<Self, ValidationError> {
        ensure_positive(footprint_radius_meters, "safety.footprint_radius")?;
        ensure_non_negative(
            minimum_obstacle_clearance_meters,
            "safety.minimum_obstacle_clearance",
        )?;
        Ok(Self {
            footprint_radius: Meters::new(footprint_radius_meters)?,
            minimum_obstacle_clearance: Meters::new(minimum_obstacle_clearance_meters)?,
        })
    }

    pub const fn footprint_radius_meters(self) -> f64 {
        self.footprint_radius.get()
    }

    pub const fn minimum_obstacle_clearance_meters(self) -> f64 {
        self.minimum_obstacle_clearance.get()
    }

    fn required_clearance(self) -> f64 {
        self.footprint_radius.get() + self.minimum_obstacle_clearance.get() + SAFETY_EPSILON_METERS
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SafetyOutcome {
    Accept,
    SubstituteWait,
    ControlledStop,
    EmergencyStop,
    RejectNotReady,
}

impl SafetyOutcome {
    pub const fn code(self) -> &'static str {
        match self {
            Self::Accept => "accept",
            Self::SubstituteWait => "substitute_wait",
            Self::ControlledStop => "controlled_stop",
            Self::EmergencyStop => "emergency_stop",
            Self::RejectNotReady => "reject_not_ready",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SafetyReason {
    None,
    InvalidAction,
    MapBoundary,
    StaticObstacleClearance,
    ActuatorStuck,
    EmergencyStopLatched,
    InvalidMotionState,
    KinematicLimit,
    CollisionInvariant,
}

impl SafetyReason {
    pub const fn code(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::InvalidAction => "invalid_action",
            Self::MapBoundary => "map_boundary",
            Self::StaticObstacleClearance => "static_obstacle_clearance",
            Self::ActuatorStuck => "actuator_stuck",
            Self::EmergencyStopLatched => "emergency_stop_latched",
            Self::InvalidMotionState => "invalid_motion_state",
            Self::KinematicLimit => "kinematic_limit",
            Self::CollisionInvariant => "collision_invariant",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SafetyDecision {
    pub requested_action: ActionCandidate,
    pub applied_action: ActionCandidate,
    pub outcome: SafetyOutcome,
    pub reason: SafetyReason,
    pub next_state: RobotState,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SafetyError {
    NoSafeEmergencyTransition,
}

impl fmt::Display for SafetyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoSafeEmergencyTransition => {
                formatter.write_str("no collision-free emergency braking transition exists")
            }
        }
    }
}

impl std::error::Error for SafetyError {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SafetyKernel {
    config: SafetyConfig,
}

impl SafetyKernel {
    pub const fn new(config: SafetyConfig) -> Self {
        Self { config }
    }

    pub const fn config(self) -> SafetyConfig {
        self.config
    }

    pub fn decide(
        &self,
        map: &GridMap,
        state: RobotState,
        adapted: AdaptedAction,
        motion_limits: MotionLimits,
        actuator: ActuatorEffect,
        emergency_stop_latched: bool,
    ) -> Result<SafetyDecision, SafetyError> {
        self.decide_with_target(
            map,
            state,
            adapted,
            motion_limits,
            actuator,
            emergency_stop_latched,
            None,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn decide_with_target(
        &self,
        map: &GridMap,
        state: RobotState,
        adapted: AdaptedAction,
        motion_limits: MotionLimits,
        actuator: ActuatorEffect,
        emergency_stop_latched: bool,
        target: Option<MotionTarget>,
    ) -> Result<SafetyDecision, SafetyError> {
        let requested = adapted.candidate();
        if emergency_stop_latched {
            return self.emergency_stop_decision(
                map,
                state,
                requested,
                SafetyReason::EmergencyStopLatched,
                motion_limits,
            );
        }
        if actuator.stuck() {
            return self.controlled_stop_decision(
                map,
                state,
                requested,
                SafetyReason::ActuatorStuck,
                motion_limits,
            );
        }
        if state.velocity().magnitude()
            > motion_limits.max_linear_speed_mps() + SAFETY_EPSILON_METERS
        {
            return self.controlled_stop_decision(
                map,
                state,
                requested,
                SafetyReason::KinematicLimit,
                motion_limits,
            );
        }
        let requested_preview = match target.map_or_else(
            || preview_holonomic_motion(state, requested, motion_limits, actuator),
            |target| {
                preview_motion_to_route_target(state, requested, target, motion_limits, actuator)
            },
        ) {
            Ok(preview) => preview,
            Err(_) => {
                return self.emergency_stop_decision(
                    map,
                    state,
                    requested,
                    SafetyReason::InvalidMotionState,
                    motion_limits,
                );
            }
        };
        if !kinematics_are_valid(&requested_preview, motion_limits) {
            return self.controlled_stop_decision(
                map,
                state,
                requested,
                SafetyReason::KinematicLimit,
                motion_limits,
            );
        }
        let requested_clearance = self.motion_clearance(map, &requested_preview);
        if !adapted.invalid_input()
            && requested_clearance.is_ok()
            && self.emergency_stopping_clear(map, requested_preview.next, motion_limits)
        {
            return Ok(SafetyDecision {
                requested_action: requested,
                applied_action: requested,
                outcome: SafetyOutcome::Accept,
                reason: SafetyReason::None,
                next_state: requested_preview.next,
            });
        }

        let fallback_reason = if adapted.invalid_input() {
            SafetyReason::InvalidAction
        } else {
            requested_clearance
                .err()
                .unwrap_or(SafetyReason::StaticObstacleClearance)
        };
        let wait_preview =
            match preview_holonomic_motion(state, ActionCandidate::Wait, motion_limits, actuator) {
                Ok(preview) => preview,
                Err(_) => {
                    return self.emergency_stop_decision(
                        map,
                        state,
                        requested,
                        SafetyReason::InvalidMotionState,
                        motion_limits,
                    );
                }
            };
        if !kinematics_are_valid(&wait_preview, motion_limits) {
            return self.emergency_stop_decision(
                map,
                state,
                requested,
                SafetyReason::KinematicLimit,
                motion_limits,
            );
        }
        if self.motion_clearance(map, &wait_preview).is_ok()
            && self.emergency_stopping_clear(map, wait_preview.next, motion_limits)
        {
            Ok(SafetyDecision {
                requested_action: requested,
                applied_action: ActionCandidate::Wait,
                outcome: SafetyOutcome::SubstituteWait,
                reason: fallback_reason,
                next_state: wait_preview.next,
            })
        } else {
            self.emergency_stop_decision(map, state, requested, fallback_reason, motion_limits)
        }
    }

    pub fn state_is_clear(&self, map: &GridMap, state: RobotState) -> bool {
        self.segment_clearance(map, state.position(), state.position(), 0.0)
            .is_ok()
    }

    pub fn state_has_safe_emergency_stop(
        &self,
        map: &GridMap,
        state: RobotState,
        motion_limits: MotionLimits,
    ) -> bool {
        self.state_is_clear(map, state) && self.emergency_stopping_clear(map, state, motion_limits)
    }

    fn controlled_stop_decision(
        &self,
        map: &GridMap,
        state: RobotState,
        requested: ActionCandidate,
        reason: SafetyReason,
        limits: MotionLimits,
    ) -> Result<SafetyDecision, SafetyError> {
        let preview = preview_stop(
            state,
            limits.max_deceleration_mps2(),
            limits.max_jerk_mps3(),
            limits,
        )
        .map_err(|_| SafetyError::NoSafeEmergencyTransition)?;
        if kinematics_are_valid(&preview, limits)
            && self.motion_clearance(map, &preview).is_ok()
            && self.emergency_stopping_clear(map, preview.next, limits)
        {
            return Ok(SafetyDecision {
                requested_action: requested,
                applied_action: ActionCandidate::Wait,
                outcome: SafetyOutcome::ControlledStop,
                reason,
                next_state: preview.next,
            });
        }
        self.emergency_stop_decision(map, state, requested, reason, limits)
    }

    fn emergency_stop_decision(
        &self,
        map: &GridMap,
        state: RobotState,
        requested: ActionCandidate,
        reason: SafetyReason,
        limits: MotionLimits,
    ) -> Result<SafetyDecision, SafetyError> {
        let preview = preview_stop(
            state,
            limits.max_emergency_deceleration_mps2(),
            limits.max_emergency_jerk_mps3(),
            limits,
        )
        .map_err(|_| SafetyError::NoSafeEmergencyTransition)?;
        if !kinematics_are_valid(&preview, limits)
            || self.motion_clearance(map, &preview).is_err()
            || !self.emergency_stopping_clear(map, preview.next, limits)
        {
            return Err(SafetyError::NoSafeEmergencyTransition);
        }
        Ok(SafetyDecision {
            requested_action: requested,
            applied_action: ActionCandidate::Wait,
            outcome: SafetyOutcome::EmergencyStop,
            reason,
            next_state: preview.next,
        })
    }

    fn emergency_stopping_clear(
        &self,
        map: &GridMap,
        state: RobotState,
        limits: MotionLimits,
    ) -> bool {
        preview_stop_trajectory(
            state,
            limits.max_emergency_deceleration_mps2(),
            limits.max_emergency_jerk_mps3(),
            limits,
        )
        .is_ok_and(|segments| {
            segments
                .iter()
                .all(|segment| self.swept_segment_clear(map, *segment).is_ok())
        })
    }

    fn motion_clearance(&self, map: &GridMap, preview: &MotionPreview) -> Result<(), SafetyReason> {
        preview
            .swept_segments
            .iter()
            .try_for_each(|segment| self.swept_segment_clear(map, *segment))
    }

    fn swept_segment_clear(
        &self,
        map: &GridMap,
        segment: SweptSegment,
    ) -> Result<(), SafetyReason> {
        self.segment_clearance(
            map,
            segment.start,
            segment.end,
            segment.curvature_margin_meters,
        )
    }

    fn segment_clearance(
        &self,
        map: &GridMap,
        start: WorldPosition,
        end: WorldPosition,
        trajectory_margin_meters: f64,
    ) -> Result<(), SafetyReason> {
        let required = self.config.required_clearance() + trajectory_margin_meters;
        let (world_min_x, world_min_y, world_max_x, world_max_y) = map.world_bounds();
        for position in [start, end] {
            if position.x_meters() - world_min_x < required
                || world_max_x - position.x_meters() < required
                || position.y_meters() - world_min_y < required
                || world_max_y - position.y_meters() < required
            {
                return Err(SafetyReason::MapBoundary);
            }
        }
        let required_squared = required * required;
        let swept_bounds = (
            start.x_meters().min(end.x_meters()) - required,
            start.y_meters().min(end.y_meters()) - required,
            start.x_meters().max(end.x_meters()) + required,
            start.y_meters().max(end.y_meters()) + required,
        );
        for cell in map.blocked_cells_in_bounds(swept_bounds) {
            let bounds = map.cell_bounds(cell);
            if segment_aabb_distance_squared(start, end, bounds) < required_squared {
                return Err(SafetyReason::StaticObstacleClearance);
            }
        }
        Ok(())
    }
}

fn kinematics_are_valid(preview: &MotionPreview, limits: MotionLimits) -> bool {
    let previous_velocity = preview.previous.velocity();
    let next = preview.next.velocity();
    let previous_acceleration = preview.previous.acceleration();
    let next_acceleration = preview.next.acceleration();
    let acceleration_delta = (next_acceleration.x_mps2() - previous_acceleration.x_mps2())
        .hypot(next_acceleration.y_mps2() - previous_acceleration.y_mps2());
    let acceleration_bound = previous_acceleration
        .magnitude()
        .max(preview.acceleration_limit_mps2);
    let velocity_bound = previous_velocity
        .magnitude()
        .max(limits.max_linear_speed_mps());
    next.magnitude() <= velocity_bound + SAFETY_EPSILON_METERS
        && next_acceleration.magnitude() <= acceleration_bound + SAFETY_EPSILON_METERS
        && acceleration_delta / CONTROL_TICK_SECONDS
            <= preview.jerk_limit_mps3 + SAFETY_EPSILON_METERS
        && preview.next.yaw_radians()
            == if next.magnitude() > 1.0e-9 {
                next.y_mps().atan2(next.x_mps())
            } else {
                preview.previous.yaw_radians()
            }
}

fn segment_aabb_distance_squared(
    start: WorldPosition,
    end: WorldPosition,
    (min_x, min_y, max_x, max_y): (f64, f64, f64, f64),
) -> f64 {
    if segment_intersects_aabb(start, end, min_x, min_y, max_x, max_y) {
        return 0.0;
    }
    let corners = [
        (min_x, min_y, max_x, min_y),
        (max_x, min_y, max_x, max_y),
        (max_x, max_y, min_x, max_y),
        (min_x, max_y, min_x, min_y),
    ];
    corners
        .into_iter()
        .map(|(x1, y1, x2, y2)| {
            segment_segment_distance_squared(
                (start.x_meters(), start.y_meters()),
                (end.x_meters(), end.y_meters()),
                (x1, y1),
                (x2, y2),
            )
        })
        .fold(f64::INFINITY, f64::min)
}

fn segment_intersects_aabb(
    start: WorldPosition,
    end: WorldPosition,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
) -> bool {
    let dx = end.x_meters() - start.x_meters();
    let dy = end.y_meters() - start.y_meters();
    let mut lower = 0.0_f64;
    let mut upper = 1.0_f64;
    for (origin, delta, minimum, maximum) in [
        (start.x_meters(), dx, min_x, max_x),
        (start.y_meters(), dy, min_y, max_y),
    ] {
        if delta == 0.0 {
            if origin < minimum || origin > maximum {
                return false;
            }
        } else {
            let first = (minimum - origin) / delta;
            let second = (maximum - origin) / delta;
            lower = lower.max(first.min(second));
            upper = upper.min(first.max(second));
            if lower > upper {
                return false;
            }
        }
    }
    upper >= 0.0 && lower <= 1.0
}

fn segment_segment_distance_squared(
    p0: (f64, f64),
    p1: (f64, f64),
    q0: (f64, f64),
    q1: (f64, f64),
) -> f64 {
    if segments_intersect(p0, p1, q0, q1) {
        return 0.0;
    }
    point_segment_distance_squared(p0, q0, q1)
        .min(point_segment_distance_squared(p1, q0, q1))
        .min(point_segment_distance_squared(q0, p0, p1))
        .min(point_segment_distance_squared(q1, p0, p1))
}

fn segments_intersect(p0: (f64, f64), p1: (f64, f64), q0: (f64, f64), q1: (f64, f64)) -> bool {
    let o1 = cross(p0, p1, q0);
    let o2 = cross(p0, p1, q1);
    let o3 = cross(q0, q1, p0);
    let o4 = cross(q0, q1, p1);
    (o1 == 0.0 && point_on_segment(q0, p0, p1))
        || (o2 == 0.0 && point_on_segment(q1, p0, p1))
        || (o3 == 0.0 && point_on_segment(p0, q0, q1))
        || (o4 == 0.0 && point_on_segment(p1, q0, q1))
        || ((o1 > 0.0) != (o2 > 0.0) && (o3 > 0.0) != (o4 > 0.0))
}

fn cross(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

fn point_on_segment(point: (f64, f64), start: (f64, f64), end: (f64, f64)) -> bool {
    point.0 >= start.0.min(end.0)
        && point.0 <= start.0.max(end.0)
        && point.1 >= start.1.min(end.1)
        && point.1 <= start.1.max(end.1)
}

fn point_segment_distance_squared(point: (f64, f64), start: (f64, f64), end: (f64, f64)) -> f64 {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let length_squared = dx * dx + dy * dy;
    if length_squared == 0.0 {
        return (point.0 - start.0).powi(2) + (point.1 - start.1).powi(2);
    }
    let projection =
        (((point.0 - start.0) * dx + (point.1 - start.1) * dy) / length_squared).clamp(0.0, 1.0);
    let nearest_x = start.0 + projection * dx;
    let nearest_y = start.1 + projection * dy;
    (point.0 - nearest_x).powi(2) + (point.1 - nearest_y).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action::ActionAdapter;
    use crate::motion::ActuatorEffect;
    use crate::types::{Acceleration, Velocity, WorldPosition};
    use crate::world::GridCell;

    #[test]
    fn bounded_clearance_matches_full_map_scan() {
        for resolution in [0.1, 1.0, 2.5] {
            let origin = WorldPosition::new(-7.25, 3.5).unwrap();
            let cells: Vec<_> = (0..40)
                .flat_map(|row| (0..64).map(move |column| GridCell::new(column, row)))
                .filter(|cell| (cell.column() + 3 * cell.row()) % 7 == 0)
                .collect();
            let map = GridMap::new(64, 40, origin, resolution, cells.iter().copied()).unwrap();
            let kernel = SafetyKernel::new(SafetyConfig::new(0.2, 0.05).unwrap());
            // Stationary, short, long, reversed and diagonal sweeps; include exact cell edges.
            for sample in 0..400 {
                let point = |column: f64, row: f64| {
                    WorldPosition::new(
                        origin.x_meters() + column * resolution,
                        origin.y_meters() + row * resolution,
                    )
                    .unwrap()
                };
                let start = point(f64::from(sample % 64), f64::from(sample * 13 % 40));
                let end = match sample % 4 {
                    0 => start,
                    1 => point(
                        f64::from(sample % 64) + 0.2,
                        f64::from(sample * 13 % 40) + 0.1,
                    ),
                    _ => point(f64::from(sample * 17 % 64), f64::from(sample * 23 % 40)),
                };
                let margin = f64::from(sample % 5) * 0.1;
                let required = kernel.config.required_clearance() + margin;
                let (min_x, min_y, max_x, max_y) = map.world_bounds();
                let boundary = [start, end].iter().any(|position| {
                    position.x_meters() - min_x < required
                        || max_x - position.x_meters() < required
                        || position.y_meters() - min_y < required
                        || max_y - position.y_meters() < required
                });
                let expected = if boundary {
                    Err(SafetyReason::MapBoundary)
                } else if cells.iter().any(|cell| {
                    segment_aabb_distance_squared(start, end, map.cell_bounds(*cell))
                        < required * required
                }) {
                    Err(SafetyReason::StaticObstacleClearance)
                } else {
                    Ok(())
                };
                assert_eq!(kernel.segment_clearance(&map, start, end, margin), expected);
            }
        }
    }

    #[test]
    fn swept_circle_rejects_an_obstacle_before_commit() {
        let map = GridMap::new(
            4,
            3,
            WorldPosition::new(0.0, 0.0).unwrap(),
            1.0,
            [GridCell::new(2, 1)],
        )
        .unwrap();
        let state = RobotState::new(
            WorldPosition::new(1.4, 1.5).unwrap(),
            Velocity::new(2.0, 0.0).unwrap(),
            Acceleration::ZERO,
            0.0,
        )
        .unwrap();
        let kernel = SafetyKernel::new(SafetyConfig::new(0.2, 0.05).unwrap());
        let decision = kernel
            .decide(
                &map,
                state,
                ActionAdapter::from_index(2),
                MotionLimits::new(2.0, 10.0, 10.0, 20.0, 100.0, 200.0).unwrap(),
                ActuatorEffect::nominal(),
                false,
            )
            .unwrap();
        assert_eq!(decision.outcome, SafetyOutcome::SubstituteWait);
        assert_eq!(decision.reason, SafetyReason::StaticObstacleClearance);
        assert!(decision.next_state.velocity().x_mps() < state.velocity().x_mps());
    }
}
