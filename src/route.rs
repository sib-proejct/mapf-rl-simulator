//! Time-indexed route following for a prepared Core plan.

use crate::motion::MotionTarget;
use crate::protocol::OrderRoute;
use crate::types::{RobotState, SimulationTimeMs, WorldPosition};
use crate::world::{GridCell, GridMap, MapError};
use std::fmt;

pub const NODE_CENTER_TOLERANCE_METERS: f64 = 1.0e-6;

pub fn action_toward_position(position: WorldPosition, target: WorldPosition) -> i32 {
    let dx = target.x_meters() - position.x_meters();
    let dy = target.y_meters() - position.y_meters();
    if dx.abs() > NODE_CENTER_TOLERANCE_METERS {
        if dx > 0.0 { 2 } else { 4 }
    } else if dy.abs() > NODE_CENTER_TOLERANCE_METERS {
        if dy > 0.0 { 1 } else { 3 }
    } else {
        0
    }
}

pub fn action_for_route(
    map: &GridMap,
    position: WorldPosition,
    simulation_time: SimulationTimeMs,
    route: &OrderRoute,
) -> Result<i32, RouteError> {
    Ok(action_toward_position(
        position,
        target_for_route(map, position, simulation_time, route)?,
    ))
}

pub fn target_for_route(
    map: &GridMap,
    position: WorldPosition,
    simulation_time: SimulationTimeMs,
    route: &OrderRoute,
) -> Result<WorldPosition, RouteError> {
    let current = map.world_to_grid(position)?;
    let time = u64::try_from(simulation_time.get()).map_err(|_| RouteError::InvalidTime)?;
    let desired = route
        .waypoints
        .iter()
        .find(|waypoint| {
            waypoint.start_simulation_time_ms <= time && time < waypoint.end_simulation_time_ms
        })
        .or_else(|| {
            route
                .waypoints
                .first()
                .filter(|first| time < first.start_simulation_time_ms)
        })
        .or_else(|| route.waypoints.last())
        .ok_or(RouteError::EmptyRoute)?;
    let target = GridCell::new(desired.column, desired.row);
    let distance =
        current.column().abs_diff(target.column()) + current.row().abs_diff(target.row());
    if distance > 1 {
        return Err(RouteError::RouteDeviation);
    }
    let center = map.grid_to_world(target)?;
    // Finish centering on the incoming axis before starting a perpendicular edge.
    let cross_track_error = if current.column() != target.column() {
        (position.y_meters() - center.y_meters()).abs()
    } else if current.row() != target.row() {
        (position.x_meters() - center.x_meters()).abs()
    } else {
        0.0
    };
    if cross_track_error > NODE_CENTER_TOLERANCE_METERS {
        Ok(map.grid_to_world(current)?)
    } else {
        Ok(center)
    }
}

/// Keep the legacy time-indexed target as the authority, extending a straight
/// run only when a bounded cruise speed cannot reach unreleased nodes early.
/// The endpoint remains a full stop for corners, waits and the final goal.
pub fn motion_target_for_route(
    map: &GridMap,
    state: RobotState,
    simulation_time: SimulationTimeMs,
    route: &OrderRoute,
) -> Result<MotionTarget, RouteError> {
    if route.execution_control.as_deref() == Some("occupancy-rights-v1") {
        return occupancy_target(map, state, route);
    }
    if let Some(motion) = &route.motion {
        return target_for_motion_segments(map, state, simulation_time, motion);
    }
    let position = state.position();
    let stop = target_for_route(map, position, simulation_time, route)?;
    let time = u64::try_from(simulation_time.get()).map_err(|_| RouteError::InvalidTime)?;
    let Some(index) = route.waypoints.iter().position(|waypoint| {
        waypoint.start_simulation_time_ms <= time && time < waypoint.end_simulation_time_ms
    }) else {
        return Ok(stop.into());
    };
    if index == 0 {
        return Ok(stop.into());
    }
    let points = &route.waypoints;
    let cell = |index: usize| GridCell::new(points[index].column, points[index].row);
    if stop != map.grid_to_world(cell(index))? {
        return Ok(stop.into()); // Finish the incoming axis at a corner.
    }
    let edge = |index: usize| {
        (
            i64::from(points[index].column) - i64::from(points[index - 1].column),
            i64::from(points[index].row) - i64::from(points[index - 1].row),
        )
    };
    let direction = edge(index);
    if direction.0.abs() + direction.1.abs() != 1 {
        return Ok(stop.into()); // Explicit wait or invalid edge: never look through it.
    }
    let mut first = index;
    while first > 1 && edge(first - 1) == direction {
        first -= 1;
    }
    let mut last = index;
    while last + 1 < points.len() && edge(last + 1) == direction {
        last += 1;
    }
    if first == last {
        return Ok(stop.into());
    }
    let interval_ms = points[first]
        .end_simulation_time_ms
        .saturating_sub(points[first].start_simulation_time_ms);
    if interval_ms == 0 {
        return Err(RouteError::InvalidTime);
    }
    if points[first..=last].iter().any(|point| {
        point
            .end_simulation_time_ms
            .saturating_sub(point.start_simulation_time_ms)
            != interval_ms
    }) || points[first - 1..=last]
        .windows(2)
        .any(|pair| pair[0].end_simulation_time_ms != pair[1].start_simulation_time_ms)
    {
        return Ok(stop.into()); // Nonuniform schedules retain the legacy controller.
    }
    let speed = map.resolution_meters() * 1000.0 / interval_ms as f64;
    let remaining_seconds = (points[index].end_simulation_time_ms - time) as f64 / 1000.0;
    let distance = (stop.x_meters() - position.x_meters()) * direction.0 as f64
        + (stop.y_meters() - position.y_meters()) * direction.1 as f64;
    if distance + NODE_CENTER_TOLERANCE_METERS < speed * remaining_seconds
        || state.velocity().magnitude() > speed + NODE_CENTER_TOLERANCE_METERS
    {
        return Ok(stop.into()); // An early/resumed robot must stop at the active node.
    }
    Ok(MotionTarget {
        position: map.grid_to_world(cell(last))?,
        cruise_speed_mps: Some(speed),
    })
}

fn target_for_motion_segments(
    map: &GridMap,
    state: RobotState,
    simulation_time: SimulationTimeMs,
    motion: &crate::contracts::provisioning_generated::MotionRoutePlan,
) -> Result<MotionTarget, RouteError> {
    let time = u64::try_from(simulation_time.get()).map_err(|_| RouteError::InvalidTime)?;
    let first = motion.segments.first().ok_or(RouteError::EmptyRoute)?;
    let center = |column: u64, row: u64| -> Result<WorldPosition, RouteError> {
        let column = u32::try_from(column).map_err(|_| RouteError::RouteDeviation)?;
        let row = u32::try_from(row).map_err(|_| RouteError::RouteDeviation)?;
        Ok(map.grid_to_world(GridCell::new(column, row))?)
    };
    if time < first.start_simulation_time_ms {
        return Ok(center(first.start_column, first.start_row)?.into());
    }
    let position = state.position();
    let last = motion.segments.last().ok_or(RouteError::EmptyRoute)?;
    if time >= last.end_simulation_time_ms {
        let goal = center(last.end_column, last.end_row)?;
        if (position.x_meters() - goal.x_meters()).abs() > NODE_CENTER_TOLERANCE_METERS
            || (position.y_meters() - goal.y_meters()).abs() > NODE_CENTER_TOLERANCE_METERS
            || state.velocity().magnitude() >= 1e-6
            || state.acceleration().magnitude() >= 1e-6
        {
            return Err(RouteError::RouteDeviation); // Never outlive the corridor reservation.
        }
    }
    for (index, segment) in motion.segments.iter().enumerate() {
        if time < segment.start_simulation_time_ms {
            break;
        }
        let start = center(segment.start_column, segment.start_row)?;
        let end = center(segment.end_column, segment.end_row)?;
        let dx = end.x_meters() - start.x_meters();
        let dy = end.y_meters() - start.y_meters();
        let along = (position.x_meters() - start.x_meters()) * dx.signum()
            + (position.y_meters() - start.y_meters()) * dy.signum();
        let length = dx.abs() + dy.abs();
        let cross = if dx != 0.0 {
            (position.y_meters() - start.y_meters()).abs()
        } else {
            (position.x_meters() - start.x_meters()).abs()
        };
        if cross > NODE_CENTER_TOLERANCE_METERS
            || along < -NODE_CENTER_TOLERANCE_METERS
            || along > length + NODE_CENTER_TOLERANCE_METERS
        {
            continue;
        }
        if (along - length).abs() <= NODE_CENTER_TOLERANCE_METERS
            && state.velocity().magnitude() < 1e-6
            && state.acceleration().magnitude() < 1e-6
            && motion
                .segments
                .get(index + 1)
                .is_some_and(|next| time >= next.start_simulation_time_ms)
        {
            continue;
        }
        return Ok(MotionTarget {
            position: end,
            cruise_speed_mps: Some(motion.limits.max_linear_speed_mps),
        });
    }
    Err(RouteError::RouteDeviation)
}

/// Route progress follows measured position. Timing fields are estimates only.
fn occupancy_target(
    map: &GridMap,
    state: RobotState,
    route: &OrderRoute,
) -> Result<MotionTarget, RouteError> {
    let points = &route.waypoints;
    let goal = points.last().ok_or(RouteError::EmptyRoute)?;
    let goal = map.grid_to_world(GridCell::new(goal.column, goal.row))?;
    let position = state.position();
    if position == goal {
        return Ok(goal.into());
    }
    let mut index = 1;
    while index < points.len() {
        let start = map.grid_to_world(GridCell::new(
            points[index - 1].column,
            points[index - 1].row,
        ))?;
        let direction = (
            i64::from(points[index].column) - i64::from(points[index - 1].column),
            i64::from(points[index].row) - i64::from(points[index - 1].row),
        );
        if direction.0.abs() + direction.1.abs() != 1 {
            return Err(RouteError::RouteDeviation);
        }
        let mut last = index;
        while last + 1 < points.len()
            && (
                i64::from(points[last + 1].column) - i64::from(points[last].column),
                i64::from(points[last + 1].row) - i64::from(points[last].row),
            ) == direction
        {
            last += 1;
        }
        let end = map.grid_to_world(GridCell::new(points[last].column, points[last].row))?;
        let along = (position.x_meters() - start.x_meters()) * direction.0 as f64
            + (position.y_meters() - start.y_meters()) * direction.1 as f64;
        let length =
            (end.x_meters() - start.x_meters()).abs() + (end.y_meters() - start.y_meters()).abs();
        let cross = if direction.0 != 0 {
            (position.y_meters() - start.y_meters()).abs()
        } else {
            (position.x_meters() - start.x_meters()).abs()
        };
        if cross <= NODE_CENTER_TOLERANCE_METERS
            && along >= -NODE_CENTER_TOLERANCE_METERS
            && along <= length + NODE_CENTER_TOLERANCE_METERS
        {
            if along >= length - NODE_CENTER_TOLERANCE_METERS
                && state.velocity().magnitude() < 1e-6
                && state.acceleration().magnitude() < 1e-6
                && last + 1 < points.len()
            {
                index = last + 1;
                continue;
            }
            return Ok(MotionTarget {
                position: end,
                cruise_speed_mps: route.motion.as_ref().map(|m| m.limits.max_linear_speed_mps),
            });
        }
        index = last + 1;
    }
    // Replanning starts at the observed cell, which need not be centered after
    // traffic braking. Align inside that cell before entering the first edge.
    let first = points.first().ok_or(RouteError::EmptyRoute)?;
    let cell = GridCell::new(first.column, first.row);
    if map.world_to_grid(position)? == cell {
        let center = map.grid_to_world(cell)?;
        let target =
            if (position.x_meters() - center.x_meters()).abs() > NODE_CENTER_TOLERANCE_METERS {
                WorldPosition::new(center.x_meters(), position.y_meters())
                    .map_err(|_| RouteError::RouteDeviation)?
            } else {
                center
            };
        return Ok(target.into());
    }
    Err(RouteError::RouteDeviation)
}

#[derive(Debug)]
pub enum RouteError {
    EmptyRoute,
    InvalidTime,
    RouteDeviation,
    World(MapError),
}

impl From<MapError> for RouteError {
    fn from(value: MapError) -> Self {
        Self::World(value)
    }
}

impl fmt::Display for RouteError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyRoute => formatter.write_str("prepared route is empty"),
            Self::InvalidTime => formatter.write_str("simulation time cannot index the route"),
            Self::RouteDeviation => {
                formatter.write_str("robot deviated from its time-indexed route")
            }
            Self::World(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for RouteError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::OrderRouteWaypoint;

    fn route() -> OrderRoute {
        OrderRoute {
            execution_control: None,
            robot_id: "r1".into(),
            order_id: "o1".into(),
            release_after_robot_id: None,
            plan_digest_sha256: "f".repeat(64),
            motion: None,
            waypoints: vec![
                OrderRouteWaypoint {
                    column: 1,
                    row: 0,
                    start_simulation_time_ms: 0,
                    end_simulation_time_ms: 1000,
                },
                OrderRouteWaypoint {
                    column: 1,
                    row: 1,
                    start_simulation_time_ms: 1000,
                    end_simulation_time_ms: 2000,
                },
            ],
        }
    }

    #[test]
    fn entering_the_destination_cell_does_not_count_as_center_arrival() {
        let map = GridMap::new(3, 3, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
        assert_eq!(
            action_for_route(
                &map,
                WorldPosition::new(1.05, 0.5).unwrap(),
                SimulationTimeMs::ZERO,
                &route()
            )
            .unwrap(),
            2
        );
        assert_eq!(
            action_for_route(
                &map,
                WorldPosition::new(1.5, 0.5).unwrap(),
                SimulationTimeMs::ZERO,
                &route()
            )
            .unwrap(),
            0
        );
    }

    #[test]
    fn corner_target_finishes_incoming_axis_before_turning() {
        let map = GridMap::new(3, 3, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
        let time = SimulationTimeMs::new(1000).unwrap();
        let position = WorldPosition::new(1.2, 0.5).unwrap();
        assert_eq!(
            target_for_route(&map, position, time, &route()).unwrap(),
            WorldPosition::new(1.5, 0.5).unwrap()
        );
        assert_eq!(action_for_route(&map, position, time, &route()).unwrap(), 2);
        assert_eq!(
            action_for_route(&map, WorldPosition::new(1.5, 0.5).unwrap(), time, &route()).unwrap(),
            1
        );
    }
    fn timed_route(cells: &[(u32, u32)]) -> OrderRoute {
        let mut route = route();
        route.waypoints = cells
            .iter()
            .enumerate()
            .map(|(index, &(column, row))| OrderRouteWaypoint {
                column,
                row,
                start_simulation_time_ms: index as u64 * 3000,
                end_simulation_time_ms: (index as u64 + 1) * 3000,
            })
            .collect();
        route
    }

    fn engine() -> crate::simulation::SimulationEngine<crate::simulation::ManualMonotonicClock> {
        engine_at(0, 0)
    }

    fn engine_at(
        column: u32,
        row: u32,
    ) -> crate::simulation::SimulationEngine<crate::simulation::ManualMonotonicClock> {
        use crate::motion::MotionLimits;
        use crate::safety::SafetyConfig;
        use crate::sensing::SensorConfig;
        use crate::simulation::{EngineConfig, ManualMonotonicClock, SimulationEngine};
        use crate::types::{Acceleration, RobotId, Velocity};
        SimulationEngine::new(
            ManualMonotonicClock::default(),
            RobotId::new("r1").unwrap(),
            GridMap::new(20, 20, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap(),
            RobotState::new(
                WorldPosition::new(column as f64 + 0.5, row as f64 + 0.5).unwrap(),
                Velocity::ZERO,
                Acceleration::ZERO,
                0.0,
            )
            .unwrap(),
            EngineConfig {
                motion_limits: MotionLimits::new(2.0, 2.0, 3.0, 6.0, 10.0, 30.0).unwrap(),
                safety: SafetyConfig::new(0.1, 0.02).unwrap(),
                sensor: SensorConfig::new(0.0, 0).unwrap(),
            },
            1,
            &[],
        )
        .unwrap()
    }

    #[test]
    fn straight_run_passes_intermediate_centers_without_stopping_or_running_early() {
        let route = timed_route(&[(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]);
        let mut engine = engine();
        let mut crossed = [false; 3];
        for _ in 0..180 {
            let before = engine.state();
            let target =
                motion_target_for_route(engine.map(), before, engine.simulation_time(), &route)
                    .unwrap();
            let action = action_toward_position(before.position(), target.position);
            let record = engine.step_with_target(action, Some(target)).unwrap();
            for (index, crossed) in crossed.iter_mut().enumerate() {
                let center = index as f64 + 1.5;
                if before.position().x_meters() < center
                    && record.state.position().x_meters() >= center
                {
                    assert!(record.state.velocity().magnitude() > 0.3);
                    assert!(engine.simulation_time().get() >= (index as i64 + 2) * 3000);
                    *crossed = true;
                }
            }
        }
        assert!(crossed.iter().all(|crossed| *crossed));
        assert!((engine.state().position().x_meters() - 4.5).abs() < 1e-6);
        assert!(engine.state().velocity().magnitude() < 1e-6);
    }

    #[test]
    fn waits_and_corners_remain_stop_endpoints() {
        for cells in [
            vec![(0, 0), (1, 0), (2, 0), (2, 0), (3, 0), (4, 0)],
            vec![(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)],
        ] {
            let route = timed_route(&cells);
            let mut engine = engine();
            let mut stopped_at_boundary = false;
            for _ in 0..230 {
                let before = engine.state();
                let target =
                    motion_target_for_route(engine.map(), before, engine.simulation_time(), &route)
                        .unwrap();
                let action = action_toward_position(before.position(), target.position);
                let record = engine.step_with_target(action, Some(target)).unwrap();
                if (record.state.position().x_meters() - 2.5).abs() < 1e-6
                    && (record.state.position().y_meters() - 0.5).abs() < 1e-6
                    && record.state.velocity().magnitude() < 1e-6
                {
                    stopped_at_boundary = true;
                }
            }
            assert!(stopped_at_boundary);
            let &(column, row) = cells.last().unwrap();
            assert!((engine.state().position().x_meters() - (column as f64 + 0.5)).abs() < 1e-6);
            assert!((engine.state().position().y_meters() - (row as f64 + 0.5)).abs() < 1e-6);
            assert!(engine.state().velocity().magnitude() < 1e-6);
        }
    }

    #[test]
    fn early_or_fast_recovery_does_not_look_through_unreleased_nodes() {
        use crate::types::{Acceleration, Velocity};
        let engine = engine();
        let route = timed_route(&[(0, 0), (1, 0), (2, 0), (3, 0)]);
        for (x, speed) in [(1.4, 0.0), (0.5, 1.0)] {
            let state = RobotState::new(
                WorldPosition::new(x, 0.5).unwrap(),
                Velocity::new(speed, 0.0).unwrap(),
                Acceleration::ZERO,
                0.0,
            )
            .unwrap();
            let target = motion_target_for_route(
                engine.map(),
                state,
                SimulationTimeMs::new(3000).unwrap(),
                &route,
            )
            .unwrap();
            assert_eq!(target, WorldPosition::new(1.5, 0.5).unwrap().into());
        }
    }
    #[test]
    fn all_cardinal_runs_are_continuous_and_deterministic() {
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let cells: Vec<_> = (0..5)
                .map(|i| ((5 + dx * i) as u32, (5 + dy * i) as u32))
                .collect();
            let route = timed_route(&cells);
            let mut engines = [engine_at(5, 5), engine_at(5, 5)];
            let mut crossings = 0;
            for _ in 0..180 {
                let before = engines[0].state();
                for engine in &mut engines {
                    let target = motion_target_for_route(
                        engine.map(),
                        engine.state(),
                        engine.simulation_time(),
                        &route,
                    )
                    .unwrap();
                    let action = action_toward_position(engine.state().position(), target.position);
                    engine.step_with_target(action, Some(target)).unwrap();
                }
                assert_eq!(engines[0].state(), engines[1].state());
                let progress = |state: RobotState| {
                    (state.position().x_meters() - 5.5) * dx as f64
                        + (state.position().y_meters() - 5.5) * dy as f64
                };
                for center in 1..4 {
                    if progress(before) < center as f64
                        && progress(engines[0].state()) >= center as f64
                    {
                        assert!(engines[0].state().velocity().magnitude() > 0.3);
                        crossings += 1;
                    }
                }
            }
            assert_eq!(crossings, 3);
            let goal = engines[0]
                .map()
                .grid_to_world(GridCell::new(cells[4].0, cells[4].1))
                .unwrap();
            assert!((engines[0].state().position().x_meters() - goal.x_meters()).abs() < 1e-6);
            assert!((engines[0].state().position().y_meters() - goal.y_meters()).abs() < 1e-6);
            assert!(engines[0].state().velocity().magnitude() < 1e-6);
        }
    }

    #[test]
    fn nonuniform_schedule_retains_active_node_stop() {
        let engine = engine();
        let mut route = timed_route(&[(0, 0), (1, 0), (2, 0), (3, 0)]);
        route.waypoints[2].end_simulation_time_ms += 1000;
        route.waypoints[3].start_simulation_time_ms += 1000;
        route.waypoints[3].end_simulation_time_ms += 1000;
        let target = motion_target_for_route(
            engine.map(),
            engine.state(),
            SimulationTimeMs::new(3000).unwrap(),
            &route,
        )
        .unwrap();
        assert_eq!(target, WorldPosition::new(1.5, 0.5).unwrap().into());
    }

    #[test]
    fn revoked_motion_brakes_a_continuously_moving_robot() {
        let mut engine = engine();
        let route = timed_route(&[(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]);
        for _ in 0..80 {
            let target = motion_target_for_route(
                engine.map(),
                engine.state(),
                engine.simulation_time(),
                &route,
            )
            .unwrap();
            let action = action_toward_position(engine.state().position(), target.position);
            engine.step_with_target(action, Some(target)).unwrap();
        }
        assert!(engine.state().velocity().magnitude() > 0.3);
        for _ in 0..30 {
            engine.step(0).unwrap();
        }
        assert!(engine.state().velocity().magnitude() < 1e-6);
        assert!(engine.state().position().x_meters() < 2.5);
    }
    #[test]
    fn core_motion_fixture_reaches_requested_cruise_speed_and_stops_at_the_corner() {
        let command: crate::protocol::OrderCommand = serde_json::from_str(include_str!(
            "../../mapf-rl-core/packages/contracts/fixtures/valid/ws-motion-order-command.json"
        ))
        .unwrap();
        command.validate().unwrap();
        let route = command.payload.route.unwrap();
        let motion = route.motion.as_ref().unwrap();
        let mut engine = engine_at(0, 2);
        engine
            .apply_motion_limits(motion.limits.motion_limits().unwrap())
            .unwrap();
        let mut peak: f64 = 0.0;
        let mut corner_stop = false;
        for _ in 0..180 {
            let target = motion_target_for_route(
                engine.map(),
                engine.state(),
                engine.simulation_time(),
                &route,
            )
            .unwrap();
            let action = action_toward_position(engine.state().position(), target.position);
            let state = engine.step_with_target(action, Some(target)).unwrap().state;
            peak = peak.max(state.velocity().magnitude());
            assert!(state.velocity().magnitude() <= 1.5 + 1e-8);
            assert!(state.acceleration().magnitude() <= 1.0 + 1e-8);
            if (state.position().x_meters() - 10.5).abs() < 1e-6
                && (state.position().y_meters() - 2.5).abs() < 1e-6
                && state.velocity().magnitude() < 1e-6
            {
                corner_stop = true;
            }
        }
        assert!(
            peak > 1.49,
            "long straight routes must reach the configured speed"
        );
        assert!(corner_stop);
        assert!((engine.state().position().x_meters() - 10.5).abs() < 1e-6);
        assert!((engine.state().position().y_meters() - 5.5).abs() < 1e-6);
        assert!(engine.state().velocity().magnitude() < 1e-6);
    }

    #[test]
    fn motion_route_deadline_does_not_authorize_late_motion() {
        let command: crate::protocol::OrderCommand = serde_json::from_str(include_str!(
            "../../mapf-rl-core/packages/contracts/fixtures/valid/ws-motion-order-command.json"
        ))
        .unwrap();
        let route = command.payload.route.unwrap();
        let deadline = route
            .motion
            .as_ref()
            .unwrap()
            .segments
            .last()
            .unwrap()
            .end_simulation_time_ms;
        let engine = engine_at(0, 2);
        assert!(matches!(
            motion_target_for_route(
                engine.map(),
                engine.state(),
                SimulationTimeMs::new(deadline as i64).unwrap(),
                &route
            ),
            Err(RouteError::RouteDeviation)
        ));
    }
}
