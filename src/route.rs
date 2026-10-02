//! Time-indexed route following for a prepared Core plan.

use crate::protocol::OrderRoute;
use crate::types::{SimulationTimeMs, WorldPosition};
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
            robot_id: "r1".into(),
            order_id: "o1".into(),
            release_after_robot_id: None,
            plan_digest_sha256: "f".repeat(64),
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
}
