//! Time-indexed route following for a prepared Core plan.

use crate::protocol::OrderRoute;
use crate::types::{SimulationTimeMs, WorldPosition};
use crate::world::{GridCell, GridMap, MapError};
use std::fmt;

pub fn action_for_route(
    map: &GridMap,
    position: WorldPosition,
    simulation_time: SimulationTimeMs,
    route: &OrderRoute,
) -> Result<i32, RouteError> {
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
    if current == target {
        return Ok(0);
    }
    let distance =
        current.column().abs_diff(target.column()) + current.row().abs_diff(target.row());
    if distance != 1 {
        return Err(RouteError::RouteDeviation);
    }
    Ok(if current.column() < target.column() {
        2
    } else if current.column() > target.column() {
        4
    } else if current.row() < target.row() {
        1
    } else {
        3
    })
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
