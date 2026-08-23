//! Production composition of Core transport, durable session state, and the Phase 1 engine.

use crate::controller::{BaselineController, RuntimeProfile};
use crate::core_client::{ApiKey, CoreClient, CoreClientConfig};
use crate::motion::MotionLimits;
use crate::protocol::{MapIdentity, OrderGoal, PoseReport, RasterMapContent};
use crate::runtime::{Phase2Runtime, RuntimeEvent};
use crate::safety::SafetyConfig;
use crate::sensing::SensorConfig;
use crate::session::CoreSession;
use crate::simulation::{EngineConfig, ManualMonotonicClock, SimulationEngine};
use crate::spool::DurableSpool;
use crate::types::{Acceleration, RobotId, RobotState, Velocity, WorldPosition};
use crate::world::{GridCell, GridMap};
use std::env;
use std::error::Error;
use std::io;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use url::Url;
use uuid::Uuid;

struct OperationalConfig {
    profile: RuntimeProfile,
    simulator_id: String,
    robot_id: String,
    rest_base_url: Url,
    websocket_url: Url,
    api_key: ApiKey,
    spool_path: PathBuf,
    map: MapIdentity,
    start: GridCell,
    master_seed: u64,
    exit_after_completion: bool,
}

impl OperationalConfig {
    fn from_environment() -> Result<Self, Box<dyn Error>> {
        let profile = RuntimeProfile::parse(&required("MAPF_PROFILE")?)?;
        let simulator_id = required("MAPF_SIMULATOR_ID")?;
        let robot_id = required("MAPF_SIMULATOR_ROBOT_ID")?;
        let rest_base_url = Url::parse(&required("MAPF_SIMULATOR_CORE_REST_URL")?)?;
        let websocket_url = Url::parse(&required("MAPF_SIMULATOR_CORE_WS_URL")?)?;
        let api_key = ApiKey::new(&required("MAPF_SIMULATOR_API_KEY")?)?;
        let spool_path = PathBuf::from(required("MAPF_SIMULATOR_SPOOL_PATH")?);
        let map = MapIdentity {
            map_id: Uuid::parse_str(&required("MAPF_SIMULATOR_MAP_ID")?)?,
            revision: parse("MAPF_SIMULATOR_MAP_REVISION")?,
            content_digest_sha256: required("MAPF_SIMULATOR_MAP_DIGEST_SHA256")?,
        };
        map.validate()?;
        Ok(Self {
            profile,
            simulator_id,
            robot_id,
            rest_base_url,
            websocket_url,
            api_key,
            spool_path,
            map,
            start: GridCell::new(
                parse_or("MAPF_SIMULATOR_START_COLUMN", 0)?,
                parse_or("MAPF_SIMULATOR_START_ROW", 0)?,
            ),
            master_seed: parse_or("MAPF_SIMULATOR_MASTER_SEED", 0x5eed_2026)?,
            exit_after_completion: parse_bool_or("MAPF_SIMULATOR_EXIT_AFTER_COMPLETION", false)?,
        })
    }
}

pub async fn run() -> Result<(), Box<dyn Error>> {
    let config = OperationalConfig::from_environment()?;
    let controller = BaselineController::explicit(config.profile)?;
    let spool = DurableSpool::open(&config.spool_path, &config.simulator_id)?;
    let session = CoreSession::new(
        config.simulator_id.clone(),
        config.robot_id.clone(),
        config.map.clone(),
        controller.report().clone(),
        spool,
    )?;
    let client = CoreClient::new(CoreClientConfig::new(
        config.profile,
        config.simulator_id.clone(),
        config.rest_base_url,
        config.websocket_url,
        config.api_key,
    )?)?;
    let mut runtime = Phase2Runtime::new(client, session);
    let started = Instant::now();
    let snapshot = runtime.connect_and_replay(0).await?;
    let robot = snapshot
        .robots
        .iter()
        .find(|robot| robot.robot_id == config.robot_id)
        .ok_or_else(|| invalid("Core snapshot omits the configured robot"))?;
    let map_content = robot
        .map_content
        .as_ref()
        .ok_or_else(|| invalid("Core snapshot omits authoritative map content"))?;
    let map = build_map(map_content)?;
    let initial_position = map.grid_to_world(config.start)?;
    if map.is_blocked(config.start)? {
        return Err(invalid("configured start cell is blocked").into());
    }
    let mut engine = SimulationEngine::new(
        ManualMonotonicClock::default(),
        RobotId::new(config.robot_id)?,
        map,
        RobotState::new(initial_position, Velocity::ZERO, Acceleration::ZERO, 0.0)?,
        EngineConfig {
            motion_limits: MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0)?,
            safety: SafetyConfig::new(0.2, 0.05)?,
            sensor: SensorConfig::new(0.0, 0)?,
        },
        config.master_seed,
        &[],
    )?;
    let mut completion_queued = false;
    let mut interval = tokio::time::interval(Duration::from_millis(100));

    loop {
        let monotonic_ms = elapsed_milliseconds(started);
        tokio::select! {
            _ = interval.tick() => {
                let applied = runtime.applied_order().await?;
                let action = applied
                    .as_ref()
                    .and_then(|order| order.goal)
                    .map_or(0, |goal| action_toward_goal(&engine, goal));
                let record = engine.step(action)?;
                let occurred_at = utc_now_milliseconds()?;
                runtime.publish_state_if_due(
                    monotonic_ms,
                    engine.tick().get(),
                    engine.simulation_time().get(),
                    PoseReport {
                        x_meters: record.state.position().x_meters(),
                        y_meters: record.state.position().y_meters(),
                        yaw_radians: record.state.yaw_radians(),
                    },
                    occurred_at,
                ).await?;
                if !completion_queued
                    && applied.as_ref().and_then(|order| order.goal).is_some_and(|goal| {
                        goal_reached_and_stopped(&engine, goal)
                    })
                {
                    runtime.publish_order_completed(
                        engine.simulation_time().get(),
                        utc_now_milliseconds()?,
                        monotonic_ms,
                    ).await?;
                    completion_queued = true;
                }
            }
            event = runtime.receive_one(monotonic_ms, utc_now_milliseconds()?) => {
                match event? {
                    RuntimeEvent::Order { decision, command } => {
                        if decision.apply_to_robot {
                            let goal = command.payload.goal.ok_or_else(|| {
                                invalid("Core Order command omits its goal")
                            })?;
                            validate_goal(engine.map(), goal)?;
                            completion_queued = false;
                        }
                    }
                    RuntimeEvent::ReportAcknowledged(_) => {
                        if completion_queued && runtime.applied_order().await?.is_none() {
                            if config.exit_after_completion {
                                return Ok(());
                            }
                            completion_queued = false;
                        }
                    }
                }
            }
        }
    }
}

fn build_map(content: &RasterMapContent) -> Result<GridMap, Box<dyn Error>> {
    content.validate()?;
    let blocked = content
        .cells
        .iter()
        .copied()
        .enumerate()
        .filter(|(_, cell)| *cell == 1)
        .map(|(index, _)| {
            GridCell::new(
                (index % content.width_cells as usize) as u32,
                (index / content.width_cells as usize) as u32,
            )
        });
    Ok(GridMap::new(
        content.width_cells,
        content.height_cells,
        WorldPosition::new(content.origin.x_meters, content.origin.y_meters)?,
        content.resolution_meters,
        blocked,
    )?)
}

fn validate_goal(map: &GridMap, goal: OrderGoal) -> Result<(), Box<dyn Error>> {
    map.grid_to_world(GridCell::new(goal.column, goal.row))?;
    if map.is_blocked(GridCell::new(goal.column, goal.row))? {
        return Err(invalid("Core Order goal is blocked").into());
    }
    Ok(())
}

fn action_toward_goal(engine: &SimulationEngine<ManualMonotonicClock>, goal: OrderGoal) -> i32 {
    let Ok(cell) = engine.map().world_to_grid(engine.state().position()) else {
        return 0;
    };
    if cell.column() < goal.column {
        2
    } else if cell.column() > goal.column {
        4
    } else if cell.row() < goal.row {
        1
    } else if cell.row() > goal.row {
        3
    } else {
        0
    }
}

fn goal_reached_and_stopped(
    engine: &SimulationEngine<ManualMonotonicClock>,
    goal: OrderGoal,
) -> bool {
    engine
        .map()
        .world_to_grid(engine.state().position())
        .is_ok_and(|cell| cell == GridCell::new(goal.column, goal.row))
        && engine.state().velocity().x_mps().abs() < 1.0e-6
        && engine.state().velocity().y_mps().abs() < 1.0e-6
}

fn required(name: &'static str) -> Result<String, io::Error> {
    env::var(name).map_err(|_| invalid(&format!("{name} is required")))
}

fn parse<T>(name: &'static str) -> Result<T, io::Error>
where
    T: std::str::FromStr,
{
    required(name)?
        .parse()
        .map_err(|_| invalid(&format!("{name} is invalid")))
}

fn parse_or<T>(name: &'static str, fallback: T) -> Result<T, io::Error>
where
    T: std::str::FromStr,
{
    match env::var(name) {
        Ok(value) => value
            .parse()
            .map_err(|_| invalid(&format!("{name} is invalid"))),
        Err(env::VarError::NotPresent) => Ok(fallback),
        Err(_) => Err(invalid(&format!("{name} is invalid"))),
    }
}

fn parse_bool_or(name: &'static str, fallback: bool) -> Result<bool, io::Error> {
    match env::var(name) {
        Ok(value) if value == "true" => Ok(true),
        Ok(value) if value == "false" => Ok(false),
        Ok(_) => Err(invalid(&format!("{name} must be true or false"))),
        Err(env::VarError::NotPresent) => Ok(fallback),
        Err(_) => Err(invalid(&format!("{name} is invalid"))),
    }
}

fn elapsed_milliseconds(started: Instant) -> u64 {
    u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX)
}

fn utc_now_milliseconds() -> Result<String, io::Error> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| invalid("system clock predates the Unix epoch"))?;
    let seconds = i64::try_from(duration.as_secs())
        .map_err(|_| invalid("system clock exceeds supported UTC range"))?;
    let days = seconds / 86_400;
    let seconds_of_day = seconds % 86_400;
    let (year, month, day) = civil_from_days(days);
    let hour = seconds_of_day / 3_600;
    let minute = seconds_of_day % 3_600 / 60;
    let second = seconds_of_day % 60;
    Ok(format!(
        "{year:04}-{month:02}-{day:02}T{hour:02}:{minute:02}:{second:02}.{:03}Z",
        duration.subsec_millis()
    ))
}

fn civil_from_days(days_since_epoch: i64) -> (i64, i64, i64) {
    let shifted = days_since_epoch + 719_468;
    let era = shifted / 146_097;
    let day_of_era = shifted - era * 146_097;
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let mut year = year_of_era + era * 400;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_prime = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * month_prime + 2) / 5 + 1;
    let month = month_prime + if month_prime < 10 { 3 } else { -9 };
    year += i64::from(month <= 2);
    (year, month, day)
}

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unix_epoch_conversion_uses_utc_calendar() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(20_688), (2026, 8, 23));
    }
}
