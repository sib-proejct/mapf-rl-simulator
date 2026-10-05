use mapf_rl_simulator::fleet::{FleetConfig, MultiRobotEngine};
use mapf_rl_simulator::motion::MotionLimits;
use mapf_rl_simulator::plan::{PlanCoordinator, PlanRevision, PlanTarget};
use mapf_rl_simulator::protocol::{OrderRoute, OrderRouteWaypoint};
use mapf_rl_simulator::route::{action_toward_position, motion_target_for_route};
use mapf_rl_simulator::safety::SafetyConfig;
use mapf_rl_simulator::sensing::SensorConfig;
use mapf_rl_simulator::simulation::{EngineConfig, ManualMonotonicClock, SimulationEngine};
use mapf_rl_simulator::types::{Acceleration, RobotId, RobotState, Velocity, WorldPosition};
use mapf_rl_simulator::world::{GridCell, GridMap};
use std::collections::BTreeMap;
use uuid::Uuid;

fn warehouse() -> GridMap {
    let value: serde_json::Value = serde_json::from_str(include_str!(
        "../../mapf-rl-core/scripts/warehouse-map.json"
    ))
    .unwrap();
    let width = value["widthCells"].as_u64().unwrap() as u32;
    let blocked = value["cells"]
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
        .filter(|(_, v)| v.as_u64() == Some(1))
        .map(|(i, _)| GridCell::new(i as u32 % width, i as u32 / width));
    GridMap::new(
        width,
        value["heightCells"].as_u64().unwrap() as u32,
        WorldPosition::new(0.0, 0.0).unwrap(),
        value["resolutionMeters"].as_f64().unwrap(),
        blocked,
    )
    .unwrap()
}

fn berths() -> Vec<(u32, u32)> {
    let catalog: serde_json::Value = serde_json::from_str(include_str!(
        "../../mapf-rl-core/packages/contracts/buffers/warehouse-buffers.json"
    ))
    .unwrap();
    catalog["buffers"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| {
            (
                v["column"].as_u64().unwrap() as u32,
                v["row"].as_u64().unwrap() as u32,
            )
        })
        .collect()
}

fn active_plan(id: &str) -> PlanCoordinator {
    let target = PlanTarget {
        robot_id: id.into(),
        order_id: "buffer-move".into(),
        order_update_id: 1,
        content_digest_sha256: "a".repeat(64),
    };
    let revision = Uuid::parse_str("00000000-0000-4000-8000-000000000001").unwrap();
    let mut plan = PlanCoordinator::new();
    plan.begin_prepare(
        PlanRevision {
            plan_revision_id: revision,
            planning_snapshot_digest_sha256: "b".repeat(64),
            targets: vec![target.clone()],
            activation_order: vec![id.into()],
        },
        0,
    )
    .unwrap();
    plan.mark_safe_hold(revision, id, true, 0).unwrap();
    plan.activate(revision, &target, 0).unwrap();
    plan
}

fn run_berth_access(index: usize, entering: bool) -> RobotState {
    let cells = berths();
    let (column, row) = cells[index];
    let corridor_row = if row == 3 { 2 } else { 17 };
    let path = if entering {
        vec![(column, corridor_row), (column, row)]
    } else {
        vec![
            (column, row),
            (column, corridor_row),
            (column - 1, corridor_row),
        ]
    };
    let mover = RobotId::new(format!("buffer-{index}")).unwrap();
    let engines = cells.iter().enumerate().map(|(i, &(x, y))| {
        let start = if i == index { path[0] } else { (x, y) };
        let map = warehouse();
        SimulationEngine::new(
            ManualMonotonicClock::default(),
            RobotId::new(format!("buffer-{i}")).unwrap(),
            map.clone(),
            RobotState::new(
                map.grid_to_world(GridCell::new(start.0, start.1)).unwrap(),
                Velocity::ZERO,
                Acceleration::ZERO,
                0.0,
            )
            .unwrap(),
            EngineConfig {
                motion_limits: MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0).unwrap(),
                safety: SafetyConfig::new(0.2, 0.05).unwrap(),
                sensor: SensorConfig::new(0.0, 0).unwrap(),
            },
            41,
            &[],
        )
        .unwrap()
    });
    let mut fleet = MultiRobotEngine::new(engines, FleetConfig::new(0.5, 50).unwrap()).unwrap();
    let mut plans: BTreeMap<_, _> = fleet
        .robots()
        .keys()
        .map(|id| {
            (
                id.clone(),
                if id == &mover {
                    active_plan(id.as_str())
                } else {
                    PlanCoordinator::new()
                },
            )
        })
        .collect();
    let route = OrderRoute {
        execution_control: None,
        robot_id: mover.as_str().into(),
        order_id: "buffer-move".into(),
        release_after_robot_id: None,
        plan_digest_sha256: "f".repeat(64),
        motion: None,
        waypoints: path
            .iter()
            .enumerate()
            .map(|(i, &(column, row))| OrderRouteWaypoint {
                column,
                row,
                start_simulation_time_ms: i as u64 * 3000,
                end_simulation_time_ms: (i as u64 + 1) * 3000,
            })
            .collect(),
    };
    for _ in 0..240 {
        let engine = &fleet.robots()[&mover];
        let target = motion_target_for_route(
            engine.map(),
            engine.state(),
            engine.simulation_time(),
            &route,
        )
        .unwrap();
        let action = action_toward_position(engine.state().position(), target.position);
        let step = fleet
            .step_with_independent_motion_targets(
                &BTreeMap::from([(mover.clone(), action)]),
                &BTreeMap::from([(mover.clone(), target)]),
                &mut plans,
            )
            .unwrap();
        assert!(
            step.recovery.is_none(),
            "berth {index}, entering {entering}"
        );
        let moving = step.records[&mover].state.position();
        for (id, record) in &step.records {
            if id != &mover {
                let other = record.state.position();
                assert!(
                    (moving.x_meters() - other.x_meters())
                        .hypot(moving.y_meters() - other.y_meters())
                        >= 0.5
                );
            }
        }
        let state = fleet.robots()[&mover].state();
        let goal = path.last().unwrap();
        if (state.position().x_meters() - (goal.0 as f64 + 0.5)).abs() < 1e-6
            && (state.position().y_meters() - (goal.1 as f64 + 0.5)).abs() < 1e-6
            && state.velocity().magnitude() < 1e-6
        {
            break;
        }
    }
    let state = fleet.robots()[&mover].state();
    let goal = path.last().unwrap();
    assert!((state.position().x_meters() - (goal.0 as f64 + 0.5)).abs() < 1e-6);
    assert!((state.position().y_meters() - (goal.1 as f64 + 0.5)).abs() < 1e-6);
    assert!(state.velocity().magnitude() < 1e-6);
    state
}

#[test]
fn every_warehouse_buffer_remains_accessible_with_all_other_buffers_occupied() {
    assert_eq!(berths().len(), 12);
    for index in 0..12 {
        for entering in [false, true] {
            let first = run_berth_access(index, entering);
            let repeated = run_berth_access(index, entering);
            assert_eq!(first, repeated);
        }
    }
}
