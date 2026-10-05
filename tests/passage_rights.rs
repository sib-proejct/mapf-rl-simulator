use mapf_rl_simulator::{
    fleet::{FleetConfig, MultiRobotEngine},
    motion::MotionLimits,
    plan::{PlanCoordinator, PlanRevision, PlanTarget},
    protocol::{OrderRoute, OrderRouteWaypoint},
    route::{action_toward_position, motion_target_for_route},
    safety::SafetyConfig,
    sensing::SensorConfig,
    simulation::{EngineConfig, ManualMonotonicClock, SimulationEngine},
    types::{Acceleration, RobotId, RobotState, Velocity, WorldPosition},
    world::{GridCell, GridMap},
};
use std::collections::BTreeMap;
use uuid::Uuid;
fn active(id: &str) -> PlanCoordinator {
    let target = PlanTarget {
        robot_id: id.into(),
        order_id: id.into(),
        order_update_id: 1,
        content_digest_sha256: "a".repeat(64),
    };
    let revision = Uuid::from_u128(0x40008000000000000001);
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
fn engine(map: &GridMap, id: &str, cell: (u32, u32)) -> SimulationEngine<ManualMonotonicClock> {
    engine_at_position(
        map,
        id,
        map.grid_to_world(GridCell::new(cell.0, cell.1)).unwrap(),
    )
}

fn engine_at_position(
    map: &GridMap,
    id: &str,
    position: WorldPosition,
) -> SimulationEngine<ManualMonotonicClock> {
    SimulationEngine::new(
        ManualMonotonicClock::default(),
        RobotId::new(id).unwrap(),
        map.clone(),
        RobotState::new(position, Velocity::ZERO, Acceleration::ZERO, 0.0).unwrap(),
        EngineConfig {
            motion_limits: MotionLimits::new(1.5, 1.0, 1.5, 6.0, 3.0, 60.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        },
        7,
        &[],
    )
    .unwrap()
}
fn route(id: &str, cells: Vec<(u32, u32)>) -> OrderRoute {
    OrderRoute {
        execution_control: Some("occupancy-rights-v1".into()),
        robot_id: id.into(),
        order_id: id.into(),
        release_after_robot_id: None,
        plan_digest_sha256: "f".repeat(64),
        motion: None,
        // All estimates expire before the first physical tick.
        waypoints: cells
            .into_iter()
            .enumerate()
            .map(|(i, (column, row))| OrderRouteWaypoint {
                column,
                row,
                start_simulation_time_ms: i as u64,
                end_simulation_time_ms: i as u64 + 1,
            })
            .collect(),
    }
}
fn run(routes: BTreeMap<String, OrderRoute>, starts: &[(&str, (u32, u32))]) -> usize {
    let map = GridMap::new(9, 7, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let mut fleet = MultiRobotEngine::new(
        starts.iter().map(|(id, cell)| engine(&map, id, *cell)),
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let mut plans = starts
        .iter()
        .map(|(id, _)| (RobotId::new(*id).unwrap(), active(id)))
        .collect();
    let mut waits = 0;
    for _ in 0..500 {
        let mut targets = BTreeMap::new();
        let mut actions = BTreeMap::new();
        for (id, e) in fleet.robots() {
            let t = motion_target_for_route(
                e.map(),
                e.state(),
                e.simulation_time(),
                &routes[id.as_str()],
            )
            .unwrap();
            actions.insert(
                id.clone(),
                action_toward_position(e.state().position(), t.position),
            );
            targets.insert(id.clone(), t);
        }
        let step = fleet
            .step_with_passage_rights(&actions, &targets, &mut plans)
            .unwrap();
        waits += step.traffic_wait.len();
        assert!(step.recovery.is_none(), "{:?}", step.recovery);
        let positions: Vec<_> = step.records.values().map(|r| r.state.position()).collect();
        assert!(
            (positions[0].x_meters() - positions[1].x_meters())
                .hypot(positions[0].y_meters() - positions[1].y_meters())
                >= 0.5 - 1e-6
        );
    }
    for (id, e) in fleet.robots() {
        let g = routes[id.as_str()].waypoints.last().unwrap();
        assert_eq!(
            e.state().position(),
            map.grid_to_world(GridCell::new(g.column, g.row)).unwrap()
        );
        assert!(e.state().velocity().magnitude() < 1e-6);
    }
    waits
}
#[test]
fn crossing_waits_and_resumes_after_estimates_expire() {
    assert!(
        run(
            BTreeMap::from([
                ("a".into(), route("a", (0..=6).map(|x| (x, 3)).collect())),
                ("b".into(), route("b", (0..=6).map(|y| (3, y)).collect()))
            ]),
            &[("a", (0, 3)), ("b", (3, 0))]
        ) > 0
    );
}

#[test]
fn fractional_deadlock_pair_can_escape_without_stationary_padding_cycle() {
    let map = GridMap::new(32, 20, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let a = RobotId::new("r-008").unwrap();
    let b = RobotId::new("r-010").unwrap();
    let mut engines = Vec::new();
    for (id, position) in [
        (
            "r-008",
            WorldPosition::new(14.183466666666668, 2.5).unwrap(),
        ),
        (
            "r-010",
            WorldPosition::new(13.5, 3.2551316701949484).unwrap(),
        ),
    ] {
        engines.push(engine_at_position(&map, id, position));
    }
    let mut fleet = MultiRobotEngine::new(engines, FleetConfig::new(0.5, 50).unwrap()).unwrap();
    let routes = BTreeMap::from([
        (
            a.clone(),
            route("r-008", vec![(14, 2), (15, 2), (16, 2), (17, 2)]),
        ),
        (b.clone(), route("r-010", vec![(13, 3), (12, 3)])),
    ]);
    let mut plans = BTreeMap::from([(a.clone(), active("r-008")), (b.clone(), active("r-010"))]);
    for _ in 0..200 {
        let mut targets = BTreeMap::new();
        let mut actions = BTreeMap::new();
        for (id, robot) in fleet.robots() {
            let target = motion_target_for_route(
                robot.map(),
                robot.state(),
                robot.simulation_time(),
                &routes[id],
            )
            .unwrap();
            actions.insert(
                id.clone(),
                action_toward_position(robot.state().position(), target.position),
            );
            targets.insert(id.clone(), target);
        }
        let step = fleet
            .step_with_passage_rights(&actions, &targets, &mut plans)
            .unwrap();
        assert!(step.recovery.is_none(), "{:?}", step.recovery);
        let left = fleet.robots()[&a].state().position();
        let right = fleet.robots()[&b].state().position();
        assert!(
            (left.x_meters() - right.x_meters()).hypot(left.y_meters() - right.y_meters()) >= 0.5
        );
    }
    assert_eq!(
        fleet.robots()[&a].state().position(),
        WorldPosition::new(17.5, 2.5).unwrap()
    );
    assert_eq!(
        fleet.robots()[&b].state().position(),
        WorldPosition::new(12.5, 3.5).unwrap()
    );
}
#[test]
fn following_preserves_physical_occupancy() {
    assert!(
        run(
            BTreeMap::from([
                ("a".into(), route("a", (2..=8).map(|x| (x, 2)).collect())),
                ("b".into(), route("b", (0..=6).map(|x| (x, 2)).collect()))
            ]),
            &[("a", (2, 2)), ("b", (0, 2))]
        ) > 0
    );
}
#[test]
fn narrow_corridor_includes_both_exits() {
    let map = GridMap::new(
        9,
        5,
        WorldPosition::new(0.0, 0.0).unwrap(),
        1.0,
        (2..=6).flat_map(|x| (0..5).filter(|y| *y != 2).map(move |y| GridCell::new(x, y))),
    )
    .unwrap();
    assert_eq!(
        mapf_rl_simulator::traffic::corridor_resources(&map, (4, 2)),
        (1..=7).map(|x| (x, 2)).collect()
    );
}

#[test]
fn cancellation_or_connection_loss_keeps_rights_through_braking() {
    let map = GridMap::new(9, 7, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let a = RobotId::new("a").unwrap();
    let b = RobotId::new("b").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [engine(&map, "a", (0, 3)), engine(&map, "b", (3, 0))],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let routes = BTreeMap::from([
        ("a", route("a", (0..=6).map(|x| (x, 3)).collect())),
        ("b", route("b", (0..=6).map(|y| (3, y)).collect())),
    ]);
    let mut plans = BTreeMap::from([(a.clone(), active("a")), (b.clone(), active("b"))]);
    let mut observed_braking = false;
    for tick in 0..300 {
        if tick == 10 {
            plans.get_mut(&a).unwrap().hold_for_recovery(None).unwrap();
        }
        let braking = tick >= 10 && fleet.robots()[&a].state().velocity().magnitude() > 1e-6;
        let mut actions = BTreeMap::new();
        let mut targets = BTreeMap::new();
        for (id, e) in fleet.robots() {
            let target = motion_target_for_route(
                e.map(),
                e.state(),
                e.simulation_time(),
                &routes[id.as_str()],
            )
            .unwrap();
            actions.insert(
                id.clone(),
                action_toward_position(e.state().position(), target.position),
            );
            targets.insert(id.clone(), target);
        }
        let step = fleet
            .step_with_passage_rights(&actions, &targets, &mut plans)
            .unwrap();
        assert!(step.recovery.is_none());
        if braking {
            observed_braking = true;
            // A crossing beyond A's stopping envelope can proceed while A
            // brakes; actual occupied/braking cells must still remain protected.
            let left = step.records[&a].state.position();
            let right = step.records[&b].state.position();
            assert!(
                (left.x_meters() - right.x_meters()).hypot(left.y_meters() - right.y_meters())
                    >= 0.5 - 1e-6
            );
        }
    }
    assert!(observed_braking);
    assert!(fleet.robots()[&a].state().velocity().magnitude() < 1e-6);
    assert!(fleet.robots()[&a].state().position().x_meters() < 3.0);
    let actual = fleet.robots()[&b].state().position();
    let expected = map.grid_to_world(GridCell::new(3, 6)).unwrap();
    assert!(
        (actual.x_meters() - expected.x_meters()).hypot(actual.y_meters() - expected.y_meters())
            < 1e-6
    );
}

#[test]
fn opposite_corridor_requests_stop_before_entry_and_request_replan() {
    let map = GridMap::new(
        9,
        5,
        WorldPosition::new(0.0, 0.0).unwrap(),
        1.0,
        (2..=6).flat_map(|x| (0..5).filter(|y| *y != 2).map(move |y| GridCell::new(x, y))),
    )
    .unwrap();
    let a = RobotId::new("a").unwrap();
    let b = RobotId::new("b").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [engine(&map, "a", (1, 2)), engine(&map, "b", (7, 2))],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([(a.clone(), active("a")), (b.clone(), active("b"))]);
    let targets = BTreeMap::from([
        (
            a.clone(),
            map.grid_to_world(GridCell::new(7, 2)).unwrap().into(),
        ),
        (
            b.clone(),
            map.grid_to_world(GridCell::new(1, 2)).unwrap().into(),
        ),
    ]);
    let step = fleet
        .step_with_passage_rights(
            &BTreeMap::from([(a.clone(), 2), (b.clone(), 4)]),
            &targets,
            &mut plans,
        )
        .unwrap();
    let recovery = step.recovery.unwrap();
    assert!(recovery.replan_required);
    assert_eq!(recovery.held_robots, vec!["a", "b"]);
    assert_eq!(
        fleet.robots()[&a].state().position(),
        map.grid_to_world(GridCell::new(1, 2)).unwrap()
    );
    assert_eq!(
        fleet.robots()[&b].state().position(),
        map.grid_to_world(GridCell::new(7, 2)).unwrap()
    );
}

fn following_trace(emergency: bool, weaker_follower: bool) -> Vec<(RobotState, RobotState)> {
    let map = GridMap::new(30, 7, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let a = RobotId::new("a").unwrap();
    let b = RobotId::new("b").unwrap();
    let leader = engine(&map, "a", (4, 3));
    let follower = if weaker_follower {
        SimulationEngine::new(
            ManualMonotonicClock::default(),
            b.clone(),
            map.clone(),
            RobotState::new(
                map.grid_to_world(GridCell::new(0, 3)).unwrap(),
                Velocity::ZERO,
                Acceleration::ZERO,
                0.0,
            )
            .unwrap(),
            EngineConfig {
                motion_limits: MotionLimits::new(2.0, 1.0, 0.5, 1.0, 2.0, 4.0).unwrap(),
                safety: SafetyConfig::new(0.2, 0.05).unwrap(),
                sensor: SensorConfig::new(0.0, 0).unwrap(),
            },
            7,
            &[],
        )
        .unwrap()
    } else {
        engine(&map, "b", (0, 3))
    };
    let mut fleet =
        MultiRobotEngine::new([leader, follower], FleetConfig::new(0.5, 50).unwrap()).unwrap();
    let mut plans = BTreeMap::from([(a.clone(), active("a")), (b.clone(), active("b"))]);
    let routes = BTreeMap::from([
        (a.clone(), route("a", (4..=26).map(|x| (x, 3)).collect())),
        (b.clone(), route("b", (0..=22).map(|x| (x, 3)).collect())),
    ]);
    let mut trace = Vec::new();
    let mut simultaneous_motion = false;
    for tick in 0..400 {
        if emergency && tick == 70 {
            fleet.latch_robot_emergency_stop(&a).unwrap();
        }
        let mut actions = BTreeMap::new();
        let mut targets = BTreeMap::new();
        for (id, e) in fleet.robots() {
            let target =
                motion_target_for_route(e.map(), e.state(), e.simulation_time(), &routes[id])
                    .unwrap();
            actions.insert(
                id.clone(),
                action_toward_position(e.state().position(), target.position),
            );
            targets.insert(id.clone(), target);
        }
        let step = fleet
            .step_with_passage_rights(&actions, &targets, &mut plans)
            .unwrap_or_else(|e| {
                panic!(
                    "tick {tick}: {e:?}, states {:?}",
                    fleet
                        .robots()
                        .iter()
                        .map(|(id, e)| (id, e.state()))
                        .collect::<Vec<_>>()
                )
            });
        assert!(step.recovery.is_none(), "tick {tick}: {:?}", step.recovery);
        let left = step.records[&a].state;
        let right = step.records[&b].state;
        assert!(left.position().x_meters() - right.position().x_meters() >= 0.5 - 1e-6);
        simultaneous_motion |=
            left.velocity().magnitude() > 0.1 && right.velocity().magnitude() > 0.1;
        trace.push((left, right));
    }
    assert!(
        simultaneous_motion,
        "follower must depart before leader stops"
    );
    assert!(trace.last().unwrap().0.velocity().magnitude() < 1e-6);
    assert!(trace.last().unwrap().1.velocity().magnitude() < 1e-6);
    trace
}

#[test]
fn following_moves_before_leader_stops_and_replays_deterministically() {
    assert_eq!(following_trace(false, false), following_trace(false, false));
}

#[test]
fn faster_weaker_follower_stops_safely_when_leader_emergency_stops() {
    assert_eq!(following_trace(true, true), following_trace(true, true));
}

#[test]
fn changed_target_brakes_to_rest_before_reversing() {
    let map = GridMap::new(9, 7, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let id = RobotId::new("a").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [engine(&map, "a", (2, 3))],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([(id.clone(), active("a"))]);
    let mut observed_braking = false;
    let mut observed_reverse = false;
    let mut stopped_after_change = false;
    for tick in 0..250 {
        let previous = fleet.robots()[&id].state();
        if tick >= 10
            && previous.velocity().magnitude() < 1e-6
            && previous.acceleration().magnitude() < 1e-6
        {
            stopped_after_change = true;
        }
        let position = map
            .grid_to_world(GridCell::new(if tick < 10 { 8 } else { 0 }, 3))
            .unwrap();
        let step = fleet
            .step_with_passage_rights(
                &BTreeMap::from([(
                    id.clone(),
                    action_toward_position(previous.position(), position),
                )]),
                &BTreeMap::from([(id.clone(), position.into())]),
                &mut plans,
            )
            .unwrap();
        assert!(step.recovery.is_none());
        let record = &step.records[&id];
        if tick >= 10 && !stopped_after_change {
            observed_braking = true;
            assert_eq!(record.raw_action_index, 0);
            assert!(record.state.velocity().x_mps() >= -1e-6);
        }
        if record.state.velocity().x_mps() < -0.1 {
            assert!(stopped_after_change);
            observed_reverse = true;
        }
    }
    assert!(observed_braking && observed_reverse);
    assert!((fleet.robots()[&id].state().position().x_meters() - 0.5).abs() < 1e-6);
}

#[test]
fn forecast_wait_cycle_requests_replan() {
    let map = GridMap::new(9, 20, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let a = RobotId::new("r-001").unwrap();
    let b = RobotId::new("r-006").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [engine(&map, "r-001", (5, 9)), engine(&map, "r-006", (5, 8))],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([(a.clone(), active("r-001")), (b.clone(), active("r-006"))]);
    let targets: BTreeMap<RobotId, mapf_rl_simulator::motion::MotionTarget> = BTreeMap::from([
        (
            a.clone(),
            map.grid_to_world(GridCell::new(5, 8)).unwrap().into(),
        ),
        (
            b.clone(),
            map.grid_to_world(GridCell::new(5, 9)).unwrap().into(),
        ),
    ]);
    let mut detected = false;
    for _ in 0..100 {
        let actions = fleet
            .robots()
            .iter()
            .map(|(id, e)| {
                (
                    id.clone(),
                    action_toward_position(e.state().position(), targets[id].position),
                )
            })
            .collect();
        let step = fleet
            .step_with_passage_rights(&actions, &targets, &mut plans)
            .unwrap();
        if let Some(recovery) = step.recovery {
            assert_eq!(recovery.held_robots, vec!["r-001", "r-006"]);
            assert!(recovery.replan_required);
            assert!(
                plans.values().all(
                    |plan| !plan.motion_authorized("r-001") && !plan.motion_authorized("r-006")
                )
            );
            detected = true;
            break;
        }
    }
    assert!(detected, "forecast wait cycle must reach Core recovery");
}

#[test]
fn replanned_route_centers_fractional_start_before_turning() {
    let map = GridMap::new(9, 20, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let state = RobotState::new(
        WorldPosition::new(5.450989873386533, 8.5).unwrap(),
        Velocity::ZERO,
        Acceleration::ZERO,
        0.0,
    )
    .unwrap();
    let path = route("r-006", vec![(5, 8), (5, 9), (6, 9)]);
    let target = motion_target_for_route(
        &map,
        state,
        mapf_rl_simulator::types::SimulationTimeMs::new(0).unwrap(),
        &path,
    )
    .unwrap();
    assert_eq!(target.position, WorldPosition::new(5.5, 8.5).unwrap());
}

#[test]
fn fractional_replan_start_completes_without_repeated_deviation() {
    let map = GridMap::new(9, 20, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap();
    let id = RobotId::new("r-006").unwrap();
    let robot = SimulationEngine::new(
        ManualMonotonicClock::default(),
        id.clone(),
        map.clone(),
        RobotState::new(
            WorldPosition::new(5.450989873386533, 8.5).unwrap(),
            Velocity::ZERO,
            Acceleration::ZERO,
            0.0,
        )
        .unwrap(),
        EngineConfig {
            motion_limits: MotionLimits::new(1.5, 1.0, 1.5, 6.0, 3.0, 60.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        },
        7,
        &[],
    )
    .unwrap();
    let mut fleet = MultiRobotEngine::new([robot], FleetConfig::new(0.5, 50).unwrap()).unwrap();
    let mut plans = BTreeMap::from([(id.clone(), active("r-006"))]);
    let path = route("r-006", vec![(5, 8), (5, 9), (6, 9)]);
    for _ in 0..150 {
        let e = &fleet.robots()[&id];
        let target = motion_target_for_route(&map, e.state(), e.simulation_time(), &path).unwrap();
        let actions = BTreeMap::from([(
            id.clone(),
            action_toward_position(e.state().position(), target.position),
        )]);
        let step = fleet
            .step_with_passage_rights(
                &actions,
                &BTreeMap::from([(id.clone(), target)]),
                &mut plans,
            )
            .unwrap();
        assert!(step.recovery.is_none());
    }
    assert_eq!(
        fleet.robots()[&id].state().position(),
        WorldPosition::new(6.5, 9.5).unwrap()
    );
    assert!(fleet.robots()[&id].state().velocity().magnitude() < 1e-6);
}

#[test]
fn parked_blocker_triggers_once_after_threshold_and_resets_on_new_plan() {
    use mapf_rl_simulator::fleet::RecoveryReason;
    let map = GridMap::new(
        32,
        7,
        WorldPosition::new(0.0, 0.0).unwrap(),
        1.0,
        (0..32).flat_map(|x| [GridCell::new(x, 2), GridCell::new(x, 4)]),
    )
    .unwrap();
    let a = RobotId::new("r-010").unwrap();
    let b = RobotId::new("r-005").unwrap();
    let mut fleet = MultiRobotEngine::new(
        [
            engine(&map, "r-010", (21, 3)),
            engine(&map, "r-005", (19, 3)),
        ],
        FleetConfig::new(0.5, 3).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([
        (a.clone(), active("r-010")),
        (b.clone(), PlanCoordinator::new()),
    ]);
    let actions = BTreeMap::from([(a.clone(), 4)]);
    let targets = BTreeMap::from([(
        a.clone(),
        map.grid_to_world(GridCell::new(11, 3)).unwrap().into(),
    )]);
    for _ in 0..2 {
        let step = fleet
            .step_with_passage_rights(&actions, &targets, &mut plans)
            .unwrap();
        assert!(step.recovery.is_none());
        assert!(step.traffic_wait["r-010"].contains("r-005"));
    }
    let step = fleet
        .step_with_passage_rights(&actions, &targets, &mut plans)
        .unwrap();
    let recovery = step.recovery.unwrap();
    assert_eq!(recovery.reason, RecoveryReason::StationaryBlocked);
    assert_eq!(recovery.held_robots, vec!["r-010"]);
    assert!(!plans[&a].motion_authorized("r-010"));
    assert_eq!(
        fleet.robots()[&b].state().position(),
        map.grid_to_world(GridCell::new(19, 3)).unwrap()
    );
    for _ in 0..5 {
        assert!(
            fleet
                .step_with_passage_rights(&actions, &targets, &mut plans)
                .unwrap()
                .recovery
                .is_none()
        );
    }
    plans.insert(a.clone(), active("r-010"));
    assert!(
        fleet
            .step_with_passage_rights(&actions, &targets, &mut plans)
            .unwrap()
            .recovery
            .is_none()
    );
    // Clearing a request discards its accumulated wait.
    fleet
        .step_with_passage_rights(&BTreeMap::new(), &BTreeMap::new(), &mut plans)
        .unwrap();
    for _ in 0..2 {
        assert!(
            fleet
                .step_with_passage_rights(&actions, &targets, &mut plans)
                .unwrap()
                .recovery
                .is_none()
        );
    }
    // A stationary robot with movement authority is ordinary traffic, not parked.
    plans.insert(b, active("r-005"));
    for _ in 0..5 {
        assert!(
            fleet
                .step_with_passage_rights(&actions, &targets, &mut plans)
                .unwrap()
                .recovery
                .is_none()
        );
    }
}
