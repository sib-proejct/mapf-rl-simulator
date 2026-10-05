use mapf_rl_simulator::fleet::{FleetConfig, MultiRobotEngine};
use mapf_rl_simulator::motion::MotionLimits;
use mapf_rl_simulator::plan::{PlanCoordinator, PlanRevision, PlanTarget};
use mapf_rl_simulator::protocol::{OrderRoute, OrderRouteWaypoint};
use mapf_rl_simulator::route::{action_toward_position, motion_target_for_route};
use mapf_rl_simulator::safety::SafetyConfig;
use mapf_rl_simulator::sensing::SensorConfig;
use mapf_rl_simulator::simulation::{EngineConfig, ManualMonotonicClock, SimulationEngine};
use mapf_rl_simulator::types::{Acceleration, RobotId, RobotState, Velocity, WorldPosition};
use mapf_rl_simulator::world::GridMap;
use std::collections::BTreeMap;
use uuid::Uuid;

fn engine(id: &str, column: u32) -> SimulationEngine<ManualMonotonicClock> {
    SimulationEngine::new(
        ManualMonotonicClock::default(),
        RobotId::new(id).unwrap(),
        GridMap::new(10, 5, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap(),
        RobotState::new(
            WorldPosition::new(column as f64 + 0.5, 2.5).unwrap(),
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
        7,
        &[],
    )
    .unwrap()
}

fn active_plan() -> PlanCoordinator {
    let target = PlanTarget {
        robot_id: "r1".into(),
        order_id: "o1".into(),
        order_update_id: 1,
        content_digest_sha256: "a".repeat(64),
    };
    let revision_id = Uuid::parse_str("00000000-0000-4000-8000-000000000001").unwrap();
    let mut plan = PlanCoordinator::new();
    plan.begin_prepare(
        PlanRevision {
            plan_revision_id: revision_id,
            planning_snapshot_digest_sha256: "b".repeat(64),
            targets: vec![target.clone()],
            activation_order: vec!["r1".into()],
        },
        0,
    )
    .unwrap();
    plan.mark_safe_hold(revision_id, "r1", true, 0).unwrap();
    plan.activate(revision_id, &target, 0).unwrap();
    plan
}

#[test]
fn continuous_route_stops_for_a_parked_robot_but_can_stop_safely_before_it() {
    let route = OrderRoute {
        execution_control: None,
        robot_id: "r1".into(),
        order_id: "o1".into(),
        release_after_robot_id: None,
        plan_digest_sha256: "f".repeat(64),
        motion: None,
        waypoints: (0..=4)
            .map(|column| OrderRouteWaypoint {
                column,
                row: 2,
                start_simulation_time_ms: u64::from(column) * 3000,
                end_simulation_time_ms: u64::from(column + 1) * 3000,
            })
            .collect(),
    };
    let r1 = RobotId::new("r1").unwrap();
    let r2 = RobotId::new("r2").unwrap();
    for parked_column in [3, 5] {
        let mut fleet = MultiRobotEngine::new(
            [engine("r1", 0), engine("r2", parked_column)],
            FleetConfig::new(0.5, 50).unwrap(),
        )
        .unwrap();
        let mut plans = BTreeMap::from([
            (r1.clone(), active_plan()),
            (r2.clone(), PlanCoordinator::new()),
        ]);
        let mut recovered = false;
        for _ in 0..180 {
            let engine = &fleet.robots()[&r1];
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
                    &BTreeMap::from([(r1.clone(), action)]),
                    &BTreeMap::from([(r1.clone(), target)]),
                    &mut plans,
                )
                .unwrap();
            let distance = step.records[&r2].state.position().x_meters()
                - step.records[&r1].state.position().x_meters();
            assert!(distance >= 0.5);
            if let Some(recovery) = step.recovery {
                assert!(recovery.held_robots.contains(&"r1".into()));
                recovered = true;
                break;
            }
        }
        if parked_column == 3 {
            assert!(recovered);
            assert!(!plans[&r1].motion_authorized("r1"));
        } else {
            assert!(!recovered);
            let state = fleet.robots()[&r1].state();
            assert!((state.position().x_meters() - 4.5).abs() < 1e-6);
            assert!(state.velocity().magnitude() < 1e-6);
        }
    }
}

#[test]
fn fast_profile_keeps_parked_robot_clear_and_survives_checkpoint_recovery() {
    use mapf_rl_simulator::motion::MotionTarget;
    let r1 = RobotId::new("r1").unwrap();
    let r2 = RobotId::new("r2").unwrap();
    let limits = MotionLimits::new(1.5, 1.0, 1.5, 6.0, 3.0, 60.0).unwrap();
    let moving = engine("r1", 0);
    let mut fleet = MultiRobotEngine::new(
        [moving, engine("r2", 8)],
        FleetConfig::new(0.5, 50).unwrap(),
    )
    .unwrap();
    let mut plans = BTreeMap::from([
        (r1.clone(), active_plan()),
        (r2.clone(), PlanCoordinator::new()),
    ]);
    fleet.apply_motion_profile(&r1, &mapf_rl_simulator::contracts::provisioning_generated::MotionProfileLimits::from_motion_limits(limits)).unwrap();
    let target = MotionTarget {
        position: WorldPosition::new(8.5, 2.5).unwrap(),
        cruise_speed_mps: Some(1.5),
    };
    let mut peak: f64 = 0.0;
    let mut recovered = false;
    for tick in 0..120 {
        let action =
            action_toward_position(fleet.robots()[&r1].state().position(), target.position);
        let step = fleet
            .step_with_independent_motion_targets(
                &BTreeMap::from([(r1.clone(), action)]),
                &BTreeMap::from([(r1.clone(), target)]),
                &mut plans,
            )
            .unwrap();
        peak = peak.max(step.records[&r1].state.velocity().magnitude());
        assert!(
            step.records[&r2].state.position().x_meters()
                - step.records[&r1].state.position().x_meters()
                >= 0.5
        );
        if tick == 25 {
            assert!(peak > 1.0);
            let checkpoint =
                fleet.checkpoint("local-fleet".into(), "a".repeat(64), PlanCoordinator::new());
            let mut restarted = MultiRobotEngine::new(
                [engine("r1", 0), engine("r2", 8)],
                FleetConfig::new(0.5, 50).unwrap(),
            )
            .unwrap();
            let temp = tempfile::TempDir::new().unwrap();
            let store =
                mapf_rl_simulator::checkpoint::CheckpointStore::new(temp.path().join("fleet.json"))
                    .unwrap();
            store.save(&checkpoint).unwrap();
            let recovered = store
                .load_for_restart("local-fleet", &"a".repeat(64))
                .unwrap()
                .unwrap();
            restarted.restore_checkpoint(&recovered).unwrap();
            assert_eq!(restarted.robots()[&r1].state(), fleet.robots()[&r1].state());
            assert_eq!(restarted.robots()[&r1].motion_limits(), limits);
        }
        if step.recovery.is_some() {
            recovered = true;
            break;
        }
    }
    assert!(peak > 1.49);
    assert!(recovered);
    assert!(!plans[&r1].motion_authorized("r1"));
}
