use mapf_rl_simulator::fault::{FaultKind, FaultSpec};
use mapf_rl_simulator::motion::MotionLimits;
use mapf_rl_simulator::safety::{SafetyConfig, SafetyOutcome, SafetyReason};
use mapf_rl_simulator::scenario::{Scenario, demo_scenario};
use mapf_rl_simulator::sensing::SensorConfig;
use mapf_rl_simulator::simulation::{EngineConfig, ManualMonotonicClock};
use mapf_rl_simulator::types::{
    Acceleration, RobotId, RobotState, ScenarioId, Velocity, WorldPosition,
};
use mapf_rl_simulator::world::{GridCell, GridMap};

const DEMO_GOLDEN_DIGEST: &str = "67bf8a9ed0dd2af9212c6b659eaae9677d3709eeaadcd9286874304e321aa6f6";

#[test]
fn same_scenario_and_seed_match_the_golden_digest() {
    let scenario = demo_scenario().unwrap();
    let first = scenario.run(ManualMonotonicClock::new(0)).unwrap();
    let clock = ManualMonotonicClock::new(9_999_999);
    clock.advance(123_456).unwrap();
    let second = scenario.run(clock).unwrap();

    assert_eq!(first.state_digest_sha256, second.state_digest_sha256);
    assert_eq!(first.state_digest_sha256, DEMO_GOLDEN_DIGEST);
    assert_eq!(first.records, second.records);
    assert_eq!(first.final_time.get(), 1_200);
}

#[test]
fn invalid_action_is_substituted_with_wait() {
    let mut scenario = open_scenario(vec![2, 77]);
    scenario.engine_config.motion_limits =
        MotionLimits::new(1.0, 10.0, 10.0, 20.0, 100.0, 200.0).unwrap();
    let result = scenario.run(ManualMonotonicClock::default()).unwrap();
    let invalid = &result.records[1];

    assert!(invalid.invalid_action);
    assert_eq!(invalid.applied_action as u8, 0);
    assert_eq!(invalid.safety_outcome, SafetyOutcome::SubstituteWait);
    assert_eq!(invalid.safety_reason, SafetyReason::InvalidAction);
    assert!(invalid.state.velocity().magnitude() <= 1.0);
    assert!(
        invalid.state.acceleration().magnitude()
            < result.records[0].state.acceleration().magnitude()
    );
}

#[test]
fn predicted_collision_is_blocked_before_motion_integration() {
    let map = GridMap::new(
        4,
        3,
        WorldPosition::new(0.0, 0.0).unwrap(),
        1.0,
        [GridCell::new(2, 1)],
    )
    .unwrap();
    let initial_state = RobotState::new(
        WorldPosition::new(1.4, 1.5).unwrap(),
        Velocity::new(2.0, 0.0).unwrap(),
        Acceleration::ZERO,
        0.0,
    )
    .unwrap();
    let scenario = Scenario {
        id: ScenarioId::new("collision-preintegration").unwrap(),
        version: "1.0.0".to_owned(),
        master_seed: 9,
        robot_id: RobotId::new("r1").unwrap(),
        map,
        initial_state,
        engine_config: EngineConfig {
            motion_limits: MotionLimits::new(2.0, 10.0, 10.0, 20.0, 100.0, 200.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        },
        action_indices: vec![2],
        fault_specs: vec![],
    };
    let result = scenario.run(ManualMonotonicClock::default()).unwrap();
    let record = &result.records[0];

    assert_eq!(record.safety_outcome, SafetyOutcome::SubstituteWait);
    assert_eq!(record.safety_reason, SafetyReason::StaticObstacleClearance);
    assert!(record.state.position().x_meters() < 1.75);
    assert!(record.state.velocity().x_mps() < initial_state.velocity().x_mps());
}

#[test]
fn stopping_envelope_rejects_a_candidate_while_the_next_pose_is_still_clear() {
    let map = GridMap::new(
        4,
        3,
        WorldPosition::new(0.0, 0.0).unwrap(),
        1.0,
        [GridCell::new(2, 1)],
    )
    .unwrap();
    let initial_state = RobotState::new(
        WorldPosition::new(1.5, 1.5).unwrap(),
        Velocity::new(1.0, 0.0).unwrap(),
        Acceleration::ZERO,
        0.0,
    )
    .unwrap();
    let scenario = Scenario {
        id: ScenarioId::new("stopping-envelope").unwrap(),
        version: "1.0.0".to_owned(),
        master_seed: 10,
        robot_id: RobotId::new("r1").unwrap(),
        map,
        initial_state,
        engine_config: EngineConfig {
            motion_limits: MotionLimits::new(2.0, 10.0, 10.0, 20.0, 100.0, 200.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        },
        action_indices: vec![2],
        fault_specs: vec![],
    };

    let record = &scenario
        .run(ManualMonotonicClock::default())
        .unwrap()
        .records[0];
    assert_eq!(record.safety_outcome, SafetyOutcome::SubstituteWait);
    assert_eq!(record.safety_reason, SafetyReason::StaticObstacleClearance);
    assert!(record.state.position().x_meters() > initial_state.position().x_meters());
    assert!(record.state.position().x_meters() < 1.75);
    assert!(record.state.velocity().x_mps() < initial_state.velocity().x_mps());
}

#[test]
fn controlled_and_emergency_stops_integrate_distinct_stopping_distances() {
    let mut controlled = open_scenario(vec![2; 8]);
    controlled.initial_state = RobotState::new(
        WorldPosition::new(4.5, 4.5).unwrap(),
        Velocity::new(1.0, 0.0).unwrap(),
        Acceleration::ZERO,
        0.0,
    )
    .unwrap();
    controlled.fault_specs = vec![FaultSpec::once(0, "stuck", FaultKind::ActuatorStuck)];
    let controlled_result = controlled.run(ManualMonotonicClock::default()).unwrap();
    let controlled_record = &controlled_result.records[0];
    assert_eq!(
        controlled_record.safety_outcome,
        SafetyOutcome::ControlledStop
    );
    assert_eq!(controlled_result.final_state.velocity(), Velocity::ZERO);
    assert_eq!(
        controlled_result.final_state.acceleration(),
        Acceleration::ZERO
    );

    let mut emergency = open_scenario(vec![2; 8]);
    emergency.initial_state = controlled.initial_state;
    emergency.fault_specs = vec![FaultSpec::once(0, "emergency", FaultKind::EmergencyStop)];
    let result = emergency.run(ManualMonotonicClock::default()).unwrap();
    assert_eq!(
        result.records[0].safety_outcome,
        SafetyOutcome::EmergencyStop
    );
    assert_eq!(
        result.records[1].safety_outcome,
        SafetyOutcome::EmergencyStop
    );
    assert_eq!(result.final_state.velocity(), Velocity::ZERO);
    assert_eq!(result.final_state.acceleration(), Acceleration::ZERO);
    assert!(
        controlled_record.state.velocity().x_mps() > result.records[0].state.velocity().x_mps()
    );
    assert!(
        controlled_result.final_state.position().x_meters()
            > result.final_state.position().x_meters()
    );
}

#[test]
fn sensor_and_fault_randomness_are_seeded_and_stream_isolated() {
    let mut scenario = open_scenario(vec![0; 32]);
    scenario.engine_config.sensor = SensorConfig::new(0.1, 0).unwrap();
    scenario.fault_specs = vec![FaultSpec {
        start_tick: 0,
        end_tick_inclusive: 31,
        probability_parts_per_million: 250_000,
        stable_source_id: "dropout".to_owned(),
        kind: FaultKind::SensorDropout,
    }];
    let first = scenario.run(ManualMonotonicClock::default()).unwrap();
    let second = scenario.run(ManualMonotonicClock::new(u64::MAX)).unwrap();
    assert_eq!(first.state_digest_sha256, second.state_digest_sha256);

    scenario.master_seed += 1;
    let changed_seed = scenario.run(ManualMonotonicClock::default()).unwrap();
    assert_ne!(first.state_digest_sha256, changed_seed.state_digest_sha256);
}

#[test]
fn fixed_tick_does_not_observe_thread_scheduling_or_wall_time() {
    let scenario = open_scenario(vec![1, 2, 3, 4, 0]);
    let clock_a = ManualMonotonicClock::new(1);
    let clock_b = ManualMonotonicClock::new(u64::MAX);
    let a = scenario.run(clock_a).unwrap();
    let b = std::thread::spawn(move || scenario.run(clock_b).unwrap())
        .join()
        .unwrap();

    assert_eq!(a.records, b.records);
    assert_eq!(a.state_digest_sha256, b.state_digest_sha256);
    assert_eq!(a.final_time.get(), 500);
}

fn open_scenario(action_indices: Vec<i32>) -> Scenario {
    Scenario {
        id: ScenarioId::new("open-grid").unwrap(),
        version: "1.0.0".to_owned(),
        master_seed: 42,
        robot_id: RobotId::new("robot-open").unwrap(),
        map: GridMap::new(9, 9, WorldPosition::new(0.0, 0.0).unwrap(), 1.0, []).unwrap(),
        initial_state: RobotState::new(
            WorldPosition::new(4.5, 4.5).unwrap(),
            Velocity::ZERO,
            Acceleration::ZERO,
            0.0,
        )
        .unwrap(),
        engine_config: EngineConfig {
            motion_limits: MotionLimits::new(1.0, 2.0, 3.0, 6.0, 30.0, 60.0).unwrap(),
            safety: SafetyConfig::new(0.2, 0.05).unwrap(),
            sensor: SensorConfig::new(0.0, 0).unwrap(),
        },
        action_indices,
        fault_specs: vec![],
    }
}
