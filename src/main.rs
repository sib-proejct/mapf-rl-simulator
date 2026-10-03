use mapf_rl_simulator::scenario::demo_scenario;
use mapf_rl_simulator::simulation::ManualMonotonicClock;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if std::env::var("MAPF_SIMULATOR_MODE").as_deref() == Ok("fleet") {
        return tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()?
            .block_on(mapf_rl_simulator::fleet_operational::run());
    }
    if std::env::var("MAPF_SIMULATOR_MODE").as_deref() == Ok("core") {
        return tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()?
            .block_on(mapf_rl_simulator::operational::run());
    }
    let scenario = demo_scenario()?;
    let result = scenario.run(ManualMonotonicClock::default())?;
    println!(
        "scenario: {}@{}",
        result.scenario_id.as_str(),
        result.scenario_version
    );
    println!("seed: {}", result.master_seed);
    println!(
        "ticks: {} × {} ms",
        result.records.len(),
        result.control_tick_ms
    );
    println!(
        "final pose: ({:.6}, {:.6}) m",
        result.final_state.position().x_meters(),
        result.final_state.position().y_meters()
    );
    println!("state digest: {}", result.state_digest_sha256);
    Ok(())
}
