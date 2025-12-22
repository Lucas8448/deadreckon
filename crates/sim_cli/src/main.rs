use sim_core::{Scenario, Sim, Status};

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let scenario = if args.len() > 1 {
        let name = &args[1];
        match Scenario::by_name(name) {
            Some(s) => s,
            None => {
                eprintln!("Unknown scenario: '{}'", name);
                eprintln!("Available scenarios:");
                for s in Scenario::all() {
                    eprintln!("  - {}", s.name);
                }
                std::process::exit(1);
            }
        }
    } else {
        sim_core::scenario::baseline()
    };

    println!("=== Scenario: {} ===", scenario.name);
    let mut sim = Sim::new(scenario.missile, scenario.target, scenario.params);

    let mut status = Status::Running;
    while status == Status::Running {
        status = sim.step();
    }

    println!("status: {:?}", status);
    println!("t: {:.2}s  range: {:.2}m  closing: {:.2}m/s  los_rate: {:.6}rad/s  a_cmd: {:.2}m/s^2",
        sim.last.t, sim.last.range, sim.last.closing_speed, sim.last.los_rate, sim.last.a_cmd
    );
    println!("missile.p: {:?} missile.v: {:?}", sim.missile.p, sim.missile.v);
    println!("target.p:  {:?} target.v:  {:?}", sim.target.p, sim.target.v);
}