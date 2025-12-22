use sim_core::monte_carlo::{SweepConfig, run_sweep};
use sim_core::{Scenario, Sim, Status};

fn print_usage() {
    eprintln!("Usage:");
    eprintln!("  sim_cli [scenario]              Run a single scenario");
    eprintln!("  sim_cli sweep [scenario] [N]    Run N Monte Carlo trials (default: 500)");
    eprintln!("  sim_cli sweep [scenario] [N] --nav=<delta>  Sweep nav_const ± delta");
    eprintln!("  sim_cli sweep [scenario] [N] --amax=<delta> Sweep a_max ± delta");
    eprintln!();
    eprintln!("Available scenarios:");
    for s in Scenario::all() {
        eprintln!("  - {}", s.name);
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    if args.len() > 1 && args[1] == "sweep" {
        run_monte_carlo(&args[2..]);
    } else if args.len() > 1 && (args[1] == "-h" || args[1] == "--help") {
        print_usage();
    } else {
        run_single(&args[1..]);
    }
}

fn run_single(args: &[String]) {
    let scenario = if !args.is_empty() {
        let name = &args[0];
        match Scenario::by_name(name) {
            Some(s) => s,
            None => {
                eprintln!("Unknown scenario: '{}'", name);
                print_usage();
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

fn run_monte_carlo(args: &[String]) {
    let scenario_name = args.first().map(|s| s.as_str()).unwrap_or("baseline");
    let scenario = match Scenario::by_name(scenario_name) {
        Some(s) => s,
        None => {
            eprintln!("Unknown scenario: '{}'", scenario_name);
            print_usage();
            std::process::exit(1);
        }
    };

    let trials: usize = args
        .get(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(500);

    let mut config = SweepConfig::new(scenario.clone(), trials);

    for arg in args.iter().skip(2) {
        if let Some(val) = arg.strip_prefix("--nav=") {
            if let Ok(delta) = val.parse::<f32>() {
                config = config.with_nav_const_sweep(delta);
            }
        } else if let Some(val) = arg.strip_prefix("--amax=") {
            if let Ok(delta) = val.parse::<f32>() {
                config = config.with_a_max_sweep(delta);
            }
        } else if let Some(val) = arg.strip_prefix("--seed=") {
            if let Ok(seed) = val.parse::<u64>() {
                config = config.with_seed(seed);
            }
        }
    }

    println!("=== Monte Carlo: {} ({} trials) ===", scenario.name, trials);
    if config.perturb.nav_const > 0.0 {
        println!("  nav_const sweep: ±{:.1}", config.perturb.nav_const);
    }
    if config.perturb.a_max > 0.0 {
        println!("  a_max sweep: ±{:.1}", config.perturb.a_max);
    }
    println!();

    let result = run_sweep(&config);
    result.print_report();
    result.print_histogram(12);
}