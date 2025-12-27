use sim_core::monte_carlo::{SweepConfig, run_sweep};
use sim_core::{Scenario, SensorNoise, Sim, Status};

fn print_usage() {
    eprintln!("Usage:");
    eprintln!("  sim_cli [scenario]              Run a single scenario");
    eprintln!("  sim_cli [scenario] --noise=<level>  Run with sensor noise (perfect|realistic|degraded)");
    eprintln!("  sim_cli sweep [scenario] [N]    Run N Monte Carlo trials (default: 500)");
    eprintln!("  sim_cli sweep [scenario] [N] --nav=<delta>  Sweep nav_const ± delta");
    eprintln!("  sim_cli sweep [scenario] [N] --amax=<delta> Sweep a_max ± delta");
    eprintln!("  sim_cli sweep [scenario] [N] --noise=<level>  Add sensor noise");
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

fn parse_noise(args: &[String]) -> SensorNoise {
    for arg in args {
        if let Some(val) = arg.strip_prefix("--noise=") {
            return match val {
                "realistic" => SensorNoise::realistic(),
                "degraded" => SensorNoise::degraded(),
                "extreme" => SensorNoise::extreme(),
                _ => SensorNoise::perfect(),
            };
        }
    }
    SensorNoise::perfect()
}

fn run_single(args: &[String]) {
    let scenario_name = args.first().map(|s| s.as_str()).unwrap_or("baseline");
    let scenario = match Scenario::by_name(scenario_name) {
        Some(s) => s,
        None => {
            eprintln!("Unknown scenario: '{}'", scenario_name);
            print_usage();
            std::process::exit(1);
        }
    };

    let noise = parse_noise(args);
    let noise_label = if noise.range_std > 0.0 { "with noise" } else { "perfect" };
    println!("=== Scenario: {} ({}) ===", scenario.name, noise_label);
    let mut sim = Sim::with_sensor_noise(scenario.missile, scenario.target, scenario.params, noise, 42);

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
        } else if let Some(val) = arg.strip_prefix("--noise=") {
            let noise = match val {
                "realistic" => SensorNoise::realistic(),
                "degraded" => SensorNoise::degraded(),
                "extreme" => SensorNoise::extreme(),
                _ => SensorNoise::perfect(),
            };
            config = config.with_sensor_noise(noise);
        }
    }

    println!("=== Monte Carlo: {} ({} trials) ===", scenario.name, trials);
    if config.perturb.nav_const > 0.0 {
        println!("  nav_const sweep: ±{:.1}", config.perturb.nav_const);
    }
    if config.perturb.a_max > 0.0 {
        println!("  a_max sweep: ±{:.1}", config.perturb.a_max);
    }
    if config.perturb.sensor_noise.range_std > 0.0 {
        println!("  sensor noise: range_std={:.1}m angle_rate_std={:.4}rad/s", 
            config.perturb.sensor_noise.range_std, config.perturb.sensor_noise.angle_rate_std);
    }
    println!();

    let result = run_sweep(&config);
    result.print_report();
    result.print_histogram(12);
}