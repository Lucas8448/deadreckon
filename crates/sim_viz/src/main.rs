//! Terminal visualization of a 3D engagement, using `physics_sandbox::viz`'s
//! three-view (SIDE / TOP / FRONT) instrument cluster — far more legible for
//! missile geometry than a single perspective view.

use std::{thread, time::Duration};

use crossterm::style::Color;
use physics_sandbox::viz::{Marker, MultiView, prepare_terminal, restore_terminal};
use sim_core::{Scenario, SensorNoise, Sim, Status};

fn parse_noise(args: &[String]) -> (SensorNoise, &'static str) {
    for arg in args {
        if let Some(val) = arg.strip_prefix("--noise=") {
            return match val {
                "realistic" => (SensorNoise::realistic(), "realistic"),
                "degraded" => (SensorNoise::degraded(), "degraded"),
                "extreme" => (SensorNoise::extreme(), "extreme"),
                _ => (SensorNoise::perfect(), "perfect"),
            };
        }
    }
    (SensorNoise::perfect(), "perfect")
}

fn main() -> std::io::Result<()> {
    let args: Vec<String> = std::env::args().collect();

    let scenario_name_arg = args.iter().skip(1).find(|a| !a.starts_with("--"));
    let scenario = if let Some(name) = scenario_name_arg {
        match Scenario::by_name(name) {
            Some(s) => s,
            None => {
                eprintln!("Unknown scenario: '{}'\nAvailable:", name);
                for s in Scenario::all() {
                    eprintln!("  - {}", s.name);
                }
                std::process::exit(1);
            }
        }
    } else {
        sim_core::scenario::baseline()
    };

    let (noise, noise_label) = parse_noise(&args);
    let scenario_name = scenario.name;

    let mut sim = Sim::with_sensor_noise(
        scenario.missile,
        scenario.target,
        scenario.params,
        noise,
        12345,
    );

    // World extents: pad initial target distance generously so the trajectories
    // stay on-screen even when missile + target overshoot.
    let target_p = scenario.target.p;
    let span = (target_p.x.abs() + 2_000.0).max(8_000.0);
    let alt_lo = -200.0_f64;
    let alt_hi = (target_p.y.abs() + 2_000.0).max(3_000.0);
    let cross = (target_p.z.abs() + 2_000.0).max(2_000.0);

    let mut view = MultiView::three_view(
        50,
        18,
        (-500.0, span),
        (alt_lo, alt_hi),
        (-cross, cross),
    );

    prepare_terminal()?;

    let mut step_count = 0u64;
    let frame_every = 3;
    let final_status;

    loop {
        let status = sim.step();
        step_count += 1;

        if step_count.is_multiple_of(frame_every) || status != Status::Running {
            let m = sim.missile_pos();
            let t = sim.target_pos();
            let markers = [
                Marker::new3(m, '\u{25B2}').with_color(Color::Rgb {
                    r: 60,
                    g: 255,
                    b: 90,
                }),
                Marker::new3(t, '\u{25CF}').with_color(Color::Rgb {
                    r: 255,
                    g: 90,
                    b: 60,
                }),
            ];
            view.draw(&markers)?;
            // HUD line printed below the panels.
            println!(
                " [{}] noise={}  t={:6.2}s  range={:8.1}m  Vc={:6.1}m/s  a_cmd={:6.1}  status={:?}        ",
                scenario_name,
                noise_label,
                sim.last.t,
                sim.last.range,
                sim.last.closing_speed,
                sim.last.a_cmd,
                status,
            );
        }

        if status != Status::Running {
            final_status = status;
            break;
        }

        // ~30 fps when frame_every matches dt.
        thread::sleep(Duration::from_millis(20));
    }

    restore_terminal()?;

    let mp = sim.missile_pos();
    let tp = sim.target_pos();
    println!();
    println!("=== Final ({}): {:?} ===", scenario_name, final_status);
    println!("  t           : {:.2} s", sim.last.t);
    println!("  miss range  : {:.2} m", sim.last.range);
    println!(
        "  missile end : ({:.1}, {:.1}, {:.1})",
        mp.x, mp.y, mp.z
    );
    println!(
        "  target  end : ({:.1}, {:.1}, {:.1})",
        tp.x, tp.y, tp.z
    );

    Ok(())
}
