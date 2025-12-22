use glam::Vec2;
use macroquad::prelude::*;
use sim_core::{Missile, Params, Sim, Status, Target};

fn world_to_screen(p: Vec2, origin: Vec2, scale: f32) -> Vec2 {
    let v = (p - origin) * scale;
    Vec2::new(v.x, -v.y)
}

#[macroquad::main("Deadreckon")]
async fn main() {
    let missile = Missile { p: Vec2::new(0.0, 0.0), v: Vec2::new(250.0, 0.0), a_max: 60.0 };
    let target  = Target  { p: Vec2::new(6000.0, 1500.0), v: Vec2::new(-180.0, 0.0) };
    let params  = Params  { dt: 0.02, nav_const: 4.0, kill_radius: 10.0, max_time: 60.0 };

    let mut sim = Sim::new(missile, target, params);

    let mut missile_traj: Vec<Vec2> = Vec::new();
    let mut target_traj: Vec<Vec2> = Vec::new();

    let origin = Vec2::new(0.0, 0.0);
    let scale = 0.08;

    loop {
        clear_background(BLACK);

        for _ in 0..3 {
            missile_traj.push(sim.missile.p);
            target_traj.push(sim.target.p);

            match sim.step() {
                Status::Running => {}
                _ => break,
            }
        }

        let center = Vec2::new(screen_width() * 0.5, screen_height() * 0.5);

        // Draw trajectories
        for w in missile_traj.windows(2) {
            let a = world_to_screen(w[0], origin, scale) + center;
            let b = world_to_screen(w[1], origin, scale) + center;
            draw_line(a.x, a.y, b.x, b.y, 2.0, DARKGRAY);
        }
        for w in target_traj.windows(2) {
            let a = world_to_screen(w[0], origin, scale) + center;
            let b = world_to_screen(w[1], origin, scale) + center;
            draw_line(a.x, a.y, b.x, b.y, 2.0, GRAY);
        }

        // Draw missile + target
        let m = world_to_screen(sim.missile.p, origin, scale) + center;
        let t = world_to_screen(sim.target.p, origin, scale) + center;
        draw_circle(m.x, m.y, 5.0, WHITE);
        draw_circle(t.x, t.y, 6.0, LIGHTGRAY);

        // LOS line
        draw_line(m.x, m.y, t.x, t.y, 1.0, DARKGRAY);

        draw_text(
            &format!(
                "t={:.2}s  range={:.1}m  Vc={:.1}m/s  los_rate={:.5}  a_cmd={:.1}",
                sim.last.t, sim.last.range, sim.last.closing_speed, sim.last.los_rate, sim.last.a_cmd
            ),
            16.0,
            24.0,
            20.0,
            GRAY,
        );

        next_frame().await
    }
}