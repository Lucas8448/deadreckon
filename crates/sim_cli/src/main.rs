use glam::Vec2;
use sim_core::{Missile, Params, Sim, Status, Target};

fn main() {
    let missile = Missile {
        p: Vec2::new(0.0, 0.0),
        v: Vec2::new(250.0, 0.0), // m/s
        a_max: 60.0, // (60 m/s^2)
    };

    let target = Target {
        p: Vec2::new(6000.0, 1500.0),
        v: Vec2::new(-180.0, 0.0),
    };

    let params = Params {
        dt: 0.02,
        nav_const: 4.0,
        kill_radius: 10.0,
        max_time: 60.0,
    };

    let mut sim = Sim::new(missile, target, params);

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