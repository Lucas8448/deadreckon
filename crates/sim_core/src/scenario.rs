//! Built-in 3D engagement scenarios.
//!
//! Conventions: missile starts at origin moving down +X; target somewhere
//! downrange. Y is altitude, Z is cross-range. Speeds in m/s, ranges in m.

use physics_sandbox::math::Vec3;

use crate::{Missile, Params, Target, TargetManeuver};

#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: &'static str,
    pub missile: Missile,
    pub target: Target,
    pub params: Params,
}

impl Scenario {
    pub fn all() -> Vec<Scenario> {
        vec![
            baseline(),
            head_on(),
            crossing(),
            fast_target(),
            turning_target(),
            weaving_target(),
        ]
    }

    pub fn by_name(name: &str) -> Option<Scenario> {
        Self::all()
            .into_iter()
            .find(|s| s.name.eq_ignore_ascii_case(name))
    }
}

fn default_params() -> Params {
    Params {
        dt: 0.02,
        nav_const: 4.0,
        kill_radius: 15.0,
        max_time: 60.0,
    }
}

fn missile_default() -> Missile {
    Missile {
        p: Vec3::new(0.0, 0.0, 0.0),
        v: Vec3::new(250.0, 0.0, 0.0),
        mass: 100.0,
        a_max: 80.0,
        gimbal_limit: 0.7, // ~40°
    }
}

pub fn baseline() -> Scenario {
    Scenario {
        name: "baseline",
        missile: missile_default(),
        target: Target {
            p: Vec3::new(6000.0, 1500.0, 0.0),
            v: Vec3::new(-180.0, 0.0, 0.0),
            mass: 5_000.0,
            maneuver: TargetManeuver::ConstantVelocity,
        },
        params: default_params(),
    }
}

pub fn head_on() -> Scenario {
    Scenario {
        name: "head_on",
        missile: Missile {
            v: Vec3::new(300.0, 0.0, 0.0),
            ..missile_default()
        },
        target: Target {
            // True head-on: same altitude (Y=0) and cross-range (Z=0) as the
            // missile, flying back along -X. No offset bypass.
            p: Vec3::new(8000.0, 0.0, 0.0),
            v: Vec3::new(-200.0, 0.0, 0.0),
            mass: 5_000.0,
            maneuver: TargetManeuver::ConstantVelocity,
        },
        params: default_params(),
    }
}

pub fn crossing() -> Scenario {
    Scenario {
        name: "crossing",
        missile: Missile {
            v: Vec3::new(280.0, 0.0, 0.0),
            a_max: 100.0,
            ..missile_default()
        },
        target: Target {
            p: Vec3::new(5000.0, 1000.0, 0.0),
            // Crossing in the cross-range (Z) direction so the engagement
            // exercises out-of-plane guidance.
            v: Vec3::new(0.0, 0.0, 200.0),
            mass: 5_000.0,
            maneuver: TargetManeuver::ConstantVelocity,
        },
        params: default_params(),
    }
}

pub fn fast_target() -> Scenario {
    Scenario {
        name: "fast_target",
        missile: Missile {
            v: Vec3::new(350.0, 0.0, 0.0),
            a_max: 120.0,
            ..missile_default()
        },
        target: Target {
            p: Vec3::new(10_000.0, 2_000.0, 1_500.0),
            v: Vec3::new(-400.0, 50.0, -50.0),
            mass: 5_000.0,
            maneuver: TargetManeuver::ConstantVelocity,
        },
        params: Params {
            nav_const: 5.0,
            kill_radius: 20.0,
            max_time: 45.0,
            ..default_params()
        },
    }
}

pub fn turning_target() -> Scenario {
    Scenario {
        name: "turning",
        missile: Missile {
            v: Vec3::new(300.0, 0.0, 0.0),
            a_max: 120.0,
            ..missile_default()
        },
        target: Target {
            p: Vec3::new(5000.0, 800.0, 0.0),
            v: Vec3::new(-150.0, 0.0, 0.0),
            // Yaw turn around the world Y (up) axis — ~8.6 deg/s.
            maneuver: TargetManeuver::ConstantTurn {
                axis: Vec3::new(0.0, 1.0, 0.0),
                omega: 0.15,
            },
            mass: 5_000.0,
        },
        params: Params {
            nav_const: 4.5,
            kill_radius: 18.0,
            ..default_params()
        },
    }
}

pub fn weaving_target() -> Scenario {
    Scenario {
        name: "weaving",
        missile: Missile {
            v: Vec3::new(320.0, 0.0, 0.0),
            a_max: 140.0,
            ..missile_default()
        },
        target: Target {
            p: Vec3::new(6000.0, 1000.0, 0.0),
            v: Vec3::new(-180.0, 0.0, 0.0),
            // Weave around the up axis: lateral (Z-direction) sinusoidal
            // acceleration ~15g peak.
            maneuver: TargetManeuver::Weave {
                axis: Vec3::new(0.0, 1.0, 0.0),
                amplitude: 150.0,
                freq: 0.6,
            },
            mass: 5_000.0,
        },
        params: Params {
            nav_const: 5.0,
            kill_radius: 20.0,
            ..default_params()
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Sim, Status, TargetManeuver};

    #[test]
    fn constant_velocity_scenarios_hit() {
        for scenario in Scenario::all() {
            if !matches!(scenario.target.maneuver, TargetManeuver::ConstantVelocity) {
                continue;
            }
            let mut sim = Sim::new(scenario.missile, scenario.target, scenario.params);
            let mut status = Status::Running;
            while status == Status::Running {
                status = sim.step();
            }
            assert_eq!(
                status,
                Status::Hit,
                "scenario '{}' did not hit (final range {:.2}m)",
                scenario.name,
                sim.last.range
            );
        }
    }

    #[test]
    fn maneuvering_scenarios_run_to_completion() {
        for scenario in Scenario::all() {
            if matches!(scenario.target.maneuver, TargetManeuver::ConstantVelocity) {
                continue;
            }
            let mut sim = Sim::new(scenario.missile, scenario.target, scenario.params);
            let mut status = Status::Running;
            while status == Status::Running {
                status = sim.step();
            }
            assert!(
                matches!(status, Status::Hit | Status::Timeout | Status::LostLock),
                "scenario '{}' unexpected status: {:?}",
                scenario.name,
                status
            );
        }
    }
}
