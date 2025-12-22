use glam::Vec2;

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

// Baseline
pub fn baseline() -> Scenario {
    Scenario {
        name: "baseline",
        missile: Missile {
            p: Vec2::new(0.0, 0.0),
            v: Vec2::new(250.0, 0.0),
            a_max: 60.0,
        },
        target: Target {
            p: Vec2::new(6000.0, 1500.0),
            v: Vec2::new(-180.0, 0.0),
            maneuver: TargetManeuver::ConstantVelocity,
        },
        params: Params {
            dt: 0.02,
            nav_const: 4.0,
            kill_radius: 10.0,
            max_time: 60.0,
        },
    }
}

// target flying directly toward missile.
pub fn head_on() -> Scenario {
    Scenario {
        name: "head_on",
        missile: Missile {
            p: Vec2::new(0.0, 0.0),
            v: Vec2::new(300.0, 0.0),
            a_max: 60.0,
        },
        target: Target {
            p: Vec2::new(8000.0, 0.0),
            v: Vec2::new(-200.0, 0.0),
            maneuver: TargetManeuver::ConstantVelocity,
        },
        params: Params {
            dt: 0.02,
            nav_const: 4.0,
            kill_radius: 10.0,
            max_time: 60.0,
        },
    }
}

// target moving perpendicular to initial LOS.
pub fn crossing() -> Scenario {
    Scenario {
        name: "crossing",
        missile: Missile {
            p: Vec2::new(0.0, 0.0),
            v: Vec2::new(280.0, 0.0),
            a_max: 80.0,
        },
        target: Target {
            p: Vec2::new(5000.0, 0.0),
            v: Vec2::new(0.0, 150.0), // moving up
            maneuver: TargetManeuver::ConstantVelocity,
        },
        params: Params {
            dt: 0.02,
            nav_const: 4.0,
            kill_radius: 10.0,
            max_time: 60.0,
        },
    }
}

// supersonic target
pub fn fast_target() -> Scenario {
    Scenario {
        name: "fast_target",
        missile: Missile {
            p: Vec2::new(0.0, 0.0),
            v: Vec2::new(350.0, 0.0),
            a_max: 100.0,
        },
        target: Target {
            p: Vec2::new(10000.0, 2000.0),
            v: Vec2::new(-400.0, 50.0),
            maneuver: TargetManeuver::ConstantVelocity,
        },
        params: Params {
            dt: 0.02,
            nav_const: 5.0, // higher N
            kill_radius: 15.0,
            max_time: 45.0,
        },
    }
}

/// Target executing a constant-rate turn (the "oh damn" moment).
pub fn turning_target() -> Scenario {
    Scenario {
        name: "turning",
        missile: Missile {
            p: Vec2::new(0.0, 0.0),
            v: Vec2::new(300.0, 0.0),
            a_max: 100.0,
        },
        target: Target {
            p: Vec2::new(5000.0, 500.0),
            v: Vec2::new(-150.0, 0.0),
            maneuver: TargetManeuver::ConstantTurn { omega: 0.15 }, // ~8.6 deg/s
        },
        params: Params {
            dt: 0.02,
            nav_const: 4.5,
            kill_radius: 12.0,
            max_time: 60.0,
        },
    }
}

/// Target weaving sinusoidally—hard to track.
pub fn weaving_target() -> Scenario {
    Scenario {
        name: "weaving",
        missile: Missile {
            p: Vec2::new(0.0, 0.0),
            v: Vec2::new(320.0, 0.0),
            a_max: 120.0,
        },
        target: Target {
            p: Vec2::new(6000.0, 0.0),
            v: Vec2::new(-180.0, 0.0),
            maneuver: TargetManeuver::Weave {
                amplitude: 150.0, // m/s² peak (~15g, aggressive)
                freq: 0.6,        // Hz
            },
        },
        params: Params {
            dt: 0.02,
            nav_const: 5.0,
            kill_radius: 15.0,
            max_time: 60.0,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Sim, Status, TargetManeuver};

    /// Non-maneuvering scenarios should always hit.
    #[test]
    fn constant_velocity_scenarios_hit() {
        for scenario in Scenario::all() {
            // Only test constant-velocity targets (guaranteed hits)
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
                "scenario '{}' did not hit (got {:?})",
                scenario.name,
                status
            );
        }
    }

    /// Maneuvering scenarios should at least complete without panic.
    #[test]
    fn maneuvering_scenarios_run() {
        for scenario in Scenario::all() {
            if matches!(scenario.target.maneuver, TargetManeuver::ConstantVelocity) {
                continue;
            }
            let mut sim = Sim::new(scenario.missile, scenario.target, scenario.params);
            let mut status = Status::Running;
            while status == Status::Running {
                status = sim.step();
            }
            // Just verify it terminates (Hit or Timeout)
            assert!(
                status == Status::Hit || status == Status::Timeout,
                "scenario '{}' unexpected status: {:?}",
                scenario.name,
                status
            );
        }
    }
}
