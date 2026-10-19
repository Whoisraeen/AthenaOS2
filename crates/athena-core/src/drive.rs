//! Explicit homeostatic policy. These scores are engineering heuristics, not
//! evidence of affect, consciousness, or a variational free-energy model.
use crate::{BodyFrame, MotorOutput};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drive {
    Protect,
    Cool,
    Recharge,
    Consolidate,
    Observe,
    Task,
    Explore,
}

#[derive(Clone, Copy, Debug)]
pub struct DriveDecision {
    pub drive: Drive,
    pub urgency: f32,
}

pub fn evaluate(body: &BodyFrame, spinal: &MotorOutput) -> DriveDecision {
    let (drive, urgency) =
        if spinal.faults.any() || spinal.active_reflex_mask != 0 || !body.is_valid() {
            (Drive::Protect, 1.0)
        } else if body.max_temperature_c() >= 70.0 {
            (
                Drive::Cool,
                ((body.max_temperature_c() - 50.0) / 40.0).clamp(0.0, 1.0),
            )
        } else if body.battery_soc < 0.3 {
            (
                Drive::Recharge,
                (1.0 - body.battery_soc / 0.3).clamp(0.0, 1.0),
            )
        } else if body.docked
            && body.battery_soc >= 0.5
            && body.max_temperature_c() < 50.0
            && body.uncertainty < 0.5
            && body.task_demand < 0.1
            && !spinal.watchdog_expired
        {
            (Drive::Consolidate, 0.2)
        } else if body.uncertainty >= 0.7 {
            (Drive::Observe, body.uncertainty)
        } else if body.task_demand > 0.1 {
            (Drive::Task, body.task_demand)
        } else {
            (Drive::Explore, 0.25)
        };
    DriveDecision { drive, urgency }
}
