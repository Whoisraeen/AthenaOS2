pub const JOINTS: usize = 42;
pub const LIMBS: usize = 6;
pub const TICK_US: u64 = 250;
pub const WATCHDOG_US: u64 = 50_000;
pub const MAX_SENSOR_AGE_US: u64 = 1_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Limb {
    LeftArm,
    RightArm,
    LeftLeg,
    RightLeg,
    Torso,
    Head,
}

impl Limb {
    pub const ALL: [Self; LIMBS] = [
        Self::LeftArm,
        Self::RightArm,
        Self::LeftLeg,
        Self::RightLeg,
        Self::Torso,
        Self::Head,
    ];

    pub const fn mask(self) -> u8 {
        1 << self as u8
    }
}

#[derive(Clone, Copy, Debug)]
pub struct JointState {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub measured_torque_nm: f32,
    pub temperature_c: f32,
}

impl JointState {
    pub const REST: Self = Self {
        position_rad: 0.0,
        velocity_rad_s: 0.0,
        measured_torque_nm: 0.0,
        temperature_c: 25.0,
    };
}

#[derive(Clone, Copy, Debug)]
pub struct BodyFrame {
    pub sampled_at_us: u64,
    pub joints: [JointState; JOINTS],
    /// Supplied by a platform estimator; Athena does not infer a support polygon yet.
    pub support_error_m: [f32; 3],
    pub battery_soc: f32,
    pub uncertainty: f32,
    pub task_demand: f32,
    pub docked: bool,
}

impl BodyFrame {
    pub const REST: Self = Self {
        sampled_at_us: 0,
        joints: [JointState::REST; JOINTS],
        support_error_m: [0.0; 3],
        battery_soc: 0.8,
        uncertainty: 0.1,
        task_demand: 0.5,
        docked: false,
    };

    pub fn is_valid(&self) -> bool {
        let unit = |v: f32| v.is_finite() && (0.0..=1.0).contains(&v);
        unit(self.battery_soc)
            && unit(self.uncertainty)
            && unit(self.task_demand)
            && self.support_error_m.iter().all(|v| v.is_finite())
            && self.joints.iter().all(|j| {
                j.position_rad.is_finite()
                    && j.velocity_rad_s.is_finite()
                    && j.measured_torque_nm.is_finite()
                    && j.temperature_c.is_finite()
                    && (-50.0..=250.0).contains(&j.temperature_c)
            })
    }

    pub fn max_temperature_c(&self) -> f32 {
        self.joints
            .iter()
            .fold(-50.0_f32, |t, j| t.max(j.temperature_c))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modality {
    Nociception,
    Instability,
    Overload,
    Slip,
    Salience,
}

#[derive(Clone, Copy, Debug)]
pub struct Spike {
    pub timestamp_us: u64,
    pub source_id: u16,
    /// Q0.16 normalized salience, not a physical temperature or force.
    pub intensity: u16,
    /// All set bits are handled. Zero and bits outside the six limbs are invalid.
    pub limb_mask: u8,
    pub modality: Modality,
}

impl Spike {
    pub fn is_critical(self) -> bool {
        matches!(
            self.modality,
            Modality::Nociception | Modality::Instability | Modality::Overload
        )
    }
}

#[derive(Clone, Copy, Debug)]
pub enum MotorGoal {
    HoldRestPose,
    /// A bounded joint-space fixture interface; Cartesian planning is a later layer.
    JointPose([f32; JOINTS]),
}

#[derive(Clone, Copy, Debug)]
pub struct MotorIntent {
    pub sequence: u64,
    pub issued_at_us: u64,
    pub valid_until_us: u64,
    pub goal: MotorGoal,
}

impl MotorIntent {
    pub const fn hold(sequence: u64, now_us: u64) -> Self {
        Self {
            sequence,
            issued_at_us: now_us,
            valid_until_us: now_us.saturating_add(WATCHDOG_US),
            goal: MotorGoal::HoldRestPose,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct JointConfig {
    pub limb: Limb,
    pub min_position_rad: f32,
    pub max_position_rad: f32,
    pub rest_position_rad: f32,
    /// Must be calibrated to move away from a hazard on an actual mechanism.
    pub withdrawal_position_rad: f32,
    pub max_torque_nm: f32,
    pub max_torque_rate_nm_s: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub derate_start_c: f32,
    pub cutoff_c: f32,
}

impl JointConfig {
    pub const fn fixture(limb: Limb) -> Self {
        Self {
            limb,
            min_position_rad: -1.5,
            max_position_rad: 1.5,
            rest_position_rad: 0.0,
            withdrawal_position_rad: -0.4,
            max_torque_nm: 12.0,
            max_torque_rate_nm_s: 2_000.0,
            stiffness: 20.0,
            damping: 2.0,
            derate_start_c: 70.0,
            cutoff_c: 90.0,
        }
    }

    pub fn is_valid(&self) -> bool {
        [
            self.min_position_rad,
            self.max_position_rad,
            self.rest_position_rad,
            self.withdrawal_position_rad,
            self.max_torque_nm,
            self.max_torque_rate_nm_s,
            self.stiffness,
            self.damping,
            self.derate_start_c,
            self.cutoff_c,
        ]
        .iter()
        .all(|v| v.is_finite())
            && self.min_position_rad < self.max_position_rad
            && self.min_position_rad >= -100.0
            && self.max_position_rad <= 100.0
            && (self.min_position_rad..=self.max_position_rad).contains(&self.rest_position_rad)
            && (self.min_position_rad..=self.max_position_rad)
                .contains(&self.withdrawal_position_rad)
            && (0.0..=1_000.0).contains(&self.max_torque_nm)
            && self.max_torque_nm > 0.0
            && (0.0..=1_000_000.0).contains(&self.max_torque_rate_nm_s)
            && self.max_torque_rate_nm_s > 0.0
            && (0.0..=1_000.0).contains(&self.stiffness)
            && (0.0..=1_000.0).contains(&self.damping)
            && self.derate_start_c < self.cutoff_c
            && self.derate_start_c >= -50.0
            && self.cutoff_c <= 250.0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BodyConfig {
    pub joints: [JointConfig; JOINTS],
}

impl BodyConfig {
    /// Illustrative six-chain morphology, deliberately not a hardware calibration.
    pub const fn fixture() -> Self {
        let mut joints = [JointConfig::fixture(Limb::LeftArm); JOINTS];
        let mut i = 0;
        while i < JOINTS {
            joints[i].limb = Limb::ALL[i / 7];
            i += 1;
        }
        Self { joints }
    }
}
