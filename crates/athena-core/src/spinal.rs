use crate::{bus::SensoryReader, *};

pub const EVENTS_PER_CLASS_PER_TICK: usize = 16;
pub const WITHDRAWAL_HOLD_US: u64 = 200_000;
pub const PAIN_THRESHOLD: u16 = 48_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlMode {
    Voluntary,
    AutonomicDamping,
    Withdrawal,
    Disabled,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Faults(pub u16);

impl Faults {
    pub const SENSOR_INVALID: u16 = 1;
    pub const SENSOR_STALE: u16 = 2;
    pub const CLOCK_REGRESSION: u16 = 4;
    pub const INGRESS_OVERFLOW: u16 = 8;
    pub const EVENT_INVALID: u16 = 16;
    pub const COMPUTATION_INVALID: u16 = 32;

    pub fn any(self) -> bool {
        self.0 != 0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MotorOutput {
    pub torque_nm: [f32; JOINTS],
    pub drive_enabled: [bool; JOINTS],
    pub modes: [ControlMode; LIMBS],
    pub active_reflex_mask: u8,
    pub pain_salience: f32,
    pub watchdog_expired: bool,
    /// Edge notification to the cognitive runtime; hardware IPI is a platform hook.
    pub notify_cortex: bool,
    pub faults: Faults,
    pub processed_spikes: usize,
    pub rejected_intents: u64,
    pub missed_ticks: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidConfiguration(pub usize);

pub struct SpinalKernel {
    config: BodyConfig,
    withdrawal_until: [u64; LIMBS],
    damping_until: [u64; LIMBS],
    last_intent: Option<MotorIntent>,
    previous_torque: [f32; JOINTS],
    last_tick_us: Option<u64>,
    faults: Faults,
    rejected_intents: u64,
    missed_ticks: u64,
}

impl SpinalKernel {
    pub fn new(config: BodyConfig) -> Result<Self, InvalidConfiguration> {
        for (i, joint) in config.joints.iter().enumerate() {
            if !joint.is_valid() {
                return Err(InvalidConfiguration(i));
            }
        }
        Ok(Self {
            config,
            withdrawal_until: [0; LIMBS],
            damping_until: [0; LIMBS],
            last_intent: None,
            previous_torque: [0.0; JOINTS],
            last_tick_us: None,
            faults: Faults::default(),
            rejected_intents: 0,
            missed_ticks: 0,
        })
    }

    fn accept_intent(&mut self, now_us: u64, intent: MotorIntent) {
        let valid = intent.sequence > self.last_intent.map_or(0, |i| i.sequence)
            && intent.issued_at_us <= now_us
            && intent.valid_until_us > now_us
            && intent.valid_until_us > intent.issued_at_us
            && intent.valid_until_us - intent.issued_at_us <= WATCHDOG_US
            && match intent.goal {
                MotorGoal::HoldRestPose => true,
                MotorGoal::JointPose(targets) => {
                    targets.iter().zip(self.config.joints).all(|(q, cfg)| {
                        q.is_finite() && (cfg.min_position_rad..=cfg.max_position_rad).contains(q)
                    })
                }
            };
        if valid {
            self.last_intent = Some(intent);
        } else {
            self.rejected_intents = self.rejected_intents.saturating_add(1);
        }
    }

    fn evaluate_spike(&mut self, now_us: u64, spike: Spike) -> bool {
        if spike.limb_mask == 0
            || spike.limb_mask & !0x3f != 0
            || spike.timestamp_us > now_us
            || now_us - spike.timestamp_us > 10_000
        {
            self.faults.0 |= Faults::EVENT_INVALID;
            return true;
        }
        let pain = spike.modality == Modality::Nociception && spike.intensity >= PAIN_THRESHOLD;
        let damping = matches!(spike.modality, Modality::Instability | Modality::Overload);
        for limb in Limb::ALL {
            let i = limb as usize;
            if spike.limb_mask & limb.mask() == 0 {
                continue;
            }
            if pain {
                if self.withdrawal_until[i] <= now_us {
                    // Do not carry a voluntary torque through a hard reflex handover.
                    for (j, cfg) in self.config.joints.iter().enumerate() {
                        if cfg.limb == limb {
                            self.previous_torque[j] = 0.0;
                        }
                    }
                }
                self.withdrawal_until[i] = now_us.saturating_add(WITHDRAWAL_HOLD_US);
            } else if damping {
                self.damping_until[i] = now_us.saturating_add(50_000);
            }
        }
        pain || damping
    }

    /// One bounded logical tick. No heap, I/O, locks, waiting, or model inference.
    /// `now_us` must come from the platform clock, independently of sensor time.
    /// Faults latch until the supervisor reconstructs the kernel after diagnosis.
    pub fn tick<const N: usize>(
        &mut self,
        now_us: u64,
        body: &BodyFrame,
        intent: Option<MotorIntent>,
        ingress: &mut SensoryReader<'_, N>,
    ) -> MotorOutput {
        let previous_faults = self.faults;
        if let Some(last) = self.last_tick_us {
            if now_us <= last {
                self.faults.0 |= Faults::CLOCK_REGRESSION;
            } else if now_us - last > TICK_US {
                self.missed_ticks = self
                    .missed_ticks
                    .saturating_add((now_us - last - 1) / TICK_US);
            }
        }
        self.last_tick_us = Some(now_us);
        if !body.is_valid() {
            self.faults.0 |= Faults::SENSOR_INVALID;
        }
        if body.sampled_at_us > now_us
            || now_us.saturating_sub(body.sampled_at_us) > MAX_SENSOR_AGE_US
        {
            self.faults.0 |= Faults::SENSOR_STALE;
        }
        if ingress.take_overflow() {
            self.faults.0 |= Faults::INGRESS_OVERFLOW;
        }
        if let Some(intent) = intent {
            self.accept_intent(now_us, intent);
        }

        let mut notify_cortex = false;
        let mut processed_spikes = 0;
        // Critical ingress is always examined before ordinary sensory traffic.
        for critical in [true, false] {
            for _ in 0..EVENTS_PER_CLASS_PER_TICK {
                let event = if critical {
                    ingress.critical()
                } else {
                    ingress.ordinary()
                };
                let Some(event) = event else {
                    break;
                };
                notify_cortex |= self.evaluate_spike(now_us, event);
                processed_spikes += 1;
            }
        }
        let watchdog_expired = self.last_intent.is_none_or(|i| now_us >= i.valid_until_us);
        let mut output = MotorOutput {
            torque_nm: [0.0; JOINTS],
            drive_enabled: [true; JOINTS],
            modes: [ControlMode::Voluntary; LIMBS],
            active_reflex_mask: 0,
            pain_salience: 0.0,
            watchdog_expired,
            notify_cortex,
            faults: self.faults,
            processed_spikes,
            rejected_intents: self.rejected_intents,
            missed_ticks: self.missed_ticks,
        };
        for limb in Limb::ALL {
            let i = limb as usize;
            output.modes[i] = if self.withdrawal_until[i] > now_us {
                output.active_reflex_mask |= limb.mask();
                output.pain_salience = 1.0;
                ControlMode::Withdrawal
            } else if self.damping_until[i] > now_us || watchdog_expired {
                ControlMode::AutonomicDamping
            } else {
                ControlMode::Voluntary
            };
        }
        for (j, cfg) in self.config.joints.iter().enumerate() {
            if self.faults.any() {
                break;
            }
            let state = body.joints[j];
            let mode = output.modes[cfg.limb as usize];
            let raw = match mode {
                ControlMode::Withdrawal => {
                    cfg.stiffness * (cfg.withdrawal_position_rad - state.position_rad)
                        - cfg.damping * state.velocity_rad_s
                }
                ControlMode::Voluntary => {
                    let target = match self.last_intent.map(|i| i.goal) {
                        Some(MotorGoal::JointPose(targets)) => targets[j],
                        _ => cfg.rest_position_rad,
                    };
                    cfg.stiffness * (target - state.position_rad)
                        - cfg.damping * state.velocity_rad_s
                }
                ControlMode::AutonomicDamping => -cfg.damping * state.velocity_rad_s,
                ControlMode::Disabled => 0.0,
            };
            if !raw.is_finite() {
                self.faults.0 |= Faults::COMPUTATION_INVALID;
                break;
            }
            let thermal_factor = ((cfg.cutoff_c - state.temperature_c)
                / (cfg.cutoff_c - cfg.derate_start_c))
                .clamp(0.0, 1.0);
            let limit = cfg.max_torque_nm * thermal_factor;
            let step = cfg.max_torque_rate_nm_s * (TICK_US as f32 / 1_000_000.0);
            // Envelope is applied after slew limiting: a new safety limit wins immediately.
            let torque = raw
                .clamp(
                    self.previous_torque[j] - step,
                    self.previous_torque[j] + step,
                )
                .clamp(-limit, limit);
            let enabled = state.temperature_c < cfg.cutoff_c;
            output.torque_nm[j] = if enabled { torque } else { 0.0 };
            output.drive_enabled[j] = enabled;
        }
        if self.faults.any() {
            output.torque_nm.fill(0.0);
            output.drive_enabled.fill(false);
            output.modes.fill(ControlMode::Disabled);
            output.notify_cortex |= self.faults != previous_faults;
        }
        output.faults = self.faults;
        self.previous_torque = output.torque_nm;
        output
    }
}
