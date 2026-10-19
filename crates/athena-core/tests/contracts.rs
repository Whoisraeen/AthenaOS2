use athena_core::{
    bus::{SensoryBus, Snapshot, SpscRing},
    drive::{self, Drive},
    *,
};
use std::thread;

fn pain(mask: u8, timestamp_us: u64) -> Spike {
    Spike {
        timestamp_us,
        source_id: 1,
        intensity: u16::MAX,
        limb_mask: mask,
        modality: Modality::Nociception,
    }
}

#[test]
fn ring_is_bounded_and_preserves_fifo_through_wraps() {
    let mut queue = SpscRing::<u64, 4>::new();
    let (mut tx, mut rx) = queue.split();
    for cycle in 0..100 {
        for i in 0..4 {
            tx.push(cycle * 4 + i).unwrap();
        }
        assert_eq!(tx.push(999), Err(999));
        for i in 0..4 {
            assert_eq!(rx.pop(), Some(cycle * 4 + i));
        }
        assert_eq!(rx.pop(), None);
    }
}

#[test]
fn concurrent_queue_transfers_initialized_payloads_in_order() {
    let mut queue = SpscRing::<[u64; 8], 64>::new();
    let (mut tx, mut rx) = queue.split();
    thread::scope(|scope| {
        scope.spawn(move || {
            for i in 0..50_000 {
                let mut value = [i; 8];
                while let Err(returned) = tx.push(value) {
                    value = returned;
                    thread::yield_now();
                }
            }
        });
        for expected in 0..50_000 {
            let value = loop {
                if let Some(value) = rx.pop() {
                    break value;
                }
                thread::yield_now();
            };
            assert_eq!(value, [expected; 8]);
        }
    });
}

#[test]
fn snapshot_never_returns_a_torn_frame_under_concurrency() {
    let mut snapshot = Snapshot::new([0_u64; 64]);
    {
        let (mut writer, mut reader) = snapshot.split();
        thread::scope(|scope| {
            let handle = scope.spawn(move || {
                for i in 1..=50_000 {
                    writer.publish([i; 64]);
                }
            });
            let mut last = 0;
            for _ in 0..50_000 {
                let frame = reader.latest();
                assert!(frame.iter().all(|word| *word == frame[0]));
                assert!(frame[0] >= last);
                last = frame[0];
            }
            handle.join().unwrap();
            assert_eq!(reader.latest(), [50_000; 64]);
        });
    }
    // Re-leasing the endpoints must preserve ownership of all three slots.
    let (mut writer, mut reader) = snapshot.split();
    writer.publish([50_001; 64]);
    assert_eq!(reader.latest(), [50_001; 64]);
}

#[test]
fn pain_preempts_every_addressed_limb_and_preserves_other_limbs() {
    let mut bus = SensoryBus::<64>::new();
    let (mut tx, mut rx) = bus.split();
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    tx.send(pain(Limb::LeftArm.mask() | Limb::RightArm.mask(), 0))
        .unwrap();
    let output = kernel.tick(
        0,
        &BodyFrame::REST,
        Some(MotorIntent {
            goal: MotorGoal::JointPose([1.0; JOINTS]),
            ..MotorIntent::hold(1, 0)
        }),
        &mut rx,
    );
    assert_eq!(output.active_reflex_mask, 3);
    assert_eq!(output.modes[0], ControlMode::Withdrawal);
    assert_eq!(output.modes[1], ControlMode::Withdrawal);
    assert_eq!(output.modes[2], ControlMode::Voluntary);
    assert!(output.torque_nm[0] < 0.0);
    assert!(output.torque_nm[14] > 0.0);
    assert!(output.notify_cortex);
}

#[test]
fn critical_spikes_bypass_a_full_ordinary_queue_and_tick_work_is_bounded() {
    let mut bus = SensoryBus::<64>::new();
    let (mut tx, mut rx) = bus.split();
    for _ in 0..64 {
        tx.send(Spike {
            modality: Modality::Salience,
            ..pain(1, 0)
        })
        .unwrap();
    }
    for _ in 0..64 {
        tx.send(pain(1, 0)).unwrap();
    }
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    let output = kernel.tick(0, &BodyFrame::REST, Some(MotorIntent::hold(1, 0)), &mut rx);
    assert_eq!(output.processed_spikes, 32);
    assert_eq!(output.active_reflex_mask, 1);
    assert!(!output.faults.any());
}

#[test]
fn overflow_is_visible_even_when_the_hazard_event_cannot_be_queued() {
    let mut bus = SensoryBus::<2>::new();
    let (mut tx, mut rx) = bus.split();
    tx.send(pain(1, 0)).unwrap();
    tx.send(pain(1, 0)).unwrap();
    assert!(tx.send(pain(2, 0)).is_err());
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    let output = kernel.tick(0, &BodyFrame::REST, Some(MotorIntent::hold(1, 0)), &mut rx);
    assert_ne!(output.faults.0 & Faults::INGRESS_OVERFLOW, 0);
    assert!(output.drive_enabled.iter().all(|enabled| !enabled));
    assert!(output.torque_nm.iter().all(|q| *q == 0.0));
}

#[test]
fn a_replayed_sequence_cannot_extend_the_watchdog() {
    let mut bus = SensoryBus::<4>::new();
    let (_, mut rx) = bus.split();
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    kernel.tick(0, &BodyFrame::REST, Some(MotorIntent::hold(1, 0)), &mut rx);
    let body = BodyFrame {
        sampled_at_us: 49_000,
        ..BodyFrame::REST
    };
    let output = kernel.tick(49_000, &body, Some(MotorIntent::hold(1, 49_000)), &mut rx);
    assert_eq!(output.rejected_intents, 1);
    let body = BodyFrame {
        sampled_at_us: WATCHDOG_US,
        ..body
    };
    let output = kernel.tick(WATCHDOG_US, &body, None, &mut rx);
    assert!(output.watchdog_expired);
    assert_eq!(output.modes, [ControlMode::AutonomicDamping; LIMBS]);
    assert!(!output.faults.any());
}

#[test]
fn malformed_cortical_intents_are_rejected_without_touching_motor_limits() {
    for intent in [
        MotorIntent::hold(0, 0),
        MotorIntent::hold(1, 1),
        MotorIntent {
            valid_until_us: WATCHDOG_US + 1,
            ..MotorIntent::hold(1, 0)
        },
        MotorIntent {
            goal: MotorGoal::JointPose([f32::NAN; JOINTS]),
            ..MotorIntent::hold(1, 0)
        },
        MotorIntent {
            goal: MotorGoal::JointPose([99.0; JOINTS]),
            ..MotorIntent::hold(1, 0)
        },
    ] {
        let mut bus = SensoryBus::<4>::new();
        let (_, mut rx) = bus.split();
        let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
        let output = kernel.tick(0, &BodyFrame::REST, Some(intent), &mut rx);
        assert!(output.watchdog_expired);
        assert_eq!(output.rejected_intents, 1);
        assert!(output.torque_nm.iter().all(|q| *q == 0.0));
    }
}

#[test]
fn invalid_sensors_and_arithmetic_overflow_disable_every_drive_without_panicking() {
    for (temperature, velocity) in [(f32::NAN, 0.0), (25.0, f32::MAX)] {
        let mut bus = SensoryBus::<4>::new();
        let (_, mut rx) = bus.split();
        let mut body = BodyFrame::REST;
        body.joints[0].temperature_c = temperature;
        body.joints[0].velocity_rad_s = velocity;
        let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
        let output = kernel.tick(0, &body, Some(MotorIntent::hold(1, 0)), &mut rx);
        assert!(output.faults.any());
        assert_eq!(output.drive_enabled, [false; JOINTS]);
        assert_eq!(output.torque_nm, [0.0; JOINTS]);
        let body = BodyFrame {
            sampled_at_us: TICK_US,
            ..BodyFrame::REST
        };
        assert!(kernel.tick(TICK_US, &body, None, &mut rx).faults.any());
    }
}

#[test]
fn stale_sensor_frames_and_regressing_clocks_latch_faults() {
    let mut bus = SensoryBus::<4>::new();
    let (_, mut rx) = bus.split();
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    let first = kernel.tick(2_000, &BodyFrame::REST, None, &mut rx);
    assert_ne!(first.faults.0 & Faults::SENSOR_STALE, 0);
    let body = BodyFrame {
        sampled_at_us: 1_000,
        ..BodyFrame::REST
    };
    let second = kernel.tick(1_000, &body, None, &mut rx);
    assert_ne!(second.faults.0 & Faults::CLOCK_REGRESSION, 0);
}

#[test]
fn invalid_limb_masks_fault_instead_of_selecting_a_default_limb() {
    for mask in [0, 0x80, 0x41] {
        let mut bus = SensoryBus::<4>::new();
        let (mut tx, mut rx) = bus.split();
        tx.send(pain(mask, 0)).unwrap();
        let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
        let output = kernel.tick(0, &BodyFrame::REST, None, &mut rx);
        assert_ne!(output.faults.0 & Faults::EVENT_INVALID, 0);
        assert_eq!(output.drive_enabled, [false; JOINTS]);
    }
}

#[test]
fn torque_envelope_overrides_slew_when_a_joint_overheats() {
    let mut bus = SensoryBus::<4>::new();
    let (_, mut rx) = bus.split();
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    let intent = MotorIntent {
        goal: MotorGoal::JointPose([1.0; JOINTS]),
        ..MotorIntent::hold(1, 0)
    };
    let mut body = BodyFrame::REST;
    for tick in 0..40 {
        body.sampled_at_us = tick * TICK_US;
        let output = kernel.tick(
            body.sampled_at_us,
            &body,
            (tick == 0).then_some(intent),
            &mut rx,
        );
        assert!(output.torque_nm[0] <= (tick + 1) as f32 * 0.5);
        assert!(output.torque_nm[0] <= 12.0);
    }
    body.sampled_at_us += TICK_US;
    body.joints[0].temperature_c = 89.0;
    let output = kernel.tick(body.sampled_at_us, &body, None, &mut rx);
    assert!(output.torque_nm[0] <= 0.600_001);
    body.sampled_at_us += TICK_US;
    body.joints[0].temperature_c = 90.0;
    let output = kernel.tick(body.sampled_at_us, &body, None, &mut rx);
    assert!(!output.drive_enabled[0]);
    assert_eq!(output.torque_nm[0], 0.0);
    assert!(output.drive_enabled[1]);
}

#[test]
fn withdrawal_hold_uses_elapsed_time_and_expires_at_the_boundary() {
    let mut bus = SensoryBus::<4>::new();
    let (mut tx, mut rx) = bus.split();
    tx.send(pain(1, 0)).unwrap();
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    kernel.tick(0, &BodyFrame::REST, Some(MotorIntent::hold(1, 0)), &mut rx);
    let body = BodyFrame {
        sampled_at_us: WITHDRAWAL_HOLD_US - 1,
        ..BodyFrame::REST
    };
    assert_eq!(
        kernel
            .tick(
                body.sampled_at_us,
                &body,
                Some(MotorIntent::hold(2, body.sampled_at_us)),
                &mut rx
            )
            .modes[0],
        ControlMode::Withdrawal
    );
    let body = BodyFrame {
        sampled_at_us: WITHDRAWAL_HOLD_US,
        ..body
    };
    assert_eq!(
        kernel.tick(body.sampled_at_us, &body, None, &mut rx).modes[0],
        ControlMode::Voluntary
    );
}

#[test]
fn malformed_configuration_is_rejected_before_boot() {
    let mut config = BodyConfig::fixture();
    config.joints[7].cutoff_c = config.joints[7].derate_start_c;
    assert!(matches!(
        SpinalKernel::new(config),
        Err(InvalidConfiguration(7))
    ));
}

#[test]
fn homeostasis_prioritizes_protection_and_gates_replay() {
    let mut bus = SensoryBus::<4>::new();
    let (mut tx, mut rx) = bus.split();
    let mut kernel = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    let mut body = BodyFrame {
        docked: true,
        task_demand: 0.0,
        ..BodyFrame::REST
    };
    let output = kernel.tick(0, &body, Some(MotorIntent::hold(1, 0)), &mut rx);
    assert_eq!(drive::evaluate(&body, &output).drive, Drive::Consolidate);
    body.battery_soc = 0.1;
    assert_eq!(drive::evaluate(&body, &output).drive, Drive::Recharge);
    body.joints[0].temperature_c = 75.0;
    assert_eq!(drive::evaluate(&body, &output).drive, Drive::Cool);
    body.sampled_at_us = TICK_US;
    tx.send(pain(1, TICK_US)).unwrap();
    let output = kernel.tick(TICK_US, &body, None, &mut rx);
    assert_eq!(drive::evaluate(&body, &output).drive, Drive::Protect);
}
