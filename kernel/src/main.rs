#![no_std]
#![no_main]

use athena_core::{
    bus::SensoryBus,
    drive::{self, Drive},
    *,
};
use core::{
    arch::{asm, global_asm},
    fmt::{self, Write},
    panic::PanicInfo,
};

global_asm!(include_str!("boot.S"));

struct Serial;

unsafe fn out8(port: u16, value: u8) {
    // SAFETY: Called only by this Ring 0 kernel for QEMU PC port I/O.
    unsafe {
        asm!("out dx, al", in("dx") port, in("al") value, options(nomem, nostack));
    }
}

unsafe fn in8(port: u16) -> u8 {
    let value: u8;
    // SAFETY: Called only by this Ring 0 kernel for QEMU PC port I/O.
    unsafe {
        asm!("in al, dx", in("dx") port, out("al") value, options(nomem, nostack));
    }
    value
}

impl Serial {
    fn init() {
        // SAFETY: COM1 is the emulator's legacy serial device, exclusively owned here.
        unsafe {
            out8(0x3f9, 0);
            out8(0x3fb, 0x80);
            out8(0x3f8, 1);
            out8(0x3f9, 0);
            out8(0x3fb, 3);
            out8(0x3fa, 0xc7);
            out8(0x3fc, 0x0b);
        }
    }
}

impl Write for Serial {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for byte in text.bytes() {
            let mut ready = false;
            for _ in 0..100_000 {
                // SAFETY: Reading the COM1 line status register in the emulator.
                if unsafe { in8(0x3fd) } & 0x20 != 0 {
                    ready = true;
                    break;
                }
            }
            if !ready {
                return Err(fmt::Error);
            }
            // SAFETY: COM1 transmit register, exclusively owned by this kernel.
            unsafe {
                out8(0x3f8, byte);
            }
        }
        Ok(())
    }
}

macro_rules! log {
    ($($arg:tt)*) => {{ let _ = writeln!(Serial, $($arg)*); }};
}

fn exit(success: bool) -> ! {
    let value: u32 = if success { 0x10 } else { 0x11 };
    // SAFETY: The test harness installs isa-debug-exit at this port. This is not
    // a physical actuator or a general host shutdown mechanism.
    unsafe {
        asm!("out dx, eax", in("dx") 0xf4_u16, in("eax") value, options(nomem, nostack));
    }
    loop {
        // SAFETY: Interrupts remain disabled in this single-core boot exercise.
        unsafe {
            asm!("cli; hlt", options(nomem, nostack));
        }
    }
}

#[unsafe(no_mangle)]
extern "C" fn kernel_main() -> ! {
    Serial::init();
    log!("ATHENA / x86-64 spinal kernel / milestone 0");
    log!("boot: long mode, identity paging, static stack, no allocator");
    log!("clock: logical 250us ticks; real timer/deadline validation pending");
    let mut bus = SensoryBus::<64>::new();
    let (mut sensors, mut ingress) = bus.split();
    let mut spinal = SpinalKernel::new(BodyConfig::fixture()).unwrap();
    let mut body = BodyFrame::REST;
    let mut withdrawals = 0;
    let mut watchdog_seen = false;
    let mut recharge_seen = false;
    let mut sleep_seen = false;
    for tick in 0..1_600_u64 {
        let now = tick * TICK_US;
        body.sampled_at_us = now;
        // No cortex refresh during 220..300ms; the independent spinal loop runs on.
        let intent = if tick % 40 == 0 && !(880..1_200).contains(&tick) {
            Some(MotorIntent::hold(tick + 1, now))
        } else {
            None
        };
        if tick == 4 {
            sensors
                .send(Spike {
                    timestamp_us: now,
                    source_id: 1,
                    intensity: u16::MAX,
                    limb_mask: Limb::LeftArm.mask() | Limb::RightArm.mask(),
                    modality: Modality::Nociception,
                })
                .unwrap();
        }
        if tick >= 1_240 {
            body.battery_soc = 0.12;
        }
        if tick >= 1_400 {
            body.battery_soc = 0.8;
            body.docked = true;
            body.task_demand = 0.0;
        }
        let output = spinal.tick(now, &body, intent, &mut ingress);
        assert!(!output.faults.any());
        assert!(
            output
                .torque_nm
                .iter()
                .all(|q| q.is_finite() && q.abs() <= 12.0)
        );
        if tick == 4 {
            assert_eq!(output.active_reflex_mask, 3);
            assert_eq!(output.modes[0], ControlMode::Withdrawal);
            assert_eq!(output.modes[2], ControlMode::Voluntary);
            assert!(output.torque_nm[0] < 0.0);
            log!("PASS nociception: both arms preempt cortex in the ingress tick");
            withdrawals += 1;
        }
        if tick == 1_100 {
            assert!(output.watchdog_expired);
            assert_eq!(output.modes[0], ControlMode::AutonomicDamping);
            log!("PASS watchdog: expired intent -> autonomic damping");
            watchdog_seen = true;
        }
        let decision = drive::evaluate(&body, &output);
        if tick == 1_300 {
            assert_eq!(decision.drive, Drive::Recharge);
            recharge_seen = true;
            log!("PASS homeostasis: low energy -> recharge");
        }
        if tick == 1_500 {
            assert_eq!(decision.drive, Drive::Consolidate);
            sleep_seen = true;
            log!("PASS sleep gate: docked, cool, idle, healthy");
        }
    }
    assert_eq!(withdrawals, 1);
    assert!(watchdog_seen && recharge_seen && sleep_seen);
    // Explicitly prove invalid sensory data cannot put NaN on the actuator boundary.
    body.sampled_at_us = 400_000;
    body.joints[0].temperature_c = f32::NAN;
    let output = spinal.tick(400_000, &body, None, &mut ingress);
    assert!(output.faults.any());
    assert!(output.drive_enabled.iter().all(|enabled| !enabled));
    assert!(output.torque_nm.iter().all(|q| *q == 0.0));
    log!("PASS invalid sensor: latched fault, every drive disabled");
    log!("ATHENA_BOOT_OK");
    exit(true)
}

#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    log!("ATHENA_PANIC: {}", info);
    exit(false)
}
