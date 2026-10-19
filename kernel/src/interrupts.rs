use crate::out8;
use core::{
    arch::{asm, global_asm},
    ptr,
    sync::atomic::{AtomicU64, Ordering},
};

global_asm!(include_str!("interrupts.S"));

const PIT_HZ: u64 = 1_193_182;
const PIT_DIVISOR: u16 = 298;
static TICKS: AtomicU64 = AtomicU64::new(0);

#[derive(Clone, Copy)]
#[repr(C, packed)]
struct Gate {
    low: u16,
    selector: u16,
    ist: u8,
    attributes: u8,
    middle: u16,
    high: u32,
    reserved: u32,
}

impl Gate {
    const EMPTY: Self = Self {
        low: 0,
        selector: 0,
        ist: 0,
        attributes: 0,
        middle: 0,
        high: 0,
        reserved: 0,
    };
    fn new(address: u64) -> Self {
        Self {
            low: address as u16,
            selector: 8,
            ist: 0,
            attributes: 0x8e,
            middle: (address >> 16) as u16,
            high: (address >> 32) as u32,
            reserved: 0,
        }
    }
}

#[repr(C, packed)]
struct TablePointer {
    limit: u16,
    base: u64,
}

static mut IDT: [Gate; 256] = [Gate::EMPTY; 256];

unsafe extern "C" {
    fn timer_entry();
    fn exception_entry();
}

pub fn init() {
    // SAFETY: Single BSP initialization, before interrupts are enabled. No Rust
    // reference to IDT is formed; after lidt, the table is immutable for the CPU.
    unsafe {
        asm!("cli", options(nomem, nostack));
        let table = ptr::addr_of_mut!(IDT).cast::<Gate>();
        for index in 0..256 {
            table
                .add(index)
                .write(Gate::new(exception_entry as *const () as u64));
        }
        table
            .add(32)
            .write(Gate::new(timer_entry as *const () as u64));
        let pointer = TablePointer {
            limit: (core::mem::size_of::<[Gate; 256]>() - 1) as u16,
            base: table as u64,
        };
        asm!("lidt [{}]", in(reg) &pointer, options(readonly, nostack));
        // Legacy PIC remap: only IRQ0 is unmasked. I/O waits support the PC model.
        for (port, value) in [
            (0x20, 0x11),
            (0xa0, 0x11),
            (0x21, 0x20),
            (0xa1, 0x28),
            (0x21, 4),
            (0xa1, 2),
            (0x21, 1),
            (0xa1, 1),
            (0x21, 0xfe),
            (0xa1, 0xff),
        ] {
            out8(port, value);
            out8(0x80, 0);
        }
        // PIT channel 0, rate-generator mode; 1193182/298 ≈ 4003.97 Hz.
        out8(0x43, 0x34);
        out8(0x40, PIT_DIVISOR as u8);
        out8(0x40, (PIT_DIVISOR >> 8) as u8);
        asm!("sti", options(nomem, nostack));
    }
}

#[unsafe(no_mangle)]
extern "C" fn athena_timer_interrupt() {
    TICKS.fetch_add(1, Ordering::Relaxed);
    // SAFETY: This is the IRQ0 handler and the sole owner of PIC acknowledgments.
    unsafe {
        out8(0x20, 0x20);
    }
}

#[unsafe(no_mangle)]
extern "C" fn athena_exception_fatal() -> ! {
    crate::fatal_exception()
}

pub fn wait_after(previous: u64) -> u64 {
    loop {
        // SAFETY: Single-core Ring 0. STI's interrupt shadow makes STI;HLT one
        // wakeup sequence, avoiding the check-then-sleep lost-interrupt race.
        unsafe {
            asm!("cli", options(nomem, nostack));
            let current = TICKS.load(Ordering::Relaxed);
            if current > previous {
                asm!("sti", options(nomem, nostack));
                return current;
            }
            asm!("sti; hlt", options(nomem, nostack));
        }
    }
}

pub fn micros(ticks: u64) -> u64 {
    ticks * u64::from(PIT_DIVISOR) * 1_000_000 / PIT_HZ
}
