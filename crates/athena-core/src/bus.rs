//! In-process SPSC channels. These are not a stable cross-address-space ABI.
use crate::Spike;
use core::{
    cell::UnsafeCell,
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

#[repr(align(64))]
struct CacheLine<T>(T);

pub struct SpscRing<T: Copy, const N: usize> {
    head: CacheLine<AtomicUsize>,
    tail: CacheLine<AtomicUsize>,
    slots: [UnsafeCell<MaybeUninit<T>>; N],
}

// SAFETY: Only split endpoints access slots, and they cannot be cloned. Producer
// release / consumer acquire transfers initialized slots; tail release / head-side
// acquire prevents reuse until the consumer finishes its copy. T is Copy + Send.
unsafe impl<T: Copy + Send, const N: usize> Sync for SpscRing<T, N> {}

impl<T: Copy, const N: usize> SpscRing<T, N> {
    pub const fn new() -> Self {
        assert!(N > 0 && N.is_power_of_two());
        Self {
            head: CacheLine(AtomicUsize::new(0)),
            tail: CacheLine(AtomicUsize::new(0)),
            slots: [const { UnsafeCell::new(MaybeUninit::uninit()) }; N],
        }
    }

    pub fn split(&mut self) -> (Producer<'_, T, N>, Consumer<'_, T, N>) {
        (Producer { ring: self }, Consumer { ring: self })
    }
}

impl<T: Copy, const N: usize> Default for SpscRing<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Producer<'a, T: Copy, const N: usize> {
    ring: &'a SpscRing<T, N>,
}
pub struct Consumer<'a, T: Copy, const N: usize> {
    ring: &'a SpscRing<T, N>,
}

impl<T: Copy, const N: usize> Producer<'_, T, N> {
    pub fn push(&mut self, value: T) -> Result<(), T> {
        let head = self.ring.head.0.load(Ordering::Relaxed);
        let tail = self.ring.tail.0.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= N {
            return Err(value);
        }
        // SAFETY: This endpoint exclusively owns the unpublished head slot.
        unsafe {
            (*self.ring.slots[head & (N - 1)].get()).write(value);
        }
        self.ring
            .head
            .0
            .store(head.wrapping_add(1), Ordering::Release);
        Ok(())
    }
}

impl<T: Copy, const N: usize> Consumer<'_, T, N> {
    pub fn pop(&mut self) -> Option<T> {
        let tail = self.ring.tail.0.load(Ordering::Relaxed);
        if tail == self.ring.head.0.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: The acquired head publishes this initialized slot. The producer
        // cannot reuse it until the release store after the copy finishes.
        let value = unsafe { (*self.ring.slots[tail & (N - 1)].get()).assume_init_read() };
        self.ring
            .tail
            .0
            .store(tail.wrapping_add(1), Ordering::Release);
        Some(value)
    }
}

const DIRTY: usize = 4;

struct SnapshotShared<T: Copy> {
    middle: CacheLine<AtomicUsize>,
    slots: [UnsafeCell<T>; 3],
}

// SAFETY: Each endpoint exclusively owns one slot. A single atomic exchange
// transfers ownership of the third slot with acquire/release synchronization.
// Readers return copies, so no reference can outlive the reader's slot ownership.
unsafe impl<T: Copy + Send> Sync for SnapshotShared<T> {}

/// Latest-value triple buffer: one bounded publish and one bounded read, without
/// a racing non-atomic seqlock payload. Reader metadata must be writable.
pub struct Snapshot<T: Copy> {
    shared: SnapshotShared<T>,
    front: usize,
    back: usize,
}

impl<T: Copy> Snapshot<T> {
    pub const fn new(initial: T) -> Self {
        Self {
            shared: SnapshotShared {
                middle: CacheLine(AtomicUsize::new(1)),
                slots: [
                    UnsafeCell::new(initial),
                    UnsafeCell::new(initial),
                    UnsafeCell::new(initial),
                ],
            },
            front: 0,
            back: 2,
        }
    }

    pub fn split(&mut self) -> (SnapshotWriter<'_, T>, SnapshotReader<'_, T>) {
        (
            SnapshotWriter {
                shared: &self.shared,
                back: &mut self.back,
            },
            SnapshotReader {
                shared: &self.shared,
                front: &mut self.front,
            },
        )
    }
}

pub struct SnapshotWriter<'a, T: Copy> {
    shared: &'a SnapshotShared<T>,
    back: &'a mut usize,
}
pub struct SnapshotReader<'a, T: Copy> {
    shared: &'a SnapshotShared<T>,
    front: &'a mut usize,
}

impl<T: Copy> SnapshotWriter<'_, T> {
    pub fn publish(&mut self, value: T) {
        // SAFETY: The back slot belongs only to this non-clonable writer.
        unsafe {
            *self.shared.slots[*self.back].get() = value;
        }
        *self.back = self
            .shared
            .middle
            .0
            .swap(*self.back | DIRTY, Ordering::AcqRel)
            & 3;
    }
}

impl<T: Copy> SnapshotReader<'_, T> {
    pub fn latest(&mut self) -> T {
        if self.shared.middle.0.load(Ordering::Acquire) & DIRTY != 0 {
            *self.front = self.shared.middle.0.swap(*self.front, Ordering::AcqRel) & 3;
        }
        // SAFETY: The front slot belongs only to this reader until its next swap.
        unsafe { *self.shared.slots[*self.front].get() }
    }
}

pub struct SensoryBus<const N: usize> {
    critical: SpscRing<Spike, N>,
    ordinary: SpscRing<Spike, N>,
    overflow: AtomicBool,
}

impl<const N: usize> SensoryBus<N> {
    pub const fn new() -> Self {
        Self {
            critical: SpscRing::new(),
            ordinary: SpscRing::new(),
            overflow: AtomicBool::new(false),
        }
    }

    pub fn split(&mut self) -> (SensoryWriter<'_, N>, SensoryReader<'_, N>) {
        let (critical_tx, critical_rx) = self.critical.split();
        let (ordinary_tx, ordinary_rx) = self.ordinary.split();
        (
            SensoryWriter {
                critical: critical_tx,
                ordinary: ordinary_tx,
                overflow: &self.overflow,
            },
            SensoryReader {
                critical: critical_rx,
                ordinary: ordinary_rx,
                overflow: &self.overflow,
            },
        )
    }
}

impl<const N: usize> Default for SensoryBus<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct SensoryWriter<'a, const N: usize> {
    critical: Producer<'a, Spike, N>,
    ordinary: Producer<'a, Spike, N>,
    overflow: &'a AtomicBool,
}
pub struct SensoryReader<'a, const N: usize> {
    critical: Consumer<'a, Spike, N>,
    ordinary: Consumer<'a, Spike, N>,
    overflow: &'a AtomicBool,
}

impl<const N: usize> SensoryWriter<'_, N> {
    pub fn send(&mut self, spike: Spike) -> Result<(), Spike> {
        let result = if spike.is_critical() {
            self.critical.push(spike)
        } else {
            self.ordinary.push(spike)
        };
        if result.is_err() {
            self.overflow.store(true, Ordering::Release);
        }
        result
    }
}

impl<const N: usize> SensoryReader<'_, N> {
    pub(crate) fn critical(&mut self) -> Option<Spike> {
        self.critical.pop()
    }
    pub(crate) fn ordinary(&mut self) -> Option<Spike> {
        self.ordinary.pop()
    }
    pub(crate) fn take_overflow(&mut self) -> bool {
        self.overflow.swap(false, Ordering::AcqRel)
    }
}
