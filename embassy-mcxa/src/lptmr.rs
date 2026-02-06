//! LPTMR Time Driver.
use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering, compiler_fence};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use nxp_pac::lptmr::vals::Tie;

use crate::clkout::Div4;
use crate::clocks::periph_helpers::{LpTmrClockSel, LpTmrConfig};
use crate::clocks::{PoweredClock, enable_and_reset};
use crate::interrupt;
use crate::interrupt::InterruptExt;
use crate::pac::LPTMR0;
use crate::peripherals::LPTMR0;

/// Calculate the timestamp from the period count and the tick count.
///
/// The Low Power Timer is a 32-bit free running timer running from
/// one of several sources:
///
/// - fro_lf_div
/// - fro_hf_div
/// - clk_1m
/// - clk_in
/// - pll1_clk_div
///
/// The clock source fed into LPTMR must be at most 22.5MHz and at
/// least 16kHz (when bypassing the LPTMR clock divider and using
/// CLK_16K directly).
///
/// We define a period to be 2^31 ticks, such that each overflow is 2
/// periods.
///
/// To get `now()`, `period` is read first, then `counter` is read. If
/// the counter value matches the expected range for the `period`
/// parity, we're done. If it doesn't, this means that a new period
/// start has raced us between reading `period` and `counter`, so we
/// assume the `counter` value corresponds to the next period.
///
/// `period` is a 32-bit integer, it overlows, in the worst case, on
/// 2^32 * 2^31 / 22_500_000 seconds of uptime, which is about 1299
/// years. In the best case, that number jumps to over 18 million
/// years at the cost of time resolution.
fn calc_now(period: u32, counter: u32) -> u64 {
    ((period as u64) << 31) + ((counter ^ ((period & 1) << 31)) as u64)
}

struct AlarmState {
    timestamp: Cell<u64>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: LpTmr = LpTmr {
    period: AtomicU32::new(0),
    alarms:  Mutex::const_new(CriticalSectionRawMutex::new(), AlarmState::new()),
    queue: Mutex::new(RefCell::new(Queue::new())),
});

struct LpTmr {
    /// Number of 2^31 periods elapsed since boot.
    period: AtomicU32,

    /// Timestamp at which to fire alarm. u64::MAX if no alarm is
    /// scheduled.
    alarms: Mutex<CriticalSectionRawMutex, AlarmState>,
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

impl LpTmr {
    fn init(&'static self, irq_prio: crate::interrupt::Priority) {
        // init alarms
        critical_section::with(|cs| {
            let alarm = DRIVER.alarms.borrow(cs);
            alarm.timestamp.set(u64::MAX);
        });

        let parts = unsafe {
            enable_and_reset::<LPTMR0>(&LpTmrConfig {
                power: PoweredClock::AlwaysEnabled,
                source: LpTmrClockSel::Clk1M,
                div: const { Div4::no_div() },
            })
            .expect("Enabling LpTmr clock should not fail")
        };

        // Currently does nothing as Clk1M is always enabled anyway, this is here
        // to make sure that doesn't change in a refactoring.
        core::mem::forget(parts.wake_guard);

        interrupt::LPTMR0.disable();

        // Make sure interrupt is masked
        LPTMR0.csr().modify(|w| w.set_tie(Tie::TIE1));

        // Default to the end of time
        LPTMR0.cmr().write(|w| w.0 = u32::MAX);

        interrupt::LPTMR0.unpend();
        interrupt::LPTMR0.set_priority(irq_prio);
        unsafe { interrupt::LPTMR0.enable() };
    }

    fn next_period(&self) {
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);
        let t = (period as u64) << 31;

        critical_section::with(move |cs| {
            let alarm = self.alarms.borrow(cs);
            let at = alarm.timestamp.get();

            if at < t + 0xc000_0000 {
                LPTMR0.csr().modify(|w| w.set_tie(Tie::TIE1));
            }
        });
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let alarm = self.alarms.borrow(cs);
        alarm.timestamp.set(timestamp);

        let t = self.now();
        if timestamp <= t {
            // If timestamp has passed, alarm will not fire.
            // Disarm it and return `false`.
            LPTMR0.csr().modify(|w| w.set_tie(Tie::TIE0));
            self.alarms.borrow(cs).timestamp.set(u64::MAX);
            return false;
        }

        LPTMR0.cmr().write(|w| w.0 = timestamp as u32);

        let diff = timestamp - t;
        LPTMR0
            .csr()
            .modify(|w| w.set_tie(if diff < 0xc000_0000 { Tie::TIE1 } else { Tie::TIE0 }));

        let t = self.now();
        if timestamp <= t {
            // If timestamp has passed, alarm will not fire.
            // Disarm it and return `false`.
            LPTMR0.csr().modify(|w| w.set_tie(Tie::TIE0));
            self.alarms.borrow(cs).timestamp.set(u64::MAX);
            return false;
        }

        true
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        crate::perf_counters::incr_interrupt_ostimer_alarm();
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }
}

static INIT: AtomicBool = AtomicBool::new(false);

impl Driver for LpTmr {
    fn now(&self) -> u64 {
        // Don't try to read the timer before the LpTmr is actually enabled.
        // This leads to faults on the MCX-A.
        if !INIT.load(Ordering::Relaxed) {
            return 0;
        }

        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        // Before reading the counter, we must first write any value
        // to it.
        LPTMR0.cnr().write(|w| w.0 = 0);
        let counter = LPTMR0.cnr().read().counter();
        calc_now(period, counter)
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn LPTMR0() {
    crate::perf_counters::incr_interrupt_lptmr();
    DRIVER.on_interrupt()
}

pub(crate) fn init(irq_prio: crate::interrupt::Priority) {
    DRIVER.init(irq_prio);
    INIT.store(true, Ordering::Relaxed);
}
