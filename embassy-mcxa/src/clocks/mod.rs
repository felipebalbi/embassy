//! # Clock Module
//!
//! For the MCX-A, we separate clock and peripheral control into two main stages:
//!
//! 1. At startup, e.g. when `embassy_mcxa::init()` is called, we configure the
//!    core system clocks, including external and internal oscillators. This
//!    configuration is then largely static for the duration of the program.
//! 2. When HAL drivers are created, e.g. `Lpuart::new()` is called, the driver
//!    is responsible for two main things:
//!     * Ensuring that any required "upstream" core system clocks necessary for
//!       clocking the peripheral is active and configured to a reasonable value
//!     * Enabling the clock gates for that peripheral, and resetting the peripheral
//!
//! From a user perspective, only step 1 is visible. Step 2 is automatically handled
//! by HAL drivers, using interfaces defined in this module.
//!
//! It is also possible to *view* the state of the clock configuration after [`init()`]
//! has been called, using the [`with_clocks()`] function, which provides a view of the
//! [`Clocks`] structure.
//!
//! ## For HAL driver implementors
//!
//! The majority of peripherals in the MCXA chip are fed from either a "hard-coded" or
//! configurable clock source, e.g. selecting the FROM12M or `clk_1m` as a source. This
//! selection, as well as often any pre-scaler division from that source clock, is made
//! through MRCC registers.
//!
//! Any peripheral that is controlled through the MRCC register can automatically implement
//! the necessary APIs using the `impl_cc_gate!` macro in this module. You will also need
//! to define the configuration surface and steps necessary to fully configure that peripheral
//! from a clocks perspective by:
//!
//! 1. Defining a configuration type in the [`periph_helpers`] module that contains any selects
//!    or divisions available to the HAL driver
//! 2. Implementing the [`periph_helpers::SPConfHelper`] trait, which should check that the
//!    necessary input clocks are reasonable

use core::cell::RefCell;
use core::sync::atomic::{AtomicUsize, Ordering};

use config::{ClocksConfig, CoreSleep, MainClockSource};
use critical_section::CriticalSection;
use grounded::uninit::GroundedCell;

use crate::pac;

pub mod config;
mod gate;
mod operator;
pub mod periph_helpers;
mod power_down_record;
mod sleep;
mod types;

// Re-exports
pub use config::VddLevel;
pub(crate) use gate::default_register_value;
pub use gate::{Gate, assert_reset, disable, enable, enable_and_reset, is_reset_released, pulse_reset, release_reset};
#[cfg(feature = "mcxa2xx")]
pub(crate) use power_down_record::CONFIGURED_FIRC;
pub(crate) use power_down_record::{CONFIGURED_SIRC, CONFIGURED_SOSC, CONFIGURED_SPLL, PowerDownRecoveryContext};
pub use power_down_record::{PowerDownFailStopReport, PowerDownRecoveryCause, PowerDownRecoverySnapshot};
pub use sleep::{deep_sleep_if_possible, power_down_if_possible};
pub use types::{Clock, ClockError, Clocks, PoweredClock, WakeGuard};

//
// Statics/Consts
//

/// The state of system core clocks.
///
/// Initialized by [`init()`], and then unchanged for the remainder of the program.
pub(super) static CLOCKS: critical_section::Mutex<RefCell<Option<Clocks>>> =
    critical_section::Mutex::new(RefCell::new(None));
pub(super) static LIVE_HP_TOKENS: AtomicUsize = AtomicUsize::new(0);

// SYSRESETREQ is also used by probes, so SRS alone cannot identify fail-stops.
// Keep the record in linker-placed .uninit so normal startup does not erase it
// on warm resets; fixed addresses would conflict with linker RAM placement.
const POLL_STEPS: u32 = 1000;

#[unsafe(link_section = ".uninit.EMBASSY_MCXA_PD_FAILSTOP")]
static PD_FAIL_STOP_MARKER: GroundedCell<[u32; power_down_record::RECORD_WORDS]> = GroundedCell::uninit();

fn pd_fail_stop_words() -> *mut u32 {
    PD_FAIL_STOP_MARKER.get().cast::<u32>()
}

// The critical section serializes all retained-word access. SRAM words may
// be arbitrary; volatile u32 reads do not create references to uninitialized data.
fn pd_fail_stop_read(_cs: &CriticalSection) -> [u32; power_down_record::RECORD_WORDS] {
    let p = pd_fail_stop_words();
    let mut words = [0u32; power_down_record::RECORD_WORDS];
    let mut i = 0;
    while i < power_down_record::RECORD_WORDS {
        words[i] = unsafe { core::ptr::read_volatile(p.add(i)) };
        i += 1;
    }
    words
}

fn pd_fail_stop_marker_is_exact(cs: &CriticalSection) -> bool {
    power_down_record::header_is_recognised(&pd_fail_stop_read(cs))
}

// Decode and clear under one critical section; unrecognised residue is untouched.
fn pd_fail_stop_consume(cs: &CriticalSection) -> Option<PowerDownFailStopReport> {
    let words = pd_fail_stop_read(cs);
    let canary_valid = pd_probe_ready_observe(cs);
    let report = power_down_record::decode(&words, canary_valid);

    if report.is_some() {
        let p = pd_fail_stop_words();
        unsafe {
            // Invalidate headers before clearing payload, so an interrupted clear
            // cannot leave a recognised header with a partially erased payload.
            core::ptr::write_volatile(p, 0);
            core::ptr::write_volatile(p.add(1), 0);
            cortex_m::asm::dsb();

            let mut i = 2;
            while i < power_down_record::RECORD_WORDS {
                core::ptr::write_volatile(p.add(i), 0);
                i += 1;
            }
            cortex_m::asm::dsb();
        }
    }

    report
}

/// Consume a recognised Power Down fail-stop record, returning whether one existed.
///
/// This and [`take_power_down_fail_stop_report`] share at-most-once consumption,
/// including across thread and interrupt callers. Use one reader, not both.
/// Unrecognised residue is not cleared. A recognised header does not identify
/// the reset issuer or guarantee a valid payload; use the report API for detail.
/// A reset after clearing can lose delivery.
pub fn take_power_down_fail_stop() -> bool {
    critical_section::with(|cs| pd_fail_stop_consume(&cs).is_some())
}

/// Consume a Power Down fail-stop report and clear its recognised record.
///
/// Returns `None` for an unrecognised header without clearing residue. Legacy,
/// damaged or unknown payloads, or a missing pre-arm canary, yield
/// [`PowerDownRecoveryCause::Unavailable`] with no snapshot.
/// Shares at-most-once consumption with [`take_power_down_fail_stop`].
/// Integrity checks do not establish freshness or producer identity.
pub fn take_power_down_fail_stop_report() -> Option<PowerDownFailStopReport> {
    critical_section::with(|cs| pd_fail_stop_consume(&cs))
}

const PROBE_READY_MAGIC: u32 = 0x5A5A_C3C3;
const PROBE_READY_COMPLEMENT: u32 = 0xA5A5_3C3C;

// The canary shares the record's retained-section contract, not its address.
#[unsafe(link_section = ".uninit.EMBASSY_MCXA_PD_PROBE_READY")]
static PD_PROBE_READY_CANARY: GroundedCell<[u32; 2]> = GroundedCell::uninit();

fn pd_probe_ready_words() -> *mut u32 {
    PD_PROBE_READY_CANARY.get().cast::<u32>()
}

fn pd_probe_ready_is_exact(_cs: &CriticalSection) -> bool {
    let p = pd_probe_ready_words();
    unsafe {
        core::ptr::read_volatile(p) == PROBE_READY_MAGIC && core::ptr::read_volatile(p.add(1)) == PROBE_READY_COMPLEMENT
    }
}

fn pd_probe_ready_arm(cs: &CriticalSection) {
    if pd_probe_ready_is_exact(cs) {
        return;
    }
    let p = pd_probe_ready_words();
    unsafe {
        core::ptr::write_volatile(p, PROBE_READY_MAGIC);
        core::ptr::write_volatile(p.add(1), PROBE_READY_COMPLEMENT);
        cortex_m::asm::dsb();
    }
}

// Latch the pre-arm observation once; this boot's writes cannot upgrade it.
fn pd_probe_ready_observe(cs: &CriticalSection) -> bool {
    use core::sync::atomic::Ordering::Relaxed;
    if !PD_PROBE_READY_LATCHED.load(Relaxed) {
        PD_PROBE_READY_PRESENT.store(pd_probe_ready_is_exact(cs), Relaxed);
        PD_PROBE_READY_LATCHED.store(true, Relaxed);
        pd_probe_ready_arm(cs);
    }
    PD_PROBE_READY_PRESENT.load(Relaxed)
}

/// Return this boot's latched pre-arm retained-canary observation.
///
/// The first call observes the retained words, then arms them for the next boot.
/// Absence can mean first boot, startup zeroing, relinking or RAM destruction;
/// this is not detection of `zero-init-ram` or proof that `.uninit` is retained.
/// Warm admission requires the same image/address and retaining startup/reset.
pub fn power_down_retention_canary_is_present() -> bool {
    critical_section::with(|cs| pd_probe_ready_observe(&cs))
}

static PD_PROBE_READY_LATCHED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
static PD_PROBE_READY_PRESENT: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

const SRS_POR: u32 = 1 << 1;
const SRS_VD: u32 = 1 << 2;

const fn srs_is_exact_cold_start(srs: u32) -> bool {
    srs == SRS_POR || srs == SRS_POR | SRS_VD
}

// Caller holds the critical section through capture, publication and reset.
// Failures after ISO_CLR have already de-isolated pads; publication cannot
// restore isolation. Capture is sequential, not a coherent hardware snapshot.
fn power_down_fail_stop_reset(
    _cs: &CriticalSection,
    cause: PowerDownRecoveryCause,
    context: PowerDownRecoveryContext,
) -> ! {
    let cmc = pac::CMC;
    let scg = pac::SCG0;
    let spc = pac::SPC0;

    let registers = [
        cmc.srs().read().0,
        cmc.ckctrl().read().0,
        scg.ldocsr().read().0,
        scg.spllcsr().read().0,
        scg.csr().read().0,
        scg.rccr().read().0,
        scg.sirccsr().read().0,
        spc.sc().read().0,
        spc.pd_status0().read().0,
    ];

    let record = power_down_record::encode(cause, &registers, &context);
    let p = pd_fail_stop_words();
    let mut attempts = 0u32;

    while attempts < POLL_STEPS {
        let verified = unsafe {
            // Invalidate headers, write payload, then commit headers last.
            // DSB orders stores; full readback verifies them before reset.
            core::ptr::write_volatile(p, 0);
            core::ptr::write_volatile(p.add(1), 0);
            cortex_m::asm::dsb();

            let mut i = 2;
            while i < power_down_record::RECORD_WORDS {
                core::ptr::write_volatile(p.add(i), record[i]);
                i += 1;
            }
            cortex_m::asm::dsb();

            core::ptr::write_volatile(p, record[0]);
            core::ptr::write_volatile(p.add(1), record[1]);
            cortex_m::asm::dsb();

            let mut equal = true;
            let mut j = 0;
            while j < power_down_record::RECORD_WORDS {
                equal &= core::ptr::read_volatile(p.add(j)) == record[j];
                j += 1;
            }
            equal
        };

        if verified {
            cortex_m::asm::dsb();
            cortex_m::peripheral::SCB::sys_reset();
        }

        attempts += 1;
    }

    // An unverified record cannot safely authorize a reset. Halt masked;
    // external or watchdog resets remain possible.
    loop {
        cortex_m::asm::nop();
    }
}

//
// Free functions
//

/// Initialize the core system clocks with the given [`ClocksConfig`].
///
/// This function should be called EXACTLY once at start-up, usually via a
/// call to [`embassy_mcxa::init()`](crate::init()). Subsequent calls will
/// return an error.
///
/// Power Down rejects an unconsumed fail-stop record, or a missing pre-arm
/// canary on a noncold boot, without clearing the record. Non-PowerDown modes
/// may consume recognised evidence. Power Down also rejects SPLL as the main
/// clock: entry drops its LDO before WFE and no clock handover is implemented.
/// A configured but unselected SPLL remains supported.
///
/// These checks precede this function's clock programming. The top-level
/// [`crate::init`] has already taken peripherals and set NVIC priorities, and
/// unwraps errors before application LEDs or GPIO diagnostics exist.
pub fn init(settings: ClocksConfig) -> Result<(), ClockError> {
    // Read SRS without clearing the application's sticky SSRS history.
    let boot_srs = pac::CMC.srs().read().0;

    // Observe/arm before admission decisions or early returns.
    let retention_canary_present = power_down_retention_canary_is_present();
    let marker_present = critical_section::with(|cs| pd_fail_stop_marker_is_exact(&cs));

    match settings.vdd_power.core_sleep {
        CoreSleep::PowerDown => {
            if marker_present {
                return Err(ClockError::BadConfig {
                    clock: "power_down",
                    reason: "an unconsumed Power Down masked-recovery fail-stop record is present",
                });
            }

            if !retention_canary_present && !srs_is_exact_cold_start(boot_srs) {
                return Err(ClockError::BadConfig {
                    clock: "power_down",
                    reason: "the retained-RAM pre-arm canary is missing or corrupt on a non-cold boot",
                });
            }

            // Entry disables the PLL supply before WFE; reject only selection,
            // not a configured PLL with the core running from another source.
            if matches!(settings.main_clock.source, MainClockSource::SPll1) {
                return Err(ClockError::BadConfig {
                    clock: "power_down",
                    reason: "CoreSleep::PowerDown does not support SPLL as the main clock",
                });
            }
        }
        CoreSleep::WfeUngated | CoreSleep::WfeGated | CoreSleep::DeepSleep => {
            if marker_present {
                let _ = take_power_down_fail_stop();
            }
        }
    }

    critical_section::with(|cs| {
        if CLOCKS.borrow_ref(cs).is_some() {
            Err(ClockError::AlreadyInitialized)
        } else {
            Ok(())
        }
    })?;

    let mut clocks = Clocks::default();
    // Validate the board budget before clock/power programming.
    clocks.power_down_clock_recovery_budget_us = settings.validated_power_down_clock_recovery_budget_us()?;

    let mut operator = operator::ClockOperator {
        clocks: &mut clocks,
        config: &settings,
        sirc_forced: false,

        _mrcc0: pac::MRCC0,
        scg0: pac::SCG0,
        syscon: pac::SYSCON,
        vbat0: pac::VBAT0,
        spc0: pac::SPC0,
        fmu0: pac::FMU0,
        cmc: pac::CMC,
    };

    operator.unlock_mrcc();

    // Before applying any requested clocks, apply the requested VDD_CORE
    // voltage level
    operator.configure_voltages()?;

    // Enable SIRC clocks FIRST, in case we need to use SIRC as main_clk for
    // a short while.
    operator.configure_sirc_clocks_early()?;
    operator.configure_firc_clocks()?;
    operator.configure_fro16k_clocks()?;

    // NOTE: OSC32K must be configured AFTER FRO16K.
    #[cfg(all(feature = "mcxa5xx", feature = "unstable-osc32k", not(feature = "rosc-32k-as-gpio")))]
    operator.configure_osc32k_clocks()?;

    #[cfg(not(feature = "sosc-as-gpio"))]
    operator.configure_sosc()?;
    operator.configure_spll()?;

    // Finally, setup main clock
    operator.configure_main_clk()?;

    // If we were keeping SIRC enabled, now we can release it.
    operator.configure_sirc_clocks_late();

    critical_section::with(|cs| {
        let mut clks = CLOCKS.borrow_ref_mut(cs);
        assert!(clks.is_none(), "Clock setup race!");
        *clks = Some(clocks);
    });

    Ok(())
}

/// Obtain the full clocks structure, calling the given closure in a critical section.
///
/// The given closure will be called with read-only access to the state of the system
/// clocks. This can be used to query and return the state of a given clock.
///
/// As the caller's closure will be called in a critical section, care must be taken
/// not to block or cause any other undue delays while accessing.
///
/// Calls to this function will not succeed until after a successful call to `init()`,
/// and will always return None.
pub fn with_clocks<R: 'static, F: FnOnce(&Clocks) -> R>(f: F) -> Option<R> {
    critical_section::with(|cs| {
        let c = CLOCKS.borrow_ref(cs);
        let c = c.as_ref()?;
        Some(f(c))
    })
}

/// Are there active `WakeGuard`s?
///
/// Requires a critical section to ensure this doesn't race between getting the guard
/// count and performing some action like setting up deep sleep
#[inline(always)]
pub fn active_wake_guards(_cs: &CriticalSection) -> bool {
    // Relaxed is okay: we are in a critical section
    LIVE_HP_TOKENS.load(Ordering::Relaxed) != 0
}
