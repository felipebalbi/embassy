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

use config::ClocksConfig;
use critical_section::CriticalSection;

use crate::pac;

mod calc;
pub mod config;
mod gate;
mod operator;
pub mod periph_helpers;
mod program;
mod sleep;
mod types;

// Re-exports
pub use config::VddLevel;
pub(crate) use gate::default_register_value;
pub use gate::{Gate, assert_reset, disable, enable, enable_and_reset, is_reset_released, pulse_reset, release_reset};
pub use sleep::deep_sleep_if_possible;
pub use types::{Clock, ClockError, Clocks, PoweredClock, WakeGuard};

//
// Compile-time validated clock configuration
//

/// A [`ClocksConfig`] for which [`ClocksConfig::resolve()`] has SUCCEEDED.
///
/// Values produced by the [`validated_clocks!`](crate::validated_clocks) macro are
/// resolved during CONST evaluation, so an illegal configuration fails the BUILD at
/// the point of declaration.
///
/// The inner field is private, so safe external code cannot construct one by struct
/// literal, and this type deliberately implements neither [`Clone`], [`Copy`], nor
/// [`Default`].
///
/// # What this does and does not guarantee
///
/// The macro is NOT the only safe route to a value of this type. The constructor
/// [`__validated_clocks_config()`] is `#[doc(hidden)]` but necessarily `pub`, because
/// the macro expands inside the user's crate; safe code can therefore call it
/// directly, including in NON-const context, where a failed `resolve()` is a RUNTIME
/// panic rather than a build failure. What that constructor does still guarantee is
/// the value invariant: it re-runs `resolve()` itself, so it cannot return a token
/// wrapping a configuration that does not resolve.
///
/// Handing this token to [`init_validated()`](crate::init_validated) guarantees that
/// the same `ClocksConfig` VALUE that `resolve()` accepted is the one supplied to
/// runtime clock initialisation. It does NOT go further than that:
///
/// * [`init()`] resolves the configuration exactly once through `calc::resolve_program()`,
///   producing a `ResolvedClockProgram` that contains both the public [`Clocks`] state
///   and every configuration-derived register value. `operator::ClockOperator` receives
///   only a reference to that program: it holds no [`ClocksConfig`] and no mutable
///   [`Clocks`], so it applies the resolved decisions rather than making them. The
///   [`Clocks`] published to `CLOCKS` is the resolver's value, not a parallel
///   reconstruction.
/// * All pure validation happens during resolution, before [`init()`] touches a
///   register. This structure does not prove hardware realisation: the operator could
///   still apply a resolved value to the wrong register or omit a write.
/// * Pure resolution cannot represent genuinely live hardware state, including the
///   `CSR.SCS` read-back, LDO readiness, or any error/ready poll. Those checks
///   necessarily remain local to the clock operator.
pub struct ValidatedClocksConfig {
    config: ClocksConfig,
}

impl ValidatedClocksConfig {
    /// Unwrap the validated configuration, for handing to [`init()`].
    pub(crate) fn into_inner(self) -> ClocksConfig {
        self.config
    }
}

/// Map a [`ClockError`] onto a const-evaluation panic.
///
/// NOTE: only ONE `&'static str` can be surfaced in the compiler diagnostic,
/// because a const-evaluated `panic!` supports at most a single formatted
/// argument. For [`ClockError::BadConfig`] the `reason` is the more specific of
/// the two strings, so that is the one reported.
const fn panic_on_clock_error(result: Result<u32, ClockError>) {
    match result {
        Ok(_) => {}
        Err(ClockError::BadConfig { reason, .. }) => panic!("{}", reason),
        Err(ClockError::NotImplemented { clock }) => panic!("{}", clock),
        Err(ClockError::NeverInitialized) => panic!("system clocks were never initialized"),
        Err(ClockError::AlreadyInitialized) => panic!("system clocks were already initialized"),
        Err(ClockError::UnimplementedConfig) => panic!("peripheral clock config is unimplemented"),
    }
}

/// Resolve a [`ClocksConfig`], panicking if it is not legal.
///
/// NOTE: the panic is a COMPILE-TIME error only when this is evaluated in const
/// context - which is how the [`validated_clocks!`](crate::validated_clocks) macro
/// uses it, via a `const` item. Called from a non-const context it is an ordinary
/// RUNTIME panic.
///
/// NOTE: this is `pub` rather than `pub(crate)` because the
/// [`validated_clocks!`](crate::validated_clocks) macro expands inside the
/// *user's* crate, where a `pub(crate)` item of this crate would be
/// unreachable. `$crate` fixes path hygiene, but NOT privacy.
#[doc(hidden)]
pub const fn __resolve_clocks_or_panic(config: &ClocksConfig) -> Clocks {
    match config.resolve() {
        Ok(clocks) => clocks,
        Err(ClockError::BadConfig { reason, .. }) => panic!("{}", reason),
        Err(ClockError::NotImplemented { clock }) => panic!("{}", clock),
        Err(ClockError::NeverInitialized) => panic!("system clocks were never initialized"),
        Err(ClockError::AlreadyInitialized) => panic!("system clocks were already initialized"),
        Err(ClockError::UnimplementedConfig) => panic!("peripheral clock config is unimplemented"),
    }
}

/// Wrap a [`ClocksConfig`] into a [`ValidatedClocksConfig`], panicking if it is not
/// legal.
///
/// NOTE: the panic is a COMPILE-TIME error only when this is evaluated in const
/// context - which is how the [`validated_clocks!`](crate::validated_clocks) macro
/// uses it, via a `const` item. Called from a non-const context it is an ordinary
/// RUNTIME panic. Either way, this cannot return a token wrapping a configuration
/// that does not resolve.
///
/// NOTE: this is `pub` rather than `pub(crate)` because the
/// [`validated_clocks!`](crate::validated_clocks) macro expands inside the
/// *user's* crate, where a `pub(crate)` item of this crate would be
/// unreachable. `$crate` fixes path hygiene, but NOT privacy. A consequence is
/// that safe user code CAN call this directly, so the macro is not the only route
/// to a [`ValidatedClocksConfig`].
///
/// NOTE: the call to `resolve()` here is DELIBERATELY redundant with the one in
/// [`__resolve_clocks_or_panic()`]. It is what makes this constructor sound on
/// its own: even somebody who bypasses the macro and calls this doc-hidden
/// function directly still cannot wrap a configuration that does not resolve.
/// Do not "optimise" it away.
#[doc(hidden)]
pub const fn __validated_clocks_config(config: ClocksConfig) -> ValidatedClocksConfig {
    match config.resolve() {
        Ok(_) => {}
        Err(ClockError::BadConfig { reason, .. }) => panic!("{}", reason),
        Err(ClockError::NotImplemented { clock }) => panic!("{}", clock),
        Err(ClockError::NeverInitialized) => panic!("system clocks were never initialized"),
        Err(ClockError::AlreadyInitialized) => panic!("system clocks were already initialized"),
        Err(ClockError::UnimplementedConfig) => panic!("peripheral clock config is unimplemented"),
    }
    ValidatedClocksConfig { config }
}

/// Panic if a peripheral clock configuration failed validation.
///
/// NOTE: the panic is a COMPILE-TIME error only when this is evaluated in const
/// context - which is how the in-crate assertions and the
/// [`validated_clocks!`](crate::validated_clocks) macro use it, via `const _: ()`
/// items. Called from a non-const context it is an ordinary RUNTIME panic.
///
/// This is the single source of truth for the `ClockError` -> compiler
/// diagnostic mapping, shared by the in-crate assertions in
/// [`periph_helpers`] and by the [`validated_clocks!`](crate::validated_clocks)
/// macro.
///
/// NOTE: this is `pub` rather than `pub(crate)` because the
/// [`validated_clocks!`](crate::validated_clocks) macro expands inside the
/// *user's* crate, where a `pub(crate)` item of this crate would be
/// unreachable. `$crate` fixes path hygiene, but NOT privacy.
#[doc(hidden)]
pub const fn __assert_peripheral_clock_valid(result: Result<u32, ClockError>) {
    panic_on_clock_error(result)
}

//
// Statics/Consts
//

/// The state of system core clocks.
///
/// Initialized by [`init()`], and then unchanged for the remainder of the program.
pub(super) static CLOCKS: critical_section::Mutex<RefCell<Option<Clocks>>> =
    critical_section::Mutex::new(RefCell::new(None));
pub(super) static LIVE_HP_TOKENS: AtomicUsize = AtomicUsize::new(0);

//
// Free functions
//

/// Initialize the core system clocks with the given [`ClocksConfig`].
///
/// This function should be called EXACTLY once at start-up, usually via a
/// call to [`embassy_mcxa::init()`](crate::init()). Subsequent calls will
/// return an error.
pub fn init(settings: ClocksConfig) -> Result<(), ClockError> {
    critical_section::with(|cs| {
        if CLOCKS.borrow_ref(cs).is_some() {
            Err(ClockError::AlreadyInitialized)
        } else {
            Ok(())
        }
    })?;

    // Resolve the requested configuration in full *before* touching any hardware.
    // Everything the operator applies below comes from this result.
    let resolved = calc::resolve_program(&settings)?;

    let mut operator = operator::ClockOperator {
        resolved: &resolved,

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
        *clks = Some(resolved.clocks);
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
