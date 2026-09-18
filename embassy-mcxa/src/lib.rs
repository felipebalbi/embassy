#![no_std]
#![allow(async_fn_in_trait)]
#![doc = include_str!("../README.md")]
// Clippy Exceptions
//
// Allow functions with too many args - we have a lot of HAL constructors like this for now
#![allow(clippy::too_many_arguments)]

/// Module for MCXA2xx-specific HAL drivers
///
/// NOTE: *for now*, some items are here because we haven't validated them on the MCXA5xx yet.
/// This note will be removed when the two reach parity.
#[cfg(feature = "mcxa2xx")]
#[path = "."]
mod mcxa2xx_exclusive {
    pub use crate::chips::mcxa2xx::{init, init_validated};
}

/// Module for MCXA5xx-specific HAL drivers
#[cfg(feature = "mcxa5xx")]
#[path = "."]
mod mcxa5xx_exclusive {
    pub use crate::chips::mcxa5xx::{init, init_validated};
}

pub mod rom;
pub mod trace;

#[cfg(mcxa_adc)]
pub mod adc;
#[cfg(mcxa_cdog)]
pub mod cdog;
#[cfg(any(mcxa_mrcc5xx, mcxa_mrcc2xx))]
pub mod clkout; // TODO: Add dummy driver to metadata
#[cfg(any(mcxa_mrcc5xx, mcxa_mrcc2xx))]
pub mod clocks;
pub mod config;
#[cfg(mcxa_crc)]
pub mod crc;
#[cfg(mcxa_ctimer)]
pub mod ctimer;
#[cfg(mcxa_dma)]
pub mod dma;
#[cfg(feature = "executor-platform")]
pub mod executor;
#[cfg(mcxa_can)]
pub mod flexcan;
#[cfg(mcxa_flexspi)]
pub mod flexspi;
#[cfg(mcxa_gpio)]
pub mod gpio;
#[cfg(mcxa_lpi2c)]
pub mod i2c;
#[cfg(mcxa_i3c)]
pub mod i3c;
#[cfg(mcxa_inputmux)]
pub mod inputmux;
#[cfg(mcxa_lpuart)]
pub mod lpuart;
#[cfg(mcxa_ostimer)]
pub mod ostimer;
pub mod perf_counters;
#[cfg(mcxa_cmc)]
pub mod reset_reason;
#[cfg(mcxa_rtc5xx)]
#[path = "rtc/mcxa5xx.rs"]
pub mod rtc;
#[cfg(mcxa_rtc2xx)]
#[path = "rtc/mcxa2xx.rs"]
pub mod rtc;
#[cfg(mcxa_sgi)]
pub mod sgi;
#[cfg(mcxa_lpspi)]
pub mod spi;
#[cfg(mcxa_trng)]
pub mod trng;
#[cfg(mcxa_wwdt)]
pub mod wwdt;

#[cfg(feature = "mcxa2xx")]
pub use mcxa2xx_exclusive::*;
#[cfg(feature = "mcxa5xx")]
pub use mcxa5xx_exclusive::*;

pub(crate) mod chips;

pub(crate) mod _generated {
    #![allow(dead_code)]
    #![allow(unused_imports)]
    #![allow(non_snake_case)]
    #![allow(missing_docs)]
    #![allow(clippy::missing_safety_doc)]

    include!(concat!(env!("OUT_DIR"), "/_generated.rs"));
}

// Re-export interrupt traits and types
// Re-export Peri and PeripheralType to allow applications to express Peri types and requirements.
pub use embassy_hal_internal::{Peri, PeripheralType};
#[cfg(feature = "unstable-pac")]
pub use nxp_pac as pac;
#[cfg(not(feature = "unstable-pac"))]
pub(crate) use nxp_pac as pac;

pub use crate::_generated::{Peripherals, interrupt, peripherals};

const HALS_SELECTED: usize = const { cfg!(feature = "mcxa2xx") as usize + cfg!(feature = "mcxa5xx") as usize };

/// Ensure exactly one chip feature is set.
#[doc(hidden)]
pub const _SINGLE_HAL_CHECK: bool = const {
    assert!(HALS_SELECTED == 1, "Select exactly one chip feature!");
    HALS_SELECTED == 1
};

/// Macro to bind interrupts to handlers, similar to embassy-imxrt.
///
/// Example:
/// - Bind OS_EVENT to the OSTIMER time-driver handler
///   bind_interrupts!(struct Irqs { OS_EVENT => crate::ostimer::time_driver::OsEventHandler; });
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[unsafe(no_mangle)]
            $(#[cfg($cond_irq)])?
            unsafe extern "C" fn $irq() {
                use embassy_mcxa::interrupt::typelevel::Interrupt;

                $crate::trace::irq_start($crate::interrupt::typelevel::$irq::IRQ);
                unsafe {
                    $(
                        $(#[cfg($cond_handler)])?
                        <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();
                    )*
                }
                $crate::trace::irq_end($crate::interrupt::typelevel::$irq::IRQ);
            }

            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
                )*
            );
        )*
    };
    (@inner $($t:tt)*) => {
        $($t)*
    }
}

/// Declare a clock configuration that is resolved and checked at compile time.
///
/// This macro generates a module holding the configuration, the [`Clocks`] tree
/// it resolves to, and a [`ValidatedClocksConfig`] token. If the configuration
/// is not legal, the build FAILS at the point of declaration, with the clock
/// subsystem's own error message as the diagnostic.
///
/// Passing the generated `VALIDATED` token to
/// [`init_validated()`](crate::init_validated) ensures that the very
/// `ClocksConfig` value that was checked at compile time is the one supplied to
/// runtime clock initialisation. Read the section below for the precise extent
/// of that.
///
/// # What this does and does not guarantee
///
/// GUARANTEED: the [`ClocksConfig`] value checked by `resolve()` at compile time
/// is the same value handed to runtime clock initialisation, when
/// [`init_validated()`](crate::init_validated) is used.
///
/// GUARANTEED: an illegal clock tree, or a declared peripheral config that
/// violates its limits, fails the BUILD at the point of declaration.
///
/// NOT GUARANTEED: you can still call [`init()`](crate::init) or
/// [`clocks::init()`](crate::clocks::init) directly with an unvalidated
/// configuration. Nothing forces you through this macro.
///
/// NOT GUARANTEED: declaring and asserting a peripheral const does NOT force you
/// to pass that const to the driver. You can assert one value and pass another;
/// nothing detects it. Using the declared const is a convention, not an
/// enforcement.
///
/// NOT GUARANTEED: the const resolver and the runtime clock operator are separate
/// implementations of the same clock tree. They share arithmetic helpers, but the
/// control flow is duplicated, so the asserted [`Clocks`] tree is not proven equal
/// to the [`Clocks`] the runtime installs.
///
/// NOT GUARANTEED: hardware readiness - PLL lock, oscillator validity, error
/// flags, divider stability - is not modelled by `resolve()` at all. It is
/// checked only at runtime.
///
/// # Macro hygiene
///
/// The generated module does `use super::*;`, so the declaring scope's imports
/// resolve the same way they would at the call site. Two consequences:
///
/// * A glob-imported item can SHADOW a prelude macro inside the generated module.
///   In particular, a file that does `use defmt::{.., panic, ..}` shadows the
///   prelude `panic!`, so a `panic!` written inside a `clock_config:` or
///   peripheral expression silently becomes `defmt::panic!` and breaks const
///   evaluation. The symptom is a const-eval error pointing at machinery you did
///   not write. Workaround: write `core::panic!(..)` explicitly.
/// * The generated names `CONFIG`, `CLOCKS` and `VALIDATED` can collide with
///   names glob-imported from the parent scope.
///
/// [`ClocksConfig`]: crate::clocks::config::ClocksConfig
/// [`Clocks`]: crate::clocks::Clocks
/// [`ValidatedClocksConfig`]: crate::clocks::ValidatedClocksConfig
///
/// # Example
///
/// ```ignore
/// use embassy_mcxa::clocks::PoweredClock;
/// use embassy_mcxa::clocks::config::{ClocksConfig, MainClockSource};
/// use embassy_mcxa::clocks::periph_helpers::{Div4, FlexspiClockSel};
/// use embassy_mcxa::flexspi::ClockConfig as FlexspiClockConfig;
///
/// embassy_mcxa::validated_clocks! {
///     pub mod board_clocks {
///         clock_config: {
///             let mut c = ClocksConfig::new();
///             c.main_clock.source = MainClockSource::FircHfRoot;
///             c
///         };
///         peripherals: {
///             /// Clock settings for the on-board QSPI flash.
///             pub const FLASH_CLK: FlexspiClockConfig = FlexspiClockConfig {
///                 power: PoweredClock::NormalEnabledDeepSleepDisabled,
///                 source: FlexspiClockSel::FroHf,
///                 div: Div4::no_div(),
///             };
///             validate: FlexspiClockConfig::validate_clock;
///         }
///     }
/// }
///
/// let p = embassy_mcxa::init_validated(Default::default(), board_clocks::VALIDATED);
/// ```
///
/// Each `peripherals` entry names a `const` of a driver's public clock-config
/// type, plus a `validate:` path to a `const fn(&T, &Clocks) -> Result<u32,
/// ClockError>`. The assertion is emitted as a `const _: () = { .. }` item, so
/// it is checked whether or not the constant is ever used.
///
/// The `peripherals` block may be omitted entirely.
#[macro_export]
macro_rules! validated_clocks {
    (
        $(#[$attr:meta])*
        $vis:vis mod $name:ident {
            clock_config: $clock_config:expr;
            peripherals: {
                $(
                    $(#[$pattr:meta])*
                    $pvis:vis const $pname:ident : $pty:ty = $pcfg:expr;
                    validate: $validator:path;
                )*
            }
        }
    ) => {
        $(#[$attr])*
        $vis mod $name {
            // Bring the declaring scope's imports (clock config types, driver
            // config types, ..) into this generated module, so that the
            // user-written expressions below resolve the same way they would
            // have at the macro call site.
            #[allow(unused_imports)]
            use super::*;

            /// The clock configuration declared at this site.
            pub const CONFIG: $crate::clocks::config::ClocksConfig = $clock_config;

            /// `CONFIG` resolved at compile time. The build fails here if it is not legal.
            pub const CLOCKS: $crate::clocks::Clocks = $crate::clocks::__resolve_clocks_or_panic(&CONFIG);

            /// Proof that `CONFIG` resolves; pass to `init_validated()`.
            pub const VALIDATED: $crate::clocks::ValidatedClocksConfig =
                $crate::clocks::__validated_clocks_config(CONFIG);

            $(
                $(#[$pattr])*
                $pvis const $pname: $pty = $pcfg;

                const _: () = {
                    $crate::clocks::__assert_peripheral_clock_valid($validator(&$pname, &CLOCKS));
                };
            )*
        }
    };
    (
        $(#[$attr:meta])*
        $vis:vis mod $name:ident {
            clock_config: $clock_config:expr;
        }
    ) => {
        $crate::validated_clocks! {
            $(#[$attr])*
            $vis mod $name {
                clock_config: $clock_config;
                peripherals: {}
            }
        }
    };
}
