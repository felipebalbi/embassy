//! CTimer driver.

use core::marker::PhantomData;

use embassy_hal_internal::{Peri, PeripheralType};
use maitake_sync::WaitCell;

use crate::clkout::Div4;
use crate::clocks::periph_helpers::{CTimerClockSel, CTimerConfig};
use crate::clocks::{ClockError, Gate, PoweredClock, WakeGuard, enable_and_reset};
use crate::gpio::{GpioPin, SealedPin};
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, pac};

/// Error information type
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Clock configuration error.
    ClockSetup(ClockError),

    /// Other internal errors or unexpected state.
    Other,
}

/// CTimer configuration
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct Config;

/// CTimer core driver.
pub struct CTimer<'d> {
    info: &'static Info,
    _wg: Option<WakeGuard>,
    _phantom: PhantomData<&'d mut ()>,
}

impl<'d> CTimer<'d> {
    /// Create a new instance of the CTimer core cdriver.
    pub fn new<T: Instance>(
        _peri: Peri<'d, T>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        _config: Config,
    ) -> Result<Self, Error> {
        // Enable clocks
        let conf = CTimerConfig {
            power: PoweredClock::NormalEnabledDeepSleepDisabled,
            source: CTimerClockSel::FroLfDiv,
            div: const { Div4::no_div() },
            instance: T::CLOCK_INSTANCE,
        };

        let parts = unsafe { enable_and_reset::<T>(&conf).map_err(Error::ClockSetup)? };

        let inst = Self {
            info: T::info(),
            _wg: parts.wake_guard,
            _phantom: PhantomData,
        };

        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        // Enable CTimer
        inst.info.regs().tcr().modify(|_, w| w.cen().enabled());

        Ok(inst)
    }
}

/// CTimer interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // Clear interrupt status
        T::info().regs().ir().write(|w| {
            w.mr0int()
                .set_bit()
                .mr1int()
                .set_bit()
                .mr2int()
                .set_bit()
                .mr3int()
                .set_bit()
                .cr0int()
                .set_bit()
                .cr1int()
                .set_bit()
                .cr2int()
                .set_bit()
                .cr3int()
                .set_bit()
        });

        T::info().wait_cell().wake();
    }
}

struct Info {
    regs: *const pac::ctimer0::RegisterBlock,
    wait_cell: WaitCell,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> &pac::ctimer0::RegisterBlock {
        unsafe { &*self.regs }
    }

    #[inline(always)]
    fn wait_cell(&self) -> &WaitCell {
        &self.wait_cell
    }
}

unsafe impl Sync for Info {}

trait SealedInstance {
    fn info() -> &'static Info;
}

/// CTimer Instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send + Gate<MrccPeriphConfig = CTimerConfig> {
    /// Interrupt for this I2C instance.
    type Interrupt: interrupt::typelevel::Interrupt;
    /// Clock instance
    const CLOCK_INSTANCE: crate::clocks::periph_helpers::CTimerInstance;
}

macro_rules! impl_instance {
    ($peri:ident, $mod:ident, $clock:ident) => {
        impl SealedInstance for crate::peripherals::$peri {
            fn info() -> &'static Info {
                static INFO: Info = Info {
                    regs: pac::$mod::ptr(),
                    wait_cell: WaitCell::new(),
                };
                &INFO
            }
        }

        impl Instance for crate::peripherals::$peri {
            type Interrupt = crate::interrupt::typelevel::$peri;
            const CLOCK_INSTANCE: crate::clocks::periph_helpers::CTimerInstance =
                crate::clocks::periph_helpers::CTimerInstance::$clock;
        }
    };
}

impl_instance!(CTIMER0, Ctimer0, CTimer0);
impl_instance!(CTIMER1, Ctimer1, CTimer1);
impl_instance!(CTIMER2, Ctimer2, CTimer2);
impl_instance!(CTIMER3, Ctimer3, CTimer3);
impl_instance!(CTIMER4, Ctimer4, CTimer4);

/// Seal a trait
trait SealedInputPin {
    fn number(&self) -> u8;
}

/// Seal a trait
trait SealedOutputPin<T: Instance> {
    fn number(&self) -> u8;
}

/// CTimer input pin.
#[allow(private_bounds)]
pub trait InputPin: GpioPin + SealedInputPin + PeripheralType {
    fn mux(&self);
}

/// CTimer output pin.
#[allow(private_bounds)]
pub trait OutputPin<T: Instance>: GpioPin + SealedOutputPin<T> + PeripheralType {
    fn mux(&self);
}

macro_rules! impl_pin {
    ($pin:ident, $fn:ident, $n:expr) => {
        impl SealedInputPin for crate::peripherals::$pin {
            #[inline(always)]
            fn number(&self) -> u8 {
                $n + 1
            }
        }

        impl InputPin for crate::peripherals::$pin {
            #[inline(always)]
            fn mux(&self) {
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_slew_rate(crate::gpio::SlewRate::Fast.into());
                self.set_drive_strength(crate::gpio::DriveStrength::Double.into());
                self.set_function(crate::pac::port0::pcr0::Mux::$fn);
                self.set_enable_input_buffer();
            }
        }
    };

    ($pin:ident, $peri:ident, $fn:ident, $n:expr) => {
        impl SealedOutputPin<crate::peripherals::$peri> for crate::peripherals::$pin {
            #[inline(always)]
            fn number(&self) -> u8 {
                $n + 32
            }
        }

        impl OutputPin<crate::peripherals::$peri> for crate::peripherals::$pin {
            #[inline(always)]
            fn mux(&self) {
                self.set_pull(crate::gpio::Pull::Disabled);
                self.set_slew_rate(crate::gpio::SlewRate::Fast.into());
                self.set_drive_strength(crate::gpio::DriveStrength::Double.into());
                self.set_function(crate::pac::port0::pcr0::Mux::$fn);
                self.set_enable_input_buffer();
            }
        }
    };
}

// Input pins

#[cfg(feature = "swd-as-gpio")]
impl_pin!(P0_0, Mux4, 0);
#[cfg(feature = "swd-as-gpio")]
impl_pin!(P0_1, Mux4, 1);
#[cfg(feature = "jtag-extras-as-gpio")]
impl_pin!(P0_6, Mux4, 2);

impl_pin!(P0_20, Mux4, 0);
impl_pin!(P0_21, Mux4, 1);
impl_pin!(P0_22, Mux4, 2);
impl_pin!(P0_23, Mux4, 3);

impl_pin!(P1_0, Mux4, 4);
impl_pin!(P1_1, Mux4, 5);
impl_pin!(P1_2, Mux5, 0);
impl_pin!(P1_3, Mux5, 1);
impl_pin!(P1_6, Mux4, 6);
impl_pin!(P1_7, Mux4, 7);
impl_pin!(P1_8, Mux4, 8);
impl_pin!(P1_9, Mux4, 9);
impl_pin!(P1_14, Mux4, 10);
impl_pin!(P1_15, Mux4, 11);

#[cfg(feature = "sosc-as-gpio")]
impl_pin!(P1_30, Mux4, 16);
#[cfg(feature = "sosc-as-gpio")]
impl_pin!(P1_31, Mux4, 17);

impl_pin!(P2_0, Mux4, 16);
impl_pin!(P2_1, Mux4, 17);
impl_pin!(P2_2, Mux4, 12);
impl_pin!(P2_3, Mux4, 13);
impl_pin!(P2_4, Mux4, 14);
impl_pin!(P2_5, Mux4, 15);
impl_pin!(P2_6, Mux4, 18);
impl_pin!(P2_7, Mux4, 19);

impl_pin!(P3_0, Mux4, 16);
impl_pin!(P3_1, Mux4, 17);
impl_pin!(P3_8, Mux4, 4);
impl_pin!(P3_9, Mux4, 5);
impl_pin!(P3_14, Mux4, 6);
impl_pin!(P3_15, Mux4, 7);
impl_pin!(P3_16, Mux4, 8);
impl_pin!(P3_17, Mux4, 9);
impl_pin!(P3_22, Mux4, 10);
impl_pin!(P3_27, Mux4, 13);
impl_pin!(P3_28, Mux4, 12);
impl_pin!(P3_29, Mux4, 3);

impl_pin!(P4_6, Mux4, 6);
impl_pin!(P4_7, Mux4, 7);

// Output pins
#[cfg(feature = "swd-swo-as-gpio")]
impl_pin!(P0_2, CTIMER0, Mux4, 0);
#[cfg(feature = "jtag-extras-as-gpio")]
impl_pin!(P0_3, CTIMER0, Mux4, 1);
impl_pin!(P0_16, CTIMER0, Mux4, 0);
impl_pin!(P0_17, CTIMER0, Mux4, 1);
impl_pin!(P0_18, CTIMER0, Mux4, 2);
impl_pin!(P0_19, CTIMER0, Mux4, 3);
impl_pin!(P0_22, CTIMER0, Mux5, 0);
impl_pin!(P0_23, CTIMER0, Mux5, 1);

impl_pin!(P1_0, CTIMER0, Mux5, 2);
impl_pin!(P1_1, CTIMER0, Mux5, 3);
impl_pin!(P1_2, CTIMER1, Mux4, 0);
impl_pin!(P1_3, CTIMER1, Mux4, 1);
impl_pin!(P1_4, CTIMER1, Mux4, 2);
impl_pin!(P1_5, CTIMER1, Mux4, 3);
impl_pin!(P1_6, CTIMER4, Mux5, 0);
impl_pin!(P1_7, CTIMER4, Mux5, 1);
impl_pin!(P1_8, CTIMER0, Mux5, 2);
impl_pin!(P1_9, CTIMER0, Mux5, 3);
impl_pin!(P1_10, CTIMER2, Mux4, 0);
impl_pin!(P1_11, CTIMER2, Mux4, 1);
impl_pin!(P1_12, CTIMER2, Mux4, 2);
impl_pin!(P1_13, CTIMER2, Mux4, 3);
impl_pin!(P1_14, CTIMER3, Mux5, 0);
impl_pin!(P1_15, CTIMER3, Mux5, 1);

impl_pin!(P2_0, CTIMER2, Mux5, 0);
impl_pin!(P2_1, CTIMER2, Mux5, 1);
impl_pin!(P2_2, CTIMER2, Mux5, 2);
impl_pin!(P2_3, CTIMER2, Mux5, 3);
impl_pin!(P2_4, CTIMER1, Mux5, 0);
impl_pin!(P2_5, CTIMER1, Mux5, 1);
impl_pin!(P2_6, CTIMER1, Mux5, 2);
impl_pin!(P2_7, CTIMER1, Mux5, 3);
impl_pin!(P2_10, CTIMER3, Mux4, 2);
impl_pin!(P2_11, CTIMER3, Mux4, 3);
impl_pin!(P2_12, CTIMER4, Mux4, 0);
impl_pin!(P2_12, CTIMER0, Mux5, 0);
impl_pin!(P2_13, CTIMER4, Mux4, 1);
impl_pin!(P2_13, CTIMER0, Mux5, 1);
impl_pin!(P2_15, CTIMER4, Mux5, 3);
impl_pin!(P2_15, CTIMER0, Mux5, 2);
impl_pin!(P2_16, CTIMER3, Mux5, 0);
impl_pin!(P2_16, CTIMER0, Mux5, 2);
impl_pin!(P2_17, CTIMER3, Mux5, 1);
impl_pin!(P2_17, CTIMER0, Mux5, 3);
impl_pin!(P2_19, CTIMER3, Mux4, 3);
impl_pin!(P2_20, CTIMER2, Mux4, 0);
impl_pin!(P2_21, CTIMER2, Mux4, 1);
impl_pin!(P2_23, CTIMER2, Mux4, 3);

impl_pin!(P3_2, CTIMER4, Mux4, 0);
impl_pin!(P3_6, CTIMER4, Mux4, 2);
impl_pin!(P3_7, CTIMER4, Mux4, 3);
impl_pin!(P3_10, CTIMER1, Mux4, 0);
impl_pin!(P3_11, CTIMER1, Mux4, 1);
impl_pin!(P3_12, CTIMER1, Mux4, 2);
impl_pin!(P3_13, CTIMER1, Mux4, 3);
impl_pin!(P3_18, CTIMER2, Mux4, 0);
impl_pin!(P3_19, CTIMER2, Mux4, 1);
impl_pin!(P3_20, CTIMER2, Mux4, 2);
impl_pin!(P3_21, CTIMER2, Mux4, 3);
impl_pin!(P3_27, CTIMER3, Mux5, 1);
impl_pin!(P3_28, CTIMER3, Mux5, 2);
#[cfg(feature = "dangerous-reset-as-gpio")]
impl_pin!(P3_29, CTIMER3, Mux5, 3);
#[cfg(feature = "sosc-as-gpio")]
impl_pin!(P3_30, CTIMER0, Mux4, 2);
#[cfg(feature = "sosc-as-gpio")]
impl_pin!(P3_31, CTIMER0, Mux4, 3);

impl_pin!(P4_2, CTIMER4, Mux4, 0);
impl_pin!(P4_3, CTIMER4, Mux4, 1);
impl_pin!(P4_4, CTIMER4, Mux4, 2);
impl_pin!(P4_5, CTIMER4, Mux4, 3);
