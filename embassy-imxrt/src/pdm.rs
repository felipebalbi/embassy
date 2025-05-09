//! Pulse Density Modulation (PDM) microphone driver.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

// do I need fixed??
use crate::gpio::{AnyPin, GpioPin};
use crate::interrupt::typelevel::Interrupt;
use crate::sealed::Sealed;
use crate::{interrupt, pac, peripherals};

/// Interrupt handler
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO!
        T::waker().wake();
    }
}

/// PDM microphone interface
pub struct Pdm<'d, T: Instance> {
    _peri: Peri<'d, T>,
    _clk: Peri<'d, AnyPin>,
    _din: Peri<'d, AnyPin>,
}

/// PDM error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Other error
    Other,
}

impl<'d, T: Instance> Pdm<'d, T> {
    /// Create PDM driver
    pub fn new(
        _pdm: Peri<'d, T>,
        _clk: Peri<'d, impl PdmClk<T>>,
        _din: Peri<'d, impl PdmDin<T>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        _clk.as_clk();
        _din.as_din();

        Self::new_inner(_pdm, _clk.into(), _din.into(), config)
    }

    fn new_inner(_pdm: Peri<'d, T>, _clk: Peri<'d, AnyPin>, _din: Peri<'d, AnyPin>, config: Config) -> Self {
        todo!()
    }

    /// Adjust the gain of the PDM microphone on the fly
    pub fn set_gain(&mut self, gain_left: u32, gain_right: u32) {
        todo!()
    }

    /// Start sampling microphone data into a dummy buffer.
    /// Useful to start the microphone and keep it active between recording samples.
    pub async fn start(&mut self) {
        todo!()
    }

    /// Stop sampling microphone data inta a dummy buffer
    pub async fn stop(&mut self) {
        todo!()
    }

    /// Sample data into the given buffer
    pub async fn sample(&mut self, buffer: &mut [i16]) -> Result<(), Error> {
        todo!()
    }

    async fn wait_for_sample() {
        todo!()
    }
}

impl Default for Config {
    fn default() -> Self {
        Self { mode: Mode::Mono }
    }
}

/// PDM microphone driver Config
pub struct Config {
    /// Use stereo or mono operation
    pub mode: Mode,
}

/// PDM operating mode
#[derive(PartialEq)]
pub enum Mode {
    /// Mono (1 channel)
    Mono,
    /// Stereo (2 channels)
    Stereo,
}

pub(crate) trait SealedInstance {
    fn regs() -> &'static pac::dmic0::RegisterBlock;
    fn waker() -> &'static AtomicWaker;
}

/// PDM peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this peripheral
    type Interrupt: interrupt::typelevel::Interrupt;
}

macro_rules! impl_instance {
    ($peri:ident, $irq:ident) => {
        impl SealedInstance for peripherals::$peri {
            fn regs() -> &'static pac::dmic0::RegisterBlock {
                unsafe { &*pac::Dmic0::ptr() }
            }

            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }
        }

        impl Instance for peripherals::$peri {
            type Interrupt = interrupt::typelevel::$irq;
        }
    };
}

impl_instance!(DMIC0, DMIC0);

/// PDM clock pin trait
pub trait PdmClk<T: Instance>: GpioPin + Sealed + PeripheralType {
    /// Set pin into a PDM clk function
    fn as_clk(&self);
}

macro_rules! impl_clk {
    ($peri:ident) => {
        impl Sealed for peripherals::$peri {}
        impl<T: Instance> PdmClk<T> for peripherals::$peri {
            fn as_clk(&self) {
                todo!()
            }
        }
    };
}

impl_clk!(PIO2_16);
impl_clk!(PIO2_17);
impl_clk!(PIO2_18);
impl_clk!(PIO2_19);

impl_clk!(PIO3_0);
impl_clk!(PIO3_1);
impl_clk!(PIO3_2);
impl_clk!(PIO3_3);

/// PDM data pin trait
pub trait PdmDin<T: Instance>: GpioPin + Sealed + PeripheralType {
    /// Set pin into a PDM data function
    fn as_din(&self);
}

macro_rules! impl_din {
    ($peri:ident) => {
        impl Sealed for peripherals::$peri {}
        impl<T: Instance> PdmDin<T> for peripherals::$peri {
            fn as_din(&self) {
                todo!()
            }
        }
    };
}

impl_din!(PIO2_20);
impl_din!(PIO2_21);
impl_din!(PIO2_22);
impl_din!(PIO2_23);

impl_din!(PIO3_4);
impl_din!(PIO3_5);
impl_din!(PIO3_6);
impl_din!(PIO3_7);
