//! I3C Controller driver.

use core::marker::PhantomData;

use embassy_hal_internal::Peri;

use super::{Async, Blocking, Error, Info, InterruptHandler, Mode, Result, SclPin, SdaPin};
use crate::clocks::periph_helpers::{Div4, I3cClockSel, I3cConfig};
use crate::clocks::{PoweredClock, enable_and_reset};
use crate::gpio::AnyPin;
use crate::interrupt::typelevel;
use crate::peripherals::I3C0;

/// Bus speed (nominal SCL, no clock stretching)
#[derive(Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    #[default]
    /// 100 kbit/sec
    Standard,
    /// 400 kbit/sec
    Fast,
    /// 1 Mbit/sec
    FastPlus,
    /// 3.4 Mbit/sec
    UltraFast,
}

impl From<Speed> for (u8, u8, u8, u8) {
    fn from(value: Speed) -> (u8, u8, u8, u8) {
        match value {
            Speed::Standard => (0x3d, 0x37, 0x3b, 0x1d),
            Speed::Fast => (0x0e, 0x0c, 0x0d, 0x06),
            Speed::FastPlus => (0x04, 0x03, 0x03, 0x02),

            // UltraFast is "special". Leaving it unimplemented until
            // the driver and the clock API is further stabilized.
            Speed::UltraFast => todo!(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum SendStop {
    No,
    Yes,
}

/// I3C controller configuration
pub struct Config {
    /// Bus speed
    pub speed: Speed,
}

/// I3C controller driver.
pub struct I3c<'d, M: Mode> {
    info: &'static Info,
    _scl: Peri<'d, AnyPin>,
    _sda: Peri<'d, AnyPin>,
    _phantom: PhantomData<M>,
}

impl<'d, M: Mode> I3c<'d, M> {
    fn new_inner(
        _peri: Peri<'d, I3C0>,
        scl: Peri<'d, impl SclPin<I3C0>>,
        sda: Peri<'d, impl SdaPin<I3C0>>,
        config: Config,
    ) -> Result<Self> {
        let (power, source, div) = Self::clock_config(config.speed);

        // Enable clocks
        let conf = I3cConfig { power, source, div };

        _ = unsafe { enable_and_reset::<I3C0>(&conf).map_err(Error::ClockSetup)? };

        scl.mux();
        sda.mux();

        let _scl = scl.into();
        let _sda = sda.into();

        let inst = Self {
            info: super::info(),
            _scl,
            _sda,
            _phantom: PhantomData,
        };

        inst.set_configuration(&config)?;

        Ok(inst)
    }

    // REVISIT: turn this into a function of the speed parameter
    fn clock_config(_speed: Speed) -> (PoweredClock, I3cClockSel, Div4) {
        (
            PoweredClock::NormalEnabledDeepSleepDisabled,
            I3cClockSel::FroLfDiv,
            const { Div4::no_div() },
        )
    }

    fn set_configuration(&self, _config: &Config) -> Result<()> {
        let _regs = self.info.regs();

        todo!()
    }

    fn blocking_read_internal(&self, _address: u8, _read: &mut [u8], _send_stop: SendStop) -> Result<()> {
        todo!()
    }

    fn blocking_write_internal(&self, _address: u8, _write: &[u8], _send_stop: SendStop) -> Result<()> {
        todo!()
    }

    // Public API: Blocking

    /// Read from address into buffer blocking caller until done.
    pub fn blocking_read(&mut self, address: u8, read: &mut [u8]) -> Result<()> {
        self.blocking_read_internal(address, read, SendStop::Yes)
    }

    /// Write to address from buffer blocking caller until done.
    pub fn blocking_write(&mut self, address: u8, write: &[u8]) -> Result<()> {
        self.blocking_write_internal(address, write, SendStop::Yes)
    }

    /// Write to address from bytes and read from address into buffer blocking caller until done.
    pub fn blocking_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<()> {
        self.blocking_write_internal(address, write, SendStop::No)?;
        self.blocking_read_internal(address, read, SendStop::Yes)
    }
}

impl<'d> I3c<'d, Blocking> {
    /// Create a new blocking instance of the I3C controller bus driver.
    pub fn new_blocking(
        peri: Peri<'d, I3C0>,
        scl: Peri<'d, impl SclPin<I3C0>>,
        sda: Peri<'d, impl SdaPin<I3C0>>,
        config: Config,
    ) -> Result<Self> {
        Self::new_inner(peri, scl, sda, config)
    }
}

impl<'d> I3c<'d, Async> {
    /// Create a new asynchronous instance of the I3C controller bus driver.
    pub fn new_async(
        peri: Peri<'d, I3C0>,
        scl: Peri<'d, impl SclPin<I3C0>>,
        sda: Peri<'d, impl SdaPin<I3C0>>,
        _irq: impl typelevel::Binding<typelevel::I3C0, InterruptHandler> + 'd,
        config: Config,
    ) -> Result<Self> {
        Self::new_inner(peri, scl, sda, config)
    }
}
