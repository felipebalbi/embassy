//! I3C target driver

use core::marker::PhantomData;

use embassy_hal_internal::Peri;
use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::interrupt::InterruptExt;
use nxp_pac::i3c::vals::{SdatactrlRxtrig, SdatactrlTxtrig};

use super::{Async, Blocking, Dma, Info, Instance, InterruptHandler, Mode, SclPin, SdaPin};
use crate::clocks::periph_helpers::{Div4, I3cClockSel, I3cConfig};
use crate::clocks::{ClockError, PoweredClock, WakeGuard, enable_and_reset};
use crate::dma::{Channel, DmaChannel};
use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt;
use crate::interrupt::typelevel::Interrupt;

const MAX_CHUNK_SIZE: usize = 255;

/// Setup Errors
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SetupError {
    /// Clock configuration error.
    ClockSetup(ClockError),
    /// Other internal errors or unexpected state.
    Other,
}

/// I/O Errors
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IOError {
    /// Other internal errors or unexpected state.
    Other,
}

/// I3C target configuration
#[derive(Clone)]
#[non_exhaustive]
pub struct Config {
    /// Address to respond to
    pub address: u8,

    /// Clock configuration
    pub clock_config: ClockConfig,

    /// 15-bit MIPI Vendor ID.
    ///
    /// If bit 15 is set, it will be ignored.
    pub vendor_id: u16,

    /// Part number
    pub partno: u32,

    /// Maximum write length.
    ///
    /// Valid range is between 8 and 4096 inclusive.
    pub max_write_length: u16,

    /// Maximum read length
    ///
    /// Valid range is between 16 and 4096 inclusive.
    pub max_read_length: u16,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            address: 0x1e,
            clock_config: Default::default(),
            vendor_id: 0,
            partno: 0,
            max_write_length: 256,
            max_read_length: 256,
        }
    }
}

/// I3C target clock configuration
#[derive(Clone)]
#[non_exhaustive]
pub struct ClockConfig {
    /// Powered clock configuration
    pub power: PoweredClock,
    /// LPI2C clock source
    pub source: I3cClockSel,
    /// LPI2C pre-divider
    pub div: Div4,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            power: PoweredClock::NormalEnabledDeepSleepDisabled,
            source: I3cClockSel::FroLfDiv,
            div: const { Div4::no_div() },
        }
    }
}

/// I3C target events
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Request {
    /// Controller wants to write data to this Target
    Write(u16),
    /// Controller wants to read data from this Target
    Read(u16),
    /// Controller issued Stop condition for this Target
    Stop(u16),
}

/// I3C target driver.
pub struct I3c<'d, M: Mode> {
    info: &'static Info,
    _scl: Peri<'d, AnyPin>,
    _sda: Peri<'d, AnyPin>,
    mode: M,
    _wg: Option<WakeGuard>,
}

impl<'d, M: Mode> I3c<'d, M> {
    fn new_inner<T: Instance>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
        mode: M,
    ) -> Result<Self, SetupError> {
        let ClockConfig { power, source, div } = config.clock_config;

        // Enable clocks
        let conf = I3cConfig { power, source, div };

        let parts = unsafe { enable_and_reset::<T>(&conf).map_err(SetupError::ClockSetup)? };

        scl.mux();
        sda.mux();

        let _scl = scl.into();
        let _sda = sda.into();

        let inst = Self {
            info: T::info(),
            _scl,
            _sda,
            mode,
            _wg: parts.wake_guard,
        };

        inst.set_configuration(&config)?;

        Ok(inst)
    }

    fn set_configuration(&self, config: &Config) -> Result<(), SetupError> {
        // Disable the target
        self.info.regs().sconfig().write(|w| {
            w.set_slvena(false);
        });

        self.info.regs().sdatactrl().modify(|w| {
            w.set_flushtb(true);
            w.set_flushfb(true);
            w.set_unlock(true);
            w.set_txtrig(SdatactrlTxtrig::TRIGGRONELESS);
            w.set_rxtrig(SdatactrlRxtrig::TRIGGRNOTEMPTY);
        });

        self.info
            .regs()
            .svendorid()
            .write(|w| w.set_vid(config.vendor_id & 0x7fff));

        self.info.regs().sidpartno().write(|w| w.set_partno(config.partno));

        self.info.regs().smaxlimits().write(|w| {
            w.set_maxrd(config.max_read_length & 0x0fff);
            w.set_maxwr(config.max_write_length & 0x0fff);
        });

        self.info.regs().sconfig().modify(|w| {
            w.set_saddr(config.address << 1);
            w.set_slvena(false);
        });

        Ok(())
    }
}

impl<'d> I3c<'d, Blocking> {
    /// Create a new blocking instance of the I3C Target bus driver.
    ///
    /// Any external pin will be placed into Disabled state upon Drop.
    pub fn new_blocking<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        config: Config,
    ) -> Result<Self, SetupError> {
        Self::new_inner(peri, scl, sda, config, Blocking)
    }
}

impl<'d> I3c<'d, Async> {
    /// Create a new blocking instance of the I3C Target bus driver.
    ///
    /// Any external pin will be placed into Disabled state upon Drop.
    pub fn new_async<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, SetupError> {
        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        Self::new_inner(peri, scl, sda, config, Async)
    }
}

impl<'d> I3c<'d, Dma<'d>> {
    /// Create a new blocking instance of the I3C Target bus driver.
    ///
    /// Any external pin will be placed into Disabled state upon Drop.
    pub fn new_async<T: Instance>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T>>,
        sda: Peri<'d, impl SdaPin<T>>,
        tx_dma: Peri<'d, impl Channel>,
        rx_dma: Peri<'d, impl Channel>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, SetupError> {
        T::Interrupt::unpend();

        // Safety: `_irq` ensures an Interrupt Handler exists.
        unsafe { T::Interrupt::enable() };

        // enable this channel's interrupt
        let tx_dma = DmaChannel::new(tx_dma);
        let rx_dma = DmaChannel::new(rx_dma);

        tx_dma.enable_interrupt();
        rx_dma.enable_interrupt();

        Self::new_inner(
            peri,
            scl,
            sda,
            config,
            Dma {
                tx_dma,
                rx_dma,
                tx_request: T::TX_DMA_REQUEST,
                rx_request: T::RX_DMA_REQUEST,
            },
        )
    }
}

impl<'d, M: Mode> I3c<'d, M> {
    // Public API: Blocking

    /// Block waiting for new events
    pub fn blocking_listen(&mut self) -> Result<Request, IOError> {
        let status = self.info.regs().sstatus().read();
        defmt::info!("{}", status);
        Ok(Request::Write(0x2a))
    }

    /// Transmit the contents of `buf` to the I3C controller.
    ///
    /// Returns either an `Ok(usize)` containing the number of bytes
    /// transmitted, or an `Error`.
    pub fn blocking_respond_to_read(&mut self, buf: &[u8]) -> Result<usize, IOError> {
        todo!()
    }

    /// Receive data from the I3C controller into `buf`.
    ///
    /// Care is taken to guarantee that we receive at most `buf.len()`
    /// bytes. On success returns `Ok(usize)` containing the number of
    /// bytes received or an `Error`.
    pub fn blocking_respond_to_write(&mut self, buf: &mut [u8]) -> Result<usize, IOError> {
        todo!()
    }
}
