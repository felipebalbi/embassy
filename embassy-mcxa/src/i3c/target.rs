//! I3C Target Support

use core::marker::PhantomData;

use embassy_hal_internal::Peri;

use super::{Async, AsyncMode, Dma, Info, Instance, Mode, SclPin, SdaPin};
pub use crate::clocks::periph_helpers::{Div4, I3cClockSel, I3cConfig};
use crate::clocks::{ClockError, PoweredClock, WakeGuard, enable_and_reset};
use crate::dma::{Channel, DmaChannel};
use crate::gpio::{AnyPin, SealedPin};
use crate::interrupt::typelevel;
use crate::interrupt::typelevel::Interrupt;
use crate::pac::i3c::Sstatus;
use crate::pac::i3c::{Evdet, Mstena, SctrlEvent, SstatusStart, SstatusTxnotfull, Stnotstop, Type};

/// Setup Errors
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SetupError {
    /// Clock configuration error.
    ClockSetup(ClockError),
    /// User provided an invalid configuration
    InvalidConfiguration,
    /// Invalid Vendor ID
    InvalidVendorId,
    /// Invalid Part Number
    InvalidPartNumber,
    /// Other internal errors or unexpected state.
    Other,
}

/// I/O Errors
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IOError {
    /// Overrun error
    Overrun,
    /// Underrun error
    Underrun,
    /// Underrun and NACK error
    UnderrunNack,
    /// Terminated error
    Terminated,
    /// Invalid start error
    InvalidStart,
    /// SDR parity error
    SdrParity,
    /// HDR parity error
    HdrParity,
    /// HDR-DDR CRC error
    HdrDdrCrc,
    /// TE0 or TE1 Error
    TE0TE1,
    /// Overread error
    Overread,
    /// Overwrite
    Overwrite,
    /// IBI request was NACKed by the controller and won't be retried.
    IbiNacked,
    /// Other internal errors or unexpected state.
    Other,
}

impl From<crate::dma::InvalidParameters> for IOError {
    fn from(_value: crate::dma::InvalidParameters) -> Self {
        Self::Other
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(dead_code)]
pub enum BusType {
    /// I3C SDR
    #[default]
    I3cSdr,
    /// Legacy I2C
    I2c,
    /// I3C DDR
    I3cDdr,
}

impl From<BusType> for Type {
    fn from(value: BusType) -> Self {
        match value {
            BusType::I3cSdr => Self::I3C,
            BusType::I2c => Self::I2C,
            BusType::I3cDdr => Self::DDR,
        }
    }
}

/// I3C target configuration
#[non_exhaustive]
pub struct Config {
    /// 7-bit target address.
    pub address: Option<u8>,

    /// Vendor ID
    pub vendor_id: Option<u16>,

    /// Part number
    pub partno: Option<u32>,

    /// Max write length.
    ///
    /// Must be within the range 8..=4095. Values outside this range
    /// will be clamped accordingly.
    pub max_write_len: u16,

    /// Max read length
    ///
    /// Must be within the range 16..=4095. Values outside this range
    /// will be clamped accordingly.
    pub max_read_len: u16,

    /// Advertise IBI request capability (BCR[1]).
    ///
    /// When `true`, the controller can see via `GETBCR` that this target
    /// is capable of generating In-Band Interrupts.  Set this before
    /// calling [`I3c::async_send_ibi`].
    pub ibi_capable: bool,

    /// Advertise that IBIs are followed by a mandatory data byte (BCR[2]).
    ///
    /// When `true`, the controller will expect one MDB byte to follow the
    /// IBI address header.  Set `SCTRL.IBIDATA` before asserting the event.
    pub ibi_has_payload: bool,

    /// Clock configuration
    pub clock_config: ClockConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            address: None,
            vendor_id: None,
            partno: None,
            max_write_len: 256,
            max_read_len: 256,
            ibi_capable: false,
            ibi_has_payload: false,
            clock_config: ClockConfig::default(),
        }
    }
}

/// I3C controller clock configuration
#[derive(Clone)]
#[non_exhaustive]
pub struct ClockConfig {
    /// Powered clock configuration
    pub power: PoweredClock,
    /// I3C clock source
    pub source: I3cClockSel,
    /// I3C pre-divider
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

/// Possible completitions of a read transaction.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadStatus {
    /// Transaction Complete, but controller stopped reading bytes before we ran out
    EarlyStop(usize),
    /// Transaction Complete, controller naked our last byte
    Complete(usize),
    /// Transaction Incomplete, controller trying to read more bytes than were provided
    Incomplete(usize),
}

/// I3C controller driver.
pub struct I3c<'d, M: Mode> {
    info: &'static Info,
    _scl: Peri<'d, AnyPin>,
    _sda: Peri<'d, AnyPin>,
    _mode: M,
    freq: u32,
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
            _mode: mode,
            freq: parts.freq,
            _wg: parts.wake_guard,
        };

        inst.set_configuration(&config)?;

        Ok(inst)
    }

    fn check_status(&self) -> Result<(), IOError> {
        let status = self.info.regs().sstatus().read();
        let errwarn = self.info.regs().serrwarn().read();

        if status.errwarn() {
            if errwarn.orun() {
                Err(IOError::Overrun)
            } else if errwarn.urun() {
                Err(IOError::Underrun)
            } else if errwarn.urunnack() {
                Err(IOError::UnderrunNack)
            } else if errwarn.term() {
                Err(IOError::Terminated)
            } else if errwarn.invstart() {
                Err(IOError::InvalidStart)
            } else if errwarn.spar() {
                Err(IOError::SdrParity)
            } else if errwarn.hpar() {
                Err(IOError::HdrParity)
            } else if errwarn.hcrc() {
                Err(IOError::HdrDdrCrc)
            } else if errwarn.s0s1() {
                Err(IOError::TE0TE1)
            } else if errwarn.oread() {
                Err(IOError::Overread)
            } else if errwarn.owrite() {
                Err(IOError::Overwrite)
            } else {
                Err(IOError::Other)
            }
        } else {
            Ok(())
        }
    }

    fn clear_status(&self) -> Sstatus {
        let status = self.info.regs().sstatus().read();
        self.info.regs().sstatus().write(|w| w.0 = status.0);
        status
    }

    fn flush_fifos(&self) {
        self.info.regs().sdatactrl().write(|w| {
            w.set_flushfb(true);
            w.set_flushtb(true);
        });
    }

    fn set_configuration(&self, config: &Config) -> Result<(), SetupError> {
        self.info.regs().mconfig().write(|w| w.set_mstena(Mstena::MASTER_OFF));

        // Disable target
        self.info.regs().sconfig().write(|w| {
            w.set_slvena(false);
            w.set_saddr(config.address.unwrap_or(0));
            w.set_matchss(true);
            w.set_s0ignore(true);
            #[cfg(feature = "mcxa5xx")]
            w.set_hdrok(true);
            w.set_bamatch((self.freq / 1_000_000 - 1) as u8);
        });

        if config.partno.is_some() {
            let partno = config.partno.unwrap();

            if partno == 0 {
                return Err(SetupError::InvalidPartNumber);
            }

            self.info.regs().sidpartno().write(|w| w.set_partno(partno));
        }

        if config.vendor_id.is_some() {
            let vendor_id = config.vendor_id.unwrap();

            if vendor_id == 0 {
                return Err(SetupError::InvalidVendorId);
            }

            self.info.regs().svendorid().write(|w| w.set_vid(vendor_id));
        }

        self.info.regs().smaxlimits().write(|w| {
            w.set_maxwr(config.max_write_len.clamp(8, 4095));
            w.set_maxrd(config.max_read_len.clamp(16, 4095));
        });

        // Configure BCR (Bus Characteristics Register) — visible to the controller via GETBCR.
        // BCR[1]: IBI Request Capable; BCR[2]: IBI Payload (mandatory data byte follows).
        let bcr: u8 = if config.ibi_capable { 0x02 } else { 0 } | if config.ibi_has_payload { 0x04 } else { 0 };
        self.info.regs().sidext().modify(|w| w.set_bcr(bcr));

        self.clear_status();
        self.flush_fifos();

        // Enable target
        self.info.regs().sconfig().modify(|w| w.set_slvena(true));

        Ok(())
    }
}

impl<'d> I3c<'d, Async> {
    /// Create a new asynchronous instance of the I2C Target bus driver.
    ///
    /// This function initializes the I2C target driver in asynchronous mode. It configures the
    /// I2C peripheral, sets up the clock, and prepares the pins for operation. Any external
    /// pin will be placed into the Disabled state upon `Drop`.
    ///
    /// # Parameters
    ///
    /// - `peri`: The I2C peripheral instance.
    /// - `scl`: The SCL pin.
    /// - `sda`: The SDA pin.
    /// - `_irq`: The interrupt binding for the I2C peripheral.
    /// - `config`: The configuration for the I2C target.
    ///
    /// # Returns
    ///
    /// - `Ok(Self)` on success.
    /// - `Err(SetupError)` if initialization fails.
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
    /// Create a new asynchronous instance of the I2C Target bus driver with DMA support.
    ///
    /// This function initializes the I2C target driver in asynchronous mode with DMA support.
    /// It configures the I2C peripheral, sets up the clock, and prepares the pins for operation.
    /// Any external pin will be placed into the Disabled state upon `Drop`, and the DMA channels
    /// are also disabled.
    ///
    /// # Parameters
    ///
    /// - `peri`: The I2C peripheral instance.
    /// - `scl`: The SCL pin.
    /// - `sda`: The SDA pin.
    /// - `tx_dma`: The DMA channel for transmitting data.
    /// - `rx_dma`: The DMA channel for receiving data.
    /// - `_irq`: The interrupt binding for the I2C peripheral.
    /// - `config`: The configuration for the I2C target.
    ///
    /// # Returns
    ///
    /// - `Ok(Self)` on success.
    /// - `Err(SetupError)` if initialization fails.
    pub fn new_async_with_dma<T: Instance>(
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

trait AsyncEngine {
    fn async_respond_to_read_internal<'a>(
        &'a mut self,
        buf: &'a [u8],
    ) -> impl Future<Output = Result<ReadStatus, IOError>> + 'a;

    fn async_respond_to_write_internal<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<usize, IOError>> + 'a;
}

impl<'d> AsyncEngine for I3c<'d, Async> {
    async fn async_respond_to_read_internal<'a>(&'a mut self, buf: &'a [u8]) -> Result<ReadStatus, IOError> {
        let mut count = 0;

        self.clear_status();

        if let Some((last, data)) = buf.split_last() {
            for byte in data.iter() {
                match self.check_status() {
                    Ok(()) => {}
                    Err(IOError::Terminated) => return Ok(ReadStatus::EarlyStop(count)),
                    Err(e) => return Err(e),
                }

                if self.info.regs().sstatus().read().txnotfull() == SstatusTxnotfull::FULL {
                    // Wait until we have space in the FIFO.
                    self.info
                        .wait_cell()
                        .wait_for(|| {
                            self.info.regs().sintset().write(|w| {
                                w.set_errwarn(true);
                                w.set_txsend(true);
                            });

                            let status = self.info.regs().sstatus().read();
                            status.errwarn() || status.txnotfull() == SstatusTxnotfull::NOT_FULL
                        })
                        .await
                        .map_err(|_| IOError::Other)?;

                    match self.check_status() {
                        Ok(()) => {}
                        Err(IOError::Terminated) => return Ok(ReadStatus::EarlyStop(count)),
                        Err(e) => return Err(e),
                    }
                }

                self.info.regs().swdatab().write(|w| w.set_data(*byte));
                count += 1;

                // If we see a STOP or REPEATED START, break out
                let status = self.info.regs().sstatus().read();
                if status.stnotstop() == Stnotstop::STOPPED || status.start() == SstatusStart::START_DETECTED {
                    #[cfg(feature = "defmt")]
                    defmt::trace!("STOP or Repeated-start received");
                    break;
                }
            }

            match self.check_status() {
                Ok(()) => {}
                Err(IOError::Terminated) => return Ok(ReadStatus::EarlyStop(count)),
                Err(e) => return Err(e),
            }

            if self.info.regs().sstatus().read().txnotfull() == SstatusTxnotfull::FULL {
                // Wait until we have space in the FIFO.
                self.info
                    .wait_cell()
                    .wait_for(|| {
                        self.info.regs().sintset().write(|w| {
                            w.set_errwarn(true);
                            w.set_txsend(true);
                        });

                        let status = self.info.regs().sstatus().read();
                        status.errwarn() || status.txnotfull() == SstatusTxnotfull::NOT_FULL
                    })
                    .await
                    .map_err(|_| IOError::Other)?;

                match self.check_status() {
                    Ok(()) => {}
                    Err(IOError::Terminated) => return Ok(ReadStatus::EarlyStop(count)),
                    Err(e) => return Err(e),
                }
            }

            self.info.regs().swdatabe().write(|w| w.set_data(*last));
            count += 1;
        }

        Ok(ReadStatus::Complete(count))
    }

    async fn async_respond_to_write_internal<'a>(&'a mut self, buf: &'a mut [u8]) -> Result<usize, IOError> {
        let mut count = 0;

        for byte in buf.iter_mut() {
            // Check rxcount
            if self.info.regs().sdatactrl().read().rxcount() == 0 {
                // Wait until we have data in the FIFO.
                self.info
                    .wait_cell()
                    .wait_for(|| {
                        self.info.regs().sintset().write(|w| {
                            w.set_errwarn(true);
                            w.set_rxpend(true);
                        });

                        let status = self.info.regs().sstatus().read();
                        status.errwarn() || status.rx_pend()
                    })
                    .await
                    .map_err(|_| IOError::Other)?;

                match self.check_status() {
                    Ok(()) => {}
                    Err(IOError::Terminated) => return Ok(count),
                    Err(e) => return Err(e),
                }
            }

            *byte = self.info.regs().srdatab().read().data0();
            defmt::debug!("{:02x}", *byte);
            count += 1;

            // If we see a STOP or REPEATED START, break out
            let status = self.info.regs().sstatus().read();
            defmt::info!("{}", status);
            if status.stnotstop() == Stnotstop::STOPPED || status.start() == SstatusStart::START_DETECTED {
                #[cfg(feature = "defmt")]
                defmt::trace!("STOP or Repeated-start received");
                break;
            }
        }

        Ok(count)
    }
}

impl<'d> AsyncEngine for I3c<'d, Dma<'d>> {
    fn async_respond_to_read_internal<'a>(
        &'a mut self,
        _buf: &'a [u8],
    ) -> impl Future<Output = Result<ReadStatus, IOError>> + 'a {
        async move { todo!() }
    }

    fn async_respond_to_write_internal<'a>(
        &'a mut self,
        _buf: &'a mut [u8],
    ) -> impl Future<Output = Result<usize, IOError>> + 'a {
        async move { todo!() }
    }
}

// Public API: Async
#[allow(private_bounds)]
impl<'d, M: AsyncMode> I3c<'d, M>
where
    Self: AsyncEngine,
{
    /// Asynchronously transmit data to the I2C controller.
    ///
    /// This function sends the contents of the provided buffer to the I2C controller. It
    /// asynchronously waits until the data is transmitted or an error occurs.
    ///
    /// # Parameters
    ///
    /// - `buf`: The buffer containing the data to transmit.
    ///
    /// # Returns
    ///
    /// - `Ok(usize)` with the number of bytes transmitted.
    /// - `Err(IOError)` if an error occurs.
    pub fn async_respond_to_read<'a>(
        &'a mut self,
        buf: &'a [u8],
    ) -> impl Future<Output = Result<ReadStatus, IOError>> + 'a {
        <Self as AsyncEngine>::async_respond_to_read_internal(self, buf)
    }

    /// Asynchronously receive data from the I2C controller.
    ///
    /// This function receives data from the I2C controller into the provided buffer. It
    /// asynchronously waits until the buffer is filled or an error occurs.
    ///
    /// # Parameters
    ///
    /// - `buf`: The buffer to store the received data.
    ///
    /// # Returns
    ///
    /// - `Ok(usize)` with the number of bytes received.
    /// - `Err(IOError)` if an error occurs.
    pub fn async_respond_to_write<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<usize, IOError>> + 'a {
        <Self as AsyncEngine>::async_respond_to_write_internal(self, buf)
    }

    /// Asynchronously send an In-Band Interrupt (IBI) to the controller.
    ///
    /// Asserts an IBI request on the bus and waits for the controller to ACK it.
    /// The device must be configured with `Config::ibi_capable = true` (and optionally
    /// `Config::ibi_has_payload = true`) for this to be meaningful.
    ///
    /// If `payload` is non-empty, `payload[0]` is placed in `SCTRL.IBIDATA` as the
    /// mandatory data byte (requires `ibi_has_payload = true` / BCR[2]=1).
    ///
    /// # Returns
    ///
    /// - `Ok(())` once the controller ACKs the IBI.
    /// - `Err(IOError::IbiNacked)` if the controller permanently rejects the IBI.
    /// - `Err(IOError)` for bus errors.
    pub async fn async_send_ibi(&mut self, payload: &[u8]) -> Result<(), IOError> {
        self.clear_status();
        self.flush_fifos();

        // Set mandatory data byte if the caller provided one.
        self.info.regs().sctrl().modify(|w| {
            if let Some(&b) = payload.first() {
                w.set_ibidata(b);
            }
            w.set_event(SctrlEvent::IBI);
        });

        // Wait until the controller ACKs (EVDET = ACKED).
        // The hardware retries automatically on NACK so SSTATUS.event may fire several
        // times before reaching ACKED.  We clear the event flag each time so the next
        // NACK/ACK still generates an interrupt.
        self.info
            .wait_cell()
            .wait_for(|| {
                self.info.regs().sintset().write(|w| {
                    w.set_event(true);
                    w.set_errwarn(true);
                });

                let status = self.info.regs().sstatus().read();

                if status.errwarn() {
                    return true;
                }

                if status.event() {
                    // Clear the event flag so a future NACK→retry→ACK still fires.
                    self.info.regs().sstatus().write(|w| w.set_event(true));
                    return status.evdet() == Evdet::ACKED;
                }

                false
            })
            .await
            .map_err(|_| IOError::Other)?;

        // Release EVENT field back to normal.
        self.info
            .regs()
            .sctrl()
            .modify(|w| w.set_event(SctrlEvent::NORMAL_MODE));

        self.check_status()?;

        Ok(())
    }
}

impl<'d, M: Mode> Drop for I3c<'d, M> {
    fn drop(&mut self) {
        self._scl.set_as_disabled();
        self._sda.set_as_disabled();
    }
}

/// I3C interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let status = T::info().regs().sintmasked().read();
        T::PERF_INT_INCR();

        if status.0 != 0 {
            T::info().regs().sintclr().write(|w| w.0 = status.0);
            T::PERF_INT_WAKE_INCR();
            T::info().wait_cell().wake();
        }
    }
}
