//! eSPI driver.
//!
//! An instance of the eSPI driver.
//!
//! # Microchip canonical virtual wire mapping
//!
//! ## System Event Virtual Wires
//!
//! |VW Idx | VW reg | SRC_ID3      | SRC_ID2      | SRC_ID1   | SRC_ID0     |
//! |-------|--------|--------------|--------------|-----------|-------------|
//! | 2h    | MSVW00 | _Reserved_   | SLP_S5#      | SLP_S4#   | SLP_S3#     |
//! | 3h    | MSVW01 | _Reserved_   | OOB_RST_WARN | PLTRST#   | SUS_STAT#   |
//! | 4h    | SMVW00 | PME#         | WAKE#        | _Reserved_| OOB_RST_ACK |
//! | 5h    | SMVW01 | SLV_BOOT_STS | ERR_NONFATAL | ERR_FATAL | SLV_BT_DONE |
//! | 6h    | SMVW02 | HOST_RST_ACK | RCIN#        | SMI#      | SCI#        |
//! | 7h    | MSVW02 | _Reserved_   | NMIOUT#      | SMIOUT#   | HOS_RST_WARN|
//!
//! ## Platform specific virtual wi_Reserved_
//!
//! |VW Idx | VW reg | SRC_ID3      | SRC_ID2      | SRC_ID1   | SRC_ID0     |
//! |-------|--------|--------------|--------------|-----------|-------------|
//! | 40h   | SMVW03 | _Reserved_   | _Reserved_   | DNX_ACK   | SUS_ACK#    |
//! | 41h   | MSVW03 | SLP_A#       | _Reserved_   | SUS_PDNACK| SUS_WARN#   |
//! | 42h   | MSVW04 | _Reserved_   | _Reserved_   | SLP_WLAN# | SLP_LAN#    |
//! | 43h   | MSVW05 | generic      | generic      | generic   | generic     |
//! | 44h   | MSVW06 | generic      | generic      | generic   | generic     |
//! | 45h   | SMVW04 | generic      | generic      | generic   | generic     |
//! | 46h   | SMVW05 | generic      | generic      | generic   | generic     |
//! | 47h   | MSVW07 | _Reserved_   | _Reserved_   | _Reserved_| HOST_C10    |
//! | 4Ah   | MSVW08 | _Reserved_   | _Reserved_   | DNX_WARN  | _Reserved_  |
//! | 50h   | SMVW06 | ocb_3        | ocb_2        | ocb_1     | ocb_0       |
//! | 51h   | SMVW07 | gpio_7       | gpio_6       | gpio_5    | gpio_4      |
//! | 52h   | SMVW08 | gpio_11      | gpio_10      | gpio_9    | gpio_8      |

use core::future;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use cortex_m::interrupt::InterruptNumber;
use embassy_espi_driver::{oob, vwire, Driver, EspiError, Event};
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{Pin, SealedPin};
use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::{interrupt, pac};

/// Pin that can be used as ESPI_IO0.
pub type Io0Pin = crate::peripherals::GPIO70;
/// Pin that can be used as ESPI_IO1.
pub type Io1Pin = crate::peripherals::GPIO71;
/// Pin that can be used as ESPI_IO2.
pub type Io2Pin = crate::peripherals::GPIO72;
/// Pin that can be used as ESPI_IO3.
pub type Io3Pin = crate::peripherals::GPIO73;
/// Pin that can be used as ESPI_CS.
pub type CsPin = crate::peripherals::GPIO66;
/// Pin that can be used as ESPI_RST.
pub type RstPin = crate::peripherals::GPIO61;
/// Pin that can be used as ESPI_CLK.
pub type ClkPin = crate::peripherals::GPIO65;
/// Pin that can be used as ESPI_ALERT.
pub type AlertPin = crate::peripherals::GPIO63;

static VWIRE_PENDING: AtomicBool = AtomicBool::new(false);

/// Operating frequency.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Frequency {
    /// 20MHz (default).
    _20MHz,

    /// 25MHz
    _25MHz,

    /// 33MHz.
    _33MHz,

    /// 50MHz.
    _50MHz,

    /// 66MHz.
    _66MHz,
}

impl From<Frequency> for u8 {
    fn from(value: Frequency) -> u8 {
        match value {
            Frequency::_20MHz => 0,
            Frequency::_25MHz => 1,
            Frequency::_33MHz => 2,
            Frequency::_50MHz => 3,
            Frequency::_66MHz => 4,
        }
    }
}

/// eSPI VWire configuration.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VWConfig {
    /// Maximum number of VWires.
    ///
    /// Must be 6 < max_count < 64.
    pub max_count: u8,
}

/// eSPI configuration.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Operating frequency.
    pub frequency: Frequency,

    /// Peripheral channel configuration.
    pub peripheral: Option<()>,

    /// Virtual Wire channel configuration.
    pub vwire: Option<VWConfig>,

    /// OOB channel configuration.
    pub oob: Option<()>,

    /// Flash Access channel configuration.
    pub flash_access: Option<()>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: Frequency::_20MHz,
            peripheral: None,
            vwire: None,
            oob: None,
            flash_access: None,
        }
    }
}

const OOB_BUFLEN: usize = 256;

pub struct Espi<'p, T: Instance> {
    _peri: Peri<'p, T>,
    rxbuf: [u8; OOB_BUFLEN],
    txbuf: [u8; OOB_BUFLEN],
}

impl<'p, T: Instance> Espi<'p, T> {
    /// Create a new instance of Single eSPI without dedicated ALERTn.
    pub fn new_single(
        _peri: Peri<'p, T>,
        _io0: Peri<'p, Io0Pin>,
        _io1: Peri<'p, Io1Pin>,
        _cs: Peri<'p, CsPin>,
        _rst: Peri<'p, RstPin>,
        _clk: Peri<'p, ClkPin>,
        _irqs: impl Binding<T::PCInterrupt, PCInterruptHandler<T>>
            + Binding<T::BM1Interrupt, BM1InterruptHandler<T>>
            + Binding<T::BM2Interrupt, BM2InterruptHandler<T>>
            + Binding<T::LtrInterrupt, LtrInterruptHandler<T>>
            + Binding<T::OobUpInterrupt, OobUpInterruptHandler<T>>
            + Binding<T::OobDownInterrupt, OobDownInterruptHandler<T>>
            + Binding<T::FlashInterrupt, FlashInterruptHandler<T>>
            + Binding<T::VWireInterrupt, VWireInterruptHandler<T>>
            + Binding<T::ResetInterrupt, ResetInterruptHandler<T>>
            + Binding<T::GIRQ24Interrupt, GIRQ24InterruptHandler<T>>
            + Binding<T::GIRQ25Interrupt, GIRQ25InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            _peri, _io0, _io1, None, None, _cs, _rst, _clk, None, _irqs, false, config,
        )
    }

    /// Create a new instance of Dual eSPI without dedicated ALERTn.
    pub fn new_dual(
        _peri: Peri<'p, T>,
        _io0: Peri<'p, Io0Pin>,
        _io1: Peri<'p, Io1Pin>,
        _cs: Peri<'p, CsPin>,
        _rst: Peri<'p, RstPin>,
        _clk: Peri<'p, ClkPin>,
        _irqs: impl Binding<T::PCInterrupt, PCInterruptHandler<T>>
            + Binding<T::BM1Interrupt, BM1InterruptHandler<T>>
            + Binding<T::BM2Interrupt, BM2InterruptHandler<T>>
            + Binding<T::LtrInterrupt, LtrInterruptHandler<T>>
            + Binding<T::OobUpInterrupt, OobUpInterruptHandler<T>>
            + Binding<T::OobDownInterrupt, OobDownInterruptHandler<T>>
            + Binding<T::FlashInterrupt, FlashInterruptHandler<T>>
            + Binding<T::VWireInterrupt, VWireInterruptHandler<T>>
            + Binding<T::ResetInterrupt, ResetInterruptHandler<T>>
            + Binding<T::GIRQ24Interrupt, GIRQ24InterruptHandler<T>>
            + Binding<T::GIRQ25Interrupt, GIRQ25InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            _peri, _io0, _io1, None, None, _cs, _rst, _clk, None, _irqs, true, config,
        )
    }

    /// Create a new instance of Quad eSPI without dedicated ALERTn.
    pub fn new_quad(
        _peri: Peri<'p, T>,
        _io0: Peri<'p, Io0Pin>,
        _io1: Peri<'p, Io1Pin>,
        _io2: Peri<'p, Io2Pin>,
        _io3: Peri<'p, Io3Pin>,
        _cs: Peri<'p, CsPin>,
        _rst: Peri<'p, RstPin>,
        _clk: Peri<'p, ClkPin>,
        _irqs: impl Binding<T::PCInterrupt, PCInterruptHandler<T>>
            + Binding<T::BM1Interrupt, BM1InterruptHandler<T>>
            + Binding<T::BM2Interrupt, BM2InterruptHandler<T>>
            + Binding<T::LtrInterrupt, LtrInterruptHandler<T>>
            + Binding<T::OobUpInterrupt, OobUpInterruptHandler<T>>
            + Binding<T::OobDownInterrupt, OobDownInterruptHandler<T>>
            + Binding<T::FlashInterrupt, FlashInterruptHandler<T>>
            + Binding<T::VWireInterrupt, VWireInterruptHandler<T>>
            + Binding<T::ResetInterrupt, ResetInterruptHandler<T>>
            + Binding<T::GIRQ24Interrupt, GIRQ24InterruptHandler<T>>
            + Binding<T::GIRQ25Interrupt, GIRQ25InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            _peri,
            _io0,
            _io1,
            Some(_io2),
            Some(_io3),
            _cs,
            _rst,
            _clk,
            None,
            _irqs,
            false,
            config,
        )
    }

    /// Create a new instance of Single eSPI with dedicated ALERTn.
    pub fn new_single_with_alert(
        _peri: Peri<'p, T>,
        _io0: Peri<'p, Io0Pin>,
        _io1: Peri<'p, Io1Pin>,
        _io2: Peri<'p, Io2Pin>,
        _io3: Peri<'p, Io3Pin>,
        _cs: Peri<'p, CsPin>,
        _rst: Peri<'p, RstPin>,
        _clk: Peri<'p, ClkPin>,
        _alert: Peri<'p, AlertPin>,
        _irqs: impl Binding<T::PCInterrupt, PCInterruptHandler<T>>
            + Binding<T::BM1Interrupt, BM1InterruptHandler<T>>
            + Binding<T::BM2Interrupt, BM2InterruptHandler<T>>
            + Binding<T::LtrInterrupt, LtrInterruptHandler<T>>
            + Binding<T::OobUpInterrupt, OobUpInterruptHandler<T>>
            + Binding<T::OobDownInterrupt, OobDownInterruptHandler<T>>
            + Binding<T::FlashInterrupt, FlashInterruptHandler<T>>
            + Binding<T::VWireInterrupt, VWireInterruptHandler<T>>
            + Binding<T::ResetInterrupt, ResetInterruptHandler<T>>
            + Binding<T::GIRQ24Interrupt, GIRQ24InterruptHandler<T>>
            + Binding<T::GIRQ25Interrupt, GIRQ25InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            _peri,
            _io0,
            _io1,
            Some(_io2),
            Some(_io3),
            _cs,
            _rst,
            _clk,
            Some(_alert),
            _irqs,
            false,
            config,
        )
    }

    /// Create a new instance of Dual eSPI with dedicated ALERTn.
    pub fn new_dual_with_alert(
        _peri: Peri<'p, T>,
        _io0: Peri<'p, Io0Pin>,
        _io1: Peri<'p, Io1Pin>,
        _cs: Peri<'p, CsPin>,
        _rst: Peri<'p, RstPin>,
        _clk: Peri<'p, ClkPin>,
        _alert: Peri<'p, AlertPin>,
        _irqs: impl Binding<T::PCInterrupt, PCInterruptHandler<T>>
            + Binding<T::BM1Interrupt, BM1InterruptHandler<T>>
            + Binding<T::BM2Interrupt, BM2InterruptHandler<T>>
            + Binding<T::LtrInterrupt, LtrInterruptHandler<T>>
            + Binding<T::OobUpInterrupt, OobUpInterruptHandler<T>>
            + Binding<T::OobDownInterrupt, OobDownInterruptHandler<T>>
            + Binding<T::FlashInterrupt, FlashInterruptHandler<T>>
            + Binding<T::VWireInterrupt, VWireInterruptHandler<T>>
            + Binding<T::ResetInterrupt, ResetInterruptHandler<T>>
            + Binding<T::GIRQ24Interrupt, GIRQ24InterruptHandler<T>>
            + Binding<T::GIRQ25Interrupt, GIRQ25InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            _peri,
            _io0,
            _io1,
            None,
            None,
            _cs,
            _rst,
            _clk,
            Some(_alert),
            _irqs,
            true,
            config,
        )
    }

    /// Create a new instance of Quad eSPI with dedicated ALERTn.
    pub fn new_quad_with_alert(
        _peri: Peri<'p, T>,
        _io0: Peri<'p, Io0Pin>,
        _io1: Peri<'p, Io1Pin>,
        _io2: Peri<'p, Io2Pin>,
        _io3: Peri<'p, Io3Pin>,
        _cs: Peri<'p, CsPin>,
        _rst: Peri<'p, RstPin>,
        _clk: Peri<'p, ClkPin>,
        _alert: Peri<'p, AlertPin>,
        _irqs: impl Binding<T::PCInterrupt, PCInterruptHandler<T>>
            + Binding<T::BM1Interrupt, BM1InterruptHandler<T>>
            + Binding<T::BM2Interrupt, BM2InterruptHandler<T>>
            + Binding<T::LtrInterrupt, LtrInterruptHandler<T>>
            + Binding<T::OobUpInterrupt, OobUpInterruptHandler<T>>
            + Binding<T::OobDownInterrupt, OobDownInterruptHandler<T>>
            + Binding<T::FlashInterrupt, FlashInterruptHandler<T>>
            + Binding<T::VWireInterrupt, VWireInterruptHandler<T>>
            + Binding<T::ResetInterrupt, ResetInterruptHandler<T>>
            + Binding<T::GIRQ24Interrupt, GIRQ24InterruptHandler<T>>
            + Binding<T::GIRQ25Interrupt, GIRQ25InterruptHandler<T>>,
        config: Config,
    ) -> Self {
        Self::new_inner(
            _peri,
            _io0,
            _io1,
            Some(_io2),
            Some(_io3),
            _cs,
            _rst,
            _clk,
            Some(_alert),
            _irqs,
            false,
            config,
        )
    }

    fn new_inner(
        _peri: Peri<'p, T>,
        _io0: Peri<'p, Io0Pin>,
        _io1: Peri<'p, Io1Pin>,
        _io2: Option<Peri<'p, Io2Pin>>,
        _io3: Option<Peri<'p, Io3Pin>>,
        _cs: Peri<'p, CsPin>,
        _rst: Peri<'p, RstPin>,
        _clk: Peri<'p, ClkPin>,
        _alert: Option<Peri<'p, AlertPin>>,
        _irqs: impl Binding<T::PCInterrupt, PCInterruptHandler<T>>
            + Binding<T::BM1Interrupt, BM1InterruptHandler<T>>
            + Binding<T::BM2Interrupt, BM2InterruptHandler<T>>
            + Binding<T::LtrInterrupt, LtrInterruptHandler<T>>
            + Binding<T::OobUpInterrupt, OobUpInterruptHandler<T>>
            + Binding<T::OobDownInterrupt, OobDownInterruptHandler<T>>
            + Binding<T::FlashInterrupt, FlashInterruptHandler<T>>
            + Binding<T::VWireInterrupt, VWireInterruptHandler<T>>
            + Binding<T::ResetInterrupt, ResetInterruptHandler<T>>
            + Binding<T::GIRQ24Interrupt, GIRQ24InterruptHandler<T>>
            + Binding<T::GIRQ25Interrupt, GIRQ25InterruptHandler<T>>,
        is_dual: bool,
        config: Config,
    ) -> Self {
        let mut inst = Self {
            _peri,
            rxbuf: [0; OOB_BUFLEN],
            txbuf: [0; OOB_BUFLEN],
        };

        critical_section::with(|_| {
            crate::pac::PCR.slp_en_2().modify(|w| w.set_espi_slp_en(false));
            crate::pac::PCR.pwr_rst_ctrl().modify(|w| w.set_h_rst_sel(true));

            _io0.as_espi();
            _io1.as_espi();
            _cs.as_espi();
            _rst.as_espi();
            _clk.as_espi();

            // Configure maximum frequency and default to single I/O.
            T::io().glb_cap1().modify(|w| {
                w.set_io_mode(0);
                w.set_max_freq(config.frequency.into())
            });

            // Configure channels
            T::io().glb_cap0().modify(|w| {
                w.set_phl_chn(config.peripheral.is_some());
                w.set_vw_chn(config.vwire.is_some());
                w.set_oob_msg_chn(config.oob.is_some());
                w.set_fc_chn(config.flash_access.is_some());
            });

            // REVISIT: Make this configurable
            if let Some(vw) = config.vwire {
                T::io().pltrst_src().modify(|w| w.set_src(false));
                T::io().vw_cap().modify(|w| w.set_max_cnt(vw.max_count));
            }

            if is_dual {
                // Dual/Single
                T::io().glb_cap1().modify(|w| w.set_io_mode(1));
            }

            if let Some(io2) = _io2 {
                io2.as_espi();
            }

            if let Some(io3) = _io3 {
                io3.as_espi();

                // Quad/Dual/Single
                T::io().glb_cap1().modify(|w| w.set_io_mode(3));
            }

            if let Some(alert) = _alert {
                alert.as_espi();
            }

            T::io().activate().modify(|w| w.set_act(true));

            // Enable interrupt within GIRQ
            inst.clear_interrupts();
            inst.enable_interrupts();

            // TODO:
            //
            // - Clear any potentially pending IRQ events.
            // - Configure channel capabilities (payload size and vwire count)
            T::io().reset_sts().modify(|w| w.set_esp_intr(true));
            T::io().pc_status().modify(|w| w.set_chn_en_sts(true));

            inst.vwire_init();

            if config.oob.is_some() {
                inst.oob_init();
            }
        });

        inst.clear_interrupts();

        T::PCInterrupt::unpend();
        T::BM1Interrupt::unpend();
        T::BM2Interrupt::unpend();
        T::LtrInterrupt::unpend();
        T::OobUpInterrupt::unpend();
        T::OobDownInterrupt::unpend();
        T::FlashInterrupt::unpend();
        T::VWireInterrupt::unpend();
        T::ResetInterrupt::unpend();
        T::GIRQ24Interrupt::unpend();
        T::GIRQ25Interrupt::unpend();

        // Safety: _irqs ensures an interrupt handler is bound
        unsafe {
            T::PCInterrupt::enable();
            T::BM1Interrupt::enable();
            T::BM2Interrupt::enable();
            T::LtrInterrupt::enable();
            T::OobUpInterrupt::enable();
            T::OobDownInterrupt::enable();
            T::FlashInterrupt::enable();
            T::VWireInterrupt::enable();
            T::ResetInterrupt::enable();
            T::GIRQ24Interrupt::enable();
            T::GIRQ25Interrupt::enable();
        }

        inst
    }

    fn vwire_init(&mut self) {
        // Configure all VWires as rising and falling edge
        // triggers.
        T::msvw0().msvw00_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw0().msvw01_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw0().msvw02_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw0().msvw03_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw0().msvw04_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw0().msvw05_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw0().msvw06_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw1().msvw07_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw1().msvw08_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw1().msvw09_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });

        T::msvw1().msvw10_dw1().modify(|w| {
            w.set_src0irq_sel(0x0f);
            w.set_src1irq_sel(0x0f);
            w.set_src2irq_sel(0x0f);
            w.set_src3irq_sel(0x0f);
        });
    }

    fn oob_init(&mut self) {
        T::io()
            .oob_rx_addr_lsw()
            .write(|w| w.set_rx_buf(self.rxbuf.as_ptr() as *const u32 as u32));
        T::io().oob_rx_len().write(|w| w.set_buf((OOB_BUFLEN - 1) as u16));
        T::io().oob_rx_ctrl().write(|w| w.set_set_rx_avail(true));

        T::io()
            .oob_tx_addr_lsw()
            .write(|w| w.set_tx_buf(self.txbuf.as_ptr() as *const u32 as u32));
        T::io().oob_tx_len().write(|w| w.set_msg(8));
    }

    /// Calls `f` to check if we are ready or not.
    /// If not, `g` is called once(to eg enable the required interrupts).
    /// The waker will always be registered prior to calling `f`.
    async fn wait_on<F, U, G>(&mut self, mut f: F, mut g: G) -> U
    where
        F: FnMut(&mut Self) -> Poll<U>,
        G: FnMut(&mut Self),
    {
        future::poll_fn(|cx| {
            // Register prior to checking the condition
            T::waker().register(cx.waker());
            let r = f(self);

            if r.is_pending() {
                g(self);
            }
            r
        })
        .await
    }

    fn clear_interrupts(&self) {
        pac::ECIA.src19().write_value(0x0000_0fff);
        pac::ECIA.src24().write_value(0x0fff_ffff);
        pac::ECIA.src25().write_value(0x0000_ffff);
    }

    fn enable_interrupts(&self) {
        pac::ECIA.en_set19().write_value(0x0000_0fff);
        pac::ECIA.en_set24().write_value(0x0fff_ffff);
        pac::ECIA.en_set25().write_value(0x0000_ffff);

        let mut blk_en = pac::ECIA.blk_en_set().read().vtor_en_set();
        blk_en |= 1 << (8 + interrupt::GIRQ24.number()) | 1 << (8 + interrupt::GIRQ25.number());
        pac::ECIA.blk_en_set().write(|w| w.set_vtor_en_set(blk_en));
    }

    // fn disable_interrupts(&self) {
    //     pac::ECIA.en_clr19().write_value(0x0000_0fff);
    //     pac::ECIA.en_clr24().write_value(0x0fff_ffff);
    //     pac::ECIA.en_clr25().write_value(0x0000_ffff);
    // }
}

impl<'p, T: Instance> Driver<'p> for Espi<'p, T> {
    async fn listen(&mut self) -> Result<Event, EspiError> {
        self.wait_on(
            |me| {
                if T::io().reset_sts().read().esp_intr() {
                    T::io().reset_sts().write(|w| w.set_esp_intr(true));
                    if !T::io().reset_sts().read().espi_rst_pin_state() {
                        Poll::Ready(Ok(Event::Reset))
                    } else {
                        Poll::Pending
                    }
                } else if VWIRE_PENDING.load(Ordering::Relaxed) {
                    pac::ECIA.src24().write_value(0x0fff_ffff);
                    pac::ECIA.src25().write_value(0x0000_ffff);
                    VWIRE_PENDING.store(false, Ordering::Relaxed);
                    Poll::Ready(Ok(Event::VWire))
                } else if T::io().oob_tx_sts().read().chn_en_chng_sts() {
                    defmt::error!("===============> OOB CHANNEL ENABLED <===============");
                    T::io().oob_tx_sts().write(|w| w.set_chn_en_chng_sts(true));
                    me.oob_init();
                    T::io().oob_rx_ctrl().write(|w| w.set_set_rx_avail(true));
                    Poll::Pending
                } else if T::io().oob_rx_sts().read().done() {
                    T::io().oob_rx_sts().write(|w| w.set_done(true));
                    Poll::Ready(Ok(Event::Oob))
                } else {
                    pac::ECIA.en_set24().write_value(0x0fff_ffff);
                    pac::ECIA.en_set25().write_value(0x0000_ffff);
                    Poll::Pending
                }
            },
            |_| {
                T::io().reset_ien().write(|w| w.set_esp_rien(true));
                T::io().pc_ien().write(|w| w.set_en_chng_en(true));
                critical_section::with(|_| T::io().oob_rx_ien().modify(|w| w.set_rx_ien(true)));
                critical_section::with(|_| T::io().oob_tx_ien().modify(|w| w.set_chn_en_chng(true)));
                T::io().oob_rx_ctrl().write(|w| w.set_set_rx_avail(true));
            },
        )
        .await
    }
}

impl<'p, T: Instance> vwire::VWireChannel for Espi<'p, T> {
    fn get_vwire(&mut self, vwire: vwire::VWire) -> Result<(), embassy_espi_driver::EspiError> {
        match vwire {
            vwire::VWire::System(index, system) => match index {
                4 => {
                    let bit0 = system.wire0() == vwire::Level::High;
                    let bit1 = system.wire1() == vwire::Level::High;
                    let bit2 = system.wire2() == vwire::Level::High;
                    let bit3 = system.wire3() == vwire::Level::High;

                    critical_section::with(|_| {
                        T::smvw().smvw00_dw1().modify(|w| {
                            if system.valid0() == vwire::Valid::Valid {
                                w.set_src0(bit0);
                            }

                            if system.valid1() == vwire::Valid::Valid {
                                w.set_src1(bit1);
                            }

                            if system.valid2() == vwire::Valid::Valid {
                                w.set_src2(bit2);
                            }

                            if system.valid3() == vwire::Valid::Valid {
                                w.set_src3(bit3);
                            }
                        })
                    });

                    Ok(())
                }
                5 => {
                    let bit0 = system.wire0() == vwire::Level::High;
                    let bit1 = system.wire1() == vwire::Level::High;
                    let bit2 = system.wire2() == vwire::Level::High;
                    let bit3 = system.wire3() == vwire::Level::High;

                    critical_section::with(|_| {
                        T::smvw().smvw01_dw1().modify(|w| {
                            if system.valid0() == vwire::Valid::Valid {
                                w.set_src0(bit0);
                            }

                            if system.valid1() == vwire::Valid::Valid {
                                w.set_src1(bit1);
                            }

                            if system.valid2() == vwire::Valid::Valid {
                                w.set_src2(bit2);
                            }

                            if system.valid3() == vwire::Valid::Valid {
                                w.set_src3(bit3);
                            }
                        })
                    });

                    Ok(())
                }
                6 => {
                    let bit0 = system.wire0() == vwire::Level::High;
                    let bit1 = system.wire1() == vwire::Level::High;
                    let bit2 = system.wire2() == vwire::Level::High;
                    let bit3 = system.wire3() == vwire::Level::High;

                    critical_section::with(|_| {
                        T::smvw().smvw02_dw1().modify(|w| {
                            if system.valid0() == vwire::Valid::Valid {
                                w.set_src0(bit0);
                            }

                            if system.valid1() == vwire::Valid::Valid {
                                w.set_src1(bit1);
                            }

                            if system.valid2() == vwire::Valid::Valid {
                                w.set_src2(bit2);
                            }

                            if system.valid3() == vwire::Valid::Valid {
                                w.set_src3(bit3);
                            }
                        })
                    });

                    Ok(())
                }
                _ => Err(embassy_espi_driver::EspiError::UnsupportedVWire),
            },
            _ => Err(embassy_espi_driver::EspiError::UnsupportedVWire),
        }
    }

    fn put_vwire(&mut self, index: u8) -> Result<vwire::VWire, embassy_espi_driver::EspiError> {
        // microchip ignores the valid bit. Wires are always valid.
        match index {
            2 => {
                let bit0 = (T::msvw0().msvw00_dw2().read().src0() as u8) << 0 | 1 << 4;
                let bit1 = (T::msvw0().msvw00_dw2().read().src1() as u8) << 1 | 1 << 5;
                let bit2 = (T::msvw0().msvw00_dw2().read().src2() as u8) << 2 | 1 << 6;
                let bit3 = (T::msvw0().msvw00_dw2().read().src3() as u8) << 3 | 1 << 7;
                let system = vwire::System::from_bits(bit0 | bit1 | bit2 | bit3);
                Ok(vwire::VWire::System(index, system))
            }
            3 => {
                let bit0 = (T::msvw0().msvw01_dw2().read().src0() as u8) << 0 | 1 << 4;
                let bit1 = (T::msvw0().msvw01_dw2().read().src1() as u8) << 1 | 1 << 5;
                let bit2 = (T::msvw0().msvw01_dw2().read().src2() as u8) << 2 | 1 << 6;
                let bit3 = (T::msvw0().msvw01_dw2().read().src3() as u8) << 3 | 1 << 7;
                let system = vwire::System::from_bits(bit0 | bit1 | bit2 | bit3);
                Ok(vwire::VWire::System(index, system))
            }
            7 => {
                let bit0 = (T::msvw0().msvw02_dw2().read().src0() as u8) << 0 | 1 << 4;
                let bit1 = (T::msvw0().msvw02_dw2().read().src1() as u8) << 1 | 1 << 5;
                let bit2 = (T::msvw0().msvw02_dw2().read().src2() as u8) << 2 | 1 << 6;
                let bit3 = (T::msvw0().msvw02_dw2().read().src3() as u8) << 3 | 1 << 7;
                let system = vwire::System::from_bits(bit0 | bit1 | bit2 | bit3);
                Ok(vwire::VWire::System(index, system))
            }
            _ => return Err(embassy_espi_driver::EspiError::UnsupportedVWire),
        }
    }
}

impl<'p, T: Instance> oob::OobChannel for Espi<'p, T> {
    async fn oob_receive(&mut self, buf: &mut [u8]) -> Result<usize, embassy_espi_driver::EspiError> {
        let rxlen = T::io().oob_rx_len().read().msg() as usize;

        if rxlen > OOB_BUFLEN {
            Err(embassy_espi_driver::EspiError::DataSize(rxlen as u32))
        } else if T::io().oob_rx_sts().read().int_bus_err_sts() || T::io().oob_rx_sts().read().ovrun_sts() {
            Err(embassy_espi_driver::EspiError::Other)
        } else {
            buf.copy_from_slice(&self.rxbuf);
            self.rxbuf.fill(0x00);
            Ok(rxlen)
        }
    }

    async fn oob_send(&mut self, buf: &[u8]) -> Result<usize, embassy_espi_driver::EspiError> {
        if buf.len() > OOB_BUFLEN {
            Err(embassy_espi_driver::EspiError::DataSize(self.txbuf.len() as u32))
        } else if !T::io().oob_tx_sts().read().chn_en_img() || !T::io().oob_tx_sts().read().tx_busy() {
            Err(embassy_espi_driver::EspiError::Other)
        } else {
            self.txbuf.copy_from_slice(buf);

            // packet length - 3 bytes for the header?
            T::io().oob_tx_len().write(|w| w.set_msg(buf.len() as u16));

            critical_section::with(|_| T::io().oob_tx_ctrl().modify(|w| w.set_tx_strt(true)));

            self.wait_on(
                |_| {
                    if T::io().oob_tx_sts().read().done() {
                        T::io().oob_tx_sts().write(|w| w.set_done(true));
                        Poll::Ready(Ok(buf.len()))
                    } else if T::io().oob_tx_sts().read().bad_req() {
                        T::io().oob_tx_sts().write(|w| w.set_bad_req(true));
                        Poll::Ready(Err(embassy_espi_driver::EspiError::UnsupportedOob))
                    } else {
                        Poll::Pending
                    }
                },
                |_| {
                    critical_section::with(|_| {
                        T::io().oob_tx_ien().write(|w| {
                            w.set_done(true);
                            w.set_chn_en_chng(true);
                        })
                    });
                },
            )
            .await
        }
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct PCInterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::PCInterrupt> for PCInterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
        defmt::trace!("Peripheral Channel interrupt");

        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct BM1InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::BM1Interrupt> for BM1InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
        defmt::trace!("BM1 interrupt");

        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct BM2InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::BM2Interrupt> for BM2InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
        defmt::trace!("BM2 interrupt");

        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct LtrInterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::LtrInterrupt> for LtrInterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
        defmt::trace!("LTR interrupt");

        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct OobUpInterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::OobUpInterrupt> for OobUpInterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
        defmt::trace!("OOB Up interrupt: {}", T::io().oob_tx_sts().read());

        T::io().oob_tx_ien().modify(|w| {
            w.set_done(false);
            w.set_chn_en_chng(false);
        });

        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct OobDownInterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::OobDownInterrupt> for OobDownInterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
        defmt::trace!("OOB Down interrupt");

        T::io().oob_rx_ien().modify(|w| w.set_rx_ien(false));
        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct FlashInterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::FlashInterrupt> for FlashInterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
        defmt::trace!("Flash channel interrupt");

        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct VWireInterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> VWireInterruptHandler<T> {
    fn clear() {
        pac::ECIA.src19().write_value(1 << 8);
    }

    fn disable() {
        pac::ECIA.en_clr19().write_value(1 << 8);
    }
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::VWireInterrupt> for VWireInterruptHandler<T> {
    unsafe fn on_interrupt() {
        // TODO
        defmt::trace!("VWIRE channel enable interrupt");

        if T::io().vw_en_sts().read().chn_en_sts() {
            T::io().vw_ready().modify(|w| w.set_chn(true));
            Self::clear();
            Self::disable();
        }
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct ResetInterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> ResetInterruptHandler<T> {
    fn clear() {
        pac::ECIA.src19().write_value(1 << 7);
    }
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::ResetInterrupt> for ResetInterruptHandler<T> {
    unsafe fn on_interrupt() {
        defmt::trace!(
            "RESET INTERRUPT: Pin state: {}",
            T::io().reset_sts().read().espi_rst_pin_state()
        );

        // Disable Reset Interrupt
        T::io().reset_ien().write(|w| w.set_esp_rien(false));

        // Enable OOB channel
        T::oob_init();

        Self::clear();

        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct GIRQ24InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::GIRQ24Interrupt> for GIRQ24InterruptHandler<T> {
    unsafe fn on_interrupt() {
        pac::ECIA.src24().write_value(0x0fff_ffff);
        VWIRE_PENDING.store(true, Ordering::Relaxed);
        T::waker().wake()
    }
}

/// The interrupt handler for the [Espi] controller.
pub struct GIRQ25InterruptHandler<T> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> crate::interrupt::typelevel::Handler<T::GIRQ25Interrupt> for GIRQ25InterruptHandler<T> {
    unsafe fn on_interrupt() {
        pac::ECIA.en_clr25().write_value(0x0000_ffff);
        VWIRE_PENDING.store(true, Ordering::Relaxed);
        T::waker().wake()
    }
}

mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait SealedInstance {
        fn waker() -> &'static AtomicWaker;
        fn io() -> crate::pac::espi_io::EspiIo;
        fn msvw0() -> crate::pac::espi_msvw00_06::EspiMsvw0006;
        fn msvw1() -> crate::pac::espi_msvw07_10::EspiMsvw0710;
        fn smvw() -> crate::pac::espi_smvw00_10::EspiSmvw0010;
    }
}

/// A marker trait implemented for all eSPI instances.
pub trait Instance: sealed::SealedInstance + PeripheralType + 'static + Send {
    /// Peripheral Channel interrupt.
    type PCInterrupt: crate::interrupt::typelevel::Interrupt;

    /// Bus mastering channel 1 interrupt.
    type BM1Interrupt: crate::interrupt::typelevel::Interrupt;

    /// Bus mastering channel 2 interrupt.
    type BM2Interrupt: crate::interrupt::typelevel::Interrupt;

    /// LTR interrupt.
    type LtrInterrupt: crate::interrupt::typelevel::Interrupt;

    /// OOB up interrupt.
    type OobUpInterrupt: crate::interrupt::typelevel::Interrupt;

    /// OOB down interrupt.
    type OobDownInterrupt: crate::interrupt::typelevel::Interrupt;

    /// Flash channel interrupt.
    type FlashInterrupt: crate::interrupt::typelevel::Interrupt;

    /// VWire interrupt.
    type VWireInterrupt: crate::interrupt::typelevel::Interrupt;

    /// Reset interrupt.
    type ResetInterrupt: crate::interrupt::typelevel::Interrupt;

    /// GIRQ24 interrupt.
    type GIRQ24Interrupt: crate::interrupt::typelevel::Interrupt;

    /// GIRQ25 interrupt.
    type GIRQ25Interrupt: crate::interrupt::typelevel::Interrupt;

    // /// EC completion interrupt.
    // type EcCmpltnInterrupt: crate::interrupt::typelevel::Interrupt;

    // /// Error interrupt.
    // type ErrorInterrupt: crate::interrupt::typelevel::Interrupt;
}

/// A marker trait implemented by all pins usable as eSPI.
pub trait EspiPin: Pin + PeripheralType {
    /// Configure the pin function for eSPI usage.
    fn as_espi(&self);
}

// macro_rules! vwire {
//     ($signal:ident, $src:ident, $set:ident, $result:ident, $clr:ident, $bit:expr) => {
//         VWireInfo {
//             signal: VWireSignal::$signal,
//             src: crate::pac::ECIA.$src(),
//             set: crate::pac::ECIA.$set(),
//             result: crate::pac::ECIA.$result(),
//             clr: crate::pac::ECIA.$clr(),
//             bit: $bit,
//         }
//     };
// }

macro_rules! impl_instance {
    ($peri:ident, $pc:ident, $bm1:ident, $bm2:ident, $ltr:ident, $oobup:ident, $oobdn:ident, $flash:ident, $vw:ident, $rst:ident) => {
        impl sealed::SealedInstance for crate::peripherals::$peri {
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }

            fn io() -> crate::pac::espi_io::EspiIo {
                pac::ESPI_IO
            }

            fn msvw0() -> crate::pac::espi_msvw00_06::EspiMsvw0006 {
                pac::ESPI_MSVW00_06
            }

            fn msvw1() -> crate::pac::espi_msvw07_10::EspiMsvw0710 {
                pac::ESPI_MSVW07_10
            }

            fn smvw() -> crate::pac::espi_smvw00_10::EspiSmvw0010 {
                pac::ESPI_SMVW00_10
            }
        }

        impl Instance for crate::peripherals::$peri {
            type PCInterrupt = crate::interrupt::typelevel::$pc;
            type BM1Interrupt = crate::interrupt::typelevel::$bm1;
            type BM2Interrupt = crate::interrupt::typelevel::$bm2;
            type LtrInterrupt = crate::interrupt::typelevel::$ltr;
            type OobUpInterrupt = crate::interrupt::typelevel::$oobup;
            type OobDownInterrupt = crate::interrupt::typelevel::$oobdn;
            type FlashInterrupt = crate::interrupt::typelevel::$flash;
            type VWireInterrupt = crate::interrupt::typelevel::$vw;
            type ResetInterrupt = crate::interrupt::typelevel::$rst;
            type GIRQ24Interrupt = crate::interrupt::typelevel::GIRQ24;
            type GIRQ25Interrupt = crate::interrupt::typelevel::GIRQ25;
        }
    };
}

macro_rules! impl_pin {
    ($peri:ident, $($pin:ident),*) => {
	$(
            impl EspiPin for crate::peripherals::$pin {
		#[inline(always)]
		fn as_espi(&self) {
		    critical_section::with(|_| {
			self.regs().ctrl1.modify(|w| {
			    w.set_mux_ctrl(crate::pac::Function::F1);
                            w.set_inp_dis(false);
                            w.set_inp(true);
                            w.set_out_sel(crate::pac::Sel::GROUP);
                            // w.set_pu_pd(crate::pac::Pull::DOWN);
			})
		    });
		}
	    }
	)*
    };
}

// REVISIT: There are a total of (so far) 12 different interrupt
// signals being tied to all 6 "pieces" of this controller.
//
// All 12 signals are needed for proper functionality on the various
// channels. The result here is that user will have to bind
// 12-different interrupt handlers in order to use this driver.
//
// For now, this suffices. If we even come up with a method to
// simplify for the user, we will do so. Meanwhile, KISS.
impl_instance!(
    ESPI,
    INTR_PC,
    INTR_BM1,
    INTR_BM2,
    INTR_LTR,
    INTR_OOB_UP,
    INTR_OOB_DOWN,
    INTR_FLASH,
    ESPI_VWIRE,
    ESPI_RESET
);
impl_pin!(ESPI, GPIO61, GPIO63, GPIO65, GPIO66, GPIO70, GPIO71, GPIO72, GPIO73);
