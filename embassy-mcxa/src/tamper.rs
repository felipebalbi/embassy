//! Tamper Detection (TDET) driver.

use core::marker::PhantomData;

use embassy_hal_internal::Peri;
use embassy_hal_internal::interrupt::InterruptExt;
use mcxa_pac::tdet0::cr::{Distam, Tfsr};
use mcxa_pac::tdet0::pgfr::{Gfp, Tpe, Tps, Tpsf, Tpsw};
use mcxa_pac::tdet0::ppr::{Tpp0, Tpp1, Tpp2, Tpp3, Tpp4, Tpp5};
use mcxa_pac::tdet0::ter::{Tie4, Tie5, Tie6, Tie7, Tie9, Tie10, Tpe0, Tpe1, Tpe2, Tpe3, Tpe4, Tpe5};

use crate::gpio::{AnyPin, GpioPin, SealedPin};
use crate::peripherals::TDET0;
use crate::{interrupt, pac};

/// Tamper driver configuration
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Initial value for the TDET prescaler 15-bit value.
    pub prescaler: u16,

    /// Disable prescaler after tamper detection.
    pub disable_on_tamper: DisableOnTamper,

    /// Force system reset on tamper detection?
    pub reset_on_tamper: ResetOnTamper,

    /// Enable VDD_CORE Glitch detection
    pub vdd_core_glitch_config: OnChipTamperInputConfig,

    /// Enable system low-voltage detecion
    pub system_low_voltage_config: OnChipTamperInputConfig,

    /// Enable system high-voltage detection
    pub system_high_voltage_config: OnChipTamperInputConfig,

    /// Enable code watchdog reset request detection
    pub cdog_config: OnChipTamperInputConfig,

    /// Enable MBC access violation
    pub mbc_config: OnChipTamperInputConfig,

    /// Enable RAM zeroize failure detection
    pub ram_zeroize_config: OnChipTamperInputConfig,

    /// External pin0 config
    pub ext0_config: Option<ExternalPinTamperConfig>,

    /// External pin1 config
    pub ext1_config: Option<ExternalPinTamperConfig>,

    /// External pin2 config
    pub ext2_config: Option<ExternalPinTamperConfig>,

    /// External pin3 config
    pub ext3_config: Option<ExternalPinTamperConfig>,

    /// External pin4 config
    pub ext4_config: Option<ExternalPinTamperConfig>,

    /// External pin5 config
    pub ext5_config: Option<ExternalPinTamperConfig>,
}

/// External pin tamper configuration
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ExternalPinTamperConfig {
    /// Pin polarity
    pub polarity: Polarity,

    /// Pull-up/pull-down configuration
    pub pull: Pull,

    /// Sample frequency
    pub sample_frequency: SampleFrequency,

    /// Sample width
    pub sample_width: SampleWidth,

    /// Glitch filter configuration
    pub glitch_filter_config: Option<GlitchFilterConfig>,
}

/// On-chip tamper input configuration
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct OnChipTamperInputConfig {
    /// Enable/disable tamper detection
    pub enable: Enable,
}

/// Glitch filter configuration
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct GlitchFilterConfig {
    /// Glitch filter prescaler
    pub prescaler: GlitchFilterPrescaler,

    /// Glitch filter width.
    ///
    /// Configures the number of clock edges during which the input
    /// must remain stable in order to pass through the glitch filter.
    ///
    /// Valid values are between 2 and 128 clock edges. Values outside
    /// this range will be clamped accordingly.
    pub width: u8,
}

/// Glitch filter prescaler
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum GlitchFilterPrescaler {
    /// 256Hz
    #[default]
    _256Hz,

    /// 16384Hz
    _16384Hz,
}

impl From<GlitchFilterPrescaler> for Gfp {
    fn from(value: GlitchFilterPrescaler) -> Self {
        // REVISIT: which is wrong? Documentation or SVD?
        match value {
            GlitchFilterPrescaler::_256Hz => Self::Freq512Hz,
            GlitchFilterPrescaler::_16384Hz => Self::Freq32Khz,
        }
    }
}

/// External pin pull configuration
#[derive(Copy, Clone, Default, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Pull {
    #[default]
    Disabled,
    Up,
    Down,
}

impl From<Pull> for Tpe {
    fn from(value: Pull) -> Self {
        match value {
            Pull::Disabled => Self::Disable,
            _ => Self::Enable,
        }
    }
}

impl From<Pull> for Tps {
    fn from(value: Pull) -> Self {
        match value {
            Pull::Up => Self::Assert,
            _ => Self::Negate,
        }
    }
}

/// Pin polarity
#[derive(Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Polarity {
    /// Active-high
    #[default]
    ActiveHigh,

    /// Active-low
    ActiveLow,
}

/// Sample frequency
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SampleFrequency {
    /// Every 8 cycles
    #[default]
    _8Cycles,

    /// Every 32 cycles
    _32Cycles,

    /// Every 128 cycles
    _128Cycles,

    /// Every 512 cycles
    _512Cycles,
}

impl From<SampleFrequency> for Tpsf {
    fn from(value: SampleFrequency) -> Self {
        match value {
            SampleFrequency::_8Cycles => Self::Cycles8,
            SampleFrequency::_32Cycles => Self::Cycles32,
            SampleFrequency::_128Cycles => Self::Cycles128,
            SampleFrequency::_512Cycles => Self::Cycles512,
        }
    }
}

/// Sample width
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SampleWidth {
    /// Continuous monitoring, pin sampling disabled
    #[default]
    Continuous,

    /// 2 cycles for pull enable and 1 cycle for input buffer enable
    _2x1,

    /// 4 cycles for pull enable and 2 cycles for input buffer enable
    _4x2,

    /// 8 cycles for pull enable and 4 cycles for input buffer enable
    _8x4,
}

impl From<SampleWidth> for Tpsw {
    fn from(value: SampleWidth) -> Self {
        match value {
            SampleWidth::Continuous => Self::Disable,
            SampleWidth::_2x1 => Self::Cycles2,
            SampleWidth::_4x2 => Self::Cycles4,
            SampleWidth::_8x4 => Self::Cycles8,
        }
    }
}

/// Enable tamper detection
#[derive(Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Enable {
    /// No
    #[default]
    No,

    /// Yes
    Yes,
}

/// Reset on tamper detection
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ResetOnTamper {
    /// No
    #[default]
    No,

    /// Yes
    Yes,
}

impl From<ResetOnTamper> for Tfsr {
    fn from(value: ResetOnTamper) -> Self {
        match value {
            ResetOnTamper::No => Self::NoReset,
            ResetOnTamper::Yes => Self::Reset,
        }
    }
}

/// Disable prescaler on tamper detection
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum DisableOnTamper {
    /// No
    #[default]
    No,

    /// Yes
    Yes,
}

impl From<DisableOnTamper> for Distam {
    fn from(value: DisableOnTamper) -> Self {
        match value {
            DisableOnTamper::No => Self::NoEffect,
            DisableOnTamper::Yes => Self::AutoDis,
        }
    }
}

/// Tamper detection driver.
pub struct Tdet<'d> {
    info: &'static Info,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> Tdet<'d> {
    fn new_inner(
        _peri: Peri<'d, TDET0>,
        _ext0: Option<Peri<'d, AnyPin>>,
        _ext1: Option<Peri<'d, AnyPin>>,
        _ext2: Option<Peri<'d, AnyPin>>,
        _ext3: Option<Peri<'d, AnyPin>>,
        _ext4: Option<Peri<'d, AnyPin>>,
        _ext5: Option<Peri<'d, AnyPin>>,
        config: Config,
    ) -> Self {
        let info = info();

        let inst = Self {
            info,
            _phantom: PhantomData,
        };

        inst.configure(config);

        crate::pac::Interrupt::TDET.unpend();
        unsafe {
            crate::pac::Interrupt::TDET.enable();
        }

        inst
    }

    /// Create a new instance of the Tamper Detection driver without
    /// external tamper inputs.
    pub fn new_no_ext_tamper(_peri: Peri<'d, TDET0>, config: Config) -> Self {
        Self::new_inner(_peri, None, None, None, None, None, None, config)
    }

    /// Create a new instance of the Tamper Detection driver with one
    /// external tamper input.
    pub fn new_with_one_ext_tamper(
        _peri: Peri<'d, TDET0>,
        _ext0: Peri<'d, impl TamperPin<TDET0>>,
        config: Config,
    ) -> Self {
        _ext0.mux();

        let ext0 = _ext0.into();

        Self::new_inner(_peri, Some(ext0), None, None, None, None, None, config)
    }

    /// Create a new instance of the Tamper Detection driver with two
    /// external tamper input.
    pub fn new_with_two_ext_tampers(
        _peri: Peri<'d, TDET0>,
        _ext0: Peri<'d, impl TamperPin<TDET0>>,
        _ext1: Peri<'d, impl TamperPin<TDET0>>,
        config: Config,
    ) -> Self {
        _ext0.mux();
        _ext1.mux();

        let ext0 = _ext0.into();
        let ext1 = _ext1.into();

        Self::new_inner(_peri, Some(ext0), Some(ext1), None, None, None, None, config)
    }

    /// Create a new instance of the Tamper Detection driver with three
    /// external tamper input.
    pub fn new_with_three_ext_tampers(
        _peri: Peri<'d, TDET0>,
        _ext0: Peri<'d, impl TamperPin<TDET0>>,
        _ext1: Peri<'d, impl TamperPin<TDET0>>,
        _ext2: Peri<'d, impl TamperPin<TDET0>>,
        config: Config,
    ) -> Self {
        _ext0.mux();
        _ext1.mux();
        _ext2.mux();

        let ext0 = _ext0.into();
        let ext1 = _ext1.into();
        let ext2 = _ext2.into();

        Self::new_inner(_peri, Some(ext0), Some(ext1), Some(ext2), None, None, None, config)
    }

    /// Create a new instance of the Tamper Detection driver with four
    /// external tamper input.
    pub fn new_with_four_ext_tampers(
        _peri: Peri<'d, TDET0>,
        _ext0: Peri<'d, impl TamperPin<TDET0>>,
        _ext1: Peri<'d, impl TamperPin<TDET0>>,
        _ext2: Peri<'d, impl TamperPin<TDET0>>,
        _ext3: Peri<'d, impl TamperPin<TDET0>>,
        config: Config,
    ) -> Self {
        _ext0.mux();
        _ext1.mux();
        _ext2.mux();
        _ext3.mux();

        let ext0 = _ext0.into();
        let ext1 = _ext1.into();
        let ext2 = _ext2.into();
        let ext3 = _ext3.into();

        Self::new_inner(
            _peri,
            Some(ext0),
            Some(ext1),
            Some(ext2),
            Some(ext3),
            None,
            None,
            config,
        )
    }

    /// Create a new instance of the Tamper Detection driver with five
    /// external tamper input.
    pub fn new_with_five_ext_tampers(
        _peri: Peri<'d, TDET0>,
        _ext0: Peri<'d, impl TamperPin<TDET0>>,
        _ext1: Peri<'d, impl TamperPin<TDET0>>,
        _ext2: Peri<'d, impl TamperPin<TDET0>>,
        _ext3: Peri<'d, impl TamperPin<TDET0>>,
        _ext4: Peri<'d, impl TamperPin<TDET0>>,
        config: Config,
    ) -> Self {
        _ext0.mux();
        _ext1.mux();
        _ext2.mux();
        _ext3.mux();
        _ext4.mux();

        let ext0 = _ext0.into();
        let ext1 = _ext1.into();
        let ext2 = _ext2.into();
        let ext3 = _ext3.into();
        let ext4 = _ext4.into();

        Self::new_inner(
            _peri,
            Some(ext0),
            Some(ext1),
            Some(ext2),
            Some(ext3),
            Some(ext4),
            None,
            config,
        )
    }

    /// Create a new instance of the Tamper Detection driver with six
    /// external tamper input.
    pub fn new_with_six_ext_tampers(
        _peri: Peri<'d, TDET0>,
        _ext0: Peri<'d, impl TamperPin<TDET0>>,
        _ext1: Peri<'d, impl TamperPin<TDET0>>,
        _ext2: Peri<'d, impl TamperPin<TDET0>>,
        _ext3: Peri<'d, impl TamperPin<TDET0>>,
        _ext4: Peri<'d, impl TamperPin<TDET0>>,
        _ext5: Peri<'d, impl TamperPin<TDET0>>,
        config: Config,
    ) -> Self {
        _ext0.mux();
        _ext1.mux();
        _ext2.mux();
        _ext3.mux();
        _ext4.mux();
        _ext5.mux();

        let ext0 = _ext0.into();
        let ext1 = _ext1.into();
        let ext2 = _ext2.into();
        let ext3 = _ext3.into();
        let ext4 = _ext4.into();
        let ext5 = _ext5.into();

        Self::new_inner(
            _peri,
            Some(ext0),
            Some(ext1),
            Some(ext2),
            Some(ext3),
            Some(ext4),
            Some(ext5),
            config,
        )
    }

    fn configure(&self, config: Config) {
        self.init();
        self.reset();
        self.clear_flags();

        // Enable requested events
        self.info.regs().ter().write(|w| {
            w.tie4()
                .variant(if config.vdd_core_glitch_config.enable == Enable::Yes {
                    Tie4::Enable
                } else {
                    Tie4::Disable
                })
                .tie5()
                .variant(if config.system_low_voltage_config.enable == Enable::Yes {
                    Tie5::Enable
                } else {
                    Tie5::Disable
                })
                .tie6()
                .variant(if config.system_high_voltage_config.enable == Enable::Yes {
                    Tie6::Enable
                } else {
                    Tie6::Disable
                })
                .tie7()
                .variant(if config.cdog_config.enable == Enable::Yes {
                    Tie7::Enable
                } else {
                    Tie7::Disable
                })
                .tie9()
                .variant(if config.mbc_config.enable == Enable::Yes {
                    Tie9::Enable
                } else {
                    Tie9::Disable
                })
                .tie10()
                .variant(if config.ram_zeroize_config.enable == Enable::Yes {
                    Tie10::Enable
                } else {
                    Tie10::Disable
                })
                .tpe0()
                .variant(if config.ext0_config.is_some() {
                    Tpe0::Enable
                } else {
                    Tpe0::Disable
                })
                .tpe1()
                .variant(if config.ext1_config.is_some() {
                    Tpe1::Enable
                } else {
                    Tpe1::Disable
                })
                .tpe2()
                .variant(if config.ext2_config.is_some() {
                    Tpe2::Enable
                } else {
                    Tpe2::Disable
                })
                .tpe3()
                .variant(if config.ext3_config.is_some() {
                    Tpe3::Enable
                } else {
                    Tpe3::Disable
                })
                .tpe4()
                .variant(if config.ext4_config.is_some() {
                    Tpe4::Enable
                } else {
                    Tpe4::Disable
                })
                .tpe5()
                .variant(if config.ext5_config.is_some() {
                    Tpe5::Enable
                } else {
                    Tpe5::Disable
                })
        });

        // Configure pin polarity
        self.info.regs().ppr().write(|w| {
            w.tpp0()
                .variant(
                    if config
                        .ext0_config
                        .as_ref()
                        .map(|c| c.polarity)
                        .unwrap_or(Polarity::ActiveHigh)
                        == Polarity::ActiveHigh
                    {
                        Tpp0::NoInvert
                    } else {
                        Tpp0::Invert
                    },
                )
                .tpp1()
                .variant(
                    if config
                        .ext1_config
                        .as_ref()
                        .map(|c| c.polarity)
                        .unwrap_or(Polarity::ActiveHigh)
                        == Polarity::ActiveHigh
                    {
                        Tpp1::NoInvert
                    } else {
                        Tpp1::Invert
                    },
                )
                .tpp2()
                .variant(
                    if config
                        .ext2_config
                        .as_ref()
                        .map(|c| c.polarity)
                        .unwrap_or(Polarity::ActiveHigh)
                        == Polarity::ActiveHigh
                    {
                        Tpp2::NoInvert
                    } else {
                        Tpp2::Invert
                    },
                )
                .tpp3()
                .variant(
                    if config
                        .ext3_config
                        .as_ref()
                        .map(|c| c.polarity)
                        .unwrap_or(Polarity::ActiveHigh)
                        == Polarity::ActiveHigh
                    {
                        Tpp3::NoInvert
                    } else {
                        Tpp3::Invert
                    },
                )
                .tpp4()
                .variant(
                    if config
                        .ext4_config
                        .as_ref()
                        .map(|c| c.polarity)
                        .unwrap_or(Polarity::ActiveHigh)
                        == Polarity::ActiveHigh
                    {
                        Tpp4::NoInvert
                    } else {
                        Tpp4::Invert
                    },
                )
                .tpp5()
                .variant(
                    if config
                        .ext5_config
                        .as_ref()
                        .map(|c| c.polarity)
                        .unwrap_or(Polarity::ActiveHigh)
                        == Polarity::ActiveHigh
                    {
                        Tpp5::NoInvert
                    } else {
                        Tpp5::Invert
                    },
                )
        });

        // Configure glitch filter, sample width, sample frequency,
        // and pin polarity for each of the enabled pins.
        if let Some(ext_config) = config.ext0_config {
            // GFW is calculated as (GFW + 1) * 2
            let mut gfw = ext_config
                .glitch_filter_config
                .as_ref()
                .map(|e| e.width)
                .unwrap_or(2)
                .clamp(2, 128);

            gfw /= 2;
            gfw -= 1;

            self.info.regs().pgfr(0).write(|w| {
                unsafe { w.gfw().bits(gfw) }
                    .gfp()
                    .variant(
                        ext_config
                            .glitch_filter_config
                            .as_ref()
                            .map(|e| e.prescaler)
                            .unwrap_or_default()
                            .into(),
                    )
                    .gfe()
                    .enable()
                    .tpsw()
                    .variant(ext_config.sample_width.into())
                    .tpsf()
                    .variant(ext_config.sample_frequency.into())
                    .tpe()
                    .variant(ext_config.pull.into())
                    .tps()
                    .variant(ext_config.pull.into())
            });
        }

        // Enable interrupts
        self.info.regs().ier().write(|w| {
            w.dtie()
                .enable()
                .tiie0()
                .enable()
                .tiie1()
                .enable()
                .tiie2()
                .enable()
                .tiie4()
                .enable()
                .tiie5()
                .enable()
                .tiie6()
                .enable()
                .tiie7()
                .enable()
                .tiie8()
                .enable()
                .tiie9()
                .enable()
                .tiie10()
                .enable()
                .tpie0()
                .enable()
                .tpie1()
                .enable()
                .tpie2()
                .enable()
                .tpie3()
                .enable()
                .tpie4()
                .enable()
                .tpie5()
                .enable()
        });

        // Lock all registers
        self.info.regs().lr().write(|w| {
            w.crl()
                .lock()
                .srl()
                .lock()
                .lrl()
                .lock()
                .iel()
                .lock()
                .tsl()
                .lock()
                .tel()
                .lock()
                .ppl()
                .lock()
                .gfl0()
                .lock()
                .gfl1()
                .lock()
                .gfl2()
                .lock()
                .gfl3()
                .lock()
                .gfl4()
                .lock()
                .gfl5()
                .lock()
        });

        self.info.regs().cr().modify(|_, w| {
            unsafe { w.dpr().bits(config.prescaler) }
                .distam()
                .variant(config.disable_on_tamper.into())
                .tfsr()
                .variant(config.reset_on_tamper.into())
                .den()
                .enable()
        });
    }

    fn init(&self) {
        let sr = self.info.regs().sr().read();
        if sr.dtf().is_det() && sr.taf().is_not_occur() {
            self.info.regs().cr().modify(|_, w| w.tfsr().reset());
        } else {
            self.info.regs().cr().modify(|_, w| w.tfsr().no_reset());
        }

        self.info
            .regs()
            .sr()
            .write(|w| w.taf().clear_bit_by_one().dtf().clear_bit_by_one());
    }

    fn reset(&self) {
        if self.info.regs().lr().read().crl().is_not_lock() {
            self.info.regs().cr().write(|w| w.swr().sw_reset());
            // Hold reset for a little while
            cortex_m::asm::delay(5);
            self.info.regs().cr().write(|w| w.swr().clear_bit());
        }
    }

    fn clear_flags(&self) {
        if self.info.regs().lr().read().srl().is_not_lock() {
            self.info.regs().sr().write(|w| {
                w.dtf()
                    .clear_bit_by_one()
                    .taf()
                    .clear_bit_by_one()
                    .tif0()
                    .clear_bit_by_one()
                    .tif1()
                    .clear_bit_by_one()
                    .tif2()
                    .clear_bit_by_one()
                    .tif3()
                    .clear_bit_by_one()
                    .tif4()
                    .clear_bit_by_one()
                    .tif5()
                    .clear_bit_by_one()
                    .tif6()
                    .clear_bit_by_one()
                    .tif7()
                    .clear_bit_by_one()
                    .tif8()
                    .clear_bit_by_one()
                    .tif9()
                    .clear_bit_by_one()
                    .tif10()
                    .clear_bit_by_one()
                    .tpf0()
                    .clear_bit_by_one()
                    .tpf1()
                    .clear_bit_by_one()
                    .tpf2()
                    .clear_bit_by_one()
                    .tpf3()
                    .clear_bit_by_one()
                    .tpf4()
                    .clear_bit_by_one()
                    .tpf5()
                    .clear_bit_by_one()
            });
        }
    }
}

#[interrupt]
fn TDET() {
    // what to do here?
    defmt::info!("Status {:08x}", info().regs().sr().read().bits());
    panic!("foo");
}

struct Info {
    regs: *const pac::tdet0::RegisterBlock,
}

unsafe impl Sync for Info {}

impl Info {
    #[inline(always)]
    fn regs(&self) -> &'static pac::tdet0::RegisterBlock {
        unsafe { &*self.regs }
    }
}

fn info() -> &'static Info {
    static INFO: Info = Info {
        regs: pac::Tdet0::ptr(),
    };
    &INFO
}

mod sealed {
    /// Seal a trait
    pub trait Sealed {}
}

/// Driver mode.
#[allow(private_bounds)]
pub trait TamperPin<TDET0>: GpioPin + SealedPin + sealed::Sealed {
    fn mux(&self);
}

macro_rules! impl_pin {
    ($pin:ident) => {
        impl sealed::Sealed for crate::peripherals::$pin {}

        impl TamperPin<crate::peripherals::TDET0> for crate::peripherals::$pin {
            #[inline(always)]
            fn mux(&self) {
                self.set_function(crate::pac::port0::pcr0::Mux::Mux0);
            }
        }
    };
}

// TAMPER0
impl_pin!(P3_31);

// TAMPER1
impl_pin!(P3_29);

// TAMPER2
impl_pin!(P3_26);

// TAMPER3
impl_pin!(P3_25);

// TAMPER4
impl_pin!(P3_19);

// TAMPER5
impl_pin!(P3_18);
