//! Tamper Detection (TDET) driver.

use core::marker::PhantomData;

use embassy_hal_internal::Peri;
use embassy_hal_internal::interrupt::InterruptExt;

use crate::gpio::{AnyPin, GpioPin, SealedPin};
use crate::pac::tdet::vals::{
    Crl, Distam, Gfe, Gfl0, Gfl1, Gfl2, Gfl3, Gfl4, Gfl5, Gfp, Iel, Lrl, Ppl, Srl, Swr, Tel, Tps, Tpsf, Tpsw, Tsl,
};
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
            GlitchFilterPrescaler::_256Hz => Self::FREQ_512_HZ,
            GlitchFilterPrescaler::_16384Hz => Self::FREQ_32_KHZ,
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

impl From<Pull> for bool {
    fn from(value: Pull) -> Self {
        match value {
            Pull::Disabled => false,
            _ => true,
        }
    }
}

impl From<Pull> for Tps {
    fn from(value: Pull) -> Self {
        match value {
            Pull::Up => Self::ASSERT,
            _ => Self::NEGATE,
        }
    }
}

/// Pin polarity
#[derive(Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    /// Active-high
    #[default]
    ActiveHigh,

    /// Active-low
    ActiveLow,
}

impl From<Polarity> for bool {
    fn from(value: Polarity) -> Self {
        match value {
            Polarity::ActiveHigh => false,
            Polarity::ActiveLow => true,
        }
    }
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
            SampleFrequency::_8Cycles => Self::CYCLES_8,
            SampleFrequency::_32Cycles => Self::CYCLES_32,
            SampleFrequency::_128Cycles => Self::CYCLES_128,
            SampleFrequency::_512Cycles => Self::CYCLES_512,
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
            SampleWidth::Continuous => Self::DISABLE,
            SampleWidth::_2x1 => Self::CYCLES_2,
            SampleWidth::_4x2 => Self::CYCLES_4,
            SampleWidth::_8x4 => Self::CYCLES_8,
        }
    }
}

/// Enable tamper detection
#[derive(Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Enable {
    /// No
    #[default]
    No,

    /// Yes
    Yes,
}

impl From<Enable> for bool {
    fn from(value: Enable) -> bool {
        match value {
            Enable::No => false,
            Enable::Yes => true,
        }
    }
}

/// Reset on tamper detection
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResetOnTamper {
    /// No
    #[default]
    No,

    /// Yes
    Yes,
}

impl From<ResetOnTamper> for bool {
    fn from(value: ResetOnTamper) -> Self {
        match value {
            ResetOnTamper::No => false,
            ResetOnTamper::Yes => true,
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
            DisableOnTamper::No => Self::NO_EFFECT,
            DisableOnTamper::Yes => Self::AUTO_DIS,
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
            w.set_tie4(config.vdd_core_glitch_config.enable.into());
            w.set_tie5(config.system_low_voltage_config.enable.into());
            w.set_tie6(config.system_high_voltage_config.enable.into());
            w.set_tie7(config.cdog_config.enable.into());
            w.set_tie9(config.mbc_config.enable.into());
            w.set_tie10(config.ram_zeroize_config.enable.into());
            w.set_tpe0(config.ext0_config.is_some());
            w.set_tpe1(config.ext1_config.is_some());
            w.set_tpe2(config.ext2_config.is_some());
            w.set_tpe3(config.ext3_config.is_some());
            w.set_tpe4(config.ext4_config.is_some());
            w.set_tpe5(config.ext5_config.is_some());
        });

        // Configure pin polarity
        self.info.regs().ppr().write(|w| {
            w.set_tpp0(
                config
                    .ext0_config
                    .as_ref()
                    .map(|c| c.polarity)
                    .unwrap_or(Polarity::ActiveHigh)
                    .into(),
            );

            w.set_tpp1(
                config
                    .ext1_config
                    .as_ref()
                    .map(|c| c.polarity)
                    .unwrap_or(Polarity::ActiveHigh)
                    .into(),
            );

            w.set_tpp2(
                config
                    .ext2_config
                    .as_ref()
                    .map(|c| c.polarity)
                    .unwrap_or(Polarity::ActiveHigh)
                    .into(),
            );

            w.set_tpp3(
                config
                    .ext3_config
                    .as_ref()
                    .map(|c| c.polarity)
                    .unwrap_or(Polarity::ActiveHigh)
                    .into(),
            );

            w.set_tpp4(
                config
                    .ext4_config
                    .as_ref()
                    .map(|c| c.polarity)
                    .unwrap_or(Polarity::ActiveHigh)
                    .into(),
            );

            w.set_tpp5(
                config
                    .ext5_config
                    .as_ref()
                    .map(|c| c.polarity)
                    .unwrap_or(Polarity::ActiveHigh)
                    .into(),
            );
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
                w.set_gfw(gfw);
                w.set_gfp(
                    ext_config
                        .glitch_filter_config
                        .as_ref()
                        .map(|e| e.prescaler)
                        .unwrap_or_default()
                        .into(),
                );
                w.set_gfe(Gfe::ENABLE);
                w.set_tpsw(ext_config.sample_width.into());
                w.set_tpsf(ext_config.sample_frequency.into());
                w.set_tpe(ext_config.pull.into());
                w.set_tps(ext_config.pull.into());
            });
        }

        if let Some(ext_config) = config.ext1_config {
            // GFW is calculated as (GFW + 1) * 2
            let mut gfw = ext_config
                .glitch_filter_config
                .as_ref()
                .map(|e| e.width)
                .unwrap_or(2)
                .clamp(2, 128);

            gfw /= 2;
            gfw -= 1;

            self.info.regs().pgfr(1).write(|w| {
                w.set_gfw(gfw);
                w.set_gfp(
                    ext_config
                        .glitch_filter_config
                        .as_ref()
                        .map(|e| e.prescaler)
                        .unwrap_or_default()
                        .into(),
                );
                w.set_gfe(Gfe::ENABLE);
                w.set_tpsw(ext_config.sample_width.into());
                w.set_tpsf(ext_config.sample_frequency.into());
                w.set_tpe(ext_config.pull.into());
                w.set_tps(ext_config.pull.into());
            });
        }

        if let Some(ext_config) = config.ext2_config {
            // GFW is calculated as (GFW + 1) * 2
            let mut gfw = ext_config
                .glitch_filter_config
                .as_ref()
                .map(|e| e.width)
                .unwrap_or(2)
                .clamp(2, 128);

            gfw /= 2;
            gfw -= 1;

            self.info.regs().pgfr(2).write(|w| {
                w.set_gfw(gfw);
                w.set_gfp(
                    ext_config
                        .glitch_filter_config
                        .as_ref()
                        .map(|e| e.prescaler)
                        .unwrap_or_default()
                        .into(),
                );
                w.set_gfe(Gfe::ENABLE);
                w.set_tpsw(ext_config.sample_width.into());
                w.set_tpsf(ext_config.sample_frequency.into());
                w.set_tpe(ext_config.pull.into());
                w.set_tps(ext_config.pull.into());
            });
        }

        if let Some(ext_config) = config.ext3_config {
            // GFW is calculated as (GFW + 1) * 2
            let mut gfw = ext_config
                .glitch_filter_config
                .as_ref()
                .map(|e| e.width)
                .unwrap_or(2)
                .clamp(2, 128);

            gfw /= 2;
            gfw -= 1;

            self.info.regs().pgfr(3).write(|w| {
                w.set_gfw(gfw);
                w.set_gfp(
                    ext_config
                        .glitch_filter_config
                        .as_ref()
                        .map(|e| e.prescaler)
                        .unwrap_or_default()
                        .into(),
                );
                w.set_gfe(Gfe::ENABLE);
                w.set_tpsw(ext_config.sample_width.into());
                w.set_tpsf(ext_config.sample_frequency.into());
                w.set_tpe(ext_config.pull.into());
                w.set_tps(ext_config.pull.into());
            });
        }

        if let Some(ext_config) = config.ext4_config {
            // GFW is calculated as (GFW + 1) * 2
            let mut gfw = ext_config
                .glitch_filter_config
                .as_ref()
                .map(|e| e.width)
                .unwrap_or(2)
                .clamp(2, 128);

            gfw /= 2;
            gfw -= 1;

            self.info.regs().pgfr(4).write(|w| {
                w.set_gfw(gfw);
                w.set_gfp(
                    ext_config
                        .glitch_filter_config
                        .as_ref()
                        .map(|e| e.prescaler)
                        .unwrap_or_default()
                        .into(),
                );
                w.set_gfe(Gfe::ENABLE);
                w.set_tpsw(ext_config.sample_width.into());
                w.set_tpsf(ext_config.sample_frequency.into());
                w.set_tpe(ext_config.pull.into());
                w.set_tps(ext_config.pull.into());
            });
        }

        if let Some(ext_config) = config.ext5_config {
            // GFW is calculated as (GFW + 1) * 2
            let mut gfw = ext_config
                .glitch_filter_config
                .as_ref()
                .map(|e| e.width)
                .unwrap_or(2)
                .clamp(2, 128);

            gfw /= 2;
            gfw -= 1;

            self.info.regs().pgfr(5).write(|w| {
                w.set_gfw(gfw);
                w.set_gfp(
                    ext_config
                        .glitch_filter_config
                        .as_ref()
                        .map(|e| e.prescaler)
                        .unwrap_or_default()
                        .into(),
                );
                w.set_gfe(Gfe::ENABLE);
                w.set_tpsw(ext_config.sample_width.into());
                w.set_tpsf(ext_config.sample_frequency.into());
                w.set_tpe(ext_config.pull.into());
                w.set_tps(ext_config.pull.into());
            });
        }

        // Enable interrupts
        self.info.regs().ier().write(|w| {
            w.set_dtie(true);
            w.set_tiie0(true);
            w.set_tiie1(true);
            w.set_tiie2(true);
            w.set_tiie3(true);
            w.set_tiie4(true);
            w.set_tiie5(true);
            w.set_tiie6(true);
            w.set_tiie7(true);
            w.set_tiie8(true);
            w.set_tiie9(true);
            w.set_tiie10(true);

            w.set_tpie0(true);
            w.set_tpie1(true);
            w.set_tpie2(true);
            w.set_tpie3(true);
            w.set_tpie4(true);
            w.set_tpie5(true);
        });

        // Lock all registers
        // self.info.regs().lr().write(|w| {
        //     w.set_crl(Crl::LOCK);
        //     w.set_srl(Srl::LOCK);
        //     w.set_lrl(Lrl::LOCK);
        //     w.set_iel(Iel::LOCK);
        //     w.set_tsl(Tsl::LOCK);
        //     w.set_tel(Tel::LOCK);
        //     w.set_ppl(Ppl::LOCK);
        //     w.set_gfl0(Gfl0::LOCK);
        //     w.set_gfl1(Gfl1::LOCK);
        //     w.set_gfl2(Gfl2::LOCK);
        //     w.set_gfl3(Gfl3::LOCK);
        //     w.set_gfl4(Gfl4::LOCK);
        //     w.set_gfl5(Gfl5::LOCK);
        // });

        self.info.regs().cr().modify(|w| {
            w.set_dpr(config.prescaler);
            w.set_distam(config.disable_on_tamper.into());
            w.set_tfsr(config.reset_on_tamper.into());
            w.set_den(true);
        });
    }

    fn init(&self) {
        let sr = self.info.regs().sr().read();
        if sr.dtf() && !sr.taf() {
            self.info.regs().cr().modify(|w| w.set_tfsr(true));
        } else {
            self.info.regs().cr().modify(|w| w.set_tfsr(false));
        }

        self.info.regs().sr().write(|w| {
            w.set_taf(true);
            w.set_dtf(true);
        });
    }

    fn reset(&self) {
        if self.info.regs().lr().read().crl() == Crl::NOT_LOCK {
            self.info.regs().cr().write(|w| {
                w.set_swr(Swr::SW_RESET);
            });
            // Hold reset for a little while
            cortex_m::asm::delay(5);
            self.info.regs().cr().write(|w| w.set_swr(Swr::NO_EFFECT));
        }
    }

    fn clear_flags(&self) {
        self.info.regs().lr().modify(|w| w.set_srl(Srl::NOT_LOCK));

        self.info.regs().sr().write(|w| {
            w.set_dtf(true);
            w.set_taf(true);

            w.set_tif0(true);
            w.set_tif1(true);
            w.set_tif2(true);
            w.set_tif3(true);
            w.set_tif4(true);
            w.set_tif5(true);
            w.set_tif6(true);
            w.set_tif7(true);
            w.set_tif8(true);
            w.set_tif9(true);
            w.set_tif10(true);

            w.set_tpf0(true);
            w.set_tpf1(true);
            w.set_tpf2(true);
            w.set_tpf3(true);
            w.set_tpf4(true);
            w.set_tpf5(true);
        });

        self.info.regs().lr().modify(|w| w.set_srl(Srl::LOCK));
    }
}

#[interrupt]
fn TDET() {
    // what to do here?
    let r = info().regs().sr().read();
    // defmt::info!("Status {}", r);

    info().regs().lr().write(|w| w.set_srl(Srl::NOT_LOCK));
    info().regs().sr().write(|w| w.0 = r.0);
    info().regs().lr().write(|w| w.set_srl(Srl::LOCK));

    // panic!("foo");
}

struct Info {
    regs: pac::tdet::Tdet,
}

unsafe impl Sync for Info {}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::tdet::Tdet {
        self.regs
    }
}

fn info() -> &'static Info {
    static INFO: Info = Info { regs: pac::TDET0 };
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
                self.set_function(crate::pac::port::vals::Mux::MUX0);
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
