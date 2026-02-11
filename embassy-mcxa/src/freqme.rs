//! Frequency measurement driver.

use core::convert::Infallible;
use core::marker::PhantomData;

use embassy_embedded_hal::SetConfig;
use embassy_hal_internal::{Peri, PeripheralType};
use maitake_sync::WaitCell;
use nxp_pac::freqme::vals::{
    CtrlRMeasureInProgress, CtrlWMeasureInProgress, CtrlWPulseMode, CtrlWPulsePol, GtMaxStat, LtMinStat,
};
use paste::paste;

use crate::clocks::periph_helpers::NoConfig;
use crate::clocks::{ClockError, Gate, WakeGuard, enable_and_reset};
use crate::{interrupt, pac};

/// Enable/disable continuous mode
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Continuous {
    /// No
    No,
    /// Yes
    Yes,
}

impl From<Continuous> for bool {
    fn from(value: Continuous) -> Self {
        match value {
            Continuous::No => false,
            Continuous::Yes => true,
        }
    }
}

/// Frequency measurement configuration
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FrequencyConfig {
    /// Continous measurement?
    continuous: Continuous,
    /// Reference clock scaling factor.
    ///
    /// Specifies the reference clock scaling factor in Frequency
    /// Measurement mode. The reference count cycle is
    /// 2^`ref_scale`. A higher number provides better accuracy but
    /// consumes more processing time.
    ///
    /// Only the lowest 5 bits are valid. Any value above 31 will be
    /// treated as 31.
    ref_scale: u8,
}

/// Pulse polarity
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Polarity {
    /// Active high
    ActiveHigh,
    /// Active low
    ActiveLow,
}

impl From<Polarity> for CtrlWPulsePol {
    fn from(value: Polarity) -> Self {
        match value {
            Polarity::ActiveHigh => CtrlWPulsePol::HIGH_PERIOD,
            Polarity::ActiveLow => CtrlWPulsePol::LOW_PERIOD,
        }
    }
}

/// Pulse width measurement configuration
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct PulseWidthConfig {
    /// Pulse polarity
    polarity: Polarity,
}

/// Measurement mode
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Frequency measurement mode
    Frequency(FrequencyConfig),
    /// Continuous
    PulseWidth(PulseWidthConfig),
}

impl From<Mode> for bool {
    fn from(value: Mode) -> Self {
        match value {
            Mode::Frequency(_) => false,
            Mode::PulseWidth(_) => true,
        }
    }
}

/// FREQME configuration
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    mode: Mode,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: Mode::Frequency(FrequencyConfig {
                continuous: Continuous::No,
                ref_scale: 0,
            }),
        }
    }
}

/// Errors exclusive to HW initialization
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SetupError {
    /// Clock configuration error.
    ClockSetup(ClockError),
    /// Other internal errors or unexpected state.
    Other,
}

/// Errors exclusive to frequency measurement
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MeasurementError {
    /// Measurement is greater than maximum.
    GreaterThanMax,
    /// Measurement is less than minimum.
    LessThanMin,
    /// Other internal errors or unexpected state.
    Other,
}

pub struct Freqme<'d> {
    info: &'static Info,
    config: Config,
    _wg: Option<WakeGuard>,
    _phantom: PhantomData<&'d mut ()>,
}

impl<'d> Freqme<'d> {
    /// Create a new instance of the frequency measurement driver.
    pub fn new<T: Instance>(
        _peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Result<Self, SetupError> {
        let parts = unsafe { enable_and_reset::<T>(&NoConfig).map_err(SetupError::ClockSetup)? };

        let inst = Self {
            info: T::info(),
            config,
            _wg: parts.wake_guard,
            _phantom: PhantomData,
        };

        Ok(inst)
    }

    fn check_status(&mut self) -> Result<u32, MeasurementError> {
        let stat = self.info.regs().ctrlstat().read();

        if stat.gt_max_stat() == GtMaxStat::GT_MAX {
            Err(MeasurementError::GreaterThanMax)
        } else if stat.lt_min_stat() == LtMinStat::LT_MIN {
            Err(MeasurementError::LessThanMin)
        } else if stat.result_ready_stat() {
            let f = self.info.regs().ctrl_r().read().result();
            Ok(f)
        } else {
            Err(MeasurementError::Other)
        }
    }

    pub async fn measure(&mut self) -> Result<u32, MeasurementError> {
        self.info.regs().ctrl_w().write(|w| {
            w.set_gt_max_int_en(true);
            w.set_lt_min_int_en(true);
            w.set_result_ready_int_en(true);

            match self.config.mode {
                Mode::Frequency(freq_config) => {
                    w.set_continuous_mode_en(freq_config.continuous.into());
                    w.set_ref_scale(freq_config.ref_scale.clamp(0, 31));
                    w.set_pulse_mode(CtrlWPulseMode::FREQ_ME_MODE);
                }
                Mode::PulseWidth(pulse_width_config) => {
                    w.set_pulse_pol(pulse_width_config.polarity.into());
                    w.set_pulse_mode(CtrlWPulseMode::PULSE_ME_MODE);
                }
            }

            w.set_measure_in_progress(CtrlWMeasureInProgress::INITIATE_A_FREQME_CYCLE);
        });

        self.info
            .wait_cell()
            .wait_for(|| {
                let stat = self.info.regs().ctrlstat().read();
                let gtmax = stat.gt_max_stat();
                let ltmin = stat.lt_min_stat();
                let ready = stat.result_ready_stat();

                gtmax == GtMaxStat::GT_MAX || ltmin == LtMinStat::LT_MIN || ready
            })
            .await
            .map_err(|_| MeasurementError::Other)?;

        self.check_status()
    }
}

/// FREQME interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::PERF_INT_INCR();
        if T::info().regs().ctrl_r().read().measure_in_progress() == CtrlRMeasureInProgress::CYCLE_DONE {
            T::info().regs().ctrl_w().write(|w| {
                w.set_gt_max_int_en(false);
                w.set_lt_min_int_en(false);
                w.set_result_ready_int_en(false);
            });
            T::PERF_INT_WAKE_INCR();
            T::info().wait_cell().wake();
        }
    }
}

trait SealedInstance {
    fn info() -> &'static Info;
}

/// FREQME Instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send + Gate<MrccPeriphConfig = NoConfig> {
    /// Interrupt for this FREQME instance.
    type Interrupt: interrupt::typelevel::Interrupt;
    const PERF_INT_INCR: fn();
    const PERF_INT_WAKE_INCR: fn();
}

struct Info {
    regs: pac::freqme::Freqme,
    wait_cell: WaitCell,
}

impl Info {
    #[inline(always)]
    fn regs(&self) -> pac::freqme::Freqme {
        self.regs
    }

    #[inline(always)]
    fn wait_cell(&self) -> &WaitCell {
        &self.wait_cell
    }
}

unsafe impl Sync for Info {}

macro_rules! impl_instance {
    ($($n:expr),*) => {
        $(
            paste!{
                impl SealedInstance for crate::peripherals::[<FREQME $n>] {
                    fn info() -> &'static Info {
                        static INFO: Info = Info {
                            regs: pac::[<FREQME $n>],
                            wait_cell: WaitCell::new(),
                        };
                        &INFO
                    }
                }

                impl Instance for crate::peripherals::[<FREQME $n>] {
                    type Interrupt = crate::interrupt::typelevel::[<FREQME $n>];
                    const PERF_INT_INCR: fn() = crate::perf_counters::[<incr_interrupt_freqme $n>];
                    const PERF_INT_WAKE_INCR: fn() = crate::perf_counters::[<incr_interrupt_freqme $n _wake>];
                }
            }
        )*
    };
}

impl_instance!(0);

impl<'d> SetConfig for Freqme<'d> {
    type Config = Config;
    type ConfigError = Infallible;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.config = *config;
        Ok(())
    }
}
