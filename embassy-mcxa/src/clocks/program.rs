//! The resolved clock program.
//!
//! [`super::calc::resolve_program`] produces one of these from a
//! [`ClocksConfig`](super::config::ClocksConfig). It carries both the resulting
//! [`Clocks`] state *and* the register-only values that
//! [`super::operator::ClockOperator`] needs in order to apply that state to
//! hardware, so that the operator never has to re-derive anything from the
//! configuration.
//!
//! All items here are private to the `clocks` module: they name PAC types, and
//! must not leak into the public API.

use super::config::CoreSleep;
use super::types::Clocks;
use crate::pac::scg::{Fircsten, FreqSel};
use crate::pac::spc::{
    ActiveCfgBgmode, ActiveCfgCoreldoVddDs, ActiveCfgCoreldoVddLvl, LpCfgCoreldoVddDs, LpCfgCoreldoVddLvl, Vsm,
};
#[cfg(all(feature = "mcxa5xx", feature = "unstable-osc32k", not(feature = "rosc-32k-as-gpio")))]
use crate::pac::vbat::{CoarseAmpGain, ExtalCapSel, SupplyDet, XtalCapSel};

/// A fully resolved clock configuration, ready to be applied to hardware.
pub(super) struct ResolvedClockProgram {
    /// The [`Clocks`] state that applying this program will produce.
    pub(super) clocks: Clocks,
    /// The values `configure_voltages` needs.
    pub(super) voltage: VoltageProgram,
    /// The values `configure_sirc_clocks_early`/`_late` need.
    pub(super) sirc: SircProgram,
    /// The values `configure_firc_clocks` needs.
    pub(super) firc: FircProgram,
    /// The values `configure_fro16k_clocks` needs.
    pub(super) fro16k: Fro16KProgram,
    /// The values `configure_osc32k_clocks` needs.
    #[cfg(all(feature = "mcxa5xx", feature = "unstable-osc32k", not(feature = "rosc-32k-as-gpio")))]
    pub(super) osc32k: Osc32KProgram,
}

/// Everything `configure_osc32k_clocks` derives from the configuration.
///
/// Each mode's hardware parameters live inside its own variant, so that no
/// high-power-only value is carried into (and left unread by) the low-power
/// path, or vice versa.
#[cfg(all(feature = "mcxa5xx", feature = "unstable-osc32k", not(feature = "rosc-32k-as-gpio")))]
pub(super) enum Osc32KProgram {
    /// No OSC32K configuration was requested.
    Absent,
    /// High performance transconductance oscillator mode.
    HighPower {
        /// `VBAT0[OSCCTLA.XTAL_CAP_SEL]`.
        xtal_cap_sel: XtalCapSel,
        /// `VBAT0[OSCCTLA.EXTAL_CAP_SEL]`.
        extal_cap_sel: ExtalCapSel,
        /// `VBAT0[OSCCTLA.COARSE_AMP_GAIN]`.
        coarse_amp_gain: CoarseAmpGain,
        /// `VBAT0[OSCCLKE.CLKE]`, the output-gate bitmask.
        clke: u8,
    },
    /// Low power switched oscillator mode.
    LowPower {
        /// `VBAT0[OSCCTLA.COARSE_AMP_GAIN]`.
        coarse_amp_gain: CoarseAmpGain,
        /// `VBAT0[OSCCTLA.SUPPLY_DET]`.
        supply_det: SupplyDet,
        /// `VBAT0[OSCCLKE.CLKE]`, the output-gate bitmask.
        clke: u8,
    },
}

/// Everything `configure_fro16k_clocks` derives from the configuration.
pub(super) struct Fro16KProgram {
    /// `VBAT0[FROCTLA.FRO_EN]`.
    ///
    /// NOTE: this reflects the *presence* of a FRO16K configuration, which is NOT
    /// the same as "some output domain is active": a configuration with all
    /// domains disabled still enables the oscillator while gating every output.
    pub(super) enable: bool,
    /// `VBAT0[FROCLKE.CLKE]`, the output-gate bitmask, if a FRO16K configuration
    /// was requested. `None` makes `configure_fro16k_clocks` return early before
    /// the `FROCLKE` write.
    pub(super) clke: Option<u8>,
}

/// Everything `configure_firc_clocks` derives from the configuration.
pub(super) struct FircProgram {
    /// Whether a FIRC configuration was requested at all. When `false`, FIRC is
    /// disabled and the function returns early.
    pub(super) enabled: bool,
    /// Whether the requested FIRC frequency is the chip family's reset default.
    ///
    /// NOTE: this is `false` when no FIRC configuration was requested, which is
    /// what drives the RCCR -> SIRC detour in that case.
    pub(super) is_default: bool,
    /// `FIRCCFG[FREQ_SEL]`, if FIRC is enabled.
    pub(super) freq_sel: Option<FreqSel>,
    /// `FIRCCSR[FIRCSTEN]`.
    pub(super) fircsten: Fircsten,
    /// `FIRCCSR[FIRC_FCLK_PERIPH_EN]`.
    pub(super) fro_hf_gate: bool,
    /// `FIRCCSR[FIRC_SCLK_PERIPH_EN]`.
    pub(super) fundamental_gate: bool,
    /// `SYSCON[FROHFDIV.DIV]`, if the `fro_hf_div` output is enabled.
    pub(super) fro_hf_div_bits: Option<u8>,
}

/// Everything the SIRC/FRO12M configuration steps derive from the configuration.
pub(super) struct SircProgram {
    /// `SIRCCSR[SIRCSTEN]`.
    pub(super) deep: bool,
    /// `SYSCON[FROLFDIV.DIV]`, if the `fro_lf_div` output is enabled.
    pub(super) fro_lf_div_bits: Option<u8>,
    /// SIRC's `fro_12m` is force-enabled until `main_clk` is selected, and must be
    /// gated off again by `configure_sirc_clocks_late`.
    pub(super) sirc_forced: bool,
}

/// Everything `configure_voltages` derives from the configuration.
pub(super) struct VoltageProgram {
    /// `Some` if the active-mode VDD level must be raised, with the exact
    /// `ACTIVE_CFG[CORELDO_VDD_LVL]` and `SRAMCTL[VSM]` values to write.
    pub(super) active_level_change: Option<(ActiveCfgCoreldoVddLvl, Vsm)>,
    /// `SPC[LPWKUP_DELAY]`.
    pub(super) lpwkup: u16,
    /// The low-power (`LP_CFG`) settings.
    pub(super) low_power: LowPowerProgram,
    /// The active-mode (`ACTIVE_CFG`) settings.
    pub(super) active: ActiveProgram,
    /// The requested core sleep depth.
    pub(super) core_sleep: CoreSleep,
    /// `CMC[FLASHCR.FLASHWAKE]`.
    pub(super) flash_wake: bool,
    /// `CMC[FLASHCR.FLASHDOZE]`.
    pub(super) flash_doze: bool,
}

/// Low-power-mode voltage settings.
pub(super) struct LowPowerProgram {
    /// Which drive-strength arm was requested, plus any arm-local values.
    pub(super) drive: LowPowerDrive,
    /// `LP_CFG[CORELDO_VDD_DS]`.
    pub(super) ds: LpCfgCoreldoVddDs,
    /// `LP_CFG[CORELDO_VDD_LVL]`.
    pub(super) level: LpCfgCoreldoVddLvl,
    /// Whether the low-power bandgap is enabled.
    pub(super) bandgap: bool,
}

/// The requested low-power drive strength.
pub(super) enum LowPowerDrive {
    /// Low drive; the high/low voltage detectors follow the bandgap.
    Low {
        /// Value written to `LP_CFG[SYS_HVDE/SYS_LVDE/CORE_LVDE]`.
        enable_detectors: bool,
    },
    /// Normal drive.
    Normal,
}

/// Active-mode voltage settings.
pub(super) struct ActiveProgram {
    /// Which drive-strength arm was requested, plus any arm-local values.
    ///
    /// NOTE: the resolved active bandgap state itself lives in
    /// [`ResolvedClockProgram::clocks`], as `bandgap_active`.
    pub(super) drive: ActiveDrive,
}

/// The requested active-mode drive strength.
pub(super) enum ActiveDrive {
    /// Low drive; the high/low voltage detectors follow the bandgap.
    Low {
        /// Value written to `ACTIVE_CFG[SYS_HVDE/SYS_LVDE/CORE_LVDE]`.
        enable_detectors: bool,
        /// `ACTIVE_CFG[CORELDO_VDD_DS]`.
        ds: ActiveCfgCoreldoVddDs,
        /// `ACTIVE_CFG[BGMODE]`.
        bgmode: ActiveCfgBgmode,
    },
    /// Normal drive; the level was already set to normal above.
    Normal,
}
