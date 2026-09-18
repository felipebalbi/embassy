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
use crate::pac::spc::{
    ActiveCfgBgmode, ActiveCfgCoreldoVddDs, ActiveCfgCoreldoVddLvl, LpCfgCoreldoVddDs, LpCfgCoreldoVddLvl, Vsm,
};

/// A fully resolved clock configuration, ready to be applied to hardware.
pub(super) struct ResolvedClockProgram {
    /// The [`Clocks`] state that applying this program will produce.
    pub(super) clocks: Clocks,
    /// The values `configure_voltages` needs.
    pub(super) voltage: VoltageProgram,
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
