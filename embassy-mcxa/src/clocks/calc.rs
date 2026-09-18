//! Pure clock-tree arithmetic and validation helpers.
//!
//! Everything in this module is a `const fn` or a `const` item: no register
//! access, no PAC types, no side effects. These items are extracted verbatim
//! from the `configure_*` methods in [`super::operator`] so that they can also
//! be evaluated at compile time.

use super::config::{
    ClocksConfig, Div8, FlashSleep, MainClockSource, SpllMode, SpllSource, VddDriveStrength, VddLevel,
};
use super::program::{
    ActiveDrive, ActiveProgram, LowPowerDrive, LowPowerProgram, ResolvedClockProgram, SircProgram, VoltageProgram,
};
use super::types::{Clock, ClockError, Clocks, PoweredClock};
use crate::chips::ClockLimits;
use crate::pac::spc::{
    ActiveCfgBgmode, ActiveCfgCoreldoVddDs, ActiveCfgCoreldoVddLvl, LpCfgCoreldoVddDs, LpCfgCoreldoVddLvl, Vsm,
};

//
// Frequency constants
//

/// Fixed frequency of the SIRC/FRO12M root clock.
pub(super) const FRO_12M_FREQUENCY: u32 = 12_000_000;

/// Fixed frequency of the ROSC/FRO16K clock.
pub(super) const FRO_16K_FREQUENCY: u32 = 16_384;

/// Fixed frequency of the OSC32K clock.
#[cfg(all(feature = "mcxa5xx", feature = "unstable-osc32k", not(feature = "rosc-32k-as-gpio")))]
pub(super) const OSC_32K_FREQUENCY: u32 = 32_768;

/// Frequency of the `clk_45m`/`clk_48m` FIRC fundamental output.
///
/// Deliberately preserves current behaviour on both chip families.
pub(super) const CLK_HF_FUNDAMENTAL_FREQUENCY: u32 = 45_000_000;

//
// Core helpers
//

/// Divide `source` by the divisor represented by `divider`.
pub(super) const fn divided_frequency(source: u32, divider: Div8) -> u32 {
    source / divider.into_divisor()
}

/// `clk_1m` is the FRO12M root divided by twelve.
pub(super) const fn clk_1m_frequency(source: u32) -> u32 {
    source / 12
}

/// Ensure `frequency` does not exceed `maximum`.
pub(super) const fn validate_max_frequency(
    frequency: u32,
    maximum: u32,
    clock: &'static str,
    reason: &'static str,
) -> Result<(), ClockError> {
    if frequency > maximum {
        Err(ClockError::BadConfig { clock, reason })
    } else {
        Ok(())
    }
}

//
// SOSC
//

/// The SOSC frequency range bands, as a PAC-free enum.
#[cfg(not(feature = "sosc-as-gpio"))]
pub(super) enum SoscRange {
    /// 8-16 MHz
    Freq8To16Mhz,
    /// 16-25 MHz
    Freq16To25Mhz,
    /// 25-40 MHz
    Freq25To40Mhz,
    /// 40-50 MHz
    Freq40To50Mhz,
}

/// Select the SOSC frequency range band for the given input frequency.
#[cfg(not(feature = "sosc-as-gpio"))]
pub(super) const fn sosc_range(frequency: u32) -> Result<SoscRange, ClockError> {
    if frequency < 8_000_000 {
        Err(ClockError::BadConfig {
            clock: "clk_in",
            reason: "freq too low",
        })
    } else if frequency < 16_000_000 {
        Ok(SoscRange::Freq8To16Mhz)
    } else if frequency < 25_000_000 {
        Ok(SoscRange::Freq16To25Mhz)
    } else if frequency < 40_000_000 {
        Ok(SoscRange::Freq25To40Mhz)
    } else if frequency <= 50_000_000 {
        Ok(SoscRange::Freq40To50Mhz)
    } else {
        Err(ClockError::BadConfig {
            clock: "clk_in",
            reason: "freq too high",
        })
    }
}

//
// SPLL
//

/// Check the SPLL feedback divider `M` is in range.
pub(super) const fn check_spll_m(m: u16) -> Result<u16, ClockError> {
    if m < 1 || m > u16::MAX {
        Err(ClockError::BadConfig {
            clock: "spll",
            reason: "m_mult out of range",
        })
    } else {
        Ok(m)
    }
}

/// Check the SPLL post-divider `P` is in range.
pub(super) const fn check_spll_p(p: u8) -> Result<u8, ClockError> {
    if p < 1 || p > 31 {
        Err(ClockError::BadConfig {
            clock: "spll",
            reason: "p_div out of range",
        })
    } else {
        Ok(p)
    }
}

/// Check the SPLL pre-divider `N` is in range.
pub(super) const fn check_spll_n(n: u8) -> Result<u8, ClockError> {
    if n < 1 || n > u8::MAX {
        Err(ClockError::BadConfig {
            clock: "spll",
            reason: "n_div out of range",
        })
    } else {
        Ok(n)
    }
}

/// The effective post-divider: `P`, doubled unless the `P2` divider is bypassed.
pub(super) const fn spll_post_divisor(p: u8, bypass_p2_div: bool) -> u32 {
    let mut div = p as u32;
    if !bypass_p2_div {
        div *= 2;
    }
    div
}

/// The combined pre- and post-divider: `N x P`, doubled unless the `P2` divider
/// is bypassed.
pub(super) const fn spll_pre_post_divisor(n: u8, p: u8, bypass_p2_div: bool) -> u32 {
    // This can't overflow: u8 x u8 (x 2) always fits in u32
    let mut div = (p as u32) * (n as u32);
    if !bypass_p2_div {
        div *= 2;
    }
    div
}

/// `frequency x M`, checked for overflow.
pub(super) const fn checked_spll_multiply(frequency: u32, m: u16) -> Option<u32> {
    frequency.checked_mul(m as u32)
}

/// `(frequency / divisor) x M`, checked for overflow. Divides first.
pub(super) const fn checked_spll_divide_then_multiply(frequency: u32, divisor: u32, m: u16) -> Option<u32> {
    (frequency / divisor).checked_mul(m as u32)
}

/// Unwrap an `Fcco` calculation that may have overflowed.
pub(super) const fn require_spll_fcco(v: Option<u32>) -> Result<u32, ClockError> {
    match v {
        Some(v) => Ok(v),
        None => Err(ClockError::BadConfig {
            clock: "spll",
            reason: "fcco invalid1",
        }),
    }
}

/// Unwrap an `Fout` calculation that may have overflowed.
pub(super) const fn require_spll_fout(v: Option<u32>) -> Result<u32, ClockError> {
    match v {
        Some(v) => Ok(v),
        None => Err(ClockError::BadConfig {
            clock: "spll",
            reason: "fout invalid",
        }),
    }
}

/// Fcco: 275MHz to 550MHz
pub(super) const fn validate_spll_fcco(fcco: u32) -> Result<(), ClockError> {
    if fcco < 275_000_000 || fcco > 550_000_000 {
        Err(ClockError::BadConfig {
            clock: "spll",
            reason: "fcco invalid2",
        })
    } else {
        Ok(())
    }
}

/// Fout: 4.3MHz to 2x Max CPU Frequency, and within the `pll1_clk` limit.
pub(super) const fn validate_spll_fout(fout: u32, limits: &ClockLimits) -> Result<(), ClockError> {
    let fmax = limits.cpu_clk;
    let spll_range_bad1 = fout < 4_300_000 || fout > 2 * fmax;
    let spll_range_bad2 = fout > limits.pll1_clk;

    if spll_range_bad1 || spll_range_bad2 {
        Err(ClockError::BadConfig {
            clock: "spll",
            reason: "fout invalid",
        })
    } else {
        Ok(())
    }
}

/// SELP = min(floor(M / 4) + 1, 31)
pub(super) const fn spll_selp(m: u16) -> u16 {
    // A = floor(m / 4) + 1
    let selp_a = (m / 4) + 1;
    // SELP = A  if A <  31
    //      = 31 if A >= 31
    if selp_a < 31 { selp_a } else { 31 }
}

/// SELI = min(A, 63), where A depends on the magnitude of `M`.
pub(super) const fn spll_seli(m: u16) -> u16 {
    // A = 1                    if        M >= 8000
    //   = floor(8000 / M)      if 8000 > M >= 122
    //   = 2 x floor(M / 4) / 3 if 122  > M >= 1
    let seli_a = if m >= 8000 {
        1
    } else if m >= 122 {
        8000 / m
    } else {
        (2 * (m / 4)) / 3
    };
    // SELI = A  if A <  63
    //      = 63 if A >= 63
    if seli_a < 63 { seli_a } else { 63 }
}

/// LOCK_TIME = 500us/Tref + 300, where Fref = Fin / N.
pub(super) const fn spll_lock_time(f_in: u32, n: Option<u8>) -> u32 {
    let f_ref = if let Some(n) = n { f_in / (n as u32) } else { f_in };
    f_ref.div_ceil(2000) + 300
}

//
// Flash wait states
//

/// Find the wait state count for `cpu_frequency`, falling back to `maximum`.
pub(super) const fn flash_wait_states(levels: &[(u32, u8)], maximum: u8, cpu_frequency: u32) -> u8 {
    let mut i = 0;
    while i < levels.len() {
        let (fmax, ws) = levels[i];
        if cpu_frequency <= fmax {
            return ws;
        }
        i += 1;
    }
    maximum
}

//
// Limit selectors
//

/// The clock limits for the given active-mode VDD level.
pub(super) const fn active_limits(level: VddLevel) -> &'static ClockLimits {
    match level {
        VddLevel::MidDriveMode => &ClockLimits::MID_DRIVE,
        #[cfg(feature = "mcxa5xx")]
        VddLevel::NormalMode => &ClockLimits::NORMAL_DRIVE,
        VddLevel::OverDriveMode => &ClockLimits::OVER_DRIVE,
    }
}

/// The clock limits for the given low-power-mode VDD level.
pub(super) const fn low_power_limits(level: VddLevel) -> &'static ClockLimits {
    match level {
        VddLevel::MidDriveMode => &ClockLimits::MID_DRIVE,
        #[cfg(feature = "mcxa5xx")]
        VddLevel::NormalMode => &ClockLimits::NORMAL_DRIVE,
        VddLevel::OverDriveMode => &ClockLimits::OVER_DRIVE,
    }
}

/// The lowest relevant clock limits for a clock with the given power mode.
pub(super) const fn lowest_relevant_limits(
    active_level: VddLevel,
    low_power_level: VddLevel,
    for_power: PoweredClock,
) -> &'static ClockLimits {
    // We always enforce that deep sleep has a drive <= active mode.
    match for_power {
        PoweredClock::NormalEnabledDeepSleepDisabled => active_limits(active_level),
        PoweredClock::AlwaysEnabled => low_power_limits(low_power_level),
    }
}

//
// Bandgap / VDD
//

/// Is the bandgap enabled for the given VDD drive strength?
pub(super) const fn bandgap_enabled(drive: VddDriveStrength) -> bool {
    match drive {
        VddDriveStrength::Low { enable_bandgap } => enable_bandgap,
        // "If you specify normal drive strength, you must write a value to LP[BGMODE] that enables the bandgap."
        VddDriveStrength::Normal => true,
    }
}

/// Does the configured bandgap state satisfy a clock with the given power mode?
pub(super) const fn bandgap_meets_requirement(active: bool, low_power: bool, for_power: PoweredClock) -> bool {
    match for_power {
        // We only need bandgap enabled in active Mode
        PoweredClock::NormalEnabledDeepSleepDisabled => active,
        // We need bandgaps enabled in both active and deep sleep mode
        PoweredClock::AlwaysEnabled => active && low_power,
    }
}

/// Validate the active-mode/low-power-mode VDD level pair.
///
/// Returns `(vdd_match, lpwkup)`, where `vdd_match` indicates the two levels are
/// identical, and `lpwkup` is the value written to `SPC->LPWKUP_DELAY`.
pub(super) const fn vdd_level_transition(active: VddLevel, low_power: VddLevel) -> Result<(bool, u16), ClockError> {
    const BAD_ASCENDING: Result<(bool, u16), ClockError> = Err(ClockError::BadConfig {
        clock: "vdd_power",
        reason: "Deep sleep can't have higher level than active mode",
    });

    match (active, low_power) {
        //
        // Correct "descending" options
        //
        // When voltage levels are not the same between ACTIVE mode and Low Power mode, you must write a
        // nonzero value to SPC->LPWKUP_DELAY.
        //
        // This SHOULD be covered by table 165. LPWKUP Delay, but it doesn't actually have
        // a value for the 1.0v-1.2v transition we need. For now, the C SDK always uses 0x5B.
        #[cfg(feature = "mcxa5xx")]
        (VddLevel::OverDriveMode, VddLevel::NormalMode) => Ok((false, 0x005b)),
        (VddLevel::OverDriveMode, VddLevel::MidDriveMode) => Ok((false, 0x005b)),
        #[cfg(feature = "mcxa5xx")]
        (VddLevel::NormalMode, VddLevel::MidDriveMode) => Ok((false, 0x005b)),

        //
        // Incorrect "ascending" options
        //
        // For now, enforce that active is always >= voltage to low power. I don't know if this
        // is required, but there's probably also no reason to support it?
        #[cfg(feature = "mcxa5xx")]
        (VddLevel::MidDriveMode, VddLevel::NormalMode) => BAD_ASCENDING,
        (VddLevel::MidDriveMode, VddLevel::OverDriveMode) => BAD_ASCENDING,
        #[cfg(feature = "mcxa5xx")]
        (VddLevel::NormalMode, VddLevel::OverDriveMode) => BAD_ASCENDING,

        // Correct "matching" options
        (VddLevel::MidDriveMode, VddLevel::MidDriveMode) => Ok((true, 0x0000)),
        #[cfg(feature = "mcxa5xx")]
        (VddLevel::NormalMode, VddLevel::NormalMode) => Ok((true, 0x0000)),
        (VddLevel::OverDriveMode, VddLevel::OverDriveMode) => Ok((true, 0x0000)),
    }
}

/// Do the two drive strengths match?
///
/// This is the `const`-callable form of the derived `PartialEq` comparison, including
/// that `Low { enable_bandgap: true }` and `Low { enable_bandgap: false }` are NOT equal.
pub(super) const fn vdd_drive_matches(active: VddDriveStrength, low_power: VddDriveStrength) -> bool {
    match (active, low_power) {
        (
            VddDriveStrength::Low {
                enable_bandgap: active_bandgap,
            },
            VddDriveStrength::Low {
                enable_bandgap: low_power_bandgap,
            },
        ) => active_bandgap == low_power_bandgap,
        (VddDriveStrength::Low { .. }, VddDriveStrength::Normal) => false,
        (VddDriveStrength::Normal, VddDriveStrength::Low { .. }) => false,
        (VddDriveStrength::Normal, VddDriveStrength::Normal) => true,
    }
}

//
// Resolver
//

/// Resolve a [`ClocksConfig`] into the [`Clocks`] state that [`super::init()`] would
/// produce, without touching any hardware.
///
/// This mirrors the phase ordering of [`super::init()`] exactly, and delegates every
/// frequency computation and every frequency-limit comparison to the helpers above —
/// the same helpers [`super::operator`] calls — so that the compile-time and run-time
/// views of the clock tree cannot drift apart.
pub(super) const fn resolve(config: &ClocksConfig) -> Result<Clocks, ClockError> {
    match resolve_program(config) {
        Ok(program) => Ok(program.clocks),
        Err(e) => Err(e),
    }
}

/// Resolve a [`ClocksConfig`] into the full [`ResolvedClockProgram`] that
/// [`super::init()`] applies: both the resulting [`Clocks`] state and the
/// register-only values [`super::operator`] needs to get there.
pub(super) const fn resolve_program(config: &ClocksConfig) -> Result<ResolvedClockProgram, ClockError> {
    let active_power = config.vdd_power.active_mode.level;
    let lp_power = config.vdd_power.low_power_mode.level;

    //
    // VDD: mirrors `configure_voltages`.
    //
    let (vdd_match, lpwkup) = match vdd_level_transition(active_power, lp_power) {
        Ok(value) => value,
        Err(e) => return Err(e),
    };
    let ds_match = vdd_drive_matches(
        config.vdd_power.active_mode.drive,
        config.vdd_power.low_power_mode.drive,
    );
    if ds_match && !vdd_match {
        return Err(ClockError::BadConfig {
            clock: "vdd_power",
            reason: "DS matches but LVL mismatches!",
        });
    }
    let bandgap_active = bandgap_enabled(config.vdd_power.active_mode.drive);
    let bandgap_lowpower = bandgap_enabled(config.vdd_power.low_power_mode.drive);

    // Register-only values, in the same order and from the same expressions
    // `configure_voltages` used to compute them inline.
    let active_level_change = match active_power {
        VddLevel::MidDriveMode => None,
        #[cfg(feature = "mcxa5xx")]
        VddLevel::NormalMode => Some((ActiveCfgCoreldoVddLvl::Normal, Vsm::Sram1v1)),
        VddLevel::OverDriveMode => Some((ActiveCfgCoreldoVddLvl::Over, Vsm::Sram1v2)),
    };
    let (lp_drive, lp_ds) = match config.vdd_power.low_power_mode.drive {
        VddDriveStrength::Low { enable_bandgap } => (
            LowPowerDrive::Low {
                enable_detectors: enable_bandgap,
            },
            LpCfgCoreldoVddDs::Low,
        ),
        VddDriveStrength::Normal => (LowPowerDrive::Normal, LpCfgCoreldoVddDs::Normal),
    };
    let lp_level = match lp_power {
        VddLevel::MidDriveMode => LpCfgCoreldoVddLvl::Mid,
        #[cfg(feature = "mcxa5xx")]
        VddLevel::NormalMode => LpCfgCoreldoVddLvl::Normal,
        VddLevel::OverDriveMode => LpCfgCoreldoVddLvl::Over,
    };
    let active_drive = match config.vdd_power.active_mode.drive {
        VddDriveStrength::Low { enable_bandgap } => ActiveDrive::Low {
            enable_detectors: enable_bandgap,
            ds: ActiveCfgCoreldoVddDs::Low,
            bgmode: if enable_bandgap {
                ActiveCfgBgmode::Bgmode01
            } else {
                ActiveCfgBgmode::Bgmode0
            },
        },
        VddDriveStrength::Normal => ActiveDrive::Normal,
    };
    let (flash_wake, flash_doze) = match config.vdd_power.flash_sleep {
        FlashSleep::Never => (false, false),
        FlashSleep::FlashDoze => (false, true),
        FlashSleep::FlashDozeWithFlashWake => (true, true),
    };

    let voltage = VoltageProgram {
        active_level_change,
        lpwkup,
        low_power: LowPowerProgram {
            drive: lp_drive,
            ds: lp_ds,
            level: lp_level,
            bandgap: bandgap_lowpower,
        },
        active: ActiveProgram { drive: active_drive },
        core_sleep: config.vdd_power.core_sleep,
        flash_wake,
        flash_doze,
    };

    let mut clocks = Clocks {
        active_power,
        lp_power,
        bandgap_active,
        bandgap_lowpower,
        core_sleep: config.vdd_power.core_sleep,
        #[cfg(not(feature = "sosc-as-gpio"))]
        clk_in: None,
        fro_hf_root: None,
        fro_hf: None,
        clk_hf_fundamental: None,
        fro_hf_div: None,
        fro_12m_root: None,
        fro_12m: None,
        clk_1m: None,
        fro_lf_div: None,
        clk_16k_vsys: None,
        clk_16k_vdd_core: None,
        #[cfg(feature = "mcxa5xx")]
        clk_16k_vbat: None,
        #[cfg(all(feature = "mcxa5xx", not(feature = "rosc-32k-as-gpio")))]
        clk_32k_vsys: None,
        #[cfg(all(feature = "mcxa5xx", not(feature = "rosc-32k-as-gpio")))]
        clk_32k_vdd_core: None,
        #[cfg(all(feature = "mcxa5xx", not(feature = "rosc-32k-as-gpio")))]
        clk_32k_vbat: None,
        main_clk: None,
        cpu_system_clk: None,
        pll1_clk: None,
        pll1_clk_div: None,
    };

    //
    // SIRC: mirrors `configure_sirc_clocks_early`.
    //
    let sirc_freq = FRO_12M_FREQUENCY;
    let sirc_deep = match config.sirc.power {
        PoweredClock::NormalEnabledDeepSleepDisabled => false,
        PoweredClock::AlwaysEnabled => true,
    };
    let mut sirc_fro_lf_div_bits = None;
    clocks.fro_12m_root = Some(Clock {
        frequency: sirc_freq,
        power: config.sirc.power,
    });
    // clk_1m is *before* the fro_12m clock gate
    clocks.clk_1m = Some(Clock {
        frequency: clk_1m_frequency(sirc_freq),
        power: config.sirc.power,
    });
    if config.sirc.fro_12m_enabled {
        clocks.fro_12m = Some(Clock {
            frequency: sirc_freq,
            power: config.sirc.power,
        });
    }
    if let Some(d) = config.sirc.fro_lf_div.as_ref() {
        // We need `fro_lf` to be enabled
        if !config.sirc.fro_12m_enabled {
            return Err(ClockError::BadConfig {
                clock: "fro_lf_div",
                reason: "fro_12m not enabled",
            });
        }
        clocks.fro_lf_div = Some(Clock {
            frequency: divided_frequency(sirc_freq, *d),
            power: config.sirc.power,
        });
        sirc_fro_lf_div_bits = Some(d.into_bits());
    }

    // NOTE: `sirc_forced` is resolved directly from the requested configuration.
    // It deliberately is NOT inferred from `clocks.fro_12m.is_none()`, which is
    // ambiguous.
    let sirc = SircProgram {
        deep: sirc_deep,
        fro_lf_div_bits: sirc_fro_lf_div_bits,
        sirc_forced: !config.sirc.fro_12m_enabled,
    };

    //
    // FIRC: mirrors `configure_firc_clocks`.
    //
    if let Some(firc) = config.firc.as_ref() {
        let (base_freq, _sel) = firc.frequency.to_freq_and_sel();
        let limits = lowest_relevant_limits(active_power, lp_power, firc.power);

        clocks.fro_hf_root = Some(Clock {
            frequency: base_freq,
            power: firc.power,
        });

        if !bandgap_meets_requirement(bandgap_active, bandgap_lowpower, firc.power) {
            return Err(ClockError::BadConfig {
                clock: "fro_hf",
                reason: "bandgap required to be enabled when clock enabled",
            });
        }

        if firc.fro_hf_enabled {
            match validate_max_frequency(base_freq, limits.fro_hf, "fro_hf", "exceeds max") {
                Ok(()) => {}
                Err(e) => return Err(e),
            }
            clocks.fro_hf = Some(Clock {
                frequency: base_freq,
                power: firc.power,
            });
        }

        if firc.clk_hf_fundamental_enabled {
            clocks.clk_hf_fundamental = Some(Clock {
                frequency: CLK_HF_FUNDAMENTAL_FREQUENCY,
                power: firc.power,
            });
        }

        if let Some(d) = firc.fro_hf_div.as_ref() {
            // We need `fro_hf` to be enabled
            if !firc.fro_hf_enabled {
                return Err(ClockError::BadConfig {
                    clock: "fro_hf_div",
                    reason: "fro_hf not enabled",
                });
            }
            let div_freq = divided_frequency(base_freq, *d);
            match validate_max_frequency(div_freq, limits.fro_hf_div, "fro_hf_root", "exceeds max frequency") {
                Ok(()) => {}
                Err(e) => return Err(e),
            }
            clocks.fro_hf_div = Some(Clock {
                frequency: div_freq,
                power: firc.power,
            });
        }
    }

    //
    // FRO16K: mirrors `configure_fro16k_clocks`.
    //
    if let Some(fro16k) = config.fro16k.as_ref() {
        if fro16k.vsys_domain_active {
            clocks.clk_16k_vsys = Some(Clock {
                frequency: FRO_16K_FREQUENCY,
                power: PoweredClock::AlwaysEnabled,
            });
        }
        if fro16k.vdd_core_domain_active {
            clocks.clk_16k_vdd_core = Some(Clock {
                frequency: FRO_16K_FREQUENCY,
                power: PoweredClock::AlwaysEnabled,
            });
        }
        #[cfg(feature = "mcxa5xx")]
        if fro16k.vbat_domain_active {
            clocks.clk_16k_vbat = Some(Clock {
                frequency: FRO_16K_FREQUENCY,
                power: PoweredClock::AlwaysEnabled,
            });
        }
    }

    //
    // OSC32K: mirrors `configure_osc32k_clocks`. Must come after FRO16K, as it
    // depends on `clk_16k_vbat` being active.
    //
    #[cfg(all(feature = "mcxa5xx", feature = "unstable-osc32k", not(feature = "rosc-32k-as-gpio")))]
    if let Some(cfg) = config.osc32k.as_ref() {
        use super::config::Osc32KMode;

        // NOTE(AJM): "The FRO16K must be enabled before enabling the SRAM LDO or the bandgap"
        match clocks.ensure_clk_16k_vbat_active(&PoweredClock::AlwaysEnabled) {
            Ok(_) => {}
            Err(e) => return Err(e),
        }

        let power = match cfg.mode {
            Osc32KMode::HighPower { .. } => PoweredClock::NormalEnabledDeepSleepDisabled,
            Osc32KMode::LowPower { .. } => PoweredClock::AlwaysEnabled,
        };

        if cfg.vsys_domain_active {
            clocks.clk_32k_vsys = Some(Clock {
                frequency: OSC_32K_FREQUENCY,
                power,
            });
        }
        if cfg.vdd_core_domain_active {
            clocks.clk_32k_vdd_core = Some(Clock {
                frequency: OSC_32K_FREQUENCY,
                power,
            });
        }
        if cfg.vbat_domain_active {
            clocks.clk_32k_vbat = Some(Clock {
                frequency: OSC_32K_FREQUENCY,
                power,
            });
        }
    }

    //
    // SOSC: mirrors `configure_sosc`.
    //
    #[cfg(not(feature = "sosc-as-gpio"))]
    if let Some(parts) = config.sosc.as_ref() {
        // `configure_sosc` performs this check via `ensure_ldo_active`.
        if !bandgap_meets_requirement(bandgap_active, bandgap_lowpower, parts.power) {
            return Err(ClockError::BadConfig {
                clock: "sosc",
                reason: "LDO requires core bandgap enabled",
            });
        }

        match sosc_range(parts.frequency) {
            Ok(_) => {}
            Err(e) => return Err(e),
        }

        if !bandgap_meets_requirement(bandgap_active, bandgap_lowpower, parts.power) {
            return Err(ClockError::BadConfig {
                clock: "sosc",
                reason: "bandgap required",
            });
        }

        clocks.clk_in = Some(Clock {
            frequency: parts.frequency,
            power: parts.power,
        });
    }

    //
    // SPLL: mirrors `configure_spll`.
    //
    if let Some(cfg) = config.spll.as_ref() {
        // `configure_spll` performs this check via `ensure_ldo_active`.
        if !bandgap_meets_requirement(bandgap_active, bandgap_lowpower, cfg.power) {
            return Err(ClockError::BadConfig {
                clock: "spll",
                reason: "LDO requires core bandgap enabled",
            });
        }

        // match on the source, ensure it is active already
        let (src, missing) = match cfg.source {
            #[cfg(not(feature = "sosc-as-gpio"))]
            SpllSource::Sosc => (clocks.clk_in.as_ref(), "sosc not active"),
            SpllSource::Firc => (clocks.clk_hf_fundamental.as_ref(), "firc not active"),
            SpllSource::Sirc => (clocks.fro_12m.as_ref(), "sirc not active"),
        };
        // This checks if active
        let Some(clk) = src else {
            return Err(ClockError::BadConfig {
                clock: "spll",
                reason: missing,
            });
        };
        // This checks the correct power reqs
        if !clk.power.meets_requirement_of(&cfg.power) {
            return Err(ClockError::BadConfig {
                clock: "spll",
                reason: "needs low power source",
            });
        }
        if clk.frequency == 0 {
            return Err(ClockError::BadConfig {
                clock: "spll",
                reason: "internal error",
            });
        }

        let f_in = clk.frequency;

        // Calculate both Fout and Fcco so we can ensure they don't overflow
        // and are in range
        let fout: Option<u32>;
        let fcco: Option<u32>;

        match cfg.mode {
            // Fout = M x Fin
            SpllMode::Mode1a { m_mult } => {
                match check_spll_m(m_mult) {
                    Ok(_) => {}
                    Err(e) => return Err(e),
                }
                fcco = checked_spll_multiply(f_in, m_mult);
                fout = fcco;
            }
            // if !bypass_p2_div: Fout = (M / (2 x P)) x Fin
            // if  bypass_p2_div: Fout = (M /    P   ) x Fin
            SpllMode::Mode1b {
                m_mult,
                p_div,
                bypass_p2_div,
            } => {
                match check_spll_m(m_mult) {
                    Ok(_) => {}
                    Err(e) => return Err(e),
                }
                match check_spll_p(p_div) {
                    Ok(_) => {}
                    Err(e) => return Err(e),
                }
                let div = spll_post_divisor(p_div, bypass_p2_div);
                fcco = checked_spll_multiply(f_in, m_mult);
                fout = checked_spll_divide_then_multiply(f_in, div, m_mult);
            }
            // Fout = (M / N) x Fin
            SpllMode::Mode1c { m_mult, n_div } => {
                match check_spll_m(m_mult) {
                    Ok(_) => {}
                    Err(e) => return Err(e),
                }
                match check_spll_n(n_div) {
                    Ok(_) => {}
                    Err(e) => return Err(e),
                }
                fcco = checked_spll_divide_then_multiply(f_in, n_div as u32, m_mult);
                fout = fcco;
            }
            // if !bypass_p2_div: Fout = (M / (N x 2 x P)) x Fin
            // if  bypass_p2_div: Fout = (M / (  N x P  )) x Fin
            SpllMode::Mode1d {
                m_mult,
                n_div,
                p_div,
                bypass_p2_div,
            } => {
                match check_spll_m(m_mult) {
                    Ok(_) => {}
                    Err(e) => return Err(e),
                }
                match check_spll_p(p_div) {
                    Ok(_) => {}
                    Err(e) => return Err(e),
                }
                match check_spll_n(n_div) {
                    Ok(_) => {}
                    Err(e) => return Err(e),
                }
                let div = spll_pre_post_divisor(n_div, p_div, bypass_p2_div);
                fcco = checked_spll_divide_then_multiply(f_in, n_div as u32, m_mult);
                fout = checked_spll_divide_then_multiply(f_in, div, m_mult);
            }
        }

        // Ensure the Fcco and Fout calcs didn't overflow
        let fcco = match require_spll_fcco(fcco) {
            Ok(v) => v,
            Err(e) => return Err(e),
        };
        let fout = match require_spll_fout(fout) {
            Ok(v) => v,
            Err(e) => return Err(e),
        };

        // Fcco: 275MHz to 550MHz
        match validate_spll_fcco(fcco) {
            Ok(()) => {}
            Err(e) => return Err(e),
        }

        let limits = lowest_relevant_limits(active_power, lp_power, cfg.power);

        // Fout: 4.3MHz to 2x Max CPU Frequency
        match validate_spll_fout(fout, limits) {
            Ok(()) => {}
            Err(e) => return Err(e),
        }

        if !bandgap_meets_requirement(bandgap_active, bandgap_lowpower, cfg.power) {
            return Err(ClockError::BadConfig {
                clock: "spll",
                reason: "bandgap required when active",
            });
        }

        clocks.pll1_clk = Some(Clock {
            frequency: fout,
            power: cfg.power,
        });

        if let Some(d) = cfg.pll1_clk_div.as_ref() {
            let exp_freq = divided_frequency(fout, *d);
            match validate_max_frequency(exp_freq, limits.pll1_clk_div, "pll1_clk_div", "exceeds max frequency") {
                Ok(()) => {}
                Err(e) => return Err(e),
            }
            clocks.pll1_clk_div = Some(Clock {
                frequency: exp_freq,
                power: cfg.power,
            });
        }
    }

    //
    // Main clock: mirrors `configure_main_clk`.
    //
    let (name, src) = match config.main_clock.source {
        #[cfg(not(feature = "sosc-as-gpio"))]
        MainClockSource::SoscClkIn => ("clk_in", clocks.clk_in.as_ref()),
        MainClockSource::SircFro12M => ("fro_12m", clocks.fro_12m.as_ref()),
        MainClockSource::FircHfRoot => ("fro_hf_root", clocks.fro_hf_root.as_ref()),
        #[cfg(feature = "mcxa2xx")]
        MainClockSource::RoscFro16K => ("fro16k", clocks.clk_16k_vdd_core.as_ref()),
        #[cfg(all(feature = "mcxa5xx", not(feature = "rosc-32k-as-gpio")))]
        MainClockSource::RoscOsc32K => ("osc32k", clocks.clk_32k_vdd_core.as_ref()),
        MainClockSource::SPll1 => ("pll1_clk", clocks.pll1_clk.as_ref()),
    };
    let Some(main_clk_src) = src else {
        return Err(ClockError::BadConfig {
            clock: name,
            reason: "Needed for main_clock but not enabled",
        });
    };

    if !main_clk_src.power.meets_requirement_of(&config.main_clock.power) {
        return Err(ClockError::BadConfig {
            clock: name,
            reason: "Needed for main_clock but not low power",
        });
    }

    // NOTE: `main_clk` stores the SOURCE clock's power, not the requested power:
    // `config.main_clock.power` is only a minimum requirement, checked above.
    let main_freq = main_clk_src.frequency;
    let main_power = main_clk_src.power;

    // Is the main_clk source in range for main_clk?
    let lowest_limits = lowest_relevant_limits(active_power, lp_power, config.main_clock.power);
    match validate_max_frequency(main_freq, lowest_limits.main_clk, name, "Exceeds main_clock frequency") {
        Ok(()) => {}
        Err(e) => return Err(e),
    }

    // Calculate expected CPU frequency based on main_clk and AHB div
    let cpu_freq = divided_frequency(main_freq, config.main_clock.ahb_clk_div);

    // Is the expected CPU frequency in range for cpu_clk? Note: the CPU
    // is never running in deep sleep, so we directly use the active limits here
    match validate_max_frequency(
        cpu_freq,
        active_limits(active_power).cpu_clk,
        name,
        "Exceeds ahb max frequency",
    ) {
        Ok(()) => {}
        Err(e) => return Err(e),
    }

    clocks.main_clk = Some(Clock {
        frequency: main_freq,
        power: main_power,
    });
    clocks.cpu_system_clk = Some(Clock {
        frequency: cpu_freq,
        power: main_power,
    });

    Ok(ResolvedClockProgram { clocks, voltage, sirc })
}
