//! Pure clock-tree arithmetic and validation helpers.
//!
//! Everything in this module is a `const fn` or a `const` item: no register
//! access, no PAC types, no side effects. These items are extracted verbatim
//! from the `configure_*` methods in [`super::operator`] so that they can also
//! be evaluated at compile time.

use super::config::{Div8, VddLevel};
use super::types::{ClockError, PoweredClock};
use crate::chips::ClockLimits;

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
