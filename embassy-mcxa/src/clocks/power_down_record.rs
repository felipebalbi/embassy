//! Retained Power Down recovery record layout, encoding and decoding.
//!
//! Pure `u32` processing; [`super`] owns volatile access, synchronization,
//! publication order and canary observation. Value/complement pairs detect
//! zero/one residue and single-word damage, not freshness or authentication.
//! Writable SRAM can forge records, and correlated damage can pass checks.
//! The uniform-residue bound is `2^-64` per exact header; real SRAM need not
//! be uniform or independent.

/// Record length: two header words and 15 value/complement payload pairs.
pub const RECORD_WORDS: usize = 32;

/// Number of value/complement payload pairs, starting at word 2.
pub const PAYLOAD_PAIRS: usize = 15;

/// Word 0 of a v2 record.
pub const V2_HEADER_MAGIC: u32 = 0xD27A_6C91;

/// Word 1 of a v2 record: the exact bitwise complement of [`V2_HEADER_MAGIC`].
pub const V2_HEADER_COMPLEMENT: u32 = 0x2D85_936E;

/// Word 0 of the legacy two-word record.
pub const LEGACY_MAGIC: u32 = 0x9E37_79B9;

/// Word 1 of the legacy record: the exact complement of [`LEGACY_MAGIC`].
pub const LEGACY_COMPLEMENT: u32 = 0x61C8_8646;

/// The only payload version this image writes or accepts.
pub const RECORD_VERSION: u32 = 2;

/// High half of the cause word, so a zeroed or all-ones payload word can never
/// be mistaken for a valid cause code.
pub const CAUSE_TAG: u32 = 0x5044_0000;

/// Payload pair indices; register order matches sequential MMIO capture.
const PAIR_VERSION: usize = 0;
const PAIR_CAUSE: usize = 1;
const PAIR_CMC_SRS: usize = 2;
const PAIR_CMC_CKCTRL: usize = 3;
const PAIR_SCG_LDOCSR: usize = 4;
const PAIR_SCG_SPLLCSR: usize = 5;
const PAIR_SCG_CSR: usize = 6;
const PAIR_SCG_RCCR: usize = 7;
const PAIR_SCG_SIRCCSR: usize = 8;
const PAIR_SPC_SC: usize = 9;
const PAIR_SPC_PD_STATUS0: usize = 10;
const PAIR_SAVED_LDO: usize = 11;
const PAIR_SAVED_SPLLPWREN: usize = 12;
const PAIR_CONFIGURED_FLAGS: usize = 13;
const PAIR_CONTEXT_KNOWN_FLAGS: usize = 14;

/// `configured_flags` bit 0: SIRC / FRO12M was configured.
pub const CONFIGURED_SIRC: u32 = 1 << 0;
/// `configured_flags` bit 1: FIRC / FRO_HF was configured (MCXA2 only).
pub const CONFIGURED_FIRC: u32 = 1 << 1;
/// `configured_flags` bit 2: SOSC / `clk_in` was configured.
pub const CONFIGURED_SOSC: u32 = 1 << 2;
/// `configured_flags` bit 3: an SPLL was configured.
pub const CONFIGURED_SPLL: u32 = 1 << 3;

/// Every defined `configured_flags` bit. All other bits must be zero.
const CONFIGURED_FLAGS_MASK: u32 = CONFIGURED_SIRC | CONFIGURED_FIRC | CONFIGURED_SOSC | CONFIGURED_SPLL;

/// `context_known_flags` bit 0: the `saved_ldo` field carries a real value.
pub const CONTEXT_SAVED_LDO_KNOWN: u32 = 1 << 0;
/// `context_known_flags` bit 1: the `saved_spllpwren` field carries a real value.
pub const CONTEXT_SAVED_SPLLPWREN_KNOWN: u32 = 1 << 1;
/// `context_known_flags` bit 2: the `configured_flags` field carries a real value.
pub const CONTEXT_CONFIGURED_KNOWN: u32 = 1 << 2;

/// Every defined `context_known_flags` bit. All other bits must be zero.
const CONTEXT_FLAGS_MASK: u32 = CONTEXT_SAVED_LDO_KNOWN | CONTEXT_SAVED_SPLLPWREN_KNOWN | CONTEXT_CONFIGURED_KNOWN;

/// Failed masked-recovery predicate, not an electrical root cause.
/// Register capture is later and sequential; a timeout can precede a late lock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum PowerDownRecoveryCause {
    /// Step 1 context: no recorded `Clocks`, or no validated recovery budget.
    RecoveryContextInvalid,
    /// Step 1: the SPLL unlock, power-off write or relock was rejected.
    SpllDisableRejected,
    /// Step 2: a powered SPLL was saved against a disabled core LDO.
    SavedStateInconsistent,
    /// Step 3: restoring the saved `SCG_LDOCSR[LDOEN]` did not read back.
    LdoRestoreRejected,
    /// Step 4: `SCG_LDOCSR[VOUT_OK]` did not assert within the board budget.
    VoutTimeout,
    /// Step 5: `SCG_SIRCCSR[SIRCVLD]` did not assert within the board budget.
    SircTimeout,
    /// Step 5: `SCG_FIRCCSR[FIRCVLD]` did not assert within the board budget.
    FircTimeout,
    /// Step 5: `SCG_SOSCCSR[SOSCVLD]` did not assert within the board budget.
    SoscTimeout,
    /// Step 6: the SPLL unlock, power-restore write or relock was rejected.
    SpllRestoreRejected,
    /// Step 7: `SCG_SPLLCSR[SPLL_LOCK]` did not assert within the board budget.
    SpllLockTimeout,
    /// `SPC.PD_STATUS0[PD_LP_REQ]` did not clear.
    SpcPdRequestStuck,
    /// `SPC.SC[SPC_LP_REQ]` did not clear.
    SpcRequestStuck,
    /// The WUU could not be handed its hardware back.
    WuuReplayFailed,
    /// Recognised record without a specific cause: legacy, invalid payload,
    /// unknown version/code, or missing/corrupt pre-arm canary.
    Unavailable,
}

impl PowerDownRecoveryCause {
    /// Stable diagnostic identifier, `21..=34`; rendering belongs to the caller.
    pub const fn code(self) -> u32 {
        match self {
            Self::RecoveryContextInvalid => 21,
            Self::SpllDisableRejected => 22,
            Self::SavedStateInconsistent => 23,
            Self::LdoRestoreRejected => 24,
            Self::VoutTimeout => 25,
            Self::SircTimeout => 26,
            Self::FircTimeout => 27,
            Self::SoscTimeout => 28,
            Self::SpllRestoreRejected => 29,
            Self::SpllLockTimeout => 30,
            Self::SpcPdRequestStuck => 31,
            Self::SpcRequestStuck => 32,
            Self::WuuReplayFailed => 33,
            Self::Unavailable => 34,
        }
    }

    /// Decode an untrusted numeric identifier; unknown values yield
    /// [`Self::Unavailable`]. Do not transmute retained words into an enum.
    pub const fn from_code(code: u32) -> Self {
        match code {
            21 => Self::RecoveryContextInvalid,
            22 => Self::SpllDisableRejected,
            23 => Self::SavedStateInconsistent,
            24 => Self::LdoRestoreRejected,
            25 => Self::VoutTimeout,
            26 => Self::SircTimeout,
            27 => Self::FircTimeout,
            28 => Self::SoscTimeout,
            29 => Self::SpllRestoreRejected,
            30 => Self::SpllLockTimeout,
            31 => Self::SpcPdRequestStuck,
            32 => Self::SpcRequestStuck,
            33 => Self::WuuReplayFailed,
            _ => Self::Unavailable,
        }
    }
}

/// Sequential register capture, not a simultaneous coherent hardware image.
/// SPC and WUU failures are captured after IO de-isolation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct PowerDownRecoverySnapshot {
    cmc_srs: u32,
    cmc_ckctrl: u32,
    scg_ldocsr: u32,
    scg_spllcsr: u32,
    scg_csr: u32,
    scg_rccr: u32,
    scg_sirccsr: u32,
    spc_sc: u32,
    spc_pd_status0: u32,
    saved_ldo: Option<bool>,
    saved_spllpwren: Option<bool>,
    configured_flags: Option<u32>,
}

impl PowerDownRecoverySnapshot {
    /// `CMC.SRS` as read before the fail-stop reset.
    pub const fn cmc_srs(&self) -> u32 {
        self.cmc_srs
    }
    /// `CMC.CKCTRL` as read before the fail-stop reset.
    pub const fn cmc_ckctrl(&self) -> u32 {
        self.cmc_ckctrl
    }
    /// `SCG.LDOCSR` as read before the fail-stop reset.
    pub const fn scg_ldocsr(&self) -> u32 {
        self.scg_ldocsr
    }
    /// `SCG.SPLLCSR` as read before the fail-stop reset.
    pub const fn scg_spllcsr(&self) -> u32 {
        self.scg_spllcsr
    }
    /// `SCG.CSR` as read before the fail-stop reset.
    pub const fn scg_csr(&self) -> u32 {
        self.scg_csr
    }
    /// `SCG.RCCR` as read before the fail-stop reset.
    pub const fn scg_rccr(&self) -> u32 {
        self.scg_rccr
    }
    /// `SCG.SIRCCSR` as read before the fail-stop reset.
    pub const fn scg_sirccsr(&self) -> u32 {
        self.scg_sirccsr
    }
    /// `SPC.SC` as read before the fail-stop reset.
    pub const fn spc_sc(&self) -> u32 {
        self.spc_sc
    }
    /// `SPC.PD_STATUS0` as read before the fail-stop reset.
    pub const fn spc_pd_status0(&self) -> u32 {
        self.spc_pd_status0
    }
    /// The `LDOEN` value Power Down entry saved, when it was known.
    ///
    /// `None` means unknown, not a saved off state.
    pub const fn saved_ldo(&self) -> Option<bool> {
        self.saved_ldo
    }
    /// The retained `SPLLCSR[SPLLPWREN]` the transaction saved, when known.
    pub const fn saved_spllpwren(&self) -> Option<bool> {
        self.saved_spllpwren
    }
    /// Raw captured source-configuration flags, when known.
    ///
    /// For named source queries, use [`Self::sirc_configured`],
    /// [`Self::firc_configured`], [`Self::sosc_configured`] and
    /// [`Self::spll_configured`]. `None` means unknown, not all sources disabled.
    pub const fn configured_flags(&self) -> Option<u32> {
        self.configured_flags
    }

    /// Whether SIRC / FRO12M was configured. `None` means unknown.
    pub const fn sirc_configured(&self) -> Option<bool> {
        match self.configured_flags {
            Some(flags) => Some(flags & CONFIGURED_SIRC != 0),
            None => None,
        }
    }

    /// Whether FIRC / FRO_HF (MCXA2 only) was configured. `None` means unknown.
    ///
    /// This query is available on both families for the uniform snapshot format.
    pub const fn firc_configured(&self) -> Option<bool> {
        match self.configured_flags {
            Some(flags) => Some(flags & CONFIGURED_FIRC != 0),
            None => None,
        }
    }

    /// Whether SOSC / `clk_in` was configured. `None` means unknown.
    pub const fn sosc_configured(&self) -> Option<bool> {
        match self.configured_flags {
            Some(flags) => Some(flags & CONFIGURED_SOSC != 0),
            None => None,
        }
    }

    /// Whether an SPLL was configured. `None` means unknown.
    pub const fn spll_configured(&self) -> Option<bool> {
        match self.configured_flags {
            Some(flags) => Some(flags & CONFIGURED_SPLL != 0),
            None => None,
        }
    }
}

/// A consumed fail-stop record with a recognised, now-cleared header.
/// A specific stage is available only when [`Self::cause`] is not `Unavailable`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct PowerDownFailStopReport {
    cause: PowerDownRecoveryCause,
    snapshot: Option<PowerDownRecoverySnapshot>,
}

impl PowerDownFailStopReport {
    /// Which recovery predicate refused, or
    /// [`PowerDownRecoveryCause::Unavailable`].
    pub const fn cause(&self) -> PowerDownRecoveryCause {
        self.cause
    }

    /// The captured register snapshot, present only alongside a specific cause.
    pub const fn snapshot(&self) -> Option<PowerDownRecoverySnapshot> {
        self.snapshot
    }

    /// The aggregate report: recognised, but nothing specific may be claimed.
    const fn unavailable() -> Self {
        Self {
            cause: PowerDownRecoveryCause::Unavailable,
            snapshot: None,
        }
    }
}

/// Recovery context supplied without a nested `CLOCKS` borrow.
/// Unknown fields are `None`, encoded as zero with their known-flag cleared.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PowerDownRecoveryContext {
    /// The `LDOEN` value Power Down entry saved.
    pub saved_ldo: Option<bool>,
    /// The retained `SPLLCSR[SPLLPWREN]` the recovery transaction saved.
    pub saved_spllpwren: Option<bool>,
    /// Which sources were configured, as [`CONFIGURED_SIRC`] and friends.
    pub configured_flags: Option<u32>,
}

/// Index of the value word of payload pair `pair`.
const fn value_index(pair: usize) -> usize {
    2 + pair * 2
}

/// Build the complete 32-word record for one fail-stop.
///
/// `registers` holds the nine captured words in this order:
/// `CMC.SRS`, `CMC.CKCTRL`, `SCG.LDOCSR`, `SCG.SPLLCSR`, `SCG.CSR`,
/// `SCG.RCCR`, `SCG.SIRCCSR`, `SPC.SC`, `SPC.PD_STATUS0`.
///
/// The caller controls ordered publication; this only encodes contents.
pub fn encode(
    cause: PowerDownRecoveryCause,
    registers: &[u32; 9],
    context: &PowerDownRecoveryContext,
) -> [u32; RECORD_WORDS] {
    let mut words = [0u32; RECORD_WORDS];

    let mut known = 0u32;
    if context.saved_ldo.is_some() {
        known |= CONTEXT_SAVED_LDO_KNOWN;
    }
    if context.saved_spllpwren.is_some() {
        known |= CONTEXT_SAVED_SPLLPWREN_KNOWN;
    }
    if context.configured_flags.is_some() {
        known |= CONTEXT_CONFIGURED_KNOWN;
    }

    let values = [
        RECORD_VERSION,
        CAUSE_TAG | cause.code(),
        registers[0],
        registers[1],
        registers[2],
        registers[3],
        registers[4],
        registers[5],
        registers[6],
        registers[7],
        registers[8],
        bool_word(context.saved_ldo),
        bool_word(context.saved_spllpwren),
        context.configured_flags.unwrap_or(0) & CONFIGURED_FLAGS_MASK,
        known,
    ];

    let mut pair = 0;
    while pair < PAYLOAD_PAIRS {
        let at = value_index(pair);
        words[at] = values[pair];
        words[at + 1] = !values[pair];
        pair += 1;
    }

    words[0] = V2_HEADER_MAGIC;
    words[1] = V2_HEADER_COMPLEMENT;
    words
}

/// Encode an optional boolean as the canonical `0`/`1` word, with `None`
/// encoded as a zero value alongside a cleared known-flag.
const fn bool_word(value: Option<bool>) -> u32 {
    match value {
        Some(true) => 1,
        Some(false) | None => 0,
    }
}

/// Does `words` carry either recognised exact header pair?
///
/// Recognition is deliberately header-only, so a caller can refuse Power Down
/// on an unconsumed record without interpreting — or disturbing — its payload.
pub const fn header_is_recognised(words: &[u32; RECORD_WORDS]) -> bool {
    header_is_v2(words) || header_is_legacy(words)
}

/// Does `words` carry the exact v2 header pair?
pub const fn header_is_v2(words: &[u32; RECORD_WORDS]) -> bool {
    words[0] == V2_HEADER_MAGIC && words[1] == V2_HEADER_COMPLEMENT
}

/// Does `words` carry the exact legacy header pair?
///
/// Only the two original words are interpreted; trailing residue is not payload.
pub const fn header_is_legacy(words: &[u32; RECORD_WORDS]) -> bool {
    words[0] == LEGACY_MAGIC && words[1] == LEGACY_COMPLEMENT
}

/// Decode using the boot's latched pre-arm `canary_valid` observation.
///
/// Returns `None` for unrecognised headers; the caller must not clear them.
/// Recognised legacy, invalid or unknown payloads, or an invalid canary, yield
/// `Unavailable`. A specific cause/snapshot requires a valid canary and v2 payload.
pub fn decode(words: &[u32; RECORD_WORDS], canary_valid: bool) -> Option<PowerDownFailStopReport> {
    if header_is_legacy(words) {
        // Legacy has no payload to interpret.
        return Some(PowerDownFailStopReport::unavailable());
    }

    if !header_is_v2(words) {
        return None;
    }

    // A recognised header with invalid payload is Unavailable, not absent.
    let mut fields = [0u32; PAYLOAD_PAIRS];
    let mut pair = 0;
    while pair < PAYLOAD_PAIRS {
        let at = value_index(pair);
        let value = words[at];
        let complement = words[at + 1];
        if complement != !value {
            return Some(PowerDownFailStopReport::unavailable());
        }
        fields[pair] = value;
        pair += 1;
    }

    if fields[PAIR_VERSION] != RECORD_VERSION {
        return Some(PowerDownFailStopReport::unavailable());
    }

    let cause_word = fields[PAIR_CAUSE];
    if cause_word & 0xFFFF_0000 != CAUSE_TAG {
        return Some(PowerDownFailStopReport::unavailable());
    }
    let cause = PowerDownRecoveryCause::from_code(cause_word & 0x0000_FFFF);
    if matches!(cause, PowerDownRecoveryCause::Unavailable) {
        // Do not attribute an unknown cause to a known stage.
        return Some(PowerDownFailStopReport::unavailable());
    }

    let known = fields[PAIR_CONTEXT_KNOWN_FLAGS];
    if known & !CONTEXT_FLAGS_MASK != 0 {
        return Some(PowerDownFailStopReport::unavailable());
    }

    let configured_flags_word = fields[PAIR_CONFIGURED_FLAGS];
    if configured_flags_word & !CONFIGURED_FLAGS_MASK != 0 {
        return Some(PowerDownFailStopReport::unavailable());
    }

    let Some(saved_ldo) = decode_optional_bool(fields[PAIR_SAVED_LDO], known & CONTEXT_SAVED_LDO_KNOWN != 0) else {
        return Some(PowerDownFailStopReport::unavailable());
    };
    let Some(saved_spllpwren) =
        decode_optional_bool(fields[PAIR_SAVED_SPLLPWREN], known & CONTEXT_SAVED_SPLLPWREN_KNOWN != 0)
    else {
        return Some(PowerDownFailStopReport::unavailable());
    };

    let configured_known = known & CONTEXT_CONFIGURED_KNOWN != 0;
    if !configured_known && configured_flags_word != 0 {
        // Unknown fields must have a zero value word.
        return Some(PowerDownFailStopReport::unavailable());
    }
    let configured_flags = if configured_known {
        Some(configured_flags_word)
    } else {
        None
    };

    if !canary_valid {
        // Payload integrity alone does not establish section retention.
        return Some(PowerDownFailStopReport::unavailable());
    }

    Some(PowerDownFailStopReport {
        cause,
        snapshot: Some(PowerDownRecoverySnapshot {
            cmc_srs: fields[PAIR_CMC_SRS],
            cmc_ckctrl: fields[PAIR_CMC_CKCTRL],
            scg_ldocsr: fields[PAIR_SCG_LDOCSR],
            scg_spllcsr: fields[PAIR_SCG_SPLLCSR],
            scg_csr: fields[PAIR_SCG_CSR],
            scg_rccr: fields[PAIR_SCG_RCCR],
            scg_sirccsr: fields[PAIR_SCG_SIRCCSR],
            spc_sc: fields[PAIR_SPC_SC],
            spc_pd_status0: fields[PAIR_SPC_PD_STATUS0],
            saved_ldo,
            saved_spllpwren,
            configured_flags,
        }),
    })
}

/// Decode one canonical boolean field.
///
/// Returns `None` for a non-canonical encoding: a value other than `0` or `1`,
/// or any non-zero value beside a cleared known-flag.
const fn decode_optional_bool(value: u32, known: bool) -> Option<Option<bool>> {
    if !known {
        return if value == 0 { Some(None) } else { None };
    }
    match value {
        0 => Some(Some(false)),
        1 => Some(Some(true)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{PowerDownRecoveryCause, PowerDownRecoveryContext, RECORD_WORDS, decode, encode};

    fn configured_record(configured_flags: Option<u32>) -> [u32; RECORD_WORDS] {
        encode(
            PowerDownRecoveryCause::VoutTimeout,
            &[0x101, 0x202, 0x303, 0x404, 0x505, 0x606, 0x707, 0x808, 0x909],
            &PowerDownRecoveryContext {
                saved_ldo: Some(true),
                saved_spllpwren: Some(false),
                configured_flags,
            },
        )
    }

    #[test]
    fn unknown_configuration_is_not_reported_as_unconfigured() {
        let report = decode(&configured_record(None), true).unwrap();
        assert_eq!(report.cause(), PowerDownRecoveryCause::VoutTimeout);
        let snapshot = report.snapshot().unwrap();

        assert_eq!(snapshot.configured_flags(), None);
        assert_eq!(snapshot.sirc_configured(), None);
        assert_eq!(snapshot.firc_configured(), None);
        assert_eq!(snapshot.sosc_configured(), None);
        assert_eq!(snapshot.spll_configured(), None);
    }

    #[test]
    fn known_configuration_reports_each_source_independently() {
        for mask in 0u32..=15 {
            let report = decode(&configured_record(Some(mask)), true).unwrap();
            assert_eq!(report.cause(), PowerDownRecoveryCause::VoutTimeout);
            let snapshot = report.snapshot().unwrap();

            assert_eq!(snapshot.configured_flags(), Some(mask));
            assert_eq!(snapshot.sirc_configured(), Some(mask & 0b0001 != 0), "mask {mask}");
            assert_eq!(snapshot.firc_configured(), Some(mask & 0b0010 != 0), "mask {mask}");
            assert_eq!(snapshot.sosc_configured(), Some(mask & 0b0100 != 0), "mask {mask}");
            assert_eq!(snapshot.spll_configured(), Some(mask & 0b1000 != 0), "mask {mask}");
        }
    }

    #[test]
    fn damaged_complements_do_not_expose_configuration() {
        for configured_flags in core::iter::once(None).chain((0u32..=15).map(Some)) {
            let record = configured_record(configured_flags);
            for complement_index in (3..RECORD_WORDS).step_by(2) {
                let mut damaged = record;
                damaged[complement_index] ^= 1;
                let report = decode(&damaged, true).unwrap();

                assert_eq!(report.cause(), PowerDownRecoveryCause::Unavailable);
                assert_eq!(report.snapshot(), None);
            }
        }
    }
}
