//! Deep sleep and Power Down entry, exit, and clock recovery.

use core::cell::Ref;
use core::ops::Deref;

use critical_section::CriticalSection;

use super::types::{Clocks, PoweredClock};
use super::{CLOCKS, PowerDownRecoveryCause, PowerDownRecoveryContext};
use crate::pac;
use crate::pac::cmc::{Ckmode, PmctrlmainLpmode};
#[cfg(feature = "mcxa2xx")]
use crate::pac::scg::Fircvld;
use crate::pac::scg::{Sircvld, SpllLock, SpllcsrLk};
use crate::pac::spc::{PdLpReq, SpcLpReq};

/// Attempt to go to deep sleep if possible.
///
/// If we successfully went and returned from deep sleep, this function returns a `true`.
/// If we were unsuccessful due to active `WakeGuard`s, this function returns a `false`.
///
/// ## SAFETY
///
/// Care must be taken that we have ensured that the system is ready to go to deep
/// sleep, otherwise HAL peripherals may misbehave. `crate::clocks::init()` must
/// have been called and returned successfully, with a `CoreSleep` configuration
/// set to DeepSleep (or lower).
pub unsafe fn deep_sleep_if_possible(cs: &CriticalSection) -> bool {
    let inhibit = crate::clocks::active_wake_guards(cs);
    if inhibit {
        return false;
    }

    unsafe {
        // Yep, it's time to go to deep sleep. WHILE STILL IN the CS, get ready
        setup_deep_sleep();

        // Here we go!
        //
        // It is okay to WFE with interrupts disabled: we have enabled SEVONPEND
        cortex_m::asm::dsb();
        cortex_m::asm::wfe();

        // Wakey wakey, eggs and bakey
        recover_deep_sleep(cs);
    }

    true
}

/// Prepare the system for deep sleep
///
/// ## SAFETY
///
/// Care must be taken that we have ensured that the system is ready to go to deep
/// sleep, otherwise HAL peripherals may misbehave. `crate::clocks::init()` must
/// have been called and returned successfully, with a `CoreSleep` configuration
/// set to DeepSleep (or lower).
unsafe fn setup_deep_sleep() {
    let cmc = nxp_pac::CMC;

    // To configure for Deep Sleep Low-Power mode entry:
    //
    // Write Fh to Clock Control (CKCTRL)
    cmc.ckctrl().modify(|w| w.set_ckmode(Ckmode::Ckmode1111));
    // Write 1h to Power Mode Protection (PMPROT)
    cmc.pmprot().write(|w| w.0 = 1);
    // Write 1h to Global Power Mode Control (GPMCTRL)
    cmc.gpmctrl().modify(|w| w.set_lpmode(0b0001));
    // Redundant?
    // cmc.pmctrlmain().modify(|w| w.set_lpmode(PmctrlmainLpmode::LPMODE0001));

    // From the C SDK:
    //
    // Before executing WFI instruction read back the last register to
    // ensure all registers writes have completed.
    let _ = cmc.gpmctrl().read();
}

/// Start back up after deep sleep returns
///
/// ## SAFETY
///
/// Care must be taken that we have ensured that the system is ready to go to deep
/// sleep, otherwise HAL peripherals may misbehave. `crate::clocks::init()` must
/// have been called and returned successfully, with a `CoreSleep` configuration
/// set to DeepSleep (or lower).
unsafe fn recover_deep_sleep(cs: &CriticalSection) {
    let cmc = nxp_pac::CMC;

    // Restart any necessary clocks
    unsafe {
        restart_active_only_clocks(cs);
    }

    // Re-raise the sleep level to WFE sleep in the off chance that the
    // user decides to call `wfe` on their own accord, and to avoid having
    // to re-set if we chill in WFE sleep mostly
    cmc.ckctrl().modify(|w| w.set_ckmode(Ckmode::Ckmode0001));
}

/// Perform any actions necessary to re-initialize clocks after returning to active
/// mode after a low power (e.g. deep sleep, power-off) state.
///
/// ## Safety
///
/// This should only be called in a critical section, immediately after waking up.
unsafe fn restart_active_only_clocks(_cs: &CriticalSection) {
    let bref: Ref<'_, Option<Clocks>> = CLOCKS.borrow_ref(*_cs);
    let dref: &Option<Clocks> = bref.deref();
    let Some(clocks) = dref else {
        return;
    };
    let scg = pac::SCG0;

    // TODO: Restart clock monitors if necessary? Needs to be re-enabled
    // AFTER FRO12M has been started, and probably after clocks are
    // valid again.
    //
    // TODO: Timeout? Check error fields (at least for SPLL)? Clear
    // or reset any status bits?

    // Ensure FRO12M is up and running
    if let Some(fro12m) = clocks.fro_12m_root.as_ref()
        && !matches!(fro12m.power, PoweredClock::AlwaysEnabled)
    {
        while scg.sirccsr().read().sircvld() != Sircvld::EnabledAndValid {}
    }

    // Ensure FRO45M is up and running
    #[cfg(feature = "mcxa2xx")]
    if let Some(frohf) = clocks.fro_hf_root.as_ref()
        && !matches!(frohf.power, PoweredClock::AlwaysEnabled)
    {
        while scg.firccsr().read().fircvld() != Fircvld::EnabledAndValid {}
    }

    // Ensure SOSC is up and running
    #[cfg(not(feature = "sosc-as-gpio"))]
    if let Some(clk_in) = clocks.clk_in.as_ref()
        && !matches!(clk_in.power, PoweredClock::AlwaysEnabled)
    {
        while !scg.sosccsr().read().soscvld() {}
    }

    // Ensure SPLL is up and running
    if let Some(spll) = clocks.pll1_clk.as_ref()
        && !matches!(spll.power, PoweredClock::AlwaysEnabled)
    {
        while scg.spllcsr().read().spll_lock() != SpllLock::EnabledAndValid {}
    }
}

/// Fallback core frequency when no main clock is recorded. A slower actual
/// core lengthens delays. 150 MHz is the RM's maximum SCG input frequency.
const ASSUMED_MAX_CORE_HZ: u32 = 150_000_000;

/// Poll at most `steps` times, delaying `step_cycles` after each failed check.
/// Cycle delays are nominal minimums, not elapsed-time upper bounds.
fn poll_until_valid(steps: u32, step_cycles: u32, mut ready: impl FnMut() -> bool) -> bool {
    let mut taken = 0u32;
    while taken < steps {
        if ready() {
            return true;
        }
        cortex_m::asm::delay(step_cycles);
        taken += 1;
    }
    false
}

/// Attempt Power Down with no active clock votes and a live CPU-resuming WUU
/// source. Returns `true` after entry and recovery, or `false` on refusal so
/// the caller can fall back to ordinary WFE. Setup refusal restores LDO/CKCTRL;
/// PMPROT remains restricted to Power Down.
///
/// # Safety
/// [`super::init`] must have succeeded with `CoreSleep::PowerDown`.
/// The caller must keep interrupts masked throughout entry and recovery.
pub unsafe fn power_down_if_possible(cs: &CriticalSection) -> bool {
    // Check clock votes and hardware wake eligibility before entry writes.
    if crate::clocks::active_wake_guards(cs) {
        return false;
    }

    if !crate::wuu::power_down_preflight(cs) {
        crate::perf_counters::incr_power_down_no_live_wake();
        return false;
    }

    let mut saved_ldoen = false;

    // SAFETY: initialized for Power Down, preflight passed, interrupts masked.
    unsafe {
        if !setup_power_down(&mut saved_ldoen) {
            return false;
        }

        // setup_power_down issued DSB; SEVONPEND allows masked WFE to wake.
        cortex_m::asm::wfe();
        recover_power_down(cs, saved_ldoen);
    }

    true
}

/// Prepare Power Down and save the prior LDOEN state. Success includes DSB;
/// failure restores LDO/CKCTRL and must not be followed by Power Down WFE.
///
/// # Safety
/// Interrupts must be masked and [`power_down_if_possible`] preflight passed.
unsafe fn setup_power_down(saved_ldoen: &mut bool) -> bool {
    let cmc = nxp_pac::CMC;
    let scg = pac::SCG0;

    // Save LDOEN for exact restoration after wake.
    let prev = scg.ldocsr().read().ldoen();
    *saved_ldoen = prev;
    scg.ldocsr().modify(|w| w.set_ldoen(false));

    cmc.ckctrl().modify(|w| w.set_ckmode(Ckmode::Ckmode1111));

    // Authorize only Power Down, then request it.
    cmc.pmprot().write(|w| w.0 = 0x0000_0002);
    cmc.gpmctrl().modify(|w| w.set_lpmode(0b0011));

    // PMPROT-blocked writes may be ignored. Verify the latch before WFE.
    if cmc.pmctrlmain().read().lpmode() != PmctrlmainLpmode::Lpmode0011 {
        cmc.ckctrl().modify(|w| w.set_ckmode(Ckmode::Ckmode0001));
        scg.ldocsr().modify(|w| w.set_ldoen(prev));
        crate::perf_counters::incr_power_down_setup_aborts();
        return false;
    }

    // Complete register writes before entering the low-power state.
    let _ = cmc.gpmctrl().read();
    cortex_m::asm::dsb();

    true
}

/// Restore clocks before de-isolating IO, clear SPC requests, then replay WUU.
/// Power Down retains PORT/GPIO state; Deep Power Down pad restoration does
/// not apply. Clock failures occur before ISO_CLR; SPC/WUU failures occur after.
///
/// # Safety
/// Called masked immediately after [`power_down_if_possible`]'s WFE returns.
unsafe fn recover_power_down(cs: &CriticalSection, saved_ldo: bool) {
    let cmc = nxp_pac::CMC;
    let spc = pac::SPC0;

    // Pass context to later fail-stops without borrowing CLOCKS again.
    let context = unsafe { restart_all_configured_clocks(cs, saved_ldo) };

    // W1C pure-mask writes preserve unrelated flags; do not use modify.
    // The SDK clears isolation for Power Down as well as Deep Power Down.
    spc.sc().write(|w| w.set_iso_clr(true));

    spc.pd_status0().write(|w| w.set_pd_lp_req(PdLpReq::ReqYes));
    if spc.pd_status0().read().pd_lp_req() != PdLpReq::ReqNo {
        crate::perf_counters::incr_power_down_spc_failures();
        super::power_down_fail_stop_reset(cs, PowerDownRecoveryCause::SpcPdRequestStuck, context);
    }
    spc.sc().write(|w| w.set_spc_lp_req(SpcLpReq::LowPower));
    if spc.sc().read().spc_lp_req() != SpcLpReq::Active {
        crate::perf_counters::incr_power_down_spc_failures();
        super::power_down_fail_stop_reset(cs, PowerDownRecoveryCause::SpcRequestStuck, context);
    }

    // SAFETY: interrupts are masked and required clocks have been restored.
    if unsafe { crate::wuu::recover_after_power_down(cs) }.is_err() {
        crate::perf_counters::incr_power_down_wuu_replay_failures();
        super::power_down_fail_stop_reset(cs, PowerDownRecoveryCause::WuuReplayFailed, context);
    }

    cmc.ckctrl().modify(|w| w.set_ckmode(Ckmode::Ckmode0001));
}

/// Restore LDO and every configured source; unlike DeepSleep, Power Down
/// requires revalidation even for sources marked AlwaysEnabled.
///
/// SPLL ordering: save SPLLPWREN, disable it, restore LDOEN, wait for VOUT_OK
/// and references, restore saved SPLLPWREN, then wait for lock if powered.
/// Every stage uses the board budget independently, with bounded polling.
/// Five nominal budgets are not a physical elapsed-time ceiling: cycle delays
/// take longer when the recovering core runs below its configured frequency.
///
/// # Safety
/// Called in a critical section immediately after Power Down wake.
unsafe fn restart_all_configured_clocks(cs: &CriticalSection, saved_ldo: bool) -> PowerDownRecoveryContext {
    // Polling granularity; the per-step delay supplies the nominal budget.
    const POLL_STEPS: u32 = 1000;

    // SPLLERR is W1C; force it to zero on writes to preserve its pending state.
    const SPLLCSR_W1C_MASK: u32 = 1 << 26;

    let scg = pac::SCG0;

    // Publish a predicate-specific failure before de-isolation, then reset.
    fn fail_stop(cs: &CriticalSection, cause: PowerDownRecoveryCause, context: PowerDownRecoveryContext) -> ! {
        crate::perf_counters::incr_power_down_clock_timeouts();
        super::power_down_fail_stop_reset(cs, cause, context)
    }

    // Unknown context stays None rather than claiming a saved false value.
    let mut context = PowerDownRecoveryContext {
        saved_ldo: Some(saved_ldo),
        saved_spllpwren: None,
        configured_flags: None,
    };

    let bref: Ref<'_, Option<Clocks>> = CLOCKS.borrow_ref(*cs);
    let dref: &Option<Clocks> = bref.deref();
    let Some(clocks) = dref else {
        scg.ldocsr().modify(|w| w.set_ldoen(saved_ldo));
        fail_stop(cs, PowerDownRecoveryCause::RecoveryContextInvalid, context);
    };

    let mut configured_flags = 0u32;
    if clocks.fro_12m_root.is_some() {
        configured_flags |= super::CONFIGURED_SIRC;
    }
    #[cfg(feature = "mcxa2xx")]
    if clocks.fro_hf_root.is_some() {
        configured_flags |= super::CONFIGURED_FIRC;
    }
    #[cfg(not(feature = "sosc-as-gpio"))]
    if clocks.clk_in.is_some() {
        configured_flags |= super::CONFIGURED_SOSC;
    }
    if clocks.pll1_clk.is_some() {
        configured_flags |= super::CONFIGURED_SPLL;
    }
    context.configured_flags = Some(configured_flags);

    let Some(budget_us) = clocks.power_down_clock_recovery_budget_us else {
        fail_stop(cs, PowerDownRecoveryCause::RecoveryContextInvalid, context);
    };

    let core_hz = clocks
        .main_clk
        .as_ref()
        .map_or(ASSUMED_MAX_CORE_HZ, |main_clk| main_clk.frequency);

    // ceil(budget_us * core_hz / (1_000_000 * POLL_STEPS)); use u64 to avoid wrap.
    let divisor = 1_000_000u64 * u64::from(POLL_STEPS);
    let step_cycles = u64::from(budget_us) * u64::from(core_hz);
    let step_cycles = step_cycles.div_ceil(divisor);
    let step_cycles = u32::try_from(step_cycles).unwrap_or(u32::MAX);

    // Unlock separately and verify before changing power. Restore the saved
    // lock even if the power write fails; verify both final fields.
    let set_spll_pwren = |want: bool| -> bool {
        let before = scg.spllcsr().read();
        let saved_lk = before.lk();

        if saved_lk == SpllcsrLk::WriteDisabled {
            let mut unlocked = before;
            unlocked.set_lk(SpllcsrLk::WriteEnabled);
            scg.spllcsr().write(|w| w.0 = unlocked.0 & !SPLLCSR_W1C_MASK);

            if scg.spllcsr().read().lk() != SpllcsrLk::WriteEnabled {
                return false;
            }
        }

        let mut wanted = scg.spllcsr().read();
        wanted.set_spllpwren(want);
        scg.spllcsr().write(|w| w.0 = wanted.0 & !SPLLCSR_W1C_MASK);

        let power_took = scg.spllcsr().read().spllpwren() == want;

        let mut relocked = scg.spllcsr().read();
        relocked.set_lk(saved_lk);
        scg.spllcsr().write(|w| w.0 = relocked.0 & !SPLLCSR_W1C_MASK);

        let after = scg.spllcsr().read();
        power_took && after.lk() == saved_lk && after.spllpwren() == want
    };

    let spll_configured = clocks.pll1_clk.is_some();

    // Disable retained SPLLPWREN before raising LDOEN. Without a configured
    // SPLL, this transaction does not write SPLLCSR.
    let saved_spllpwren = if spll_configured {
        let saved = scg.spllcsr().read().spllpwren();
        context.saved_spllpwren = Some(saved);
        if !set_spll_pwren(false) {
            fail_stop(cs, PowerDownRecoveryCause::SpllDisableRejected, context);
        }
        saved
    } else {
        false
    };

    // A powered SPLL requires an enabled LDO; reject inconsistent saved state.
    if spll_configured && saved_spllpwren && !saved_ldo {
        fail_stop(cs, PowerDownRecoveryCause::SavedStateInconsistent, context);
    }

    scg.ldocsr().modify(|w| w.set_ldoen(saved_ldo));
    if scg.ldocsr().read().ldoen() != saved_ldo {
        fail_stop(cs, PowerDownRecoveryCause::LdoRestoreRejected, context);
    }

    // No VOUT_OK wait when the saved LDO was off.
    if saved_ldo && !poll_until_valid(POLL_STEPS, step_cycles, || scg.ldocsr().read().vout_ok()) {
        fail_stop(cs, PowerDownRecoveryCause::VoutTimeout, context);
    }

    // Poll every configured reference; retain the first failure attribution.
    let mut first_failure: Option<PowerDownRecoveryCause> = None;

    if clocks.fro_12m_root.is_some()
        && !poll_until_valid(POLL_STEPS, step_cycles, || {
            scg.sirccsr().read().sircvld() == Sircvld::EnabledAndValid
        })
        && first_failure.is_none()
    {
        first_failure = Some(PowerDownRecoveryCause::SircTimeout);
    }

    #[cfg(feature = "mcxa2xx")]
    if clocks.fro_hf_root.is_some()
        && !poll_until_valid(POLL_STEPS, step_cycles, || {
            scg.firccsr().read().fircvld() == Fircvld::EnabledAndValid
        })
        && first_failure.is_none()
    {
        first_failure = Some(PowerDownRecoveryCause::FircTimeout);
    }

    #[cfg(not(feature = "sosc-as-gpio"))]
    if clocks.clk_in.is_some()
        && !poll_until_valid(POLL_STEPS, step_cycles, || scg.sosccsr().read().soscvld())
        && first_failure.is_none()
    {
        first_failure = Some(PowerDownRecoveryCause::SoscTimeout);
    }

    if let Some(cause) = first_failure {
        fail_stop(cs, cause, context);
    }

    // Restore exactly the saved power-enable, not an unconditional true.
    if spll_configured && !set_spll_pwren(saved_spllpwren) {
        fail_stop(cs, PowerDownRecoveryCause::SpllRestoreRejected, context);
    }

    // An intentionally unpowered SPLL cannot report lock.
    if spll_configured
        && saved_spllpwren
        && !poll_until_valid(POLL_STEPS, step_cycles, || {
            scg.spllcsr().read().spll_lock() == SpllLock::EnabledAndValid
        })
    {
        fail_stop(cs, PowerDownRecoveryCause::SpllLockTimeout, context);
    }

    context
}
