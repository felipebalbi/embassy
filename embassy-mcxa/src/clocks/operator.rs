//! `ClockOperator` — init-time clock configuration sequencing.
//!
//! This module contains the private `ClockOperator` struct and all of its
//! `configure_*` methods. It is only used during [`super::init()`].

use config::CoreSleep;
use cortex_m::peripheral::SCB;

use super::config;
use super::program::{ActiveDrive, LowPowerDrive, ResolvedClockProgram};
use super::types::ClockError;
use crate::pac;
use crate::pac::cmc::Ckmode;
use crate::pac::scg::{
    Fircacc, FircaccIe, FirccsrLk, Fircerr, FircerrIe, Fircsten, Scs, SirccsrLk, Sircerr, Sircvld, SosccsrLk, Soscerr,
    SpllLock, SpllcsrLk, Spllerr, TrimUnlock,
};
use crate::pac::spc::{ActiveCfgCoreldoVddDs, LpCfgBgmode};
use crate::pac::syscon::{
    AhbclkdivUnstab, FrohfdivHalt, FrohfdivReset, FrohfdivUnstab, FrolfdivHalt, FrolfdivReset, FrolfdivUnstab,
    Pll1clkdivHalt, Pll1clkdivReset, Pll1clkdivUnstab, Unlock,
};

/// The ClockOperator is a private helper type that contains the methods used
/// during system clock initialization.
///
/// # SAFETY
///
/// Concurrent access to clock-relevant peripheral registers, such as `MRCC`, `SCG`,
/// `SYSCON`, and `VBAT` should not be allowed for the duration of the [`init()`](super::init) function.
#[allow(dead_code)]
pub(super) struct ClockOperator<'a> {
    /// The fully-resolved program that this operator applies to hardware
    pub(super) resolved: &'a ResolvedClockProgram,

    // We hold on to stolen peripherals
    pub(super) _mrcc0: pac::mrcc::Mrcc,
    pub(super) scg0: pac::scg::Scg,
    pub(super) syscon: pac::syscon::Syscon,
    pub(super) vbat0: pac::vbat::Vbat,
    pub(super) spc0: pac::spc::Spc,
    pub(super) fmu0: pac::fmu::Fmu,
    pub(super) cmc: pac::cmc::Cmc,
}

impl ClockOperator<'_> {
    pub(super) fn unlock_mrcc(&mut self) {
        // On the MCXA5xx, this is default *locked*, preventing any writes to
        // MRCC registers re enable/div settings. For now, just leave it unlocked,
        // we might want to actively unlock/lock in periph helpers in the future.
        self.syscon.clkunlock().modify(|w| w.set_unlock(Unlock::Enable));
    }

    /// Configure the FIRC/FRO180M/FRO192M clock family
    pub(super) fn configure_firc_clocks(&mut self) -> Result<(), ClockError> {
        // Three options here:
        //
        // * Firc is disabled -> Switch main clock to SIRC and return
        // * Firc is enabled and !default ->
        //   * Switch main clock to SIRC
        //   * Make FIRC changes
        //   * Switch main clock back to FIRC
        // * Firc is enabled and default -> nop
        let is_default = self.resolved.firc.is_default;

        // If we are not default, then we need to switch to SIRC
        if !is_default {
            // Set SIRC (fro_12m) as the source
            self.scg0.rccr().modify(|w| w.set_scs(Scs::Sirc));

            // Wait for the change to complete
            while self.scg0.csr().read().scs() != Scs::Sirc {}
        }

        // Enable CSR writes
        self.scg0.firccsr().modify(|w| w.set_lk(FirccsrLk::WriteEnabled));

        // Did the user give us a FIRC config?
        //
        // NOTE: `freq_sel` is `Some` exactly when `enabled` is true.
        let sel = match (self.resolved.firc.enabled, self.resolved.firc.freq_sel) {
            (true, Some(sel)) => sel,
            _ => {
                // Nope, and we've already switched to fro_12m. Disable FIRC.
                self.scg0.firccsr().modify(|w| {
                    w.set_fircsten(Fircsten::DisabledInStopModes);
                    w.set_fircerr_ie(FircerrIe::ErrorNotDetected);
                    w.set_firc_fclk_periph_en(false);
                    w.set_firc_sclk_periph_en(false);
                    w.set_fircen(false);
                });

                self.scg0.firccsr().modify(|w| w.set_lk(FirccsrLk::WriteDisabled));
                return Ok(());
            }
        };

        // If we are here, we WANT FIRC. If we are !default, let's disable FIRC before
        // we mess with it. If we are !default, we have already switched to SIRC instead!
        if !is_default {
            // Unlock
            self.scg0.firccsr().modify(|w| w.set_lk(FirccsrLk::WriteEnabled));

            // Disable FIRC
            self.scg0.firccsr().modify(|w| {
                w.set_fircen(false);
                w.set_fircsten(Fircsten::DisabledInStopModes);
                w.set_fircerr_ie(FircerrIe::ErrorNotDetected);
                w.set_fircacc_ie(FircaccIe::Fircaccnot);
                w.set_firc_sclk_periph_en(false);
                w.set_firc_fclk_periph_en(false);
            });
        }

        // Set frequency (if not the default!), re-enable FIRC
        self.scg0.firccfg().modify(|w| w.set_freq_sel(sel));
        self.scg0.firccsr().modify(|w| w.set_fircen(true));

        // Wait for FIRC to be enabled, error-free, and accurate
        let mut firc_ok = false;
        while !firc_ok {
            let csr = self.scg0.firccsr().read();

            firc_ok =
                csr.fircen() && csr.fircacc() == Fircacc::EnabledAndValid && csr.fircerr() == Fircerr::ErrorNotDetected;
        }

        // Note that the fro_hf_root is active

        // Okay! Now we're past that, let's enable all the downstream clocks.
        let pow_set = self.resolved.firc.fircsten;

        // Do we enable the `fro_hf` output?
        let fro_hf_set = self.resolved.firc.fro_hf_gate;

        // Do we enable the `clk_45m`/`clk_48m` output?
        let clk_fund_set = self.resolved.firc.fundamental_gate;

        self.scg0.firccsr().modify(|w| {
            w.set_fircsten(pow_set);
            w.set_firc_fclk_periph_en(fro_hf_set);
            w.set_firc_sclk_periph_en(clk_fund_set);
        });

        // Last write to CSR, re-lock
        self.scg0.firccsr().modify(|w| w.set_lk(FirccsrLk::WriteDisabled));

        // Do we enable the `fro_hf_div` output?
        //
        // NOTE: the `fro_hf_div` requires `fro_hf` dependency is enforced during
        // resolution, before we get here.
        if let Some(div) = self.resolved.firc.fro_hf_div_bits {
            // Halt and reset the div; then set our desired div.
            self.syscon.frohfdiv().write(|w| {
                w.set_halt(FrohfdivHalt::Halt);
                w.set_reset(FrohfdivReset::Asserted);
                w.set_div(div);
            });
            // Then unhalt it, and reset it
            self.syscon.frohfdiv().write(|w| {
                w.set_halt(FrohfdivHalt::Run);
                w.set_reset(FrohfdivReset::Released);
                w.set_div(div);
            });

            // Wait for clock to stabilize
            while self.syscon.frohfdiv().read().unstab() == FrohfdivUnstab::Ongoing {}

            // Store off the clock info
        }

        Ok(())
    }

    /// Configure the SIRC/FRO12M clock family
    pub(super) fn configure_sirc_clocks_early(&mut self) -> Result<(), ClockError> {
        // Allow writes
        self.scg0.sirccsr().modify(|w| w.set_lk(SirccsrLk::WriteEnabled));

        let deep = self.resolved.sirc.deep;

        // clk_1m is *before* the fro_12m clock gate

        // If the user wants fro_12m to be disabled, FOR now, we ignore their
        // wish to ensure fro_12m is selectable as a main_clk source at least until
        // we select the CPU clock. We still mark it as not enabled though, to prevent
        // other peripherals using it, as we will gate if off at `configure_sirc_clocks_late`.

        // Set sleep/peripheral usage
        self.scg0.sirccsr().modify(|w| {
            w.set_sircsten(deep);
            // Always on, for now at least! Will be resolved in `configure_sirc_clocks_late`
            w.set_sirc_clk_periph_en(true);
        });

        while self.scg0.sirccsr().read().sircvld() == Sircvld::DisabledOrNotValid {}
        if self.scg0.sirccsr().read().sircerr() == Sircerr::ErrorDetected {
            return Err(ClockError::BadConfig {
                clock: "sirc",
                reason: "error set",
            });
        }

        // reset lock
        self.scg0.sirccsr().modify(|w| w.set_lk(SirccsrLk::WriteDisabled));

        // Do we enable the `fro_lf_div` output?
        //
        // NOTE: the `fro_lf_div` requires `fro_12m` dependency is enforced during
        // resolution, before we get here.
        if let Some(div) = self.resolved.sirc.fro_lf_div_bits {
            // Halt and reset the div; then set our desired div.
            self.syscon.frolfdiv().write(|w| {
                w.set_halt(FrolfdivHalt::Halt);
                w.set_reset(FrolfdivReset::Asserted);
                w.set_div(div);
            });
            // Then unhalt it, and reset it
            self.syscon.frolfdiv().modify(|w| {
                w.set_halt(FrolfdivHalt::Run);
                w.set_reset(FrolfdivReset::Released);
                w.set_div(div);
            });

            // Wait for clock to stabilize
            while self.syscon.frolfdiv().read().unstab() == FrolfdivUnstab::Ongoing {}

            // Store off the clock info
        }

        Ok(())
    }

    pub(super) fn configure_sirc_clocks_late(&mut self) {
        // If we forced SIRC's fro_12m to be enabled, disable it now.
        if self.resolved.sirc.sirc_forced {
            // Allow writes
            self.scg0.sirccsr().modify(|w| w.set_lk(SirccsrLk::WriteEnabled));

            // Disable clk_12m
            self.scg0.sirccsr().modify(|w| w.set_sirc_clk_periph_en(false));

            // reset lock
            self.scg0.sirccsr().modify(|w| w.set_lk(SirccsrLk::WriteDisabled));
        }
    }

    /// Configure the ROSC/FRO16K/clk_16k clock family
    pub(super) fn configure_fro16k_clocks(&mut self) -> Result<(), ClockError> {
        // If we have a config: ensure fro16k is enabled. If not: ensure it is disabled.
        let enable = self.resolved.fro16k.enable;
        self.vbat0.froctla().modify(|w| w.set_fro_en(enable));

        // Lock the control register
        self.vbat0.frolcka().modify(|w| w.set_lock(true));

        // If we're disabled, we're done!
        let Some(bits) = self.resolved.fro16k.clke else {
            return Ok(());
        };

        // Enabled, now set up.
        //
        // Enable clock outputs to both VSYS and VDD_CORE domains
        // Bit 0: clk_16k0 to VSYS domain
        // Bit 1: clk_16k1 to VDD_CORE/CORE_MAIN domain
        // Bit 2: clk_16k2 to VBAT domain (5xx only)
        //
        // TODO: Define sub-fields for this register with a PAC patch?
        #[cfg(feature = "mcxa5xx")]
        {}
        self.vbat0.froclke().modify(|w| w.set_clke(bits));

        Ok(())
    }

    /// Configure the ROSC/OSC32K clock family
    #[cfg(all(feature = "mcxa5xx", feature = "unstable-osc32k", not(feature = "rosc-32k-as-gpio")))]
    pub(super) fn configure_osc32k_clocks(&mut self) -> Result<(), ClockError> {
        use nxp_pac::vbat::{ExtalCapSel, InitTrim, ModeEn, StatusaLdoRdy, StatusaOscRdy, XtalCapSel};

        use super::program::Osc32KProgram;

        // Unlock the control first
        self.vbat0.ldolcka().modify(|w| w.set_lock(false));

        let program = &self.resolved.osc32k;
        if let Osc32KProgram::Absent = program {
            // TODO: how to ensure disabled?
            // ???

            // Re-lock after disabling
            self.vbat0.ldolcka().modify(|w| w.set_lock(true));
            return Ok(());
        }

        // To enable and lock the LDO and bandgap:
        //
        // NOTE(AJM): "The FRO16K must be enabled before enabling the SRAM LDO or the bandgap"
        //
        // 1. Enable the FRO16K.
        //   * NOTE(AJM): clk_16k is always enabled if enabled at all.
        //   * TODO(AJM): I'm not sure which domain needs to be active for this requirement.
        //     It seems reasonable that it would be the vbat domain?
        //
        // NOTE: the `clk_16k_vbat` prerequisite is enforced during resolution, before
        // we get here.

        // 2. Write 7h to LDO_RAM Control A (LDOCTLA).
        self.vbat0.ldoctla().write(|w| {
            w.set_refresh_en(true);
            w.set_ldo_en(true);
            w.set_bg_en(true);
        });

        // 3. Wait for STATUSA[LDO_RDY] to become 1.
        while self.vbat0.statusa().read().ldo_rdy() != StatusaLdoRdy::Set {}

        // 4. Write 1h to LDOLCKA[LOCK].
        self.vbat0.ldolcka().modify(|w| w.set_lock(true));

        match *program {
            // Handled by the early return above; no registers are touched here.
            Osc32KProgram::Absent => {}
            Osc32KProgram::HighPower {
                xtal_cap_sel,
                extal_cap_sel,
                coarse_amp_gain,
                clke,
            } => {
                // To configure and lock OSC32kHz for normal mode operation:
                //
                // 1. Configure OSCCTLA[EXTAL_CAP_SEL], OSCCTLA[XTAL_CAP_SEL] and OSCCTLA[COARSE_AMP_GAIN] as
                // required based on the external crystal component ESR and CL values, and by the PCB parasitics on the EXTAL32K and
                // XTAL32K pins. Configure 0h to OSCCTLA[MODE_EN], 1h to OSCCTLA[CAP_SEL_EN], and 1h to OSCCTLA[OSC_EN].
                //   * NOTE(AJM): You must write 1 to this field and OSCCTLA[OSC_EN] simultaneously.
                self.vbat0.oscctla().modify(|w| {
                    w.set_xtal_cap_sel(xtal_cap_sel);
                    w.set_extal_cap_sel(extal_cap_sel);
                    w.set_coarse_amp_gain(coarse_amp_gain);
                    w.set_mode_en(ModeEn::Hp);
                    w.set_cap_sel_en(true);
                    w.set_osc_en(true);
                });

                // 2. Wait for STATUSA[OSC_RDY] to become 1.
                while self.vbat0.statusa().read().osc_rdy() != StatusaOscRdy::Set {}

                // 3. Write 1h to OSCLCKA[LOCK].
                self.vbat0.osclcka().modify(|w| w.set_lock(true));

                // 4. Write 0h to OSCCTLA[EXTAL_CAP_SEL] and 0h to OSCCTLA[XTAL_CAP_SEL].
                self.vbat0.oscctla().modify(|w| {
                    w.set_xtal_cap_sel(XtalCapSel::Sel0);
                    w.set_extal_cap_sel(ExtalCapSel::Sel0);
                });

                // 5. Alter OSCCLKE[CLKE] to clock gate different OSC32K outputs to different peripherals to reduce power consumption.
                self.vbat0.oscclke().modify(|w| {
                    w.set_clke(clke);
                });
            }
            Osc32KProgram::LowPower {
                coarse_amp_gain,
                supply_det,
                clke,
            } => {
                // To configure OSC32kHz for low power mode operation:
                //
                // 1. Write 3h to OSCCFGA[INIT_TRIM].
                //   * NOTE(AJM): This is "1 second"?
                self.vbat0.osccfga().modify(|w| w.set_init_trim(InitTrim::Sel3));

                // 2. Configure OSCCTLA[EXTAL_CAP_SEL], OSCCTLA[XTAL_CAP_SEL] and OSCCTLA[COARSE_AMP_GAIN] as
                // required based on the external crystal component ESR and CL values, and by the PCB parasitics on the EXTAL32K and
                // XTAL32K pins. Configure 1h to OSCCTLA[MODE_EN], 1h to OSCCTLA[CAP_SEL_EN], and 1h to OSCCTLA[OSC_EN].
                //   * NOTE(AJM): The configuration EXTAL_CAP_SEL=0000 and CAP_SEL_EN=1 is required in low power
                //     mode and is not supported in other modes
                self.vbat0.oscctla().modify(|w| {
                    // TODO(AJM): Do we need to set these to reasonable values during the "startup" phase, and THEN
                    // restore them to 0? RM is very unclear here.
                    w.set_xtal_cap_sel(XtalCapSel::Sel0);
                    w.set_extal_cap_sel(ExtalCapSel::Sel0);

                    w.set_coarse_amp_gain(coarse_amp_gain);

                    // TODO: This naming is bad
                    //
                    // pub enum ModeEn {
                    //     #[doc = "Normal mode"]
                    //     HP = 0x0,
                    //     #[doc = "Startup mode"]
                    //     LP = 0x01,
                    //     _RESERVED_2 = 0x02,
                    //     #[doc = "Low power mode"]
                    //     SW = 0x03,
                    // }

                    w.set_mode_en(ModeEn::Lp);
                    w.set_cap_sel_en(true);
                    w.set_osc_en(true);
                });

                // 3. Wait for STATUSA[OSC_RDY] to become 1.
                while self.vbat0.statusa().read().osc_rdy() != StatusaOscRdy::Set {}

                // 4. Write 0h to OSCCFGA[INIT_TRIM].
                self.vbat0.osccfga().modify(|w| w.set_init_trim(InitTrim::Sel0));

                // 5. Configure 3h to OSCCTLA[MODE_EN], 0h to OSCCTLA[EXTAL_CAP_SEL] and 0h to OSCCTLA[XTAL_CAP_SEL].
                // Configure OSCCTLA[SUPPLY_DET] as required by application.
                self.vbat0.oscctla().modify(|w| {
                    w.set_mode_en(ModeEn::Sw);
                    w.set_xtal_cap_sel(XtalCapSel::Sel0);
                    w.set_extal_cap_sel(ExtalCapSel::Sel0);
                    w.set_supply_det(supply_det);
                });

                // 6. Wait for STATUSA[OSC_RDY] to become 1.
                while self.vbat0.statusa().read().osc_rdy() != StatusaOscRdy::Set {}

                // 7. Alter OSCCLKE[CLKE] to clock gate different OSC32K outputs to different peripherals to reduce power consumption.
                self.vbat0.oscclke().modify(|w| {
                    w.set_clke(clke);
                });
            }
        }

        Ok(())
    }

    fn ensure_ldo_active(&mut self) {
        // TODO: Config for the LDO? For now, just enable
        // using the default settings:
        // LDOBYPASS: 0/not bypassed
        // VOUT_SEL: 0b100: 1.1v
        // LDOEN: 0/Disabled
        let already_enabled = {
            let ldocsr = self.scg0.ldocsr().read();
            ldocsr.ldoen() && ldocsr.vout_ok()
        };
        if !already_enabled {
            self.scg0.ldocsr().modify(|w| w.set_ldoen(true));
            while !self.scg0.ldocsr().read().vout_ok() {}
        }
    }

    /// Configure the SOSC/clk_in oscillator
    #[cfg(not(feature = "sosc-as-gpio"))]
    pub(super) fn configure_sosc(&mut self) -> Result<(), ClockError> {
        use super::program::SoscProgram;

        let SoscProgram::Enabled { erefs, range, soscsten } = self.resolved.sosc else {
            return Ok(());
        };

        // Enable (and wait for) LDO to be active
        self.ensure_ldo_active();

        // Set source/erefs and range
        self.scg0.sosccfg().modify(|w| {
            w.set_erefs(erefs);
            w.set_range(range);
        });

        // Disable lock
        self.scg0.sosccsr().modify(|w| w.set_lk(SosccsrLk::WriteEnabled));

        // TODO: We could enable the SOSC clock monitor. There are some things to
        // figure out first:
        //
        // * This requires SIRC to be enabled, not sure which branch. Maybe fro12m_root?
        // * If SOSC needs to work in deep sleep, AND the monitor is enabled:
        //   * SIRC also need needs to be low power
        // * We need to decide if we need an interrupt or a reset if the monitor trips

        // Apply remaining config
        self.scg0.sosccsr().modify(|w| {
            // For now, just disable the monitor. See above.
            w.set_sosccm(false);

            // Set deep sleep mode if needed
            w.set_soscsten(soscsten);

            // Enable SOSC
            w.set_soscen(true)
        });

        // Wait for SOSC to be valid, check for errors
        while !self.scg0.sosccsr().read().soscvld() {}
        if self.scg0.sosccsr().read().soscerr() == Soscerr::EnabledAndError {
            return Err(ClockError::BadConfig {
                clock: "clk_in",
                reason: "soscerr is set",
            });
        }

        // Re-lock the sosc
        self.scg0.sosccsr().modify(|w| w.set_lk(SosccsrLk::WriteDisabled));

        Ok(())
    }

    pub(super) fn configure_spll(&mut self) -> Result<(), ClockError> {
        use super::program::SpllProgram;

        // # Vocab
        //
        // | Name   | Meaning                                                     |
        // | :---   | :---                                                        |
        // | Fin    | Frequency of clkin                                          |
        // | clkout | Output clock of the PLL                                     |
        // | Fout   | Frequency of clkout (depends on mode)                       |
        // | clkref | PLL Reference clock, the input clock to the PFD             |
        // | Fref   | Frequency of clkref, Fref = Fin / N                         |
        // | Fcco   | Frequency of the output clock of the CCO, Fcco = M * Fref   |
        // | N      | Predivider value                                            |
        // | M      | Feedback divider value                                      |
        // | P      | Postdivider value                                           |
        // | Tpon   | PLL start-up time                                           |

        // No PLL? Nothing to do!
        let SpllProgram::Enabled {
            source,
            selp,
            seli,
            selr,
            m,
            n,
            p,
            bp_pre,
            bp_post,
            bp_post2,
            lock_time,
            spllsten,
            pll1_clk_div_bits,
        } = self.resolved.spll
        else {
            return Ok(());
        };

        // Ensure the LDO is active
        //
        // NOTE: the bandgap requirement is enforced during resolution, before we get here.
        self.ensure_ldo_active();

        // NOTE: source selection, the mode arithmetic (Fout/Fcco), and every
        // frequency-limit check are all performed during resolution, before we
        // get here.

        // Dump all the PLL calcs if needed for debugging
        #[cfg(feature = "defmt")]
        {
            defmt::debug!("bp_pre: {:?}", bp_pre);
            defmt::debug!("bp_post: {:?}", bp_post);
            defmt::debug!("bp_post2: {:?}", bp_post2);
            defmt::debug!("m: {:?}", m);
            defmt::debug!("p: {:?}", p);
            defmt::debug!("n: {:?}", n);
        }

        self.scg0.spllctrl().modify(|w| {
            w.set_source(source);
            w.set_selp(selp);
            w.set_seli(seli);
            w.set_selr(selr);
        });

        if let Some(n) = n {
            self.scg0.spllndiv().modify(|w| w.set_ndiv(n));
        }
        if let Some(p) = p {
            self.scg0.spllpdiv().modify(|w| w.set_pdiv(p));
        }
        self.scg0.spllmdiv().modify(|w| w.set_mdiv(m));

        self.scg0.spllctrl().modify(|w| {
            w.set_bypassprediv(bp_pre);
            w.set_bypasspostdiv(bp_post);
            w.set_bypasspostdiv2(bp_post2);

            // TODO: support FRM?
            w.set_frm(false);
        });

        // Unlock
        self.scg0.spllcsr().modify(|w| w.set_lk(SpllcsrLk::WriteEnabled));

        // TODO: Support clock monitors?
        // self.scg0.spllcsr().modify(|w| w.spllcm().?);

        self.scg0.trim_lock().write(|w| {
            w.set_trim_lock_key(0x5a5a);
            w.set_trim_unlock(TrimUnlock::NotLocked)
        });

        self.scg0.splllock_cnfg().write(|w| w.set_lock_time(lock_time));

        // TODO: Support Spread spectrum?

        self.scg0.spllcsr().modify(|w| {
            w.set_spllclken(true);
            w.set_spllpwren(true);
            w.set_spllsten(spllsten);
        });

        // Wait for SPLL to set up
        loop {
            let csr = self.scg0.spllcsr().read();
            if csr.spll_lock() == SpllLock::EnabledAndValid {
                if csr.spllerr() == Spllerr::EnabledAndError {
                    return Err(ClockError::BadConfig {
                        clock: "spll",
                        reason: "spllerr is set",
                    });
                }
                break;
            }
        }

        // Re-lock SPLL CSR
        self.scg0.spllcsr().modify(|w| w.set_lk(SpllcsrLk::WriteDisabled));

        // Store clock state

        // Do we enable the `pll1_clk_div` output?
        if let Some(div) = pll1_clk_div_bits {
            // Halt and reset the div; then set our desired div.
            self.syscon.pll1clkdiv().write(|w| {
                w.set_halt(Pll1clkdivHalt::Halt);
                w.set_reset(Pll1clkdivReset::Asserted);
                w.set_div(div);
            });
            // Then unhalt it, and reset it
            //
            // NOTE: `write()` zeroes any field not set here, so the divisor MUST
            // be repeated. Omitting it silently reset DIV to divide-by-1 while
            // `clocks.pll1_clk_div` still recorded `fout / (d + 1)`, so every
            // downstream `fmax` check was validated against a frequency that was
            // `(d + 1)` times too low. Matches the `frohfdiv` sequence above.
            self.syscon.pll1clkdiv().write(|w| {
                w.set_halt(Pll1clkdivHalt::Run);
                w.set_reset(Pll1clkdivReset::Released);
                w.set_div(div);
            });

            // Wait for clock to stabilize
            while self.syscon.pll1clkdiv().read().unstab() == Pll1clkdivUnstab::Ongoing {}

            // Store off the clock info
        }

        Ok(())
    }

    pub(super) fn configure_main_clk(&mut self) -> Result<(), ClockError> {
        // NOTE: source selection, its availability/power checks, and both
        // frequency-limit checks are performed during resolution, before we get here.
        let scs = self.resolved.main_clock.scs;
        let ahb_div_bits = self.resolved.main_clock.ahb_div_bits;

        // BEFORE we switch, update the flash wait states to the appropriate levels
        let wait_states = self.resolved.main_clock.wait_states;
        self.fmu0.fctrl().modify(|w| w.set_rwsc(wait_states));

        // TODO: (Double) check if clock is actually valid before switching?
        // Are we already on the right clock?
        let now = self.scg0.csr().read().scs();
        if now != scs {
            // Set RCCR
            self.scg0.rccr().modify(|w| w.set_scs(scs));

            // Wait for match
            while self.scg0.csr().read().scs() != scs {}
        }

        // Update AHB clock division, if necessary
        if ahb_div_bits != 0 {
            // AHB has no halt/reset fields - it's different to other DIV8s!
            self.syscon.ahbclkdiv().modify(|w| w.set_div(ahb_div_bits));
            // Wait for clock to stabilize
            while self.syscon.ahbclkdiv().read().unstab() == AhbclkdivUnstab::Ongoing {}
        }

        Ok(())
    }

    pub(super) fn configure_voltages(&mut self) -> Result<(), ClockError> {
        // Determine if we need to change the active mode voltage levels
        //
        // NOTE: In the `MidDriveMode` case this is `None`: that is the default mode,
        // and we don't believe we need to do anything.
        //
        // "The LVDE and HVDE fields reset only with a POR.
        // All other fields reset only with a system reset."
        let to_change = self.resolved.voltage.active_level_change;

        if let Some((vdd, vsm)) = to_change {
            // You can change the core VDD levels for the LDO_CORE low power regulator only
            // when CORELDO_VDD_DS=1.
            //
            // When switching CORELDO_VDD_DS from low to normal drive strength, ensure the LDO_CORE high
            // VDD LVL setting is set to the same level that was set prior to switching to the LDO_CORE drive strength
            // (CORELDO_VDD_DS). Otherwise, if the LVDs are enabled, an unexpected LVD can occur.
            //
            // Ensure drive strength is normal (BEFORE shifting level)
            self.spc0
                .active_cfg()
                .modify(|w| w.set_coreldo_vdd_ds(ActiveCfgCoreldoVddDs::Normal));

            // ## DS 26.3.2:
            //
            // When increasing voltage and frequency in Active mode, you must perform the following steps:
            //
            // 1. Increase voltage to a new level (ACTIVE_CFG[CORELDO_VDD_LVL]).
            self.spc0.active_cfg().modify(|w| w.set_coreldo_vdd_lvl(vdd));

            // 2. Wait for voltage change to complete (SC[BUSY] = 0).
            while self.spc0.sc().read().busy() {}

            // 3. Configure flash memory to support higher voltage level and frequency (FMU_FCTRL[RWSC].
            //
            // NOTE: This step skipped - we will update RWSC when we later apply main cpu clock
            // frequency changes.

            // 4. Configure SRAM to support higher voltage levels (SRAMCTL[VSM]).
            self.spc0.sramctl().modify(|w| w.set_vsm(vsm));

            // 5. Request SRAM voltage update (write 1 to SRAMCTL[REQ]).
            self.spc0.sramctl().modify(|w| w.set_req(true));

            // 6. Wait for SRAM voltage change to complete (SRAMCTL[ACK] = 1).
            while !self.spc0.sramctl().read().ack() {}

            // 7. Clear request for SRAM voltage change (write 0 to SRAMCTL[REQ]).
            self.spc0.sramctl().modify(|w| w.set_req(false));

            // 8. Increase frequency to a new level (for example, SCG_RCCR).
            //
            // NOTE: This step skipped - we will update RCCR when we later apply main cpu clock
            // frequency changes.

            // 9. You can continue execution.
            // :)
        }

        // If the CORELDO_VDD_DS fields are set to the same value in both the ACTIVE_CFG and LP_CFG registers,
        // the CORELDO_VDD_LVL's in the ACTIVE_CFG and LP_CFG register must be set to the same voltage
        // level settings.
        //
        // NOTE: this condition is enforced during resolution, before we get here.
        let lpwkup = self.resolved.voltage.lpwkup;
        self.spc0.lpwkup_delay().write(|w| w.set_lpwkup_delay(lpwkup));

        // You can change the core VDD levels for the LDO_CORE low power regulator only when
        // ACTIVE_CFG[CORELDO_VDD_DS] = 1. So, before entering any of the low-power states (DSLEEP,
        // PDOWN, DPDOWN) with LDO_CORE low power regulator selected (LP_CFG[CORELDO_VDD_DS] = 0),
        // you must use CORELDO_VDD_LVL to select the correct regulation level during ACTIVE run mode.
        //
        // NOTE(AJM): We've set drive strength to "normal" above, and do not (potentially) set it to
        // "low" until later below.

        // NOTE(AJM): The reference manual doesn't have any similar configuration requirements
        // for low power mode. We'll just configure it, I guess?
        //
        // NOTE(AJM): "LP_CFG: This register resets only after a POR or LVD event."
        let bgap = self.resolved.voltage.low_power.bandgap;
        if let LowPowerDrive::Low { enable_detectors } = self.resolved.voltage.low_power.drive {
            // If the bandgap is enabled, also enable the high/low voltage
            // detectors. if it is disabled, these must also be disabled.
            self.spc0.lp_cfg().modify(|w| {
                w.set_sys_hvde(enable_detectors);
                w.set_sys_lvde(enable_detectors);
                w.set_core_lvde(enable_detectors);
            });
        }
        // NOTE: in the `Normal` drive case:
        // "If you specify normal drive strength, you must write a value to LP[BGMODE] that enables the bandgap."
        let ds = self.resolved.voltage.low_power.ds;
        let lvl = self.resolved.voltage.low_power.level;
        self.spc0.lp_cfg().modify(|w| w.set_coreldo_vdd_ds(ds));

        // If we're enabling the bandgap, ensure we do it BEFORE changing the VDD level
        // If we're disabling the bandgap, ensure we do it AFTER changing the VDD level
        if bgap {
            self.spc0.lp_cfg().modify(|w| w.set_bgmode(LpCfgBgmode::Bgmode01));
            self.spc0.lp_cfg().modify(|w| w.set_coreldo_vdd_lvl(lvl));
        } else {
            self.spc0.lp_cfg().modify(|w| w.set_coreldo_vdd_lvl(lvl));
            self.spc0.lp_cfg().modify(|w| w.set_bgmode(LpCfgBgmode::Bgmode0));
        }

        // Updating CORELDO_VDD_LVL sets the SC[BUSY] flag. That flag remains set for at least the total time
        // delay that Active Voltage Trim Delay (ACTIVE_VDELAY) specifies.
        //
        // Before changing CORELDO_VDD_LVL, you must wait until the SC[BUSY] flag clears before entering the
        // selected low-power sleep
        //
        // NOTE(AJM): Let's just proactively wait now so we don't have to worry about it on subsequent sleeps
        while self.spc0.sc().read().busy() {}

        // NOTE(AJM): I don't really know if this is valid! I'm guessing in most cases you would want to
        // use the low drive strength for lp mode, and high drive strength for active mode?
        match self.resolved.voltage.active.drive {
            ActiveDrive::Low {
                enable_detectors,
                ds,
                bgmode,
            } => {
                // If the bandgap is enabled, also enable the high/low voltage
                // detectors. if it is disabled, these must also be disabled.
                self.spc0.active_cfg().modify(|w| {
                    w.set_sys_hvde(enable_detectors);
                    w.set_sys_lvde(enable_detectors);
                    w.set_core_lvde(enable_detectors);
                });

                // optionally disable bandgap AFTER setting vdd strength to low
                self.spc0.active_cfg().modify(|w| w.set_coreldo_vdd_ds(ds));
                self.spc0.active_cfg().modify(|w| w.set_bgmode(bgmode));
            }
            ActiveDrive::Normal => {
                // Already set to normal above
            }
        }

        // NOTE: calling `cortex_m::Peripherals::steal()` still marks the core peripherals as taken. See
        // https://github.com/embassy-rs/embassy/issues/5563 for discussion. Since this
        // is a ZST, transmuting from `()` is reasonable.
        let mut scb: SCB = unsafe { core::mem::transmute(()) };

        // Apply sleep settings
        match self.resolved.voltage.core_sleep {
            CoreSleep::WfeUngated => {
                // Do not gate
                self.cmc.ckctrl().modify(|w| w.set_ckmode(Ckmode::Ckmode0000));

                // Debug is enabled when core sleeps
                self.cmc.dbgctl().modify(|w| w.set_sod(false));

                // Don't allow the core to be gated to avoid killing the debugging session
                scb.clear_sleepdeep();
            }
            CoreSleep::WfeGated => {
                // Allow automatic gating of the core when in LIGHT sleep
                self.cmc.ckctrl().modify(|w| w.set_ckmode(Ckmode::Ckmode0001));

                // Debug is disabled when core sleeps
                self.cmc.dbgctl().modify(|w| w.set_sod(true));

                // Allow the core to be gated - this WILL kill the debugging session!
                scb.set_sleepdeep();
            }
            CoreSleep::DeepSleep => {
                // We can only support deep sleep with a custom executor which properly
                // handles going to sleep and returning
                #[cfg(all(not(feature = "executor-platform"), feature = "defmt"))]
                defmt::warn!("deep sleep enabled without custom executor");

                // For now, just enable light sleep. The executor will set deep sleep when
                // appropriate
                self.cmc.ckctrl().modify(|w| w.set_ckmode(Ckmode::Ckmode0001));

                // Debug is disabled when core sleeps
                self.cmc.dbgctl().modify(|w| w.set_sod(true));

                // Allow the core to be gated - this WILL kill the debugging session!
                scb.set_sleepdeep();

                // Enable sevonpend, to allow us to wake from WFE sleep with interrupts disabled
                unsafe {
                    // TODO: wait for https://github.com/rust-embedded/cortex-m/commit/1be630fdd06990bd14251eabe4cca9307bde549d
                    // to be released, until then, manual version of SCB.set_sevonpend();
                    scb.scr.modify(|w| w | (1 << 4));
                }
            }
        }

        // Allow automatic gating of the flash memory
        let wake = self.resolved.voltage.flash_wake;
        let doze = self.resolved.voltage.flash_doze;

        self.cmc.flashcr().modify(|w| {
            w.set_flashdoze(doze);
            w.set_flashwake(wake);
        });

        // At init, disable all analog peripherals. These can be re-enabled
        // if necessary for HAL drivers.
        self.spc0.active_cfg1().write(|w| w.0 = 0);
        self.spc0.lp_cfg1().write(|w| w.0 = 0);

        // Update status

        Ok(())
    }
}
