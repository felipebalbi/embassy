//! Power Down with SW2 wake on FRDM-MCXA577: P3_17, WUU input 26, falling edge.
//!
//! Run from `examples/mcxa5xx`:
//! ```text
//! cargo run --release --no-default-features --features=powerdown-diag --bin power-powerdown-wuu
//! ```
//!
//! Blue: armed and waiting. Green: one wake completed. Red: diagnostic pulses,
//! then terminal park. Diagnostics start after `hal::init`; an init failure is dark.
//! SWD/RTT drops at Power Down entry. Recover by holding ISP, tapping RESET,
//! then releasing ISP before reflashing.
//! The core uses SIRC at 12 MHz, with no SPLL, FIRC or SOSC; FRO16K stays on.
//! OSTIMER stops in Power Down, so the button wait has no deadline.
//! This example has not been hardware-tested; no MCXA5 current measurement is available.
//! SRAM retention uses the reset default.

#![no_std]
#![no_main]

// Link the HAL's defmt logger; diagnostics here do not use RTT.
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_mcxa as hal;
use embassy_mcxa::clocks::config::{
    CoreSleep, Div8, FlashSleep, Fro16KConfig, MainClockConfig, MainClockSource, VddDriveStrength, VddLevel,
};
use embassy_mcxa::clocks::{PoweredClock, WakeGuard};
use embassy_mcxa::gpio::{DriveStrength, Level, Output, Pull, SlewRate};
use embassy_mcxa::pac;
use embassy_mcxa::perf_counters;
use embassy_mcxa::reset_reason::{ResetReason, ResetReasonRaw, reset_reason};
use embassy_mcxa::spc::ExternalVoltageDomainPolicy;
use embassy_mcxa::wuu::{self, Edge, PinWake, PowerDownOnly, Wuu, WuuArmError};
use embassy_mcxa::{bind_interrupts, peripherals};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    WUU0 => wuu::InterruptHandler<peripherals::WUU0>;
});

// Nonsecure aliases from MCXA577_COMMON.h.
const GPIO3_BASE_NS: usize = 0x4004_E000;
const PORT3_BASE_NS: usize = 0x400B_F000;
const CORE_HZ: u32 = 12_000_000;
// Completed button waits, not executor entries within a wait.
const WAKE_BUDGET: u32 = 8;
const MAX_PULSES: u32 = 1000;

// Red pulse counts; each pulse is 250 ms on and 250 ms off, then steady red.
// | Count | Meaning |
// | 2 / 4 | No live wake source / entry setup aborted |
// | 8 / 10 / 12 / 13 | EVD readback / abnormal reset / WUU construction / source map |
// | 14 / 15 | Clocks unavailable / incompatible detection scope |
// | 16 / 17 / 18 | NVIC disabled / SEVONPEND clear / arm readback mismatch |
// | 19 / 20 | Unknown arm error / fail-stop or insufficient retention evidence |
const SOFTWARE_RESET_PULSES: u32 = 20;
const ABNORMAL_BOOT_PULSES: u32 = 10;
const WUU_CONSTRUCT_FAILURE_PULSES: u32 = 12;
const SOURCE_MAP_FAILURE_PULSES: u32 = 13;
const ARM_CLOCKS_UNAVAILABLE_PULSES: u32 = 14;
const ARM_INCOMPATIBLE_SCOPE_PULSES: u32 = 15;
const ARM_INTERRUPT_DISABLED_PULSES: u32 = 16;
const ARM_SEVONPEND_CLEAR_PULSES: u32 = 17;
const ARM_READBACK_MISMATCH_PULSES: u32 = 18;
const ARM_UNKNOWN_VARIANT_PULSES: u32 = 19;
const PULSE_ON_MS: u32 = 250;
const PULSE_OFF_MS: u32 = 250;
const PULSE_ON_CYCLES: u32 = (CORE_HZ / 1_000) * PULSE_ON_MS;
const PULSE_OFF_CYCLES: u32 = (CORE_HZ / 1_000) * PULSE_OFF_MS;
const GREEN_DWELL_CYCLES: u32 = CORE_HZ / 4;

// Conservative board policy, not a documented SIRC startup maximum.
// With no SPLL configured, validation requires only a nonzero budget.
const POWER_DOWN_CLOCK_RECOVERY_BUDGET_US: u32 = 1_000_000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Colour {
    Off,
    Red,
    Green,
    Blue,
}

struct Leds {
    red: Output<'static>,
    green: Output<'static>,
    blue: Output<'static>,
}

impl Leds {
    // Active-low RGB: turn every channel off before selecting one colour.
    fn show(&mut self, colour: Colour) {
        self.red.set_high();
        self.green.set_high();
        self.blue.set_high();

        match colour {
            Colour::Off => {}
            Colour::Red => self.red.set_low(),
            Colour::Green => self.green.set_low(),
            Colour::Blue => self.blue.set_low(),
        }
    }

    fn pulse(&mut self, colour: Colour, count: u32) {
        let bounded = if count > MAX_PULSES { MAX_PULSES } else { count };

        for _ in 0..bounded {
            self.show(colour);
            cortex_m::asm::delay(PULSE_ON_CYCLES);
            self.show(Colour::Off);
            cortex_m::asm::delay(PULSE_OFF_CYCLES);
        }

        self.show(colour);
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Boot {
    // Aggregate retention refusal, not identification of the reset issuer.
    Software,
    // Admitted startup or probe-workflow reset.
    Por,
    Abnormal,
}

fn is_cold_start(reset: ResetReasonRaw) -> bool {
    reset.is_exactly(&[ResetReason::Por]) || reset.is_exactly(&[ResetReason::Por, ResetReason::VoltageDetect])
}

// A recognised marker blocks every candidate; noncold candidates also need
// the boot's pre-arm canary. Neither marker nor canary authenticates a producer.
fn classify(reset: ResetReasonRaw, fail_stop_marker: bool, canary: bool) -> Boot {
    if reset.is_software() {
        if !reset.is_exactly(&[ResetReason::Software, ResetReason::Warm]) {
            return Boot::Abnormal;
        }

        if fail_stop_marker || !canary {
            return Boot::Software;
        }

        return Boot::Por;
    }

    let admitted = is_cold_start(reset)
        || reset.is_exactly(&[ResetReason::Pin, ResetReason::Warm])
        || reset.is_exactly(&[ResetReason::Dap, ResetReason::Warm])
        || reset.is_exactly(&[ResetReason::Jtag, ResetReason::Fatal, ResetReason::Warm])
        || reset.is_exactly(&[ResetReason::Dap]);
    if !admitted {
        return Boot::Abnormal;
    }

    if fail_stop_marker {
        return Boot::Software;
    }

    if !canary && !is_cold_start(reset) {
        return Boot::Software;
    }

    Boot::Por
}

// Active-memory counters reset at startup and require powerdown-diag.
struct Perf {
    power_downs: u32,
    no_live_wake: u32,
    setup_aborts: u32,
    clock_timeouts: u32,
    spc_failures: u32,
    wuu_replay_failures: u32,
}

impl Perf {
    fn snapshot() -> Self {
        Self {
            power_downs: perf_counters::get_power_downs(),
            no_live_wake: perf_counters::get_power_down_no_live_wake(),
            setup_aborts: perf_counters::get_power_down_setup_aborts(),
            clock_timeouts: perf_counters::get_power_down_clock_timeouts(),
            spc_failures: perf_counters::get_power_down_spc_failures(),
            wuu_replay_failures: perf_counters::get_power_down_wuu_replay_failures(),
        }
    }

    fn delta(&self, later: &Self) -> Self {
        Self {
            power_downs: later.power_downs.wrapping_sub(self.power_downs),
            no_live_wake: later.no_live_wake.wrapping_sub(self.no_live_wake),
            setup_aborts: later.setup_aborts.wrapping_sub(self.setup_aborts),
            clock_timeouts: later.clock_timeouts.wrapping_sub(self.clock_timeouts),
            spc_failures: later.spc_failures.wrapping_sub(self.spc_failures),
            wuu_replay_failures: later.wuu_replay_failures.wrapping_sub(self.wuu_replay_failures),
        }
    }

    // Only returnable refusals are observable here. Recovery failures publish
    // a retained record and reset, or halt if publication fails.
    fn refusal_signature(&self) -> Option<u32> {
        if self.no_live_wake != 0 {
            Some(2)
        } else if self.setup_aborts != 0 {
            Some(4)
        } else {
            None
        }
    }
}

// Check PAC aliases, not the part's runtime security attribution.
fn source_map_matches() -> bool {
    pac::GPIO3.as_ptr() as usize == GPIO3_BASE_NS && pac::PORT3.as_ptr() as usize == PORT3_BASE_NS
}

// EVD_CFG resets only on POR; replace stale policy and verify both fields.
fn apply_evd_policy() -> bool {
    ExternalVoltageDomainPolicy::all_domains_powered().apply().is_ok()
}

// Prevent further Power Down. Ordinary WFE is not a debug-access guarantee.
async fn park(leds: &mut Leds) -> ! {
    let _wg = WakeGuard::new();

    leds.show(Colour::Red);

    let never: core::convert::Infallible = core::future::pending().await;
    match never {}
}

// The stock executor does not enter the configured low-power mode.
#[cfg(not(feature = "executor-platform"))]
compile_error!(
    "power-powerdown-wuu requires --no-default-features --features=executor-platform. Under the stock \
     embassy executor CoreSleep only warns and never sleeps, so a build without this feature \
     is a silent no-op that demonstrates nothing."
);

#[cfg_attr(
    feature = "executor-platform",
    embassy_executor::main(executor = "embassy_mcxa::executor::Executor", entry = "cortex_m_rt::entry")
)]
async fn main(_spawner: Spawner) {
    // reset_reason caches SRS and clears SSRS, not the original sticky history.
    // Consume once before init so refused boots can select WfeUngated.
    let reset = reset_reason();
    let fail_stop_marker = embassy_mcxa::clocks::take_power_down_fail_stop();
    let canary = embassy_mcxa::clocks::power_down_retention_canary_is_present();
    let boot = classify(reset, fail_stop_marker, canary);

    let mut cfg = hal::config::Config::default();

    cfg.clock_cfg.firc = None;

    // SIRC retention is not an LDO request.
    cfg.clock_cfg.sirc.fro_12m_enabled = true;
    cfg.clock_cfg.sirc.fro_lf_div = None;
    cfg.clock_cfg.sirc.power = PoweredClock::AlwaysEnabled;

    // clk_16k[0] supplies the WUU detector.
    cfg.clock_cfg.fro16k = Some(Fro16KConfig::default());

    cfg.clock_cfg.sosc = None;
    cfg.clock_cfg.spll = None;

    // Keep CORE_HZ consistent with the undivided SIRC main clock.
    cfg.clock_cfg.main_clock = MainClockConfig {
        source: MainClockSource::SircFro12M,
        power: PoweredClock::NormalEnabledDeepSleepDisabled,
        ahb_clk_div: Div8::no_div(),
    };

    // Low drive/bandgap-off applies only while the CPU is stopped.
    cfg.clock_cfg.vdd_power.active_mode.level = VddLevel::OverDriveMode;
    cfg.clock_cfg.vdd_power.active_mode.drive = VddDriveStrength::Normal;
    cfg.clock_cfg.vdd_power.low_power_mode.level = VddLevel::MidDriveMode;
    cfg.clock_cfg.vdd_power.low_power_mode.drive = VddDriveStrength::Low { enable_bandgap: false };

    // Avoid a PowerDown admission error before the diagnostic LEDs exist.
    cfg.clock_cfg.vdd_power.core_sleep = match boot {
        Boot::Software | Boot::Abnormal => CoreSleep::WfeUngated,
        Boot::Por => CoreSleep::PowerDown,
    };

    cfg.clock_cfg.vdd_power.flash_sleep = FlashSleep::FlashDoze;
    cfg.clock_cfg.power_down_clock_recovery_budget_us = Some(POWER_DOWN_CLOCK_RECOVERY_BUDGET_US);

    let p = hal::init(cfg);

    // Low spans WFE and masked recovery, including fallback WFE; it alone
    // does not prove Power Down. Check completed power_downs as well.
    #[cfg(feature = "executor-platform")]
    embassy_mcxa::executor::set_executor_debug_gpio(p.P2_3);

    let mut leds = Leds {
        red: Output::new(p.P2_14, Level::High, DriveStrength::Normal, SlewRate::Slow),
        green: Output::new(p.P2_22, Level::High, DriveStrength::Normal, SlewRate::Slow),
        blue: Output::new(p.P2_23, Level::High, DriveStrength::Normal, SlewRate::Slow),
    };

    match boot {
        Boot::Software => {
            leds.pulse(Colour::Red, SOFTWARE_RESET_PULSES);
            park(&mut leds).await
        }
        Boot::Abnormal => {
            leds.pulse(Colour::Red, ABNORMAL_BOOT_PULSES);
            park(&mut leds).await
        }
        Boot::Por => {}
    }

    if !source_map_matches() {
        leds.pulse(Colour::Red, SOURCE_MAP_FAILURE_PULSES);
        park(&mut leds).await
    }

    if !apply_evd_policy() {
        leds.pulse(Colour::Red, 8);
        park(&mut leds).await
    }

    let button = p.P3_17;

    let ctrl = match Wuu::new(p.WUU0, Irqs) {
        Ok(ctrl) => ctrl,
        Err(_failure) => {
            leds.pulse(Colour::Red, WUU_CONSTRUCT_FAILURE_PULSES);
            park(&mut leds).await
        }
    };

    // Power Down supports the low-leakage-only detector.
    let wake_cfg: PinWake<wuu::Interrupt, PowerDownOnly> = PinWake::interrupt(Edge::Falling);

    let mut wake = match ctrl.arm_pin(button, Pull::Up, wake_cfg) {
        Ok(handle) => handle,
        Err(failure) => {
            match failure.error {
                WuuArmError::ClocksUnavailable => leds.pulse(Colour::Red, ARM_CLOCKS_UNAVAILABLE_PULSES),
                WuuArmError::IncompatibleScope => leds.pulse(Colour::Red, ARM_INCOMPATIBLE_SCOPE_PULSES),
                WuuArmError::InterruptDisabled => leds.pulse(Colour::Red, ARM_INTERRUPT_DISABLED_PULSES),
                WuuArmError::SevOnPendClear => leds.pulse(Colour::Red, ARM_SEVONPEND_CLEAR_PULSES),
                WuuArmError::ReadbackMismatch => leds.pulse(Colour::Red, ARM_READBACK_MISMATCH_PULSES),
                _ => leds.pulse(Colour::Red, ARM_UNKNOWN_VARIANT_PULSES),
            }
            park(&mut leds).await
        }
    };

    let mut confirmed = 0u32;
    let mut refusal = None;

    for _cycle in 0..WAKE_BUDGET {
        let before = Perf::snapshot();

        leds.show(Colour::Blue);
        wake.wait_for_wake().await;

        let delta = before.delta(&Perf::snapshot());

        if let Some(signature) = delta.refusal_signature() {
            refusal = Some(signature);
            break;
        }

        // Zero can mean counters disabled or fallback WFE, not a completed entry.
        confirmed = confirmed.saturating_add(delta.power_downs);

        leds.show(Colour::Green);
        cortex_m::asm::delay(GREEN_DWELL_CYCLES);
    }

    let _pad_back = wake.disarm();
    drop(ctrl);

    match refusal {
        Some(signature) => leds.pulse(Colour::Red, signature),
        None => leds.pulse(Colour::Green, confirmed),
    }

    park(&mut leds).await
}
