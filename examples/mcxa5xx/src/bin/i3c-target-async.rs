//! I3C controller ↔ target loopback on the same MCXA5 board.
//!
//! This example uses two I3C peripherals on the same MCU:
//!   - **I3C0** as controller (DMA mode) on P0_21 (SCL) / P0_20 (SDA)
//!   - **I3C2** as target (async mode) on P4_3 (SCL) / P4_4 (SDA)
//!
//! Wire the two buses together (SCL↔SCL, SDA↔SDA) with external pull-ups.
//!
//! Protocol:
//!   1. Controller resets DAA, then assigns target address 0x0A via SETDASA
//!   2. Controller enables IBI events (ENEC)
//!   3. Controller writes 0x55 to the target
//!   4. Target receives 0x55, queues 0xaa for read, then sends IBI
//!   5. Controller receives IBI, reads back 0xAA from target

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::bind_interrupts;
use embassy_mcxa::clocks::PoweredClock;
use embassy_mcxa::clocks::config::{
    CoreSleep, Div8, FircConfig, FircFreqSel, FlashSleep, MainClockConfig, MainClockSource, VddDriveStrength, VddLevel,
};
use embassy_mcxa::clocks::periph_helpers::{Div4, I3cClockSel};
use embassy_mcxa::i3c::controller::{BusType, Operation};
use embassy_mcxa::i3c::{Async, controller, target};
use embassy_mcxa::peripherals::{I3C0, I3C2};
use embassy_time::Timer;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

/// Target static address (used during SETDASA to locate the target on the bus).
const TARGET_STATIC_ADDR: u8 = 0x0a;

/// Dynamic address assigned to the target by the controller.
const TARGET_DYNAMIC_ADDR: u8 = 0x0a;

bind_interrupts!(
    struct Irqs {
        I3C0 => controller::InterruptHandler<I3C0>;
        I3C2 => target::InterruptHandler<I3C2>;
    }
);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut cfg = hal::config::Config::default();

    // Enable 192 MHz clock source
    let mut fcfg = FircConfig::default();
    fcfg.frequency = FircFreqSel::Mhz192;
    fcfg.power = PoweredClock::NormalEnabledDeepSleepDisabled;
    fcfg.fro_hf_enabled = true;
    fcfg.clk_hf_fundamental_enabled = false;
    fcfg.fro_hf_div = Some(const { Div8::from_divisor(4).unwrap() });
    cfg.clock_cfg.firc = Some(fcfg);

    // Enable 12 MHz slow oscillator
    cfg.clock_cfg.sirc.fro_12m_enabled = true;
    cfg.clock_cfg.sirc.fro_lf_div = Some(Div8::no_div());
    cfg.clock_cfg.sirc.power = PoweredClock::AlwaysEnabled;

    cfg.clock_cfg.fro16k = None;
    cfg.clock_cfg.sosc = None;
    cfg.clock_cfg.spll = None;

    // Core clock from 192 MHz FRO
    cfg.clock_cfg.main_clock = MainClockConfig {
        source: MainClockSource::FircHfRoot,
        power: PoweredClock::NormalEnabledDeepSleepDisabled,
        ahb_clk_div: Div8::no_div(),
    };

    cfg.clock_cfg.vdd_power.active_mode.level = VddLevel::OverDriveMode;
    cfg.clock_cfg.vdd_power.low_power_mode.level = VddLevel::MidDriveMode;
    cfg.clock_cfg.vdd_power.active_mode.drive = VddDriveStrength::Normal;
    cfg.clock_cfg.vdd_power.low_power_mode.drive = VddDriveStrength::Low { enable_bandgap: false };
    cfg.clock_cfg.vdd_power.core_sleep = CoreSleep::WfeUngated;
    cfg.clock_cfg.vdd_power.flash_sleep = FlashSleep::FlashDoze;

    let p = hal::init(cfg);

    defmt::info!("I3C controller <-> target loopback example");

    // ── Target setup (I3C2, async) ────────────────────────────────────
    let mut tgt_cfg = target::Config::default();
    tgt_cfg.clock_config.source = I3cClockSel::FroHfDiv;
    tgt_cfg.clock_config.div = Div4::from_divisor(2).unwrap();
    tgt_cfg.address = Some(TARGET_STATIC_ADDR);
    tgt_cfg.ibi_capable = true;

    let tgt = target::I3c::new_async(p.I3C2, p.P4_3, p.P4_4, Irqs, tgt_cfg).unwrap();

    spawner.spawn(target_task(tgt).unwrap());

    // ── Controller setup (I3C0, DMA) ──────────────────────────────────
    let mut ctrl_cfg = controller::Config::default();
    ctrl_cfg.clock_config.source = I3cClockSel::FroHfDiv;
    ctrl_cfg.clock_config.div = Div4::no_div();

    let mut ctrl =
        controller::I3c::new_async_with_dma(p.I3C0, p.P0_21, p.P0_20, p.DMA0_CH0, p.DMA0_CH1, Irqs, ctrl_cfg).unwrap();

    // ── Bus enumeration ───────────────────────────────────────────────
    // Allow the target some time to settle after init.
    Timer::after_millis(10).await;

    // RSTDAA
    ctrl.blocking_reset_daa().unwrap();

    // SETDASA: assign TARGET_DYNAMIC_ADDR to the target at TARGET_STATIC_ADDR
    ctrl.async_transaction(
        &mut [
            Operation::Write {
                address: 0x7e,
                buf: &[0x87], // SETDASA CCC
            },
            Operation::Write {
                address: TARGET_STATIC_ADDR,
                buf: &[TARGET_DYNAMIC_ADDR << 1],
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();
    defmt::info!("[ctrl] SETDASA done — target at 0x{:02x}", TARGET_DYNAMIC_ADDR);

    // ENEC: enable IBI events from the target
    ctrl.async_transaction(
        &mut [
            Operation::Write {
                address: 0x7e,
                buf: &[0x81], // ENEC CCC (broadcast)
            },
            Operation::Write {
                address: TARGET_DYNAMIC_ADDR,
                buf: &[TARGET_DYNAMIC_ADDR << 1, 0x01],
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();
    defmt::info!("[ctrl] ENEC done — IBI enabled");

    Timer::after_secs(1).await;

    loop {
        // Phase 1: Controller writes 0x55, target receives it.
        defmt::info!("[ctrl] write 0x55");
        ctrl.async_write(TARGET_DYNAMIC_ADDR, &[0x55], BusType::I3cSdr)
            .await
            .unwrap();

        // Phase 2: controller waits for IBI.
        let mut ibi_payload = [0u8; 8];
        let (addr, len) = ctrl.async_wait_for_ibi(&mut ibi_payload).await.unwrap();
        defmt::info!("[ctrl] IBI from 0x{:02x}, payload {} byte(s)", addr, len);

        // Phase 3: Controller reads 0xAA back, target provides it.
        let mut read_buf = [0u8; 1];
        ctrl.async_read(TARGET_DYNAMIC_ADDR, &mut read_buf, BusType::I3cSdr)
            .await
            .unwrap();
        defmt::info!("[ctrl] read back: {:02x}", read_buf);

        if read_buf[0] == 0xaa {
            defmt::info!("[ctrl] SUCCESS — loopback verified!");
        } else {
            defmt::error!("[ctrl] MISMATCH — expected 0xaa, got {:02x}", read_buf);
        }
    }
}

#[embassy_executor::task]
async fn target_task(mut tgt: target::I3c<'static, Async>) {
    loop {
        defmt::info!("[target] waiting for write from controller...");
        // Wait for a write from the controller, then print the received data.
        let mut rx_buf = [0u8; 32];
        let n = tgt.async_respond_to_write(&mut rx_buf).await.unwrap();
        defmt::info!("[target] received {} byte(s): {:02x}", n, rx_buf[..n]);

        // Queue 0xAA for read, then send IBI to notify the controller.
        defmt::info!("[target] will send IBI");
        tgt.async_send_ibi(&[]).await.unwrap();
        defmt::info!("[target] sent IBI");

        // Wait for a read from the controller, then provide the response data.
        tgt.async_respond_to_read(&[0xaa]).await.unwrap();
    }
}
