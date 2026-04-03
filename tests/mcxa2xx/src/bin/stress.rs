//! MCXA2xx Peripheral Stress Test
//!
//! Exercises multiple peripheral drivers simultaneously and sequentially, targeting
//! corner cases that isolated per-peripheral tests may miss — particularly concurrency
//! interactions between the DMA engine, interrupt handlers, and the executor.
//!
//! # Wiring
//!
//! Connect the following jumper wires before running this test:
//!
//! | From  | To    | Signal / Purpose                                  |
//! |-------|-------|---------------------------------------------------|
//! | P2_2  | P4_2  | LPUART2 TX → LPUART3 RX (cross-loopback A)        |
//! | P4_5  | P2_3  | LPUART3 TX → LPUART2 RX (cross-loopback B)        |
//! | P1_0  | P1_2  | LPSPI0 MOSI → MISO (self-loopback)                |
//! | P1_9  | P1_1  | I2C SCL: LPI2C2 controller ↔ LPI2C1 target        |
//! | P1_8  | P1_0  | I2C SDA: LPI2C2 controller ↔ LPI2C1 target        |
//! | P1_8  | P2_4  | GPIO out ↔ GPIO/ADC/CTimer in (phases 3–5)        |
//!
//! **Note:** P1_8 has two wires connected to it. During the I2C phase (phase 2),
//! P1_8 acts as I2C SDA connected to P1_0. During GPIO/ADC/CTimer phases (3–5),
//! the I2C drivers are dropped first, so P1_0 floats harmlessly while P1_8 drives P2_4.
//!
//! # Test Phases
//!
//! **Phase 1 — Concurrent tasks** (all run in parallel via the embassy executor):
//!
//!   * `uart3_echo_task`: LPUART3 buffered — echoes back received bytes for each round.
//!   * `uart2_dma_stress`: LPUART2 DMA — sends varying-size packets and verifies echo.
//!     Packet sizes: 1, 7, 64, 255, 256, 257 bytes. Each round uses a distinct byte
//!     pattern so corruption is detectable (0x00, 0xFF, 0x55, 0xAA, sequential, mixed).
//!   * `spi0_stress`: LPSPI0 async — self-loopback with 1, 3, 7, 128-byte transfers.
//!     Patterns: 0x00, 0xFF, 0x55, 0xAA. Tests that DMA-based SPI and interrupt-based
//!     UART operate correctly when both are in flight at the same time.
//!   * `trng_stress`: TRNG — verifies consecutive random values differ across all
//!     supported APIs (blocking_128, blocking_256, blocking_512, custom blocking,
//!     custom async). Tests that the TRNG hardware entropy source remains live while
//!     other DMA-heavy tasks run concurrently.
//!
//! **Phase 1 inline (DMA mem-to-mem)**: While the tasks above run, the main task
//! also exercises DMA0_CH2 with mem-to-mem copies and memsets at sizes 1, 2, 3, 4,
//! and 256 words, back-to-back on the same channel.
//!
//! **Phase 2 — I2C stress** (LPI2C2 controller + LPI2C1 target):
//!
//!   Exercises dual-address write-read, read-only, multi-byte payloads, zero-length
//!   write (address-only NAK check), and NACK on an unregistered address. Repeated
//!   20 times to amplify any intermittent timing issue.
//!
//! **Phase 3 — GPIO async edge detection** (P1_8 output → P2_4 async input):
//!
//!   Tests wait_for_high, wait_for_low, wait_for_rising_edge, wait_for_falling_edge,
//!   and wait_for_any_edge. Then performs a rapid 20-pulse burst to verify that no
//!   edges are missed at high toggle frequency.
//!
//! **Phase 4 — ADC compare modes** (P1_8 GPIO drives P2_4 ADC channel):
//!
//!   StoreIf(GreaterThan): verifies no store below threshold, store above threshold.
//!   SkipUntil(GreaterThan): verifies FifoPending while below, then conversion once
//!   the pin rises. Tests boundary sensitivity around the 0x8000 midpoint.
//!
//! **Phase 5 — CTimer PWM + Capture** (P2_4 PWM output, P1_8 capture input):
//!
//!   Sweeps duty cycle through 10 %, 25 %, 50 %, 75 % at 20 kHz. After each change
//!   verifies that the captured frequency remains within 0.1 Hz of the target.
//!   Exercises that the CTimer peripheral correctly reconfigures without requiring
//!   a full driver teardown.

#![no_std]
#![no_main]

teleprobe_meta::target!(b"frdm-mcx-a266");

use embassy_executor::Spawner;
use embassy_mcxa::adc::{Command, CommandConfig, CommandId, Trigger};
use embassy_mcxa::bind_interrupts;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::clocks::periph_helpers::CTimerClockSel;
use embassy_mcxa::config::Config;
use embassy_mcxa::ctimer::CTimer;
use embassy_mcxa::ctimer::capture::{self, Capture, Edge};
use embassy_mcxa::ctimer::pwm::{SetDutyCycle, SinglePwm};
use embassy_mcxa::dma::{DmaChannel, TransferOptions};
use embassy_mcxa::gpio::{self, DriveStrength, Input, Level, Output, Pull, SlewRate};
use embassy_mcxa::i2c::{Async, controller, target};
use embassy_mcxa::lpuart::{Buffered, Lpuart};
use embassy_mcxa::spi::controller::{InterruptHandler as SpiHandler, Spi};
use embassy_mcxa::trng::{self, Trng};
use embassy_mcxa::{adc, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, WithTimeout as _};
use embedded_io_async::{Read, Write};
use hal::pac::adc::Mode;
use static_cell::ConstStaticCell;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

// ─── Phase 1 task completion channel ────────────────────────────────────────
//
// Each phase-1 task sends () when it completes successfully. The main task waits
// for all PHASE1_TASK_COUNT completions before advancing to phase 2.
const PHASE1_TASK_COUNT: usize = 4; // uart3_echo, uart2_dma_stress, spi1_stress, trng_stress
static PHASE1_DONE: Channel<CriticalSectionRawMutex, (), PHASE1_TASK_COUNT> = Channel::new();

// ─── I2C phase coordination ───────────────────────────────────────────────────
//
// I2C_TARGET_DONE: target task → main — confirms the target has exited cleanly.
// The target task runs a fixed number of iterations (I2C_ROUNDS × 6 transactions)
// then signals done naturally, without needing an external stop signal.
static I2C_TARGET_DONE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// ─── UART static ring buffers (required by Lpuart::new_buffered) ─────────────
//
// Size 512 is chosen to accommodate the largest single packet (257 bytes) with
// margin. The ring buffers are owned by the driver for its entire lifetime.
static UART3_TX_BUF: ConstStaticCell<[u8; 512]> = ConstStaticCell::new([0u8; 512]);
static UART3_RX_BUF: ConstStaticCell<[u8; 512]> = ConstStaticCell::new([0u8; 512]);

// ─── DMA mem-to-mem static buffers ───────────────────────────────────────────
//
// The 256-word arrays live in static storage because they are too large to stack-
// allocate safely on the main task (which also drives I2C, GPIO, ADC, and CTimer).
static DMA_SRC256: ConstStaticCell<[u32; 256]> = ConstStaticCell::new([0xA5A5A5A5u32; 256]);
static DMA_DST256: ConstStaticCell<[u32; 256]> = ConstStaticCell::new([0u32; 256]);

// ─── Peripheral sizes for UART stress rounds ─────────────────────────────────
//
// Each element drives one send-echo-verify round on UART2 DMA / UART3 buffered.
// The sizes deliberately straddle power-of-2 boundaries and the DMA transfer
// alignment to surface off-by-one handling in the driver chunking logic.
const UART_SIZES: [usize; 6] = [1, 7, 64, 255, 256, 257];
/// Maximum over `UART_SIZES` — used to size stack buffers inside tasks.
const UART_MAX: usize = 257;

// ─── Interrupt bindings ──────────────────────────────────────────────────────

bind_interrupts!(struct Irqs {
    // LPUART3 buffered mode uses a single IRQ for the ring-buffer ISR.
    LPUART3 => hal::lpuart::BufferedInterruptHandler::<hal::peripherals::LPUART3>;

    // LPSPI0 async mode uses a WaitCell woken from the SPI ISR.
    LPSPI0 => SpiHandler<hal::peripherals::LPSPI0>;

    // LPI2C1 as I2C target.
    LPI2C1 => target::InterruptHandler<hal::peripherals::LPI2C1>;

    // LPI2C2 as I2C controller.
    LPI2C2 => controller::InterruptHandler<hal::peripherals::LPI2C2>;

    // GPIO2 bank interrupt for P2_4 async edge detection (phase 3).
    GPIO2 => gpio::InterruptHandler<peripherals::GPIO2>;

    // CTIMER2 interrupt for the capture channel (phase 5).
    CTIMER2 => capture::InterruptHandler<hal::peripherals::CTIMER2>;

    // ADC0 end-of-conversion interrupt (phases 4).
    ADC0 => adc::InterruptHandler<hal::peripherals::ADC0>;

    // TRNG0 entropy-ready interrupt used in the async TRNG mode (phase 1 task).
    TRNG0 => hal::trng::InterruptHandler<hal::peripherals::TRNG0>;
});

// ─── Helper ──────────────────────────────────────────────────────────────────

/// Returns true if `x` is within `epsilon` of `target`.
fn within(x: f32, target: f32, epsilon: f32) -> bool {
    (x - target).abs() <= epsilon
}

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  Phase 1 tasks — all run concurrently from the moment they are spawned  ║
// ╚══════════════════════════════════════════════════════════════════════════╝

/// **UART3 echo task** — services the "remote" side of the cross-loopback.
///
/// For each round defined in `UART_SIZES` the task reads exactly that many bytes
/// from the LPUART3 buffered driver (which receives what UART2 sent via
/// P2_2 → P4_2) and echoes them back via P4_5 → P2_3 to UART2.
///
/// Using `read_exact` instead of `read` ensures partial reads never silently
/// truncate a packet, which would stall the sender waiting for bytes that never
/// arrive.
#[embassy_executor::task]
async fn uart3_echo_task(mut uart: Lpuart<'static, Buffered>) {
    let mut buf = [0u8; UART_MAX];

    for &size in &UART_SIZES {
        defmt::debug!("uart3_echo: round {}", size);
        uart.read_exact(&mut buf[..size]).await.unwrap();
        uart.write_all(&buf[..size]).await.unwrap();
        uart.flush().await.unwrap();
    }

    defmt::info!("uart3_echo_task: done");
    PHASE1_DONE.send(()).await;
}

/// **UART2 DMA stress task** — sends packets of increasing and non-power-of-2
/// sizes, waits for the echo from UART3, and verifies byte-exact integrity.
///
/// Each round uses a distinct byte pattern so that a shifted or partially
/// delivered packet cannot accidentally pass the equality check:
///
/// | Size | Fill pattern                     |
/// |------|----------------------------------|
/// | 1    | [0x00]                           |
/// | 7    | [0xFF, 0xFF, …]                  |
/// | 64   | [0x55, 0xAA, …] alternating      |
/// | 255  | sequential 0x00–0xFE             |
/// | 256  | sequential 0x00–0xFF             |
/// | 257  | byte i → (i as u8).wrapping_mul(3)|
#[embassy_executor::task]
async fn uart2_dma_stress(mut uart: Lpuart<'static, hal::lpuart::Dma<'static>>) {
    // Drain any stale bytes left in the FIFO from driver initialization.
    let mut drain = [0u8; 16];
    while uart
        .read(&mut drain)
        .with_timeout(Duration::from_millis(2))
        .await
        .is_ok()
    {}

    let patterns: [fn(usize) -> u8; 6] = [
        |_| 0x00u8,                               // round 0 — all zeros
        |_| 0xFFu8,                               // round 1 — all ones
        |i| if i & 1 == 0 { 0x55 } else { 0xAA }, // round 2 — alternating
        |i| i as u8,                              // round 3 — sequential
        |i| i as u8,                              // round 4 — sequential (256 B)
        |i| (i as u8).wrapping_mul(3),            // round 5 — stride-3
    ];

    let mut tx_buf = [0u8; UART_MAX];
    let mut rx_buf = [0u8; UART_MAX];

    for (round, (&size, pat)) in UART_SIZES.iter().zip(patterns.iter()).enumerate() {
        defmt::debug!("uart2_dma_stress: round {} ({} bytes)", round, size);

        // Fill TX buffer with this round's pattern.
        for (i, b) in tx_buf[..size].iter_mut().enumerate() {
            *b = pat(i);
        }

        uart.write(&tx_buf[..size]).await.unwrap();
        uart.read(&mut rx_buf[..size]).await.unwrap();

        assert_eq!(
            tx_buf[..size],
            rx_buf[..size],
            "UART2 DMA round {} ({} bytes): data mismatch",
            round,
            size,
        );
    }

    defmt::info!("uart2_dma_stress: done");
    PHASE1_DONE.send(()).await;
}

/// **SPI1 async self-loopback stress task** — exercises the LPSPI0 driver with
/// four transfer sizes and four bit patterns while UART DMA is simultaneously
/// active. Tests that the SPI interrupt handler and the UART interrupt handler
/// do not clobber each other's WaitCell state.
///
/// The MOSI→MISO loopback wire (P2_13 → P2_15) makes every byte written also
/// appear on MISO, so `async_transfer_in_place` is used to write and read in a
/// single pass.
#[embassy_executor::task]
async fn spi0_stress(mut spi: Spi<'static, hal::spi::Async>) {
    // Transfer sizes chosen to cover: minimum (1), odd non-power-of-2 (3, 7),
    // and a larger payload that exercises the async wait-cell path (128).
    const SIZES: &[usize] = &[1, 3, 7, 128];
    // Patterns: all-zero, all-one, checkerboard-high, checkerboard-low.
    const PATTERNS: &[u8] = &[0x00, 0xFF, 0x55, 0xAA];

    let mut buf = [0u8; 128];

    for &size in SIZES {
        for &pat in PATTERNS {
            defmt::debug!("spi1_stress: {} bytes, pattern 0x{:02X}", size, pat);
            buf[..size].fill(pat);

            spi.async_transfer_in_place(&mut buf[..size]).await.unwrap();

            for (i, &b) in buf[..size].iter().enumerate() {
                assert_eq!(
                    b, pat,
                    "SPI loopback mismatch at byte {}: got 0x{:02X}, expected 0x{:02X}",
                    i, b, pat
                );
            }
        }
    }

    defmt::info!("spi1_stress: done");
    PHASE1_DONE.send(()).await;
}

/// **TRNG stress task** — verifies that the true random number generator produces
/// distinct output across consecutive calls in all supported modes. Runs while DMA
/// and SPI are active to ensure the TRNG's entropy-ready interrupt is not masked or
/// delayed by concurrent interrupt activity.
///
/// Note: this test has a theoretical failure probability of ~1/2^32 per assertion
/// (two consecutive identical 32-bit values), matching the existing trng.rs test.
#[embassy_executor::task]
async fn trng_stress(mut trng0: hal::Peri<'static, hal::peripherals::TRNG0>) {
    // — Blocking custom config — also verifies u64 path.
    let config = trng::Config::default();
    let mut trng = Trng::new_blocking_with_custom_config(trng0.reborrow(), config);
    assert_ne!(trng.blocking_next_u64(), trng.blocking_next_u64());
    drop(trng);

    // — Async custom config — verifies entropy-ready interrupt fires correctly
    // even while SPI and UART DMA IRQs are pending concurrently.
    let mut trng = Trng::new_with_custom_config(trng0.reborrow(), Irqs, config);
    assert_ne!(
        trng.async_next_u32().await.unwrap(),
        trng.async_next_u32().await.unwrap()
    );
    assert_ne!(
        trng.async_next_u64().await.unwrap(),
        trng.async_next_u64().await.unwrap()
    );
    // Fill-bytes path: two successive 10-byte arrays must differ.
    let mut a = [0u8; 10];
    let mut b = [0u8; 10];
    trng.async_fill_bytes(&mut a).await.unwrap();
    trng.async_fill_bytes(&mut b).await.unwrap();
    assert_ne!(a, b);
    drop(trng);

    defmt::info!("trng_stress: done");
    PHASE1_DONE.send(()).await;
}

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  Phase 2 task — I2C target service loop                                 ║
// ╚══════════════════════════════════════════════════════════════════════════╝

/// Number of I2C rounds the controller executes.
const I2C_ROUNDS: usize = 20;

/// Number of target-visible transactions per controller round:
///   - async_write(0x13)      → Write(0x13)              = 1
///   - async_write(0x37)      → Write(0x37)              = 1
///   - async_read(0x13)       → Read(0x13)               = 1
///   - async_read(0x37)       → Read(0x37)               = 1
///   - async_write_read(0x13) → Write(0x13) + Read(0x13) = 2
///   - async_write(0x01)      → NACK, target not reached = 0
///   Total per round: 6 transactions.
const I2C_TARGET_TRANSACTIONS: usize = I2C_ROUNDS * 6;

/// **I2C target task** — listens on LPI2C1 and responds to exactly
/// `I2C_TARGET_TRANSACTIONS` requests from the LPI2C2 controller.
///
/// Each address (0x13, 0x37) holds a single independent byte that the
/// controller can write and then read back. After all expected transactions
/// are served, the task signals `I2C_TARGET_DONE` and exits.
#[embassy_executor::task]
async fn i2c_target_task(mut target: target::I2c<'static, Async>) {
    // One writable/readable byte per address.
    let mut val_13 = [0u8; 1];
    let mut val_37 = [0u8; 1];

    for _ in 0..I2C_TARGET_TRANSACTIONS {
        match target.async_listen().await {
            Ok(request) => {
                defmt::debug!("i2c_target: request {}", request);
                match request {
                    target::Request::Read(0x13) => {
                        target.async_respond_to_read(&val_13).await.unwrap();
                    }
                    target::Request::Read(0x37) => {
                        target.async_respond_to_read(&val_37).await.unwrap();
                    }
                    target::Request::Write(0x13) => {
                        target.async_respond_to_write(&mut val_13).await.unwrap();
                    }
                    target::Request::Write(0x37) => {
                        target.async_respond_to_write(&mut val_37).await.unwrap();
                    }
                    _ => {
                        defmt::warn!("i2c_target: unexpected request {}", request);
                    }
                }
            }
            Err(e) => {
                defmt::warn!("i2c_target: listen error {}", e);
            }
        }
    }

    I2C_TARGET_DONE.signal(());
}

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  Main                                                                   ║
// ╚══════════════════════════════════════════════════════════════════════════╝

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ─── Clock configuration ─────────────────────────────────────────────────
    //
    // FRO 12 MHz enabled for LPSPI and LPUART baud rate generation.
    // fro_lf_div = 1 MHz (no division) for CTimer PWM/capture reference clock.
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_12m_enabled = true;
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let mut p = hal::init(config);

    defmt::info!("=== MCXA2xx stress test start ===");

    // ═══════════════════════════════════════════════════════════════════════
    // PHASE 1: Concurrent UART cross-loopback, SPI self-loopback, TRNG
    // ═══════════════════════════════════════════════════════════════════════

    defmt::info!("--- Phase 1: concurrent tasks ---");

    // ── LPUART3 buffered driver (echo side) ───────────────────────────────
    let uart_cfg = hal::lpuart::Config {
        baudrate_bps: 115_200,
        rx_fifo_watermark: 0,
        tx_fifo_watermark: 0,
        ..Default::default()
    };
    let uart3 = Lpuart::new_buffered(
        p.LPUART3,
        p.P4_5, // TX — P4_5 → P2_3 (UART2 RX)
        p.P4_2, // RX — P4_2 ← P2_2 (UART2 TX)
        Irqs,
        UART3_TX_BUF.take(),
        UART3_RX_BUF.take(),
        uart_cfg,
    )
    .unwrap();

    // ── LPUART2 DMA driver (stress side) ─────────────────────────────────
    let uart2 = Lpuart::new_async_with_dma(
        p.LPUART2, p.P2_2,     // TX — P2_2 → P4_2 (UART3 RX)
        p.P2_3,     // RX — P2_3 ← P4_5 (UART3 TX)
        p.DMA0_CH0, // TX DMA channel
        p.DMA0_CH1, // RX DMA channel
        uart_cfg,
    )
    .unwrap();

    // ── LPSPI0 async driver (self-loopback MOSI → MISO) ──────────────────
    let spi0 = Spi::new_async(
        p.LPSPI0,
        p.P1_1, // SCK
        p.P1_0, // MOSI — wired to P2_15 MISO
        p.P1_2, // MISO — wired from P2_13 MOSI
        Irqs,
        hal::spi::controller::Config::default(),
    )
    .unwrap();

    // ── Spawn phase-1 tasks ───────────────────────────────────────────────
    spawner.spawn(uart3_echo_task(uart3).unwrap());
    spawner.spawn(uart2_dma_stress(uart2).unwrap());
    spawner.spawn(spi0_stress(spi0).unwrap());
    spawner.spawn(trng_stress(p.TRNG0).unwrap());

    // ── DMA mem-to-mem stress (inline, concurrent with spawned tasks) ─────
    //
    // Uses DMA0_CH2 — independent of CH0/CH1 used by UART2 — so this runs
    // genuinely in parallel at the hardware level.
    defmt::info!("phase1 inline: DMA mem-to-mem stress on CH2");
    {
        let mut ch2 = DmaChannel::new(p.DMA0_CH2);

        // Small sizes: stack-allocate source/destination arrays.
        for size in [1usize, 2, 3, 4] {
            let src = [0xCAFEu32; 4]; // only [..size] used
            let mut dst = [0u32; 4];

            let xfer = ch2
                .mem_to_mem(&src[..size], &mut dst[..size], TransferOptions::COMPLETE_INTERRUPT)
                .unwrap();
            xfer.await.unwrap();
            assert_eq!(&src[..size], &dst[..size], "DMA mem-to-mem size {} failed", size);

            let pattern = 0xDEAD_BEEFu32;
            let xfer = ch2
                .memset(&pattern, &mut dst[..size], TransferOptions::COMPLETE_INTERRUPT)
                .unwrap();
            xfer.await.unwrap();
            assert!(
                dst[..size].iter().all(|&v| v == pattern),
                "DMA memset size {} failed",
                size
            );
        }

        // Large size (256 words): use static buffers to avoid stack overflow.
        let src256 = DMA_SRC256.take();
        let dst256 = DMA_DST256.take();

        let xfer = ch2
            .mem_to_mem(src256, dst256, TransferOptions::COMPLETE_INTERRUPT)
            .unwrap();
        xfer.await.unwrap();
        assert_eq!(src256, dst256, "DMA mem-to-mem 256 words failed");

        let pattern = 0x1234_5678u32;
        let xfer = ch2
            .memset(&pattern, dst256, TransferOptions::COMPLETE_INTERRUPT)
            .unwrap();
        xfer.await.unwrap();
        assert!(dst256.iter().all(|&v| v == pattern), "DMA memset 256 words failed");

        // Back-to-back rapid transfers: verify no stale completion flag causes
        // an early return on the second transfer.
        for _ in 0..4 {
            let src_bb = [0xABCD_EF01u32; 4];
            let mut dst_bb = [0u32; 4];
            let xfer = ch2
                .mem_to_mem(&src_bb, &mut dst_bb, TransferOptions::COMPLETE_INTERRUPT)
                .unwrap();
            xfer.await.unwrap();
            assert_eq!(src_bb, dst_bb);
        }
    }
    defmt::info!("phase1 inline: DMA stress done");

    // ── Wait for all phase-1 tasks to complete ───────────────────────────
    for i in 0..PHASE1_TASK_COUNT {
        PHASE1_DONE.receive().await;
        defmt::info!("phase 1: task {} of {} completed", i + 1, PHASE1_TASK_COUNT);
    }
    defmt::info!("--- Phase 1 complete ---");

    // ═══════════════════════════════════════════════════════════════════════
    // PHASE 2: I2C controller + target stress
    // ═══════════════════════════════════════════════════════════════════════

    defmt::info!("--- Phase 2: I2C stress ---");

    // ── LPI2C1 as target ─────────────────────────────────────────────────
    let mut target_cfg = target::Config::default();
    target_cfg.address = target::Address::Dual(0x13, 0x37);
    let i2c_target = target::I2c::new_async(
        p.LPI2C1, p.P1_1, // SCL — wired to P1_9 (LPI2C2 SCL)
        p.P1_0, // SDA — wired to P1_8 (LPI2C2 SDA) and P2_4 (later phases)
        Irqs, target_cfg,
    )
    .unwrap();

    spawner.spawn(i2c_target_task(i2c_target).unwrap());

    // Give the target task a moment to enter async_listen before the first
    // controller transaction is issued.
    Timer::after_millis(10).await;

    // ── LPI2C2 as controller ──────────────────────────────────────────────
    // Use reborrow() for P1_8/P1_9 so the pins are released after drop(i2c)
    // and can be reconfigured as GPIO/CTimer in phases 3–5.
    let mut i2c = controller::I2c::new_async(
        p.LPI2C2,
        p.P1_9.reborrow(), // SCL — wired to P1_1 (LPI2C1 SCL); released on drop
        p.P1_8.reborrow(), // SDA — wired to P1_0 (LPI2C1 SDA); released on drop
        Irqs,
        controller::Config::default(),
    )
    .unwrap();

    // Run I2C_ROUNDS repetitions to amplify intermittent timing issues.
    for round in 0..I2C_ROUNDS {
        defmt::debug!("i2c: round {}", round);

        let write_val_13 = round as u8;
        let write_val_37 = (round + 100) as u8;

        // Write a single byte to address 0x13.
        i2c.async_write(0x13, &[write_val_13]).await.unwrap();
        // Write a single byte to address 0x37.
        i2c.async_write(0x37, &[write_val_37]).await.unwrap();

        // Read back from 0x13 — should reflect the last write.
        let mut buf = [0u8; 1];
        i2c.async_read(0x13, &mut buf).await.unwrap();
        assert_eq!(buf[0], write_val_13, "I2C read-back 0x13 failed at round {}", round);

        // Read back from 0x37.
        i2c.async_read(0x37, &mut buf).await.unwrap();
        assert_eq!(buf[0], write_val_37, "I2C read-back 0x37 failed at round {}", round);

        // Write-read (repeated-start) at 0x13: write register index, read value.
        i2c.async_write_read(0x13, &[write_val_13], &mut buf).await.unwrap();
        assert_eq!(buf[0], write_val_13, "I2C write-read 0x13 failed at round {}", round);

        // NACK on an unregistered address must return AddressNack error.
        let err = i2c.async_write(0x01, &[0]).await.unwrap_err();
        assert_eq!(
            err,
            controller::IOError::AddressNack,
            "Expected AddressNack at round {}",
            round,
        );
    }

    // Wait for the I2C target task to finish all its transactions and exit.
    I2C_TARGET_DONE.wait().await;
    drop(i2c);

    defmt::info!("--- Phase 2 complete ---");

    // ═══════════════════════════════════════════════════════════════════════
    // PHASE 3: GPIO async edge detection  (P1_8 output → P2_4 async input)
    // ═══════════════════════════════════════════════════════════════════════

    defmt::info!("--- Phase 3: GPIO async edge detection ---");

    // P1_8 is SDA for the (now dropped) I2C controller. We reconfigure it as a
    // push-pull GPIO output using reborrow() so the pin is released after this
    // phase and reused as CTimer capture input in phase 5.
    // P2_4 is also reborrowed — the final move happens in phase 5 (CTimer PWM).
    let mut gpio_out = Output::new(p.P1_8.reborrow(), Level::Low, DriveStrength::Normal, SlewRate::Slow);
    let mut gpio_in = Input::new_async(p.P2_4.reborrow(), Irqs, Pull::Down);

    // ── Basic edge-detection ─────────────────────────────────────────────

    // Rising edge: output starts low, drive high, wait for high.
    Timer::after_millis(5).await;
    gpio_out.set_high();
    gpio_in.wait_for_high().await;
    Timer::after_millis(2).await;
    assert!(gpio_in.is_high(), "GPIO: expected high after rising edge");

    // Falling edge.
    gpio_out.set_low();
    gpio_in.wait_for_low().await;
    Timer::after_millis(2).await;
    assert!(gpio_in.is_low(), "GPIO: expected low after falling edge");

    // wait_for_rising_edge.
    gpio_out.set_high();
    gpio_in.wait_for_rising_edge().await;
    Timer::after_millis(2).await;
    assert!(gpio_in.is_high(), "GPIO: expected high after wait_for_rising_edge");

    // wait_for_falling_edge.
    gpio_out.set_low();
    gpio_in.wait_for_falling_edge().await;
    Timer::after_millis(2).await;
    assert!(gpio_in.is_low(), "GPIO: expected low after wait_for_falling_edge");

    // wait_for_any_edge: the next edge is a rising edge.
    gpio_out.set_high();
    gpio_in.wait_for_any_edge().await;
    Timer::after_millis(2).await;
    assert!(
        gpio_in.is_high(),
        "GPIO: expected high after wait_for_any_edge (rising)"
    );

    // ── Rapid pulse burst — 20 edges ─────────────────────────────────────
    //
    // Verifies that the GPIO interrupt handler does not miss edges when toggles
    // occur in rapid succession (no inter-toggle sleep).
    defmt::info!("GPIO: rapid-pulse burst");
    gpio_out.set_low();
    Timer::after_millis(2).await;

    for _ in 0..10 {
        gpio_out.set_high();
        gpio_in.wait_for_high().await;
        gpio_out.set_low();
        gpio_in.wait_for_low().await;
    }
    assert!(gpio_in.is_low(), "GPIO: should be low after burst");

    drop(gpio_out);
    drop(gpio_in);

    defmt::info!("--- Phase 3 complete ---");

    // ═══════════════════════════════════════════════════════════════════════
    // PHASE 4: ADC compare modes  (P1_8 GPIO output → P2_4 ADC input)
    // ═══════════════════════════════════════════════════════════════════════

    defmt::info!("--- Phase 4: ADC compare modes ---");

    // Drive voltage on P2_4 via P1_8 GPIO output. P2_4 is a dual-function pin
    // that doubles as ADC channel input. Use reborrow() for both so the final
    // move into CTimer (phase 5) is still possible.
    let mut adc_drv = Output::new(p.P1_8.reborrow(), Level::Low, DriveStrength::Normal, SlewRate::Slow);

    // ── StoreIf(GreaterThan(0x8000)) ─────────────────────────────────────
    {
        let commands = &[Command::new_single(
            p.P2_4.reborrow(),
            CommandConfig {
                resolution: Mode::DATA_16_BITS,
                compare: adc::Compare::StoreIf(adc::CompareFunction::GreaterThan(0x8000)),
                ..Default::default()
            },
        )];
        let mut adc = adc::Adc::new_async(
            p.ADC0.reborrow(),
            Irqs,
            commands,
            &[Trigger {
                target_command_id: CommandId::Cmd1,
                ..Default::default()
            }],
            adc::Config::default(),
        )
        .unwrap();
        adc.do_offset_calibration();
        adc.do_auto_calibration();

        // Below threshold — no store expected.
        adc_drv.set_low();
        Timer::after_millis(5).await;
        adc.do_software_trigger(0b0001).unwrap();
        assert!(
            adc.wait_get_conversion().await.is_none(),
            "ADC StoreIf: expected no store while below threshold"
        );

        // Above threshold — store expected.
        adc_drv.set_high();
        Timer::after_millis(5).await;
        adc.do_software_trigger(0b0001).unwrap();
        assert!(
            adc.wait_get_conversion().await.is_some(),
            "ADC StoreIf: expected store while above threshold"
        );
    }

    // ── SkipUntil(GreaterThan(0x8000)) ───────────────────────────────────
    {
        adc_drv.set_low();

        let commands = &[Command::new_single(
            p.P2_4.reborrow(),
            CommandConfig {
                resolution: Mode::DATA_16_BITS,
                compare: adc::Compare::SkipUntil(adc::CompareFunction::GreaterThan(0x8000)),
                ..Default::default()
            },
        )];
        let mut adc = adc::Adc::new_async(
            p.ADC0.reborrow(),
            Irqs,
            commands,
            &[Trigger {
                target_command_id: CommandId::Cmd1,
                ..Default::default()
            }],
            adc::Config::default(),
        )
        .unwrap();
        adc.do_offset_calibration();
        adc.do_auto_calibration();

        // Output still low — trigger fires but result is held pending.
        adc.do_software_trigger(0b0001).unwrap();
        Timer::after_millis(100).await;
        assert_eq!(
            adc.try_get_conversion(),
            Err(adc::Error::FifoPending),
            "ADC SkipUntil: expected FifoPending while below threshold"
        );

        // Now raise the pin — the pending scan should complete and deliver a result.
        adc_drv.set_high();
        Timer::after_millis(5).await;
        assert!(
            adc.wait_get_conversion().await.is_some(),
            "ADC SkipUntil: expected conversion once above threshold"
        );
    }

    drop(adc_drv);

    defmt::info!("--- Phase 4 complete ---");

    // ═══════════════════════════════════════════════════════════════════════
    // PHASE 5: CTimer PWM (P2_4) + Capture (P1_8)
    // ═══════════════════════════════════════════════════════════════════════

    defmt::info!("--- Phase 5: CTimer PWM + Capture ---");

    // CTIMER1 generates PWM on P2_4 at 20 kHz. The signal travels across the
    // jumper wire to P1_8, where CTIMER2 measures the rising-edge period.
    let mut ctimer1_cfg: hal::ctimer::Config = Default::default();
    ctimer1_cfg.source = CTimerClockSel::Clk1M; // 1 MHz tick → measurable period
    let pin_ctimer = CTimer::new(p.CTIMER1, ctimer1_cfg).unwrap();
    let mut pwm_cfg: hal::ctimer::pwm::Config = Default::default();
    pwm_cfg.freq = 20_000; // 20 kHz
    let mut pwm = SinglePwm::new(pin_ctimer, p.CTIMER1_CH0, p.CTIMER1_CH2, p.P2_4, pwm_cfg).unwrap();

    let ctimer2 = CTimer::new(p.CTIMER2, Default::default()).unwrap();
    let mut cap_cfg = capture::Config::default();
    cap_cfg.edge = Edge::RisingEdge;
    let mut cap = Capture::new_with_input_pin(ctimer2, p.CTIMER2_CH0, p.P1_8, Irqs, cap_cfg).unwrap();

    // Sweep duty cycles. After each change, take two consecutive captures and
    // verify the inter-capture interval matches the expected frequency.
    const DUTY_CYCLES: &[u8] = &[10, 25, 50, 75];
    for &duty in DUTY_CYCLES {
        pwm.set_duty_cycle_percent(duty).unwrap();

        // Discard the first measurement after a duty-cycle change to avoid
        // measuring a partial period that straddles the configuration update.
        let _ = cap.capture().await.unwrap();

        let t1 = cap.capture().await.unwrap();
        let t2 = cap.capture().await.unwrap();
        let diff = t2 - t1;
        let freq = diff.to_frequency(cap.frequency());

        defmt::info!("CTimer: duty {}%, measured freq = {} Hz (expected 20000)", duty, freq);
        assert!(
            within(freq, 20_000.0, 1.0),
            "CTimer: frequency out of tolerance at duty {}% (got {} Hz)",
            duty,
            freq
        );
    }

    defmt::info!("--- Phase 5 complete ---");

    defmt::info!("=== MCXA2xx stress test: ALL PHASES PASSED ===");
    cortex_m::asm::bkpt();
}
