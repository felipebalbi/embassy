#![no_std]
#![no_main]

//! FlexSPI XIP **library** example for MCXA577 (FRDM-MCXA577).
//!
//! This example does three things, in order:
//!
//! 1. **Locks the core to its highest frequency, no deep sleep.** The FIRC is
//!    boosted to 192 MHz and selected as the main clock under over-drive
//!    voltage, so the CPU runs at 192 MHz. `CoreSleep::WfeUngated` is kept
//!    (the default) and deep sleep is never enabled, so the part never leaves
//!    the active power mode.
//!
//! 2. **Locks FlexSPI to its highest frequency for the on-board flash.** The
//!    FRDM-MCXA577 carries a Winbond **W25Q64JV** quad NOR, driven here in 1-4-4
//!    Quad I/O (`0xEB`), which the board limits to 80 MHz. Sourcing FlexSPI from
//!    the 192 MHz FRO_HF and dividing by 3 yields a **64 MHz** SDR serial clock
//!    -- the highest integer-divisor rate under the 80 MHz cap (÷2 would be
//!    96 MHz, over the limit).
//!
//! 3. **Runs a whole mini-library out of external flash (XIP).** Unlike
//!    [`flexspi-xip-linked`], which relocates only code, this places both the
//!    library's code (`.xip_text`) *and* its 16 KiB lookup table (`.xip_rodata`)
//!    into the `XIP` window via `xip-lib.x`. The library is *linked* at its run
//!    address in the window (VMA = 0x8000_0000+) but *loaded* into internal
//!    flash (LMA, `AT > FLASH`); probe-rs programs internal flash as usual and
//!    this app copies the contiguous code+rodata image into external flash and
//!    branches to it. The XIP'd code then both *executes* and *reads its lookup
//!    table* through the FlexSPI AHB window, so the library lives **entirely** in
//!    external flash and the AHB data-read path is exercised as well.
//!
//! Self-checking: the same algorithm is mirrored in [`reference_digest`] (normal
//! internal-flash code reading an identical internal copy of the table), so the
//! test needs no hand-computed magic number -- it compares the value produced
//! *executing from flash* against the value computed normally. A `HardFault`
//! handler dumps fault status so an IBUSERR (CFSR bit 8) is unambiguous.

use defmt::{error, info, warn, unwrap};
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use hal::clocks::config::{CoreSleep, FircConfig, FircFreqSel, MainClockSource, VddDriveStrength};
use hal::clocks::periph_helpers::{Div4, FlexspiClockSel};
use hal::clocks::{PoweredClock, VddLevel};
use hal::config::Config;
use hal::flexspi::{ClockConfig as FlexspiClockConfig, Flexspi, NorFlash};
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::{FLASH_CONFIG, FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE};

/// FlexSPI AHB window base (non-secure alias) == the `XIP` region origin.
const FLEXSPI_AMBA_BASE: u32 = 0x8000_0000;

// ---------------------------------------------------------------------------
// The XIP'd mini-library.
//
// A small "digest" library: a permutation network that mixes its state through
// an S-box lookup table. The arithmetic lives in shared `#[inline(always)]`
// helpers so the external-flash (`xl_*`) and internal-flash (`reference_digest`)
// paths can never drift; only *which copy of the table* they read differs.
//
// Sizing: the S-box is 4096 * 4 = 16 KiB of `.xip_rodata`, so the library spans
// several flash sectors and many pages, with both its code and its data in
// external flash.
// ---------------------------------------------------------------------------

/// Number of entries in the S-box. Power of two so the index mask is exact and
/// callers can use unchecked loads (no bounds-check panic landing pad -- which,
/// in the XIP copy, would be an out-of-range branch back into internal flash).
const SBOX_LEN: usize = 4096;

/// Build the S-box at compile time with a Knuth multiplicative hash, so the
/// table is deterministic without a giant literal in the source.
const fn build_sbox() -> [u32; SBOX_LEN] {
    let mut t = [0u32; SBOX_LEN];
    let mut i = 0;
    while i < SBOX_LEN {
        let x = (i as u32).wrapping_mul(2_654_435_761);
        t[i] = x ^ (x >> 15);
        i += 1;
    }
    t
}

/// The canonical table contents, shared by both the XIP and reference copies.
const SBOX: [u32; SBOX_LEN] = build_sbox();

/// Mixing constants and schedule, shared by both paths.
const SEED_CONST: u32 = 0x1234_5678;
const C1: u32 = 0x9E37_79B1;
const C2: u32 = 0x85EB_CA77;
const K_STEP: u32 = 0x1000_0001;
const PERMUTE_ROUNDS: u32 = 64;

/// Index into the S-box derived from the current state. Masked to `0..SBOX_LEN`.
#[inline(always)]
fn table_index(state: u32) -> usize {
    ((state >> 11) as usize) & (SBOX_LEN - 1)
}

/// Read the S-box entry selected by `state`.
#[inline(always)]
fn load(table: &[u32; SBOX_LEN], state: u32) -> u32 {
    // SAFETY: `table_index` masks into `0..SBOX_LEN`, so the index is in bounds.
    // Using an unchecked load keeps the panic landing pad out of `.xip_text`.
    unsafe { *table.get_unchecked(table_index(state)) }
}

/// One mixing round. Pure arithmetic: no table, no calls -- guarantees the two
/// paths compute identical values once given the same table byte `t`.
#[inline(always)]
fn round_math(state: u32, k: u32, t: u32) -> u32 {
    (state.rotate_left(7) ^ t).wrapping_mul(C1).wrapping_add(k)
}

/// The external-flash library. Each function is a real `extern "C"`,
/// `#[inline(never)]` symbol placed in `.xip_text`, so the program counter
/// threads across the linked region via real `bl`s while the S-box reads come
/// from `.xip_rodata` -- both in external flash.
mod xiplib {
    use super::*;

    /// The library's S-box, placed in external flash.
    #[unsafe(link_section = ".xip_rodata")]
    static XIP_SBOX: [u32; SBOX_LEN] = SBOX;

    /// Leaf: read one S-box entry (an AHB data read from external flash).
    #[unsafe(link_section = ".xip_text")]
    #[inline(never)]
    extern "C" fn xl_lookup(state: u32) -> u32 {
        load(&XIP_SBOX, state)
    }

    /// One round: look up, then mix. Calls `xl_lookup` via a real `bl`.
    #[unsafe(link_section = ".xip_text")]
    #[inline(never)]
    extern "C" fn xl_round(state: u32, k: u32) -> u32 {
        let t = xl_lookup(state);
        round_math(state, k, t)
    }

    /// The permutation network: many rounds with an evolving key.
    #[unsafe(link_section = ".xip_text")]
    #[inline(never)]
    extern "C" fn xl_permute(seed: u32) -> u32 {
        let mut state = seed ^ SEED_CONST;
        let mut k = C2;
        let mut i = 0;
        while i < PERMUTE_ROUNDS {
            state = xl_round(state, k);
            k = k.wrapping_add(K_STEP);
            i += 1;
        }
        state
    }

    /// Final mixing with fixed keys.
    #[unsafe(link_section = ".xip_text")]
    #[inline(never)]
    extern "C" fn xl_finalize(state: u32) -> u32 {
        let a = xl_round(state, C1);
        let b = xl_round(a, C2);
        xl_round(b, C1 ^ C2)
    }

    /// Public entry point of the XIP'd library.
    #[unsafe(link_section = ".xip_text")]
    #[inline(never)]
    pub extern "C" fn xl_digest(seed: u32) -> u32 {
        let permuted = xl_permute(seed);
        xl_finalize(permuted)
    }
}

/// Internal-flash copy of the S-box, read by the reference implementation.
static REF_SBOX: [u32; SBOX_LEN] = SBOX;

/// One reference round (normal internal-flash code).
fn rl_round(state: u32, k: u32) -> u32 {
    round_math(state, k, load(&REF_SBOX, state))
}

/// Software mirror of [`xiplib::xl_digest`], computed entirely from internal
/// flash. Same operations, same order, same constants, same table values.
fn reference_digest(seed: u32) -> u32 {
    let mut state = seed ^ SEED_CONST;
    let mut k = C2;
    let mut i = 0;
    while i < PERMUTE_ROUNDS {
        state = rl_round(state, k);
        k = k.wrapping_add(K_STEP);
        i += 1;
    }
    let a = rl_round(state, C1);
    let b = rl_round(a, C2);
    rl_round(b, C1 ^ C2)
}

unsafe extern "C" {
    static __sxip: u8;
    static __exip: u8;
    static __sxip_load: u8;
}

type XipEntryFn = extern "C" fn(u32) -> u32;

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    let cfsr = unsafe { core::ptr::read_volatile(0xE000_ED28 as *const u32) };
    let hfsr = unsafe { core::ptr::read_volatile(0xE000_ED2C as *const u32) };
    let bfar = unsafe { core::ptr::read_volatile(0xE000_ED38 as *const u32) };
    error!(
        "HardFault: pc=0x{=u32:08x} cfsr=0x{=u32:08x} (bit8=IBUSERR) hfsr=0x{=u32:08x} bfar=0x{=u32:08x}",
        ef.pc(),
        cfsr,
        hfsr,
        bfar,
    );
    loop {}
}

/// Build a [`Config`] that locks the core to 192 MHz (over-drive) with deep
/// sleep disabled.
fn max_clock_config() -> Config {
    let mut cfg = Config::default();

    // FIRC (FRO_HF) at its top frequency, 192 MHz.
    let mut firc = FircConfig::default();
    firc.frequency = FircFreqSel::Mhz192;
    firc.power = PoweredClock::NormalEnabledDeepSleepDisabled;
    cfg.clock_cfg.firc = Some(firc);

    // Main clock from FRO_HF -> 192 MHz CPU. Over-drive voltage is required to
    // run the core this fast.
    cfg.clock_cfg.main_clock.source = MainClockSource::FircHfRoot;
    cfg.clock_cfg.vdd_power.active_mode.level = VddLevel::OverDriveMode;
    cfg.clock_cfg.vdd_power.active_mode.drive = VddDriveStrength::Normal;
    cfg.clock_cfg.vdd_power.low_power_mode.level = VddLevel::OverDriveMode;
    cfg.clock_cfg.vdd_power.low_power_mode.drive = VddDriveStrength::Normal;

    // No deep sleep: idle uses (ungated) WFE and never drops out of active mode.
    cfg.clock_cfg.vdd_power.core_sleep = CoreSleep::WfeUngated;

    cfg
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(max_clock_config());

    // info!("FlexSPI XIP library example: core 192 MHz, FlexSPI 192 MHz Quad I/O");

    let div = 3;

    // FlexSPI sourced from the 192 MHz FRO_HF, divided by 3 => 64 MHz SDR serial
    // clock -- the highest integer-divisor rate within the W25Q64JV's 80 MHz
    // Quad I/O limit on this board.
    let flexspi_clock = FlexspiClockConfig {
        power: PoweredClock::NormalEnabledDeepSleepDisabled,
        source: FlexspiClockSel::FroHf,
        div: unwrap!(Div4::from_divisor(div)),
    };

    let flexspi = unwrap!(Flexspi::new_blocking(
        p.FLEXSPI0,
        p.P3_0,
        p.P3_7,
        p.P3_6,
        p.P3_8,
        p.P3_9,
        p.P3_10,
        p.P3_11,
        flexspi_clock,
        FLASH_CONFIG,
    ));
    // Keep `flash` alive for the whole program: it backs the window we execute
    // (and read the lookup table) from, so it must not be dropped before/while
    // the XIP'd library runs.
    let mut flash = NorFlash::new(flexspi);

    let vma = core::ptr::addr_of!(__sxip) as usize;
    let end = core::ptr::addr_of!(__exip) as usize;
    let lma = core::ptr::addr_of!(__sxip_load) as usize;
    let len = end - vma;
    let flash_offset = (vma - FLEXSPI_AMBA_BASE as usize) as u32;
    // SAFETY: [lma, lma+len) is the loaded .xip_lib image (code + rodata) in
    // internal flash.
    let code = unsafe { core::slice::from_raw_parts(lma as *const u8, len) };

    // info!(
    //     ".xip_lib: vma=0x{=u32:08x} lma=0x{=u32:08x} len={=usize} ({=usize} pages, {=usize} sectors), \
    //      incl. {=usize}-byte S-box -> flash offset 0x{=u32:08x}",
    //     vma as u32,
    //     lma as u32,
    //     len,
    //     len.div_ceil(FLASH_PAGE_SIZE),
    //     len.div_ceil(FLASH_SECTOR_SIZE),
    //     SBOX_LEN * core::mem::size_of::<u32>(),
    //     flash_offset,
    // );

    // Copy the linked image (code + lookup table) from internal flash (LMA) into
    // external flash.
    for s in 0..len.div_ceil(FLASH_SECTOR_SIZE) {
        unwrap!(flash.blocking_erase_sector(flash_offset + (s * FLASH_SECTOR_SIZE) as u32));
    }
    let mut page_buf = [0u8; FLASH_PAGE_SIZE];
    let mut off = 0;
    while off < len {
        let chunk = core::cmp::min(FLASH_PAGE_SIZE, len - off);
        let prog = (chunk + 7) & !7;
        page_buf[..chunk].copy_from_slice(&code[off..off + chunk]);
        page_buf[chunk..prog].fill(0);
        unwrap!(flash.blocking_page_program(flash_offset + off as u32, &page_buf[..prog]));
        off += chunk;
    }

    // Verify every byte (code and rodata) through the AHB window.
    for (i, &want) in code.iter().enumerate() {
        // SAFETY: inside the live FlexSPI window.
        let got = unsafe { core::ptr::read_volatile((vma + i) as *const u8) };
        if got != want {
            error!("readback mismatch at byte {=usize}: 0x{=u8:02x} != 0x{=u8:02x}", i, got, want);
            loop {}
        }
    }
    // info!("Verified {=usize} bytes of the XIP library (code + S-box) in the window", len);

    // Execute the library from external flash via an absolute (indirect) call to
    // its entry's VMA, so no out-of-range direct `bl` is needed. Every round
    // also reads the S-box back through the AHB window.
    let entry_addr = xiplib::xl_digest as XipEntryFn as usize; // = vma_of(xl_digest) | 1
    // SAFETY: the bytes at the entry are the just-copied, verified code.
    let entry: XipEntryFn = unsafe { core::mem::transmute(entry_addr) };

    let seed: u32 = 0xC0FF_EE00;

    for _ in 1..10 {
        let start = Instant::now();
        let got = entry(seed);
        let elapsed = start.elapsed();
        let want = reference_digest(seed);

        if got == want {
            // info!("XIP library PASSED: 0x{=u32:08x} == reference (executed + read tables from flash)", got);
            warn!("{},{}", 192/div, elapsed.as_nanos());
        } else {
            error!("XIP library WRONG result: 0x{=u32:08x} != reference 0x{=u32:08x}", got, want);
        }

        Timer::after_millis(100).await;
    }

    loop {
        Timer::after_secs(1).await;
        info!("xip-library heartbeat");
    }
}
