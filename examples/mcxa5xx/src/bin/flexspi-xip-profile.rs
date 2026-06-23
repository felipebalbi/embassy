#![no_std]
#![no_main]

//! FlexSPI XIP **profiling matrix** for MCXA577 (FRDM-MCXA577).
//!
//! A companion to [`flexspi-xip-library`]. Where that example runs one large
//! digest workload, this one profiles a *set* of small kernels -- the same shape
//! a customer used to characterise XIP: `add`, FNV-1a, checksum, 4x4 matmul,
//! popcount, CRC32 (table-driven, so it exercises `.xip_rodata`), plus one
//! async task.
//!
//! Each kernel exists as **twin copies generated from one body** (so they can't
//! drift): an *internal-flash* copy (`*_int`) and an *external-flash* copy
//! (`*_xip`, placed in `.xip_text` / `.xip_rodata` via `xip-lib.x` and copied
//! into external flash at runtime). For every kernel the harness reports three
//! cycle counts via the DWT cycle counter (CYCCNT, ~5.21 ns at 192 MHz):
//!
//! * `internal` -- run from internal flash (baseline),
//! * `cold`     -- run from XIP right after invalidating the FlexSPI AHB read
//!                 buffer (so code + tables are re-fetched from external flash),
//! * `warm`     -- run from XIP again immediately, buffer now populated.
//!
//! Output is CSV, one line per kernel per run, tagged by kind so it is easy to
//! split:
//!
//! ```text
//! S,<name>,<flexspi_mhz>,<internal>,<cold>,<warm>      # sync kernel
//! A,<n_concurrent>,<flexspi_mhz>,<total>,<per_future>  # async task
//! ```
//!
//! Sweep `DIV` (re-flashing each time) to vary the FlexSPI serial clock and
//! build the full internal-vs-XIP-vs-frequency matrix.

use core::hint::black_box;

use cortex_m::peripheral::DWT;
use defmt::{error, info, warn, unwrap};
use embassy_executor::Spawner;
use embassy_futures::join::join4;
use embassy_futures::yield_now;
use embassy_time::Timer;
use hal::clocks::config::{CoreSleep, FircConfig, FircFreqSel, MainClockSource, VddDriveStrength};
use hal::clocks::periph_helpers::{Div4, FlexspiClockSel};
use hal::clocks::{PoweredClock, VddLevel};
use hal::config::Config;
use hal::flexspi::{ClockConfig as FlexspiClockConfig, Flexspi, NorFlash};
use hal::pac::flexspi::Clrahbrxbuf;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::{FLASH_CONFIG, FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE};

/// FlexSPI AHB window base (non-secure alias) == the `XIP` region origin.
const FLEXSPI_AMBA_BASE: u32 = 0x8000_0000;

/// Uniform kernel signature so one harness can time them all.
type KernelFn = extern "C" fn(u32) -> u32;

// ---------------------------------------------------------------------------
// Kernel bodies. Each is a pure `#[inline(always)]` function of `u32 -> u32`,
// using only wrapping arithmetic and unchecked indexing so the external-flash
// copies stay self-contained (no panic landing pads, no u64/div libcalls).
// ---------------------------------------------------------------------------

/// "Small hot code": a single mixing add.
#[inline(always)]
fn add_impl(x: u32) -> u32 {
    x.wrapping_add(0x9E37_79B1)
}

/// FNV-1a over the four bytes of `x`.
#[inline(always)]
fn fnv1a_impl(x: u32) -> u32 {
    let mut h = 0x811C_9DC5u32;
    let mut i = 0;
    while i < 4 {
        let b = (x >> (i * 8)) & 0xff;
        h ^= b;
        h = h.wrapping_mul(0x0100_0193);
        i += 1;
    }
    h
}

/// Rolling checksum over a short derived sequence.
#[inline(always)]
fn checksum_impl(x: u32) -> u32 {
    let mut s = 0u32;
    let mut v = x;
    let mut i = 0u32;
    while i < 64 {
        s = s.wrapping_add(v);
        v = v.rotate_left(1) ^ i;
        i += 1;
    }
    s
}

/// 4x4 u32 matrix squared, folded to a scalar. Stack-only (no rodata).
#[inline(always)]
fn matmul_impl(x: u32) -> u32 {
    let mut a = [0u32; 16];
    let mut i = 0;
    while i < 16 {
        let v = x.wrapping_add(i as u32).wrapping_mul(2_654_435_761);
        // SAFETY: i < 16.
        unsafe { *a.get_unchecked_mut(i) = v ^ (v >> 15) };
        i += 1;
    }
    let mut acc = 0u32;
    let mut r = 0;
    while r < 4 {
        let mut c = 0;
        while c < 4 {
            let mut s = 0u32;
            let mut k = 0;
            while k < 4 {
                // SAFETY: r,c,k < 4 => indices < 16.
                let av = unsafe { *a.get_unchecked(r * 4 + k) };
                let bv = unsafe { *a.get_unchecked(k * 4 + c) };
                s = s.wrapping_add(av.wrapping_mul(bv));
                k += 1;
            }
            acc = acc.wrapping_add(s.rotate_left((r * 4 + c) as u32));
            c += 1;
        }
        r += 1;
    }
    acc
}

/// Branch-free 32-bit popcount (SWAR), inlined so no libcall escapes to flash.
#[inline(always)]
fn popcnt32(mut v: u32) -> u32 {
    v = v.wrapping_sub((v >> 1) & 0x5555_5555);
    v = (v & 0x3333_3333).wrapping_add((v >> 2) & 0x3333_3333);
    v = v.wrapping_add(v >> 4) & 0x0F0F_0F0F;
    v.wrapping_mul(0x0101_0101) >> 24
}

/// Accumulated popcount over a derived sequence.
#[inline(always)]
fn popcount_impl(x: u32) -> u32 {
    let mut v = x;
    let mut acc = 0u32;
    let mut i = 0u32;
    while i < 64 {
        acc = acc.wrapping_add(popcnt32(v));
        v = v.wrapping_mul(2_654_435_761).rotate_left(1);
        i += 1;
    }
    acc
}

/// Standard CRC-32 (reflected, poly 0xEDB88320) lookup table, built at compile
/// time. Two identical copies are placed in internal and external flash so the
/// internal and XIP kernels read their own table.
const fn build_crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut n = 0u32;
    while n < 256 {
        let mut c = n;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        table[n as usize] = c;
        n += 1;
    }
    table
}
const CRC_TABLE: [u32; 256] = build_crc_table();
/// Internal-flash copy of the CRC table.
static CRC_INT: [u32; 256] = CRC_TABLE;
/// External-flash copy of the CRC table (read by the XIP kernel through the AHB
/// window).
#[unsafe(link_section = ".xip_rodata")]
static CRC_XIP: [u32; 256] = CRC_TABLE;

/// CRC-32 of the four bytes of `x` using the supplied table.
#[inline(always)]
fn crc32_impl(x: u32, table: &[u32; 256]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    let mut i = 0;
    while i < 4 {
        let b = (x >> (i * 8)) as u8;
        let idx = ((crc ^ b as u32) & 0xff) as usize;
        // SAFETY: idx is masked to 0..256.
        crc = (crc >> 8) ^ unsafe { *table.get_unchecked(idx) };
        i += 1;
    }
    !crc
}

// ---------------------------------------------------------------------------
// Twin internal / XIP copies. The macro emits both from one impl, so they can
// never drift. CRC32 is written out by hand because each copy must reference
// its own table.
// ---------------------------------------------------------------------------

macro_rules! kernel_pair {
    ($int:ident, $xip:ident, $body:path) => {
        #[inline(never)]
        extern "C" fn $int(x: u32) -> u32 {
            $body(x)
        }
        #[unsafe(link_section = ".xip_text")]
        #[inline(never)]
        extern "C" fn $xip(x: u32) -> u32 {
            $body(x)
        }
    };
}

kernel_pair!(add_int, add_xip, add_impl);
kernel_pair!(fnv1a_int, fnv1a_xip, fnv1a_impl);
kernel_pair!(checksum_int, checksum_xip, checksum_impl);
kernel_pair!(matmul_int, matmul_xip, matmul_impl);
kernel_pair!(popcount_int, popcount_xip, popcount_impl);

#[inline(never)]
extern "C" fn crc32_int(x: u32) -> u32 {
    crc32_impl(x, &CRC_INT)
}
#[unsafe(link_section = ".xip_text")]
#[inline(never)]
extern "C" fn crc32_xip(x: u32) -> u32 {
    crc32_impl(x, &CRC_XIP)
}

unsafe extern "C" {
    static __sxip: u8;
    static __exip: u8;
    static __sxip_load: u8;
}

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    let cfsr = unsafe { core::ptr::read_volatile(0xE000_ED28 as *const u32) };
    error!("HardFault: pc=0x{=u32:08x} cfsr=0x{=u32:08x} (bit8=IBUSERR)", ef.pc(), cfsr);
    loop {}
}

/// Invalidate the FlexSPI AHB read (prefetch) buffer so the next XIP access
/// re-fetches from external flash. `AHBCR.CLRAHBRXBUF` is a level "enable clear"
/// control (not self-clearing), so it is pulsed 1 -> 0.
#[inline(never)]
fn invalidate_flexspi_read_cache() {
    let ahbcr = hal::pac::FLEXSPI0.ahbcr();
    ahbcr.modify(|w| w.set_clrahbrxbuf(Clrahbrxbuf::Val1));
    ahbcr.modify(|w| w.set_clrahbrxbuf(Clrahbrxbuf::Val0));
    cortex_m::asm::dsb();
}

/// Time a single call through a function pointer, returning `(result, cycles)`.
/// `#[inline(never)]` + `black_box` keep the call pinned inside the timed region.
#[inline(never)]
fn time_call(f: KernelFn, seed: u32) -> (u32, u32) {
    let t0 = DWT::cycle_count();
    let r = f(black_box(seed));
    let cycles = DWT::cycle_count().wrapping_sub(t0);
    (black_box(r), cycles)
}

/// An async task doing XIP work split across `rounds` yields, so the executor
/// polls between chunks. Used to show per-future executor overhead amortising
/// under concurrency.
async fn profiled_future(f: KernelFn, seed: u32, rounds: u32) -> u32 {
    let mut acc = seed;
    let mut i = 0;
    while i < rounds {
        acc = f(acc);
        yield_now().await;
        i += 1;
    }
    acc
}

/// Build a [`Config`] that locks the core to 192 MHz (over-drive), no deep sleep.
fn max_clock_config() -> Config {
    let mut cfg = Config::default();

    let mut firc = FircConfig::default();
    firc.frequency = FircFreqSel::Mhz192;
    firc.power = PoweredClock::NormalEnabledDeepSleepDisabled;
    cfg.clock_cfg.firc = Some(firc);

    cfg.clock_cfg.main_clock.source = MainClockSource::FircHfRoot;
    cfg.clock_cfg.vdd_power.active_mode.level = VddLevel::OverDriveMode;
    cfg.clock_cfg.vdd_power.active_mode.drive = VddDriveStrength::Normal;
    cfg.clock_cfg.vdd_power.low_power_mode.level = VddLevel::OverDriveMode;
    cfg.clock_cfg.vdd_power.low_power_mode.drive = VddDriveStrength::Normal;
    cfg.clock_cfg.vdd_power.core_sleep = CoreSleep::WfeUngated;

    cfg
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = hal::init(max_clock_config());

    // Cycle-accurate timing via DWT CYCCNT.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    info!("# S,name,flexspi_mhz,internal,cold,warm  /  A,n,flexspi_mhz,total,per_future");

    for div in (1..=6).rev() {
        let flexspi_clock = FlexspiClockConfig {
            power: PoweredClock::NormalEnabledDeepSleepDisabled,
            source: FlexspiClockSel::FroHf,
            div: unwrap!(Div4::from_divisor(div)),
        };

        let flexspi = Flexspi::new_blocking(
            p.FLEXSPI0.reborrow(),
            p.P3_0.reborrow(),
            p.P3_7.reborrow(),
            p.P3_6.reborrow(),
            p.P3_8.reborrow(),
            p.P3_9.reborrow(),
            p.P3_10.reborrow(),
            p.P3_11.reborrow(),
            flexspi_clock,
            FLASH_CONFIG,
        ).unwrap();
        // Keep `flash` alive: it backs the window the XIP kernels execute from.
        let mut flash = NorFlash::new(flexspi);

        // Relocate the whole XIP image (all `.xip_text` kernels + the CRC table in
        // `.xip_rodata`) from its internal-flash load address into external flash.
        let vma = core::ptr::addr_of!(__sxip) as usize;
        let end = core::ptr::addr_of!(__exip) as usize;
        let lma = core::ptr::addr_of!(__sxip_load) as usize;
        let len = end - vma;
        let flash_offset = (vma - FLEXSPI_AMBA_BASE as usize) as u32;
        // SAFETY: [lma, lma+len) is the loaded .xip_lib image in internal flash.
        let code = unsafe { core::slice::from_raw_parts(lma as *const u8, len) };

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
        for (i, &want) in code.iter().enumerate() {
            // SAFETY: inside the live FlexSPI window.
            let got = unsafe { core::ptr::read_volatile((vma + i) as *const u8) };
            if got != want {
                error!("readback mismatch at byte {=usize}: 0x{=u8:02x} != 0x{=u8:02x}", i, got, want);
                loop {}
            }
        }

        let freq_mhz = (192 / div) as u32;
        let seed: u32 = 0xC0FF_EE00;

        // --- Synchronous kernels: internal vs XIP cold vs XIP warm ---------------
        let kernels: [(&str, KernelFn, KernelFn); 6] = [
            ("add", add_int, add_xip),
            ("fnv1a", fnv1a_int, fnv1a_xip),
            ("checksum", checksum_int, checksum_xip),
            ("matmul", matmul_int, matmul_xip),
            ("popcount", popcount_int, popcount_xip),
            ("crc32", crc32_int, crc32_xip),
        ];

        for (name, int_fn, xip_fn) in kernels {
            let (ri, internal) = time_call(int_fn, seed);
            invalidate_flexspi_read_cache();
            let (rc, cold) = time_call(xip_fn, seed);
            let (rw, warm) = time_call(xip_fn, seed);

            if ri != rc || ri != rw {
                error!(
                    "{=str}: MISMATCH int=0x{=u32:08x} cold=0x{=u32:08x} warm=0x{=u32:08x}",
                    name, ri, rc, rw
                );
            } else {
                warn!("S,{},{},{},{},{}", name, freq_mhz, internal, cold, warm);
            }
        }

        // --- Async task: one future vs four concurrent (per-future cost) ----------
        const AROUNDS: u32 = 16;

        let t0 = DWT::cycle_count();
        let r1 = profiled_future(fnv1a_xip, seed, AROUNDS).await;
        let single = DWT::cycle_count().wrapping_sub(t0);
        black_box(r1);

        let t0 = DWT::cycle_count();
        let r4 = join4(
            profiled_future(fnv1a_xip, seed ^ 0x11, AROUNDS),
            profiled_future(fnv1a_xip, seed ^ 0x22, AROUNDS),
            profiled_future(fnv1a_xip, seed ^ 0x33, AROUNDS),
            profiled_future(fnv1a_xip, seed ^ 0x44, AROUNDS),
        )
            .await;
        let four = DWT::cycle_count().wrapping_sub(t0);
        black_box(r4);

        warn!("A,1,{},{},{}", freq_mhz, single, single);
        warn!("A,4,{},{},{}", freq_mhz, four, four / 4);
    }

    info!("profiling complete");
    loop {
        Timer::after_secs(1).await;
    }
}
