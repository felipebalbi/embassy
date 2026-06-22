#![no_std]
#![no_main]

//! Linked-XIP reproduction for MCXA577: real compiled Rust executing from the
//! FlexSPI window, with inter-function calls, literal pools, and absolute
//! references resolved at the run address.
//!
//! The XIP routines live in `.xip_text`, which `xip.x` links at its **run
//! address** in the `XIP` window (VMA = 0x8000_0000+) but **loads into internal
//! FLASH** (LMA, via `AT > FLASH`). probe-rs programs the bytes into internal
//! flash as usual; at runtime this app copies them from the LMA into external
//! flash and branches to the VMA. probe-rs never touches external flash.
//!
//! To exercise a *large* instruction stream (the customer faults deep, ~0x100),
//! the XIP code is a deep **call chain** `xf00 -> xf01 -> ... -> xfNN`: each
//! function does a little mixing (with literal-pool constants) and tail-calls
//! the next via a real PC-relative `bl`, so the program counter threads across
//! the whole linked region with a `bl`/`ret` at every step.
//!
//! A `HardFault` handler dumps fault status so an IBUSERR (CFSR bit 8) is
//! unambiguous. Driver is UNCHANGED.

use defmt::{error, info, unwrap};
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::config::Config;
use hal::flexspi::{ClockConfig as FlexspiClockConfig, Flexspi, NorFlash};
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::{FLASH_CONFIG, FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE};

/// FlexSPI AHB window base (non-secure alias) == the `XIP` region origin.
const FLEXSPI_AMBA_BASE: u32 = 0x8000_0000;
const C1: u32 = 0x9E37_79B1;
const C2: u32 = 0x85EB_CA77;

/// Per-link mixing step (inlined into each XIP function, so each carries its own
/// literal-pool constants).
#[inline(always)]
fn g(x: u32) -> u32 {
    x.wrapping_mul(C1) ^ (x >> 11)
}
/// Terminal step.
#[inline(always)]
fn h(x: u32) -> u32 {
    x.rotate_right(7).wrapping_mul(C1).wrapping_add(C2)
}

/// Generate a chain of `extern "C"` functions in `.xip_text`: each non-terminal
/// applies `g` and tail-calls the next; the terminal applies `h`.
macro_rules! xip_chain {
    ($leaf:ident) => {
        #[unsafe(link_section = ".xip_text")]
        #[inline(never)]
        extern "C" fn $leaf(x: u32) -> u32 { h(x) }
    };
    ($head:ident, $next:ident $(, $rest:ident)*) => {
        #[unsafe(link_section = ".xip_text")]
        #[inline(never)]
        extern "C" fn $head(x: u32) -> u32 { $next(g(x)) }
        xip_chain!($next $(, $rest)*);
    };
}

// 48 non-terminal links (xf00..xf47) + 1 terminal (xf48).
xip_chain!(
    xf00, xf01, xf02, xf03, xf04, xf05, xf06, xf07, xf08, xf09, xf10, xf11, xf12, xf13, xf14, xf15,
    xf16, xf17, xf18, xf19, xf20, xf21, xf22, xf23, xf24, xf25, xf26, xf27, xf28, xf29, xf30, xf31,
    xf32, xf33, xf34, xf35, xf36, xf37, xf38, xf39, xf40, xf41, xf42, xf43, xf44, xf45, xf46, xf47,
    xf48
);

/// Number of `g` applications before the terminal `h` (non-terminal link count).
const CHAIN_DEPTH: u32 = 48;

/// Software mirror of the chain (normal internal-flash code) for self-checking.
fn ref_chain(seed: u32) -> u32 {
    let mut x = seed;
    for _ in 0..CHAIN_DEPTH {
        x = g(x);
    }
    h(x)
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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Config::default());

    info!("FlexSPI linked-XIP reproduction (driver unchanged)");

    let flexspi = unwrap!(Flexspi::new_blocking(
        p.FLEXSPI0,
        p.P3_0,
        p.P3_7,
        p.P3_6,
        p.P3_8,
        p.P3_9,
        p.P3_10,
        p.P3_11,
        FlexspiClockConfig::default(),
        FLASH_CONFIG,
    ));
    let mut flash = NorFlash::new(flexspi);

    let vma = core::ptr::addr_of!(__sxip) as usize;
    let end = core::ptr::addr_of!(__exip) as usize;
    let lma = core::ptr::addr_of!(__sxip_load) as usize;
    let len = end - vma;
    let flash_offset = (vma - FLEXSPI_AMBA_BASE as usize) as u32;
    // SAFETY: [lma, lma+len) is the loaded .xip_text image in internal flash.
    let code = unsafe { core::slice::from_raw_parts(lma as *const u8, len) };

    info!(
        ".xip_text: vma=0x{=u32:08x} lma=0x{=u32:08x} len={=usize} ({=usize} pages) -> flash offset 0x{=u32:08x}",
        vma as u32, lma as u32, len, len.div_ceil(FLASH_PAGE_SIZE), flash_offset,
    );

    // Copy the linked image from internal flash (LMA) into external flash.
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

    // Verify through the window.
    for (i, &want) in code.iter().enumerate() {
        // SAFETY: inside the live FlexSPI window.
        let got = unsafe { core::ptr::read_volatile((vma + i) as *const u8) };
        if got != want {
            error!("readback mismatch at byte {=usize}: 0x{=u8:02x} != 0x{=u8:02x}", i, got, want);
            loop {}
        }
    }
    info!("Verified {=usize} bytes of linked XIP code in the window", len);

    // TRIGGER: AHB data read at the entry immediately before branching.
    let mut sink: u32 = 0;
    for i in 0..64usize {
        // SAFETY: inside the live FlexSPI window.
        sink = sink.wrapping_add(unsafe { core::ptr::read_volatile((vma + i) as *const u8) } as u32);
    }
    info!("Primed entry reads (checksum {=u32}); branching to linked XIP chain at 0x{=u32:08x}", sink, vma as u32);

    // Execute the chain from external flash via an absolute (indirect) call to
    // the head's VMA (xf00), so no out-of-range direct `bl` is needed.
    let entry_addr = xf00 as XipEntryFn as usize; // = vma | 1 (Thumb)
    // SAFETY: the bytes at the chain head are the just-copied, verified code.
    let entry: XipEntryFn = unsafe { core::mem::transmute(entry_addr) };

    let seed: u32 = 0x1234_5678;
    let got = entry(seed);
    let want = ref_chain(seed);

    if got == want {
        info!("Linked XIP PASSED: 0x{=u32:08x} == reference (no fault)", got);
    } else {
        error!("Linked XIP WRONG result: 0x{=u32:08x} != reference 0x{=u32:08x}", got, want);
    }

    loop {
        Timer::after_secs(1).await;
        info!("linked-XIP heartbeat");
    }
}
