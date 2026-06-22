#![no_std]
#![no_main]

//! FlexSPI XIP IBUSERR reproduction for MCXA577 (driver UNCHANGED).
//!
//! Customer reports an instruction-fetch bus error (`CFSR=0x100`, IBUSERR) when
//! executing a *large* function from the FlexSPI XIP window, specifically when a
//! build performs an **AHB data read of the code region immediately before
//! branching to it**. Data reads of the same address return correct bytes;
//! small functions execute fine.
//!
//! The existing `flexspi-xip-large` example does a *full sequential* readback
//! (which ends far past the function and leaves the prefetch buffer clean), then
//! executes — and it passes. This example instead mirrors the failing pattern:
//! it does a small AHB data read **at the entry** (like checking a prologue),
//! leaving the AHB RX prefetch buffer mid-stream from the entry, then branches
//! straight to the entry and runs a multi-KB body.
//!
//! A custom `HardFault` handler dumps the fault status so a reproduction is
//! unambiguous (expect `cfsr=0x00000100` and a `pc` inside the XIP window).
//!
//! This is a DIAGNOSTIC build: no driver changes. If it faults, we have a
//! hardware reproduction to fix against; if it passes, we adjust the trigger.

use defmt::{error, info, unwrap};
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::config::Config;
use hal::flexspi::{ClockConfig as FlexspiClockConfig, Flexspi, NorFlash};
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::{FLASH_CONFIG, FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE};

/// FlexSPI AHB memory-mapped window base. The customer's faulting addresses are
/// in the NON-SECURE alias (0x8000_0000); try that here (the secure 0x9000_0000
/// alias did not reproduce).
const FLEXSPI_AMBA_BASE: u32 = 0x8000_0000;
/// Host the function at the very start of the window so the entry and the early
/// instruction stream land at `window+0x00 .. window+0x100`, matching the
/// customer's faulting addresses (0x..80 / 0x..100).
const FLASH_OFFSET: u32 = 0x0000_0000;
/// How many bytes to read **at the entry** as AHB data right before branching.
const PRIME_READ_BYTES: usize = 64;

/// Unrolled mixing rounds (kept in sync with the `.rept` below).
const KERNEL_ROUNDS: u32 = 128;
const C1: u32 = 0x9E37_79B1;
const C2: u32 = 0x85EB_CA77;

// Unlike `flexspi-xip-large`, this kernel pulls its constants from a **literal
// pool** (the `.word` entries `8:`/`9:`) via PC-relative `ldr.w`, instead of
// `movw`/`movt` immediates. That mirrors "regular compiled Rust": while the code
// executes *from the XIP window* (instruction fetch), every round also performs
// a PC-relative **data read of the code region** (the literal-pool load),
// interleaving AHB instruction fetch and AHB data reads on the FlexSPI path.
// The pool is emitted inside the start..end range, so it is copied with the code
// and remains position-independent.
//
// Per round (r0 = state):
//   ldr.w r1, 8f        ; literal-pool load of C1  (PC-relative data read)
//   mul   r0, r0, r1    ; x  = x * C1
//   ldr.w r1, 9f        ; literal-pool load of C2
//   add   r0, r0, r1    ; x  = x + C2
//   eor   r0, r0, r0, lsr #15
//   ror   r0, r0, #13
core::arch::global_asm!(
    ".syntax unified",
    ".thumb",
    ".section .text.xip_kernel,\"ax\",%progbits",
    ".align 2",
    ".global xip_kernel",
    ".thumb_func",
    "xip_kernel:",
    ".rept 128", // keep in sync with KERNEL_ROUNDS
    "ldr.w r1, 8f",
    "mul r0, r0, r1",
    "ldr.w r1, 9f",
    "add r0, r0, r1",
    "eor r0, r0, r0, lsr #15",
    "ror r0, r0, #13",
    ".endr",
    "bx lr",
    ".align 2",
    "8: .word 0x9E3779B1",
    "9: .word 0x85EBCA77",
    ".global xip_kernel_end",
    "xip_kernel_end:",
);

unsafe extern "C" {
    fn xip_kernel(seed: u32) -> u32;
    static xip_kernel_end: u8;
}

type XipKernelFn = extern "C" fn(u32) -> u32;

fn kernel_reference(mut x: u32) -> u32 {
    for _ in 0..KERNEL_ROUNDS {
        x = x.wrapping_mul(C1);
        x = x.wrapping_add(C2);
        x ^= x >> 15;
        x = x.rotate_right(13);
    }
    x
}

fn kernel_bytes() -> &'static [u8] {
    let start = (xip_kernel as unsafe extern "C" fn(u32) -> u32 as usize) & !1usize;
    let end = core::ptr::addr_of!(xip_kernel_end) as usize;
    // SAFETY: start..end is the contiguous machine code of `xip_kernel`.
    unsafe { core::slice::from_raw_parts(start as *const u8, end - start) }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    // SAFETY: fixed System Control Block fault-status registers.
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

    info!("FlexSPI XIP IBUSERR reproduction (driver unchanged)");

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

    let code = kernel_bytes();
    let len = code.len();
    info!("Kernel {=usize} B ({=usize} pages); programming at offset 0x{=u32:08x}", len, len.div_ceil(FLASH_PAGE_SIZE), FLASH_OFFSET);

    for s in 0..len.div_ceil(FLASH_SECTOR_SIZE) {
        unwrap!(flash.blocking_erase_sector(FLASH_OFFSET + (s * FLASH_SECTOR_SIZE) as u32));
    }
    let mut page_buf = [0u8; FLASH_PAGE_SIZE];
    let mut off = 0;
    while off < len {
        let chunk = core::cmp::min(FLASH_PAGE_SIZE, len - off);
        let prog = (chunk + 7) & !7;
        page_buf[..chunk].copy_from_slice(&code[off..off + chunk]);
        page_buf[chunk..prog].fill(0);
        unwrap!(flash.blocking_page_program(FLASH_OFFSET + off as u32, &page_buf[..prog]));
        off += chunk;
    }
    info!("Programmed; entry will be 0x{=u32:08x}", FLEXSPI_AMBA_BASE + FLASH_OFFSET);

    // TRIGGER: AHB data read AT THE ENTRY immediately before branching. This is
    // the pattern the customer says provokes the fault (vs. a full readback that
    // ends elsewhere). Read volatile so it is not optimised away, and leave the
    // prefetch buffer streaming forward from the entry.
    let entry = FLEXSPI_AMBA_BASE + FLASH_OFFSET;
    let mut sink: u32 = 0;
    for i in 0..PRIME_READ_BYTES {
        let a = (entry + i as u32) as *const u8;
        // SAFETY: inside the live FlexSPI window.
        sink = sink.wrapping_add(unsafe { core::ptr::read_volatile(a) } as u32);
    }
    info!("Primed {=usize} B of AHB data reads at the entry (checksum {=u32}); branching now", PRIME_READ_BYTES, sink);

    // Execute straight from the window, right after the data read.
    // SAFETY: position-independent Thumb code; `| 1` sets the Thumb bit.
    let xip: XipKernelFn = unsafe { core::mem::transmute((entry | 1) as *const ()) };
    let seed: u32 = 0x1234_5678;
    let got = xip(seed);
    let want = kernel_reference(seed);

    if got == want {
        info!("No fault: kernel returned 0x{=u32:08x} (== reference). Trigger did not reproduce.", got);
    } else {
        error!("Executed but WRONG result: 0x{=u32:08x} != reference 0x{=u32:08x}", got, want);
    }

    loop {
        Timer::after_secs(1).await;
        info!("repro heartbeat");
    }
}
