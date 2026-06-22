#![no_std]
#![no_main]

//! Large FlexSPI execute-in-place (XIP) example for MCXA577.
//!
//! A follow-up to [`flexspi-xip-execute`], built to investigate reports that
//! *large* XIP functions misbehave. Instead of an 8-byte `add`, this programs a
//! multi-kilobyte routine into external NOR flash and executes it from the
//! FlexSPI AHB window, so the instruction-fetch path is stressed across many
//! flash pages.
//!
//! The routine stays **position-independent** so no linker script / load-region
//! work is needed (the customer's constraint):
//! * It is defined with [`core::arch::global_asm!`] in the normal `.text`
//!   (internal flash), then its bytes are copied into external flash at runtime
//!   and run from there. Nothing is *placed* by the linker at the flash window.
//! * It is a leaf function: no `bl`/`blx` to other symbols (which would embed a
//!   location-dependent offset), no `.data`/`.rodata` references, and no literal
//!   pool — every constant is materialised with `movw`/`movt` immediates. Loop
//!   bodies are fully unrolled with `.rept`, so the whole thing is one
//!   self-relative blob that runs correctly at any address.
//!
//! Self-checking: the exact same arithmetic is mirrored in [`kernel_reference`],
//! so the test needs no hand-computed magic number — it just compares the value
//! produced *executing from flash* against the value computed normally.
//!
//! Notes for MCXA577:
//! * The core boots secure, so the secure window alias `0x9000_0000` is used; it
//!   lives in the `0x8000_0000..=0x9FFF_FFFF` region, executable in the Armv8-M
//!   default memory map (no MPU/SAU change needed).

use defmt::{info, panic, unwrap};
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::config::Config;
use hal::flexspi::{ClockConfig as FlexspiClockConfig, Flexspi, NorFlash};
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::{FLASH_CONFIG, FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE};

/// FlexSPI AHB memory-mapped window base (secure alias; the core boots secure).
const FLEXSPI_AMBA_BASE: u32 = 0x8000_0000;
/// Flash offset to host the routine (sector aligned, under the 16 MiB 3-byte
/// address reach of the shared `FLASH_CONFIG` read sequence).
const FLASH_OFFSET: u32 = 0x0040_0000;

/// Number of unrolled mixing rounds in the kernel.
///
/// MUST stay in sync with the `.rept` count in the `global_asm!` block below.
/// At ~12 bytes/round plus a small preamble this yields a multi-kilobyte blob
/// spanning many flash pages.
const KERNEL_ROUNDS: u32 = 512;

/// Mixing constants, shared between the asm kernel and [`kernel_reference`].
const C1: u32 = 0x9E37_79B1;
const C2: u32 = 0x85EB_CA77;

// The XIP'd routine: `extern "C" fn(u32) -> u32`.
//
// Per round (r0 = state, r1 = C1, r2 = C2, r3 = scratch):
//   mul r0, r0, r1     ; x  = x * C1
//   lsr r3, r0, #15    ; t  = x >> 15
//   eor r0, r0, r3     ; x ^= t
//   add r0, r0, r2     ; x  = x + C2
//   ror r0, r0, #13    ; x  = x.rotate_right(13)
//
// Leaf, no memory access, no calls, no literal pool -> position independent.
core::arch::global_asm!(
    ".syntax unified",
    ".thumb",
    ".section .text.xip_kernel,\"ax\",%progbits",
    ".align 2",
    ".global xip_kernel",
    ".thumb_func",
    "xip_kernel:",
    "movw r1, #0x79B1",
    "movt r1, #0x9E37",
    "movw r2, #0xCA77",
    "movt r2, #0x85EB",
    ".rept 512", // keep in sync with KERNEL_ROUNDS
    "mul r0, r0, r1",
    "lsr r3, r0, #15",
    "eor r0, r0, r3",
    "add r0, r0, r2",
    "ror r0, r0, #13",
    ".endr",
    "bx lr",
    ".global xip_kernel_end",
    "xip_kernel_end:",
);

unsafe extern "C" {
    fn xip_kernel(seed: u32) -> u32;
    static xip_kernel_end: u8;
}

/// The C ABI of [`xip_kernel`].
type XipKernelFn = extern "C" fn(u32) -> u32;

/// Software mirror of the asm kernel — same ops, same order, same constants.
fn kernel_reference(mut x: u32) -> u32 {
    for _ in 0..KERNEL_ROUNDS {
        x = x.wrapping_mul(C1);
        x ^= x >> 15;
        x = x.wrapping_add(C2);
        x = x.rotate_right(13);
    }
    x
}

/// Byte span of the kernel in the (internal-flash) image.
fn kernel_bytes() -> &'static [u8] {
    // The function-pointer value carries the Thumb bit (bit 0); mask it to get
    // the true first instruction byte. `xip_kernel_end` is a plain label placed
    // right after `bx lr`.
    let start = (xip_kernel as unsafe extern "C" fn(u32) -> u32 as usize) & !1usize;
    let end = core::ptr::addr_of!(xip_kernel_end) as usize;
    // SAFETY: start..end is the contiguous machine code of `xip_kernel` in the
    // program image; both symbols come from the same `global_asm!` block.
    unsafe { core::slice::from_raw_parts(start as *const u8, end - start) }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Config::default());

    info!("FlexSPI large-XIP (execute-from-flash) example");

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
    // Keep `flash` alive for the whole program: it backs the window we execute
    // from, so it must not be dropped before/while the XIP'd code runs.
    let mut flash = NorFlash::new(flexspi);

    let code = kernel_bytes();
    let len = code.len();
    info!(
        "Kernel is {=usize} bytes ({=usize} pages, {=usize} sectors); programming at offset 0x{=u32:08x}",
        len,
        len.div_ceil(FLASH_PAGE_SIZE),
        len.div_ceil(FLASH_SECTOR_SIZE),
        FLASH_OFFSET,
    );

    // Erase every sector the kernel spans.
    for s in 0..len.div_ceil(FLASH_SECTOR_SIZE) {
        unwrap!(flash.blocking_erase_sector(FLASH_OFFSET + (s * FLASH_SECTOR_SIZE) as u32));
    }

    // Program page by page straight from the image. The final (partial) page is
    // staged into a buffer and its length rounded up to the driver's 8-byte
    // write granularity; the pad bytes follow `bx lr`, so they never execute.
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

    // Verify every byte through the memory-mapped window before executing it.
    for (i, &want) in code.iter().enumerate() {
        let addr = (FLEXSPI_AMBA_BASE + FLASH_OFFSET + i as u32) as *const u8;
        // SAFETY: inside the FlexSPI window configured by the live driver.
        let got = unsafe { core::ptr::read_volatile(addr) };
        if got != want {
            panic!(
                "readback mismatch at kernel byte {=usize}: programmed 0x{=u8:02x}, window 0x{=u8:02x}",
                i, want, got
            );
        }
    }
    info!("All {=usize} kernel bytes verified in the memory-mapped window", len);

    // Execute the whole multi-KB routine straight out of external flash.
    let entry = FLEXSPI_AMBA_BASE + FLASH_OFFSET;
    // SAFETY: `entry` points at the verified position-independent Thumb code;
    // `| 1` sets the Thumb bit for the interworking call; the ABI matches.
    let xip_kernel_in_flash: XipKernelFn = unsafe { core::mem::transmute((entry | 1) as *const ()) };

    let seed: u32 = 0x1234_5678;
    info!("Calling {=u32}-round kernel from 0x{=u32:08x} with seed 0x{=u32:08x}", KERNEL_ROUNDS, entry, seed);
    let from_flash = xip_kernel_in_flash(seed);
    let reference = kernel_reference(seed);

    if from_flash == reference {
        info!(
            "Large XIP PASSED: flash execution == reference == 0x{=u32:08x}",
            from_flash
        );
    } else {
        panic!(
            "Large XIP FAILED: flash gave 0x{=u32:08x}, reference is 0x{=u32:08x}",
            from_flash, reference
        );
    }

    loop {
        Timer::after_secs(1).await;
        info!("FlexSPI large-XIP demo heartbeat");
    }
}
