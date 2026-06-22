#![no_std]
#![no_main]

//! FlexSPI execute-in-place (XIP) example for MCXA577.
//!
//! Goes one step beyond [`flexspi-quad-memory-mapped`]: instead of only *reading*
//! data through the FlexSPI AHB window, it has the core **fetch and execute
//! instructions** from external NOR flash.
//!
//! The flow is fully self-contained — no linker-script, boot header, or
//! debugger external-flash programming required:
//!
//! 1. [`Flexspi::new_blocking`] configures `FLEXSPI0`, which makes the external
//!    flash readable (and executable) through the AHB memory-mapped window.
//! 2. A tiny **position-independent Thumb-2 function** (`add r0, r0, r1; bx lr`)
//!    is programmed into a flash sector via IP commands.
//! 3. The bytes are read back through the window to confirm they landed and that
//!    the AHB read path returns fresh data.
//! 4. A function pointer into the window (with the Thumb bit set) is called, and
//!    the result is checked: `xip_add(20, 22) == 42` proves the instructions ran
//!    straight out of external flash.
//!
//! Notes for MCXA577:
//! * The core boots secure, so the secure window alias `0x9000_0000` is used.
//!   It lives in the `0x8000_0000..=0x9FFF_FFFF` region, which is executable in
//!   the Armv8-M default memory map (not execute-never), so no MPU/SAU change is
//!   needed.
//! * The blob is hand-encoded and position-independent (no literal pool, no
//!   absolute references), so it executes correctly at whatever address it ends
//!   up at. Its two trailing `nop`s only pad the program to the driver's 8-byte
//!   write granularity; they sit after `bx lr` and are never executed.

use defmt::{info, panic, unwrap};
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::config::Config;
use hal::flexspi::{ClockConfig as FlexspiClockConfig, Flexspi, NorFlash};
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

#[path = "../flexspi_common.rs"]
mod flexspi_common;

use flexspi_common::FLASH_CONFIG;

/// FlexSPI AHB memory-mapped window base (secure alias; the core boots secure).
const FLEXSPI_AMBA_BASE: u32 = 0x9000_0000;
/// Flash offset to host the function (sector aligned, under the 16 MiB 3-byte
/// address reach of the shared `FLASH_CONFIG` read sequence).
const FLASH_OFFSET: u32 = 0x0040_0000;

/// Machine code for the XIP'd function, as little-endian Thumb-2 halfwords:
///
/// ```text
///   4408   add r0, r0, r1   ; r0 = a + b   (AAPCS: a in r0, b in r1, ret in r0)
///   4770   bx  lr           ; return
///   bf00   nop              ; padding to the 8-byte write granularity
///   bf00   nop
/// ```
///
/// Position-independent: no literals or absolute addresses, so it runs correctly
/// from any address. The signature it implements is `extern "C" fn(u32, u32) -> u32`.
const XIP_ADD_CODE: [u8; 8] = [0x08, 0x44, 0x70, 0x47, 0x00, 0xbf, 0x00, 0xbf];

/// The C ABI of the function encoded in [`XIP_ADD_CODE`].
type XipAddFn = extern "C" fn(u32, u32) -> u32;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Config::default());

    info!("FlexSPI XIP (execute-from-flash) example");

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
    // Keep `flash` alive for the whole program: dropping it must not happen
    // before (or while) we execute from the window it backs.
    let mut flash = NorFlash::new(flexspi);

    // 1. Program the function into a freshly erased sector via IP commands.
    info!(
        "Programming {=usize}-byte XIP function at flash offset 0x{=u32:08x}",
        XIP_ADD_CODE.len(),
        FLASH_OFFSET
    );
    unwrap!(flash.blocking_erase_sector(FLASH_OFFSET));
    unwrap!(flash.blocking_page_program(FLASH_OFFSET, &XIP_ADD_CODE));

    // 2. Read it back through the memory-mapped window and confirm it matches.
    //    This also exercises the AHB read path before we fetch instructions.
    let mut readback = [0u8; XIP_ADD_CODE.len()];
    for (i, b) in readback.iter_mut().enumerate() {
        let addr = (FLEXSPI_AMBA_BASE + FLASH_OFFSET + i as u32) as *const u8;
        // SAFETY: inside the FlexSPI window configured by the live driver.
        *b = unsafe { core::ptr::read_volatile(addr) };
    }
    if readback != XIP_ADD_CODE {
        panic!(
            "readback mismatch: programmed {=[u8]:#04x} but window returned {=[u8]:#04x}",
            XIP_ADD_CODE, readback
        );
    }
    info!("Function bytes verified in the memory-mapped window");

    // 3. Build a Thumb function pointer into the window and execute it.
    let entry = FLEXSPI_AMBA_BASE + FLASH_OFFSET;
    // SAFETY: `entry` points at the position-independent Thumb code just
    // programmed and verified above. The `| 1` sets the Thumb state bit
    // required by `bx`/`blx` for an interworking call. The encoded routine
    // matches the `extern "C" fn(u32, u32) -> u32` ABI exactly.
    let xip_add: XipAddFn = unsafe { core::mem::transmute((entry | 1) as *const ()) };

    info!("Calling xip_add(20, 22) executing from 0x{=u32:08x}", entry);
    let result = xip_add(20, 22);

    if result == 42 {
        info!("XIP PASSED: code executed from external flash returned {=u32}", result);
    } else {
        panic!("XIP FAILED: expected 42, got {=u32}", result);
    }

    loop {
        Timer::after_secs(1).await;
        info!("FlexSPI XIP demo heartbeat");
    }
}
