use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Generate memory.x - put "FLASH" at start of RAM, RAM after "FLASH"
    // cortex-m-rt expects FLASH for code, RAM for data/bss/stack
    // Both are in RAM, but separated to satisfy cortex-m-rt's expectations
    // MCXA256 has 128KB RAM total
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    // Custom load-region fragment for the FlexSPI XIP example. Emitted next to
    // memory.x (same search path) and applied only to that one bin via
    // rustc-link-arg-bin, so other examples are unaffected.
    File::create(out.join("xip.x"))
        .unwrap()
        .write_all(include_bytes!("xip.x"))
        .unwrap();

    // Load-region fragment for the FlexSPI XIP *library* example. Like xip.x but
    // also relocates `.xip_rodata`, so a whole mini-library (code + lookup
    // tables) lives entirely in external flash. Applied only to that one bin.
    File::create(out.join("xip-lib.x"))
        .unwrap()
        .write_all(include_bytes!("xip-lib.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bin=flexspi-xip-linked=-Txip.x");
    println!("cargo:rustc-link-arg-bin=flexspi-xip-library=-Txip-lib.x");
    println!("cargo:rustc-link-arg-bin=flexspi-xip-profile=-Txip-lib.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=xip.x");
    println!("cargo:rerun-if-changed=xip-lib.x");
}
