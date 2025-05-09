//! Example on how to read a 24C/24LC i2c eeprom.
//!
//! Connect SDA to P0.03, SCL to P0.04

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use static_cell::ConstStaticCell;
use {defmt_rtt as _, panic_probe as _};

const ADDRESS: u8 = 0x50;

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    info!("Initializing TWI...");
    let config = twim::Config::default();
    static RAM_BUFFER: ConstStaticCell<[u8; 16]> = ConstStaticCell::new([0; 16]);
    let mut twi = Twim::new(p.TWISPI0, Irqs, p.P0_03, p.P0_04, config, RAM_BUFFER.take());

    info!("Reading...");

    let mut buf = [0u8; 16];
    unwrap!(twi.blocking_write_read(ADDRESS, &mut [0x00], &mut buf));

    info!("Read: {=[u8]:x}", buf);
}
