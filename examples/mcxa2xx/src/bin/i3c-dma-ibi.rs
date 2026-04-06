#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::bind_interrupts;
use hal::clocks::config::Div8;
use hal::config::Config;
use hal::i3c::controller::{self, BusType, I3c, InterruptHandler, Operation};
use hal::peripherals::I3C0;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        I3C0 => InterruptHandler<I3C0>;
    }
);

#[repr(u8)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum I3cCcc {
    /// Reset Dynamic Address Assignment.
    RstDaa = 0x06,
    /// Enable Events.
    Enec = 0x80,
    /// Set Dynamic Address from Static Address.
    SetDaSa = 0x87,
    /// Get Bus Characteristics Register.
    GetBcr = 0x8e,
}

impl From<I3cCcc> for u8 {
    fn from(value: I3cCcc) -> Self {
        value as u8
    }
}

const I3C_BROADCAST_ADDR: u8 = 0x7e;
/// Static address of the mcxa5xx target (matches target config.address).
const TARGET_SADDR: u8 = 0x2a;
/// Dynamic address assigned to the mcxa5xx target.
const TARGET_DADDR: u8 = 0x08;

/// Computes the SETDASA directed-frame payload: DA[6:0] with odd parity in bit 0.
///
/// The I3C spec requires the total number of 1 bits in the payload byte to be odd.
const fn setdasa_payload(daddr: u8) -> u8 {
    let parity = if daddr.count_ones() % 2 == 0 { 1u8 } else { 0u8 };
    (daddr << 1) | parity
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);

    defmt::info!("I3C controller <-> target IBI example");

    let config = controller::Config::default();
    let mut i3c = I3c::new_async_with_dma(p.I3C0, p.P1_9, p.P1_8, p.DMA0_CH0, p.DMA0_CH1, Irqs, config).unwrap();
    let mut bcr = [0u8; 1];

    defmt::info!("RSTDAA");
    i3c.async_transaction(
        &mut [Operation::Write {
            address: I3C_BROADCAST_ADDR,
            buf: &[I3cCcc::RstDaa.into()],
        }],
        BusType::I3cSdr,
    )
    .await
    .unwrap();

    Timer::after_micros(100).await;
    defmt::info!("SETDASA: 0x{:02x} -> 0x{:02x}", TARGET_SADDR, TARGET_DADDR);

    // Assign TARGET_DADDR to the target device at TARGET_SADDR.
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: I3C_BROADCAST_ADDR,
                buf: &[I3cCcc::SetDaSa.into()],
            },
            Operation::Write {
                address: TARGET_SADDR,
                buf: &[setdasa_payload(TARGET_DADDR)],
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();

    Timer::after_micros(50).await;

    // Read BCR to confirm IBI capability (BCR[1] should be set by target).
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: I3C_BROADCAST_ADDR,
                buf: &[I3cCcc::GetBcr.into()],
            },
            Operation::Read {
                address: TARGET_DADDR,
                buf: &mut bcr,
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();
    defmt::info!("BCR: 0x{:02x}", bcr[0]);

    defmt::info!("ENEC: enable IBIs");
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: I3C_BROADCAST_ADDR,
                buf: &[I3cCcc::Enec.into()],
            },
            Operation::Write {
                address: TARGET_DADDR,
                buf: &[0x01], // Enable IBIs
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();

    // TARGET_DADDR sends no mandatory byte (ibi_capable but no payload).
    i3c.configure_ibi(&[TARGET_DADDR], false).unwrap();

    let mut counter: u8 = 0;
    loop {
        defmt::info!("Writing to target (counter={})", counter);
        i3c.async_transaction(
            &mut [Operation::Write {
                address: TARGET_DADDR,
                buf: &[counter],
            }],
            BusType::I3cSdr,
        )
        .await
        .unwrap();

        defmt::info!("Waiting for IBI...");
        let mut ibi_payload = [0u8; 8];
        let (addr, payload_len) = i3c.async_wait_for_ibi(&mut ibi_payload).await.unwrap();
        defmt::info!("IBI from 0x{:02x}, payload_len={}", addr, payload_len);

        counter = counter.wrapping_add(1);
        Timer::after_millis(100).await;
    }
}

