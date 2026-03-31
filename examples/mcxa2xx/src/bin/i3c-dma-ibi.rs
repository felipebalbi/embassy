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
    /// Enter Dynamic Address Assignment.
    EntDaa = 0x07,
    /// Enable Events.
    Enec = 0x80,
    /// Disable Events.
    Disec = 0x81,
    /// Set Dynamic Address from Static Address.
    SetDaSa = 0x87,
    /// Set New Dynamic Address.
    SetNewDa = 0x88,
    /// Get Provisional ID.
    GetPid = 0x8d,
    /// Get Bus Characteristics Register.
    GetBcr = 0x8e,
    /// Get Device Characteristics Register.
    GetDcr = 0x8f,
    /// Get device status.
    GetStatus = 0x90,
    /// Target reset action.
    RstAct = 0x9a,
}

impl From<I3cCcc> for u8 {
    fn from(value: I3cCcc) -> Self {
        value as u8
    }
}

const I3C_BROADCAST_ADDR: u8 = 0x7e;
const P3T1755_SADDR: u8 = 0x48;
const P3T1755_DADDR: u8 = 0x42;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);

    defmt::info!("I3C example");

    let config = controller::Config::default();
    let mut i3c = I3c::new_async_with_dma(p.I3C0, p.P1_9, p.P1_8, p.DMA0_CH0, p.DMA0_CH1, Irqs, config).unwrap();
    let mut buf = [0u8; 2];

    defmt::info!("RESET");

    // Reset dynamic address assignment to make sure device responds to I3C requests.
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
    defmt::info!("SETDASA");

    // Set dynamic address from static address.
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: I3C_BROADCAST_ADDR,
                buf: &[I3cCcc::SetDaSa.into()],
            },
            Operation::Write {
                address: P3T1755_SADDR,
                buf: &[P3T1755_DADDR << 1],
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();

    Timer::after_micros(50).await;

    // Get BCR
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: I3C_BROADCAST_ADDR,
                buf: &[I3cCcc::GetBcr.into()],
            },
            Operation::Read {
                address: P3T1755_DADDR,
                buf: &mut buf[..1],
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();

    defmt::info!("BCR: {:02x}", buf[0]);

    defmt::info!("ENEC");
    // Enable IBIs using new dynamic address.
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: I3C_BROADCAST_ADDR,
                buf: &[I3cCcc::Enec.into()],
            },
            Operation::Write {
                address: P3T1755_DADDR,
                buf: &[0x01], // Enable IBIs
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();

    defmt::info!("CONFIG");
    let low = celsius_to_raw(25.0);
    let high = celsius_to_raw(27.0);
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: P3T1755_DADDR,
                buf: &[0x02, low[0], low[1]], // Low limit = 25C
            },
            Operation::Write {
                address: P3T1755_DADDR,
                buf: &[0x03, high[0], high[1]], // High Limit = 31C
            },
            Operation::Write {
                address: P3T1755_DADDR,
                buf: &[0x01, 0x28],
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();

    i3c.async_write_read(P3T1755_DADDR, &[0x02], &mut buf, BusType::I3cSdr)
        .await
        .unwrap();
    defmt::info!("raw low {:02x}", buf);
    let low = raw_to_celsius(buf);

    i3c.async_write_read(P3T1755_DADDR, &[0x03], &mut buf, BusType::I3cSdr)
        .await
        .unwrap();
    let high = raw_to_celsius(buf);

    i3c.async_write_read(P3T1755_DADDR, &[0x00], &mut buf, BusType::I3cSdr)
        .await
        .unwrap();
    let current = raw_to_celsius(buf);

    defmt::info!("low {}C high {}C current {}C", low, high, current);

    loop {
        defmt::info!("WAIT");
        let addr = i3c.async_wait_for_ibi().await.unwrap();

        i3c.async_write_read(addr, &[0x00], &mut buf, BusType::I3cSdr)
            .await
            .unwrap();

        let temp = raw_to_celsius(buf);
        defmt::info!("Received IBI from 0x{:02x}, temperature = {}C", addr, temp);
    }
}

fn raw_to_celsius(raw: [u8; 2]) -> f32 {
    let raw = i16::from_be_bytes(raw) / 16;
    f32::from(raw) * 0.0625
}

fn celsius_to_raw(temp: f32) -> [u8; 2] {
    let raw = ((temp / 0.0625) as i16) * 16;
    raw.to_be_bytes()
}
