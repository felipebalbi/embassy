//! Two-board I3C **controller** half (DMA, 64 KiB payload soak).
//!
//! Pairs with `examples/mcxa2xx/src/bin/i3c-target-dma-64k.rs` (or
//! either-side mate).
//!
//! Per iteration:
//!   1. write 64 KiB of `0xAA` (HAL chunks at `DMA_MAX_TRANSFER_SIZE`)
//!   2. wait for IBI
//!   3. read 64 KiB (HAL chunks at `MAX_CHUNK_SIZE = 256`, Sr between)
//!   4. verify every byte == `0x55`.
//!
//! Wiring: SCL P0_21 ↔ MCXA2 P1_9, SDA P0_20 ↔ MCXA2 P1_8, common GND.

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_mcxa::bind_interrupts;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::config::Config;
use embassy_mcxa::i3c::controller::{self, BusType, I3c, IbiSlot, InterruptHandler, Operation, Payload};
use embassy_mcxa::peripherals::I3C0;
use embassy_time::Timer;
use static_cell::ConstStaticCell;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

const TARGET_STATIC_ADDR: u8 = 0x0a;
const TARGET_DYNAMIC_ADDR: u8 = 0x0b;

const PAYLOAD_LEN: usize = 64 * 1024;
const WR_PATTERN_BYTE: u8 = 0xAA;
const RD_PATTERN_BYTE: u8 = 0x55;

// TX pattern lives in flash (`.rodata`); eDMA reads it directly.
static TX_AA: [u8; PAYLOAD_LEN] = [WR_PATTERN_BYTE; PAYLOAD_LEN];

// RX sink in RAM. 64 KiB on the stack would blow the default stack.
static RX_BUF: ConstStaticCell<[u8; PAYLOAD_LEN]> = ConstStaticCell::new([0u8; PAYLOAD_LEN]);

bind_interrupts!(
    struct Irqs {
        I3C0 => InterruptHandler<I3C0>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);

    {
        use embassy_mcxa::i3c::PurPin;
        PurPin::<I3C0>::mux(&*p.P0_2);
    }

    let cfg = controller::Config::default();
    let mut i3c = I3c::new_async_with_dma(p.I3C0, p.P0_21, p.P0_20, p.DMA0_CH0, p.DMA0_CH1, Irqs, cfg).unwrap();

    let rx_buf: &'static mut [u8; PAYLOAD_LEN] = RX_BUF.take();

    Timer::after_secs(2).await;
    info!("[ctrl] RSTDAA");
    i3c.async_write(0x7e, &[0x06], BusType::I3cSdr).await.unwrap();
    info!("[ctrl] SETDASA");
    i3c.async_transaction(
        &mut [
            Operation::Write {
                address: 0x7e,
                buf: &[0x87],
            },
            Operation::Write {
                address: TARGET_STATIC_ADDR,
                buf: &[TARGET_DYNAMIC_ADDR << 1],
            },
        ],
        BusType::I3cSdr,
    )
    .await
    .unwrap();
    info!("[ctrl] register_ibi");
    i3c.register_ibi(IbiSlot::Slot0, TARGET_DYNAMIC_ADDR, Payload::Yes)
        .unwrap();
    info!("[ctrl] entering loop (PAYLOAD={} bytes)", PAYLOAD_LEN);

    let mut iter: u32 = 0;
    loop {
        // (1) write 64 KiB of 0xAA.
        i3c.async_write(TARGET_DYNAMIC_ADDR, &TX_AA, BusType::I3cSdr)
            .await
            .unwrap();

        // (2) wait for IBI.
        let mut ibi_buf = [0u8; 8];
        let (_ibi_addr, _ibi_len) = i3c.async_wait_for_ibi(&mut ibi_buf).await.unwrap();

        // (3) read 64 KiB.
        rx_buf.fill(0);
        match i3c.async_read(TARGET_DYNAMIC_ADDR, &mut rx_buf[..], BusType::I3cSdr).await {
            Ok(n) => {
                if n != PAYLOAD_LEN {
                    defmt::error!("[ctrl] iter {} short read: n={} expected={}", iter, n, PAYLOAD_LEN);
                    panic!("ctrl short read");
                }
                // (4) verify every byte.
                if let Some((bad_idx, &bad)) =
                    rx_buf.iter().enumerate().find(|&(_, &b)| b != RD_PATTERN_BYTE)
                {
                    defmt::error!(
                        "[ctrl] iter {} pattern mismatch at idx={} got=0x{:02x} expected=0x{:02x}",
                        iter,
                        bad_idx,
                        bad,
                        RD_PATTERN_BYTE
                    );
                    panic!("ctrl pattern mismatch");
                }
                if iter < 3 {
                    info!("[ctrl] iter {} OK n={}", iter, n);
                }
            }
            Err(e) => {
                defmt::error!("[ctrl] iter {} async_read err {:?}", iter, e);
                panic!("ctrl read err");
            }
        }

        iter = iter.wrapping_add(1);
        if iter % 100 == 0 {
            info!("[ctrl] iter {} OK", iter);
        }
    }
}
