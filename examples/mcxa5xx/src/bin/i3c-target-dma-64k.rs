//! Two-board I3C **target** half (DMA, 64 KiB payload soak).
//!
//! Pairs with `examples/mcxa2xx/src/bin/i3c-controller-dma-64k.rs`
//! (or either-side mate).
//!
//! Per iteration:
//!   1. drain 64 KiB from BBQ in `RX_CHUNK`-byte slices, verifying
//!      every byte == `0xAA` on the fly.
//!   2. send 64 KiB of `0x55` via `dma_respond_to_read_with_ibi` —
//!      driver chunks the payload internally.
//!
//! Wiring: SCL P0_21 ↔ MCXA2 P1_9, SDA P0_20 ↔ MCXA2 P1_8, common GND.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_mcxa::bind_interrupts;
use embassy_mcxa::clocks::config::Div8;
use embassy_mcxa::config::Config;
use embassy_mcxa::i3c::target::{self, Div4, Event, I3cClockSel};
use embassy_mcxa::peripherals::I3C0;
use static_cell::ConstStaticCell;
use {defmt::info, defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

const TARGET_ADDR: u8 = 0x0a;

const PAYLOAD_LEN: usize = 64 * 1024;
const WR_PATTERN_BYTE: u8 = 0xAA;
const RD_PATTERN_BYTE: u8 = 0x55;

const RX_CHUNK: usize = 4096;
const RX_BUF_SIZE: usize = 2 * RX_CHUNK;
static RX_BUF: ConstStaticCell<[u8; RX_BUF_SIZE]> = ConstStaticCell::new([0u8; RX_BUF_SIZE]);

static TX_55: [u8; PAYLOAD_LEN] = [RD_PATTERN_BYTE; PAYLOAD_LEN];

bind_interrupts!(
    struct Irqs {
        I3C0 => target::InterruptHandler<I3C0>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);

    let mut tgt_cfg = target::Config::default();
    tgt_cfg.address = Some(TARGET_ADDR);
    tgt_cfg.ibi_capable = true;
    tgt_cfg.clock_config.source = I3cClockSel::FroLfDiv;
    tgt_cfg.clock_config.div = Div4::from_divisor(1).unwrap();

    let rx_buf: &'static mut [u8] = RX_BUF.take();

    let tgt = target::I3c::new_dma(
        p.I3C0, p.P0_21, p.P0_20, p.DMA0_CH0, p.DMA0_CH1, Irqs, rx_buf, RX_CHUNK, tgt_cfg,
    )
    .unwrap();
    let mut tgt = tgt;

    info!("[tgt] up, listening (PAYLOAD={} bytes)", PAYLOAD_LEN);
    let mut sink = [0u8; RX_CHUNK];
    let mut iter: u32 = 0;

    loop {
        let ev = tgt.listen().await.unwrap();
        if let Event::RxPending = ev {
            // (1) drain 64 KiB, verifying on the fly.
            let mut received: usize = 0;
            while received < PAYLOAD_LEN {
                let want = (PAYLOAD_LEN - received).min(sink.len());
                let n = tgt.dma_respond_to_write(&mut sink[..want]).await.unwrap();
                if n == 0 {
                    defmt::error!("[tgt] iter {} zero-length grant at off={}", iter, received);
                    panic!("tgt zero grant");
                }
                if let Some((bad_idx, &bad)) =
                    sink[..n].iter().enumerate().find(|&(_, &b)| b != WR_PATTERN_BYTE)
                {
                    defmt::error!(
                        "[tgt] iter {} RX mismatch at off={} got=0x{:02x} expected=0x{:02x}",
                        iter,
                        received + bad_idx,
                        bad,
                        WR_PATTERN_BYTE
                    );
                    panic!("tgt rx mismatch");
                }
                received += n;
            }

            // (2) send 64 KiB via IBI-gated DMA read.
            match tgt.dma_respond_to_read_with_ibi(&TX_55).await {
                Ok(()) => {
                    if iter < 3 {
                        info!("[tgt] iter {} OK", iter);
                    }
                }
                Err(e) => {
                    defmt::error!("[tgt] iter {} read-with-ibi err {:?}", iter, e);
                    panic!("tgt read-with-ibi failed");
                }
            }

            iter = iter.wrapping_add(1);
            if iter % 100 == 0 {
                info!("[tgt] iter {} OK", iter);
            }
        }
    }
}
