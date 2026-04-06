#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::bind_interrupts;
use hal::clocks::config::Div8;
use hal::config::Config;
use hal::i3c::controller::{self, I3c, InterruptHandler};
use hal::peripherals::I3C0;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        I3C0 => InterruptHandler<I3C0>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut cfg = Config::default();
    cfg.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(cfg);

    defmt::info!("I3C controller <-> target IBI example");

    let i3c_cfg = controller::Config::default();
    let mut i3c = I3c::new_async_with_dma(p.I3C0, p.P1_9, p.P1_8, p.DMA0_CH0, p.DMA0_CH1, Irqs, i3c_cfg).unwrap();

    /* ---------------------------
     * Dynamic Address Assignment
     * --------------------------- */

    defmt::info!("DAA: reset");
    i3c.blocking_reset_daa().unwrap();

    Timer::after_micros(100).await;

    defmt::info!("DAA: begin");
    i3c.blocking_enter_daa().unwrap();

    let mut next_addr: u8 = 0x2a;

    loop {
        match i3c.blocking_next_daa_target().unwrap() {
            Some(target) => {
                defmt::info!(
                    "DAA: found target PID={:012x} BCR={:02x} DCR={:02x}",
                    target.pid,
                    target.bcr,
                    target.dcr
                );

                defmt::info!("DAA: assign address 0x{:02x}", next_addr);
                i3c.blocking_assign_daa(next_addr).unwrap();

                next_addr += 1;
            }
            None => {
                defmt::info!("DAA: enumeration complete");
                break;
            }
        }
    }

    i3c.blocking_end_daa().unwrap();

    /* ---------------------------
     * Normal bus operation
     * --------------------------- */

    Timer::after_micros(100).await;

    defmt::info!("ENEC: enable IBIs");
    //i3c.enable_ibis().unwrap(); // assumes your existing helper

    loop {
        defmt::info!("Waiting for IBI...");
        let mut ibi_payload = [0u8; 8];

        let (addr, payload_len) = i3c.async_wait_for_ibi(&mut ibi_payload).await.unwrap();

        defmt::info!("IBI from 0x{:02x}, payload_len={}", addr, payload_len);

        Timer::after_millis(100).await;
    }
}
