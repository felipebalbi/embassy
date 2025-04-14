#![no_main]
#![no_std]

use defmt::{debug, error, info};
use embassy_espi::driver::{Driver, Event, oob::OobChannel, vwire::VWireChannel};
use embassy_executor::Spawner;
use embassy_microchip::espi::{
    BM1InterruptHandler, BM2InterruptHandler, Config, Espi, FlashInterruptHandler, GIRQ24InterruptHandler,
    GIRQ25InterruptHandler, LtrInterruptHandler, OobDownInterruptHandler, OobUpInterruptHandler, PCInterruptHandler,
    ResetInterruptHandler, VWConfig, VWireInterruptHandler,
};
use embassy_microchip::{bind_interrupts, peripherals};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        INTR_PC => PCInterruptHandler<peripherals::ESPI>;
        INTR_BM1 => BM1InterruptHandler<peripherals::ESPI>;
        INTR_BM2 => BM2InterruptHandler<peripherals::ESPI>;
        INTR_LTR => LtrInterruptHandler<peripherals::ESPI>;
        INTR_OOB_UP => OobUpInterruptHandler<peripherals::ESPI>;
        INTR_OOB_DOWN => OobDownInterruptHandler<peripherals::ESPI>;
        ESPI_VWIRE => VWireInterruptHandler<peripherals::ESPI>;
        ESPI_RESET => ResetInterruptHandler<peripherals::ESPI>;
        INTR_FLASH => FlashInterruptHandler<peripherals::ESPI>;
        GIRQ24 => GIRQ24InterruptHandler<peripherals::ESPI>;
        GIRQ25 => GIRQ25InterruptHandler<peripherals::ESPI>;
    }
);

#[embassy_executor::task]
async fn poll_regs() {
    let io = embassy_microchip::pac::ESPI_IO;

    loop {
        //         debug!(
        //             "
        // GLB CAP0: {}
        // GLB CAP1: {}
        // OOB TX CTRL {}
        // OOB TX STS {}
        // OOB RX CTRL {}
        // OOB RX STS {}
        // OOB RX LEN {}
        // ",
        //             io.glb_cap0().read(),
        //             io.glb_cap1().read(),
        //             io.oob_tx_ctrl().read(),
        //             io.oob_tx_sts().read(),
        //             io.oob_rx_ctrl().read(),
        //             io.oob_rx_sts().read(),
        //             io.oob_rx_len().read(),
        //         );
        embassy_time::Timer::after_secs(100).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        let cp = cortex_m::Peripherals::steal();
        cp.SCB.vtor.write(0x000c_0000);
    }

    let p = embassy_microchip::init(Default::default());
    let mut config = Config::default();
    config.peripheral = Some(());
    config.vwire = Some(VWConfig { max_count: 11 });
    config.oob = Some(());
    let mut espi = Espi::new_single_with_alert(
        p.ESPI, p.GPIO70, p.GPIO71, p.GPIO72, p.GPIO73, p.GPIO66, p.GPIO61, p.GPIO65, p.GPIO63, Irqs, config,
    );

    info!("Hello, world!");

    spawner.spawn(poll_regs().unwrap());

    loop {
        info!("LISTENING");
        let result = espi.listen().await;
        match result {
            Ok(event) => {
                debug!("Event: {:?}", event);

                match event {
                    Event::Reset => {
                        debug!("Reset event");
                    }

                    Event::VWire => {
                        debug!("VWire event");

                        let result = espi.put_vwire(2);
                        // info!("VWIRE {:?}", result);
                        let result = espi.put_vwire(3);
                        // info!("VWIRE {:?}", result);
                        let result = espi.put_vwire(7);
                        // info!("VWIRE {:?}", result);
                    }

                    Event::Oob => {
                        let mut rxbuf = [0_u8; 256];
                        let txbuf = [0x55_u8; 256];

                        let result = espi.oob_receive(&mut rxbuf).await;
                        info!("OOB RECEIVE {:?}", result);
                        let result = espi.oob_send(&txbuf).await;
                        info!("OOB SEND {:?}", result);
                    }

                    _ => {}
                }
            }
            Err(_) => {
                error!("Failed to listen");
            }
        }
    }
}
