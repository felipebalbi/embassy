#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::bind_interrupts;
use hal::clocks::config::Div8;
use hal::config::Config;
use hal::i2c::Dma;
use hal::i2c::controller::{I2c, InterruptHandler};
use hal::i3c::target::{self, I3c};
use hal::peripherals::LPI2C1;
use {defmt_rtt as _, embassy_mcxa as hal, panic_probe as _};

bind_interrupts!(
    struct Irqs {
        LPI2C1 => InterruptHandler<LPI2C1>;
    }
);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.clock_cfg.sirc.fro_lf_div = Div8::from_divisor(1);

    let p = hal::init(config);

    defmt::info!("I3C target example");

    let mut config = target::Config::default();
    config.address = 0x2a;
    let mut target = I3c::new_blocking(p.I3C0, p.P1_9, p.P1_8, config).unwrap();

    let i2c =
        I2c::new_async_with_dma(p.LPI2C1, p.P1_1, p.P1_0, p.DMA_CH0, p.DMA_CH1, Irqs, Default::default()).unwrap();

    spawner.spawn(i2c_task(i2c).unwrap());
    Timer::after_secs(1).await;

    loop {
        let request = target.blocking_listen().unwrap();
        defmt::info!("{:?}", request);
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn i2c_task(mut i2c: I2c<'static, Dma<'static>>) {
    defmt::error!("I2C task");
    loop {
        let result = i2c.async_write(0x2a, &[0, 1, 2, 3]).await;
        defmt::error!("I2C: Result -> {:?}", result);
        Timer::after_secs(1).await;
    }
}
