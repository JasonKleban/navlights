#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl, i2c, peripherals::Peripherals, prelude::*, spi, timer::TimerGroup, Delay,
    Rtc, Uart, IO,
};

use smart_leds::{
    colors::{BLUE, ORANGE, WHITE},
    SmartLedsWrite, RGB, RGB8,
};
use ws2812_spi::Ws2812;
use bno055;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay: Delay = Delay::new(&clocks);

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let sda = io.pins.gpio6;
    let scl = io.pins.gpio7;

    let i2c0 = i2c::I2C::new(
        peripherals.I2C0,
        sda,
        scl,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut bno055 = bno055::Bno055::new(i2c0);

    // GPS Data
    let uart1 = Uart::new(peripherals.UART1, &mut system.peripheral_clock_control);

    // Led strip data
    let mosi = io.pins.gpio10;

    let spi = hal::spi::Spi::new_mosi_only(
        peripherals.SPI2,
        mosi,
        3_333_333u32.Hz(),
        spi::SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut ws = Ws2812::new(spi);

    // loop {
    //     match bno055.is_fully_calibrated() {
    //         Ok(true) => println!("orientation sensor is fully calibrated"),
    //         Ok(false) => println!("orientation sensor is not fully calibrated"),
    //         Err(e) => println!("error with orientation sensor: {e:?}"),
    //     }

    //     // Recieve GPS Message Value
    //     let mut buf = [0_u8; 1];
    //     uart1.read(&mut buf, BLOCK).unwrap();
    // }

    const NUM_LEDS: usize = 1;
    const DELAY_MS : u32 = 5 * 1000;

    let data = [WHITE; NUM_LEDS];

    loop {
        ws.write(data.iter().cloned()).unwrap();
        println!("Wrote");
        delay.delay_ms(DELAY_MS);
    }
}
