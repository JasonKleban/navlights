#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc, spi, IO, Delay};

use smart_leds::{RGB, RGB8, colors::{ORANGE, BLUE}, SmartLedsWrite};
use ws2812_spi::Ws2812;

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
        
    let sclk = io.pins.gpio12;
    let miso = io.pins.gpio11;
    let mosi = io.pins.gpio13;
    let cs = io.pins.gpio10;

    let spi = hal::spi::Spi::new(
        peripherals.SPI2, 
        sclk,
        mosi,
        miso,
        cs,
        100u32.kHz(),
        spi::SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let mut ws = Ws2812::new(spi);

    const NUM_LEDS: usize = 10;
    const DELAY_SECONDS: u32 = 30;

    loop {
        println!("Hello world!");

        let first_rgb: RGB8 = ORANGE;
        let second_rgb: RGB8 = BLUE;

        let mut data: [RGB<u8>; 10] = [first_rgb; NUM_LEDS];

        ws.write(data.iter().cloned()).unwrap();

        for i in 0..NUM_LEDS {
            delay.delay_ms(DELAY_SECONDS);
            data[i] = second_rgb;
            ws.write(data.iter().cloned()).unwrap();
        }
        for i in 0..NUM_LEDS {
            delay.delay_ms(DELAY_SECONDS);
            data[i] = first_rgb;
            ws.write(data.iter().cloned()).unwrap();
        }
    }
}
