#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use navlights_rs as lib;
use esp_println::println;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::time::{Duration, Instant};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const DEFAULT_APP_CONFIG : lib::config::Config = lib::config::Config{ true_offset_deg: 0.0 };

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.2.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let _peripherals = esp_hal::init(config);

    let mut app_config = 
        match lib::config::load() {
            None => {
                println!("No valid config found");

                DEFAULT_APP_CONFIG
            },
            Some(app_config) => {
                println!("Valid config found {}", app_config.true_offset_deg);

                app_config
            }
        };

    app_config.true_offset_deg += 3.1415;

    lib::config::store(&app_config);

    println!("Wrote config");

    loop {
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}

        println!("Hello, world from the ESP32 no-std environment!");
    }
}