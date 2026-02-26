#![no_std]
mod nav_hat_board;
mod peripherals;
mod config;
mod math;
mod nmea_line_reader;

use esp_println::{print, println};
use esp_hal::time::Instant;

use crate::peripherals::orientation::{ OSensorStatus, CALIBRATION_STATUS_ZERO };

use crate::nav_hat_board::NavHatBoard;

pub fn program () -> ! {
    let mut delay = esp_hal::delay::Delay::new();
    let mut nav_hat_board = NavHatBoard::new().unwrap();
    let mut osensor_status = OSensorStatus::Uncalibrated(CALIBRATION_STATUS_ZERO);
    let mut nlr: nmea_line_reader::NmeaLineReader = nmea_line_reader::NmeaLineReader::new();

    println!("Initializing BNO055 ...");
    nav_hat_board.setup_bno055(&mut delay).unwrap();
    println!("BNO055 initialized.");

    println!("Checking for stored configuration ...");
    let mut app_config = config::Config::load();
    match app_config.as_ref() {
        Some(c) => {
            println!("Found stored configuration.");

            match c.bno_calibration_profile {
                Some(calib) => {
                    println!("bno055 calibration profile found");
                    nav_hat_board.bno055.set_calibration_profile(calib, &mut delay)
                        .expect("Should have been successful setting the bno055 calibration profile");

                    while nav_hat_board.bno055.get_calibration_status().unwrap().sys == 0 {
                        print!(".");
                        delay.delay_millis(100);
                    }

                    osensor_status = OSensorStatus::Ready;

                    println!("bno055 calibration profile loaded");
                },
                None => {}
            }
        },
        None => println!("No stored configuration found."),
    }

    println!("Beginning program loop ...");

    loop {
        let _ = Instant::now();

        if let OSensorStatus::Uncalibrated(calibration_status) = osensor_status {
            if nav_hat_board.bno055.is_fully_calibrated().unwrap() {
                let calibration_profile = 
                    nav_hat_board.bno055.calibration_profile(&mut delay).unwrap();

                let needs_store = app_config
                    .as_ref()
                    .and_then(|cfg| cfg.bno_calibration_profile.as_ref())
                    .map_or(true, |stored| stored != &calibration_profile);

                if needs_store {
                    println!("Storing bno055 calibration profile ...");

                    let new_config = config::Config {
                        bno_calibration_profile: Some(calibration_profile),
                        ..app_config.unwrap_or_default()
                    };

                    config::Config::store(&new_config).unwrap();

                    app_config = Some(new_config);
                }

                osensor_status = OSensorStatus::Ready;
            } else {
                let next_status = nav_hat_board.bno055.get_calibration_status().unwrap();
                
                if calibration_status != next_status {
                    
                    osensor_status = OSensorStatus::Uncalibrated(next_status);

                    println!("BNO055 calibration status: {{ sys: {}, gyr: {}, acc: {}, mag: {} }}", 
                        next_status.sys, 
                        next_status.gyr, 
                        next_status.acc, 
                        next_status.mag);
                };
            }
        } else {
            let gravity = nav_hat_board.bno055.gravity().unwrap();
            let linear_acceleration = nav_hat_board.bno055.linear_acceleration().unwrap();
            let quaternion = nav_hat_board.bno055.quaternion().unwrap();
            let temperature = nav_hat_board.bno055.temperature().unwrap();
            let orientation = math::quaternion_to_euler(&quaternion);

            // Positive values indicate right wing down (roll), nose up (pitch), and nose right (yaw).
            
            println!("BNO055 readings:");
            println!("  orientation: {:>+06.1}° roll, {:>+06.1}° pitch, {:>+06.1}° yaw", orientation.roll, orientation.pitch, orientation.yaw);
            println!("  gravity: <{:>+6.2} m/s², {:>+6.2} m/s², {:>+6.2} m/s²>", gravity.x, gravity.y, gravity.z);
            println!("  linear_acceleration: <{:>+6.2} m/s², {:>+6.2} m/s², {:>+6.2} m/s²>", linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
            //println!("  mag_data: <{:>+6.2}, {:>+6.2}, {:>+6.2}>", mag_data.x, mag_data.y, mag_data.z);
            //println!("  quaternion: <{:>+6.2}, {:>+6.2}, {:>+6.2}, {:>+6.2}>", quaternion.v.x, quaternion.v.y, quaternion.v.z, quaternion.s);
            println!("  temperature: {:.1}°C", temperature);
        }

        while let Ok(Some(byte)) = nav_hat_board.read_uart_byte() {
            if let Some(sentence) = nlr.push_byte(byte) {
                println!(" 🌎 {}", sentence.trim_ascii_end());
            }
        }

        delay.delay_millis(100);
    }
}