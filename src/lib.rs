#![no_std]
mod nav_hat_board;
mod peripherals;
mod config;
mod math;
mod nmea_line_reader;
mod fusion;
mod ws2812_rmt;

use esp_println::{print, println};
use esp_hal::time::Instant;
use core::str::FromStr;
use fusion::{Fusion, Vec3, Quaternion};

use crate::math::{DEGREES_TO_RADIANS, KTS_PER_METER_SECOND, RADIANS_TO_DEGREES};
use crate::peripherals::orientation::{ OSensorStatus, CALIBRATION_STATUS_ZERO };

use crate::nav_hat_board::NavHatBoard;

const PIXELS: usize = 64;
const ARC_START: f32 = -2.094395; // -120 deg
const ARC_END: f32 =  2.094395;   // +120 deg
const ARC_SPAN: f32 = ARC_END - ARC_START;

pub fn program () -> ! {
    let mut delay = esp_hal::delay::Delay::new();
    let mut nav_hat_board = NavHatBoard::new().unwrap();
    let mut osensor_status = OSensorStatus::Uncalibrated(CALIBRATION_STATUS_ZERO);
    let mut nlr: nmea_line_reader::NmeaLineReader = nmea_line_reader::NmeaLineReader::new();
    let mut fusion = Fusion::new();
    let mut last_instant = Instant::now();
    let mut data : [[u8; 3]; 64] = [[ 0, 0, 0]; 64];

    println!("Initializing BNO055 ...");
    nav_hat_board.setup_bno055(&mut delay).unwrap();
    println!("BNO055 initialized.");

    println!("Checking for stored configuration ...");
    let mut app_config = config::Config::load().expect("Should have been able to try to load a stored config");
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
        let now = Instant::now();
        let dt = (now - last_instant).as_micros() as f32 * 1e-6;
        let mut current_orientation_yaw : Option<f32> = None;
        let mut current_bno055_sys_calibration_score : Option<u8> = None;
        last_instant = now;

        fusion.update_time(dt);

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
            
            //println!("BNO055 readings:");
            println!("  orientation: {:>+06.1}° roll, {:>+06.1}° pitch, {:>+06.1}° yaw", 
                orientation.roll * RADIANS_TO_DEGREES, 
                orientation.pitch * RADIANS_TO_DEGREES, 
                orientation.yaw * RADIANS_TO_DEGREES + 180.0);
            println!("  gravity: <{:>+6.2} m/s², {:>+6.2} m/s², {:>+6.2} m/s²>", gravity.x, gravity.y, gravity.z);
            println!("  linear_acceleration: <{:>+6.2} m/s², {:>+6.2} m/s², {:>+6.2} m/s²>", linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
            //println!("  mag_data: <{:>+6.2}, {:>+6.2}, {:>+6.2}>", mag_data.x, mag_data.y, mag_data.z);
            //println!("  quaternion: <{:>+6.2}, {:>+6.2}, {:>+6.2}, {:>+6.2}>", quaternion.v.x, quaternion.v.y, quaternion.v.z, quaternion.s);
            println!("  temperature: {:.1}°C", temperature);

            let cal = nav_hat_board.bno055.get_calibration_status().unwrap();

            let q = Quaternion {
                w: quaternion.s,
                x: quaternion.v.x,
                y: quaternion.v.y,
                z: quaternion.v.z,
            };

            let acc = Vec3 {
                x: linear_acceleration.x,
                y: linear_acceleration.y,
                z: linear_acceleration.z,
            };

            fusion.update_imu(
                q,
                acc,
                dt,
                cal.sys
            );

            current_orientation_yaw = Some(orientation.yaw);
            current_bno055_sys_calibration_score = Some(cal.sys);

            let sog = fusion.speed_over_ground();
            let rel_dir = fusion.body_relative_direction(q);
            let yaw = fusion.yaw();
            let _velocity_ned = fusion.velocity_ned();

            println!(
                "Fused: {:>6.2} m/s, rel_dir {:>+6.1}°, {:>+06.1}° yaw",
                sog,
                rel_dir.to_degrees(),
                yaw * RADIANS_TO_DEGREES + 180.0
            );

            let mut theta = rel_dir;

            // Clamp to arc
            if theta < ARC_START { theta = ARC_START; }
            if theta > ARC_END { theta = ARC_END; }

            let normalized = (theta - ARC_START) / ARC_SPAN;

            let pixel = (normalized * (PIXELS as f32 - 1.0)) as usize;

            if true {
                data[pixel] = [ 0u8, 50u8, 0u8 ];

                for i in 0..64 {
                    let dist = (i as i32 - pixel as i32).abs();
                    if dist < 4 {
                        let intensity = 50 / (dist as u8 + 1);
                        data[i] = [ 0u8, intensity, 0u8 ];
                    } else {
                        data[i] = [ 0u8, 0u8, 0u8 ];
                    }
                }
            }

            // for i in 0..64 {
            //     data[i] = [ 0u8, 150u8, 0u8 ];
            // }

            nav_hat_board.neopixels.write(&data);
        }

        while let Ok(Some(byte)) = nav_hat_board.read_uart_byte() {
            if let Some(sentence) = nlr.push_byte(byte) {
                match sentence {
                    s if s.starts_with("$GPRMC,") => {
                        let mut spliterator = sentence.trim_ascii_end().split(",").skip(1);

                        if let (Some(time), Some("A"), Some(lat), Some(north_south), Some(lng), Some(east_west), 
                            Some(speed_kts), track_true_degrees, Some(date), mag_offset_degrees, mag_offset_east_west) = (
                            spliterator.next(),
                            spliterator.next(),
                            spliterator.next().and_then(|lat| f32::from_str(lat).ok()),
                            spliterator.next(),
                            spliterator.next().and_then(|lng| f32::from_str(lng).ok()),
                            spliterator.next(),
                            spliterator.next().and_then(|speed_kts| f32::from_str(speed_kts).ok()),
                            spliterator.next().and_then(|track_true_degrees| f32::from_str(track_true_degrees).ok()),
                            spliterator.next(),
                            spliterator.next().and_then(|mag_offset_degrees| f32::from_str(mag_offset_degrees).ok()),
                            spliterator.next()
                        ) {
                            // print!(" 🌎 20{}-{}-{} {}:{}:{} UTC: {:>8.4}° {}, {:>8.4}° {}", 
                            //     &date[4..], &date[2..4], &date[..2], &time[..2], &time[2..4], &time[4..6],
                            //     lat / 100.0, north_south,
                            //     lng / 100.0, east_west
                            // );

                            if let Some(degrees) = track_true_degrees {
                                // print!(" tracking {:>+06.1}° at {:>8.4}kts ({:>8.4} m/s)", 
                                //     degrees, 
                                //     speed_kts, 
                                //     speed_kts * KTS_PER_METER_SECOND);

                                // if let (Some(degrees), Some(east_west)) = (mag_offset_degrees, mag_offset_east_west) {
                                //     println!(" (off {:>+06.1}° {})", 
                                //         degrees,
                                //         east_west);
                                // }

                                let speed_mps = speed_kts * KTS_PER_METER_SECOND;
                                let track_rad = degrees.to_radians();

                                fusion.update_gps(speed_mps, track_rad);

                                if let Some(degrees) = mag_offset_degrees {
                                    fusion.update_declination(degrees * DEGREES_TO_RADIANS);
                                }

                                if let (Some(bno055_sys_calibration_score), Some(orientation_yaw)) = 
                                    (current_bno055_sys_calibration_score, current_orientation_yaw) {
                                    if 2 <= bno055_sys_calibration_score && 1.5 < speed_mps {
                                        fusion.correct_yaw_from_gps(
                                            orientation_yaw,
                                            track_rad);
                                    }
                                }
                            } else {
                                // println!();
                            }
                        } else {
                            println!(" 🛰 : {}", sentence.trim_ascii_end());
                        }
                    },
                    // s if s.starts_with("$GPGGA,") => {
                    //     let mut spliterator = sentence.trim_ascii_end().split(",").skip(9);

                    //     if let (Some(alt), Some(au)) = (
                    //         spliterator.next().and_then(|alt| f32::from_str(alt).ok()),
                    //         spliterator.next()
                    //     ) {
                    //         println!(" 🌎         {:>8.4} {}", alt, au);
                    //     } else {
                    //         println!(" 🛰 : {}", sentence.trim_ascii_end());
                    //     }
                    // },
                    _ => ()
                }
            }
        }

        delay.delay_millis(100);
    }
}