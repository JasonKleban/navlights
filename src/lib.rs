#![no_std]
mod config;
mod fusion;
mod math;
mod nav_hat_board;
mod nmea_line_reader;
mod peripherals;
mod perlin2d;
mod ws2812_rmt;

use bno055::{AxisRemap, BNO055AxisSign};
use esp_hal::clock::CpuClock;
use num_traits::Float;
use core::f32::consts::{FRAC_PI_6, FRAC_PI_8, PI};
use core::str::FromStr;
use esp_hal::{riscv, rmt};
use esp_hal::time::Instant;
use esp_println::{print, println};
use fusion::{Fusion};

use crate::math::{DEGREES_TO_RADIANS, KTS_PER_METER_SECOND, Quaternion, Vec3};
use crate::peripherals::orientation::{CALIBRATION_STATUS_ZERO, OSensorStatus};
use crate::nav_hat_board::NavHatBoard;

const PERLIN_GRADIENTS_HEIGHT : usize = 16;
const PERLIN_GRADIENTS_WIDTH : usize = 4;

const PIXELS: usize = 64 + 12;
const HALF_PIXELS: usize = PIXELS / 2;
const WATER_ANIMATION_DURATION: esp_hal::time::Duration = esp_hal::time::Duration::from_secs(10);

const BLACK: [u8; 3] = [0u8, 0u8, 0u8];
const WHITE: [u8; 3] = [255u8, 255u8, 255u8];
const RED: [u8; 3] = [255u8, 0u8, 0u8];
const GREEN: [u8; 3] = [0u8, 255u8, 0u8];
const BLUE: [u8; 3] = [0u8, 0u8, 255u8];
const ORANGE: [u8; 3] = [255u8, 80u8, 0u8];
const YELLOW: [u8; 3] = [255u8, 255u8, 0u8];

const STATUS_LIGHTS: [[u8; 3]; 4] = [RED, ORANGE, YELLOW, GREEN];

const BRIGHTNESS: f32 = 0.05;

const fn dull_color(rgb: [u8; 3]) -> [u8; 3] {
    [
        (BRIGHTNESS * rgb[0] as f32) as u8,
        (BRIGHTNESS * rgb[1] as f32) as u8,
        (BRIGHTNESS * rgb[2] as f32) as u8,
    ]
}

const WATER_PALETTE : [ [u8; 3] ; 11 ] = [
    dull_color([0, 0, 36]),
    dull_color([4, 2, 57]),
    dull_color([6, 5, 80]),
    dull_color([8, 7, 104]),
    dull_color([9, 9, 121]),
    dull_color([53, 58, 148]),
    dull_color([97, 107, 175]),
    dull_color([136, 151, 200]),
    dull_color([180, 201, 228]),
    dull_color([224, 250, 255]),
    dull_color([0, 0, 0]),
];

static mut NEOPIXEL_DRIVER: Option<ws2812_rmt::Ws2812<'static>> = None;

fn neopixel_driver() -> Option<&'static mut ws2812_rmt::Ws2812<'static>> {
    unsafe {
        let ptr = core::ptr::addr_of_mut!(NEOPIXEL_DRIVER);
        (*ptr).as_mut()
    }
}

pub fn program() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let rmt = rmt::Rmt::new(peripherals.RMT, esp_hal::time::Rate::from_mhz(80)).unwrap();
    let mut neopixels = ws2812_rmt::Ws2812::new(rmt.channel0, peripherals.GPIO3).unwrap();

    let mut pixel_buffer: [[u8; 3]; PIXELS] = [[0, 0, 0]; PIXELS];

    unsafe { NEOPIXEL_DRIVER = Some(neopixels) }

    neopixel_driver().unwrap().write(&pixel_buffer[7..71]);
    
    let mut nav_hat_board = NavHatBoard::new(
        peripherals.I2C0, 
        peripherals.UART1,
        peripherals.GPIO6, 
        peripherals.GPIO7, 
        peripherals.GPIO21, 
        peripherals.GPIO20
    ).unwrap();

    let mut delay = esp_hal::delay::Delay::new();
    let mut osensor_status = OSensorStatus::Uncalibrated(CALIBRATION_STATUS_ZERO);
    let mut nlr: nmea_line_reader::NmeaLineReader = nmea_line_reader::NmeaLineReader::new();
    let mut fusion = Fusion::new();
    let mut last_instant = Instant::now();
    let rng = esp_hal::rng::Rng::new();
    let perlin2d = perlin2d::Perlin2d::<PERLIN_GRADIENTS_HEIGHT, PERLIN_GRADIENTS_WIDTH>::new(move || rng.random());

    println!("Initializing BNO055 ...");
    nav_hat_board.setup_bno055(&mut delay).unwrap();
    println!("BNO055 initialized.");

    let axis_remap: AxisRemap = nav_hat_board.bno055.axis_remap().unwrap();
    let axis_sign: BNO055AxisSign = nav_hat_board.bno055.axis_sign().unwrap();

    println!("Checking for stored configuration ...");
    let mut app_config = match config::Config::load()
        .expect("Should have been able to try to load a stored config")
    {
        Some(config)
            if (
                config.axis_remap_sign[0],
                config.axis_remap_sign[1],
                config.axis_remap_sign[2],
                config.axis_remap_sign[3],
            ) == (
                axis_remap.x().bits(),
                axis_remap.y().bits(),
                axis_remap.z().bits(),
                axis_sign.bits(),
            ) =>
        {
            Some(config)
        }
        _ => None,
    };

    match app_config.as_ref() {
        Some(c) => {
            println!("Found stored configuration.");

            match c.bno_calibration_profile {
                Some(calib) => {
                    println!("bno055 calibration profile found");
                    nav_hat_board
                        .bno055
                        .set_calibration_profile(calib, &mut delay)
                        .expect(
                            "Should have been successful setting the bno055 calibration profile",
                        );

                    while nav_hat_board.bno055.get_calibration_status().unwrap().sys == 0 {
                        print!(".");
                        delay.delay_millis(100);
                    }

                    osensor_status = OSensorStatus::Ready;

                    println!("bno055 calibration profile loaded");
                }
                None => {}
            }
        }
        None => println!("No stored configuration found."),
    }

    println!("Beginning program loop ...");

    // fusion.set_debug_velocity_enu(Some(Vec3 { x: -3.0, y: 0.0, z: 0.0 }));
    // fusion.set_debug_acceleration_enu(Some(Vec3 { x: 0.1, y: 0.0, z: 0.0 }));

    loop {
        let mut current_orientation_yaw: Option<f32> = None;
        let mut current_bno055_sys_calibration_score: Option<u8> = None;

        let now = Instant::now();
        let dt = (now - last_instant).as_micros() as f32 * 1e-6;
        let fps = 1.0 / ((now - last_instant).as_micros() as f32 * 1e-6);
        last_instant = now;

        let water_frame = (now.duration_since_epoch().as_micros() % WATER_ANIMATION_DURATION.as_micros()) as f32 / WATER_ANIMATION_DURATION.as_micros() as f32;

        fusion.update_time(dt);

        if let OSensorStatus::Uncalibrated(calibration_status) = osensor_status {
            if nav_hat_board.bno055.is_fully_calibrated().unwrap() {
                let calibration_profile = nav_hat_board
                    .bno055
                    .calibration_profile(&mut delay)
                    .unwrap();

                let needs_store = app_config
                    .as_ref()
                    .and_then(|cfg| cfg.bno_calibration_profile.as_ref())
                    .map_or(true, |stored| stored != &calibration_profile);

                if needs_store {
                    println!("Storing bno055 calibration profile ...");

                    let new_config = config::Config {
                        bno_calibration_profile: Some(calibration_profile),
                        axis_remap_sign: [
                            axis_remap.x().bits(),
                            axis_remap.y().bits(),
                            axis_remap.z().bits(),
                            axis_sign.bits(),
                        ],
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

                    println!(
                        "BNO055 calibration status: {{ sys: {}, gyr: {}, acc: {}, mag: {} }}",
                        next_status.sys, next_status.gyr, next_status.acc, next_status.mag
                    );
                };

                for i in 0..PIXELS {
                    match i {
                        i if i == 7 => {
                            pixel_buffer[i] = BLUE;
                        }
                        i if i == 9 => {
                            pixel_buffer[i] = STATUS_LIGHTS[next_status.sys as usize];
                        }
                        i if i == 11 => {
                            pixel_buffer[i] = STATUS_LIGHTS[next_status.gyr as usize];
                        }
                        i if i == 13 => {
                            pixel_buffer[i] = STATUS_LIGHTS[next_status.acc as usize];
                        }
                        i if i == 15 => {
                            pixel_buffer[i] = STATUS_LIGHTS[next_status.mag as usize];
                        }
                        i if i == 70 => {
                            pixel_buffer[i] = RED;
                        }
                        _ => {
                            pixel_buffer[i] = WATER_PALETTE[perlin2d.value(water_frame, i as f32 / PIXELS as f32).ceil() as usize];
                        }
                    }
                }

                neopixel_driver().unwrap().write(&pixel_buffer[7..71]);
            }
        } else {
            let gravity = nav_hat_board.bno055.gravity().unwrap();
            let linear_acceleration = nav_hat_board.bno055.linear_acceleration().unwrap();
            let quaternion = nav_hat_board.bno055.quaternion().unwrap();
            let temperature = nav_hat_board.bno055.temperature().unwrap();

            // convert to craft-frame swaping x and y and negating z
            let q = Quaternion {
                w: quaternion.s,
                x: quaternion.v.y,
                y: quaternion.v.x,
                z: -quaternion.v.z,
            }
            .normalize();

            let orientation = math::quaternion_to_euler(&q);

            // println!("BNO055 readings:");
            // println!(
            //     "{:>+06.1}° yaw, {:>+06.1}° pitch, {:>+06.1}° roll, xyzw< {:.8}, {:.8}, {:.8}, {:.8} >",
            //     orientation.yaw * RADIANS_TO_DEGREES, 
            //     orientation.pitch * RADIANS_TO_DEGREES,
            //     orientation.roll * RADIANS_TO_DEGREES,
            //     q.x, q.y, q.z, q.w
            // );

            //println!("< {:.8}, {:.8}, {:.8}, {:.8} >", q.x, q.y, q.z, q.w);

            // println!(
            //     "  gravity: <{:>+6.2} m/s², {:>+6.2} m/s², {:>+6.2} m/s²>",
            //     gravity.x, gravity.y, gravity.z
            // );
            // println!(
            //     "  linear_acceleration: <{:>+6.2} m/s², {:>+6.2} m/s², {:>+6.2} m/s²>",
            //     linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
            // );
            //println!("  mag_data: <{:>+6.2}, {:>+6.2}, {:>+6.2}>", mag_data.x, mag_data.y, mag_data.z);
            //println!("  quaternion: <{:>+6.2}, {:>+6.2}, {:>+6.2}, {:>+6.2}>", quaternion.v.x, quaternion.v.y, quaternion.v.z, quaternion.s);
            //println!("  temperature: {:.1}°C", temperature);

            let cal = nav_hat_board.bno055.get_calibration_status().unwrap();

            let acc = Vec3 {
                x: linear_acceleration.x,
                y: linear_acceleration.y,
                z: linear_acceleration.z,
            };

            fusion.update_imu(q, acc, dt, cal.sys);

            current_orientation_yaw = Some(orientation.yaw);
            current_bno055_sys_calibration_score = Some(cal.sys);

            let sog = fusion.speed();
            let rel_dir = fusion.body_relative_direction(q);
            let rel_north = fusion.body_relative_north(q);
            let yaw = fusion.yaw();

            let relative_dir_pixel = ((((rel_dir as f32 / (2.0 * PI)) * PIXELS as f32)
                + PIXELS as f32 * 1.5) as usize)
                % PIXELS;

            let relative_north_pixel = ((((rel_north as f32 / (2.0 * PI)) * PIXELS as f32)
                + PIXELS as f32 * 1.5) as usize)
                % PIXELS;

            println!(
                "Fused: {:>6.2} m/s, rel_dir {:>+6.1}° (pixel {}), rel_north {:>+6.1}° (pixel {}), {:>+06.1}° yaw, {:>4.0} fps",
                sog,
                rel_dir.to_degrees(),
                relative_dir_pixel,
                rel_north.to_degrees(),
                relative_north_pixel,
                yaw.unwrap_or(0.0),
                fps
            );

            // 2.0 m/s ~= 4kts
            let nav_width_ratio = (sog / 2.0).clamp(0.0, 1.0);

            for i in 0..PIXELS {
                let i_rad = (((i + HALF_PIXELS) % PIXELS) as f32 / PIXELS as f32) * 2.0 * PI;

                let [starboard, _, port] = radial_profile(i_rad, 0.0, 1.0, rel_dir, nav_width_ratio * FRAC_PI_6, 10);
                let [_, full_aft, _] = radial_profile(i_rad + PI, 0.0, 1.0, rel_dir, nav_width_ratio * FRAC_PI_8, 10);

                match i {
                    i if 0.1 < starboard => {
                        pixel_buffer[i] = GREEN;
                    }
                    i if 0.1 < port => {
                        pixel_buffer[i] = RED;
                    }
                    i if 0.1 < full_aft => {
                        pixel_buffer[i] = WHITE;
                    }
                    i if i == relative_north_pixel => {
                        pixel_buffer[i] = dull_color(YELLOW);
                    }
                    _ => {
                        pixel_buffer[i] = WATER_PALETTE[perlin2d.value(water_frame, i as f32 / PIXELS as f32).ceil() as usize];
                    }
                }
            }

            neopixel_driver().unwrap().write(&pixel_buffer[7..71]);
        }

        while let Ok(Some(byte)) = nav_hat_board.read_uart_byte() {
            if let Some(sentence) = nlr.push_byte(byte) {
                match sentence {
                    s if s.starts_with("$GPRMC,") => {
                        let mut spliterator = sentence.trim_ascii_end().split(",").skip(1);

                        if let (
                            Some(time),
                            Some("A"),
                            Some(lat),
                            Some(north_south),
                            Some(lng),
                            Some(east_west),
                            Some(speed_kts),
                            track_true_degrees,
                            Some(date),
                            mag_offset_degrees,
                            mag_offset_east_west,
                        ) = (
                            spliterator.next(),
                            spliterator.next(),
                            spliterator.next().and_then(|lat| f32::from_str(lat).ok()),
                            spliterator.next(),
                            spliterator.next().and_then(|lng| f32::from_str(lng).ok()),
                            spliterator.next(),
                            spliterator
                                .next()
                                .and_then(|speed_kts| f32::from_str(speed_kts).ok()),
                            spliterator.next().and_then(|track_true_degrees| {
                                f32::from_str(track_true_degrees).ok()
                            }),
                            spliterator.next(),
                            spliterator.next().and_then(|mag_offset_degrees| {
                                f32::from_str(mag_offset_degrees).ok()
                            }),
                            spliterator.next(),
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

                                if let (Some(bno055_sys_calibration_score), Some(orientation_yaw)) = (
                                    current_bno055_sys_calibration_score,
                                    current_orientation_yaw,
                                ) {
                                    if 2 <= bno055_sys_calibration_score && 1.5 < speed_mps {
                                        fusion.correct_yaw_from_gps(orientation_yaw, track_rad);
                                    }
                                    // if let (Some(degrees), Some(east_west)) = (mag_offset_degrees, mag_offset_east_west) {
                                    //     println!(" (off {:>+06.1}° {})",
                                    //         degrees,
                                }
                            } else {
                                // println!();
                            }
                        } else {
                            // println!(" 🛰 : {}", sentence.trim_ascii_end());
                        }
                    }
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
                    _ => (),
                }
            }
        }

        //delay.delay_millis(250);
    }
}

pub fn panic_entry(info: &core::panic::PanicInfo) -> ! {
    riscv::interrupt::disable();

    println!();
    println!("====================");
    println!("CRASHED!");
    println!("{}", info);
    println!("====================");

    let pixel_buffer = [ dull_color(RED) ; PIXELS];

    if let Some(neopixels) = neopixel_driver() {
        let _ = neopixels.write(&pixel_buffer[7..71]);
    }

    loop {
        core::hint::spin_loop();
    }
}

//
// r\ =\ r_{i}+\frac{\left(r_{o}-r_{i}\right)}{1+\left(\frac{\arctan\left(\sin\left(\theta-\frac{\pi}{2}+T_{peak}\right),\cos\left(\theta-\frac{\pi}{2}+T_{peak}\right)\right)}{T_{width}}\right)^{2n}}
//
fn radial_profile(
    theta: f32,
    r_i: f32,
    r_o: f32,
    t_peak: f32,
    t_width: f32,
    n: u32,
) -> [ f32 ; 3 ] {
    let t_width_half = t_width / 2.0;

    let ccw = theta - t_peak - t_width_half;
    let full = theta - t_peak;
    let cw = theta - t_peak + t_width_half;

    // principal angle wrap (-π..π]
    let angle_ccw = libm::atan2f(libm::sinf(ccw), libm::cosf(ccw));
    let angle_full = libm::atan2f(libm::sinf(full), libm::cosf(full));
    let angle_cw = libm::atan2f(libm::sinf(cw), libm::cosf(cw));

    let ratio_ccw = angle_ccw / t_width_half;
    let ratio_full = angle_full / t_width;
    let ratio_cw = angle_cw / t_width_half;

    let exponent = 2 * n;
    let shaped_ccw = libm::powf(ratio_ccw, exponent as f32);
    let shaped_full = libm::powf(ratio_full, exponent as f32);
    let shaped_cw = libm::powf(ratio_cw, exponent as f32);

    [
        r_i + (r_o - r_i) / (1.0 + shaped_ccw),
        r_i + (r_o - r_i) / (1.0 + shaped_full),
        r_i + (r_o - r_i) / (1.0 + shaped_cw)
    ]
}

fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        libm::powf((c + 0.055) / 1.055, 2.4)
    }
}

fn linear_to_srgb(c: f32) -> f32 {
    if c <= 0.0031308 {
        12.92 * c
    } else {
        1.055 * libm::powf(c, 1.0 / 2.4) - 0.055
    }
}

fn gradient_rgb(
    a: [f32;3],
    b: [f32;3],
    t: f32
) -> [f32;3] {

    let mut out = [0.0;3];

    let t = t * t * (3.0 - 2.0 * t);

    for i in 0..3 {
        let la = srgb_to_linear(a[i]);
        let lb = srgb_to_linear(b[i]);

        let l = la + t * (lb - la);

        out[i] = linear_to_srgb(l);
    }

    out
}