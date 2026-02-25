#![no_std]
use esp_println::println;
use esp_hal::clock::CpuClock;
use esp_hal::time::{Rate};
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::time::Instant;
use bno055::{Bno055, mint};

mod config;

static CALIBRATION_STATUS_ZERO : bno055::BNO055CalibrationStatus = bno055::BNO055CalibrationStatus{ sys: 0, gyr: 0, acc: 0, mag: 0 };

enum OSensorStatus {
    Uncalibrated(bno055::BNO055CalibrationStatus),
    Ready,
    Readings {
        euler_angles: mint::EulerAngles<f32, ()>,
        gravity: mint::Vector3<f32>,
        accel_data: mint::Vector3<f32>,
        mag_data: mint::Vector3<f32>,
        quaternion: mint::Quaternion<f32>,
        temperature: i8,
    }
}

struct NavHatBoard<'d> {
    bno055: Bno055<I2c<'d, esp_hal::Blocking>>,
}

impl NavHatBoard<'_> {
    pub fn new() -> Self {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);
        let i2c0: I2c<'_, esp_hal::Blocking> = I2c::new(
            peripherals.I2C0, 
            Config::default().with_frequency(Rate::from_khz(400))).unwrap()
                .with_sda(peripherals.GPIO6)
                .with_scl(peripherals.GPIO7);
        let bno055 = Bno055::new(i2c0);

        return Self {
            bno055
        };
    }
}

pub fn program () -> ! {
    let mut delay = esp_hal::delay::Delay::new();
    let mut nav_hat_board = NavHatBoard::new();
    let mut osensor_status = OSensorStatus::Uncalibrated(CALIBRATION_STATUS_ZERO);

    println!("Initializing BNO055 ...");
    nav_hat_board.bno055.init(&mut delay).unwrap();
    nav_hat_board.bno055.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).unwrap();
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

                    osensor_status = if nav_hat_board.bno055.is_fully_calibrated().unwrap() { 
                        OSensorStatus::Ready
                    } else {
                        OSensorStatus::Uncalibrated(nav_hat_board.bno055.get_calibration_status().unwrap())
                    };

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
            osensor_status = OSensorStatus::Readings {
                euler_angles: nav_hat_board.bno055.euler_angles().unwrap(),
                gravity: nav_hat_board.bno055.gravity().unwrap(),
                accel_data: nav_hat_board.bno055.accel_data().unwrap(),
                mag_data: nav_hat_board.bno055.mag_data().unwrap(),
                quaternion: nav_hat_board.bno055.quaternion().unwrap(),
                temperature: nav_hat_board.bno055.temperature().unwrap()
            };

            if let OSensorStatus::Readings { euler_angles, gravity, accel_data, mag_data, quaternion, temperature } = osensor_status {
                println!("BNO055 readings: \n\teuler_angles: <{:.2}, {:.2}, {:.2}>, \n\tgravity: <{:.2}, {:.2}, {:.2}>, \n\taccel_data: <{:.2}, {:.2}, {:.2}>, \n\tmag_data: <{:.2}, {:.2}, {:.2}>, \n\tquaternion: <{:.2}, {:.2}, {:.2}, {:.2}>, \n\ttemperature: {}°C", 
                    euler_angles.a, euler_angles.b, euler_angles.c, 
                    gravity.x, gravity.y, gravity.z, 
                    accel_data.x, accel_data.y, accel_data.z,
                    mag_data.x, mag_data.y, mag_data.z,
                    quaternion.v.x, quaternion.v.y, quaternion.v.z, quaternion.s,
                    temperature
                );
            }
        }

        delay.delay_millis(500);
    }
}