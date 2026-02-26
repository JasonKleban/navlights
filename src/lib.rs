#![no_std]
use esp_println::println;
use esp_hal::clock::CpuClock;
use esp_hal::time::{Rate};
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::time::Instant;
use bno055::{Bno055, mint};
use bno055::{AxisRemap, BNO055AxisConfig, BNO055AxisSign};

mod config;
mod math;

static CALIBRATION_STATUS_ZERO : bno055::BNO055CalibrationStatus = bno055::BNO055CalibrationStatus{ sys: 0, gyr: 0, acc: 0, mag: 0 };

enum OSensorStatus {
    Uncalibrated(bno055::BNO055CalibrationStatus),
    Ready,
    Readings {
        gravity: mint::Vector3<f32>,
        linear_acceleration: mint::Vector3<f32>,
        //mag_data: mint::Vector3<f32>,
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
        let bno055 = Bno055::new(i2c0)
            .with_alternative_address();

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
    println!("BNO055 initialized.");

    let remap = AxisRemap::builder()
        .swap_x_with(BNO055AxisConfig::AXIS_AS_Y)
        .build()
        .expect("Should have been able to build axis remap config");
    nav_hat_board.bno055.set_axis_remap(remap).unwrap();
    nav_hat_board.bno055.set_axis_sign(BNO055AxisSign::X_NEGATIVE | BNO055AxisSign::Y_NEGATIVE)
        .expect("Unable to communicate");
    println!("BNO055 axes remapped.");

    nav_hat_board.bno055.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).unwrap();
    println!("BNO055 NDOF mode set.");

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
                gravity: nav_hat_board.bno055.gravity().unwrap(),
                linear_acceleration: nav_hat_board.bno055.linear_acceleration().unwrap(),
                //mag_data: nav_hat_board.bno055.mag_data().unwrap(),
                quaternion: nav_hat_board.bno055.quaternion().unwrap(),
                temperature: nav_hat_board.bno055.temperature().unwrap()
            };

            if let OSensorStatus::Readings { gravity, linear_acceleration, quaternion, temperature, .. } = &osensor_status {

                // Positive values indicate right wing down (roll), nose up (pitch), and nose right (yaw).
                let orientation = math::quaternion_to_euler(quaternion);

                println!("BNO055 readings:");
                println!("  orientation: {:>+06.1}° roll, {:>+06.1}° pitch, {:>+06.1}° yaw", orientation.roll, orientation.pitch, orientation.yaw);
                println!("  gravity: <{:>+6.2} m/s², {:>+6.2} m/s², {:>+6.2} m/s²>", gravity.x, gravity.y, gravity.z);
                println!("  linear_acceleration: <{:>+6.2} m/s², {:>+6.2} m/s², {:>+6.2} m/s²>", linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
                //println!("  mag_data: <{:>+6.2}, {:>+6.2}, {:>+6.2}>", mag_data.x, mag_data.y, mag_data.z);
                //println!("  quaternion: <{:>+6.2}, {:>+6.2}, {:>+6.2}, {:>+6.2}>", quaternion.v.x, quaternion.v.y, quaternion.v.z, quaternion.s);
                println!("  temperature: {:.1}°C", temperature);
            }
        }

        delay.delay_millis(100);
    }
}