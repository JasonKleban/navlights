//use bno055::mint;

pub static CALIBRATION_STATUS_ZERO : bno055::BNO055CalibrationStatus = bno055::BNO055CalibrationStatus{ sys: 0, gyr: 0, acc: 0, mag: 0 };

pub enum OSensorStatus {
    Uncalibrated(bno055::BNO055CalibrationStatus),
    Ready,
    // Readings {
    //     gravity: mint::Vector3<f32>,
    //     linear_acceleration: mint::Vector3<f32>,
    //     //mag_data: mint::Vector3<f32>,
    //     quaternion: mint::Quaternion<f32>,
    //     temperature: i8,
    // }
}
