use bno055::mint;
use num_traits::float::Float;

pub const KTS_PER_METER_SECOND : f32  = 1.94384;
pub const DEGREES_TO_RADIANS : f32  = core::f32::consts::PI / 180.0;
pub const RADIANS_TO_DEGREES : f32  = 180.0 / core::f32::consts::PI;

pub struct Orientation {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

pub fn quaternion_to_euler(q: &mint::Quaternion<f32>) -> Orientation {
    let norm = libm::sqrtf(q.s * q.s + q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z);

    let w = q.s / norm;
    let x = q.v.x / norm;
    let y = q.v.y / norm;
    let z = q.v.z / norm;

    // Roll (X axis)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (Y axis)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * core::f32::consts::FRAC_PI_2
    } else {
        sinp.asin()
    };

    // Yaw (Z axis)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = (siny_cosp).atan2(-cosy_cosp);

    Orientation { roll: roll, pitch: pitch, yaw: yaw }
}