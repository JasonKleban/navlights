use num_traits::float::Float;

pub const KTS_PER_METER_SECOND : f32  = 1.94384;
pub const DEGREES_TO_RADIANS : f32  = core::f32::consts::PI / 180.0;
pub const RADIANS_TO_DEGREES : f32  = 180.0 / core::f32::consts::PI;

pub struct Orientation {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Copy, Clone, Debug)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    pub fn norm(&self) -> f32 {
        (self.x*self.x + self.y*self.y + self.z*self.z).sqrt()
    }

    pub fn scale(self, k: f32) -> Self {
        Self { x: self.x*k, y: self.y*k, z: self.z*k }
    }

    pub fn add(self, o: Self) -> Self {
        Self { x: self.x+o.x, y: self.y+o.y, z: self.z+o.z }
    }
}

#[derive(Copy, Clone)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn normalize(&self) -> Self {
        let norm = (self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z).sqrt();

        Quaternion { w: self.w / norm, x: self.x / norm, y: self.y / norm, z: self.z / norm }
    }

    pub fn conjugate(self) -> Self {
        Self { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }

    pub fn rotate(self, v: Vec3) -> Vec3 {
        let qv = Quaternion { w: 0.0, x: v.x, y: v.y, z: v.z };
        let r = self.mul(qv).mul(self.conjugate());
        Vec3 { x: r.x, y: r.y, z: r.z }
    }

    fn mul(self, o: Self) -> Self {
        Self {
            w: self.w*o.w - self.x*o.x - self.y*o.y - self.z*o.z,
            x: self.w*o.x + self.x*o.w + self.y*o.z - self.z*o.y,
            y: self.w*o.y - self.x*o.z + self.y*o.w + self.z*o.x,
            z: self.w*o.z + self.x*o.y - self.y*o.x + self.z*o.w,
        }
    }
}

/// Converts a BNO055 quaternion into aviation-style Euler angles.
///
/// Assumptions:
/// - Quaternion rotates sensor/body frame into Earth ENU frame
///     Earth frame:
///         X = East
///         Y = North
///         Z = Up
///     Positive yaw is counter-clockwise (right-handed)
///
/// - Sensor mounted in BNO055 default orientation:
///         +X = right
///         +Y = forward
///         +Z = up
///
/// Output conventions (aviation):
/// - Roll  (rad): right wing down positive
/// - Pitch (rad): nose up positive
/// - Yaw   (rad): 0 = North, increases clockwise, range [0, 2π)
///
/// No axis remap or sign inversion should be configured in the BNO055.
pub fn quaternion_to_euler(q: &Quaternion) -> Orientation {
    // Normalize quaternion
    let norm = libm::sqrtf(
        q.w * q.w +
        q.x * q.x +
        q.y * q.y +
        q.z * q.z
    );

    let w = q.w / norm;
    let x = q.x / norm;
    let y = q.y / norm;
    let z = q.z / norm;

    // --- Standard Z-Y-X extraction (ENU frame) ---

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

    let siny = 2.0 * (w * z + x * y);
    let cosy = w*w + x*x - y*y - z*z;
    let yaw = ((siny).atan2(cosy) + 2.0 * core::f32::consts::PI) % (2.0 * core::f32::consts::PI);

    Orientation {
        roll,
        pitch,
        yaw,
    }
}