use num_traits::float::Float;

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

// principal angle using trig periodicity
fn angle_error(target: f32, estimate: f32) -> f32 {
    let d = target - estimate;
    d.sin().atan2(d.cos())
}

pub struct Fusion {
    velocity_ned: Vec3,

    yaw_estimate: f32,      // <--- added
    yaw_bias: f32,
    mag_bias: f32,

    gps_age: f32,
    imu_age: f32,

    mag_declination_radians: f32,
}

impl Fusion {

    pub fn new() -> Self {
        Self {
            velocity_ned: Vec3::ZERO,
            yaw_estimate: 0.0,
            yaw_bias: 0.0,
            mag_bias: 0.0,
            gps_age: 1e6,
            imu_age: 1e6,
            mag_declination_radians: 0.0
        }
    }

    // ------------- IMU UPDATE -------------

    pub fn update_imu(
        &mut self,
        q_body_to_ned: Quaternion,
        linear_acc_body: Vec3,
        dt: f32,
        sys_confidence: u8,
    ) {
        self.imu_age = 0.0;

        let weight = (sys_confidence as f32) / 3.0;

        let acc_ned = q_body_to_ned.rotate(linear_acc_body);

        self.velocity_ned = self.velocity_ned.add(acc_ned.scale(dt * weight));

        let tau = 1.5; // seconds to decay
        let decay = (-dt / tau).exp();
        self.velocity_ned = self.velocity_ned.scale(decay);

        // Extract yaw from quaternion
        let yaw = f32::atan2(
            2.0 * (q_body_to_ned.w * q_body_to_ned.z +
                q_body_to_ned.x * q_body_to_ned.y),
            1.0 - 2.0 * (q_body_to_ned.y * q_body_to_ned.y +
                        q_body_to_ned.z * q_body_to_ned.z),
        );

        // Convert magnetic to true north
        let yaw_true = yaw + self.mag_declination_radians;

        // Apply bias
        let yaw_corrected = yaw_true + self.mag_bias;

        // Blend into yaw_estimate
        let beta = 0.2 * weight;
        let delta = angle_error(yaw_corrected, self.yaw_estimate);
        self.yaw_estimate += beta * delta;
    }

    // ------------- GPS UPDATE -------------

    pub fn update_gps(
        &mut self,
        speed_mps: f32,
        track_true_rad: f32,
    ) {
        if speed_mps < 0.5 {
            self.velocity_ned = Vec3::ZERO;
            return;
        }

        self.gps_age = 0.0;

        let v_gps = Vec3 {
            x: speed_mps * track_true_rad.cos(),
            y: speed_mps * track_true_rad.sin(),
            z: 0.0,
        };

        let diff = Vec3 {
            x: v_gps.x - self.velocity_ned.x,
            y: v_gps.y - self.velocity_ned.y,
            z: 0.0,
        };

        if diff.norm() > 25.0 {
            return;
        }

        let alpha = 0.95;
        self.velocity_ned =
            self.velocity_ned.scale(alpha)
            .add(v_gps.scale(1.0 - alpha));
    }

    // ------------- YAW CORRECTION -------------

    pub fn correct_yaw_from_gps(
        &mut self,
        imu_yaw_rad: f32,
        track_true_rad: f32
    ) {
        // Convert IMU yaw to true north and apply biases
        let imu_true = self.corrected_yaw(imu_yaw_rad);

        // --- Slow magnetic bias refinement ---
        self.refine_mag_bias(track_true_rad, imu_true);

        // --- Fast yaw estimate correction ---
        let delta = angle_error(track_true_rad, self.yaw_estimate);

        let gain = 0.05;   // surgical constant instead of config
        self.yaw_estimate += gain * delta;
    }

    pub fn update_declination(&mut self, declination_rad: f32) {
        self.mag_declination_radians =
            0.98 * self.mag_declination_radians
          + 0.02 * declination_rad;
    }

    pub fn refine_mag_bias(
        &mut self,
        track_true_rad: f32,
        imu_true_yaw: f32,
    ) {
        let err = angle_error(track_true_rad, imu_true_yaw);

        let gamma = 0.005; // slow learning
        self.mag_bias += gamma * err;
    }

    pub fn corrected_yaw(&self, imu_yaw_rad: f32) -> f32 {
        imu_yaw_rad
        + self.mag_declination_radians
        + self.yaw_bias
        + self.mag_bias
    }

    // ------------- TIME UPDATE -------------

    pub fn update_time(&mut self, dt: f32) {
        self.gps_age += dt;
        self.imu_age += dt;

        if self.gps_age > 2.0 && self.imu_age > 2.0 {
            self.velocity_ned = self.velocity_ned.scale(0.98);
        }
    }

    // ------------- OUTPUT -------------

    pub fn yaw(&self) -> f32 {
        self.yaw_estimate
    }

    pub fn speed_over_ground(&self) -> f32 {
        (self.velocity_ned.x*self.velocity_ned.x +
         self.velocity_ned.y*self.velocity_ned.y).sqrt()
    }

    pub fn body_relative_direction(
        &self,
        q_body_to_ned: Quaternion,
    ) -> f32 {
        let v_body = q_body_to_ned.conjugate()
            .rotate(self.velocity_ned);

        v_body.y.atan2(v_body.x)
    }

    pub fn velocity_ned(&self) -> Vec3 {
        self.velocity_ned
    }
}