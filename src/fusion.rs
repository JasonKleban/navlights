use num_traits::float::Float;

use crate::math::{Quaternion, Vec3};
use core::f32::consts::PI;

/// Principal angle using trig periodicity (-π..π)
fn angle_error(target: f32, estimate: f32) -> f32 {
    (target - estimate).atan2((target - estimate).cos())
}

pub struct Fusion {
    /// Velocity in horizontal plane (X=forward, Y=right)
    velocity_body: Vec3,

    yaw_estimate: Option<f32>,
    yaw_offset: f32,

    gps_age: f32,
    imu_age: f32,

    mag_declination_radians: f32,

    filtered_acc: Vec3,

    // --- Debug overrides in ENU frame ---
    debug_velocity_enu: Option<Vec3>,      // x=East, y=North, z=Up
    debug_acc_enu: Option<Vec3>,           // x=East, y=North, z=Up
}

impl Fusion {
    pub fn new() -> Self {
        Self {
            velocity_body: Vec3::ZERO,
            yaw_estimate: None,
            yaw_offset: 0.0,
            gps_age: 1e6,
            imu_age: 1e6,
            mag_declination_radians: 0.0,
            filtered_acc: Vec3::ZERO, 
            debug_velocity_enu: None,
            debug_acc_enu: None,
        }
    }
    
    pub fn set_debug_velocity_enu(&mut self, vel_enu: Option<Vec3>) {
        self.debug_velocity_enu = vel_enu;
    }

    pub fn set_debug_acceleration_enu(&mut self, acc_enu: Option<Vec3>) {
        self.debug_acc_enu = acc_enu;
    }

    // ---------------- IMU UPDATE ----------------
    /// Update fusion with a new body quaternion and linear acceleration in body axes.
    /// Assumes quaternion rotates body → inertial frame aligned with aircraft axes:
    /// X=forward, Y=right, Z=down
    pub fn update_imu(
        &mut self,
        q_body_to_inertial: Quaternion,
        linear_acc_body: Vec3,
        dt: f32,
        sys_confidence: u8,
    ) {
        if let Some(debug_acc_enu) = &mut self.debug_acc_enu {
            if let Some(debug_velocity_enu) = &mut self.debug_velocity_enu {
                self.debug_velocity_enu =
                    Some(debug_velocity_enu.add(
                        debug_acc_enu.scale(dt)
                    ));
            }
        }

        self.imu_age = 0.0;
        let weight = (sys_confidence as f32) / 3.0;

        // Rotate body acceleration into inertial frame (horizontal plane)
        let mut acc_inertial = q_body_to_inertial.rotate(linear_acc_body);

        // --- Low pass filter (IMU smoothing) ---
        let alpha = 0.85;

        self.filtered_acc.x = self.filtered_acc.x * alpha + acc_inertial.x * (1.0 - alpha);
        self.filtered_acc.y = self.filtered_acc.y * alpha + acc_inertial.y * (1.0 - alpha);
        self.filtered_acc.z = self.filtered_acc.z * alpha + acc_inertial.z * (1.0 - alpha);

        acc_inertial = self.filtered_acc;

        let deadband = 0.05;

        if acc_inertial.x.abs() < deadband { acc_inertial.x = 0.0; }
        if acc_inertial.y.abs() < deadband { acc_inertial.y = 0.0; }

        // --- Inject debug acceleration in ENU, converted to body frame ---
        if let Some(acc_enu) = self.debug_acc_enu {
            // ENU → aircraft body axes: x=East→Y_body, y=North→X_body, z=Up→-Z_body
            acc_inertial.x += acc_enu.y;      // North → forward
            acc_inertial.y += acc_enu.x;      // East → right
            acc_inertial.z += -acc_enu.z;     // Up → down
        }

        // Integrate velocity in horizontal plane only
        self.velocity_body.x += acc_inertial.x * dt * weight;
        self.velocity_body.y += acc_inertial.y * dt * weight;

        // Simple decay to prevent drift
        let tau = 1.5;
        let decay = (-dt / tau).exp();
        self.velocity_body.x *= decay;
        self.velocity_body.y *= decay;

        // --- Inject debug velocity in ENU, converted to body frame ---
        if let Some(vel_enu) = self.debug_velocity_enu {
            self.velocity_body.x = vel_enu.y;     // North → forward
            self.velocity_body.y = vel_enu.x;     // East → right
            // Z ignored for horizontal-plane velocity
        }

        // Extract aircraft roll/pitch/yaw
        let w = q_body_to_inertial.w;
        let x = q_body_to_inertial.x;
        let y = q_body_to_inertial.y;
        let z = q_body_to_inertial.z;

        // Yaw = 0 = North, clockwise positive
        let siny = 2.0 * (w*y - z*x);
        let cosy = w*w + x*x - y*y - z*z;
        let yaw = (siny.atan2(cosy) + 2.0*PI) % (2.0*PI);

        // Apply magnetic declination and offset
        let yaw_corrected = yaw + self.mag_declination_radians + self.yaw_offset;

        // Blend into estimate
        let beta = 0.2 * weight;
        if let Some(value) = &mut self.yaw_estimate {
            let delta = angle_error(yaw_corrected, *value);
            *value += beta * delta;
        } else {
            self.yaw_estimate = Some(yaw_corrected);
        }
    }

    // ---------------- GPS UPDATE ----------------
    /// Update velocity estimate from GPS in North-East horizontal plane
    pub fn update_gps(&mut self, speed_mps: f32, track_true_rad: f32) {
        if self.debug_velocity_enu.is_some() {
            return; // skip GPS override when faking motion
        }

        if speed_mps < 0.5 {
            self.velocity_body = Vec3::ZERO;
            return;
        }

        self.gps_age = 0.0;

        // Convert GPS track to aircraft horizontal plane (X=forward/North, Y=right/East)
        let v_gps = Vec3 {
            x: speed_mps * track_true_rad.cos(),
            y: speed_mps * track_true_rad.sin(),
            z: 0.0,
        };

        let diff_x = v_gps.x - self.velocity_body.x;
        let diff_y = v_gps.y - self.velocity_body.y;
        if (diff_x*diff_x + diff_y*diff_y).sqrt() > 25.0 {
            return;
        }

        let alpha = 0.95;
        self.velocity_body.x = self.velocity_body.x*alpha + v_gps.x*(1.0-alpha);
        self.velocity_body.y = self.velocity_body.y*alpha + v_gps.y*(1.0-alpha);
    }

    // ---------------- YAW CORRECTION ----------------
    pub fn correct_yaw_from_gps(&mut self, imu_yaw_rad: f32, track_true_rad: f32) {
        let imu_true = imu_yaw_rad + self.mag_declination_radians + self.yaw_offset;
        let err = angle_error(track_true_rad, imu_true);
        self.yaw_offset += 0.05 * err;

        if let Some(value) = &mut self.yaw_estimate {
            let delta = angle_error(track_true_rad, *value);
            *value += 0.05 * delta;
        } else {
            self.yaw_estimate = Some(track_true_rad);
        }
    }

    pub fn update_declination(&mut self, declination_rad: f32) {
        self.mag_declination_radians =
            0.98*self.mag_declination_radians + 0.02*declination_rad;
    }

    // ---------------- TIME UPDATE ----------------
    pub fn update_time(&mut self, dt: f32) {
        self.gps_age += dt;
        self.imu_age += dt;

        if self.gps_age > 2.0 && self.imu_age > 2.0 {
            self.velocity_body.x *= 0.98;
            self.velocity_body.y *= 0.98;
        }
    }

    // ---------------- OUTPUT ----------------
    pub fn yaw(&self) -> Option<f32> { self.yaw_estimate }
    pub fn velocity(&self) -> Vec3 { self.velocity_body }
    pub fn speed(&self) -> f32 {
        (self.velocity_body.x*self.velocity_body.x + self.velocity_body.y*self.velocity_body.y).sqrt()
    }

    /// Returns direction of velocity relative to body X-axis (forward)
    pub fn body_relative_direction(&self, q_body_to_inertial: Quaternion) -> f32 {
        let v_body = q_body_to_inertial.conjugate().rotate(self.velocity_body);
        v_body.y.atan2(v_body.x)
    }

    /// Returns heading of inertial North relative to body X-axis
    pub fn body_relative_north(&self, q_body_to_inertial: Quaternion) -> f32 {
        let north_vec = Vec3 { x:1.0, y:0.0, z:0.0 };
        let v_body = q_body_to_inertial.conjugate().rotate(north_vec);
        v_body.y.atan2(v_body.x)
    }
}