use num_quaternion::{Q32, ReadMat3x3, UQ32};
use num_traits::Float;

#[derive(Clone, Copy, Debug)]
struct Tilt {
    pitch: f32,
    roll: f32,
}

#[derive(Clone, Copy, Debug)]
struct Vec3(pub f32, pub f32, pub f32);
impl Vec3 {
    fn normalize(self) -> Vec3 {
        let mag = self.norm();
        self.scale(1.0 / mag)
    }

    fn dot(self, other: Vec3) -> f32 {
        self.0 * other.0 + self.1 * other.1 + self.2 * other.2
    }

    fn norm(self) -> f32 {
        self.dot(self).sqrt()
    }

    fn scale(self, s: f32) -> Vec3 {
        Vec3(self.0 * s, self.1 * s, self.2 * s)
    }

    pub fn cross(self, other: Vec3) -> Vec3 {
        Vec3(
            self.1 * other.2 - self.2 * other.1,
            self.2 * other.0 - self.0 * other.2,
            self.0 * other.1 - self.1 * other.0,
        )
    }
}

impl From<(f32, f32, f32)> for Vec3 {
    fn from(value: (f32, f32, f32)) -> Self {
        Self(value.0, value.1, value.2)
    }
}

impl From<[f32; 3]> for Vec3 {
    fn from(value: [f32; 3]) -> Self {
        Self(value[0], value[1], value[2])
    }
}

struct Mat3 {
    pub m: [[f32; 3]; 3],
}

impl Mat3 {
    pub fn from_cols(c0: Vec3, c1: Vec3, c2: Vec3) -> Mat3 {
        Mat3 {
            m: [[c0.0, c1.0, c2.0], [c0.1, c1.1, c2.1], [c0.2, c1.2, c2.2]],
        }
    }
    pub fn transpose(self) -> Mat3 {
        let a = self.m;
        Mat3 {
            m: [
                [a[0][0], a[1][0], a[2][0]],
                [a[0][1], a[1][1], a[2][1]],
                [a[0][2], a[1][2], a[2][2]],
            ],
        }
    }

    pub fn mul(self, o: Mat3) -> Mat3 {
        let a = self.m;
        let b = o.m;
        let mut r = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
        Mat3 { m: r }
    }
}

impl ReadMat3x3<f32> for Mat3 {
    fn at(&self, row: usize, col: usize) -> &f32 {
        &self.m[col][row]
    }
}

fn get_tilt(accel: (f32, f32, f32)) -> Tilt {
    let roll = -f32::atan2(accel.1, -accel.2);
    let pitch = f32::atan2(accel.0, f32::sqrt(accel.1 * accel.1 + accel.2 * accel.2));
    Tilt { pitch, roll }
}

/// Find rotation from earth using TRIAD
///
/// TRIAD solves a rotation matrix given two non-colinear vectors which are known in both frames.
///
/// We have a gravity vector, and in this case a "north" vector (rather than a magnetic field
/// vector, which is slightly different) because we are not really measuring yaw, but want to use
/// the last estimate from gyro integration.
fn triad(g: impl Into<Vec3>, north: impl Into<Vec3>) -> UQ32 {
    let g: Vec3 = g.into();
    let north: Vec3 = north.into();
    let g_earth = Vec3(0.0, 0.0, -1.0);
    let n_earth = Vec3(1.0, 0.0, 0.0);

    // Body triad
    let t1b = g.normalize();
    let t2b = t1b.cross(north);
    let t3b = t1b.cross(t2b);
    let body_mat = Mat3::from_cols(t1b, t2b, t3b);

    // Earth triad
    let t1e = g_earth;
    let t2e = t1e.cross(n_earth);
    let t3e = t1e.cross(t2e);
    let earth_mat = Mat3::from_cols(t1e, t2e, t3e);

    let rot_mat = body_mat.mul(earth_mat.transpose());
    UQ32::from_rotation_matrix3x3(&rot_mat)
}

fn normalize(v: (f32, f32, f32)) -> (f32, f32, f32) {
    let mag = f32::sqrt(v.0 * v.0 + v.1 * v.1 + v.2 * v.2);
    (v.0 / mag, v.1 / mag, v.2 / mag)
}

pub struct ComplementaryTilt {
    sample_period: f32,
    alpha: f32,
    attitude: Option<UQ32>,
}

impl ComplementaryTilt {
    pub fn new(sample_period: f32, alpha: f32) -> Self {
        Self {
            sample_period,
            alpha,
            attitude: None,
        }
    }

    /// Update filter with a new sample
    ///
    /// Sample period is assumed to be constant.
    ///
    /// # Arguments
    /// - `accel`: Acceleration measuruement (x, y, z). Units are not relevant.
    /// - `gyro`: Body rates (x, y, z) in rad/s.
    pub fn update(&mut self, accel: (f32, f32, f32), gyro: (f32, f32, f32)) -> (f32, f32) {
        let accel = normalize(accel);
        let gyro: Vec3 = gyro.into();

        if self.attitude.is_none() {
            // On first sample, take the accel as truth (i.e. assume we aren't under non gravitional
            // acceleration) to init
            let tilt = get_tilt(accel);
            let attitude = UQ32::from_euler_angles(tilt.roll, tilt.pitch, 0.0);
            let north = attitude.rotate_vector([1.0, 0.0, 0.0]);
            self.attitude = Some(triad(accel, north));
            return self.attitude();
        }
        let attitude = self.attitude.as_mut().unwrap();

        // Attitude predicted by accel alone
        // Using previous heading instead of mag to determine north vector
        let north = attitude.conj().rotate_vector([1.0, 0.0, 0.0]);
        let accel_attitude = triad(accel, north);

        // Total rotation (theta)
        let half_theta = gyro.norm() * self.sample_period / 2.0;
        let norm_gyro = gyro.normalize();
        let gyro_quat = Q32::new(
            half_theta.cos(),
            norm_gyro.0 * half_theta.sin(),
            norm_gyro.1 * half_theta.sin(),
            norm_gyro.2 * half_theta.sin(),
        );
        let prop_att = (*attitude * gyro_quat).normalize().unwrap_or(UQ32::one());

        self.attitude = Some(prop_att.slerp(&accel_attitude, 1.0 - self.alpha));
        self.attitude()
    }

    /// Get current (pitch, roll) estimate
    pub fn attitude(&self) -> (f32, f32) {
        let attitude = self.attitude.unwrap_or(UQ32::one());
        let euler = attitude.to_euler_angles();
        (euler.pitch, euler.roll)
    }
}

struct HighPassValue {
    y: f32,
    last_x: f32,
    alpha: f32,
}
impl HighPassValue {
    pub fn new(alpha: f32) -> Self {
        Self {
            y: 0.0,
            last_x: 0.0,
            alpha: alpha,
        }
    }

    pub fn integrate(&mut self, delta_x: f32) -> f32 {
        self.y = self.alpha * (self.y + delta_x);
        self.y
    }
    pub fn update(&mut self, x: f32) -> f32 {
        let y_new = self.alpha * (self.y + x - self.last_x);
        self.y = y_new;
        self.last_x = x;
        y_new
    }

    pub fn value(&self) -> f32 {
        self.y
    }
}

pub struct HeaveEstimate {
    heave: HighPassValue,
    velocity: HighPassValue,
    accel: HighPassValue,
    sample_period: f32,
}

impl HeaveEstimate {
    pub fn new(sample_period: f32, alpha: f32) -> Self {
        Self {
            heave: HighPassValue::new(alpha),
            velocity: HighPassValue::new(alpha),
            accel: HighPassValue::new(alpha),
            sample_period,
        }
    }

    /// Update the heave estimate
    ///
    /// # Arguments
    /// - `pitch`: Current pitch angle in rad
    /// - `roll`: Current roll angle in rad
    /// - `accel`: Body acceleration (m/s/s)
    pub fn update(&mut self, pitch: f32, roll: f32, accel: (f32, f32, f32)) -> f32 {
        // Get the earth frame "down" vector as a unit vector in body frame
        let ux = -pitch.sin() * roll.cos();
        let uy = pitch.cos() * roll.sin();
        let uz = pitch.cos() * roll.cos();

        // Get earth vertical acceleration (removing gravity)
        let accel = self
            .accel
            .update(ux * accel.0 + uy * accel.1 + uz * accel.2 + crate::G_MPSS);

        // High-pass filter velocity
        let vel = self.velocity.integrate(accel * self.sample_period);

        // High-pass filter heave (vertical)
        self.heave.integrate(vel * self.sample_period)
    }

    #[allow(dead_code)]
    pub fn heave(&self) -> f32 {
        self.heave.value()
    }
}

pub struct FilterSample {
    pub pitch: f32,
    pub roll: f32,
    pub heave: f32,
}

pub struct Filter {
    input_period: f32,
    output_period: f32,
    tilt_estimator: ComplementaryTilt,
    heave_estimator: HeaveEstimate,
    prev_pitch: f32,
    prev_roll: f32,
    prev_heave: f32,
    samples_since_output: f32,
}

impl Filter {
    pub fn new(input_period: f32, output_period: f32) -> Self {
        const TILT_ALPHA: f32 = 0.995;
        const HEAVE_ALPHA: f32 = 0.999;
        Self {
            input_period,
            output_period,
            tilt_estimator: ComplementaryTilt::new(input_period, TILT_ALPHA),
            heave_estimator: HeaveEstimate::new(input_period, HEAVE_ALPHA),
            prev_pitch: 0.0,
            prev_roll: 0.0,
            prev_heave: 0.0,
            samples_since_output: 0.0,
        }
    }

    /// Push a new sample into the filter
    pub fn push_sample(
        &mut self,
        accel_mpss: (f32, f32, f32),
        gyro_rps: (f32, f32, f32),
    ) -> Option<FilterSample> {
        let (pitch, roll) = self.tilt_estimator.update(accel_mpss, gyro_rps);
        let heave = self.heave_estimator.update(pitch, roll, accel_mpss);
        self.samples_since_output += 1.0;
        let time_since_output = self.samples_since_output * self.input_period;
        let mut result = None;
        if time_since_output > self.output_period {
            let interp = (time_since_output - self.output_period) / self.input_period;
            // Interpolating roll/pitch independently is not technically correct. It's probably a
            // small error in this context, but a quaternion interp would be better...
            let interp_pitch = pitch * interp + self.prev_pitch * (1.0 - interp);
            let interp_roll = roll * interp + self.prev_roll * (1.0 - interp);
            let interp_heave = heave * interp + self.prev_heave * (1.0 - interp);
            result = Some(FilterSample {
                pitch: interp_pitch,
                roll: interp_roll,
                heave: interp_heave,
            });
            self.samples_since_output = interp;
        }

        self.prev_heave = heave;
        self.prev_pitch = pitch;
        self.prev_roll = roll;

        result
    }
}
