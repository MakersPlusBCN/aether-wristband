use embassy_time::{Duration, Instant};
use imu_fusion::{Fusion, FusionAhrsSettings, FusionConvention,
                 FusionMatrix, FusionQuaternion, FusionVector,
                 //FusionEuler,
};

pub struct ImuTracker {
    time: Instant,
    pub fusion: Fusion,
    //pub euler: FusionEuler,
    pub latest_delta: f32,
    pub earth_accel: FusionVector,
    pub linear_accel: FusionVector,
}

impl ImuTracker {
    pub fn new(sampling_period: Duration, now: Instant, gyr_range: f32,
               acc_misalignment: FusionMatrix, acc_offset: FusionVector,
               acc_sensitivity: FusionVector, gyr_offset: FusionVector) -> Self {
        // Set the gyroscope range in degrees/s

        let sampling_freq: f32 = 1000_f32 / sampling_period.as_millis() as f32;

        // Set AHRS algorithm settings
        let mut ahrs_settings = FusionAhrsSettings::new();
        ahrs_settings.convention = FusionConvention::NWU;

        /* A gain of 0.5 resulted in a strange, long transient that made the
         * computed euler angles drift toward zero after a rotation was over.
         */
        ahrs_settings.gain = 0f32;

        ahrs_settings.acc_rejection = 10.0f32;
        ahrs_settings.recovery_trigger_period = 5;// * sampling_freq as i32;
        ahrs_settings.gyr_range = gyr_range;

        let mut fusion = Fusion::new(sampling_freq as u32, ahrs_settings);
        fusion.acc_misalignment = acc_misalignment;
        fusion.acc_sensitivity = acc_sensitivity;
        fusion.acc_offset = acc_offset;
        fusion.gyr_offset = gyr_offset;

        Self {
            time: now,
            fusion,
            //euler: FusionEuler::zero(),
            latest_delta: 0f32,
            earth_accel: FusionVector::zero(),
            linear_accel: FusionVector::zero(),
        }
    }

    pub fn update(&mut self, time: Instant, imu_accel: FusionVector, imu_gyro: FusionVector, imu_mag: FusionVector) {
        // Gets: acceleration in units of standard gravity
        //       angular rotation in degrees/sec
        let delta = time.duration_since(self.time).as_micros() as f32 / 1e6;
        self.time = time;
        self.latest_delta = delta;
        self.fusion.update_by_duration_seconds(imu_gyro, imu_accel, imu_mag, delta);

        self.compute(imu_accel);
    }

    pub fn compute(&mut self, imu_accel: FusionVector) {
        // Gets heading in units of degrees
        //self.euler = self.fusion.euler();

        /* These doesn't seem to work well on the imu-rs implementation
        self.earth_accel = self.fusion.ahrs.earth_acc();
        self.linear_acc = self.fusion.ahrs.linear_acc();
         */
        let q = self.fusion.quaternion();
        self.earth_accel = rotate(imu_accel, q);

        self.linear_accel.x = self.earth_accel.x * 9.807;
        self.linear_accel.y = self.earth_accel.y * 9.807;
        self.linear_accel.z = (self.earth_accel.z - 1.) * 9.807;
    }

}

fn conjugate(q: &FusionQuaternion) -> FusionQuaternion {
    FusionQuaternion {
        w: q.w, x: -q.x, y: -q.y, z: -q.z
    }
}

fn rotate2(vec: FusionVector, q: FusionQuaternion) -> FusionVector {
    let qn = q.normalize();
    let rot_q = (qn * vec) * conjugate(&qn);
    FusionVector {
        x: rot_q.x, y: rot_q.y, z: rot_q.z
    }
}

fn rotate(v: FusionVector, q: FusionQuaternion) -> FusionVector {

    let qn = q.normalize();

    let ww = qn.w * qn.w;
    let xx = qn.x * qn.x;
    let yy = qn.y * qn.y;
    let zz = qn.z * qn.z;
    let wx = qn.w * qn.x;
    let wy = qn.w * qn.y;
    let wz = qn.w * qn.z;
    let xy = qn.x * qn.y;
    let xz = qn.x * qn.z;
    let yz = qn.y * qn.z;

    // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // p2.x = w*w*v.x + 2*y*w*v.z - 2*z*w*v.y + x*x*v.x + 2*y*x*v.y + 2*z*x*v.z - z*z*v.x - y*y*v.x;
    // p2.y = 2*x*y*v.x + y*y*v.y + 2*z*y*v.z + 2*w*z*v.x - z*z*v.y + w*w*v.y - 2*x*w*v.z - x*x*v.y;
    // p2.z = 2*x*z*v.x + 2*y*z*v.y + z*z*v.z - 2*w*y*v.x - y*y*v.z + 2*w*x*v.y - x*x*v.z + w*w*v.z;

    let x = ww*v.x + 2.0*wy*v.z - 2.0*wz*v.y +
            xx*v.x + 2.0*xy*v.y + 2.0*xz*v.z -
            zz*v.x - yy*v.x;
    let y = 2.0*xy*v.x + yy*v.y + 2.0*yz*v.z +
            2.0*wz*v.x - zz*v.y + ww*v.y -
            2.0*wx*v.z - xx*v.y;
    let z = 2.0*xz*v.x + 2.0*yz*v.y + zz*v.z -
            2.0*wy*v.x - yy*v.z + 2.0*wx*v.y -
            xx*v.z + ww*v.z;

    FusionVector::new(x, y, z)
}