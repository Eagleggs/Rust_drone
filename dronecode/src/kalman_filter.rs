use crate::sensor::Sensor;
use crate::yaw_pitch_roll_quaternion::yaw_pitch_roll_from_acc;
use architecture::{Frac, Velocity, YawPitchRoll};

pub struct KalmanFilter {
    acc_sphi: YawPitchRoll,
    out_rate: Velocity,
    out_phi: YawPitchRoll,
    pub integration_constant: Frac,
    c1: Frac,
    c2: Frac,
}

impl KalmanFilter {
    pub fn new(c1: Frac, c2: Frac) -> Self {
        KalmanFilter {
            acc_sphi: YawPitchRoll {
                yaw: Frac::from_num(0.),
                pitch: Frac::from_num(0.),
                roll: Frac::from_num(0.),
            },
            out_rate: Velocity {
                yaw: Frac::from_num(0.),
                pitch: Frac::from_num(0.),
                roll: Frac::from_num(0.),
            },
            out_phi: YawPitchRoll {
                yaw: Frac::from_num(0.),
                pitch: Frac::from_num(0.),
                roll: Frac::from_num(0.),
            },
            integration_constant: Frac::from_num(0.),
            c1,
            c2,
        }
    }
    pub fn fusion_algorithm(&mut self, sensor: &mut Sensor) {
        self.acc_sphi = yaw_pitch_roll_from_acc(
            sensor.data.acceleration + sensor.calibrate_offset.acceleration,
        );
        self.out_rate = sensor.data.velocity;
        self.out_phi = self.out_phi + self.out_rate * self.integration_constant;
        let e = self.out_phi - self.acc_sphi;
        // sensor.data.radius = self.out_phi;
        self.out_phi = self.out_phi - e / self.c1;
        sensor.calibrate_offset.velocity =
            sensor.calibrate_offset.velocity + (e / self.integration_constant) / self.c2;
        sensor.data.radius = self.out_phi - sensor.calibrate_offset.radius;
        sensor.data.radius.yaw = Frac::from_num(0.);
    }
}
