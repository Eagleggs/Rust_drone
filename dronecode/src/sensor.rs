use alloc::vec;
use alloc::vec::Vec;
use architecture::{Frac, Message, SensorData, SensorDriver};

use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::time::assembly_delay;
use tudelft_quadrupel::{
    battery::read_battery,
    block,
    motor::get_motors,
    mpu::{read_dmp_bytes, read_raw},
};
//use tudelft_quadrupel::barometer::read_pressure;
use crate::yaw_pitch_roll_quaternion::yaw_pitch_roll_from_quaternion;
use protocol::{DataLink, MessageLink};

#[derive(Clone)]
pub struct Sensor {
    pub data: SensorData,
    pub calibrate_offset: SensorData,
    pub calibrated: bool,
    pub cache: SensorData,
    pub fir_cache: Vec<SensorData>,
    pub gravity_scale: Frac,
    pub pressure_zero_point: f32,
    pub filter_times: i32,
    pub base_pressure: f32,
}

impl Sensor {
    pub fn new() -> Self {
        Sensor {
            data: SensorData::new(),
            calibrate_offset: SensorData::new(),
            calibrated: false,
            cache: SensorData::new(),
            fir_cache: vec![],
            gravity_scale: Frac::from_num(0),
            pressure_zero_point: 0.0,
            filter_times: 0,
            base_pressure: 100000.0,
        }
    }
    pub fn send_data<T: protocol::Link>(&self, link: &mut MessageLink<T>) {
        match link.send(&Message::SensorData {
            data: self.data.clone(),
        }) {
            Ok(_) => (),
            Err(_) => todo!(),
        };
    }
    pub fn calculate_height(&mut self, _integration_constant: Frac) {
        if self.calibrated {
            let height_from_pressure = (Frac::from_num(self.pressure_zero_point)
                - Frac::from_num(self.data.pressure))
                / Frac::from_num(11);
            self.data.height = height_from_pressure;
            //self.calibrate_offset.acceleration.z -= i32::from_fixed(err / c2 * Frac::from_num(11));
        }
    }
}
impl SensorDriver for Sensor {
    fn get_values(&mut self, cal_option: bool, raw: bool) {
        if !raw {
            self.data.radius = yaw_pitch_roll_from_quaternion(block!(read_dmp_bytes()).unwrap());
        }
        let raw = read_raw().unwrap();
        let angle_scale: Frac = Frac::PI / 180;
        self.data.velocity.yaw = Frac::from_num(raw.1.z) * angle_scale;
        self.data.velocity.roll = Frac::from_num(raw.1.x) * angle_scale;
        self.data.velocity.pitch = Frac::from_num(-raw.1.y) * angle_scale;
        self.data.motor_speeds = get_motors();
        self.data.acceleration.x = raw.0.x as i32;
        self.data.acceleration.y = raw.0.y as i32;
        self.data.acceleration.z = raw.0.z as i32;
        self.data.bat = read_battery();
        self.data.pressure = read_pressure() as f32; //get a larger result for filters;
        self.data.pressure = (self.data.pressure - self.base_pressure) * 10.0;

        // self.data.pressure = (self.data.pressure - 100000.0) * 10.0;

        if !self.calibrated {
            self.cache.pressure = self.data.pressure;
        }
        if cal_option == true {
            self.data.radius = self.data.radius - self.calibrate_offset.radius;
            self.data.acceleration = self.data.acceleration - self.calibrate_offset.acceleration;
            self.data.velocity = self.data.velocity - self.calibrate_offset.velocity;
            // self.data.pressure -= self.calibrate_offset.pressure;
            // self.calibrate_offset.pressure += self.calibrate_offset.pressure;
        }
    }
    fn calibrate(&mut self) {
        let mut temp = SensorData::new();
        let mut compensate_pressure = SensorData::new();
        for _i in 0..20 {
            self.get_values(false, false);
            compensate_pressure.pressure += self.data.pressure - temp.pressure;
            //wait for some time for sensor to get a better sample
            assembly_delay(1_000);
            temp.radius = temp.radius + self.data.radius;
            temp.velocity = temp.velocity + self.data.velocity;
            temp.acceleration = temp.acceleration + self.data.acceleration / 20;
            temp.pressure = self.data.pressure;
        }
        let scale: Frac = Frac::from_num(20);
        temp.radius.yaw /= scale;
        temp.radius.roll /= scale;
        temp.radius.pitch /= scale;
        temp.pressure = compensate_pressure.pressure / scale.to_num::<f32>();
        temp.velocity.yaw /= scale;
        temp.velocity.roll /= scale;
        temp.velocity.pitch /= scale;

        self.pressure_zero_point = self.cache.pressure;
        self.gravity_scale = Frac::from_num(temp.acceleration.z) / Frac::from_num(9.80665);
        self.calibrate_offset = temp;
        self.calibrated = true;
    }
    fn filter_ewma(&mut self, alpha: Frac, pressure_alpha: f32) {
        let pressure = self.data.pressure as f32;
        let cache_pressure = self.cache.pressure as f32;

        self.data = self.data * alpha + self.cache * (Frac::from_num(1) - alpha);
        self.data.pressure = pressure * pressure_alpha + cache_pressure * (1.0 - pressure_alpha);
        self.cache = self.data;
    }

    fn filter_fir(&mut self, alpha: Frac, beta: Frac, gama: Frac) {
        self.data = self.fir_cache[0] * alpha + self.fir_cache[1] * beta + self.fir_cache[2] * gama;
    }
}
