#![no_std]
extern crate fixed;

use core::ops;
use fixed::traits::FromFixed;
use fixed::types::I16F16;
use serde::{Deserialize, Serialize};

mod control_request_impl;

pub type Frac = I16F16;

pub const BASE_STATION: bool = false;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Copy)]
pub struct Velocity {
    pub yaw: Frac,
    pub pitch: Frac,
    pub roll: Frac,
}

impl Velocity {
    pub fn new() -> Self {
        Velocity {
            yaw: Frac::from_num(0),
            pitch: Frac::from_num(0),
            roll: Frac::from_num(0),
        }
    }
}
impl ops::Add<Velocity> for Velocity {
    type Output = Velocity;

    fn add(self, rhs: Self) -> Self::Output {
        Velocity {
            yaw: self.yaw + rhs.yaw,
            pitch: self.pitch + rhs.pitch,
            roll: self.roll + rhs.roll,
        }
    }
}
impl ops::Add<YawPitchRoll> for Velocity {
    type Output = Velocity;
    fn add(self, rhs: YawPitchRoll) -> Self::Output {
        Velocity {
            yaw: self.yaw + rhs.yaw,
            pitch: self.pitch + rhs.pitch,
            roll: self.roll + rhs.roll,
        }
    }
}
impl ops::Sub<Velocity> for Velocity {
    type Output = Velocity;
    fn sub(self, rhs: Self) -> Self::Output {
        Velocity {
            yaw: self.yaw - rhs.yaw,
            pitch: self.pitch - rhs.pitch,
            roll: self.roll - rhs.roll,
        }
    }
}
impl ops::Mul<Frac> for Velocity {
    type Output = YawPitchRoll;
    fn mul(self, rhs: Frac) -> Self::Output {
        YawPitchRoll {
            yaw: self.yaw * rhs,
            pitch: self.pitch * rhs,
            roll: self.roll * rhs,
        }
    }
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Copy)]
pub struct Accel {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}
impl Accel {
    pub fn new() -> Self {
        Accel { x: 0, y: 0, z: 0 }
    }
}
impl ops::Add for Accel {
    type Output = Accel;

    fn add(self, rhs: Self) -> Self::Output {
        Accel {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}
impl ops::Sub for Accel {
    type Output = Accel;

    fn sub(self, rhs: Self) -> Self::Output {
        Accel {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}
impl ops::Div<i32> for Accel {
    type Output = Accel;
    fn div(self, rhs: i32) -> Self::Output {
        Accel {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}
#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
pub enum Mode {
    Safe,
    Panic,
    Manual,
    Calibrate,
    YawControl,
    FullControl,
    Raw,      // optional
    Height,   // optional
    WireLess, // optional
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Message {
    ChangeMode {
        mode: Mode,
    },
    ControlInput {
        request: ControlRequest,
        base_pressure: f32,
    },
    SensorData {
        data: SensorData,
    },
    LogMessage {
        message: [u8; 16],
    }, // look up fix siz string
    TuneParameter {
        parameter: char,
        value: Frac,
    },
    ProfilerEvent(ProfilerEvent),
    ProfilerTimed {
        start: ProfilerEvent,
        stop: ProfilerEvent,
        ns: u64,
        count: u16,
    },
    LoggerMode {
        mode: LoggerMode,
    },
    LogDownload {
        entry: Option<SensorData>, // None signals end of download
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum LoggerMode {
    Enabled,
    Disabled,
    Download,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, enum_map::Enum)]
pub enum ProfilerEvent {
    MainLoopStart,
    MainLoopStop,
    MainLoopFullControlStart,
    MainLoopFullControlStop,
}

// Message handling Component
pub trait MessageHandler {
    fn handle_message(&mut self, message: &Message) -> ();
}

// Sensor data collection Component
#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorData {
    /*sax: u16,
    say: u16,
    saz: u16,
    sp: u16,
    sq: u16,
    sr: u16,*/
    pub height: Frac,
    pub v_z: Frac,
    pub pressure: f32,
    pub velocity: Velocity,
    pub radius: YawPitchRoll,
    pub acceleration: Accel,
    pub bat: u16,
    pub motor_speeds: [u16; 4], // might not be needed
                                // data for all sensors
}
impl ops::Mul<Frac> for SensorData {
    type Output = SensorData;

    fn mul(self, rhs: Frac) -> Self::Output {
        SensorData {
            height: self.height * rhs,
            v_z: self.v_z * rhs,
            pressure: self.pressure * rhs.to_num::<f32>(),
            velocity: Velocity {
                yaw: self.velocity.yaw * rhs,
                pitch: self.velocity.pitch * rhs,
                roll: self.velocity.roll * rhs,
            },
            radius: self.radius * rhs,
            acceleration: Accel {
                x: i32::from_fixed(self.acceleration.x * rhs),
                y: i32::from_fixed(self.acceleration.y * rhs),
                z: i32::from_fixed(self.acceleration.z * rhs),
            },
            bat: self.bat,
            motor_speeds: self.motor_speeds,
        }
    }
}
impl ops::Add<SensorData> for SensorData {
    type Output = SensorData;

    fn add(self, rhs: SensorData) -> Self::Output {
        SensorData {
            height: self.height + rhs.height,
            v_z: self.v_z + rhs.v_z,
            pressure: self.pressure + rhs.pressure,
            velocity: self.velocity + rhs.velocity,
            radius: self.radius + rhs.radius,
            acceleration: self.acceleration + rhs.acceleration,
            bat: self.bat,
            motor_speeds: self.motor_speeds,
        }
    }
}
impl SensorData {
    pub fn new() -> Self {
        SensorData {
            height: Frac::from_num(0.),
            v_z: Frac::from_num(0.),
            pressure: 0.0,
            velocity: Velocity {
                yaw: Frac::from_num(0.),
                pitch: Frac::from_num(0.),
                roll: Frac::from_num(0.),
            },
            radius: YawPitchRoll {
                yaw: Frac::from_num(0.),
                pitch: Frac::from_num(0.),
                roll: Frac::from_num(0.),
            },
            acceleration: Accel { x: 0, y: 0, z: 0 },
            bat: 0,
            motor_speeds: [0, 0, 0, 0],
        }
    }
}
pub trait SensorDriver {
    fn get_values(&mut self, calibration: bool, raw: bool);
    fn calibrate(&mut self);
    fn filter_ewma(&mut self, alpha: Frac, pressure_alpha: f32);
    fn filter_fir(&mut self, alpha: Frac, beta: Frac, gama: Frac);
}

// Drone Controller Component
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlRequest {
    pub radius: YawPitchRoll,
    pub throttle: i16, // should be lift
}
impl ControlRequest {
    pub fn new() -> Self {
        ControlRequest {
            radius: YawPitchRoll {
                yaw: Frac::from_num(0),
                pitch: Frac::from_num(0),
                roll: Frac::from_num(0),
            },
            throttle: 0,
        }
    }
}

// Joystick input Component
pub trait JoystickDriver {
    fn get_control_request(&mut self) -> ControlRequest;
}

// Telemetry Display Component
pub trait TelemetryDisplay {
    fn display_telemetry(&mut self, data: &SensorData) -> ();
}

/// This struct holds the yaw, pitch, and roll that the drone things it is in.
/// The struct is currently implemented using `f32`, you may want to change this to use fixed point arithmetic.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct YawPitchRoll {
    pub yaw: Frac,
    pub pitch: Frac,
    pub roll: Frac,
}

impl YawPitchRoll {
    pub fn new() -> Self {
        Self {
            yaw: Frac::from_num(0),
            pitch: Frac::from_num(0),
            roll: Frac::from_num(0),
        }
    }
}
impl ops::Add<YawPitchRoll> for YawPitchRoll {
    type Output = YawPitchRoll;
    fn add(self, rhs: YawPitchRoll) -> Self::Output {
        YawPitchRoll {
            yaw: self.yaw + rhs.yaw,
            pitch: self.pitch + rhs.pitch,
            roll: self.roll + rhs.roll,
        }
    }
}
impl ops::Sub<YawPitchRoll> for YawPitchRoll {
    type Output = YawPitchRoll;
    fn sub(self, rhs: YawPitchRoll) -> Self::Output {
        YawPitchRoll {
            yaw: self.yaw - rhs.yaw,
            pitch: self.pitch - rhs.pitch,
            roll: self.roll - rhs.roll,
        }
    }
}
impl ops::Div<Frac> for YawPitchRoll {
    type Output = YawPitchRoll;
    fn div(self, rhs: Frac) -> Self::Output {
        YawPitchRoll {
            yaw: self.yaw / rhs,
            pitch: self.pitch / rhs,
            roll: self.roll / rhs,
        }
    }
}
impl ops::Mul<Frac> for YawPitchRoll {
    type Output = YawPitchRoll;
    fn mul(self, rhs: Frac) -> Self::Output {
        YawPitchRoll {
            yaw: self.yaw * rhs,
            pitch: self.pitch * rhs,
            roll: self.roll * rhs,
        }
    }
}
#[derive(Clone)]
pub struct ControllerInput {
    pub ypr: YawPitchRoll,
    pub height: Frac,
}
impl ControllerInput {
    pub fn new() -> Self {
        ControllerInput {
            ypr: YawPitchRoll::new(),
            height: Frac::from_num(0),
        }
    }
}
