use crate::sensor::Sensor;
use alloc::vec;
use alloc::vec::Vec;
use architecture::Mode::{Calibrate, Panic, Safe};
use architecture::YawPitchRoll;
use architecture::*;
use protocol::{DataLink, MessageLink};
use tudelft_quadrupel::led::Led::Red;
use tudelft_quadrupel::motor::{get_motors, set_motors};

use tudelft_quadrupel::time::assembly_delay;

pub struct Controller {
    pub p: Frac,
    pub i: Frac,
    pub d: Frac,
    pub input: ControllerInput,
    pub output: [u16; 4],
    pub cache: Vec<ControllerInput>,
    pub mode: Mode,
    pub raw_option: bool,
    pub frequency: u64,
}

impl Controller {
    pub fn control_algo<T: protocol::Link>(
        &mut self,
        data: &mut ControlRequest,
        sensor: &mut Sensor,
        link: &mut MessageLink<T>,
    ) -> [u16; 4] {
        let lift = data.throttle / 10;
        let mut output = [0, 0, 0, 0]; //default value when powered
        if data.throttle < 1000 && self.mode != Safe && self.mode != Panic && self.mode != Calibrate
        {
            for i in 0..4 {
                output[i] = 0;
            }
        } else {
            match self.mode {
                Mode::FullControl => {
                    let yaw_diff = self.input.ypr.yaw;
                    let pitch_diff = self.input.ypr.pitch;
                    let roll_diff = self.input.ypr.roll;
                    // let mut yaw_derivative: Frac = Frac::from_num(0.);
                    let mut pitch_derivative: Frac = Frac::from_num(0.);
                    let mut roll_derivative: Frac = Frac::from_num(0.);
                    if self.cache.len() > 1 {
                        // yaw_derivative = yaw_diff - self.cache[self.cache.len() - 2].ypr.yaw;
                        pitch_derivative = pitch_diff - self.cache[self.cache.len() - 2].ypr.pitch;
                        roll_derivative = roll_diff - self.cache[self.cache.len() - 2].ypr.roll;
                    }
                    let mut yaw_acc: Frac = Frac::from_num(0.);
                    let mut pitch_acc: Frac = Frac::from_num(0.);
                    let mut roll_acc: Frac = Frac::from_num(0.);
                    for ypr in &self.cache {
                        yaw_acc += ypr.ypr.yaw;
                        pitch_acc += ypr.ypr.pitch;
                        roll_acc += ypr.ypr.roll;
                    }

                    let yaw_output: i16 = (yaw_diff * self.p / Frac::from_num(4)).to_num();
                    let pitch_output: i16 =
                        (pitch_diff * self.p + pitch_derivative * self.d + pitch_acc * self.i)
                            .to_num();
                    let roll_output: i16 =
                        (roll_diff * self.p + roll_derivative * self.d + roll_acc * self.i)
                            .to_num();
                    //motor 0: front motor 1 right motor 2 back motor 3 left
                    let max_c = 300;
                    let max = 800;
                    let min = 180;
                    if data.throttle < 50 {
                        for i in 0..4 {
                            output[i] = 0;
                        }
                    } else {
                        output[3] = ((((0 as i16)
                            .saturating_add(roll_output - yaw_output)
                            .clamp(-max_c, max_c))
                            + lift)
                            .clamp(min, max)) as u16;
                        output[1] = ((((0 as i16)
                            .saturating_sub(roll_output + yaw_output)
                            .clamp(-max_c, max_c))
                            + lift)
                            .clamp(min, max)) as u16;

                        output[0] = ((((0 as i16)
                            .saturating_add(pitch_output + yaw_output)
                            .clamp(-max_c, max_c))
                            + lift)
                            .clamp(min, max)) as u16;
                        output[2] = ((((0 as i16)
                            .saturating_sub(pitch_output - yaw_output)
                            .clamp(-max_c, max_c))
                            + lift)
                            .clamp(min, max)) as u16;
                    }
                }
                Mode::Manual => {
                    let min = 200;
                    let max = 400;
                    let scale: Frac = Frac::from_num(100);
                    output[1] = (((lift) as i16)
                        .saturating_sub(((data.radius.roll + data.radius.yaw) * scale).to_num())
                        .clamp(min, max)) as u16;
                    output[3] = (((lift) as i16)
                        .saturating_add(((data.radius.roll - data.radius.yaw) * scale).to_num())
                        .clamp(min, max)) as u16;

                    output[2] = (((lift) as i16)
                        .saturating_sub(((data.radius.pitch - data.radius.yaw) * scale).to_num())
                        .clamp(min, max)) as u16;
                    output[0] = (((lift) as i16)
                        .saturating_add(((data.radius.pitch + data.radius.yaw) * scale).to_num())
                        .clamp(min, max)) as u16;
                }
                Mode::Panic => {
                    let temp = 2;
                    for _i in (0..200).rev() {
                        let motors_val = get_motors();
                        set_motors([
                            (motors_val[0].saturating_sub(temp)).clamp(0, 400),
                            (motors_val[1].saturating_sub(temp)).clamp(0, 400),
                            (motors_val[2].saturating_sub(temp)).clamp(0, 400),
                            (motors_val[3].saturating_sub(temp)).clamp(0, 400),
                        ]);
                        assembly_delay(100000);
                    }
                    self.mode = Mode::Safe;
                    Red.off();
                    link.send(&Message::ChangeMode { mode: Mode::Safe })
                        .unwrap();
                    for i in 0..4 {
                        output[i] = 0;
                    }
                }
                Mode::Safe => {
                    for i in 0..4 {
                        output[i] = 0;
                    }
                }
                Mode::Calibrate => {
                    for i in 0..4 {
                        output[i] = 0;
                    }
                    sensor.calibrate();
                }
                Mode::YawControl => {
                    let max_c = 200;
                    let max = 500;
                    let min = 200;

                    let yaw_diff = self.input.ypr.yaw;
                    let yaw_output: i16 = (yaw_diff * self.p / Frac::from_num(8)).to_num();
                    let scale: Frac = Frac::from_num(80);
                    output[3] = ((((0 as i16)
                        .saturating_add((data.radius.roll * scale).to_num::<i16>() - yaw_output)
                        .clamp(-max_c, max_c))
                        + lift)
                        .clamp(min, max)) as u16;
                    output[1] = ((((0 as i16)
                        .saturating_sub((data.radius.roll * scale).to_num::<i16>() + yaw_output)
                        .clamp(-max_c, max_c))
                        + lift)
                        .clamp(min, max)) as u16;

                    output[0] = ((((0 as i16)
                        .saturating_add((data.radius.pitch * scale).to_num::<i16>() + yaw_output)
                        .clamp(-max_c, max_c))
                        + lift)
                        .clamp(min, max)) as u16;
                    output[2] = ((((0 as i16)
                        .saturating_sub((data.radius.pitch * scale).to_num::<i16>() - yaw_output)
                        .clamp(-max_c, max_c))
                        + lift)
                        .clamp(min, max)) as u16;
                }
                Mode::Height => {
                    let height_diff = self.input.height;
                    let yaw_diff = self.input.ypr.yaw;
                    let pitch_diff = self.input.ypr.pitch;
                    let roll_diff = self.input.ypr.roll;
                    let mut height_derivative = Frac::from_num(0);
                    // let mut yaw_derivative = Frac::from_num(0);
                    let mut pitch_derivative = Frac::from_num(0);
                    let mut roll_derivative = Frac::from_num(0);
                    if self.cache.len() > 1 {
                        height_derivative = height_diff - self.cache[self.cache.len() - 2].height;
                        // yaw_derivative = yaw_diff - self.cache[self.cache.len() - 2].ypr.yaw;
                        pitch_derivative = pitch_diff - self.cache[self.cache.len() - 2].ypr.pitch;
                        roll_derivative = roll_diff - self.cache[self.cache.len() - 2].ypr.roll;
                    }
                    let mut height_acc = Frac::from_num(0);
                    let mut yaw_acc = Frac::from_num(0);
                    let mut pitch_acc = Frac::from_num(0);
                    let mut roll_acc = Frac::from_num(0);
                    for ypr in &self.cache {
                        height_acc += ypr.height;
                        yaw_acc += ypr.ypr.yaw;
                        pitch_acc += ypr.ypr.pitch;
                        roll_acc += ypr.ypr.roll;
                    }

                    let yaw_output: i16 = (yaw_diff * self.p / Frac::from_num(8)).to_num();
                    let height_output: i16 =
                        (height_diff * self.p + height_derivative * self.d + height_acc * self.i)
                            .to_num::<i16>();
                    let pitch_output: i16 =
                        (pitch_diff * self.p + pitch_derivative * self.d + pitch_acc * self.i)
                            .to_num();
                    let roll_output: i16 =
                        (roll_diff * self.p + roll_derivative * self.d + roll_acc * self.i)
                            .to_num();
                    //motor 0: front motor 1 right motor 2 back motor 3 left
                    let max_c = 100;
                    let max = 600;
                    let min = 150;
                    let hover_lift = 300; // this is the lift for drone to hover without any disturb, may need tuning
                    if data.throttle < 50 {
                        for i in 0..4 {
                            output[i] = 0;
                        }
                    } else {
                        output[3] = ((((0 as i16)
                            .saturating_add(height_output + roll_output - yaw_output)
                            .clamp(-max_c, max_c))
                            + hover_lift)
                            .clamp(min, max)) as u16;
                        output[1] = ((((0 as i16)
                            .saturating_sub(roll_output + yaw_output - height_output)
                            .clamp(-max_c, max_c))
                            + hover_lift)
                            .clamp(min, max)) as u16;

                        output[0] = ((((0 as i16)
                            .saturating_add(pitch_output + yaw_output + height_output)
                            .clamp(-max_c, max_c))
                            + hover_lift)
                            .clamp(min, max)) as u16;
                        output[2] = ((((0 as i16)
                            .saturating_sub(pitch_output - yaw_output - height_output)
                            .clamp(-max_c, max_c))
                            + hover_lift)
                            .clamp(min, max)) as u16;
                    }
                }
                _ => {}
            }
        }
        return output;
    }
    pub fn new() -> Self {
        Controller {
            p: Frac::from_num(0),
            i: Frac::from_num(0),
            d: Frac::from_num(0),
            input: ControllerInput {
                ypr: YawPitchRoll::new(),
                height: Frac::from_num(0),
            },
            output: [0, 0, 0, 0],
            cache: vec![],
            mode: Mode::Safe,
            raw_option: false,
            frequency: 150,
        }
    }
    pub fn set_parameters(&mut self, p: Frac, i: Frac, d: Frac) {
        self.p = p;
        self.i = i;
        self.d = d;
    }
    pub fn calculate_difference(&mut self, sensor: &Sensor, request: &ControlRequest) {
        self.input.ypr = request.radius - sensor.data.radius;
        self.input.ypr.yaw = request.radius.yaw - sensor.data.velocity.yaw;
        self.input.height = Frac::from_num(request.throttle / 400) - sensor.data.height;
        //dead zone
        let pitch_roll_deadzone = 0.; //rad
        let yaw_deadzone = 0.6; //rad/s
        let height_deadzone = 0.1; // m
        if self.input.ypr.yaw < yaw_deadzone && self.input.ypr.yaw > -yaw_deadzone {
            self.input.ypr.yaw = Frac::from_num(0);
        }
        if self.input.ypr.pitch < pitch_roll_deadzone && self.input.ypr.pitch > -pitch_roll_deadzone
        {
            self.input.ypr.pitch = Frac::from_num(0);
        }
        if self.input.ypr.roll < pitch_roll_deadzone && self.input.ypr.roll > -pitch_roll_deadzone {
            self.input.ypr.roll = Frac::from_num(0);
        }
        if self.input.ypr.roll < pitch_roll_deadzone && self.input.ypr.roll > -pitch_roll_deadzone {
            self.input.ypr.roll = Frac::from_num(0);
        }
        if self.input.height < height_deadzone && self.input.height > -height_deadzone {
            self.input.height = Frac::from_num(0);
        }
    }
}
pub fn enqueue<T>(v: &mut Vec<T>, value: T) {
    if v.len() < 10 {
        v.push(value)
    } else {
        v.remove(0);
        v.push(value)
    }
}

// //motor 0+,1-: roll control motor 2+,3-:pitch control motor:1,2,3,4 yaw control(due to drift of gyroscope, commented)
// pub fn PID_Control(P:f32, I:f32, D:f32, ypr_dif:&YawPitchRoll, accumulation:&mut Vec<YawPitchRoll>, motors: [u16; 4]) ->[u16;4]
// {
//     let yaw_diff = ypr_dif.yaw;
//     let pitch_diff = ypr_dif.pitch;
//     let roll_diff = ypr_dif.roll;
//     let mut yaw_derivative = 0.;
//     let mut pitch_derivative =0.;
//     let mut roll_derivative =0.;
//     if accumulation.len()>1{
//          yaw_derivative = (yaw_diff - accumulation[accumulation.len()-2].yaw);
//          pitch_derivative =(pitch_diff - accumulation[accumulation.len()-2].pitch);
//          roll_derivative =(roll_diff - accumulation[accumulation.len()-2].roll);
//     }
//
//
//     let mut yaw_acc = 0.;
//     let mut pitch_acc = 0.;
//     let mut roll_acc =0.;
//     for ypr in accumulation{
//         yaw_acc +=ypr.yaw;
//         pitch_acc+=ypr.pitch;
//         roll_acc+=ypr.roll;
//     }
//
//     let yaw_output = (yaw_diff*P + yaw_derivative*D + yaw_acc*I) as i16;
//     let pitch_output = (pitch_diff*P + pitch_derivative*D + pitch_acc*I) as i16;
//     let roll_output = (roll_diff*P + roll_derivative*D + roll_acc*I) as i16;
//     let mut output = motors;
//
//     output[0]=(((0 as i16).saturating_add(roll_output).clamp(-200,200)) +200)as u16;
//     output[1]=(((0 as i16).saturating_sub(roll_output).clamp(-200,200)) + 200) as u16;
//
//     output[2]=(((0 as i16).saturating_add(pitch_output).clamp(-200,200)) +200)as u16;
//     output[3]=(((0 as i16).saturating_sub(pitch_output).clamp(-200,200)) +200) as u16;
//
//     // output[0]=((output[0] as i16).saturating_add(yaw_output).clamp(0,400)) as u16;
//     // output[1]=((output[1] as i16).saturating_add(yaw_output).clamp(0,400)) as u16;
//     // output[2]=((output[2] as i16).saturating_sub(yaw_output).clamp(0,400)) as u16;
//     // output[3]=((output[3] as i16).saturating_sub(yaw_output).clamp(0,400)) as u16;
//     //send_bytes(format!("pitch_Ioutput {}\n",pitch_derivative*D).as_bytes());
//     return output
//
//
// }
