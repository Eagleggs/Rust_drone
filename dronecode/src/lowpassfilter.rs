// pub fn init_ewma() -> SensorData{
//     let ewma0 = SensorData{
//         height: Frac::from_num(0.0),
//         v_z: Frac::from_num(0.0),
//         pressure: 0,
//         velocity: Velocity{ yaw: Frac::from_num(0.0), pitch: Frac::from_num(0.0), roll: Frac::from_num(0.0) },
//         radius: YawPitchRoll{ yaw: Frac::from_num(0.0), pitch: Frac::from_num(0.0), roll: Frac::from_num(0.0) },
//         acceleration: Accel{ x: 0, y: 0, z: 0},
//         bat: 0,
//         motor_speeds: [0;4],
//     };
//     ewma0
// }

// pub fn find_ewma0(data: [SensorData; 10]) -> SensorData{
//     let num_avg = Frac::from_num(10);
//     let mut ewma_height = Frac::from_num(0.0);
//     let mut ewma_v_z = Frac::from_num(0.0);
//     let mut ewma_pressure = 0;

//     let mut ewma_velocity = Velocity{
//         yaw: Frac::from_num(0.0),
//         pitch: Frac::from_num(0.0),
//         roll: Frac::from_num(0.0)
//     };
//     let mut ewma_vel_yaw = ewma_velocity.yaw;
//     let mut ewma_vel_pitch = ewma_velocity.pitch;
//     let mut ewma_vel_roll = ewma_velocity.roll;

//     let mut ewma_radius = YawPitchRoll{
//         yaw: Frac::from_num(0.0),
//         pitch: Frac::from_num(0.0),
//         roll: Frac::from_num(0.0)
//     };
//     let mut ewma_rad_yaw = ewma_radius.yaw;
//     let mut ewma_rad_pitch = ewma_radius.pitch;
//     let mut ewma_rad_roll = ewma_radius.roll;

//     let mut ewma_acceleration = Accel{ x: 0, y: 0, z: 0 };
//     let ewma_acc_x = ewma_acceleration.x;
//     let ewma_acc_y = ewma_acceleration.y;
//     let ewma_acc_z = ewma_acceleration.z;

//     let mut ewma_bat = 0;
//     let mut ewma_motor_speeds = [0;4];

//     for i in 0..10{
//         ewma_height = ewma_height + data[i].height;
//         ewma_v_z = ewma_v_z + data[i].v_z;
//         ewma_pressure = ewma_pressure + data[i].pressure;

//         ewma_vel_yaw = ewma_vel_yaw + data[i].velocity.yaw;
//         ewma_vel_pitch = ewma_vel_pitch + data[i].velocity.pitch;
//         ewma_vel_roll = ewma_vel_roll + data[i].velocity.roll;

//         ewma_rad_yaw = ewma_rad_yaw + data[i].radius.yaw;
//         ewma_rad_pitch = ewma_rad_pitch + data[i].radius.pitch;
//         ewma_rad_roll = ewma_rad_roll + data[i].radius.roll;

//         ewma_bat = ewma_bat + data[i].bat;
//         for j in 0..4{
//             ewma_motor_speeds[j] = ewma_motor_speeds[j] + data[i].motor_speeds[j];
//         }
//     }

//     ewma_velocity = Velocity{
//         yaw: Frac::from_num(ewma_vel_yaw/num_avg),
//         pitch: Frac::from_num(ewma_vel_pitch/num_avg),
//         roll: Frac::from_num(ewma_vel_roll/num_avg) };
//     ewma_radius = YawPitchRoll{
//         yaw: Frac::from_num(ewma_rad_yaw/num_avg),
//         pitch: Frac::from_num(ewma_rad_pitch/num_avg),
//         roll: Frac::from_num(ewma_rad_roll/num_avg) };
//     ewma_acceleration = Accel{ x: ewma_acc_x/10, y: ewma_acc_y/10, z: ewma_acc_z/10 };
//     for i in 0..4{
//         ewma_motor_speeds[i] = ewma_motor_speeds[i] / 10;
//     }

//     let ewma0 = SensorData {
//         height: ewma_height / num_avg,
//         v_z: ewma_v_z / num_avg,
//         pressure: ewma_pressure / 10,
//         velocity: ewma_velocity,
//         radius: ewma_radius,
//         acceleration: ewma_acceleration,
//         bat: ewma_bat/10,
//         motor_speeds: ewma_motor_speeds
//     };

//     ewma0
// }

// pub fn lpf(data: SensorData, last_ewma: SensorData, alpha_mul: f32) -> SensorData{
//     let alpha = Frac::from_num(alpha_mul);
//     let one_minus_alpha = Frac::from_num(1.0) - alpha;

//     let last_ewma_height = alpha * data.height + one_minus_alpha * last_ewma.height;
//     let last_ewma_v_z = alpha * data.v_z + one_minus_alpha * last_ewma.v_z;
//     let last_ewma_pressure = (alpha_mul * (data.pressure as f32) + (1.0 - alpha_mul) * (last_ewma.pressure as f32)) as i32;

//     let last_ewma_vel_yaw = alpha * data.velocity.yaw + one_minus_alpha * last_ewma.velocity.yaw;
//     let last_ewma_vel_pitch = alpha * data.velocity.pitch + one_minus_alpha * last_ewma.velocity.pitch;
//     let last_ewma_vel_roll = alpha * data.velocity.roll + one_minus_alpha * last_ewma.velocity.roll;

//     let last_ewma_rad_yaw = alpha * data.radius.yaw + one_minus_alpha * last_ewma.radius.yaw;
//     let last_ewma_rad_pitch = alpha * data.radius.pitch + one_minus_alpha * last_ewma.radius.pitch;
//     let last_ewma_rad_roll = alpha * data.radius.roll + one_minus_alpha * last_ewma.radius.roll;

//     let last_ewma_acc_x = (alpha_mul * (data.acceleration.x as f32) + (1.0 - alpha_mul) * (last_ewma.acceleration.x as f32)) as i32;
//     let last_ewma_acc_y = (alpha_mul * (data.acceleration.y as f32) + (1.0 - alpha_mul) * (last_ewma.acceleration.y as f32)) as i32;
//     let last_ewma_acc_z = (alpha_mul * (data.acceleration.z as f32) + (1.0 - alpha_mul) * (last_ewma.acceleration.z as f32)) as i32;

//     let last_ewma_bat = (alpha_mul * (data.bat as f32) + (1.0 - alpha_mul) * (last_ewma.bat as f32)) as u16;
//     let mut last_ewma_motor_sp: [u16; 4] = [0; 4];
//     for i in 0..4{
//         last_ewma_motor_sp[i] = (alpha_mul * (data.motor_speeds[i] as f32) + (1.0 - alpha_mul) * (last_ewma.motor_speeds[i] as f32)) as u16;
//     }

//     let last_ewma_vel = Velocity{
//         yaw: last_ewma_vel_yaw,
//         pitch: last_ewma_vel_pitch,
//         roll: last_ewma_vel_roll,
//     };

//     let last_ewma_rad = YawPitchRoll{
//         yaw: last_ewma_rad_yaw,
//         pitch: last_ewma_rad_pitch,
//         roll: last_ewma_rad_roll,
//     };

//     let last_ewma_acc = Accel{
//         x: last_ewma_acc_x,
//         y: last_ewma_acc_y,
//         z: last_ewma_acc_z,
//     };

//     let lpf_sensordata = SensorData{
//         height: last_ewma_height,
//         v_z: last_ewma_v_z,
//         pressure: last_ewma_pressure,
//         velocity: last_ewma_vel,
//         radius: last_ewma_rad,
//         acceleration: last_ewma_acc,
//         bat: last_ewma_bat,
//         motor_speeds: last_ewma_motor_sp,
//     };

//     lpf_sensordata

// }

// /*
// pub fn find_ewma0(data: &mut [f32]) -> f32{
//     let mut ewma0: f32 = 0.0;
//     for i in 0..10{
//         ewma0 = ewma0 + data[i];
//     }
//     ewma0 = ewma0 / 10.0;
//     ewma0
// }

// pub fn lpf(data: f32, last_ewma: f32, alpha: f32) -> f32 {
//     // let mut a: [f32; 100] = [0.0; 100];
//     let data_as_f32: f32 = data as f32;
//     let mut new_last_ewma: f32 = last_ewma;

//     new_last_ewma = alpha * data_as_f32 + (1.0 - alpha) * new_last_ewma;
//     new_last_ewma
// }
//  */
