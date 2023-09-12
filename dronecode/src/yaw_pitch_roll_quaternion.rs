use architecture::{Accel, Frac, YawPitchRoll};

use fixed_trigonometry::*;
use tudelft_quadrupel::mpu::structs::Quaternion;

//impl From<Quaternion> for YawPitchRoll {
/// Creates a YawPitchRoll from a Quaternion
pub fn yaw_pitch_roll_from_quaternion(q: Quaternion) -> YawPitchRoll {
    let Quaternion { w, x, y, z } = q;

    let one = Frac::from_num(1);
    let two = Frac::from_num(2);
    let w = Frac::from_num(w);
    let x = Frac::from_num(x);
    let y = Frac::from_num(y);
    let z = Frac::from_num(z);

    let gx = two * (x * z - w * y);
    let gy = two * (w * x + y * z);
    let gz = w * w - x * x - y * y + z * z;

    let yaw = atan::atan2(two * x * y - two * w * z, two * w * w + two * x * x - one);
    let pitch = atan::atan2(gx, sqrt::niirf(gy * gy + gz * gz, 2));
    let roll = atan::atan2(gy, gz);

    //---------------------------------------
    /*let Quaternion { w, x, y, z } = q;

    let w: f32 = w.to_num();
    let x: f32 = x.to_num();
    let y: f32 = y.to_num();
    let z: f32 = z.to_num();

    let gx = 2.0 * (x * z - w * y);
    let gy = 2.0 * (w * x + y * z);
    let gz = w * w - x * x - y * y + z * z;

    // todo: trigonometry functions in fixed point

    // yaw: (about Z axis)
    let yaw = Frac::from_num(
        micromath::F32Ext::atan2(2.0 * x * y - 2.0 * w * z, 2.0 * w * w + 2.0 * x * x - 1.0)
    );

    // pitch: (nose up/down, about Y axis)
    let pitch = Frac::from_num(
        micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz))
    );

    // roll: (tilt left/right, about X axis)
    let roll = Frac::from_num(
        micromath::F32Ext::atan2(gy, gz)
    );*/

    YawPitchRoll { yaw, pitch, roll }
}
//}
pub fn yaw_pitch_roll_from_acc(acc: Accel) -> YawPitchRoll {
    let mut out = YawPitchRoll::new();
    let x = acc.x as f32;
    let y = acc.y as f32;
    let z = acc.z as f32;

    // todo: trigonometry functions in fixed point

    out.roll = Frac::from_num(micromath::F32Ext::atan(
        y / micromath::F32Ext::sqrt(x * x + z * z),
    ));
    out.pitch = Frac::from_num(-micromath::F32Ext::atan(
        -x / micromath::F32Ext::sqrt(y * y + z * z),
    ));
    out
}
