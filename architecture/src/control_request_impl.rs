use crate::{ControlRequest, YawPitchRoll};
use core::ops;

impl ops::Add<ControlRequest> for ControlRequest {
    type Output = ControlRequest;

    fn add(self, rhs: ControlRequest) -> Self::Output {
        ControlRequest {
            radius: YawPitchRoll {
                yaw: self.radius.yaw + rhs.radius.yaw,
                pitch: self.radius.pitch + rhs.radius.pitch,
                roll: self.radius.roll + rhs.radius.roll,
            },
            throttle: self.throttle + rhs.throttle,
        }
    }
}
