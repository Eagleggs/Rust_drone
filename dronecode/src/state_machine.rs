use crate::control::Controller;
use architecture::Mode;

pub fn check_state(_controller: &mut Controller, _mode: Mode) -> bool {
    let current_mode = _controller.mode;

    match current_mode {
        Mode::Safe => {
            if _mode == Mode::Panic {
                return false; // drone enters safe mode automatically once it enters panic mode
            } else {
                return true; // drone should be able to enter other modes except panic when it is in safe mode
            }
        }
        Mode::Panic => {
            // once in panic, it should only enter safe mode
            if _mode == Mode::Safe {
                return true;
            } else {
                return false;
            }
        }
        // For all other modes: the drone can enter only safe or panic state
        Mode::Manual => {
            if _mode == Mode::Safe || _mode == Mode::Panic {
                return true;
            }
        }
        Mode::Calibrate => {
            if _mode == Mode::Safe || _mode == Mode::Panic {
                return true;
            }
        }
        Mode::YawControl => {
            if _mode == Mode::Safe || _mode == Mode::Panic {
                return true;
            }
        }
        Mode::FullControl => {
            if _mode == Mode::Safe || _mode == Mode::Panic {
                return true;
            }
        }
        Mode::Raw => {
            if _mode == Mode::Safe || _mode == Mode::Panic {
                return true;
            }
        }
        Mode::Height => {
            if _mode == Mode::Safe || _mode == Mode::Panic {
                return true;
            }
        }
        Mode::WireLess => {
            if _mode == Mode::Safe || _mode == Mode::Panic {
                return true;
            }
        }
    };
    return false;
}
