use crate::funcdisk::FuncDisk;
use crate::sensor::Sensor;
use crate::state_machine::check_state;
use crate::{control::Controller, liveness::Liveliness};
use architecture::Mode::{Panic, Raw};
use architecture::{ControlRequest, Frac, Message, Mode};
use log::Logger;
use protocol::{DataLink, MessageLink};
use tudelft_quadrupel::led::Led::Green;
use tudelft_quadrupel::time::{delay_ms_assembly, set_tick_frequency};

pub fn handle_message<T: protocol::Link>(
    liveliness: &mut Liveliness,
    link: &mut MessageLink<T>,
    logger: &mut Logger<FuncDisk>,
    controller: &mut Controller,
    control_request: &mut ControlRequest,
    sensor: &mut Sensor,
) {
    let msg = link.check_for_message();
    Green.on();
    match msg {
        Ok(msg) => match msg {
            Some(msg) => match msg {
                Message::ChangeMode { mode } => match link.send(&Message::ChangeMode { mode }) {
                    Ok(_) => {
                        if mode == Raw {
                            controller.raw_option = true;
                            controller.frequency = 350;
                            controller.set_parameters(
                                Frac::from_num(100),
                                Frac::from_num(10),
                                Frac::from_num(2300),
                            );
                            set_tick_frequency(controller.frequency);
                        } else {
                            controller.mode = mode;
                            if check_state(controller, mode) {
                                controller.mode = mode;
                            }
                        }
                    }
                    Err(_) => controller.mode = Panic,
                },
                Message::ControlInput {
                    request,
                    base_pressure,
                } => {
                    control_request.radius = request.radius;
                    control_request.throttle = request.throttle;
                    liveliness.notify_alive();
                    sensor.base_pressure = base_pressure;
                }
                Message::TuneParameter { parameter, value } => match parameter {
                    'p' => {
                        controller.p = value;
                    }
                    'i' => {
                        controller.i = value;
                    }
                    'd' => {
                        controller.d = value;
                    }
                    _ => {
                        match link.send(&Message::LogMessage {
                            message: <[u8; 16]>::try_from("no para".as_bytes()).unwrap(),
                        }) {
                            Ok(_) => {}
                            Err(_) => {}
                        }
                    }
                },
                Message::SensorData { .. } => (),
                Message::LogMessage { .. } => (),
                Message::LoggerMode { mode } => match mode {
                    architecture::LoggerMode::Enabled => {
                        logger.set_enabled(true);
                        logger.clear().unwrap();
                    }
                    architecture::LoggerMode::Disabled => logger.set_enabled(false),
                    architecture::LoggerMode::Download => {
                        if controller.mode == Mode::Safe {
                            let _n = logger.length().unwrap() as u32;
                            let mut address = 0;
                            for _ in 0..logger.nof_entries {
                                Green.off();
                                let data = logger.get_entry(address).unwrap();
                                match data {
                                    Some((data, size)) => {
                                        address += size as u32;
                                        link.send(&Message::LogDownload { entry: Some(data) })
                                            .unwrap();
                                        delay_ms_assembly(20);
                                    }
                                    None => unreachable!("corrupted data"),
                                }
                                Green.on();
                            }
                            link.send(&Message::LogDownload { entry: None }).unwrap();
                        }
                    }
                },
                Message::LogDownload { .. } => {
                    unreachable!("pc should not send log download events")
                }
                Message::ProfilerEvent(_) => unreachable!("pc should not send profiler events"),
                Message::ProfilerTimed { .. } => unreachable!("pc should not send profiler events"),
            },
            None => (),
        },
        Err(_) => (),
    }
    Green.off();
}
