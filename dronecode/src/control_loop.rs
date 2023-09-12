use crate::funcdisk::FuncDisk;
use crate::liveness::Liveliness;
use crate::message::handle_message;
use crate::profiling::macros::{profiler_event, profiler_event_if};

use crate::{
    control::{enqueue, Controller},
    sensor::Sensor,
};

use alloc::borrow::ToOwned;
use alloc::string::String;
use architecture::Mode::Panic;
use architecture::{ControlRequest, Frac, Message, Mode, ProfilerEvent, SensorDriver};
use log::Logger;
use protocol::{DataLink, FuncLink, MessageLink};
use tudelft_quadrupel::motor::set_motor_max;
use tudelft_quadrupel::{
    led::Led::{Blue, Green, Red},
    motor::set_motors,
    time::{set_tick_frequency, wait_for_next_tick},
    uart::{receive_bytes, send_bytes},
};

use crate::kalman_filter::KalmanFilter;
use tudelft_quadrupel::flash::flash_write_byte;
use tudelft_quadrupel::flash::flash_write_bytes;
use tudelft_quadrupel::flash::{flash_chip_erase, flash_read_byte, flash_read_bytes};

pub fn print(link: &mut MessageLink<FuncLink>, m: String) {
    let mut msg = m.as_bytes();
    while msg.len() > 0 {
        let mut msg_buf = [0; 16];
        if msg.len() > 16 {
            msg_buf.copy_from_slice(&msg[..16]);
            msg = &msg[16..];
        } else {
            msg_buf[..msg.len()].copy_from_slice(msg);
            msg = &[];
        }
        link.send(&Message::LogMessage { message: msg_buf })
            .unwrap();
    }
}

pub fn control_loop() -> ! {
    // initialization
    let link = FuncLink::from_func(send_bytes, receive_bytes);
    let mut link = MessageLink::new(link);
    let mut liveness = Liveliness::new(120);

    let disk = FuncDisk::func(
        flash_write_bytes,
        flash_read_bytes,
        flash_chip_erase,
        flash_write_byte,
        flash_read_byte,
    );
    let mut logger = Logger::new(disk);

    let mut karman_filter = KalmanFilter::new(Frac::from_num(4), Frac::from_num(5000.));
    let mut controller = Controller::new();
    let mut sensor = Sensor::new();
    let mut control_request = ControlRequest::new();
    set_tick_frequency(controller.frequency);
    controller.set_parameters(
        Frac::from_num(100),
        Frac::from_num(10),
        Frac::from_num(3000),
    );
    set_motor_max(800);
    // Check sensors
    // Send sensor data
    // Check and handle messages
    // Control algorithm
    for i in 0.. {
        profiler_event!(link, ProfilerEvent::MainLoopStart);
        profiler_event_if!(
            controller.mode == Mode::FullControl,
            link,
            ProfilerEvent::MainLoopFullControlStart
        );
        // let dt = now.duration_since(last);
        karman_filter.integration_constant = Frac::from_num(1.0 / controller.frequency as f32);
        match liveness.tick() {
            Some(_) => {
                if controller.mode != Mode::Safe {
                    controller.mode = Mode::Panic;

                    Red.on();
                }
            }
            None => (),
        }

        sensor.get_values(true, controller.raw_option);
        if logger.get_enabled() {
            logger.append(&sensor.data).unwrap();
        }
        if sensor.calibrated {
            enqueue(&mut sensor.fir_cache, sensor.data);
            if sensor.fir_cache.len() > 3 {
                //  sensor.filter_FIR(Frac::from_num(0.2),Frac::from_num(0.2),Frac::from_num(0.6))
            }
            sensor.filter_ewma(Frac::from_num(0.1), 0.015);
            sensor.calculate_height(Frac::from_num(1.0 / controller.frequency as f32));
            if controller.raw_option {
                karman_filter.fusion_algorithm(&mut sensor);
            }
        }
        if sensor.data.bat != 0 && sensor.data.bat <= 910 {
            // 9.1 V minimum safe level
            controller.mode = Panic;
            // change_mode(&mut controller, Panic, &mut link); // New function
            let message = Message::ChangeMode { mode: Panic };
            link.send(&message).unwrap();
            print(&mut link, "Battery P".to_owned());
        }

        if i % 20 == 0 {
            let _ = Blue.toggle();
        }
        if i % 40 == 0 || (!controller.raw_option && i % 20 == 0) {
            sensor.send_data(&mut link);
        }
        handle_message(
            &mut liveness,
            &mut link,
            &mut logger,
            &mut controller,
            &mut control_request,
            &mut sensor,
        );

        controller.calculate_difference(&mut sensor, &mut control_request);
        enqueue(&mut controller.cache, controller.input.clone());
        if controller.raw_option == true {
            Green.on();
        } else {
            Green.off();
        }
        set_motors(controller.control_algo(&mut control_request, &mut sensor, &mut link));
        profiler_event!(link, ProfilerEvent::MainLoopStop);
        profiler_event_if!(
            controller.mode == Mode::FullControl,
            link,
            ProfilerEvent::MainLoopFullControlStop
        );

        wait_for_next_tick();
    }
    unreachable!();
}
