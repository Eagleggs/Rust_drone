use architecture::{Message, SensorData};
use protocol::{DataLink, FuncLink, MessageLink};
use tudelft_quadrupel::{
    barometer::read_pressure,
    time::{set_tick_frequency, wait_for_next_tick},
    uart::{receive_bytes, send_bytes},
};

pub fn base_station_loop() -> ! {
    // initialization
    let link = FuncLink::from_func(send_bytes, receive_bytes);
    let mut link = MessageLink::new(link);

    set_tick_frequency(100);

    let mut data = SensorData::new();
    loop {
        data.pressure = read_pressure() as f32;

        link.send(&Message::SensorData { data: data.clone() })
            .unwrap();
        wait_for_next_tick();
    }
}
