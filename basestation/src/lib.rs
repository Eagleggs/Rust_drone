use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
    thread::{self, sleep, JoinHandle},
    time::Duration,
};

use protocol::{DataLink, Link, MessageLink};
use single_value_channel::{channel_starting_with, Receiver};

pub fn start<T: Link>(mut message_link: MessageLink<T>) -> ! {
    let mut stream = TcpStream::connect("localhost:8080").unwrap();

    let alpha: f32 = 0.015;
    let mut last: f32 = 100000.0;
    let mut i = 0;
    loop {
        match message_link.check_for_message() {
            Ok(msg) => match msg {
                Some(msg) => match msg {
                    architecture::Message::SensorData { data } => {
                        last = data.pressure * alpha + last * (1.0 - alpha);
                        stream.write_all(&last.to_le_bytes()).unwrap();
                    }
                    _ => (),
                },
                None => (),
            },
            Err(_) => (),
        }
        sleep(Duration::from_millis(10));
        if i >= 10 {
            i -= 10;
            println!("pressure: {}", last);
        }
        i += 1;
    }
}

pub fn start_receiver() -> (Receiver<f32>, JoinHandle<()>) {
    let (receiver, updater) = channel_starting_with(100000.0f32);
    let handle = thread::spawn(move || {
        let listener = TcpListener::bind("localhost:8080").unwrap();
        for stream in listener.incoming() {
            let mut buf = [0u8; 4];
            let mut filled = 0;
            match stream {
                Ok(mut stream) => {
                    for _ in 0..8 {
                        println!("Base Station connected!");
                    }
                    loop {
                        match stream.read(&mut buf[filled..]) {
                            Ok(n) => {
                                if n == 0 {
                                    break;
                                }
                                filled += n;
                                if filled == 4 {
                                    filled = 0;
                                    let pressure = f32::from_le_bytes(buf.clone());
                                    updater.update(pressure).unwrap();
                                }
                            }
                            Err(_) => break,
                        }
                    }
                    for _ in 0..8 {
                        println!("Base Station disconnected!");
                    }
                }
                Err(e) => {
                    eprintln!("Error: {}", e);
                }
            }
        }
    });

    (receiver, handle)
}
