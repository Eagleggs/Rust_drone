#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

#[macro_use]
extern crate enum_map;

extern crate alloc;
extern crate architecture;
extern crate log;
use alloc::format;
use architecture::BASE_STATION;
use base_station::base_station_loop;
use core::alloc::Layout;
use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use tudelft_quadrupel::initialize::initialize;
use tudelft_quadrupel::led::Led::{Green, Red};
use tudelft_quadrupel::time::assembly_delay;
use tudelft_quadrupel::uart::send_bytes;
use tudelft_quadrupel::{entry, uart};

mod base_station;
mod control;
mod control_loop;
mod funcdisk;
mod kalman_filter;
mod liveness;
mod lowpassfilter;
mod message;
mod profiling;
mod sensor;
mod state_machine;
mod yaw_pitch_roll_quaternion;

/// The heap size of your drone code in bytes.
/// Note: there are 8192 bytes of RAM available.
const HEAP_SIZE: usize = 4096;

#[entry]
fn main() -> ! {
    {
        static mut HEAP_MEMORY: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

        // SAFETY: HEAP_MEMORY is a local static. That means, that at
        // the end of the surrounding block scope (not the main function, the local scope)
        // it is impossible to use HEAP_MEMORY *except* through this mutable reference
        // created here. Since that means that there's only one reference to HEAP_MEMORY,
        // this is safe.
        //
        // As soon as the first driver (led driver) is initialized, the yellow led turns on.
        // That's also the last thing that's turned off. If the yellow led stays on and your
        // program doesn't run, you know that the boot procedure has failed.
        initialize(unsafe { &mut HEAP_MEMORY }, true);
    }

    if BASE_STATION {
        base_station_loop();
    } else {
        profiling::init();
        control_loop::control_loop()
    }
}

#[inline(never)]
#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // On panic:
    // * try and write the panic message on UART
    // * blink the red light

    use architecture::Message;
    use protocol::{DataLink, FuncLink, MessageLink};
    use tudelft_quadrupel::uart::receive_bytes;

    if uart::is_initialized() {
        let link = FuncLink::from_func(send_bytes, receive_bytes);
        let mut link = MessageLink::new(link);
        let m = format!("{info}\n");
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

    // Start blinking red
    loop {
        let _ = Red.toggle();
        assembly_delay(1_000_000)
    }
}

#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    // When an allocation error happens, we panic.
    // However, we do not want to have UART initialized, since
    // the panic handler uses the allocator to print a message over
    // UART. However, if UART is not initialized, it won't attempt
    // to allocate the message.
    //
    // instead, to signal this, we turn the green light on too
    // (together with blinking red of the panic)
    Green.on();

    // Safety: after this we panic and go into an infinite loop
    unsafe { uart::uninitialize() };

    panic!("out of memory: {layout:?}");
}
