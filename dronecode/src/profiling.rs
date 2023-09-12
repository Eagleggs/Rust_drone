use tudelft_quadrupel::time::Instant;

use enum_map::EnumMap;

use architecture::ProfilerEvent;
use protocol::{DataLink, FuncLink, MessageLink};

pub const PROFILING_ENABLED: bool = false;

// event based profiling config
pub const SAMPLING_RATE: usize = 40;
static mut TRIGGER_NUMBER: Option<EnumMap<ProfilerEvent, usize>> = None;

// in-controller profiling config
pub const SEND_ALL: bool = false;
pub const SEND_AT_COUNT: u16 = 120;
pub const SCOPE_PAIRS: [(ProfilerEvent, ProfilerEvent); 2] = [
    (ProfilerEvent::MainLoopStart, ProfilerEvent::MainLoopStop),
    (
        ProfilerEvent::MainLoopFullControlStart,
        ProfilerEvent::MainLoopFullControlStop,
    ),
];
static mut SCOPE_COUNT: [u16; 2] = [0, 0];
static mut SCOPE_SUM: [u64; 2] = [0, 0];
static mut SCOPE_LAST: [Option<u64>; 2] = [None, None];

pub fn init() {
    unsafe {
        if PROFILING_ENABLED && SEND_ALL {
            TRIGGER_NUMBER = Some(enum_map! {
                _ => 0,
            });
        }
    }
}

pub mod macros {
    // This is a macro so that when PROFILING_ENABLED is false
    // then the compiler will remove the code
    macro_rules! profiler_event {
        ($link:ident, $event:expr) => {
            if crate::profiling::PROFILING_ENABLED {
                crate::profiling::trigger_profiling_event(&mut $link, $event);
            }
        };
    }

    macro_rules! profiler_event_if {
        ($condition:expr, $link:ident, $event:expr) => {
            if $condition && crate::profiling::PROFILING_ENABLED {
                crate::profiling::trigger_profiling_event(&mut $link, $event);
            }
        };
    }

    pub(crate) use profiler_event;
    pub(crate) use profiler_event_if;
}

pub fn trigger_profiling_event(link: &mut MessageLink<FuncLink>, event: ProfilerEvent) {
    if SEND_ALL {
        send_profiling_event(link, event);
    } else {
        count_profiling_event(link, event);
    }
}

pub fn count_profiling_event(link: &mut MessageLink<FuncLink>, event: ProfilerEvent) {
    let now = Instant::now();
    unsafe {
        let ns = now.ns_since_start();
        for (i, (start, stop)) in SCOPE_PAIRS.iter().enumerate() {
            if *start == event {
                SCOPE_LAST[i] = Some(ns);
            } else if *stop == event {
                if let Some(last) = SCOPE_LAST[i] {
                    SCOPE_SUM[i] += ns - last;
                    SCOPE_COUNT[i] += 1;
                }

                if SCOPE_COUNT[i] == SEND_AT_COUNT {
                    link.send(&architecture::Message::ProfilerTimed {
                        start: *start,
                        stop: *stop,
                        count: SCOPE_COUNT[i],
                        ns: SCOPE_SUM[i],
                    })
                    .unwrap();
                    SCOPE_COUNT[i] = 0;
                    SCOPE_SUM[i] = 0;
                }
            }
        }
    }
}

pub fn send_profiling_event(link: &mut MessageLink<FuncLink>, event: ProfilerEvent) {
    unsafe {
        match &mut TRIGGER_NUMBER {
            Some(sample_number_in) => {
                let sample = sample_number_in[event] + 1;
                if sample == SAMPLING_RATE {
                    link.send(&architecture::Message::ProfilerEvent(event))
                        .unwrap();
                    sample_number_in[event] = 0;
                } else {
                    sample_number_in[event] = sample;
                }
            }
            None => unreachable!(),
        }
    }
}
