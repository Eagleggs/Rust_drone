pub struct Liveliness {
    current_tick: u16,
    last_msg_tick: u16,

    got_first_msg: bool,

    max_link_wait: u16,
}

#[derive(Debug, PartialEq, Eq)]
pub enum LivelinessError {
    LinkDisconnected,
}

const MAX_U16: u16 = 65535;

impl Liveliness {
    pub fn new(max_link_wait_ticks: u16) -> Liveliness {
        Liveliness {
            current_tick: 0,
            last_msg_tick: 0,

            got_first_msg: false,

            max_link_wait: max_link_wait_ticks,
        }
    }

    pub fn tick(&mut self) -> Option<LivelinessError> {
        if self.current_tick == MAX_U16 {
            self.current_tick = 0;
        } else {
            self.current_tick += 1;
        }

        if self.got_first_msg && self.tick_distance(self.last_msg_tick) >= self.max_link_wait {
            Some(LivelinessError::LinkDisconnected)
        } else {
            None
        }
    }

    pub fn notify_alive(&mut self) {
        self.got_first_msg = true;
        self.last_msg_tick = self.current_tick;
    }

    fn tick_distance(&self, past_tick: u16) -> u16 {
        if past_tick <= self.current_tick {
            self.current_tick - past_tick
        } else {
            // the numbers overflowed, assume wrap around
            MAX_U16 - past_tick + self.current_tick
        }
    }
}
