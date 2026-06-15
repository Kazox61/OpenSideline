use std::time::Instant;

pub struct Timer {
    start: Instant,
}

impl Timer {
    pub fn new() -> Self {
        Timer {
            start: Instant::now(),
        }
    }
    pub fn restart(&mut self) {
        self.start = Instant::now();
    }
    pub fn duration(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }
}

impl Default for Timer {
    fn default() -> Self {
        Timer::new()
    }
}

pub struct GuardedTimer {
    timer: Timer,
    msg: String,
    enabled: bool,
}

impl GuardedTimer {
    pub fn new(msg: &str) -> Self {
        GuardedTimer {
            timer: Timer::new(),
            msg: msg.to_string(),
            enabled: true,
        }
    }

    pub fn disabled(msg: &str) -> Self {
        GuardedTimer {
            timer: Timer::new(),
            msg: msg.to_string(),
            enabled: false,
        }
    }
}

impl Drop for GuardedTimer {
    fn drop(&mut self) {
        if self.enabled {
            let d = self.timer.duration() * 1000.0;
            println!("{}: {:.1} milliseconds.", self.msg, d);
        }
    }
}
