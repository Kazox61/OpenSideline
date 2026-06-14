/// Critically-damped spring follower (Game Programming Gems 4 / Unity SmoothDamp).
/// Call `update` once per time step; never overshoots, eases naturally to target.
pub struct SmoothDamp {
    omega: f32,
    pub current: f32,
    velocity: f32,
}

impl SmoothDamp {
    pub fn new(initial: f32, smooth_time: f32) -> Self {
        let smooth_time = smooth_time.max(1e-4);
        Self { omega: 2.0 / smooth_time, current: initial, velocity: 0.0 }
    }

    pub fn update(&mut self, target: f32, dt: f32, max_speed: Option<f32>) -> f32 {
        let x = self.omega * dt;
        let exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);

        let mut change = self.current - target;
        if let Some(ms) = max_speed {
            let max_change = ms * (2.0 / self.omega); // smooth_time * max_speed
            change = change.clamp(-max_change, max_change);
        }

        let temp = (self.velocity + self.omega * change) * dt;
        self.velocity = (self.velocity - self.omega * temp) * exp;
        let new = (self.current - change) + (change + temp) * exp;

        // overshoot guard
        if (target - self.current > 0.0) == (new > target) {
            self.velocity = (new - target) / dt;
            self.current = target;
        } else {
            self.current = new;
        }
        self.current
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converges_to_target() {
        let smooth_time = 1.0_f32;
        let dt = 1.0 / 30.0;
        let mut sd = SmoothDamp::new(0.0, smooth_time);
        let target = 100.0_f32;
        // run for 5× smooth_time
        let steps = (5.0 / dt) as usize;
        for _ in 0..steps {
            sd.update(target, dt, None);
        }
        assert!((sd.current - target).abs() < 1.0, "current = {}", sd.current);
    }

    #[test]
    fn no_overshoot() {
        let mut sd = SmoothDamp::new(0.0, 1.0);
        let target = 50.0_f32;
        let dt = 1.0 / 30.0;
        for _ in 0..200 {
            let v = sd.update(target, dt, None);
            assert!(v <= target + f32::EPSILON, "overshot: {v}");
        }
    }
}
