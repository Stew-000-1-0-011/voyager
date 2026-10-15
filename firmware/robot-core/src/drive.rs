//! Arcade mixing, slew limiting and DRV8871 bridge output (brake decay in both directions).

const FULL: i32 = 1000;

pub const SLOW_LIMIT: i16 = 400;
/// 100Hz control loop: 0 -> 1000 in 0.25s.
pub const ACCEL_STEP: i16 = 40;

/// Left = throttle + turn, right = throttle - turn; scaled down together past ±1000, then clipped to ±limit.
pub fn mix(throttle: i16, turn: i16, limit: i16) -> (i16, i16) {
    let (mut l, mut r) = (throttle as i32 + turn as i32, throttle as i32 - turn as i32);
    let peak = l.abs().max(r.abs());
    if peak > FULL {
        l = l * FULL / peak;
        r = r * FULL / peak;
    }
    let lim = limit.clamp(0, FULL as i16) as i32;
    (l.clamp(-lim, lim) as i16, r.clamp(-lim, lim) as i16)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SlewLimiter {
    pub current: i16,
}

impl SlewLimiter {
    pub fn step(&mut self, target: i16, max_step: i16) -> i16 {
        let delta = (target as i32 - self.current as i32).clamp(-(max_step as i32), max_step as i32);
        self.current = (self.current as i32 + delta) as i16;
        self.current
    }
}

/// Compare values for the two DRV8871 inputs. `period` means "always high".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BridgeOutput {
    pub in1: u16,
    pub in2: u16,
}

/// One input is held high and the other carries the inverted duty, so the off-phase is
/// always brake (IN1 = IN2 = high) and forward/reverse respond identically.
pub fn bridge(cmd: i16, period: u16) -> BridgeOutput {
    let mag = (cmd as i32).abs().min(FULL);
    let low = (period as i32 - period as i32 * mag / FULL) as u16;
    match cmd {
        c if c > 0 => BridgeOutput { in1: period, in2: low },
        c if c < 0 => BridgeOutput { in1: low, in2: period },
        _ => BridgeOutput { in1: period, in2: period },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mix_straight() {
        assert_eq!(mix(500, 0, 1000), (500, 500));
    }

    #[test]
    fn mix_spin() {
        assert_eq!(mix(0, 300, 1000), (300, -300));
    }

    #[test]
    fn mix_saturates_keeping_ratio() {
        assert_eq!(mix(1000, 500, 1000), (1000, 333));
    }

    #[test]
    fn mix_slow_limit() {
        assert_eq!(mix(1000, 0, SLOW_LIMIT), (400, 400));
    }

    #[test]
    fn slew_limits_step() {
        let mut s = SlewLimiter::default();
        for _ in 0..3 {
            s.step(1000, 40);
        }
        assert_eq!(s.current, 120);
        let mut s = SlewLimiter::default();
        for _ in 0..3 {
            s.step(-1000, 40);
        }
        assert_eq!(s.current, -120);
    }

    #[test]
    fn bridge_forward_has_brake_off_phase() {
        assert_eq!(bridge(250, 3600), BridgeOutput { in1: 3600, in2: 2700 });
    }

    #[test]
    fn bridge_reverse_mirrors_forward() {
        assert_eq!(bridge(-250, 3600), BridgeOutput { in1: 2700, in2: 3600 });
    }

    #[test]
    fn bridge_zero_is_brake() {
        assert_eq!(bridge(0, 3600), BridgeOutput { in1: 3600, in2: 3600 });
    }

    #[test]
    fn bridge_full() {
        assert_eq!(bridge(1000, 3600).in2, 0);
    }
}
