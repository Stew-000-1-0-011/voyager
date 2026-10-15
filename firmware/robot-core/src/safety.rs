//! Link watchdog, battery monitoring and e-stop latch.

pub const LINK_TIMEOUT_MS: u32 = 200;
/// Below this the battery is treated as disconnected (e-stop pressed).
pub const ABSENT_MV: u16 = 3000;
pub const CRITICAL_MV: u32 = 9900;
pub const LOW_ENTER_MV: u32 = 10500;
pub const LOW_EXIT_MV: u32 = 10700;
/// Critical latches only after the average has stayed below CRITICAL_MV this many ticks (0.5s at 100Hz).
pub const CRITICAL_HOLD_TICKS: u16 = 50;

#[derive(Clone, Copy, Debug, Default)]
pub struct Watchdog {
    last_ms: Option<u32>,
}

impl Watchdog {
    pub fn feed(&mut self, now_ms: u32) {
        self.last_ms = Some(now_ms);
    }

    pub fn expired(&self, now_ms: u32) -> bool {
        match self.last_ms {
            None => true,
            Some(t) => now_ms.wrapping_sub(t) >= LINK_TIMEOUT_MS,
        }
    }
}

/// ADC counts (12 bit, 3.3V ref) behind the 47k/10k divider -> battery mV.
pub fn adc_to_mv(raw: u16) -> u16 {
    (raw as u32 * 3300 * 57 / (4095 * 10)) as u16
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BatteryLevel {
    Ok,
    Low,
    Critical,
    Absent,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BatteryMonitor {
    filtered_mv: Option<u32>,
    critical: bool,
    low: bool,
    below_critical: u16,
}

impl BatteryMonitor {
    /// Exponential moving average (new sample weight 1/16). `Absent` (e-stop / switch off) is reported
    /// immediately and clears the history, so the bus decaying through 9.9V on its way to 0V cannot latch
    /// `Critical`. `Critical` latches after CRITICAL_HOLD_TICKS below the limit, until power is cut.
    pub fn update(&mut self, raw_mv: u16) -> BatteryLevel {
        if raw_mv < ABSENT_MV {
            *self = Self::default();
            return BatteryLevel::Absent;
        }
        let f = match self.filtered_mv {
            None => raw_mv as u32,
            Some(f) => (f * 15 + raw_mv as u32) / 16,
        };
        self.filtered_mv = Some(f);
        self.below_critical = if f < CRITICAL_MV { self.below_critical.saturating_add(1) } else { 0 };
        if self.below_critical >= CRITICAL_HOLD_TICKS {
            self.critical = true;
        }
        if f < LOW_ENTER_MV {
            self.low = true;
        } else if f > LOW_EXIT_MV {
            self.low = false;
        }
        if self.critical {
            BatteryLevel::Critical
        } else if self.low {
            BatteryLevel::Low
        } else {
            BatteryLevel::Ok
        }
    }

    pub fn filtered_mv(&self) -> u16 {
        self.filtered_mv.unwrap_or(0) as u16
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EstopLatch {
    latched: bool,
    need_neutral: bool,
}

impl EstopLatch {
    /// Returns true while driving is inhibited.
    pub fn update(&mut self, absent: bool, release: bool, stick_neutral: bool) -> bool {
        if absent {
            self.latched = true;
            self.need_neutral = false;
            return true;
        }
        if self.latched {
            if !release {
                return true;
            }
            self.latched = false;
            self.need_neutral = true;
        }
        if self.need_neutral {
            if !stick_neutral {
                return true;
            }
            self.need_neutral = false;
        }
        false
    }

    pub fn latched(&self) -> bool {
        self.latched || self.need_neutral
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(m: &mut BatteryMonitor, mv: u16, n: usize) -> BatteryLevel {
        let mut last = BatteryLevel::Ok;
        for _ in 0..n {
            last = m.update(mv);
        }
        last
    }

    #[test]
    fn watchdog_expires_after_200ms() {
        let mut w = Watchdog::default();
        w.feed(1000);
        assert!(!w.expired(1199));
        assert!(w.expired(1200));
    }

    #[test]
    fn watchdog_unfed_is_expired() {
        assert!(Watchdog::default().expired(0));
    }

    #[test]
    fn watchdog_handles_wraparound() {
        let mut w = Watchdog::default();
        w.feed(u32::MAX - 50);
        assert!(!w.expired(100));
    }

    #[test]
    fn adc_full_scale() {
        assert_eq!(adc_to_mv(4095), 18810);
        assert_eq!(adc_to_mv(0), 0);
    }

    #[test]
    fn battery_low_hysteresis() {
        let mut m = BatteryMonitor::default();
        assert_eq!(run(&mut m, 10400, 100), BatteryLevel::Low);
        assert_eq!(run(&mut m, 10600, 100), BatteryLevel::Low);
        assert_eq!(run(&mut m, 10800, 100), BatteryLevel::Ok);
    }

    #[test]
    fn brief_sag_does_not_trip_critical() {
        let mut m = BatteryMonitor::default();
        m.update(12000);
        assert_ne!(run(&mut m, 9000, 3), BatteryLevel::Critical);
        assert_eq!(run(&mut m, 12000, 50), BatteryLevel::Ok);
    }

    #[test]
    fn critical_latches() {
        let mut m = BatteryMonitor::default();
        assert_eq!(run(&mut m, 9500, 100), BatteryLevel::Critical);
        assert_eq!(run(&mut m, 12000, 100), BatteryLevel::Critical);
    }

    #[test]
    fn absent_is_immediate() {
        let mut m = BatteryMonitor::default();
        m.update(12000);
        assert_eq!(m.update(0), BatteryLevel::Absent);
    }

    #[test]
    fn estop_does_not_poison_the_filter() {
        // a long e-stop (raw 0V) must not drag the average into a latched Critical
        let mut m = BatteryMonitor::default();
        run(&mut m, 12000, 10);
        run(&mut m, 0, 500);
        assert_eq!(m.update(12000), BatteryLevel::Ok);
    }

    #[test]
    fn power_off_ramp_does_not_latch_critical() {
        // e-stop / main switch: the bus capacitor decays 12V -> 0 over ~200ms, then power returns
        let mut m = BatteryMonitor::default();
        run(&mut m, 12000, 20);
        for k in 0..20u16 {
            m.update(12000 - k * 600);
        }
        run(&mut m, 0, 10);
        assert_eq!(run(&mut m, 12000, 5), BatteryLevel::Ok);
    }

    #[test]
    fn genuinely_low_battery_relatches_after_power_cycle() {
        let mut m = BatteryMonitor::default();
        assert_eq!(run(&mut m, 9500, 100), BatteryLevel::Critical);
        run(&mut m, 0, 5);
        assert_eq!(run(&mut m, 9500, 100), BatteryLevel::Critical);
    }

    #[test]
    fn estop_latches_until_release() {
        let mut e = EstopLatch::default();
        assert!(e.update(true, false, true));
        assert!(e.update(false, false, true));
        assert!(!e.update(false, true, true));
        assert!(!e.update(false, false, true));
    }

    #[test]
    fn estop_release_requires_neutral_stick() {
        let mut e = EstopLatch::default();
        assert!(e.update(true, false, false));
        assert!(e.update(false, true, false));
        assert!(e.update(false, false, false));
        assert!(!e.update(false, false, true));
    }
}
