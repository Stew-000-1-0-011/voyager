//! One 100Hz control step: link watchdog, e-stop latch, battery limits, drive and gripper.

use crate::drive::{ACCEL_STEP, SLOW_LIMIT, SlewLimiter, mix};
use crate::gripper::{Gripper, GripperConfig};
use crate::protocol::{
    Command, FAULT_BATTERY_CRITICAL, FAULT_BATTERY_LOW, FAULT_GRIP_TIMEOUT, FAULT_LINK_TIMEOUT, FLAG_ESTOP_RELEASE,
    FLAG_SLOW, GripCmd, GripState,
};
use crate::safety::{BatteryLevel, BatteryMonitor, EstopLatch, Watchdog};

/// Drive limit while the battery is low.
pub const LOW_BATTERY_LIMIT: i16 = 500;
const FULL_LIMIT: i16 = 1000;

#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    pub now_ms: u32,
    /// a command that arrived during this tick
    pub cmd: Option<Command>,
    pub battery_raw_mv: u16,
    pub enc_grip: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outputs {
    pub left: i16,
    pub right: i16,
    pub grip: i16,
    pub faults: u8,
    pub estop_latched: bool,
    pub grip_state: GripState,
    pub box_count: u8,
    pub battery_mv: u16,
}

pub struct Controller {
    cfg: GripperConfig,
    watchdog: Watchdog,
    battery: BatteryMonitor,
    estop: EstopLatch,
    gripper: Gripper,
    latest: Option<Command>,
    slew_left: SlewLimiter,
    slew_right: SlewLimiter,
}

impl Controller {
    pub fn new(cfg: GripperConfig) -> Self {
        Self {
            cfg,
            watchdog: Watchdog::default(),
            battery: BatteryMonitor::default(),
            estop: EstopLatch::default(),
            gripper: Gripper::new(),
            latest: None,
            slew_left: SlewLimiter::default(),
            slew_right: SlewLimiter::default(),
        }
    }

    pub fn step(&mut self, i: &Inputs) -> Outputs {
        if let Some(c) = i.cmd {
            self.watchdog.feed(i.now_ms);
            self.latest = Some(c);
        }
        let timed_out = self.watchdog.expired(i.now_ms);
        let level = self.battery.update(i.battery_raw_mv);
        let absent = level == BatteryLevel::Absent;

        let live = if timed_out { None } else { self.latest };
        let release = live.is_some_and(|c| c.flags & FLAG_ESTOP_RELEASE != 0);
        let neutral = live.is_none_or(|c| c.throttle == 0 && c.turn == 0);
        let estop_hold = self.estop.update(absent, release, neutral);
        let critical = level == BatteryLevel::Critical;

        let (left, right) = match live {
            Some(c) if !estop_hold && !critical => {
                let mut limit = if c.flags & FLAG_SLOW != 0 { SLOW_LIMIT } else { FULL_LIMIT };
                if level == BatteryLevel::Low {
                    limit = limit.min(LOW_BATTERY_LIMIT);
                }
                let (l, r) = mix(c.throttle, c.turn, limit);
                (self.slew_left.step(l, ACCEL_STEP), self.slew_right.step(r, ACCEL_STEP))
            }
            _ => {
                self.slew_left = SlewLimiter::default();
                self.slew_right = SlewLimiter::default();
                (0, 0)
            }
        };

        let grip_cmd = if estop_hold || critical {
            GripCmd::Stop
        } else {
            live.map_or(GripCmd::Keep, |c| c.grip)
        };
        let grip = self.gripper.update(grip_cmd, i.enc_grip, &self.cfg);
        let grip_state = self.gripper.state();

        let mut faults = 0;
        if timed_out {
            faults |= FAULT_LINK_TIMEOUT;
        }
        if grip_state == GripState::Fault {
            faults |= FAULT_GRIP_TIMEOUT;
        }
        match level {
            BatteryLevel::Low => faults |= FAULT_BATTERY_LOW,
            BatteryLevel::Critical => faults |= FAULT_BATTERY_CRITICAL,
            _ => {}
        }

        Outputs {
            left,
            right,
            grip,
            faults,
            estop_latched: self.estop.latched(),
            grip_state,
            box_count: self.gripper.box_count(i.enc_grip, &self.cfg),
            battery_mv: if absent { i.battery_raw_mv } else { self.battery.filtered_mv() },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gripper::DEFAULT_GRIPPER;
    use crate::protocol::{FAULT_LINK_TIMEOUT, GripCmd, GripState};

    const FULL_MV: u16 = 12_000;

    fn cmd(throttle: i16, grip: GripCmd) -> Command {
        Command { seq: 0, throttle, turn: 0, grip, flags: 0 }
    }

    struct Rig {
        c: Controller,
        now: u32,
    }

    impl Rig {
        fn new() -> Self {
            Self { c: Controller::new(DEFAULT_GRIPPER), now: 0 }
        }
        fn tick(&mut self, cmd: Option<Command>, mv: u16, enc: i32) -> Outputs {
            self.now += 10;
            self.c.step(&Inputs { now_ms: self.now, cmd, battery_raw_mv: mv, enc_grip: enc })
        }
    }

    #[test]
    fn no_command_means_brake() {
        let mut r = Rig::new();
        let o = r.tick(None, FULL_MV, 0);
        assert_eq!((o.left, o.right), (0, 0));
        assert!(o.faults & FAULT_LINK_TIMEOUT != 0);
    }

    #[test]
    fn command_drives_with_slew() {
        let mut r = Rig::new();
        let lefts: [i16; 3] = core::array::from_fn(|_| r.tick(Some(cmd(1000, GripCmd::Keep)), FULL_MV, 0).left);
        assert_eq!(lefts, [40, 80, 120]);
    }

    #[test]
    fn timeout_brakes_but_keeps_grip() {
        let mut r = Rig::new();
        for _ in 0..11 {
            r.tick(Some(cmd(0, GripCmd::Open)), FULL_MV, 0);
        }
        let mut enc = 0;
        for _ in 0..20 {
            enc += 50;
            r.tick(Some(cmd(300, GripCmd::Close)), FULL_MV, enc);
        }
        let mut o = r.tick(Some(cmd(300, GripCmd::Keep)), FULL_MV, enc);
        for _ in 0..11 {
            o = r.tick(Some(cmd(300, GripCmd::Keep)), FULL_MV, enc);
        }
        assert_eq!(o.grip_state, GripState::Holding);
        for _ in 0..21 {
            o = r.tick(None, FULL_MV, enc);
        }
        assert_eq!((o.left, o.right), (0, 0));
        assert_eq!(o.grip, DEFAULT_GRIPPER.hold_cmd);
    }

    #[test]
    fn estop_cuts_everything() {
        let mut r = Rig::new();
        for _ in 0..5 {
            r.tick(Some(cmd(1000, GripCmd::Close)), FULL_MV, 0);
        }
        let o = r.tick(Some(cmd(1000, GripCmd::Close)), 0, 0);
        assert_eq!((o.left, o.right, o.grip), (0, 0, 0));
        assert!(o.estop_latched);
    }

    #[test]
    fn low_battery_limits_output() {
        let mut r = Rig::new();
        let mut o = r.tick(Some(cmd(1000, GripCmd::Keep)), 10_400, 0);
        for _ in 0..100 {
            o = r.tick(Some(cmd(1000, GripCmd::Keep)), 10_400, 0);
        }
        assert!(o.left <= 500 && o.left > 0, "{}", o.left);
    }
}
