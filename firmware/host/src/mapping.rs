//! Gamepad state -> robot Command.

use robot_core::protocol::{Command, FLAG_ESTOP_RELEASE, FLAG_SLOW, GripCmd};
use serde::Deserialize;

/// Raw pad snapshot. Buttons are true while pressed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PadState {
    pub left_y: f32,
    pub right_x: f32,
    pub close: bool,
    pub open: bool,
    pub slow_toggle: bool,
    pub estop_release: bool,
}

/// Button names are gilrs `Button` Debug names, e.g. "South".
#[derive(Clone, Debug, Deserialize, PartialEq)]
pub struct Mapping {
    pub deadzone: f32,
    pub close: String,
    pub open: String,
    pub slow_toggle: String,
    pub estop_release: String,
    pub invert_left_y: bool,
}

impl Default for Mapping {
    fn default() -> Self {
        Self {
            deadzone: 0.08,
            close: "South".into(),
            open: "East".into(),
            slow_toggle: "North".into(),
            estop_release: "Start".into(),
            invert_left_y: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Mapper {
    pub slow: bool,
    pub prev: PadState,
    pub seq: u8,
}

/// Stick value -> -1000..=1000 with the deadzone removed and the remainder rescaled.
fn axis(v: f32, deadzone: f32) -> i16 {
    let mag = v.abs().min(1.0);
    if mag <= deadzone {
        return 0;
    }
    let scaled = (mag - deadzone) / (1.0 - deadzone) * 1000.0;
    (scaled.round() as i16).clamp(0, 1000) * if v < 0.0 { -1 } else { 1 }
}

impl Mapper {
    pub fn command(&mut self, pad: &PadState, m: &Mapping) -> Command {
        if pad.slow_toggle && !self.prev.slow_toggle {
            self.slow = !self.slow;
        }
        self.prev = *pad;
        self.seq = self.seq.wrapping_add(1);

        let y = if m.invert_left_y { -pad.left_y } else { pad.left_y };
        let grip = match (pad.close, pad.open) {
            (true, false) => GripCmd::Close,
            (false, true) => GripCmd::Open,
            _ => GripCmd::Keep,
        };
        let mut flags = 0;
        if self.slow {
            flags |= FLAG_SLOW;
        }
        if pad.estop_release {
            flags |= FLAG_ESTOP_RELEASE;
        }
        Command { seq: self.seq, throttle: axis(y, m.deadzone), turn: axis(pad.right_x, m.deadzone), grip, flags }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robot_core::protocol::GripCmd;

    fn m() -> Mapping {
        Mapping::default()
    }

    fn pad(left_y: f32, right_x: f32) -> PadState {
        PadState { left_y, right_x, ..PadState::default() }
    }

    #[test]
    fn deadzone_maps_to_zero() {
        assert_eq!(Mapper::default().command(&pad(0.05, 0.0), &m()).throttle, 0);
    }

    #[test]
    fn full_stick_is_1000() {
        // gilrs reports stick-up as +1.0 on every platform
        let c = Mapper::default().command(&pad(1.0, -1.0), &m());
        assert_eq!(c.throttle, 1000);
        assert_eq!(c.turn, -1000);
    }

    #[test]
    fn edge_after_deadzone_is_small() {
        let t = Mapper::default().command(&pad(0.09, 0.0), &m()).throttle;
        assert!((1..=20).contains(&t), "{t}");
    }

    #[test]
    fn slow_toggles_on_rising_edge() {
        let mut mp = Mapper::default();
        let held = PadState { slow_toggle: true, ..PadState::default() };
        for _ in 0..3 {
            mp.command(&held, &m());
        }
        assert!(mp.slow);
        mp.command(&PadState::default(), &m());
        mp.command(&held, &m());
        assert!(!mp.slow);
    }

    #[test]
    fn open_and_close_together_is_keep() {
        let p = PadState { open: true, close: true, ..PadState::default() };
        assert_eq!(Mapper::default().command(&p, &m()).grip, GripCmd::Keep);
    }

    #[test]
    fn seq_increments_and_wraps() {
        let mut mp = Mapper { seq: 254, ..Mapper::default() };
        let p = PadState::default();
        assert_eq!(mp.command(&p, &m()).seq, 255);
        assert_eq!(mp.command(&p, &m()).seq, 0);
    }
}
