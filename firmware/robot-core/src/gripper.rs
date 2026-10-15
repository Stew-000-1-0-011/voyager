//! Stall-based gripper: drive until the encoder stops, then hold with a weak command.

pub use crate::protocol::{GripCmd, GripState};

#[derive(Clone, Copy, Debug)]
pub struct GripperConfig {
    /// +1 if the encoder counts up while closing, -1 otherwise. Motor direction is fixed by wiring
    /// (swap the two motor wires if Close opens).
    pub enc_dir: i8,
    pub open_cmd: i16,
    pub close_cmd: i16,
    pub hold_cmd: i16,
    /// encoder change per tick still counted as "not moving"
    pub stall_eps: i32,
    pub stall_ticks: u16,
    pub timeout_ticks: u16,
    /// closed distance from the open stop (counts) at or below which 2 boxes are held
    pub two_box_max: i32,
    pub one_box_max: i32,
}

/// 63 counts/mm ≈ 11 pulses × 4 × ~90:1 gearbox / 62.8mm pinion circumference (measure, see docs/tuning.md).
pub const DEFAULT_GRIPPER: GripperConfig = GripperConfig {
    enc_dir: 1,
    open_cmd: -600,
    close_cmd: 500,
    hold_cmd: 200,
    stall_eps: 2,
    stall_ticks: 10,
    timeout_ticks: 300,
    two_box_max: 63 * 80,
    one_box_max: 63 * 130,
};

#[derive(Clone, Copy, Debug)]
pub struct Gripper {
    state: GripState,
    homed: bool,
    home: i32,
    last_enc: Option<i32>,
    still: u16,
    ticks: u16,
}

impl Default for Gripper {
    fn default() -> Self {
        Self::new()
    }
}

impl Gripper {
    pub const fn new() -> Self {
        Self { state: GripState::Unhomed, homed: false, home: 0, last_enc: None, still: 0, ticks: 0 }
    }

    pub fn state(&self) -> GripState {
        self.state
    }

    fn start(&mut self, s: GripState) {
        self.state = s;
        self.still = 0;
        self.ticks = 0;
    }

    /// Motor command for this 10ms tick (positive = closing).
    pub fn update(&mut self, cmd: GripCmd, enc: i32, cfg: &GripperConfig) -> i16 {
        use GripState::*;
        match cmd {
            GripCmd::Stop => self.state = Idle,
            GripCmd::Open if !matches!(self.state, Opening | Open) => self.start(Opening),
            GripCmd::Close => match self.state {
                Unhomed | Idle | Open | Fault if !self.homed => self.start(Opening),
                Unhomed | Idle | Open | Fault => self.start(Closing),
                _ => {}
            },
            _ => {}
        }

        let moved = self.last_enc.map_or(i32::MAX, |l| (enc - l).abs());
        self.last_enc = Some(enc);
        if matches!(self.state, Opening | Closing) {
            self.ticks = self.ticks.saturating_add(1);
            self.still = if moved <= cfg.stall_eps { self.still.saturating_add(1) } else { 0 };
            if self.still >= cfg.stall_ticks {
                if self.state == Opening {
                    if !self.homed {
                        self.home = enc;
                        self.homed = true;
                    }
                    self.state = Open;
                } else {
                    self.state = Holding;
                }
            } else if self.ticks >= cfg.timeout_ticks {
                self.state = Fault;
            }
        }

        match self.state {
            Opening => cfg.open_cmd,
            Closing => cfg.close_cmd,
            Holding => cfg.hold_cmd,
            _ => 0,
        }
    }

    /// Boxes held (0..=2); 0 unless holding.
    pub fn box_count(&self, enc: i32, cfg: &GripperConfig) -> u8 {
        if self.state != GripState::Holding {
            return 0;
        }
        let d = (enc - self.home) * cfg.enc_dir as i32;
        if d <= cfg.two_box_max {
            2
        } else if d <= cfg.one_box_max {
            1
        } else {
            0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const C: GripperConfig = DEFAULT_GRIPPER;

    /// Runs `n` cycles with the encoder fixed at `enc`; returns the last output.
    fn hold_at(g: &mut Gripper, cmd: GripCmd, enc: i32, n: usize, cfg: &GripperConfig) -> i16 {
        let mut out = 0;
        for _ in 0..n {
            out = g.update(cmd, enc, cfg);
        }
        out
    }

    /// Homed at enc = 0, then closed until the encoder stops at `stop`.
    fn closed_at(stop: i32, cfg: &GripperConfig) -> Gripper {
        let mut g = Gripper::new();
        g.update(GripCmd::Open, 0, cfg);
        hold_at(&mut g, GripCmd::Keep, 0, 11, cfg);
        let mut enc = 0;
        g.update(GripCmd::Close, enc, cfg);
        while enc != stop {
            enc += (stop - enc).signum() * 50.min((stop - enc).abs());
            g.update(GripCmd::Keep, enc, cfg);
        }
        hold_at(&mut g, GripCmd::Keep, stop, 11, cfg);
        g
    }

    #[test]
    fn first_close_homes_then_closes() {
        let mut g = Gripper::new();
        assert_eq!(g.update(GripCmd::Close, 0, &C), -600);
        assert_eq!(g.state(), GripState::Opening);
        hold_at(&mut g, GripCmd::Keep, 0, 11, &C);
        assert_eq!(g.state(), GripState::Open);
        assert_eq!(g.update(GripCmd::Close, 0, &C), 500);
        assert_eq!(g.state(), GripState::Closing);
    }

    #[test]
    fn close_until_stall_then_hold() {
        let g = closed_at(63 * 60, &C);
        assert_eq!(g.state(), GripState::Holding);
        let mut g = g;
        assert_eq!(g.update(GripCmd::Keep, 63 * 60, &C), 200);
    }

    #[test]
    fn box_count_two() {
        let g = closed_at(63 * 50, &C);
        assert_eq!(g.box_count(63 * 50, &C), 2);
    }

    #[test]
    fn box_count_one_and_zero() {
        let g = closed_at(63 * 125, &C);
        assert_eq!(g.box_count(63 * 125, &C), 1);
        let g = closed_at(63 * 135, &C);
        assert_eq!(g.box_count(63 * 135, &C), 0);
    }

    #[test]
    fn timeout_goes_fault() {
        let mut g = Gripper::new();
        g.update(GripCmd::Open, 0, &C);
        hold_at(&mut g, GripCmd::Keep, 0, 11, &C);
        let mut enc = 0;
        g.update(GripCmd::Close, enc, &C);
        let mut out = 1;
        for _ in 0..300 {
            enc += 10;
            out = g.update(GripCmd::Keep, enc, &C);
        }
        assert_eq!(g.state(), GripState::Fault);
        assert_eq!(out, 0);
    }

    #[test]
    fn stop_keeps_home() {
        let mut g = closed_at(63 * 60, &C);
        assert_eq!(g.update(GripCmd::Stop, 63 * 60, &C), 0);
        assert_eq!(g.state(), GripState::Idle);
        assert_eq!(g.update(GripCmd::Close, 63 * 60, &C), 500);
        assert_eq!(g.state(), GripState::Closing);
    }

    #[test]
    fn reversed_encoder_with_dir_flag() {
        // motor wiring already fixed (Close closes); only the encoder counts down while closing
        let cfg = GripperConfig { enc_dir: -1, ..DEFAULT_GRIPPER };
        let mut g = Gripper::new();
        assert_eq!(g.update(GripCmd::Open, 0, &cfg), -600);
        let g = closed_at(-63 * 50, &cfg);
        assert_eq!(g.state(), GripState::Holding);
        assert_eq!(g.box_count(-63 * 50, &cfg), 2);
        let mut g = g;
        assert_eq!(g.update(GripCmd::Keep, -63 * 50, &cfg), 200);
    }
}
