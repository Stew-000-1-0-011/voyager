//! HC-SR04 echo capture: EXTI on PB12..PB15 (both edges), timed with the DWT cycle counter.

use robot_core::ultrasonic::echo_to_mm;

/// Bit of each sensor's echo pin in GPIOB (left-front, right-front, left-rear, right-rear).
pub const ECHO_BITS: [u32; 4] = [12, 13, 14, 15];

#[derive(Default)]
pub struct EchoCapture {
    rise: [Option<u32>; 4],
}

impl EchoCapture {
    /// Called on any edge of channel `ch`. Returns a distance when a falling edge closes a pulse.
    pub fn edge(&mut self, ch: usize, high: bool, now_cycles: u32, cpu_hz: u32) -> Option<u16> {
        if high {
            self.rise[ch] = Some(now_cycles);
            None
        } else {
            self.rise[ch].take().map(|t| echo_to_mm(now_cycles.wrapping_sub(t), cpu_hz))
        }
    }
}
