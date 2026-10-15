//! Software quadrature decoder (x4) for the gripper encoder on EXTI pins.

#[derive(Clone, Copy, Debug, Default)]
pub struct QuadDecoder {
    count: i32,
    last: u8,
}

impl QuadDecoder {
    pub fn update(&mut self, a: bool, b: bool) {
        let now = ((a as u8) << 1) | b as u8;
        // Gray order for +1: 00 -> 10 -> 11 -> 01 -> 00
        let step = match (self.last, now) {
            (0b00, 0b10) | (0b10, 0b11) | (0b11, 0b01) | (0b01, 0b00) => 1,
            (0b10, 0b00) | (0b11, 0b10) | (0b01, 0b11) | (0b00, 0b01) => -1,
            (x, y) if x == y => 0,
            _ => {
                // both channels changed at once: direction unknown, ignore
                self.last = now;
                return;
            }
        };
        self.count = self.count.wrapping_add(step);
        self.last = now;
    }

    pub fn count(&self) -> i32 {
        self.count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seq(q: &mut QuadDecoder, s: &[(bool, bool)]) {
        for &(a, b) in s {
            q.update(a, b);
        }
    }

    #[test]
    fn quad_counts_forward_and_back() {
        let mut q = QuadDecoder::default();
        seq(&mut q, &[(true, false), (true, true), (false, true), (false, false)]);
        assert_eq!(q.count(), 4);
        seq(&mut q, &[(false, true), (true, true), (true, false), (false, false)]);
        assert_eq!(q.count(), 0);
    }

    #[test]
    fn quad_ignores_invalid_transition() {
        let mut q = QuadDecoder::default();
        q.update(true, true);
        assert_eq!(q.count(), 0);
    }
}
