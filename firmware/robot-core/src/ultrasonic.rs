//! HC-SR04 echo width -> distance.

pub const MIN_MM: u64 = 20;
pub const MAX_MM: u64 = 1500;

/// Echo high time in CPU cycles -> distance in mm (speed of sound 343 m/s); 0 when out of range.
pub fn echo_to_mm(cycles: u32, cpu_hz: u32) -> u16 {
    let mm = cycles as u64 * 343_000 / (cpu_hz as u64 * 2);
    if (MIN_MM..=MAX_MM).contains(&mm) { mm as u16 } else { 0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn echo_1m() {
        let mm = echo_to_mm(419_825, 72_000_000);
        assert!((999..=1001).contains(&mm), "{mm}");
    }

    #[test]
    fn echo_out_of_range_is_zero() {
        assert_eq!(echo_to_mm(4_198, 72_000_000), 0); // ~10mm
        assert_eq!(echo_to_mm(839_650, 72_000_000), 0); // ~2000mm
    }
}
