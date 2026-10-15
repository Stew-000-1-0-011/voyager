//! Command / telemetry packets. Framing: COBS (0x00 delimiter); integrity: CRC-8/SMBUS.
//! Multi-byte values are little-endian. The last payload byte is the CRC of the bytes before it.

use crc::{CRC_8_SMBUS, Crc};

pub const MAX_FRAME: usize = 48;
pub const FLAG_SLOW: u8 = 0x01;
pub const FLAG_ESTOP_RELEASE: u8 = 0x02;

pub const FAULT_GRIP_TIMEOUT: u8 = 0x01;
pub const FAULT_BATTERY_LOW: u8 = 0x02;
pub const FAULT_BATTERY_CRITICAL: u8 = 0x04;
pub const FAULT_LINK_TIMEOUT: u8 = 0x08;

const TYPE_COMMAND: u8 = 0x01;
const TYPE_TELEMETRY: u8 = 0x02;
const COMMAND_LEN: usize = 9;
const TELEMETRY_LEN: usize = 29;
const AXIS_MAX: i16 = 1000;

const CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_SMBUS);

pub fn crc8(bytes: &[u8]) -> u8 {
    CRC8.checksum(bytes)
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GripCmd {
    Keep = 0,
    Open = 1,
    Close = 2,
    Stop = 3,
}

impl GripCmd {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Keep,
            1 => Self::Open,
            2 => Self::Close,
            3 => Self::Stop,
            _ => return None,
        })
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GripState {
    Unhomed = 0,
    Idle = 1,
    Opening = 2,
    Open = 3,
    Closing = 4,
    Holding = 5,
    Fault = 6,
}

impl GripState {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Unhomed,
            1 => Self::Idle,
            2 => Self::Opening,
            3 => Self::Open,
            4 => Self::Closing,
            5 => Self::Holding,
            6 => Self::Fault,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Cobs,
    Length,
    Crc,
    Type,
    Value,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    pub seq: u8,
    pub throttle: i16,
    pub turn: i16,
    pub grip: GripCmd,
    pub flags: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Telemetry {
    pub seq: u8,
    pub battery_mv: u16,
    pub enc_left: i32,
    pub enc_right: i32,
    pub enc_grip: i32,
    pub grip_state: GripState,
    pub box_count: u8,
    /// left-front, right-front, left-rear, right-rear; 0 = no reading
    pub sonar_mm: [u16; 4],
    pub estop_latched: bool,
    pub faults: u8,
}

fn check(payload: &[u8], len: usize, ty: u8) -> Result<(), DecodeError> {
    if payload.len() != len {
        return Err(DecodeError::Length);
    }
    if crc8(&payload[..len - 1]) != payload[len - 1] {
        return Err(DecodeError::Crc);
    }
    if payload[0] != ty {
        return Err(DecodeError::Type);
    }
    Ok(())
}

fn seal<const N: usize>(mut p: [u8; N], out: &mut [u8; MAX_FRAME]) -> usize {
    p[N - 1] = crc8(&p[..N - 1]);
    corncobs::encode_buf(&p, out)
}

fn i16_at(p: &[u8], i: usize) -> i16 {
    i16::from_le_bytes([p[i], p[i + 1]])
}

fn u16_at(p: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([p[i], p[i + 1]])
}

fn i32_at(p: &[u8], i: usize) -> i32 {
    i32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]])
}

impl Command {
    pub(crate) fn payload(&self) -> [u8; COMMAND_LEN] {
        let mut p = [0u8; COMMAND_LEN];
        p[0] = TYPE_COMMAND;
        p[1] = self.seq;
        p[2..4].copy_from_slice(&self.throttle.to_le_bytes());
        p[4..6].copy_from_slice(&self.turn.to_le_bytes());
        p[6] = self.grip as u8;
        p[7] = self.flags;
        p[8] = crc8(&p[..8]);
        p
    }

    pub fn encode(&self, out: &mut [u8; MAX_FRAME]) -> usize {
        seal(self.payload(), out)
    }

    pub fn decode(payload: &[u8]) -> Result<Command, DecodeError> {
        check(payload, COMMAND_LEN, TYPE_COMMAND)?;
        let throttle = i16_at(payload, 2);
        let turn = i16_at(payload, 4);
        if !(-AXIS_MAX..=AXIS_MAX).contains(&throttle) || !(-AXIS_MAX..=AXIS_MAX).contains(&turn) {
            return Err(DecodeError::Value);
        }
        let grip = GripCmd::from_u8(payload[6]).ok_or(DecodeError::Value)?;
        Ok(Command { seq: payload[1], throttle, turn, grip, flags: payload[7] })
    }
}

impl Telemetry {
    fn payload(&self) -> [u8; TELEMETRY_LEN] {
        let mut p = [0u8; TELEMETRY_LEN];
        p[0] = TYPE_TELEMETRY;
        p[1] = self.seq;
        p[2..4].copy_from_slice(&self.battery_mv.to_le_bytes());
        p[4..8].copy_from_slice(&self.enc_left.to_le_bytes());
        p[8..12].copy_from_slice(&self.enc_right.to_le_bytes());
        p[12..16].copy_from_slice(&self.enc_grip.to_le_bytes());
        p[16] = self.grip_state as u8;
        p[17] = self.box_count;
        for (i, mm) in self.sonar_mm.iter().enumerate() {
            p[18 + 2 * i..20 + 2 * i].copy_from_slice(&mm.to_le_bytes());
        }
        p[26] = self.estop_latched as u8;
        p[27] = self.faults;
        p
    }

    pub fn encode(&self, out: &mut [u8; MAX_FRAME]) -> usize {
        seal(self.payload(), out)
    }

    pub fn decode(payload: &[u8]) -> Result<Telemetry, DecodeError> {
        check(payload, TELEMETRY_LEN, TYPE_TELEMETRY)?;
        let grip_state = GripState::from_u8(payload[16]).ok_or(DecodeError::Value)?;
        let mut sonar_mm = [0u16; 4];
        for (i, mm) in sonar_mm.iter_mut().enumerate() {
            *mm = u16_at(payload, 18 + 2 * i);
        }
        Ok(Telemetry {
            seq: payload[1],
            battery_mv: u16_at(payload, 2),
            enc_left: i32_at(payload, 4),
            enc_right: i32_at(payload, 8),
            enc_grip: i32_at(payload, 12),
            grip_state,
            box_count: payload[17],
            sonar_mm,
            estop_latched: payload[26] & 0x01 != 0,
            faults: payload[27],
        })
    }
}

/// Accumulates bytes until a 0x00 delimiter, then yields the COBS-decoded payload.
/// Overlong frames are discarded up to the next delimiter.
pub struct FrameDecoder {
    buf: [u8; MAX_FRAME],
    len: usize,
    overflow: bool,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub const fn new() -> Self {
        Self { buf: [0; MAX_FRAME], len: 0, overflow: false }
    }

    pub fn push(&mut self, byte: u8) -> Option<&[u8]> {
        if byte != 0 {
            if self.len < MAX_FRAME - 1 {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }
        let (len, overflow) = (self.len, self.overflow);
        self.len = 0;
        self.overflow = false;
        if overflow || len == 0 {
            return None;
        }
        self.buf[len] = 0;
        let n = corncobs::decode_in_place(&mut self.buf[..=len]).ok()?;
        Some(&self.buf[..n])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame_of(buf: &[u8]) -> ([u8; MAX_FRAME], usize) {
        let mut out = [0u8; MAX_FRAME];
        out[..buf.len()].copy_from_slice(buf);
        (out, buf.len())
    }

    fn feed<'a>(dec: &'a mut FrameDecoder, bytes: &[u8]) -> Option<&'a [u8]> {
        let (last, rest) = bytes.split_last().unwrap();
        for &b in rest {
            assert!(dec.push(b).is_none());
        }
        dec.push(*last)
    }

    #[test]
    fn command_roundtrip() {
        let cmd = Command { seq: 7, throttle: -1000, turn: 999, grip: GripCmd::Close, flags: FLAG_SLOW };
        let mut out = [0u8; MAX_FRAME];
        let n = cmd.encode(&mut out);
        assert_eq!(out[n - 1], 0);
        assert!(out[..n - 1].iter().all(|&b| b != 0));
        let mut dec = FrameDecoder::new();
        let payload = feed(&mut dec, &out[..n]).unwrap();
        assert_eq!(Command::decode(payload), Ok(cmd));
    }

    #[test]
    fn telemetry_roundtrip() {
        let t = Telemetry {
            seq: 200,
            battery_mv: 12_345,
            enc_left: i32::MIN,
            enc_right: i32::MAX,
            enc_grip: -42,
            grip_state: GripState::Holding,
            box_count: 2,
            sonar_mm: [0, 1500, 20, 65535],
            estop_latched: true,
            faults: FAULT_GRIP_TIMEOUT | FAULT_LINK_TIMEOUT,
        };
        let mut out = [0u8; MAX_FRAME];
        let n = t.encode(&mut out);
        let mut dec = FrameDecoder::new();
        let payload = feed(&mut dec, &out[..n]).unwrap();
        assert_eq!(Telemetry::decode(payload), Ok(t));
    }

    #[test]
    fn crc_error_detected() {
        let mut p = Command { seq: 1, throttle: 10, turn: 0, grip: GripCmd::Keep, flags: 0 }.payload();
        p[2] ^= 0x01;
        assert_eq!(Command::decode(&p), Err(DecodeError::Crc));
    }

    #[test]
    fn out_of_range_rejected() {
        let mut p = Command { seq: 1, throttle: 0, turn: 0, grip: GripCmd::Keep, flags: 0 }.payload();
        p[2..4].copy_from_slice(&1001i16.to_le_bytes());
        let crc = crc8(&p[..p.len() - 1]);
        let last = p.len() - 1;
        p[last] = crc;
        assert_eq!(Command::decode(&p), Err(DecodeError::Value));
    }

    #[test]
    fn seq_wraparound_is_accepted() {
        for seq in [255u8, 0] {
            let cmd = Command { seq, throttle: 0, turn: 0, grip: GripCmd::Keep, flags: 0 };
            let mut out = [0u8; MAX_FRAME];
            let n = cmd.encode(&mut out);
            let mut dec = FrameDecoder::new();
            assert_eq!(Command::decode(feed(&mut dec, &out[..n]).unwrap()), Ok(cmd));
        }
    }

    #[test]
    fn decoder_recovers_after_garbage() {
        let mut dec = FrameDecoder::new();
        for _ in 0..100 {
            let _ = dec.push(0x13);
        }
        let cmd = Command { seq: 3, throttle: 5, turn: -5, grip: GripCmd::Open, flags: 0 };
        let mut out = [0u8; MAX_FRAME];
        let n = cmd.encode(&mut out);
        // the first 0x00 terminates the garbage; the next frame decodes cleanly
        let _ = dec.push(0);
        let (buf, len) = frame_of(&out[..n]);
        assert_eq!(Command::decode(feed(&mut dec, &buf[..len]).unwrap()), Ok(cmd));
    }
}
