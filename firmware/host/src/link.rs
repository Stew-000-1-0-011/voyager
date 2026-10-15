//! USB serial link to the robot: discovery, reconnect, framed send/receive.

use std::io::{ErrorKind, Read, Write};
use std::time::Duration;

use robot_core::protocol::{Command, FrameDecoder, MAX_FRAME, Telemetry};

pub const USB_VID: u16 = 0x16C0;
pub const USB_PID: u16 = 0x27DD;

/// First USB serial port with the robot's VID/PID.
pub fn find_port() -> Option<String> {
    serialport::available_ports().ok()?.into_iter().find_map(|p| match p.port_type {
        serialport::SerialPortType::UsbPort(u) if u.vid == USB_VID && u.pid == USB_PID => Some(p.port_name),
        _ => None,
    })
}

pub struct Link {
    port: Box<dyn serialport::SerialPort>,
    decoder: FrameDecoder,
}

impl Link {
    pub fn open(path: &str) -> anyhow::Result<Self> {
        let port = serialport::new(path, 115_200).timeout(Duration::from_millis(1)).open()?;
        Ok(Self { port, decoder: FrameDecoder::new() })
    }

    pub fn send(&mut self, cmd: &Command) -> anyhow::Result<()> {
        let mut buf = [0u8; MAX_FRAME];
        let n = cmd.encode(&mut buf);
        self.port.write_all(&buf[..n])?;
        Ok(())
    }

    /// Drains available bytes; returns every telemetry frame decoded.
    pub fn receive(&mut self) -> anyhow::Result<Vec<Telemetry>> {
        let mut raw = [0u8; 256];
        let mut out = Vec::new();
        loop {
            let n = match self.port.read(&mut raw) {
                Ok(0) => break,
                Ok(n) => n,
                Err(e) if e.kind() == ErrorKind::TimedOut => break,
                Err(e) => return Err(e.into()),
            };
            for &b in &raw[..n] {
                if let Some(payload) = self.decoder.push(b) {
                    if let Ok(t) = Telemetry::decode(payload) {
                        out.push(t);
                    }
                }
            }
        }
        Ok(out)
    }
}
