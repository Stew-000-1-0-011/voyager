//! One-line status display and CSV telemetry log.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use robot_core::protocol::Telemetry;

pub const CSV_HEADER: &str = "host_ms,seq,battery_mv,enc_left,enc_right,enc_grip,grip_state,box_count,\
sonar_lf,sonar_rf,sonar_lr,sonar_rr,estop_latched,faults";

pub fn csv_row(host_ms: u128, t: &Telemetry) -> String {
    let s = t.sonar_mm;
    format!(
        "{host_ms},{},{},{},{},{},{:?},{},{},{},{},{},{},{}",
        t.seq, t.battery_mv, t.enc_left, t.enc_right, t.enc_grip, t.grip_state, t.box_count, s[0], s[1], s[2], s[3],
        t.estop_latched as u8, t.faults
    )
}

pub fn status_line(connected: bool, slow: bool, t: Option<&Telemetry>) -> String {
    let link = if connected { "LINK" } else { "----" };
    let mode = if slow { "SLOW" } else { "FAST" };
    match t {
        None => format!("{link} {mode} (no telemetry)"),
        Some(t) => format!(
            "{link} {mode} bat {:5.2}V grip {:?}/{} box estop {} faults 0x{:02x} sonar {:?}",
            t.battery_mv as f32 / 1000.0,
            t.grip_state,
            t.box_count,
            if t.estop_latched { "LATCHED" } else { "ok" },
            t.faults,
            t.sonar_mm
        ),
    }
}

pub struct CsvLog {
    out: BufWriter<File>,
}

impl CsvLog {
    pub fn create(dir: &Path, stamp: u64) -> std::io::Result<Self> {
        fs::create_dir_all(dir)?;
        let mut out = BufWriter::new(File::create(dir.join(format!("telemetry-{stamp}.csv")))?);
        writeln!(out, "{CSV_HEADER}")?;
        Ok(Self { out })
    }

    pub fn write(&mut self, host_ms: u128, t: &Telemetry) -> std::io::Result<()> {
        writeln!(self.out, "{}", csv_row(host_ms, t))
    }
}
