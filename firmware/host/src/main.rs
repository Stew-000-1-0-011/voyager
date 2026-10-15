//! Teleop: gamepad -> Command at 50Hz over USB serial; telemetry shown and logged to CSV.

mod link;
mod log;
mod mapping;

use std::io::Write;
use std::path::PathBuf;
use std::thread::sleep;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::Context;
use gilrs::{Axis, Button, Gilrs};

use link::{Link, find_port};
use mapping::{Mapper, Mapping, PadState};

const PERIOD: Duration = Duration::from_millis(20);
const RECONNECT: Duration = Duration::from_millis(500);
const DISPLAY: Duration = Duration::from_millis(100);

const BUTTONS: [Button; 19] = [
    Button::South, Button::East, Button::North, Button::West, Button::C, Button::Z,
    Button::LeftTrigger, Button::LeftTrigger2, Button::RightTrigger, Button::RightTrigger2,
    Button::Select, Button::Start, Button::Mode, Button::LeftThumb, Button::RightThumb,
    Button::DPadUp, Button::DPadDown, Button::DPadLeft, Button::DPadRight,
];

fn button(name: &str) -> anyhow::Result<Button> {
    BUTTONS.into_iter().find(|b| format!("{b:?}") == name).with_context(|| format!("unknown button {name:?}"))
}

struct Args {
    port: Option<String>,
    config: PathBuf,
}

fn parse_args() -> anyhow::Result<Args> {
    let mut args = Args { port: None, config: PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/config.toml")) };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--port" => args.port = Some(it.next().context("--port needs a path")?),
            "--config" => args.config = it.next().context("--config needs a path")?.into(),
            other => anyhow::bail!("unknown argument {other:?} (use --port <path> --config <path>)"),
        }
    }
    Ok(args)
}

fn read_pad(gilrs: &Gilrs, b: &[Button; 4]) -> PadState {
    let Some((_, pad)) = gilrs.gamepads().next() else {
        return PadState::default();
    };
    PadState {
        left_y: pad.value(Axis::LeftStickY),
        right_x: pad.value(Axis::RightStickX),
        close: pad.is_pressed(b[0]),
        open: pad.is_pressed(b[1]),
        slow_toggle: pad.is_pressed(b[2]),
        estop_release: pad.is_pressed(b[3]),
    }
}

fn main() -> anyhow::Result<()> {
    let args = parse_args()?;
    let mapping: Mapping = toml::from_str(&std::fs::read_to_string(&args.config)?)
        .with_context(|| format!("reading {}", args.config.display()))?;
    let buttons = [button(&mapping.close)?, button(&mapping.open)?, button(&mapping.slow_toggle)?,
                   button(&mapping.estop_release)?];
    let mut gilrs = Gilrs::new().map_err(|e| anyhow::anyhow!("gamepad init: {e}"))?;
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH)?.as_secs();
    let mut csv = log::CsvLog::create(&PathBuf::from("logs"), stamp)?;

    let start = Instant::now();
    let mut mapper = Mapper::default();
    let mut link: Option<Link> = None;
    let mut last_try = Instant::now() - RECONNECT;
    let mut last_display = Instant::now();
    let mut last_telemetry = None;

    loop {
        let tick = Instant::now();
        while gilrs.next_event().is_some() {}

        if link.is_none() && last_try.elapsed() >= RECONNECT {
            last_try = Instant::now();
            if let Some(path) = args.port.clone().or_else(find_port) {
                link = Link::open(&path).ok();
            }
        }

        let cmd = mapper.command(&read_pad(&gilrs, &buttons), &mapping);
        if let Some(l) = link.as_mut() {
            let result = l.send(&cmd).and_then(|_| l.receive());
            match result {
                Ok(frames) => {
                    for t in frames {
                        csv.write(start.elapsed().as_millis(), &t)?;
                        last_telemetry = Some(t);
                    }
                }
                Err(_) => link = None,
            }
        }

        if last_display.elapsed() >= DISPLAY {
            last_display = Instant::now();
            print!("\r{:<110}", log::status_line(link.is_some(), mapper.slow, last_telemetry.as_ref()));
            std::io::stdout().flush()?;
        }
        if let Some(rest) = PERIOD.checked_sub(tick.elapsed()) {
            sleep(rest);
        }
    }
}
