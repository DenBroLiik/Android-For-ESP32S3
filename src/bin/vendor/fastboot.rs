use core::fmt::Write;

use esp_hal::{Blocking, usb_serial_jtag::UsbSerialJtag};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FastbootCommand {
    Help,
    Status,
    Pins,
    Memory,
    Rtc,
    LogsOn,
    LogsOff,
    Boot,
    Reboot,
    LedOff,
    LedError,
    LedSuccess,
    LedStartup,
    LedFastboot,
    LedWrite,
    Unknown,
}

pub struct FastbootPlus<'d> {
    console: UsbSerialJtag<'d, Blocking>,
    line: [u8; 64],
    len: usize,
}

impl<'d> FastbootPlus<'d> {
    pub fn new(console: UsbSerialJtag<'d, Blocking>) -> Self {
        Self {
            console,
            line: [0; 64],
            len: 0,
        }
    }

    pub fn print_banner(&mut self) {
        self.write_line("");
        self.write_line("fastboot+ ready");
        self.write_line("Type `help` for commands.");
        self.write_prompt();
    }

    pub fn print_help(&mut self) {
        self.write_line("Commands:");
        self.write_line("  help         - list commands");
        self.write_line("  status       - board/component status");
        self.write_line("  pins         - configured pin map");
        self.write_line("  memory       - memory summary");
        self.write_line("  rtc          - current RTC date/time");
        self.write_line("  logs on      - enable live fastboot logs");
        self.write_line("  logs off     - disable live fastboot logs");
        self.write_line("  led off      - turn status LED off");
        self.write_line("  led error    - red error indication");
        self.write_line("  led success  - green success indication");
        self.write_line("  led startup  - blue startup indication");
        self.write_line("  led fastboot - amber fastboot indication");
        self.write_line("  led write    - yellow write indication");
        self.write_line("  boot         - try continuing boot");
        self.write_line("  reboot       - software reset");
        self.write_prompt();
    }

    pub fn write_line(&mut self, line: &str) {
        let _ = writeln!(self.console, "{line}");
        let _ = self.console.flush_tx();
    }

    pub fn write_fmt_line(&mut self, args: core::fmt::Arguments<'_>) {
        let _ = self.console.write_fmt(args);
        let _ = writeln!(self.console);
        let _ = self.console.flush_tx();
    }

    pub fn write_prompt(&mut self) {
        let _ = write!(self.console, "fastboot+> ");
        let _ = self.console.flush_tx();
    }

    pub fn poll_command(&mut self) -> Option<FastbootCommand> {
        while let Ok(byte) = self.console.read_byte() {
            match byte {
                b'\r' | b'\n' => {
                    if self.len == 0 {
                        self.write_prompt();
                        continue;
                    }

                    let command = parse_command(&self.line[..self.len]);
                    self.len = 0;
                    return Some(command);
                }
                0x08 | 0x7F => {
                    self.len = self.len.saturating_sub(1);
                }
                byte if (byte == b' ' || byte.is_ascii_graphic()) && self.len < self.line.len() => {
                    self.line[self.len] = byte;
                    self.len += 1;
                }
                _ => {
                    // Ignore non-printable noise from the monitor transport.
                }
            }
        }

        None
    }
}

fn parse_command(raw: &[u8]) -> FastbootCommand {
    let command = trim_ascii(raw);

    match command {
        b"help" => FastbootCommand::Help,
        b"status" => FastbootCommand::Status,
        b"pins" => FastbootCommand::Pins,
        b"memory" => FastbootCommand::Memory,
        b"rtc" => FastbootCommand::Rtc,
        b"logs on" => FastbootCommand::LogsOn,
        b"logs off" => FastbootCommand::LogsOff,
        b"boot" => FastbootCommand::Boot,
        b"reboot" => FastbootCommand::Reboot,
        b"led off" => FastbootCommand::LedOff,
        b"led error" => FastbootCommand::LedError,
        b"led success" => FastbootCommand::LedSuccess,
        b"led startup" => FastbootCommand::LedStartup,
        b"led fastboot" => FastbootCommand::LedFastboot,
        b"led write" => FastbootCommand::LedWrite,
        _ => FastbootCommand::Unknown,
    }
}

fn trim_ascii(bytes: &[u8]) -> &[u8] {
    let mut start = 0;
    let mut end = bytes.len();

    while start < end && bytes[start].is_ascii_whitespace() {
        start += 1;
    }

    while end > start && bytes[end - 1].is_ascii_whitespace() {
        end -= 1;
    }

    &bytes[start..end]
}
