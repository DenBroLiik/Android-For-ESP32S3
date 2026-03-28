use esp_hal::{
    Blocking,
    i2c::master::I2c,
};

const SSD1306_ADDR: u8 = 0x3C;
const WIDTH: usize = 128;
const HEIGHT: usize = 64;
const PAGES: usize = HEIGHT / 8;

pub struct Ssd1306Display<'d> {
    i2c: I2c<'d, Blocking>,
    framebuffer: [u8; WIDTH * PAGES],
}

impl<'d> Ssd1306Display<'d> {
    pub fn new(mut i2c: I2c<'d, Blocking>) -> Result<Self, ()> {
        let init_sequence = [
            0xAE, 0x20, 0x00, 0x40, 0xA1, 0xC8, 0x81, 0x7F, 0xA6, 0xA8, 0x3F, 0xD3, 0x00, 0xD5,
            0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x40, 0x8D, 0x14, 0xAF,
        ];

        for command in init_sequence {
            write_command(&mut i2c, command)?;
        }

        let mut display = Self {
            i2c,
            framebuffer: [0u8; WIDTH * PAGES],
        };
        display.clear()?;
        Ok(display)
    }

    pub fn clear(&mut self) -> Result<(), ()> {
        self.framebuffer.fill(0);
        self.flush()
    }

    pub fn show_lines(&mut self, lines: [&str; 4]) -> Result<(), ()> {
        self.framebuffer.fill(0);
        for (row, line) in lines.iter().enumerate() {
            self.draw_text(0, (row as i16) * 16, line, 2, true);
        }
        self.flush()
    }

    pub fn show_fastboot_logo(&mut self) -> Result<(), ()> {
        self.framebuffer.fill(0);

        let title = "FASTBOOT";
        let title_scale = 2;
        let title_width = text_width(title, title_scale, title_scale as i16);
        let title_x = ((WIDTH as i16 - title_width).max(0)) / 2;
        let title_y = 18;

        self.draw_text(title_x + 2, title_y + 2, title, title_scale, false);
        self.draw_text(title_x, title_y, title, title_scale, true);

        let subtitle = "Zephyr";
        let subtitle_scale = 1;
        let subtitle_spacing = 0;
        let subtitle_width = text_width(subtitle, subtitle_scale, subtitle_spacing);
        let subtitle_x = ((WIDTH as i16 - subtitle_width).max(0)) / 2;
        let subtitle_y = 54;

        self.draw_text_with_spacing(
            subtitle_x,
            subtitle_y,
            subtitle,
            subtitle_scale,
            subtitle_spacing,
            true,
        );

        self.flush()
    }

    fn draw_text(&mut self, x: i16, y: i16, text: &str, scale: u8, on: bool) {
        self.draw_text_with_spacing(x, y, text, scale, scale as i16, on);
    }

    fn draw_text_with_spacing(
        &mut self,
        x: i16,
        y: i16,
        text: &str,
        scale: u8,
        spacing: i16,
        on: bool,
    ) {
        let mut cursor_x = x;
        let step = (5 * scale as i16) + spacing;

        for ch in text.bytes().take(21) {
            self.draw_glyph(cursor_x, y, ch.to_ascii_uppercase(), scale, on);
            cursor_x += step;
        }
    }

    fn draw_glyph(&mut self, x: i16, y: i16, ch: u8, scale: u8, on: bool) {
        let glyph = glyph(ch);
        let scale = scale.max(1) as i16;

        for (column_index, column) in glyph.iter().enumerate() {
            for row in 0..7 {
                if (column >> row) & 0x01 == 0 {
                    continue;
                }

                let base_x = x + (column_index as i16 * scale);
                let base_y = y + (row as i16 * scale);

                for dx in 0..scale {
                    for dy in 0..scale {
                        self.set_pixel(base_x + dx, base_y + dy, on);
                    }
                }
            }
        }
    }

    fn set_pixel(&mut self, x: i16, y: i16, on: bool) {
        if x < 0 || y < 0 || x >= WIDTH as i16 || y >= HEIGHT as i16 {
            return;
        }

        let x = x as usize;
        let y = y as usize;
        let index = x + (y / 8) * WIDTH;
        let mask = 1u8 << (y % 8);

        if on {
            self.framebuffer[index] |= mask;
        } else {
            self.framebuffer[index] &= !mask;
        }
    }

    fn flush(&mut self) -> Result<(), ()> {
        for page in 0..PAGES {
            self.set_cursor(page as u8, 0)?;

            for chunk in self.framebuffer[page * WIDTH..(page + 1) * WIDTH].chunks(16) {
                let mut packet = [0u8; 17];
                packet[0] = 0x40;
                packet[1..1 + chunk.len()].copy_from_slice(chunk);
                self.i2c
                    .write(SSD1306_ADDR, &packet[..1 + chunk.len()])
                    .map_err(|_| ())?;
            }
        }

        Ok(())
    }

    fn set_cursor(&mut self, page: u8, column: u8) -> Result<(), ()> {
        write_command(&mut self.i2c, 0xB0 | (page & 0x07))?;
        write_command(&mut self.i2c, column & 0x0F)?;
        write_command(&mut self.i2c, 0x10 | ((column >> 4) & 0x0F))?;
        Ok(())
    }
}

fn write_command(i2c: &mut I2c<'_, Blocking>, command: u8) -> Result<(), ()> {
    i2c.write(SSD1306_ADDR, &[0x00, command]).map_err(|_| ())
}

fn text_width(text: &str, scale: u8, spacing: i16) -> i16 {
    let chars = text.chars().take(21).count() as i16;
    if chars == 0 {
        0
    } else {
        chars * ((5 * scale as i16) + spacing) - spacing
    }
}

fn glyph(ch: u8) -> [u8; 5] {
    match ch {
        b' ' => [0x00, 0x00, 0x00, 0x00, 0x00],
        b'+' => [0x08, 0x08, 0x3E, 0x08, 0x08],
        b'-' => [0x08, 0x08, 0x08, 0x08, 0x08],
        b'.' => [0x00, 0x60, 0x60, 0x00, 0x00],
        b'/' => [0x20, 0x10, 0x08, 0x04, 0x02],
        b':' => [0x00, 0x36, 0x36, 0x00, 0x00],
        b'0' => [0x3E, 0x51, 0x49, 0x45, 0x3E],
        b'1' => [0x00, 0x42, 0x7F, 0x40, 0x00],
        b'2' => [0x62, 0x51, 0x49, 0x49, 0x46],
        b'3' => [0x22, 0x41, 0x49, 0x49, 0x36],
        b'4' => [0x18, 0x14, 0x12, 0x7F, 0x10],
        b'5' => [0x2F, 0x49, 0x49, 0x49, 0x31],
        b'6' => [0x3E, 0x49, 0x49, 0x49, 0x32],
        b'7' => [0x01, 0x71, 0x09, 0x05, 0x03],
        b'8' => [0x36, 0x49, 0x49, 0x49, 0x36],
        b'9' => [0x26, 0x49, 0x49, 0x49, 0x3E],
        b'A' => [0x7E, 0x11, 0x11, 0x11, 0x7E],
        b'B' => [0x7F, 0x49, 0x49, 0x49, 0x36],
        b'C' => [0x3E, 0x41, 0x41, 0x41, 0x22],
        b'D' => [0x7F, 0x41, 0x41, 0x22, 0x1C],
        b'E' => [0x7F, 0x49, 0x49, 0x49, 0x41],
        b'F' => [0x7F, 0x09, 0x09, 0x09, 0x01],
        b'G' => [0x3E, 0x41, 0x49, 0x49, 0x7A],
        b'H' => [0x7F, 0x08, 0x08, 0x08, 0x7F],
        b'I' => [0x00, 0x41, 0x7F, 0x41, 0x00],
        b'J' => [0x20, 0x40, 0x41, 0x3F, 0x01],
        b'K' => [0x7F, 0x08, 0x14, 0x22, 0x41],
        b'L' => [0x7F, 0x40, 0x40, 0x40, 0x40],
        b'M' => [0x7F, 0x02, 0x0C, 0x02, 0x7F],
        b'N' => [0x7F, 0x04, 0x08, 0x10, 0x7F],
        b'O' => [0x3E, 0x41, 0x41, 0x41, 0x3E],
        b'P' => [0x7F, 0x09, 0x09, 0x09, 0x06],
        b'Q' => [0x3E, 0x41, 0x51, 0x21, 0x5E],
        b'R' => [0x7F, 0x09, 0x19, 0x29, 0x46],
        b'S' => [0x26, 0x49, 0x49, 0x49, 0x32],
        b'T' => [0x01, 0x01, 0x7F, 0x01, 0x01],
        b'U' => [0x3F, 0x40, 0x40, 0x40, 0x3F],
        b'V' => [0x1F, 0x20, 0x40, 0x20, 0x1F],
        b'W' => [0x7F, 0x20, 0x18, 0x20, 0x7F],
        b'X' => [0x63, 0x14, 0x08, 0x14, 0x63],
        b'Y' => [0x03, 0x04, 0x78, 0x04, 0x03],
        b'Z' => [0x61, 0x51, 0x49, 0x45, 0x43],
        _ => [0x00, 0x00, 0x5F, 0x00, 0x00],
    }
}
