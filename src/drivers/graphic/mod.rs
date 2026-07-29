// drivers/graphic/mod.rs
// Общий монохромный framebuffer + 2D-примитивы, никаких аллокаций.
// Организация памяти — страничная (как в SSD1306): 8 строк на байт, LSB сверху.
// Размер для 128x64: 128 * 64 / 8 = 1024 байта.

pub mod font;

use font::glyph;

/// `W`, `H` — ширина/высота в пикселях. `BYTES` должен быть равен `W * H / 8`
/// (проверяется `const`-ассертом в `new()`, ошибка размеров ловится на этапе сборки).
pub struct Framebuffer<const W: usize, const H: usize, const BYTES: usize> {
    data: [u8; BYTES],
}

impl<const W: usize, const H: usize, const BYTES: usize> Framebuffer<W, H, BYTES> {
    pub const fn new() -> Self {
        assert!(H % 8 == 0, "height must be a multiple of 8 (page-based layout)");
        assert!(BYTES == W * (H / 8), "BYTES must equal W * H / 8");
        Self { data: [0u8; BYTES] }
    }

    pub const fn width(&self) -> usize { W }
    pub const fn height(&self) -> usize { H }
    pub const fn pages(&self) -> usize { H / 8 }

    /// Сырой буфер целиком (для DMA / поблочной передачи).
    pub fn as_bytes(&self) -> &[u8] { &self.data }

    /// Одна страница (8 строк) — то, что SSD1306 принимает за один кадр.
    pub fn page_bytes(&self, page: usize) -> &[u8] {
        &self.data[page * W..(page + 1) * W]
    }

    // ── Базовые операции ──────────────────────────────────────────────

    pub fn clear(&mut self) { self.data.fill(0x00); }
    pub fn fill(&mut self)  { self.data.fill(0xFF); }

    pub fn set_pixel(&mut self, x: i32, y: i32, on: bool) {
        if x < 0 || y < 0 || x >= W as i32 || y >= H as i32 { return; }
        let (x, y) = (x as usize, y as usize);
        let idx = x + (y / 8) * W;
        let mask = 1u8 << (y % 8);
        if on { self.data[idx] |= mask; } else { self.data[idx] &= !mask; }
    }

    pub fn get_pixel(&self, x: i32, y: i32) -> bool {
        if x < 0 || y < 0 || x >= W as i32 || y >= H as i32 { return false; }
        let (x, y) = (x as usize, y as usize);
        let idx = x + (y / 8) * W;
        (self.data[idx] >> (y % 8)) & 1 != 0
    }

    // ── Линии и фигуры ─────────────────────────────────────────────────

    pub fn draw_hline(&mut self, x: i32, y: i32, w: i32, on: bool) {
        for i in 0..w { self.set_pixel(x + i, y, on); }
    }

    pub fn draw_vline(&mut self, x: i32, y: i32, h: i32, on: bool) {
        for i in 0..h { self.set_pixel(x, y + i, on); }
    }

    /// Линия по Brezenham—любые две точки, без аллокаций.
    pub fn draw_line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, on: bool) {
        let (mut x0, mut y0) = (x0, y0);
        let dx = (x1 - x0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let dy = -(y1 - y0).abs();
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            self.set_pixel(x0, y0, on);
            if x0 == x1 && y0 == y1 { break; }
            let e2 = 2 * err;
            if e2 >= dy { err += dy; x0 += sx; }
            if e2 <= dx { err += dx; y0 += sy; }
        }
    }

    /// Контур прямоугольника.
    pub fn draw_rect(&mut self, x: i32, y: i32, w: i32, h: i32, on: bool) {
        if w <= 0 || h <= 0 { return; }
        self.draw_hline(x, y, w, on);
        self.draw_hline(x, y + h - 1, w, on);
        self.draw_vline(x, y, h, on);
        self.draw_vline(x + w - 1, y, h, on);
    }

    /// Залитый прямоугольник.
    pub fn fill_rect(&mut self, x: i32, y: i32, w: i32, h: i32, on: bool) {
        for row in 0..h { self.draw_hline(x, y + row, w, on); }
    }

    /// Контур окружности (midpoint circle algorithm).
    pub fn draw_circle(&mut self, cx: i32, cy: i32, r: i32, on: bool) {
        let mut x = r;
        let mut y = 0;
        let mut err = 0i32;
        while x >= y {
            self.set_pixel(cx + x, cy + y, on);
            self.set_pixel(cx + y, cy + x, on);
            self.set_pixel(cx - y, cy + x, on);
            self.set_pixel(cx - x, cy + y, on);
            self.set_pixel(cx - x, cy - y, on);
            self.set_pixel(cx - y, cy - x, on);
            self.set_pixel(cx + y, cy - x, on);
            self.set_pixel(cx + x, cy - y, on);
            y += 1;
            if err <= 0 { err += 2 * y + 1; }
            if err > 0  { x -= 1; err -= 2 * x + 1; }
        }
    }

    /// Залитая окружность.
    pub fn fill_circle(&mut self, cx: i32, cy: i32, r: i32, on: bool) {
        let mut x = r;
        let mut y = 0;
        let mut err = 0i32;
        while x >= y {
            self.draw_hline(cx - x, cy + y, 2 * x + 1, on);
            self.draw_hline(cx - x, cy - y, 2 * x + 1, on);
            self.draw_hline(cx - y, cy + x, 2 * y + 1, on);
            self.draw_hline(cx - y, cy - x, 2 * y + 1, on);
            y += 1;
            if err <= 0 { err += 2 * y + 1; }
            if err > 0  { x -= 1; err -= 2 * x + 1; }
        }
    }

    // ── Текст ───────────────────────────────────────────────────────────────────

    /// Один символ 5x7 (масштабируемый, без аллокаций — через fill_rect на каждый бит).
    pub fn draw_glyph(&mut self, x: i32, y: i32, ch: u8, scale: i32, on: bool) {
        let g = glyph(ch);
        let scale = scale.max(1);
        for (col_idx, col) in g.iter().enumerate() {
            for row in 0..7 {
                if (col >> row) & 0x01 == 0 { continue; }
                let bx = x + col_idx as i32 * scale;
                let by = y + row as i32 * scale;
                self.fill_rect(bx, by, scale, scale, on);
            }
        }
    }

    /// Строка с шагом между символами равным scale (как в оригинальном draw_text).
    pub fn draw_text(&mut self, x: i32, y: i32, text: &str, scale: i32, on: bool) {
        self.draw_text_spaced(x, y, text, scale, scale, on);
    }

    /// Строка с произвольным интервалом между символами.
    pub fn draw_text_spaced(&mut self, x: i32, y: i32, text: &str, scale: i32, spacing: i32, on: bool) {
        let mut cx = x;
        let step = 5 * scale + spacing;
        for ch in text.bytes() {
            self.draw_glyph(cx, y, ch.to_ascii_uppercase(), scale, on);
            cx += step;
        }
    }

    /// Ширина строки в пикселях при данном scale/spacing — для центрирования.
    pub fn text_width(text: &str, scale: i32, spacing: i32) -> i32 {
        let n = text.len() as i32;
        if n == 0 { 0 } else { n * (5 * scale + spacing) - spacing }
    }
}