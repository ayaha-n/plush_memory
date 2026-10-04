//! Handwritten text stages: lay out a sentence as pen strokes (via the
//! riddle-derived src/script.rs) and write it onto the panel a few points at
//! a time, each step a tiny UFAST partial update — the same way riddle has
//! Tom write his replies. Strokes are pure black on white, which is exactly
//! the content UFAST reveals without fading (see README's e-ink notes), so
//! there's no color-landing step for text.

use crate::script;
use ab_glyph::FontRef;

pub const FONT_TTF: &[u8] = include_bytes!("../fonts/Yomogi-Regular.ttf");

/// Pen radius in px — riddle's 2 reads as a fine nib at this panel density.
pub const PEN_R: i32 = 2;
/// Points drawn per partial update, and the pause between updates. Same
/// pacing riddle uses for Tom's live replies (26 points / 14ms).
pub const POINTS_PER_STEP: usize = 26;
pub const STEP_MS: u64 = 14;

const LINE_SPACING: f32 = 1.5;

/// Screen-space strokes for `text`, word-wrapped into a box `w` wide whose
/// top-left is (x, y). Lines are left-aligned, like a picture book's text
/// block. Returns the strokes plus the y just below the last line.
pub fn plan(font: &FontRef, text: &str, x: i32, y: i32, w: i32, px: f32) -> (Vec<Vec<(i32, i32)>>, i32) {
    let lines = script::wrap(font, text, px, w as f32);
    let line_h = (px * LINE_SPACING) as i32;
    let mut strokes = Vec::new();
    let mut cy = y;
    for line_text in &lines {
        let mut raster = script::rasterize_line(font, line_text, px);
        script::thin(&mut raster);
        for s in script::trace(&raster) {
            strokes.push(s.iter().map(|&(sx, sy)| (x + sx, cy + sy)).collect());
        }
        cy += line_h;
    }
    (strokes, cy)
}

/// Dirty-rectangle accumulator for one partial update.
#[derive(Default)]
pub struct Dirty {
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
    any: bool,
}

impl Dirty {
    fn add(&mut self, x: i32, y: i32, r: i32) {
        if !self.any {
            (self.x0, self.y0, self.x1, self.y1, self.any) = (x - r, y - r, x + r, y + r, true);
        } else {
            self.x0 = self.x0.min(x - r);
            self.y0 = self.y0.min(y - r);
            self.x1 = self.x1.max(x + r);
            self.y1 = self.y1.max(y + r);
        }
    }

    /// (x, y, w, h), clipped to the panel.
    pub fn rect(&self, screen_w: usize, screen_h: usize) -> Option<(i32, i32, i32, i32)> {
        if !self.any {
            return None;
        }
        let x0 = self.x0.max(0);
        let y0 = self.y0.max(0);
        let x1 = self.x1.min(screen_w as i32 - 1);
        let y1 = self.y1.min(screen_h as i32 - 1);
        (x1 >= x0 && y1 >= y0).then(|| (x0, y0, x1 - x0 + 1, y1 - y0 + 1))
    }
}

/// Walks a stroke plan, drawing up to POINTS_PER_STEP points per call.
pub struct Writer {
    strokes: Vec<Vec<(i32, i32)>>,
    stroke_i: usize,
    point_i: usize,
}

impl Writer {
    pub fn new(strokes: Vec<Vec<(i32, i32)>>) -> Self {
        Writer { strokes, stroke_i: 0, point_i: 0 }
    }

    pub fn done(&self) -> bool {
        self.stroke_i >= self.strokes.len()
    }

    /// Draw the next few points into the RGB565 framebuffer and return the
    /// region that changed.
    pub fn step(&mut self, fb: &mut [u8], screen_w: usize, screen_h: usize) -> Dirty {
        let mut dirty = Dirty::default();
        let mut budget = POINTS_PER_STEP;
        while budget > 0 && !self.done() {
            let stroke = &self.strokes[self.stroke_i];
            if self.point_i >= stroke.len() {
                self.stroke_i += 1;
                self.point_i = 0;
                continue;
            }
            let (x, y) = stroke[self.point_i];
            if self.point_i > 0 {
                let (px, py) = stroke[self.point_i - 1];
                brush_line(fb, screen_w, screen_h, px, py, x, y);
            } else {
                stamp(fb, screen_w, screen_h, x, y);
            }
            dirty.add(x, y, PEN_R + 2);
            self.point_i += 1;
            budget -= 1;
        }
        dirty
    }
}

fn put_black(fb: &mut [u8], screen_w: usize, screen_h: usize, x: i32, y: i32) {
    if x < 0 || y < 0 || x as usize >= screen_w || y as usize >= screen_h {
        return;
    }
    let i = (y as usize * screen_w + x as usize) * 2;
    fb[i..i + 2].copy_from_slice(&0u16.to_le_bytes());
}

// stamp/brush_line: same round-nib brush as riddle's src/surface.rs.
fn stamp(fb: &mut [u8], sw: usize, sh: usize, cx: i32, cy: i32) {
    let r = PEN_R;
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                put_black(fb, sw, sh, cx + dx, cy + dy);
            }
        }
    }
}

fn brush_line(fb: &mut [u8], sw: usize, sh: usize, x0: i32, y0: i32, x1: i32, y1: i32) {
    let steps = (x1 - x0).abs().max((y1 - y0).abs()).max(1);
    for i in 0..=steps {
        stamp(fb, sw, sh, x0 + (x1 - x0) * i / steps, y0 + (y1 - y0) * i / steps);
    }
}
