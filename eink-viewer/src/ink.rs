//! Handwritten text stages: lay out a sentence as pen strokes (via the
//! riddle-derived src/script.rs) and write it onto the panel a few points at
//! a time, each step a tiny UFAST partial update — the same way riddle has
//! Tom write his replies. Strokes are pure black on white, which is exactly
//! the content UFAST reveals without fading (see README's e-ink notes), so
//! there's no color-landing step for text.

use crate::script;
use ab_glyph::FontRef;

pub const FONT_TTF: &[u8] = include_bytes!("../fonts/Yomogi-Regular.ttf");

/// Default pen radius in px — riddle's 2 reads as a fine nib at this panel
/// density. A text stage can ask for a broader one (a cover's title).
pub const PEN_R: i32 = 2;
/// Default points drawn per partial update, and the pause between updates —
/// the pacing riddle uses for Tom's live replies (26 points / 14ms). A text
/// stage can override both (see Stage::Text in main.rs).
pub const POINTS_PER_STEP: usize = 26;
pub const STEP_MS: u64 = 14;

const LINE_SPACING: f32 = 1.5;

/// Glyphs are rasterized at this multiple of their size, thinned, and the
/// traced points scaled back down. Zhang-Suen thinning erases a diagonal
/// that is only 2px thick outright — at 56px Yomogi's strokes are about
/// that thin, so e.g. the lower half of く vanished. Supersampling, plus a
/// 1px dilation for the hairline tails, makes every stroke thick enough to
/// leave a skeleton.
const SUPERSAMPLE: f32 = 3.0;

/// Screen-space strokes for `text`, word-wrapped into a box `w` wide whose
/// top-left is (x, y). Lines are left-aligned, like a picture book's text
/// block, or centered in the box (a cover's title). Returns the strokes plus
/// the y just below the last line.
pub fn plan(font: &FontRef, text: &str, x: i32, y: i32, w: i32, px: f32, center: bool) -> (Vec<Vec<(i32, i32)>>, i32) {
    let lines = script::wrap(font, text, px, w as f32);
    let line_h = (px * LINE_SPACING) as i32;
    let mut strokes = Vec::new();
    let mut cy = y;
    for line_text in &lines {
        let x = if center { x + ((w as f32 - script::measure(font, line_text, px)) / 2.0).max(0.0) as i32 } else { x };
        let mut raster = script::rasterize_line(font, line_text, px * SUPERSAMPLE);
        dilate(&mut raster);
        script::thin(&mut raster);
        for s in script::trace(&raster) {
            let mut pts: Vec<(i32, i32)> = Vec::with_capacity(s.len() / 2);
            for &(sx, sy) in &s {
                let p = (x + (sx as f32 / SUPERSAMPLE).round() as i32, cy + (sy as f32 / SUPERSAMPLE).round() as i32);
                if pts.last() != Some(&p) {
                    pts.push(p);
                }
            }
            strokes.push(pts);
        }
        cy += line_h;
    }
    (strokes, cy)
}

/// Grow the inked mask by one pixel (8-neighborhood).
fn dilate(line: &mut script::Line) {
    let (w, h) = (line.width, line.height);
    let src = line.mask.clone();
    for y in 0..h {
        for x in 0..w {
            if src[y * w + x] {
                continue;
            }
            let ys = y.saturating_sub(1)..(y + 2).min(h);
            line.mask[y * w + x] = ys.into_iter().any(|ny| {
                (x.saturating_sub(1)..(x + 2).min(w)).any(|nx| src[ny * w + nx])
            });
        }
    }
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

/// Walks a stroke plan, drawing up to `points_per_step` points per call
/// with a round nib `pen_r` px in radius.
pub struct Writer {
    strokes: Vec<Vec<(i32, i32)>>,
    stroke_i: usize,
    point_i: usize,
    points_per_step: usize,
    pen_r: i32,
}

impl Writer {
    pub fn new(strokes: Vec<Vec<(i32, i32)>>, points_per_step: usize, pen_r: i32) -> Self {
        Writer { strokes, stroke_i: 0, point_i: 0, points_per_step: points_per_step.max(1), pen_r: pen_r.max(1) }
    }

    pub fn done(&self) -> bool {
        self.stroke_i >= self.strokes.len()
    }

    /// Draw the next few points into the RGB565 framebuffer and return the
    /// region that changed.
    pub fn step(&mut self, fb: &mut [u8], screen_w: usize, screen_h: usize) -> Dirty {
        let mut dirty = Dirty::default();
        let mut budget = self.points_per_step;
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
                brush_line(fb, screen_w, screen_h, self.pen_r, px, py, x, y);
            } else {
                stamp(fb, screen_w, screen_h, self.pen_r, x, y);
            }
            dirty.add(x, y, self.pen_r + 2);
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
fn stamp(fb: &mut [u8], sw: usize, sh: usize, r: i32, cx: i32, cy: i32) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                put_black(fb, sw, sh, cx + dx, cy + dy);
            }
        }
    }
}

fn brush_line(fb: &mut [u8], sw: usize, sh: usize, r: i32, x0: i32, y0: i32, x1: i32, y1: i32) {
    let steps = (x1 - x0).abs().max((y1 - y0).abs()).max(1);
    for i in 0..=steps {
        stamp(fb, sw, sh, r, x0 + (x1 - x0) * i / steps, y0 + (y1 - y0) * i / steps);
    }
}
