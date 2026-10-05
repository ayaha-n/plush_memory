//! plush_memory e-ink viewer: a windowed AppLoad app (qtfb protocol, no
//! takeover — xochitl keeps running) that watches a directory for
//! "memory events" dropped there over SSH/scp by the PC-side ROS pipeline,
//! and blits each event's frames onto the panel as a sequence of small
//! partial-refresh updates (no full-screen flash), producing a soft
//! materializing appearance instead of a hard pop-in.
//!
//! The qtfb client (src/qtfb.rs) and the overall display-backend split are
//! adapted from MaximeRivest/riddle (MIT License):
//! https://github.com/MaximeRivest/riddle — see src/qtfb.rs and
//! src/display.rs there. Riddle proved this exact mechanism (connect to
//! AppLoad's qtfb socket, write RGB565 into the shared-memory framebuffer,
//! request a partial-region refresh) works on this device for exactly this
//! kind of "soft ink appearing without a full-panel flash" effect; we reuse
//! it rather than re-deriving the wire protocol.

mod ink;
mod qtfb;
mod script;

use ab_glyph::FontRef;
use qtfb::{
    QtfbClient, ServerEvent, FBFMT_RMPP_RGB565, REFRESH_MODE_UFAST, ROTATION_180, ROTATION_L90,
    ROTATION_R90,
};
use serde::Deserialize;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const SCREEN_W: usize = 1620;
const SCREEN_H: usize = 2160;
const POLL_INTERVAL: Duration = Duration::from_millis(150);
// A page whose page_end never came (PC side stopped mid-page) stops holding
// back a rotation after this long with nothing to draw.
const PAGE_TIMEOUT: Duration = Duration::from_secs(180);
// Fallback only, if an old manifest has no "bg" — see Manifest::bg below,
// which is the real source of truth (eink_memory_push.py's BG constant).
const DEFAULT_BG_RGB: (u8, u8, u8) = (255, 255, 255);

#[derive(Deserialize)]
struct Manifest {
    #[serde(default)]
    clear_first: bool,
    #[serde(default)]
    settle_after: bool,
    #[serde(default)]
    bg: Option<[u8; 3]>,
    /// With clear_first: pause after the blank before the first stage, so
    /// the panel finishes clearing before anything is drawn over it.
    #[serde(default)]
    clear_hold_ms: u64,
    stages: Vec<Stage>,
    /// "portrait"/"landscape": the orientation the PC laid this event out
    /// for. Skipped if the panel has been turned the other way since.
    #[serde(default)]
    layout: Option<String>,
    /// First event of a touch's page, and the (empty) last one: in between,
    /// a rotation waits — see `wanted_turns` in main().
    #[serde(default)]
    page_start: bool,
    #[serde(default)]
    page_end: bool,
}

/// One step of an event. Untagged: a stage with `image` blits a PNG frame;
/// one with `text` writes the sentence by hand into a box `w` wide at (x, y).
/// A text stage without `y` continues below the previous text stage, `gap`
/// glyph heights further down, so the PC side never has to guess how many
/// lines the wrapping produced.
#[derive(Deserialize)]
#[serde(untagged)]
enum Stage {
    Image {
        x: i32,
        y: i32,
        image: String,
        hold_ms: u64,
    },
    Text {
        x: i32,
        #[serde(default)]
        y: Option<i32>,
        #[serde(default)]
        gap: f32,
        w: i32,
        text: String,
        px: f32,
        #[serde(default)]
        hold_ms: u64,
        /// Writing pace: pen points per partial update, and ms between
        /// updates. Defaults are riddle's (ink::POINTS_PER_STEP / STEP_MS).
        #[serde(default)]
        points_per_step: Option<usize>,
        #[serde(default)]
        step_ms: Option<u64>,
    },
    /// Internal: Job queues this after the last stage when settle_after.
    #[serde(skip)]
    Settle,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("--preview") {
        preview(&args[2..]);
        return;
    }

    let key: i32 = std::env::var("QTFB_KEY")
        .expect("QTFB_KEY not set — this app must be launched by AppLoad in windowed mode")
        .parse()
        .expect("QTFB_KEY is not an integer");

    let client = QtfbClient::connect(key, FBFMT_RMPP_RGB565, SCREEN_W, SCREEN_H, 2)
        .expect("failed to connect to qtfb");
    // Tried REFRESH_MODE_FAST (mode 1, "balanced/text") expecting less
    // flicker at some speed cost, but empirically it was worse on both axes
    // for photographic content: it collapsed grayscale to near-B&W and still
    // caused a visible full-panel flash. UFAST (mode 0, "ink") is the one
    // that actually keeps gray levels and avoids flashing on this panel —
    // the brief positive/negative flicker it does show on a transition is
    // the real tradeoff to accept here.
    let _ = client.set_refresh_mode(REFRESH_MODE_UFAST);

    let events_dir = events_dir_path();
    fs::create_dir_all(&events_dir).expect("failed to create events dir");
    eprintln!("plush_memory_viewer: watching {}", events_dir.display());
    // Which way the panel is turned, for the PC side to lay pages out by
    // (eink_memory_push.read_orientation).
    let orientation_path = events_dir.with_file_name("orientation");
    // How the tablet itself is set down, written by the PC side
    // (~eink_orientation): portrait, landscape_cw or landscape_ccw.
    let orientation_conf_path = events_dir.with_file_name("orientation.conf");

    let font = FontRef::try_from_slice(ink::FONT_TTF).expect("bundled font");
    let mut screen = Screen::new(client);
    write_orientation(&orientation_path, screen.layout());
    let mut jobs: Vec<Job> = Vec::new();
    let mut last_poll = Instant::now() - POLL_INTERVAL;

    // Which way round to draw, in clockwise quarter turns, from two sources
    // that add up: the rotation AppLoad paints the window with (AppLoad
    // v0.6.0+ with "supportsRotation", following xochitl's auto-rotate;
    // always 0 on older AppLoad), and orientation.conf, for a tablet turned
    // on its side while xochitl stays upright (auto-rotate off) — the only
    // way to fill a sideways panel on older AppLoad, which always letterboxes
    // a sideways window to portrait.
    //
    // A change is only applied while the panel is at rest: nothing queued,
    // and no page half way through (a page's events can arrive many seconds
    // apart while an illustration generates). Until then the page keeps
    // being drawn the old way round, and the next page is laid out for the
    // new orientation.
    let mut appload_turns = 0;
    let mut conf_turns = read_orientation_conf(&orientation_conf_path);
    let mut in_page = false;
    let mut last_busy = Instant::now();

    // Each READY event becomes a Job, queued in name order (the PC side
    // names them so that sorts in push order). Only the front job runs —
    // one step per pass (one image stage, or one small batch of pen points)
    // — so the page is drawn in exactly the order it was sent: text, then
    // pictures, then the closing text.
    loop {
        // Drain input/window events; an Err means AppLoad closed our window.
        match screen.client.drain_events() {
            Err(_) => {
                eprintln!("plush_memory_viewer: window closed, exiting");
                break;
            }
            Ok(events) => {
                for ev in events {
                    if let ServerEvent::Rotation(r) = ev {
                        appload_turns = appload_rotation_turns(r);
                    }
                }
            }
        }

        if last_poll.elapsed() >= POLL_INTERVAL {
            last_poll = Instant::now();
            conf_turns = read_orientation_conf(&orientation_conf_path);
            for dir in ready_events(&events_dir) {
                if jobs.iter().any(|j| j.dir == dir) {
                    continue;
                }
                match Job::start(dir.clone()) {
                    Ok(job) => jobs.push(job),
                    Err(e) => {
                        eprintln!("plush_memory_viewer: event {:?} failed: {e}", dir);
                        let _ = fs::remove_dir_all(&dir);
                    }
                }
            }
        }

        let now = Instant::now();
        if let Some(job) = jobs.first_mut().filter(|j| j.next_at <= now) {
            if let Some(layout) = job.layout.as_deref().filter(|&l| l != screen.layout()) {
                eprintln!(
                    "plush_memory_viewer: event {:?} laid out {layout}, panel is {}; skipped",
                    job.dir,
                    screen.layout()
                );
                job.finished = true;
            } else {
                in_page |= std::mem::take(&mut job.page_start);
                if let Err(e) = job.step(&mut screen, &font) {
                    eprintln!("plush_memory_viewer: event {:?} failed: {e}", job.dir);
                    job.finished = true;
                }
            }
            if job.finished && job.page_end {
                in_page = false;
            }
        }
        jobs.retain(|j| {
            if j.finished {
                let _ = fs::remove_dir_all(&j.dir);
            }
            !j.finished
        });
        if !jobs.is_empty() {
            last_busy = now;
        }

        let wanted_turns = (appload_turns + conf_turns) % 4;
        if wanted_turns != screen.turns
            && jobs.is_empty()
            && (!in_page || last_busy.elapsed() >= PAGE_TIMEOUT)
        {
            in_page = false;
            screen.set_turns(wanted_turns);
            write_orientation(&orientation_path, screen.layout());
            eprintln!(
                "plush_memory_viewer: drawing {wanted_turns} quarter turn(s) round ({}; AppLoad {appload_turns}, orientation.conf {conf_turns})",
                screen.layout()
            );
        }

        let wake = jobs.first().map_or(now + POLL_INTERVAL, |j| j.next_at);
        let wait = wake.saturating_duration_since(Instant::now()).min(POLL_INTERVAL);
        std::thread::sleep(wait.max(Duration::from_millis(1)));
    }
}

fn write_orientation(path: &Path, layout: &str) {
    let tmp = path.with_extension("tmp");
    if let Err(e) = fs::write(&tmp, layout).and_then(|_| fs::rename(&tmp, path)) {
        eprintln!("plush_memory_viewer: could not write {:?}: {e}", path);
    }
}

/// orientation.conf -> clockwise quarter turns the panel is set down at.
/// Missing or unrecognized means upright.
fn read_orientation_conf(path: &Path) -> u8 {
    match fs::read_to_string(path).unwrap_or_default().trim() {
        "landscape_cw" => 1,
        "landscape_ccw" => 3,
        _ => 0,
    }
}

/// AppLoad's rotation -> clockwise quarter turns it paints the buffer at
/// (Qt painter rotate: L90 = -90deg, R90 = +90deg).
fn appload_rotation_turns(rotation: i32) -> u8 {
    match rotation {
        ROTATION_R90 => 1,
        ROTATION_180 => 2,
        ROTATION_L90 => 3,
        _ => 0,
    }
}

/// What jobs draw on: a canvas the size of the panel as the reader sees it
/// (1620x2160 upright, 2160x1620 on its side), copied out to the qtfb
/// framebuffer counter-rotated by `turns` (clockwise quarter turns the
/// reader sees the framebuffer at — see `wanted_turns` in main()), so a
/// sideways panel is filled edge to edge, the right way up.
struct Screen {
    client: QtfbClient,
    turns: u8,
    w: usize,
    h: usize,
    canvas: Vec<u8>,
}

impl Screen {
    fn new(mut client: QtfbClient) -> Screen {
        // Start from whatever is already in the buffer, as before rotation
        // support — nothing is blanked at launch.
        let canvas = client.framebuffer()[..SCREEN_W * SCREEN_H * 2].to_vec();
        Screen { client, turns: 0, w: SCREEN_W, h: SCREEN_H, canvas }
    }

    fn landscape(&self) -> bool {
        self.turns % 2 == 1
    }

    fn layout(&self) -> &'static str {
        if self.landscape() {
            "landscape"
        } else {
            "portrait"
        }
    }

    /// Switch to `turns`. Turning the panel 180deg keeps the page — it's
    /// just copied out the other way round. Between upright and sideways it
    /// can't be kept, so the panel starts blank (one ordinary whole-panel
    /// update, like clear_first — no flash).
    fn set_turns(&mut self, turns: u8) {
        let was_landscape = self.landscape();
        self.turns = turns;
        if self.landscape() != was_landscape {
            (self.w, self.h) = (self.h, self.w);
            fill_bg(&mut self.canvas, DEFAULT_BG_RGB);
        }
        self.present_all();
    }

    /// Copy the canvas region (x, y, w, h) to the framebuffer and refresh
    /// just that part of the panel.
    fn present(&mut self, x: i32, y: i32, w: i32, h: i32) {
        let (x0, y0) = (x.max(0), y.max(0));
        let (x1, y1) = ((x + w).min(self.w as i32), (y + h).min(self.h as i32));
        if x1 <= x0 || y1 <= y0 {
            return;
        }
        self.copy_out(x0 as usize, y0 as usize, x1 as usize, y1 as usize);
        let (px, py, pw, ph) = rect_to_panel(self.turns, self.w, self.h, x0, y0, x1 - x0, y1 - y0);
        let _ = self.client.update_partial(px, py, pw, ph);
    }

    fn present_all(&mut self) {
        self.copy_out(0, 0, self.w, self.h);
        let _ = self.client.update_all();
    }

    fn copy_out(&mut self, x0: usize, y0: usize, x1: usize, y1: usize) {
        let (turns, lw, lh) = (self.turns, self.w, self.h);
        let fb = self.client.framebuffer();
        for y in y0..y1 {
            if turns == 0 {
                let row = (y * lw + x0) * 2..(y * lw + x1) * 2;
                fb[row.clone()].copy_from_slice(&self.canvas[row]);
                continue;
            }
            for x in x0..x1 {
                let (px, py) = to_panel(turns, lw, lh, x, y);
                let src = (y * lw + x) * 2;
                let dst = (py * SCREEN_W + px) * 2;
                fb[dst..dst + 2].copy_from_slice(&self.canvas[src..src + 2]);
            }
        }
    }
}

/// Canvas pixel (x, y) -> framebuffer pixel, for a canvas lw x lh: the
/// content goes in turned back the other way, so a panel seen `turns`
/// quarter turns clockwise shows it upright.
fn to_panel(turns: u8, lw: usize, lh: usize, x: usize, y: usize) -> (usize, usize) {
    match turns {
        1 => (y, lw - 1 - x),
        2 => (lw - 1 - x, lh - 1 - y),
        3 => (lh - 1 - y, x),
        _ => (x, y),
    }
}

fn rect_to_panel(turns: u8, lw: usize, lh: usize, x: i32, y: i32, w: i32, h: i32) -> (i32, i32, i32, i32) {
    let (lw, lh) = (lw as i32, lh as i32);
    match turns {
        1 => (y, lw - (x + w), h, w),
        2 => (lw - (x + w), lh - (y + h), w, h),
        3 => (lh - (y + h), x, h, w),
        _ => (x, y, w, h),
    }
}

/// Directory the binary lives in, same resolution trick Riddle's scripts use
/// so the app works wherever AppLoad installed it.
fn events_dir_path() -> PathBuf {
    let exe = std::env::current_exe().expect("current_exe");
    exe.parent().expect("exe has no parent dir").join("events")
}

/// Event dirs are only picked up once a `READY` sentinel file exists in
/// them, so we never read a manifest the PC side is still scp-ing.
fn ready_events(events_dir: &Path) -> Vec<PathBuf> {
    let mut dirs: Vec<PathBuf> = match fs::read_dir(events_dir) {
        Ok(rd) => rd
            .filter_map(|e| e.ok())
            .map(|e| e.path())
            .filter(|p| p.is_dir() && p.join("READY").is_file())
            .collect(),
        Err(_) => Vec::new(),
    };
    dirs.sort();
    dirs
}

/// One event in progress. `step` does one unit of work and says (via
/// `next_at`) when the next one is due, so several jobs can interleave.
struct Job {
    dir: PathBuf,
    stages: std::collections::VecDeque<Stage>,
    clear_bg: Option<(u8, u8, u8)>, // clear_first, done on the job's first step
    clear_hold_ms: u64,
    writer: Option<(ink::Writer, u64, u64)>, // a text stage mid-write, + its step_ms, hold_ms
    text_bottom: i32,
    settle_after: bool,
    settled: bool,
    next_at: Instant,
    finished: bool,
    layout: Option<String>,
    page_start: bool,
    page_end: bool,
}

impl Job {
    fn start(dir: PathBuf) -> io::Result<Job> {
        let manifest_bytes = fs::read(dir.join("manifest.json"))?;
        let manifest: Manifest = serde_json::from_slice(&manifest_bytes)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

        Ok(Job {
            clear_bg: manifest
                .clear_first
                .then(|| manifest.bg.map(|c| (c[0], c[1], c[2])).unwrap_or(DEFAULT_BG_RGB)),
            clear_hold_ms: manifest.clear_hold_ms,
            dir,
            stages: manifest.stages.into(),
            writer: None,
            text_bottom: 0,
            settle_after: manifest.settle_after,
            settled: false,
            next_at: Instant::now(),
            finished: false,
            layout: manifest.layout,
            page_start: manifest.page_start,
            page_end: manifest.page_end,
        })
    }

    fn step(&mut self, screen: &mut Screen, font: &FontRef) -> io::Result<()> {
        // One full-panel flash per *touch*, not per image event: dozens of tiny
        // UFAST partial updates in a row (the tile reveal below) leave faint
        // random-looking ghosting from un-settled gray levels, since nothing
        // ever resets the panel's electronic state. A touch can push many image
        // events (the SHOW_IMAGE batch, then TMP_IMAGE/APPEND_IMAGE), so only
        // the first one sets clear_first — see eink_hook.py's show(). Matches
        // the HTML display's own reset point: it empties image-container on
        // SHOW_IMAGE too, not on every individual image.
        //
        // Fill to background, then one ordinary whole-panel UFAST update (not
        // request_full_refresh()) — same reasoning as the color settle stage
        // below: a single update settles cleanly on its own, it's only a long
        // run of sequential updates that ghosts. That means no GC16-style
        // flash/invert here either, and no need to wait out a flash before the
        // reveal starts.
        //
        // Done when the job reaches the front of the queue, not when it
        // arrives, so it never wipes a page that's still being drawn.
        //
        // Then wait clear_hold_ms: handwriting's rapid tiny updates started
        // right on top of the blank, before the panel had finished it, and
        // the previous page ghosted through.
        if let Some(bg) = self.clear_bg.take() {
            fill_bg(&mut screen.canvas, bg);
            screen.present_all();
            self.next_at = Instant::now() + Duration::from_millis(self.clear_hold_ms);
            return Ok(());
        }

        let now = Instant::now();
        if let Some((writer, step_ms, hold_ms)) = &mut self.writer {
            let dirty = writer.step(&mut screen.canvas, screen.w, screen.h);
            if let Some((dx, dy, dw, dh)) = dirty.rect(screen.w, screen.h) {
                screen.present(dx, dy, dw, dh);
            }
            if writer.done() {
                self.next_at = now + Duration::from_millis(*hold_ms);
                self.writer = None;
            } else {
                self.next_at = now + Duration::from_millis(*step_ms);
            }
            return Ok(());
        }

        match self.stages.pop_front() {
            Some(Stage::Image { x, y, image, hold_ms }) => {
                let (w, h, rgb) = decode_png_rgb8(&self.dir.join(&image))?;
                blit_rgb8(&mut screen.canvas, screen.w, screen.h, x, y, w, h, &rgb);
                screen.present(x, y, w as i32, h as i32);
                self.next_at = now + Duration::from_millis(hold_ms);
            }
            Some(Stage::Text { x, y, gap, w, text, px, hold_ms, points_per_step, step_ms }) => {
                let y = y.unwrap_or(self.text_bottom + (gap * px) as i32);
                let (strokes, bottom) = ink::plan(font, &text, x, y, w, px);
                self.text_bottom = bottom;
                let writer = ink::Writer::new(strokes, points_per_step.unwrap_or(ink::POINTS_PER_STEP));
                self.writer = Some((writer, step_ms.unwrap_or(ink::STEP_MS), hold_ms));
                self.next_at = now;
            }
            None if self.settle_after && !self.settled => {
                // This touch's last image: the UFAST tile-by-tile reveal above leaves
            // the picture at whatever partially-settled gray level each tile's
            // waveform reached, not a clean one. No fill_bg here — unlike
            // clear_first this must not blank what was just drawn, only re-drive it
            // with a full-quality waveform so it settles crisp.
                // A beat after the last stage lands before the settle
                // flash, so the finished picture reads as a distinct "done"
                // moment rather than the flash feeling glued to it.
                self.settled = true;
                self.next_at = now + Duration::from_millis(600);
                self.stages.push_back(Stage::Settle);
            }
            Some(Stage::Settle) => {
                let _ = screen.client.request_full_refresh();
                self.finished = true;
            }
            None => self.finished = true,
        }
        Ok(())
    }
}

/// Decode a PNG to a flat RGB8 buffer, normalizing whatever color type/bit
/// depth it was saved as (Pillow emits 8-bit RGB/RGBA/Grayscale — covers the
/// three cases below). Alpha is dropped: opacity is pre-baked into the pixel
/// values by the PC-side compositor, since the panel can't alpha-blend.
fn decode_png_rgb8(path: &Path) -> io::Result<(usize, usize, Vec<u8>)> {
    use png::{BitDepth, ColorType};

    let decoder = png::Decoder::new(fs::File::open(path)?);
    let mut reader = decoder
        .read_info()
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    let mut buf = vec![0u8; reader.output_buffer_size()];
    let info = reader
        .next_frame(&mut buf)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    let (w, h) = (info.width as usize, info.height as usize);

    if info.bit_depth != BitDepth::Eight {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unsupported PNG bit depth {:?} in {:?}", info.bit_depth, path),
        ));
    }

    let rgb = match info.color_type {
        ColorType::Rgb => buf[..w * h * 3].to_vec(),
        ColorType::Rgba => {
            let mut out = Vec::with_capacity(w * h * 3);
            for px in buf[..w * h * 4].chunks_exact(4) {
                out.extend_from_slice(&px[..3]);
            }
            out
        }
        ColorType::Grayscale => {
            let mut out = Vec::with_capacity(w * h * 3);
            for &v in &buf[..w * h] {
                out.extend_from_slice(&[v, v, v]);
            }
            out
        }
        other => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unsupported PNG color type {:?} in {:?}", other, path),
            ))
        }
    };
    Ok((w, h, rgb))
}

/// Fill the whole panel framebuffer with `bg`, so a clear_first flash
/// reveals blank paper instead of re-driving whatever tiles were already
/// sitting in the buffer from a previous touch.
fn fill_bg(fb: &mut [u8], bg: (u8, u8, u8)) {
    let (r, g, b) = bg;
    let px565: u16 = ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3);
    let bytes = px565.to_le_bytes();
    for px in fb.chunks_exact_mut(2) {
        px.copy_from_slice(&bytes);
    }
}

/// Write an RGB8 image into an RGB565 canvas `sw` x `sh` at (x, y), clipped
/// to its bounds.
#[allow(clippy::too_many_arguments)]
fn blit_rgb8(fb: &mut [u8], sw: usize, sh: usize, x: i32, y: i32, w: usize, h: usize, rgb: &[u8]) {
    for row in 0..h {
        let dest_y = y + row as i32;
        if dest_y < 0 || dest_y as usize >= sh {
            continue;
        }
        for col in 0..w {
            let dest_x = x + col as i32;
            if dest_x < 0 || dest_x as usize >= sw {
                continue;
            }
            let src = (row * w + col) * 3;
            let (r, g, b) = (rgb[src], rgb[src + 1], rgb[src + 2]);
            let px565: u16 = ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3);
            let dest = (dest_y as usize * sw + dest_x as usize) * 2;
            fb[dest..dest + 2].copy_from_slice(&px565.to_le_bytes());
        }
    }
}

/// `plush_memory_viewer --preview OUT_PREFIX PX W TEXT...` — off-device check
/// of a text stage: writes TEXT (one paragraph per argument) into a box W
/// wide on a blank panel-sized buffer and saves PNG snapshots at 1/3, 2/3
/// and the end of the writing (OUT_PREFIX_1.png ... _3.png) plus the step
/// count, so layout and pacing can be judged without the tablet.
fn preview(args: &[String]) {
    let usage = "usage: --preview OUT_PREFIX PX W TEXT...";
    let out = args.first().expect(usage);
    let px: f32 = args.get(1).expect(usage).parse().expect(usage);
    let w: i32 = args.get(2).expect(usage).parse().expect(usage);
    // PREVIEW_FONT=path.ttf tries another font without rebuilding.
    let other = std::env::var("PREVIEW_FONT").ok().map(|p| fs::read(p).expect("read PREVIEW_FONT"));
    let font = FontRef::try_from_slice(other.as_deref().unwrap_or(ink::FONT_TTF)).expect("font");

    let margin = (SCREEN_W as i32 - w) / 2;
    let mut y = 160;
    let mut strokes = Vec::new();
    for para in &args[3..] {
        let (s, next_y) = ink::plan(&font, para, margin, y, w, px);
        strokes.extend(s);
        y = next_y + (px * 0.6) as i32;
    }

    let mut fb = vec![0xffu8; SCREEN_W * SCREEN_H * 2];
    let mut steps = 0;
    {
        let mut counter = ink::Writer::new(strokes.clone(), ink::POINTS_PER_STEP);
        let mut scratch = fb.clone();
        while !counter.done() {
            counter.step(&mut scratch, SCREEN_W, SCREEN_H);
            steps += 1;
        }
    }
    let mut writer = ink::Writer::new(strokes, ink::POINTS_PER_STEP);
    let mut i = 0;
    let mut shot = 1;
    while !writer.done() {
        writer.step(&mut fb, SCREEN_W, SCREEN_H);
        i += 1;
        if i == steps / 3 || i == 2 * steps / 3 || writer.done() {
            save_png_rgb565(&fb, &format!("{out}_{shot}.png"));
            shot += 1;
        }
    }
    println!(
        "steps={steps} (~{:.1}s at {}ms/step, before panel latency)",
        steps as f64 * ink::STEP_MS as f64 / 1000.0,
        ink::STEP_MS
    );
}

fn save_png_rgb565(fb: &[u8], path: &str) {
    let mut rgb = Vec::with_capacity(SCREEN_W * SCREEN_H * 3);
    for px in fb.chunks_exact(2) {
        let v = u16::from_le_bytes([px[0], px[1]]);
        let (r, g, b) = ((v >> 11) as u8, ((v >> 5) & 0x3f) as u8, (v & 0x1f) as u8);
        rgb.extend_from_slice(&[(r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)]);
    }
    let file = fs::File::create(path).expect("create preview png");
    let mut enc = png::Encoder::new(io::BufWriter::new(file), SCREEN_W as u32, SCREEN_H as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header().unwrap().write_image_data(&rgb).unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every canvas pixel of a rect lands inside the panel and inside the
    /// rect `rect_to_panel` asks to refresh, and no two pixels collide.
    #[test]
    fn rotation_mapping_is_consistent() {
        for rot in 0..4u8 {
            let (lw, lh) = if rot % 2 == 1 { (SCREEN_H, SCREEN_W) } else { (SCREEN_W, SCREEN_H) };
            for &(x, y, w, h) in &[(0, 0, lw as i32, lh as i32), (37, 911, 320, 85), (lw as i32 - 5, lh as i32 - 9, 5, 9)] {
                let (px, py, pw, ph) = rect_to_panel(rot, lw, lh, x, y, w, h);
                assert!(px >= 0 && py >= 0 && px + pw <= SCREEN_W as i32 && py + ph <= SCREEN_H as i32);
                assert_eq!(pw * ph, w * h);
                let mut seen = std::collections::HashSet::new();
                for cy in y..y + h {
                    for cx in x..x + w {
                        let (qx, qy) = to_panel(rot, lw, lh, cx as usize, cy as usize);
                        let (qx, qy) = (qx as i32, qy as i32);
                        assert!(qx >= px && qx < px + pw && qy >= py && qy < py + ph, "rot {rot}");
                        assert!(seen.insert((qx, qy)));
                    }
                }
            }
        }
    }
}
