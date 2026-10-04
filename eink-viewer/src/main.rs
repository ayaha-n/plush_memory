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
use qtfb::{QtfbClient, FBFMT_RMPP_RGB565, REFRESH_MODE_UFAST};
use serde::Deserialize;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const SCREEN_W: usize = 1620;
const SCREEN_H: usize = 2160;
const STRIDE: usize = SCREEN_W * 2; // RGB565, 2 bytes/pixel
const POLL_INTERVAL: Duration = Duration::from_millis(150);
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

    let mut client = QtfbClient::connect(key, FBFMT_RMPP_RGB565, SCREEN_W, SCREEN_H, 2)
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

    let font = FontRef::try_from_slice(ink::FONT_TTF).expect("bundled font");
    let mut jobs: Vec<Job> = Vec::new();
    let mut last_poll = Instant::now() - POLL_INTERVAL;

    // Each READY event becomes a Job, queued in name order (the PC side
    // names them so that sorts in push order). Only the front job runs —
    // one step per pass (one image stage, or one small batch of pen points)
    // — so the page is drawn in exactly the order it was sent: text, then
    // pictures, then the closing text.
    loop {
        // Drain input/window events; an Err means AppLoad closed our window.
        if client.drain_events().is_err() {
            eprintln!("plush_memory_viewer: window closed, exiting");
            break;
        }

        if last_poll.elapsed() >= POLL_INTERVAL {
            last_poll = Instant::now();
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
            if let Err(e) = job.step(&mut client, &font) {
                eprintln!("plush_memory_viewer: event {:?} failed: {e}", job.dir);
                job.finished = true;
            }
        }
        jobs.retain(|j| {
            if j.finished {
                let _ = fs::remove_dir_all(&j.dir);
            }
            !j.finished
        });

        let wake = jobs.first().map_or(now + POLL_INTERVAL, |j| j.next_at);
        let wait = wake.saturating_duration_since(Instant::now()).min(POLL_INTERVAL);
        std::thread::sleep(wait.max(Duration::from_millis(1)));
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
        })
    }

    fn step(&mut self, client: &mut QtfbClient, font: &FontRef) -> io::Result<()> {
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
            fill_bg(client.framebuffer(), bg);
            let _ = client.update_all();
            self.next_at = Instant::now() + Duration::from_millis(self.clear_hold_ms);
            return Ok(());
        }

        let now = Instant::now();
        if let Some((writer, step_ms, hold_ms)) = &mut self.writer {
            let dirty = writer.step(client.framebuffer(), SCREEN_W, SCREEN_H);
            if let Some((dx, dy, dw, dh)) = dirty.rect(SCREEN_W, SCREEN_H) {
                let _ = client.update_partial(dx, dy, dw, dh);
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
                blit_rgb8(client.framebuffer(), x, y, w, h, &rgb);
                let _ = client.update_partial(x, y, w as i32, h as i32);
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
                let _ = client.request_full_refresh();
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

/// Write an RGB8 image into the RGB565 panel framebuffer at (x, y), clipped
/// to the panel bounds.
fn blit_rgb8(fb: &mut [u8], x: i32, y: i32, w: usize, h: usize, rgb: &[u8]) {
    for row in 0..h {
        let dest_y = y + row as i32;
        if dest_y < 0 || dest_y as usize >= SCREEN_H {
            continue;
        }
        for col in 0..w {
            let dest_x = x + col as i32;
            if dest_x < 0 || dest_x as usize >= SCREEN_W {
                continue;
            }
            let src = (row * w + col) * 3;
            let (r, g, b) = (rgb[src], rgb[src + 1], rgb[src + 2]);
            let px565: u16 = ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3);
            let dest = dest_y as usize * STRIDE + dest_x as usize * 2;
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
