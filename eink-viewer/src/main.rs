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

mod qtfb;

use qtfb::{QtfbClient, FBFMT_RMPP_RGB565, REFRESH_MODE_UFAST};
use serde::Deserialize;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::time::Duration;

const SCREEN_W: usize = 1620;
const SCREEN_H: usize = 2160;
const STRIDE: usize = SCREEN_W * 2; // RGB565, 2 bytes/pixel
const POLL_INTERVAL: Duration = Duration::from_millis(150);
// Matches eink_memory_push.py's BG (#fefaf5) — the paper tone a clear_first
// flash should reveal, not pure white, so it matches the tiles' own
// background fill.
const BG_RGB: (u8, u8, u8) = (254, 250, 245);

#[derive(Deserialize)]
struct Manifest {
    #[serde(default)]
    clear_first: bool,
    #[serde(default)]
    settle_after: bool,
    stages: Vec<Stage>,
}

#[derive(Deserialize)]
struct Stage {
    x: i32,
    y: i32,
    image: String,
    hold_ms: u64,
}

fn main() {
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

    loop {
        // Drain input/window events; an Err means AppLoad closed our window.
        if client.drain_events().is_err() {
            eprintln!("plush_memory_viewer: window closed, exiting");
            break;
        }

        if let Some(event_path) = next_ready_event(&events_dir) {
            if let Err(e) = process_event(&mut client, &event_path) {
                eprintln!("plush_memory_viewer: event {:?} failed: {e}", event_path);
            }
            let _ = fs::remove_dir_all(&event_path);
        }

        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Directory the binary lives in, same resolution trick Riddle's scripts use
/// so the app works wherever AppLoad installed it.
fn events_dir_path() -> PathBuf {
    let exe = std::env::current_exe().expect("current_exe");
    exe.parent().expect("exe has no parent dir").join("events")
}

/// An event dir is only picked up once a `READY` sentinel file exists in it,
/// so we never read a manifest the PC side is still scp-ing. Oldest first
/// (sorted by directory name — the PC side uses zero-padded timestamps).
fn next_ready_event(events_dir: &Path) -> Option<PathBuf> {
    let mut dirs: Vec<PathBuf> = fs::read_dir(events_dir)
        .ok()?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.is_dir() && p.join("READY").is_file())
        .collect();
    dirs.sort();
    dirs.into_iter().next()
}

fn process_event(client: &mut QtfbClient, event_dir: &Path) -> io::Result<()> {
    let manifest_bytes = fs::read(event_dir.join("manifest.json"))?;
    let manifest: Manifest = serde_json::from_slice(&manifest_bytes)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

    // One full-panel flash per *touch*, not per image event: dozens of tiny
    // UFAST partial updates in a row (the tile reveal below) leave faint
    // random-looking ghosting from un-settled gray levels, since nothing
    // ever resets the panel's electronic state. A touch can push many image
    // events (the SHOW_IMAGE batch, then TMP_IMAGE/APPEND_IMAGE), so only
    // the first one sets clear_first — see eink_hook.py's show(). Matches
    // the HTML display's own reset point: it empties image-container on
    // SHOW_IMAGE too, not on every individual image.
    //
    // request_full_refresh() alone just re-drives whatever is already in
    // the framebuffer with a cleaner waveform — it does not blank it, so
    // without filling it first this "flash" still shows the previous
    // touch's leftover tiles, not a clean white reset. Fill to background
    // first so the flash actually reveals blank paper, same as the HTML's
    // innerHTML = "" moment.
    if manifest.clear_first {
        fill_bg(client.framebuffer());
        let _ = client.request_full_refresh();
        // request_full_refresh() returns as soon as the message is sent —
        // it doesn't wait for the panel to actually render the flash. Without
        // a pause here, the tile loop below started overwriting the (blank)
        // framebuffer with real tiles before the panel had rendered the
        // blank frame at all, so the white moment never actually appeared on
        // screen. qtfb.rs documents a ~1s server-side stall on this message;
        // give it that long before drawing over the buffer again. 1000ms
        // still left the first few reveal rows looking faint (drawn while
        // the flash was still settling), so give it more margin.
        std::thread::sleep(Duration::from_millis(1800));
    }

    for stage in manifest.stages {
        let (w, h, rgb) = decode_png_rgb8(&event_dir.join(&stage.image))?;
        blit_rgb8(client.framebuffer(), stage.x, stage.y, w, h, &rgb);
        let _ = client.update_partial(stage.x, stage.y, w as i32, h as i32);
        std::thread::sleep(Duration::from_millis(stage.hold_ms));
    }

    // This touch's last image: the UFAST tile-by-tile reveal above leaves
    // the picture at whatever partially-settled gray level each tile's
    // waveform reached, not a clean one. No fill_bg here — unlike
    // clear_first this must not blank what was just drawn, only re-drive it
    // with a full-quality waveform so it settles crisp.
    if manifest.settle_after {
        // A beat after the last tile lands (on top of its own hold_ms)
        // before the settle flash, so the finished picture reads as a
        // distinct "done" moment rather than the flash feeling glued to
        // the last tile's reveal.
        std::thread::sleep(Duration::from_millis(600));
        let _ = client.request_full_refresh();
    }
    Ok(())
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

/// Fill the whole panel framebuffer with BG_RGB, so a clear_first flash
/// reveals blank paper instead of re-driving whatever tiles were already
/// sitting in the buffer from a previous touch.
fn fill_bg(fb: &mut [u8]) {
    let (r, g, b) = BG_RGB;
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
