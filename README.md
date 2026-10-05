# plush_memory

An interactive art piece, *Plush Memories* (「ぬいぐるみの記憶」) — see [the work's page](https://ayaha-n.github.io/work-exhibition.html).

Touch the plush bear's head, hand, foot, or belly, and it photographs that moment through its camera, generates an illustration of you together with the bear, and lets it surface on screen as a "memory."

## How it works

```
touch sensors (/xxx_touch_trigger) ─┐
                                     ├─> touch_image_camera_new.py (ROS node)
camera (D405)                       ┘        │
                                              ├─ draw_on_touch.py: capture → illustrate (OpenAI API)
                                              │
                                              ├─ WebSocket (ws://*:8765) ──> plush_memory_camera.html
                                              │                              (browser display, "classic" style)
                                              └─ eink_hook.py ──────────────> eink-viewer (Rust, AppLoad)
                                                                               (reMarkable Paper Pro, "shepard" style)
```

Two ways to see it.

- **HTML version**: the original way to view it, opening `plush_memory_camera.html` in a browser on a PC or monitor. Generated images are flat-color cartoon style (`classic`).
- **e-ink version**: shown on a reMarkable Paper Pro screen as a picture-book page — a short text written out by hand, stroke by stroke, at the top, illustrations materializing softly through partial refreshes in the middle, and a closing line or two at the bottom (see [Picture-book text](#picture-book-text)). Generated images are pen-and-ink with a light watercolor wash, transparent background (`shepard` style, after E. H. Shepard's illustrations).

Pick one look per node launch with `_display_target` (the two are never generated in parallel — still one API call per touch).

## Running it

```bash
# HTML version (classic, browser display)
rosrun plush_memory touch_image_camera_new.py _display_target:=html _enable_generation:=True

# e-ink version (shepard, reMarkable Paper Pro display)
rosrun plush_memory touch_image_camera_new.py _display_target:=eink _enable_generation:=True
```

- `_enable_generation:=False` skips new generation and uses existing images only (useful for testing without a camera attached): up to 9 are picked, and the last one takes the "latest" spot in the middle.
- `_text_lang:=en` switches the e-ink page's text to English (default `ja`).
- `_eink_orientation:=landscape_cw` (or `landscape_ccw`) lays the e-ink page out sideways, for a tablet set down a quarter turn clockwise (or counterclockwise) from upright; default `portrait`. See [Turning the tablet sideways](#turning-the-tablet-sideways).
- Image generation needs the `OPENAI_API_KEY` environment variable (`scripts/illustration_and_combine_new.py`, using the `gpt-image-1` image-edit API).

### Viewing the HTML version

Opening `plush_memory_camera.html` directly via `file://` leaves `location.hostname` empty, so the WebSocket connection fails — serve it instead.

```bash
cd plush_memory && python3 -m http.server 8000
# open http://localhost:8000/plush_memory_camera.html in a browser
```

### Viewing the e-ink version

The reMarkable Paper Pro needs `eink-viewer/` (Rust) built and installed via AppLoad beforehand (xovi + AppLoad setup, using `remagic` etc. — documented separately). `eink_memory_push.py` sends images to the tablet over SSH.

Building the viewer needs only a Rust toolchain ([rustup](https://rustup.rs/)) — it cross-compiles to a static musl binary with the bundled `rust-lld`, so no aarch64 C toolchain is required (`eink-viewer/.cargo/config.toml`):

```bash
rustup target add aarch64-unknown-linux-musl
cd eink-viewer && cargo build --release --target aarch64-unknown-linux-musl
scp target/aarch64-unknown-linux-musl/release/plush_memory_viewer external.manifest.json icon.png \
    root@10.11.99.1:/home/root/xovi/exthome/appload/plush_memory_viewer/
# then close and relaunch the app from AppLoad (after a manifest/icon change,
# also press reload in the AppLoad list while no app is open)
```

`cargo run --release -- --preview out 56 1480 "some text"` renders a text stage off-device into `out_1.png`…`out_3.png` (progress snapshots), for checking layout without the tablet.

```bash
ssh-copy-id root@10.11.99.1          # once, to set up SSH key auth
# launch the "Plush Memory" app from AppLoad on the tablet
```

The target host comes from the `EINK_HOST` environment variable (default `10.11.99.1`, i.e. USB). To run without the USB cable, the tablet can instead be reached wirelessly over a VPN such as [Tailscale](https://tailscale.com/) (tablet-side setup documented separately). If its SSH server listens on a non-default port, add an entry to `~/.ssh/config` so `ssh`/`scp` pick it up — for example:

```
Host plush-eink
    HostName plush-eink
    User root
    Port 2222
```

then launch with:

```bash
EINK_HOST=plush-eink rosrun plush_memory touch_image_camera_new.py _display_target:=eink _enable_generation:=False
```

The wireless route won't work when:

- the network has no internet access (Tailscale needs to reach its coordination server), e.g. a closed exhibition LAN;
- the Wi-Fi requires a browser login (captive portal), which the tablet can't get through on its own;
- the tablet is asleep or off Wi-Fi;
- the tablet's OS was updated and the Tailscale setup hasn't been redone.

In those cases, fall back to the USB cable (unset `EINK_HOST`). Networks that block Tailscale's direct UDP traffic still work, just via a relay with higher latency.

### Turning the tablet sideways

The viewer draws on a canvas the shape of the panel as the reader sees it
(1620x2160 upright, 2160x1620 sideways) and copies it out turned to match,
and the PC side lays each page out for whichever shape the viewer reports
(`orientation` in the app's directory). How far round to turn comes from two
sources that add up:

- **`_eink_orientation`** (written to `orientation.conf` on the tablet): how
  the tablet itself is set down. Use this with **xochitl's auto-rotate
  turned off** in the tablet's settings, so xochitl's own screen stays
  upright while the tablet lies on its side. `landscape_cw` is turned a
  quarter turn clockwise from upright (its top edge to the right),
  `landscape_ccw` counterclockwise; if the page comes out upside down, use
  the other one.
- **The rotation AppLoad paints the window with**. With AppLoad v0.6.0 or
  later, `"supportsRotation": true` in `external.manifest.json` makes
  AppLoad turn the window with xochitl's auto-rotate and tell the viewer,
  so leave `_eink_orientation` at `portrait` and let the tablet rotate by
  itself. (Fullscreen only — a windowed app turns with its title-bar
  button instead.)

Older AppLoad (v0.5.x — the one for firmware 3.27; v0.6.0 targets 3.28+)
has no rotation support: it ignores `supportsRotation`, never reports a
rotation, and always shows an app in a portrait-shaped area, so with
auto-rotate on, a sideways tablet gets the page shrunk to the middle with
blank bars left and right. Hence `_eink_orientation` with auto-rotate off.
Both paths are kept so that updating AppLoad (and the firmware) later needs
no viewer change.

Either change is only applied while the panel is at rest: a page being
drawn when the tablet turns (or `orientation.conf` changes) is finished the
old way round, then the panel is blanked and the next touch's page is laid
out for the new orientation. `orientation.conf` is re-read while the viewer
runs, so changing `_eink_orientation` needs no app restart.

### How the e-ink reveal avoids flashing/ghosting

This took a lot of on-device trial and error, so it's worth writing down
what was actually learned, not just the final knobs.

**The core finding**: the panel's fast partial-refresh waveform (UFAST)
reveals a *pure black/white* image, on a pure white background, with no
fading or ghosting at all — however many small partial updates are done in
a row. A grayscale/watercolor version of the exact same regions *does*
fade, more the longer it's been since the last full refresh. A single
color update settles cleanly on its own, so the problem isn't "too much
area updated" or "the same pixels touched repeatedly" either — bands are
non-overlapping (each pixel is written once) and still fade in color after
only ~20 of them. What actually seems to accumulate is the *count* of
sequential UFAST color/grayscale operations issued, regardless of which
pixels each one targets — some internal waveform/calibration state that
only a full refresh resets. Pure black/white isn't just "simpler" content;
UFAST is seemingly tuned for exactly that binary case, so it never drifts
in the first place and nothing needs resetting.

That reframes the whole design:

- **The reveal animation** (`scripts/eink_memory_push.py`) always binarizes
  the illustration to pure black/white first (`_binarize`), and only
  reveals *that*, never the real colors, while animating. Several reveal
  shapes were tried: square tiles in dissolve order, and tracing the
  binarized ink's skeleton into strokes and revealing them like pen
  strokes (literally porting MaximeRivest/riddle's glyph-rendering
  technique — see git history). Both worked technically, but a detailed
  illustration's skeleton is mostly short, disconnected fragments rather
  than one continuous line, so the stroke version read as jumping around.
  Full-height/width **bands swept in one direction** (`REVEAL_STYLE =
  "band"`) ended up looking the most natural — simple continuous motion,
  no jumping. `"tile"` is kept as a second option.
- **The color reveal** is one single ordinary partial update of the real
  image over the whole area, once the black/white bands finish — no extra
  full-quality flash needed, because (per the finding above) a single
  update is never the problem.
- **Blanking the panel for a new touch** (`clear_first`) no longer uses
  `request_full_refresh()` (the GC16-style flash with the visible black/
  white invert) — that call refreshes the *entire panel* (the qtfb
  protocol carries no region for it), so it used to disturb every other
  memory already sitting elsewhere on screen just to reset one spot. It's
  now a background fill plus one ordinary whole-panel `update_all()` —
  again, a single update, so no flash and no disturbance.

Net effect: nothing on the e-ink display ever triggers the inverting
full-refresh flash any more. Every visible change — the band reveal, the
color landing, the per-touch blank — is built from partial updates that
are each either pure black/white or a single one-shot update, which is
exactly the case that was measured to never ghost.

### Tuning the e-ink reveal

A handful of constants at the top of `scripts/eink_memory_push.py` — edit
the file and restart the node, no rebuild needed:

| Constant | What it controls |
|---|---|
| `REVEAL_STYLE` | `"band"` (full-height/width strips swept in one direction — the one that looks good) or `"tile"` (square dissolve) |
| `BAND_COUNT` / `BAND_DIRECTION` / `BAND_HOLD_MS` | Band reveal: how many strips, `"ltr"` or `"ttb"`, delay between them |
| `REVEAL_ORDER` | Tile reveal only: `"dither"` (scattered) or `"raster"` (scan order) |
| `TILE_COLUMNS` / `MIN_TILE_PX` / `TILE_HOLD_MS` | Tile reveal only: grid size and pacing |
| `FINAL_HOLD_MS` | Pause on the finished black/white picture before the color stage lands |
| `BW_THRESHOLD` | Luminance cutoff for binarizing the reveal |
| `BG` | Background color — the *only* place it's set; `eink-viewer` reads it from the manifest instead of hardcoding its own copy |
| `CLEAR_BEFORE_TOUCH` | Whether a new touch blanks the panel to background before its page starts (on by default; only the touch's first event ever sets this, so toggling it doesn't cause more than one blank) |
| `CLEAR_HOLD_MS` | Pause after that blank before anything is drawn — without it, the handwriting starting right away let the previous page ghost through |
| `TEXT_POINTS_PER_STEP` / `TEXT_STEP_MS` | Handwriting pace: pen points per partial update, and ms between updates |

## Picture-book text

On the e-ink page, each touch writes a short text below the illustrations, in the plush's narrator voice:

1. an **opening** — it has been touched there many times (fixed per part)
2. an **episode** — one memory, picked at random per touch (never the same twice in a row)
3. the **evidence** — the trace those touches left on it (fixed per part)

and, once the newest illustration appears, a **closing** that records this very touch and says it will be remembered too. The page is drawn in reading order, one thing at a time: the memory at the top, then the illustrations in the middle (top-left first), then the closing at the bottom, under the newest illustration. `scripts/eink_hook.py` pushes every event through one queue so they reach the tablet in that order, and the viewer runs them one after another.

All of it lives in `data/memory_texts.json` — one entry per part (`opening` / `episodes` / `evidence`), and `_touch_line` for the closing (a template with `{year}`, `{month}`, `{day_ja}`, `{part_ja}`, … filled in by `scripts/memory_text.py`). Every sentence is a `ja`/`en` pair. The file is re-read on each touch, so edits apply without restarting the node. Write Japanese with spaces between words (分かち書き) — the viewer wraps lines on spaces. Every sentence should fit on one line of the text area (about 26 kana at the current size); the layout is sized for that.

The handwriting itself happens on the tablet: the viewer rasterizes the text in the bundled Yomogi font, thins it to 1px skeleton strokes and traces them into ordered pen paths, then draws a few points per tiny partial refresh — the technique from [riddle](https://github.com/MaximeRivest/riddle) (see [Dependencies and credits](#dependencies-and-credits)). Pure black strokes on white are the case UFAST reveals without fading, so text needs no color-landing step.

Page layout (`scripts/eink_hook.py`): the memory text at the top, the closing at the bottom, and between them images in fixed, non-overlapping slots — the newest one in a 520px box in the middle, up to ten 320px boxes around it — each image fit inside its box.

## Image data layout

```
data/images/
├── combined_image_<id>.png          # bear + person illustration, combined (transparent PNG)
├── person_image_<id>.png            # person illustration (classic)
├── classic/
│   └── generated_drawing_<id>_<part>.png   # for the HTML display (opaque, flat color)
├── shepard/
│   └── generated_drawing_<id>_<part>.png   # for the e-ink display (transparent, pen + watercolor)
└── generated_drawing_<id>_<part>.png       # (optional) older images from before the style split.
                                              # Read only by a "classic" run, as a fallback when
                                              # classic/ is missing or has nothing unshown left.
                                              # "shepard" never uses this fallback, to keep an
                                              # opaque image from slipping onto the e-ink display.
```

`<id>` is the person's id, `<part>` is one of `larm`/`rarm`/`lleg`/`rleg`/`head`/`stomach`. Multiple parts from the same person share the same `<id>`.

## Scripts

| File | Role |
|---|---|
| `scripts/touch_image_camera_new.py` | Main ROS node: touch detection, WebSocket broadcast, and the e-ink hookup |
| `scripts/capture_on_touch.py` | Captures a photo from the camera (D405) |
| `scripts/draw_on_touch.py` | Capture → illustrate the person → combine with the bear into the final image |
| `scripts/illustration_and_combine_new.py` | OpenAI image-edit API calls, prompt and style definitions |
| `scripts/eink_hook.py` | Rides along on the HTML side's SHOW_IMAGE/TMP_IMAGE/APPEND_IMAGE to build the matching e-ink display events; owns the page layout |
| `scripts/eink_memory_push.py` | Turns a generated image into staged materialize frames for e-ink, or a text into a handwriting event, and pushes them to the tablet over SSH |
| `scripts/memory_text.py` | Picks the picture-book text for a touch from `data/memory_texts.json` |
| `data/memory_texts.json` | The picture-book text, per part, in Japanese and English |
| `eink-viewer/` | Rust AppLoad app running on the tablet; runs events in order — blits image frames and writes text stroke by stroke, via partial refresh |
| `plush_memory_camera.html` | The HTML display page |

## Dependencies and credits

- **[MaximeRivest/riddle](https://github.com/MaximeRivest/riddle)** (MIT License) — the e-ink viewer is built on code taken from it, almost verbatim: `eink-viewer/src/qtfb.rs` (the AppLoad qtfb client) and `eink-viewer/src/script.rs` (text → handwriting strokes: rasterize, Zhang-Suen thinning, trace), plus its pen-drawing pacing in `eink-viewer/src/ink.rs`. Its license is in `eink-viewer/LICENSE-riddle`.
- **[Yomogi](https://github.com/satsuyako/YomogiFont)** font (SIL Open Font License 1.1) — the handwriting on the e-ink page; bundled as `eink-viewer/fonts/Yomogi-Regular.ttf`, license in `eink-viewer/fonts/OFL.txt`.
- **xovi + AppLoad** on the tablet (set up separately) — the viewer runs as an AppLoad app.
- **OpenAI API** (`gpt-image-1`) — illustration generation.

## Known limitations

- OpenAI's output moderation occasionally rejects a generation even for the same input/prompt that worked before — retrying once or twice usually gets through.
- The eyes in e-ink-style illustrations (whether they have a highlight, and how large it is) aren't fully stable from the prompt alone.
