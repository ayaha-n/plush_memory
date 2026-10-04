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
- **e-ink version**: shown on a reMarkable Paper Pro screen, materializing softly through partial refreshes. Generated images are pen-and-ink with a light watercolor wash, transparent background (`shepard` style, after E. H. Shepard's illustrations).

Pick one look per node launch with `_display_target` (the two are never generated in parallel — still one API call per touch).

## Running it

```bash
# HTML version (classic, browser display)
rosrun plush_memory touch_image_camera_new.py _display_target:=html _enable_generation:=True

# e-ink version (shepard, reMarkable Paper Pro display)
rosrun plush_memory touch_image_camera_new.py _display_target:=eink _enable_generation:=True
```

- `_enable_generation:=False` skips new generation and only falls back to existing images (useful for testing without a camera attached).
- Image generation needs the `OPENAI_API_KEY` environment variable (`scripts/illustration_and_combine_new.py`, using the `gpt-image-1` image-edit API).

### Viewing the HTML version

Opening `plush_memory_camera.html` directly via `file://` leaves `location.hostname` empty, so the WebSocket connection fails — serve it instead.

```bash
cd plush_memory && python3 -m http.server 8000
# open http://localhost:8000/plush_memory_camera.html in a browser
```

### Viewing the e-ink version

The reMarkable Paper Pro needs `eink-viewer/` (Rust) built and installed via AppLoad beforehand (xovi + AppLoad setup, using `remagic` etc. — documented separately). `eink_memory_push.py` sends images to the tablet over SSH.

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
| `CLEAR_BEFORE_TOUCH` | Whether a new touch blanks the panel to background before its reveal starts (on by default; only the touch's first image ever sets this, so toggling it doesn't cause more than one blank) |

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
| `scripts/eink_hook.py` | Rides along on the HTML side's SHOW_IMAGE/TMP_IMAGE/APPEND_IMAGE to build the matching e-ink display events |
| `scripts/eink_memory_push.py` | Turns a generated image into staged materialize frames for e-ink and pushes them to the tablet over SSH |
| `eink-viewer/` | Rust AppLoad app running on the tablet; blits frames in turn via partial refresh |
| `plush_memory_camera.html` | The HTML display page |

## Known limitations

- OpenAI's output moderation occasionally rejects a generation even for the same input/prompt that worked before — retrying once or twice usually gets through.
- The eyes in e-ink-style illustrations (whether they have a highlight, and how large it is) aren't fully stable from the prompt alone.
