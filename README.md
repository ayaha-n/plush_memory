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

### Tuning the e-ink reveal/flash

All of the materialize/flash behavior is a handful of constants at the top
of `scripts/eink_memory_push.py` — edit the file and restart the node, no
rebuild needed (only `eink-viewer/` itself needs rebuilding, for the
`clear_first`/`settle_after` handling in `eink-viewer/src/main.rs`):

| Constant | What it controls |
|---|---|
| `REVEAL_ORDER` | `"dither"` (scattered, more natural) or `"raster"` (top-left to bottom-right, like a scan) |
| `TILE_COLUMNS` / `MIN_TILE_PX` | Tile grid size — fixed column count so a bigger image gets bigger tiles instead of many more of them |
| `TILE_HOLD_MS` | Delay between tiles |
| `FINAL_HOLD_MS` | Extra pause on the last tile before moving on |
| `CLEAR_BEFORE_TOUCH` | Whether a new touch blanks the panel to background and flashes before its tiles start (on by default; `eink_hook.py`'s `show()` only sets this for a touch's first image either way, so toggling it doesn't cause more than one flash) |

There's also an unconditional flash once a touch's images finish revealing
(`settle_after`, wired in `eink_hook.py`'s `show()` last image and
`append_latest()`) — it re-drives the just-drawn picture with a
full-quality waveform so it settles crisp instead of staying at whatever
partial gray level the fast tile-by-tile reveal left it at. That one isn't
behind a toggle; it's cheap (no blanking) and always worth doing.

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
