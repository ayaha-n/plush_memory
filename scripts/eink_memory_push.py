#!/usr/bin/env python3
"""Push a "materializing" memory image to the plush_memory_viewer AppLoad app
on the reMarkable Paper Pro.

This targets the plush_memory_viewer Rust app (../eink-viewer): it watches
`<app dir>/events/<id>/` for a manifest.json + PNG frames + a READY sentinel,
then blits each frame in turn with a partial (non-flashing) e-ink refresh,
holding each one for `hold_ms` before moving to the next.

The "materializing" look is a single smooth sweep across the image (see
REVEAL_STYLE/_slice_bands) — inspired by how MaximeRivest/riddle
(https://github.com/MaximeRivest/riddle) draws Tom's ink replies stroke by
stroke via many small, fast partial refreshes rather than one big one. A
literal port of that (skeletonize the illustration's ink, trace it into
point sequences, reveal small chunks along each one) was tried and
measured — see git history on this file — but a detailed illustration's
skeleton is mostly short, disconnected fragments rather than one
continuous line, so it read as jumping around rather than a sweep. Plain
full-height/width bands, swept in one direction, is both simpler and reads
as more natural movement; "tile" (square dissolve) is kept as a second
option.

Measured on-device: a pure black/white rendering, on a pure white
background, reveals with *no* fading or ghosting at all under UFAST,
however many small partial updates. A grayscale/watercolor version of the
same regions does fade (the longer since the last full refresh, the more).
So the reveal always uses a binarized (pure black/white) rendering of the
image — see _binarize() — regardless of how colorful the actual
illustration is. Once the reveal finishes, one last stage blits the real
color image over the whole area in a single ordinary partial update — that
alone settles cleanly (it's one update, not the long run of sequential
ones that caused fading), so no extra full-quality flash is needed, and
unlike request_full_refresh() it never disturbs other memories already
sitting elsewhere on the panel.
"""
import json
import os
import subprocess
import tempfile
import time

from PIL import Image

EINK_HOST = os.environ.get("EINK_HOST", "10.11.99.1")
EINK_APP_DIR = "/home/root/xovi/exthome/appload/plush_memory_viewer"

SCREEN_W, SCREEN_H = 1620, 2160
BG = (255, 255, 255)  # pure white — e-ink only, not the HTML display's #fefaf5
                       # (plush_memory_camera.html is untouched by this file).
                       # Keeping this the same white the binarized reveal
                       # phase maps its background to avoids a visible
                       # white-to-cream jump when the color stage lands.

# "dither": scattered (Bayer) reveal order — more natural-looking materialize.
# "raster": top-left to bottom-right, like a scan — simpler, more mechanical.
REVEAL_ORDER = "dither"

# Whether a new touch blanks the panel to background first (see clear_first
# below) before its tiles start appearing. Off leaves whatever the previous
# touch left on screen in place until these new tiles draw over it.
CLEAR_BEFORE_TOUCH = True

TILE_COLUMNS = 8       # fixed column count, not a fixed pixel size — a
                       # larger image gets proportionally larger tiles (and
                       # roughly the same total tile count / reveal time,
                       # ~64 tiles either way) instead of exploding into way
                       # more tiny tiles the bigger it is
MIN_TILE_PX = 8        # floor, so a narrow image doesn't get degenerate tiles
TILE_HOLD_MS = 45      # delay between tiles — more time for the panel's
                       # gray levels to actually settle before the next
                       # partial update, not just a pacing choice
FINAL_HOLD_MS = 800    # pause on the finished (still black/white) picture
                       # before the color stage replaces it
BW_THRESHOLD = 140     # luminance cutoff for the reveal's binarization —
                       # BG's ~250 average comfortably maps to white, ink/
                       # color areas below this map to black

# "band": full-height (or full-width) strips swept left-to-right (or
# top-to-bottom) — smooth, like a single continuous wipe, no jumping
# between disconnected regions. This is the one that actually looked good.
# "tile": square-tile dissolve (see _slice_tiles) — kept as a second option.
REVEAL_STYLE = "band"
BAND_COUNT = 20        # number of strips — tile size scales with the image,
                       # same reasoning as TILE_COLUMNS
BAND_DIRECTION = "ltr"  # "ltr": vertical strips, left to right.
                        # "ttb": horizontal strips, top to bottom.
BAND_HOLD_MS = 150     # delay between bands — fewer, bigger updates than
                       # tiles, so each one can afford more settle time

# Handwritten text (push_text). Glyph height in px; the pen width is fixed
# in the viewer (ink.rs PEN_R).
TEXT_PX = 56  # every sentence fits one line in eink_hook's TEXT_W at this size
TEXT_PARAGRAPH_GAP = 0.6       # extra space between paragraphs, in glyph heights
TEXT_PARAGRAPH_PAUSE_MS = 700  # beat between finishing one paragraph and starting the next
# Writing pace: pen points drawn per partial update, and ms between updates.
# riddle's own pace (26 / 14) reads as brisk; this is a slower, calmer hand.
TEXT_POINTS_PER_STEP = 12
TEXT_STEP_MS = 20


def _tight_bbox(img, alpha_threshold=16):
    """Bounding box of the non-near-transparent pixels, so we only ever
    refresh the part of the panel the subject actually occupies — not the
    padding around it, even though that padding is already background-
    colored and so wouldn't look wrong left in. Falls back to the full image
    for pictures with no real alpha (the older opaque assets)."""
    if img.mode not in ("RGBA", "LA"):
        return (0, 0, img.width, img.height)
    alpha = img.split()[-1]
    mask = alpha.point(lambda a: 255 if a > alpha_threshold else 0)
    return mask.getbbox() or (0, 0, img.width, img.height)


def _flatten_onto_bg(img, bg):
    """If img has real alpha (a cutout, not just an opaque RGBA file), paste
    it onto a solid bg-colored canvas using that alpha as the mask, so a
    transparent background disappears into the panel's paper color instead
    of the subject's edges staying a hard rectangle."""
    if img.mode in ("RGBA", "LA") or (img.mode == "P" and "transparency" in img.info):
        img = img.convert("RGBA")
        canvas = Image.new("RGB", img.size, bg)
        canvas.paste(img, mask=img.split()[-1])
        return canvas
    return img.convert("RGB")


# Classic 4x4 ordered-dither (Bayer) matrix. Indexing a tile's (row, col) by
# this mod 4 gives a reveal priority that's evenly scattered rather than a
# left-to-right/top-to-bottom scan — the same "materializes out of scattered
# noise" look a dissolve transition has (Riddle's own fb.rs calls this kind
# of thing a "dissolve region"), without changing anything about *how* each
# tile is drawn — still one small rectangle per partial update, just in a
# different order, so it costs no more total refreshed area than raster
# order did.
BAYER4 = (
    (0, 8, 2, 10),
    (12, 4, 14, 6),
    (3, 11, 1, 9),
    (15, 7, 13, 5),
)


def _prepare_image(image_path, target_w, max_h=None):
    """Crop/flatten/resize the source image to `target_w` wide — or less, if
    that would make it taller than `max_h`. Returns (img, w, h) — img is the
    real color image, still at full quality; callers decide separately
    whether to binarize it (see _binarize)."""
    raw = Image.open(image_path)
    raw = raw.crop(_tight_bbox(raw))  # drop the fully-transparent margin
    img = _flatten_onto_bg(raw, BG)
    scale = target_w / img.width
    if max_h is not None:
        scale = min(scale, max_h / img.height)
    w, h = round(img.width * scale), round(img.height * scale)
    return img.resize((w, h), Image.LANCZOS), w, h


def _binarize(img, threshold=BW_THRESHOLD):
    """Pure black/white version of img — see the module docstring for why
    the tile-by-tile reveal always uses this instead of the real colors."""
    gray = img.convert("L")
    bw = gray.point(lambda p: 255 if p > threshold else 0)
    return bw.convert("RGB")


def _slice_tiles(img, out_dir, prefix=""):
    """Slice img into a fixed TILE_COLUMNS grid (tile size scales with the
    image, so a bigger image doesn't balloon into far more tiles / a much
    longer reveal). Returns tiles in dissolve (Bayer-dithered) order rather
    than raster order — see BAYER4 above — unless REVEAL_ORDER is "raster".
    Each tile is (rel_x, rel_y, tile_w, tile_h, fname), relative to img's
    own top-left corner."""
    target_w, target_h = img.size
    tile_px = max(MIN_TILE_PX, round(target_w / TILE_COLUMNS))

    tiles = []
    row = 0
    for top in range(0, target_h, tile_px):
        tile_h = min(tile_px, target_h - top)
        col = 0
        for left in range(0, target_w, tile_px):
            tile_w = min(tile_px, target_w - left)
            fname = f"{prefix}tile_{row}_{col}.png"
            img.crop((left, top, left + tile_w, top + tile_h)).save(os.path.join(out_dir, fname))
            dither = BAYER4[row % 4][col % 4]
            tiles.append((dither, left, top, tile_w, tile_h, fname))
            col += 1
        row += 1

    if REVEAL_ORDER == "raster":
        tiles.sort(key=lambda t: (t[2], t[1]))      # top, then left
    else:
        tiles.sort(key=lambda t: (t[0], t[2], t[1]))  # dither value, then top/left for stable ties
    return [t[1:] for t in tiles]


def _slice_bands(img, out_dir):
    """Slice img into BAND_COUNT full-height (or full-width) strips, in a
    single sweep (left-to-right or top-to-bottom per BAND_DIRECTION) rather
    than a scattered dissolve — a smooth continuous wipe instead of tiles
    popping in all over. Each band is (rel_x, rel_y, w, h, fname), relative
    to img's own top-left corner."""
    w, h = img.size
    bands = []
    if BAND_DIRECTION == "ttb":
        band_px = max(MIN_TILE_PX, round(h / BAND_COUNT))
        i = 0
        for top in range(0, h, band_px):
            band_h = min(band_px, h - top)
            fname = f"band_{i}.png"
            img.crop((0, top, w, top + band_h)).save(os.path.join(out_dir, fname))
            bands.append((0, top, w, band_h, fname))
            i += 1
    else:
        band_px = max(MIN_TILE_PX, round(w / BAND_COUNT))
        i = 0
        for left in range(0, w, band_px):
            band_w = min(band_px, w - left)
            fname = f"band_{i}.png"
            img.crop((left, 0, left + band_w, h)).save(os.path.join(out_dir, fname))
            bands.append((left, 0, band_w, h, fname))
            i += 1
    return bands


def push_memory(image_path, cx, cy, target_w=360, event_id=None,
                 clear_first=False, settle_after=False, host=EINK_HOST,
                 max_h=None, clear_rect=None):
    """Composite `image_path` and push a two-phase reveal to the tablet over
    SSH, centered at (cx, cy): first a binarized (pure black/white) version
    tile by tile — see the module docstring for why — then, in one final
    stage, the real color image over the whole area at once. The refreshed
    region is the tight bounding box of the subject itself (post-crop), not
    a full target_w square, so the panel only repaints where something
    actually appears.

    `clear_first` asks the viewer to blank the panel to background and flash
    before this event's tiles — pass it only for the first image of a touch
    (eink_hook's show() sets this, gated by CLEAR_BEFORE_TOUCH), not for
    every image a single touch ends up showing, so the panel flashes once
    per touch instead of once per image.

    `settle_after` asks the viewer for one more flash once this event's
    color stage is drawn — re-driving it with a full-quality waveform so it
    settles crisp instead of staying at whatever the tile-by-tile reveal
    left it at. Pass it for a touch's last image (eink_hook's
    append_latest() sets this).

    `max_h` caps the height too, so the image fits a target_w x max_h box.
    `clear_rect` (x, y, w, h) blanks that area to BG first, in one partial
    update — for reusing a spot another image already occupies.

    Returns the event id used."""
    event_id = event_id or str(int(time.time() * 1000))

    with tempfile.TemporaryDirectory() as tmp:
        color_img, w, h = _prepare_image(image_path, target_w, max_h)
        bw_img = _binarize(color_img)
        if REVEAL_STYLE == "band":
            reveal = _slice_bands(bw_img, tmp)
            reveal_hold_ms = BAND_HOLD_MS
        else:
            reveal = _slice_tiles(bw_img, tmp, prefix="bw_")
            reveal_hold_ms = TILE_HOLD_MS
        ox, oy = round(cx - w / 2), round(cy - h / 2)
        stages = [
            {"x": ox + left, "y": oy + top, "image": fname, "hold_ms": reveal_hold_ms}
            for left, top, tw, th, fname in reveal
        ]
        stages[-1]["hold_ms"] = FINAL_HOLD_MS

        color_fname = "color_final.png"
        color_img.save(os.path.join(tmp, color_fname))
        stages.append({"x": ox, "y": oy, "image": color_fname, "hold_ms": FINAL_HOLD_MS})

        extra = []
        if clear_rect:
            bx, by, bw, bh = clear_rect
            Image.new("RGB", (bw, bh), BG).save(os.path.join(tmp, "blank.png"))
            extra.append(os.path.join(tmp, "blank.png"))
            stages.insert(0, {"x": bx, "y": by, "image": "blank.png", "hold_ms": 0})

        manifest = {
            "clear_first": clear_first,
            "settle_after": settle_after,
            "bg": list(BG),  # single source of truth for the fill color a
                             # clear_first flash reveals — the viewer no
                             # longer hardcodes its own copy of this
            "stages": stages,
        }
        manifest_path = os.path.join(tmp, "manifest.json")
        with open(manifest_path, "w") as f:
            json.dump(manifest, f)

        remote_dir = f"{EINK_APP_DIR}/events/{event_id}"
        subprocess.run(["ssh", f"root@{host}", f"mkdir -p {remote_dir}"], check=True)
        subprocess.run(
            [
                "scp",
                "-q",
                *[os.path.join(tmp, fname) for *_, fname in reveal],
                os.path.join(tmp, color_fname),
                *extra,
                manifest_path,
                f"root@{host}:{remote_dir}/",
            ],
            check=True,
        )
        # READY last and separately: the viewer only picks up a dir once this
        # file exists, so the manifest/frames above are always complete first.
        subprocess.run(["ssh", f"root@{host}", f"touch {remote_dir}/READY"], check=True)

    return event_id


def push_text(paragraphs, x, y, w, px=TEXT_PX, event_id=None, clear_first=False,
              clear_rect=None, host=EINK_HOST):
    """Have the viewer write `paragraphs` by hand (stroke by stroke, the
    riddle way — see eink-viewer/src/ink.rs) into a box `w` px wide whose
    top-left is (x, y). The viewer stacks paragraphs with TEXT_PARAGRAPH_GAP
    between them; the viewer word-wraps on spaces, so Japanese text should
    be written with spaces between words (分かち書き). Only the text goes
    over the wire — the strokes are traced on the tablet.

    `clear_rect` (x, y, w, h) blanks that area to BG first, in one partial
    update, so text left there by an earlier touch doesn't show through.

    Returns the event id used."""
    event_id = event_id or str(int(time.time() * 1000))
    stages = [{"x": x, "w": w, "text": text, "px": px, "gap": TEXT_PARAGRAPH_GAP,
               "hold_ms": TEXT_PARAGRAPH_PAUSE_MS,
               "points_per_step": TEXT_POINTS_PER_STEP, "step_ms": TEXT_STEP_MS}
              for text in paragraphs]
    stages[0]["y"] = y  # later paragraphs continue below, laid out by the viewer
    stages[-1]["hold_ms"] = 0
    manifest = {"clear_first": clear_first, "bg": list(BG), "stages": stages}

    with tempfile.TemporaryDirectory() as tmp:
        files = []
        if clear_rect:
            cx, cy, cw, ch = clear_rect
            blank = os.path.join(tmp, "blank.png")
            Image.new("RGB", (cw, ch), BG).save(blank)
            files.append(blank)
            stages.insert(0, {"x": cx, "y": cy, "image": "blank.png", "hold_ms": 0})
        manifest_path = os.path.join(tmp, "manifest.json")
        with open(manifest_path, "w") as f:
            json.dump(manifest, f, ensure_ascii=False)
        remote_dir = f"{EINK_APP_DIR}/events/{event_id}"
        subprocess.run(["ssh", f"root@{host}", f"mkdir -p {remote_dir}"], check=True)
        subprocess.run(["scp", "-q", *files, manifest_path, f"root@{host}:{remote_dir}/"], check=True)
        subprocess.run(["ssh", f"root@{host}", f"touch {remote_dir}/READY"], check=True)
    return event_id


if __name__ == "__main__":
    import sys

    if len(sys.argv) >= 3 and sys.argv[1] == "--text":
        eid = push_text(sys.argv[2:], x=160, y=1500, w=1300, clear_first=True)
    elif len(sys.argv) == 4:
        image_path, cx, cy = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
        eid = push_memory(image_path, cx, cy)
    else:
        print(f"usage: {sys.argv[0]} <image.png> <center_x> <center_y>\n"
              f"       {sys.argv[0]} --text <paragraph> [<paragraph> ...]")
        sys.exit(1)
    print(f"pushed event {eid}")
