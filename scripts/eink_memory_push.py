#!/usr/bin/env python3
"""Push a "materializing" memory image to the plush_memory_viewer AppLoad app
on the reMarkable Paper Pro.

This targets the plush_memory_viewer Rust app (../eink-viewer): it watches
`<app dir>/events/<id>/` for a manifest.json + PNG frames + a READY sentinel,
then blits each frame in turn with a partial (non-flashing) e-ink refresh,
holding each one for `hold_ms` before moving to the next.

The "materializing" look is small *tiles* of the final image, revealed one
at a time in raster order (top-left to bottom-right) rather than the whole
picture fading in at once. This mirrors how MaximeRivest/riddle
(https://github.com/MaximeRivest/riddle) draws Tom's ink replies: it
traces each glyph down to a 1px-wide stroke (src/script.rs: rasterize with
ab_glyph, skeletonize with Zhang-Suen thinning, trace into point sequences)
and draws it as many tiny, fast partial refreshes rather than one big one —
small update regions are what actually keeps it from flashing, more than
the takeover-vs-windowed distinction. We don't have stroke paths for a
photo, so tiles are the raster equivalent: each one is a small, cheap qtfb
partial update (the same mechanism src/qtfb.rs documents there), and
revealing them in sequence reads as the picture being filled in.
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
BG = (254, 250, 245)  # matches plush_memory's HTML body background #fefaf5

# "dither": scattered (Bayer) reveal order — more natural-looking materialize.
# "raster": top-left to bottom-right, like a scan — simpler, more mechanical.
REVEAL_ORDER = "dither"

TILE_COLUMNS = 8       # fixed column count, not a fixed pixel size — a
                       # larger image gets proportionally larger tiles (and
                       # roughly the same total tile count / reveal time,
                       # ~64 tiles either way) instead of exploding into way
                       # more tiny tiles the bigger it is
MIN_TILE_PX = 8        # floor, so a narrow image doesn't get degenerate tiles
TILE_HOLD_MS = 45      # delay between tiles — more time for the panel's
                       # gray levels to actually settle before the next
                       # partial update, not just a pacing choice
FINAL_HOLD_MS = 450    # pause on the finished picture before moving on


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


def _make_tiles(image_path, out_dir, target_w):
    """Crop/flatten/resize the source image, then slice it into a fixed
    TILE_COLUMNS grid (tile size scales with the image, so a bigger image
    doesn't balloon into far more tiles / a much longer reveal). Tiles are
    returned in dissolve (Bayer-dithered) order rather than raster order —
    see BAYER4 above. Returns (tiles, w, h) where each tile is (rel_x,
    rel_y, tile_w, tile_h, fname), relative to the image's own top-left
    corner."""
    raw = Image.open(image_path)
    raw = raw.crop(_tight_bbox(raw))  # drop the fully-transparent margin
    img = _flatten_onto_bg(raw, BG)
    scale = target_w / img.width
    target_h = round(img.height * scale)
    img = img.resize((target_w, target_h), Image.LANCZOS)

    tile_px = max(MIN_TILE_PX, round(target_w / TILE_COLUMNS))

    tiles = []
    row = 0
    for top in range(0, target_h, tile_px):
        tile_h = min(tile_px, target_h - top)
        col = 0
        for left in range(0, target_w, tile_px):
            tile_w = min(tile_px, target_w - left)
            fname = f"tile_{row}_{col}.png"
            img.crop((left, top, left + tile_w, top + tile_h)).save(os.path.join(out_dir, fname))
            dither = BAYER4[row % 4][col % 4]
            tiles.append((dither, left, top, tile_w, tile_h, fname))
            col += 1
        row += 1

    if REVEAL_ORDER == "raster":
        tiles.sort(key=lambda t: (t[2], t[1]))      # top, then left
    else:
        tiles.sort(key=lambda t: (t[0], t[2], t[1]))  # dither value, then top/left for stable ties
    tiles = [t[1:] for t in tiles]
    return tiles, target_w, target_h


def push_memory(image_path, cx, cy, target_w=360, event_id=None, clear_first=False, host=EINK_HOST):
    """Composite `image_path`, slice it into small tiles, and push a reveal
    sequence (top-left to bottom-right, one tiny partial refresh per tile)
    centered at (cx, cy) to the tablet over SSH. The refreshed region is the
    tight bounding box of the subject itself (post-crop), not a full
    target_w square, so the panel only repaints where something actually
    appears.

    `clear_first` asks the viewer to do one full-panel flash before this
    event's tiles — pass it only for the first image of a touch (eink_hook
    sets this), not for every image a single touch ends up showing, so the
    panel flashes once per touch instead of once per image.

    Returns the event id used."""
    event_id = event_id or str(int(time.time() * 1000))

    with tempfile.TemporaryDirectory() as tmp:
        tiles, w, h = _make_tiles(image_path, tmp, target_w)
        ox, oy = round(cx - w / 2), round(cy - h / 2)
        stages = [
            {"x": ox + left, "y": oy + top, "image": fname, "hold_ms": TILE_HOLD_MS}
            for left, top, tw, th, fname in tiles
        ]
        stages[-1]["hold_ms"] = FINAL_HOLD_MS
        manifest = {"clear_first": clear_first, "stages": stages}
        manifest_path = os.path.join(tmp, "manifest.json")
        with open(manifest_path, "w") as f:
            json.dump(manifest, f)

        remote_dir = f"{EINK_APP_DIR}/events/{event_id}"
        subprocess.run(["ssh", f"root@{host}", f"mkdir -p {remote_dir}"], check=True)
        subprocess.run(
            [
                "scp",
                "-q",
                *[os.path.join(tmp, fname) for *_, fname in tiles],
                manifest_path,
                f"root@{host}:{remote_dir}/",
            ],
            check=True,
        )
        # READY last and separately: the viewer only picks up a dir once this
        # file exists, so the manifest/frames above are always complete first.
        subprocess.run(["ssh", f"root@{host}", f"touch {remote_dir}/READY"], check=True)

    return event_id


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 4:
        print(f"usage: {sys.argv[0]} <image.png> <center_x> <center_y>")
        sys.exit(1)
    image_path, cx, cy = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    eid = push_memory(image_path, cx, cy)
    print(f"pushed event {eid}")
