#!/usr/bin/env python3
"""Push a "materializing" memory image to the plush_memory_viewer AppLoad app
on the reMarkable Paper Pro.

This targets the plush_memory_viewer Rust app (../eink-viewer): it watches
`<app dir>/events/<id>/` for a manifest.json + PNG frames + a READY sentinel,
then blits each frame in turn with a partial (non-flashing) e-ink refresh,
holding each one for `hold_ms` before moving to the next. Since the panel
can't alpha-blend, the "fade in" is faked here on the PC side by rendering
several frames of the source image pre-blended toward the paper background
at increasing strength (25% / 50% / 75% / 100%) and letting the viewer just
flip through them quickly — the same "soft materialize instead of a hard
pop-in" effect Tom's replies use in MaximeRivest/riddle
(https://github.com/MaximeRivest/riddle), built on the same qtfb partial-
refresh mechanism that project's src/qtfb.rs documents.
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

LEVELS = (0.25, 0.5, 0.75, 1.0)
HOLD_MS = (110, 110, 110, 450)


def _lerp_image(img, bg, t):
    """Blend img toward a solid bg color at strength t (1.0 = img, 0.0 = bg)."""
    bg_img = Image.new("RGB", img.size, bg)
    return Image.blend(bg_img, img, t)


def _make_stage_frames(image_path, out_dir, target_w):
    img = Image.open(image_path).convert("RGB")
    scale = target_w / img.width
    target_h = round(img.height * scale)
    img = img.resize((target_w, target_h), Image.LANCZOS)

    stage_files = []
    for i, t in enumerate(LEVELS):
        frame = _lerp_image(img, BG, t)
        fname = f"stage{i}.png"
        frame.save(os.path.join(out_dir, fname))
        stage_files.append(fname)
    return stage_files, target_w, target_h


def push_memory(image_path, x, y, target_w=360, event_id=None, host=EINK_HOST):
    """Composite `image_path` into a materialize sequence centered at (x, y)
    (top-left of the resized image) and push it to the tablet over SSH.
    Returns the event id used."""
    event_id = event_id or str(int(time.time() * 1000))

    with tempfile.TemporaryDirectory() as tmp:
        stage_files, w, h = _make_stage_frames(image_path, tmp, target_w)
        manifest = {
            "stages": [
                {"x": x, "y": y, "image": fname, "hold_ms": hold}
                for fname, hold in zip(stage_files, HOLD_MS)
            ]
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
                *[os.path.join(tmp, f) for f in stage_files],
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
        print(f"usage: {sys.argv[0]} <image.png> <x> <y>")
        sys.exit(1)
    image_path, x, y = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    eid = push_memory(image_path, x, y)
    print(f"pushed event {eid}")
