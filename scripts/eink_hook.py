"""Optional e-ink mirror of plush_memory's touch-memory collage onto the
reMarkable Paper Pro, via eink_memory_push.push_memory().

This is purely additive: every entry point here is fire-and-forget
(asyncio.create_task) and swallows its own errors, so the existing
WebSocket/HTML path (touch_image_camera_new.py + plush_memory_camera.html)
behaves exactly as before whether or not the tablet is reachable.

The position layout mirrors plush_memory_camera.html's `positions8` /
`position_latest` tables (percent-of-viewport, image anchored at its center),
reinterpreted against the Paper Pro panel's native 1620x2160 resolution
instead of a landscape monitor. There is no rotation here (unlike the HTML's
random `rotate(...)`) — the e-ink viewer only does axis-aligned partial
updates, so stages stay unrotated rectangles. Likewise HIDE_IMAGE /
HIDE_IMAGE_ALL are intentionally not mirrored: e-ink doesn't flicker, so a
retired memory is simply left on the panel until the next one overwrites
that spot — more "diary page" than "disappearing toast", and one less thing
to keep in sync with the HTML's fade-out timing.
"""
import asyncio
import os
import random

import rospy

import eink_memory_push

ENABLED = True

SCREEN_W, SCREEN_H = 1620, 2160

# plush_memory_camera.html's `positions8` (top%, left%), unchanged.
POSITIONS8 = [
    (0.25, 0.10),
    (0.18, 0.85),
    (0.32, 0.41),
    (0.52, 0.18),
    (0.58, 0.70),
    (0.70, 0.10),
    (0.74, 0.42),
    (0.72, 0.87),
]
# html's `position_latest`.
POSITION_LATEST = (0.47, 0.50)
# html's TMP_TOP_MIN/MAX, TMP_LEFT_MIN/MAX (fractions instead of percent).
TMP_TOP_RANGE = (0.15, 0.75)
TMP_LEFT_RANGE = (0.10, 0.90)

# html: wrapper width 22vw * JS scale (0.45-0.6 random) for a normal image,
# 28vw * fixed 0.7 for the latest one. We don't replicate the random JS
# scale — just its rough midpoint — since the materialize-frame approach
# already needs a single fixed size per call.
NORMAL_WIDTH_PX = round(0.22 * SCREEN_W * 0.5)  # ~178px
LATEST_WIDTH_PX = round(0.28 * SCREEN_W * 0.7)  # ~317px

# html's `displaying_time`: stagger between images in a SHOW_IMAGE batch.
STAGGER_SEC = 2.0

_image_dir = os.path.join(os.path.dirname(__file__), "../data/images")

# Set by touch_image_camera_new.py's main() right after it reads its own
# ~display_target, so this looks in the same data/images/<style>/ folder
# that node is actually writing new generations into.
IMAGE_STYLE = "shepard"


def _image_path(kind: str, img_id):
    """The generated image for this id, under data/images/<style>/ (see
    IMAGE_STYLE above). Mirrors touch_image_camera_new.py's _list_ids: only
    "classic" also falls back to data/images/ directly, for images generated
    there before the data/images/<style>/ split — those are all opaque/
    flat-color, so safe for an HTML (classic) run to pick up. "shepard"
    (eink) gets no such fallback: an unstyled root image isn't guaranteed
    transparent/pen-and-ink, and showing an opaque one on the e-ink viewer
    is exactly the style-mixing this split exists to prevent. In practice
    this function only runs at all when eink_hook.ENABLED (i.e. IMAGE_STYLE
    == "shepard"), but it mirrors the classic-only fallback rule anyway in
    case that ever changes."""
    fname = f"generated_drawing_{img_id}_{kind}.png"
    dirs = (
        (os.path.join(_image_dir, IMAGE_STYLE), _image_dir)
        if IMAGE_STYLE == "classic"
        else (os.path.join(_image_dir, IMAGE_STYLE),)
    )
    for d in dirs:
        path = os.path.join(d, fname)
        if os.path.exists(path):
            return path
    return None


async def _push(kind: str, img_id, top_frac: float, left_frac: float, width_px: int,
                 event_id: str, clear_first: bool = False, settle_after: bool = False):
    if not ENABLED:
        return
    path = _image_path(kind, img_id)
    if path is None:
        return
    cx = left_frac * SCREEN_W
    cy = top_frac * SCREEN_H
    loop = asyncio.get_event_loop()
    try:
        # push_memory crops to the subject's own tight bounding box and
        # centers that on (cx, cy) — it no longer assumes a square target_w
        # region, since a cutout's post-crop aspect ratio isn't 1:1.
        await loop.run_in_executor(
            None, eink_memory_push.push_memory, path, cx, cy, width_px, event_id,
            clear_first, settle_after,
        )
    except Exception as e:
        rospy.logwarn(f"eink_hook: push failed for {event_id}: {e}")


async def _delayed_push(kind, img_id, top_frac, left_frac, width_px, delay_sec, event_id,
                         clear_first=False, settle_after=False):
    if delay_sec > 0:
        await asyncio.sleep(delay_sec)
    await _push(kind, img_id, top_frac, left_frac, width_px, event_id, clear_first, settle_after)


def show(kind: str, selected_ids):
    """Mirror a SHOW_IMAGE broadcast: stagger up to 8 ids onto POSITIONS8,
    same 2s-apart pacing as the HTML. Only the very first image of this
    touch asks the viewer for its one blank-and-flash (see push_memory's
    clear_first, gated by CLEAR_BEFORE_TOUCH) — a touch can end up pushing
    many images (this batch, plus later TMP_IMAGE/APPEND_IMAGE calls), and
    flashing for every one of them was the actual source of the flicker,
    not the per-tile reveal itself.

    The *last* image of this batch also asks for settle_after. Normally
    APPEND_IMAGE (append_latest, below) is the true last image of a touch
    and owns that job — but when generation is disabled/fails and no
    fallback is available, APPEND_IMAGE never fires at all (_pick_fallback_id
    returns nothing), and this batch is all the touch ever shows. Without
    this, that case left the whole batch's tile-reveal fading uncorrected
    forever. A touch with a successful APPEND_IMAGE just settles twice."""
    if not ENABLED:
        return
    positions = random.sample(POSITIONS8, min(len(selected_ids), len(POSITIONS8)))
    last_idx = len(positions) - 1
    for idx, img_id in enumerate(selected_ids[: len(positions)]):
        top, left = positions[idx]
        asyncio.create_task(
            _delayed_push(
                kind, img_id, top, left, NORMAL_WIDTH_PX,
                idx * STAGGER_SEC, f"show_{kind}_{img_id}",
                clear_first=(idx == 0 and eink_memory_push.CLEAR_BEFORE_TOUCH),
                settle_after=(idx == last_idx),
            )
        )


def tmp(kind: str, img_id):
    """Mirror a TMP_IMAGE broadcast: one image at a random spot."""
    if not ENABLED:
        return
    top = random.uniform(*TMP_TOP_RANGE)
    left = random.uniform(*TMP_LEFT_RANGE)
    asyncio.create_task(_push(kind, img_id, top, left, NORMAL_WIDTH_PX, f"tmp_{kind}_{img_id}"))


def append_latest(kind: str, img_id):
    """Mirror an APPEND_IMAGE broadcast: the newly generated/fallback image,
    larger, at the fixed 'latest' spot — and this touch's last image, so it
    asks for settle_after (push_memory): one flash *after* its tiles are
    drawn to settle them crisp, with no blanking beforehand (no second
    clear_first — that would re-blank the whole panel mid-touch)."""
    if not ENABLED:
        return
    top, left = POSITION_LATEST
    asyncio.create_task(
        _push(kind, img_id, top, left, LATEST_WIDTH_PX, f"latest_{kind}_{img_id}", settle_after=True)
    )
