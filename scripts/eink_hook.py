"""Optional e-ink mirror of plush_memory's touch-memory collage onto the
reMarkable Paper Pro, via eink_memory_push.push_memory().

This is purely additive: every entry point here is fire-and-forget
(asyncio.create_task) and swallows its own errors, so the existing
WebSocket/HTML path (touch_image_camera_new.py + plush_memory_camera.html)
behaves exactly as before whether or not the tablet is reachable.

Unlike the HTML's freely overlapping collage, images here go into fixed,
non-overlapping slots (see SLOTS) on the Paper Pro panel's native
1620x2160 resolution. There is no rotation here either (unlike the HTML's
random `rotate(...)`) — the e-ink viewer only does axis-aligned partial
updates, so stages stay unrotated rectangles. Likewise HIDE_IMAGE /
HIDE_IMAGE_ALL are intentionally not mirrored: e-ink doesn't flicker, so a
retired memory is simply left on the panel until the next one overwrites
that spot — more "diary page" than "disappearing toast", and one less thing
to keep in sync with the HTML's fade-out timing.

Unlike the HTML, the e-ink page is laid out like a picture book: the
illustrations fill the upper part of the panel, and the bottom is a text
area where the viewer writes, by hand, the touched part's memory (right
after the touch's first image) and then a line recording this touch itself (right after the newly generated "latest" image) — see
memory_text.py.
"""
import asyncio
import os
import random
import time

import rospy

import eink_memory_push
import memory_text

ENABLED = True

SCREEN_W, SCREEN_H = 1620, 2160


# Picture area (y 20-1420), above the text. The latest image gets the
# middle; around it, SLOTS are SLOT_PX boxes laid out so none of them
# overlap each other or the latest one: four down each side, plus one above
# and one below the middle. Images are fit inside their box (width and
# height), so even a tall cutout stays in its slot.
SLOT_PX = 320
LATEST_PX = 520
LATEST_CENTER = (810, 720)
SLOTS = (
    [(185, y) for y in (180, 540, 900, 1260)]
    + [(1435, y) for y in (180, 540, 900, 1260)]
    + [(810, 180), (810, 1260)]
)

# Picture-book text area, below the pictures. Two fixed slots at
# eink_memory_push.TEXT_PX: the memory (three sentences), then this
# touch's closing lines (two). TEXT_W is wide enough that every sentence
# in data/memory_texts.json, ja and en, fits on one line.
# Each slot is blanked just before it's written, so the previous touch's
# text never shows through when CLEAR_BEFORE_TOUCH is off.
TEXT_X, TEXT_W = 70, 1480
MEMORY_TEXT_Y = 1480
TOUCH_LINE_Y = 1850
_MEMORY_SLOT = (TEXT_X - 20, MEMORY_TEXT_Y - 20, TEXT_W + 40, TOUCH_LINE_Y - MEMORY_TEXT_Y - 10)
_TOUCH_SLOT = (TEXT_X - 20, TOUCH_LINE_Y - 20, TEXT_W + 40, SCREEN_H - TOUCH_LINE_Y)

# Slots not yet used on the current page, in the order they'll be handed
# out. show() starts a fresh page; when they run out (many TMP_IMAGEs during
# a slow generation), the slots are reused — each push blanks its box first.
_free_slots = []


def _next_slot():
    if not _free_slots:
        _free_slots.extend(random.sample(SLOTS, len(SLOTS)))
    return _free_slots.pop(0)


def _box(center, size):
    cx, cy = center
    return (cx - size // 2, cy - size // 2, size, size)

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


async def _push(kind: str, img_id, center, size: int,
                 event_id: str, clear_first: bool = False, settle_after: bool = False):
    if not ENABLED:
        return
    path = _image_path(kind, img_id)
    if path is None:
        return
    cx, cy = center
    loop = asyncio.get_event_loop()
    try:
        # push_memory crops to the subject's own tight bounding box, fits it
        # in the size x size box and centers it on (cx, cy). The box is
        # blanked first (unless the whole page just was), so a reused slot
        # doesn't keep the edges of whatever image was there before.
        await loop.run_in_executor(
            None, lambda: eink_memory_push.push_memory(
                path, cx, cy, size, event_id, clear_first, settle_after,
                max_h=size, clear_rect=None if clear_first else _box(center, size)))
    except Exception as e:
        rospy.logwarn(f"eink_hook: push failed for {event_id}: {e}")


async def _delayed_push(kind, img_id, center, size, delay_sec, event_id,
                         clear_first=False, settle_after=False):
    if delay_sec > 0:
        await asyncio.sleep(delay_sec)
    await _push(kind, img_id, center, size, event_id, clear_first, settle_after)


async def _push_text(paragraphs, y, slot, event_id, clear_first=False):
    if not ENABLED or not paragraphs:
        return
    loop = asyncio.get_event_loop()
    try:
        await loop.run_in_executor(
            None, lambda: eink_memory_push.push_text(
                paragraphs, TEXT_X, y, TEXT_W, event_id=event_id,
                clear_first=clear_first, clear_rect=slot))
    except Exception as e:
        rospy.logwarn(f"eink_hook: text push failed for {event_id}: {e}")


async def _first_image_then_memory(kind, push_first):
    """Write the part's memory once the touch's first image (and its
    clear_first blank) has been handed to the viewer, so the blank can't
    wipe the text. The viewer runs events one at a time, so the memory is
    written while the rest of the batch is still on its way."""
    await push_first
    await _push_text(memory_text.memory_paragraphs(kind), MEMORY_TEXT_Y, _MEMORY_SLOT,
                     f"text_memory_{kind}_{int(time.time() * 1000)}")


def show(kind: str, selected_ids):
    """Mirror a SHOW_IMAGE broadcast: stagger up to 8 ids into free SLOTS
    on a fresh page, same 2s-apart pacing as the HTML. Only the very first image of this
    touch asks the viewer for its one blank-and-flash (see push_memory's
    clear_first, gated by CLEAR_BEFORE_TOUCH) — a touch can end up pushing
    many images (this batch, plus later TMP_IMAGE/APPEND_IMAGE calls), and
    flashing for every one of them was the actual source of the flicker,
    not the per-tile reveal itself.

    No settle_after here (or in append_latest, below) any more: measured on
    -device, the final color stage settles cleanly from its own ordinary
    UFAST partial update — it's a single update, not the long run of
    sequential tile updates that caused fading, so it never needed the
    extra full-quality waveform. request_full_refresh() also flashes the
    *whole* panel (the qtfb message carries no region), so skipping it
    stops this touch's color settle from disturbing every other memory
    already sitting elsewhere on screen."""
    if not ENABLED:
        return
    _free_slots[:] = random.sample(SLOTS, len(SLOTS))
    if not selected_ids:
        # Nothing for the grid (only one image exists, held back as the
        # latest): still start a fresh page and write the memory.
        asyncio.create_task(_push_text(
            memory_text.memory_paragraphs(kind), MEMORY_TEXT_Y, _MEMORY_SLOT,
            f"text_memory_{kind}_{int(time.time() * 1000)}",
            clear_first=eink_memory_push.CLEAR_BEFORE_TOUCH))
        return
    for idx, img_id in enumerate(selected_ids[:8]):
        push = _delayed_push(
            kind, img_id, _next_slot(), SLOT_PX,
            idx * STAGGER_SEC, f"show_{kind}_{img_id}",
            clear_first=(idx == 0 and eink_memory_push.CLEAR_BEFORE_TOUCH),
        )
        asyncio.create_task(_first_image_then_memory(kind, push) if idx == 0 else push)


def tmp(kind: str, img_id):
    """Mirror a TMP_IMAGE broadcast: one image in the next free slot."""
    if not ENABLED:
        return
    asyncio.create_task(_push(kind, img_id, _next_slot(), SLOT_PX, f"tmp_{kind}_{img_id}"))


def append_latest(kind: str, img_id):
    """Mirror an APPEND_IMAGE broadcast: the newly generated/fallback image,
    larger, at the fixed 'latest' spot. No clear_first or settle_after here
    — see show()'s docstring for why settle_after isn't needed anywhere any
    more, and clear_first stays SHOW_IMAGE-only to match the HTML's own
    single reset point per touch."""
    if not ENABLED:
        return
    asyncio.create_task(_latest_then_touch_line(kind, img_id))


async def _latest_then_touch_line(kind, img_id):
    """The newest memory, then the line that records this touch under it."""
    await _push(kind, img_id, LATEST_CENTER, LATEST_PX, f"latest_{kind}_{img_id}")
    await _push_text(memory_text.touch_line(kind), TOUCH_LINE_Y, _TOUCH_SLOT,
                     f"text_touch_{kind}_{int(time.time() * 1000)}")
