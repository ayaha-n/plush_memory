"""Optional e-ink mirror of plush_memory's touch-memory collage onto the
reMarkable Paper Pro, via eink_memory_push.push_memory().

This is purely additive: every entry point here is fire-and-forget
(asyncio.create_task) and swallows its own errors, so the existing
WebSocket/HTML path (touch_image_camera_new.py + plush_memory_camera.html)
behaves exactly as before whether or not the tablet is reachable.

Unlike the HTML's freely overlapping collage, images here go into fixed,
non-overlapping slots (see LAYOUTS) on the Paper Pro panel's native
resolution, 1620x2160 held upright or 2160x1620 turned on its side. There
is no per-image rotation here either (unlike the HTML's
random `rotate(...)`) — the e-ink viewer only does axis-aligned partial
updates, so stages stay unrotated rectangles. Likewise HIDE_IMAGE /
HIDE_IMAGE_ALL are intentionally not mirrored: e-ink doesn't flicker, so a
retired memory is simply left on the panel until the next one overwrites
that spot — more "diary page" than "disappearing toast", and one less thing
to keep in sync with the HTML's fade-out timing.

Unlike the HTML, the e-ink page is laid out like a picture book, and
drawn in reading order: first the touched part's memory is written by hand
at the top, then the illustrations appear in the middle (top-left first),
and last, under the newest one, the lines recording this touch itself —
see memory_text.py. To keep that order, every event goes through one queue
(_enqueue) and is pushed only after the previous one has landed on the
tablet; the viewer then runs them strictly one after another.
"""
import asyncio
import os
import random
import time

import rospy

import eink_memory_push
import memory_text

ENABLED = True

# The page is laid out for whichever way the tablet is turned when the touch
# starts (the viewer reports it — see eink_memory_push.read_orientation),
# and the whole page keeps that layout; the viewer only applies a rotation
# between pages.
#
# Picture area between the memory text above and the closing lines below.
# The latest image gets the middle; around it, SLOTS are SLOT_PX boxes laid
# out so none of them overlap each other or the latest one. Images are fit
# inside their box (width and height), so even a tall cutout stays in its
# slot.
#
# Picture-book text: two fixed slots at eink_memory_push.TEXT_PX, the memory
# (three sentences) above the pictures, this touch's closing lines (two)
# below them. TEXT_W is wide enough that every sentence in
# data/memory_texts.json, ja and en, fits on one line. Each slot is blanked
# just before it's written, so the previous touch's text never shows
# through when CLEAR_BEFORE_TOUCH is off.
SLOT_PX = 320
LATEST_PX = 520


class Layout:
    def __init__(self, name, screen, latest_center, slots, text_x, text_w,
                 memory_text_y, touch_line_y):
        self.name = name
        self.latest_center = latest_center
        self.slots = slots
        self.text_x, self.text_w = text_x, text_w
        self.memory_text_y, self.touch_line_y = memory_text_y, touch_line_y
        _, screen_h = screen
        self.memory_slot = (text_x - 20, memory_text_y - 20, text_w + 40, 350)
        self.touch_slot = (text_x - 20, touch_line_y - 20, text_w + 40, screen_h - touch_line_y + 20)


LAYOUTS = {
    # 1620x2160, pictures in y 410-1870: four down each side, plus one above
    # and one below the middle.
    "portrait": Layout(
        "portrait", (1620, 2160), latest_center=(810, 1140),
        slots=[(185, y) for y in (570, 950, 1330, 1710)]
        + [(1435, y) for y in (570, 950, 1330, 1710)]
        + [(810, 570), (810, 1710)],
        text_x=70, text_w=1480, memory_text_y=60, touch_line_y=1920),
    # 2160x1620, pictures in y 400-1380: two columns of three on each side.
    "landscape": Layout(
        "landscape", (2160, 1620), latest_center=(1080, 890),
        slots=[(x, y) for x in (200, 540, 1620, 1960) for y in (560, 890, 1220)],
        text_x=70, text_w=2020, memory_text_y=50, touch_line_y=1400),
}

# The layout of the page being drawn, set by show() once it has asked the
# tablet which way it's turned.
_layout = LAYOUTS["portrait"]

# Slots not yet used on the current page, in the order they'll be handed
# out. show() starts a fresh page; when they run out (many TMP_IMAGEs during
# a slow generation), the slots are reused — each push blanks its box first.
_free_slots = []


def _reading_order(slots):
    """Top-left first, row by row."""
    return sorted(slots, key=lambda c: (c[1], c[0]))


def _next_slot():
    if not _free_slots:
        _free_slots.extend(_reading_order(_layout.slots))
    return _free_slots.pop(0)


def _box(center, size):
    cx, cy = center
    return (cx - size // 2, cy - size // 2, size, size)


# One queue for every e-ink event, pushed strictly one at a time, so they
# reach the tablet (and get drawn) in the order they were asked for.
_queue = None
_seq = 0


def _enqueue(job):
    """Queue `job` (an async callable) behind everything already queued."""
    global _queue
    if _queue is None:
        _queue = asyncio.Queue()
        asyncio.get_event_loop().create_task(_run_queue())
    _queue.put_nowait(job)


async def _run_queue():
    while True:
        job = await _queue.get()
        await job()


def _event_id():
    """Ids sort in push order — the viewer picks up events by name."""
    global _seq
    _seq += 1
    return f"{int(time.time() * 1000):013d}_{_seq:05d}"


# A touch is only taken while no page is being drawn: from begin_page() (the
# touch is accepted) until end_page()'s turn in the queue comes and the
# tablet has drawn everything. Touches in between are ignored, not queued —
# false triggers are frequent, and queueing them kept the panel busy with
# pages nobody asked for.
DRAIN_POLL_SEC = 1.0
DRAIN_TIMEOUT_SEC = 300.0
_busy = False


def set_device_orientation(orientation: str):
    """Pass ~eink_orientation on to the viewer (eink_memory_push.
    write_orientation_conf). Pages are still laid out by what the viewer
    reports back, so a tablet that can't be reached just keeps its last
    setting."""
    try:
        eink_memory_push.write_orientation_conf(orientation)
    except ValueError as e:
        rospy.logerr(f"eink_hook: {e}")
    except Exception as e:
        rospy.logwarn(f"eink_hook: could not set tablet orientation ({e})")


def busy():
    return ENABLED and _busy


def begin_page():
    global _busy
    if ENABLED:
        _busy = True


def end_page():
    """Called once a touch has pushed everything it will push. The page
    stays busy until those pushes land and the tablet has drawn them."""
    if not ENABLED:
        return
    _enqueue(_finish_page)


async def _finish_page():
    loop = asyncio.get_event_loop()
    try:
        await loop.run_in_executor(None, eink_memory_push.push_page_end, _event_id())
    except Exception as e:
        rospy.logwarn(f"eink_hook: page-end push failed: {e}")
    await _wait_drained()


async def _wait_drained():
    global _busy
    loop = asyncio.get_event_loop()
    deadline = time.time() + DRAIN_TIMEOUT_SEC
    try:
        while time.time() < deadline:
            if await loop.run_in_executor(None, eink_memory_push.pending_events) == 0:
                return
            await asyncio.sleep(DRAIN_POLL_SEC)
        rospy.logwarn("eink_hook: tablet still drawing after timeout; accepting touches again")
    except Exception as e:
        rospy.logwarn(f"eink_hook: could not check tablet events ({e}); accepting touches again")
    finally:
        _busy = False


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


async def _push(kind: str, img_id, center, size: int):
    event_id = _event_id()
    path = _image_path(kind, img_id)
    if path is None:
        return
    cx, cy = center
    loop = asyncio.get_event_loop()
    try:
        # push_memory crops to the subject's own tight bounding box, fits it
        # in the size x size box and centers it on (cx, cy). The box is
        # blanked first, so a reused slot doesn't keep the edges of whatever
        # image was there before.
        await loop.run_in_executor(
            None, lambda: eink_memory_push.push_memory(
                path, cx, cy, size, f"{event_id}_{kind}_{img_id}",
                max_h=size, clear_rect=_box(center, size), layout=_layout.name))
    except Exception as e:
        rospy.logwarn(f"eink_hook: push failed for {event_id}: {e}")


async def _push_text(paragraphs, y, slot, clear_first=False, page_start=False):
    if not paragraphs:
        return
    event_id = _event_id()
    loop = asyncio.get_event_loop()
    try:
        await loop.run_in_executor(
            None, lambda: eink_memory_push.push_text(
                paragraphs, _layout.text_x, y, _layout.text_w, event_id=f"{event_id}_text",
                clear_first=clear_first, clear_rect=slot, layout=_layout.name,
                page_start=page_start))
    except Exception as e:
        rospy.logwarn(f"eink_hook: text push failed for {event_id}: {e}")


def show(kind: str, selected_ids):
    """Mirror a SHOW_IMAGE broadcast as a fresh page: the part's memory,
    then up to 8 ids in slots, drawn top-left first. The slots themselves
    are picked at random so a short batch still spreads over the page.

    Only the memory text asks the viewer for the touch's one blank of the
    panel (clear_first, gated by CLEAR_BEFORE_TOUCH) — a touch can push
    many events, and blanking for each was what used to cause flicker.
    No settle_after anywhere: measured on-device, the final color stage
    settles cleanly from its own single UFAST partial update, and
    request_full_refresh() would flash the *whole* panel."""
    if not ENABLED:
        return
    ids = selected_ids[:8]
    paragraphs = memory_text.memory_paragraphs(kind)

    async def start_page():
        global _layout
        loop = asyncio.get_event_loop()
        try:
            orientation = await loop.run_in_executor(None, eink_memory_push.read_orientation)
        except Exception as e:
            rospy.logwarn(f"eink_hook: could not read tablet orientation ({e}); assuming portrait")
            orientation = "portrait"
        _layout = LAYOUTS[orientation]
        chosen = random.sample(_layout.slots, len(ids))
        _free_slots[:] = _reading_order(c for c in _layout.slots if c not in chosen)
        await _push_text(paragraphs, _layout.memory_text_y, _layout.memory_slot,
                         clear_first=eink_memory_push.CLEAR_BEFORE_TOUCH, page_start=True)
        for img_id, slot in zip(ids, _reading_order(chosen)):
            await _push(kind, img_id, slot, SLOT_PX)

    _enqueue(start_page)


def tmp(kind: str, img_id):
    """Mirror a TMP_IMAGE broadcast: one image in the next free slot."""
    if not ENABLED:
        return
    _enqueue(lambda: _push(kind, img_id, _next_slot(), SLOT_PX))


def append_latest(kind: str, img_id):
    """Mirror an APPEND_IMAGE broadcast: the newly generated/fallback image,
    larger, in the middle — then the lines that record this touch, at the
    bottom of the page."""
    if not ENABLED:
        return
    lines = memory_text.touch_line(kind)
    _enqueue(lambda: _push(kind, img_id, _layout.latest_center, LATEST_PX))
    _enqueue(lambda: _push_text(lines, _layout.touch_line_y, _layout.touch_slot))
