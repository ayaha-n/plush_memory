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
import tempfile
import time

from PIL import Image, ImageDraw

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


def _grid(xs, ys):
    """Cells (x, y, w, h) between consecutive xs and ys."""
    return [(x0, y0, x1 - x0, y1 - y0)
            for y0, y1 in zip(ys, ys[1:]) for x0, x1 in zip(xs, xs[1:])]


class Layout:
    def __init__(self, name, screen, latest_center, slots, text_x, text_w,
                 memory_text_y, touch_line_y, scatter_cells):
        self.name = name
        self.latest_center = latest_center
        self.slots = slots
        self.scatter_cells = scatter_cells
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
        text_x=70, text_w=1480, memory_text_y=60, touch_line_y=1920,
        # Pictures in x 50-1570, y 400-1890: 3x3, the middle column and row
        # wider, so the middle cell is the big one.
        scatter_cells=_grid(xs=(50, 510, 1110, 1570), ys=(400, 850, 1440, 1890))),
    # 2160x1620, pictures in y 400-1380: two columns of three on each side.
    "landscape": Layout(
        "landscape", (2160, 1620), latest_center=(1080, 890),
        slots=[(x, y) for x in (200, 540, 1620, 1960) for y in (560, 890, 1220)],
        text_x=70, text_w=2020, memory_text_y=50, touch_line_y=1400,
        # Pictures in x 50-2110, y 390-1370: a full-height middle cell for the
        # big one, two by two on either side of it.
        scatter_cells=[(780, 390, 600, 980)]
        + _grid(xs=(50, 415, 780), ys=(390, 880, 1370))
        + _grid(xs=(1380, 1745, 2110), ys=(390, 880, 1370))),
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
        # The only worker: one job failing must not stop every page after it.
        try:
            await job()
        except Exception as e:
            rospy.logerr(f"eink_hook: queued job failed: {e!r}")


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
TOUCH_CLEAR_HOLD_MS = 2000  # Let the blank settle before writing over a cover.
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
    global _busy, _page_gen
    if ENABLED:
        _busy = True
        _page_gen += 1  # stops a cover being pushed, and its idle timer


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
        _arm_cover()


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
                page_start=page_start,
                clear_hold_ms=(TOUCH_CLEAR_HOLD_MS if clear_first
                               else eink_memory_push.CLEAR_HOLD_MS)))
    except Exception as e:
        rospy.logwarn(f"eink_hook: text push failed for {event_id}: {e}")


def _scattered(layout, n):
    """n (center, size, big) spots, in reading order (top-left to bottom-right
    by each cell's top-left corner). The biggest cell is always one of them
    and gets a big image, from the middle slot's size up to what fits; the
    rest are n-1 cells picked at random, each image a random size from the
    small slots' up to just under that. Each image sits at a random spot in
    its cell."""
    cells = layout.scatter_cells
    big = max(cells, key=lambda c: min(c[2], c[3]))
    chosen = random.sample([c for c in cells if c is not big], n - 1) + [big]
    spots = []
    for c in sorted(chosen, key=lambda c: (c[1], c[0])):
        x, y, w, h = c
        lo, hi = (LATEST_PX, min(w, h)) if c is big else (SLOT_PX, min(LATEST_PX - 1, w, h))
        size = random.randint(lo, max(lo, int(hi)))
        cx = x + size / 2 + random.uniform(0, max(0, w - size))
        cy = y + size / 2 + random.uniform(0, max(0, h - size))
        spots.append(((round(cx), round(cy)), size, c is big))
    return spots


# Without generation, how few images a page may show (see show()).
SCATTER_MIN = 3

# Set by show() when the page has no illustration still being generated
# (_enable_generation:=False): the id it was told would be appended as the
# latest, which instead goes in among the rest.
_scatter_latest = None


def show(kind: str, selected_ids, latest_id=None):
    """Mirror a SHOW_IMAGE broadcast as a fresh page: the part's memory,
    then up to 8 ids in slots, drawn top-left first. The slots themselves
    are picked at random so a short batch still spreads over the page.

    With `latest_id` (no generation: the image the HTML appends as the
    latest is already known), it's just one more to pick from: a random
    number of them (SCATTER_MIN to all) go in a grid at random sizes, one at
    random big in the middle, drawn top-left to bottom-right; append_latest()
    then only writes the closing lines.

    Only the memory text asks the viewer for the touch's one blank of the
    panel (clear_first, gated by CLEAR_BEFORE_TOUCH) — a touch can push
    many events, and blanking for each was what used to cause flicker.
    No settle_after anywhere: measured on-device, the final color stage
    settles cleanly from its own single UFAST partial update, and
    request_full_refresh() would flash the *whole* panel."""
    global _scatter_latest
    if not ENABLED:
        return
    ids = selected_ids[:8]
    _scatter_latest = latest_id
    if latest_id is not None:
        # Without generation there's no real "latest": all of them are just
        # picked from what's there. A random number, SCATTER_MIN up to all
        # nine, so pages don't all look equally full; the last one of the
        # (shuffled) pick goes big in the middle.
        ids.append(latest_id)
        ids = random.sample(ids, random.randint(min(SCATTER_MIN, len(ids)), len(ids)))
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
        if latest_id is not None:
            # The last of the shuffled pick goes in the big cell, the others
            # fill the rest.
            spots = _scattered(_layout, min(len(ids), len(_layout.scatter_cells)))
            rest = iter(ids[:-1])
            placed = [(ids[-1] if big else next(rest), center, size)
                      for center, size, big in spots]
        else:
            chosen = random.sample(_layout.slots, len(ids))
            _free_slots[:] = _reading_order(c for c in _layout.slots if c not in chosen)
            placed = [(img_id, slot, SLOT_PX) for img_id, slot in zip(ids, _reading_order(chosen))]
        await _push_text(paragraphs, _layout.memory_text_y, _layout.memory_slot,
                         clear_first=eink_memory_push.CLEAR_BEFORE_TOUCH, page_start=True)
        for img_id, center, size in placed:
            await _push(kind, img_id, center, size)

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
    if img_id != _scatter_latest:  # already drawn among the rest by show()
        _enqueue(lambda: _push(kind, img_id, _layout.latest_center, LATEST_PX))
    _enqueue(lambda: _push_text(lines, _layout.touch_line_y, _layout.touch_slot))


# The cover: shown once nobody has touched for IDLE_COVER_SEC, so a visitor
# walking up finds the work's title, what it is about, and what to do,
# instead of whoever's page was left there. Laid out like a picture book's
# cover — a hand-drawn double frame, the title written large, a line under
# it, the bear in the middle of the page (shaking a hand: touch is the whole
# idea) and a few lines of text, centered, a sentence to a line.
#
# The cover is only ever second to a touch. It's pushed by a task of its own,
# not through the page queue, so a touch's page never waits behind it;
# begin_page() bumps _page_gen, which stops the cover's next push; and every
# cover event is `idle`, which the viewer drops — queued, half drawn, or
# landing late from a push that was already under way — once a touch's page
# arrives.
IDLE_COVER_SEC = 20.0
_COVER_BEAR = os.path.join(os.path.dirname(__file__), "../data/cover/bear.png")
_page_gen = 0


class Cover:
    def __init__(self, screen, frame_margin, title_y, title_px, tagline_y,
                 bear_y, bear_size, body_y, body_px):
        self.screen = screen
        self.frame_margin = frame_margin
        self.title_y, self.title_px, self.tagline_y = title_y, title_px, tagline_y
        self.bear_y, self.bear_size = bear_y, bear_size
        self.body_y, self.body_px = body_y, body_px

    def bear_center(self):
        """Where the bear's picture goes so the bear itself — not the
        picture, which also holds the hand reaching in from the right —
        stands in the middle of the page. Returns (center, width)."""
        bw, bh = Image.open(_COVER_BEAR).size
        w = min(self.bear_size[0], self.bear_size[1] * bw / bh)
        return (round(self.screen[0] / 2 + (0.5 - _COVER_BEAR_X) * w), self.bear_y), round(w)


# Across data/cover/bear.png, the bear (head and body, not the arm it holds
# out) is centered this far in from the left; the rest is the hand.
_COVER_BEAR_X = 0.29

COVERS = {
    # Sized so that, with the bear in the middle, the picture's right edge
    # — where the hand's sleeve is cut off — lands on the frame: the hand
    # reaches in from outside the page.
    "portrait": Cover((1620, 2160), frame_margin=50, title_y=150, title_px=150,
                      tagline_y=400, bear_y=1000, bear_size=(1045, 1010),
                      body_y=1560, body_px=46),
    # Less height to spare: a smaller title and bear, the text under them.
    "landscape": Cover((2160, 1620), frame_margin=50, title_y=90, title_px=120,
                       tagline_y=290, bear_y=745, bear_size=(1000, 710),
                       body_y=1120, body_px=42),
}
COVER_TAGLINE_PX = 56
COVER_TITLE_PEN_R = 4
COVER_POINTS_PER_STEP = 18  # a little brisker than a page's text: there's more of it


def _frame_png(cover, path):
    """A hand-drawn-looking double frame just inside the panel edge,
    transparent inside (push_memory flattens it onto the page)."""
    w, h = cover.screen
    m = cover.frame_margin
    img = Image.new("RGBA", (w, h), (255, 255, 255, 0))
    d = ImageDraw.Draw(img)
    rnd = random.Random(7)  # the same wobble every time

    def wobbly_rect(inset, width):
        x0, y0, x1, y1 = m + inset, m + inset, w - m - inset, h - m - inset
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        pts = []
        for (ax, ay), (bx, by) in zip(corners, corners[1:]):
            n = max(2, int(max(abs(bx - ax), abs(by - ay)) / 60))
            for i in range(n):
                t = i / n
                pts.append((ax + (bx - ax) * t + rnd.uniform(-1.5, 1.5),
                            ay + (by - ay) * t + rnd.uniform(-1.5, 1.5)))
        pts.append(pts[0])
        d.line(pts, fill=(0, 0, 0, 255), width=width, joint="curve")

    wobbly_rect(0, 5)
    wobbly_rect(16, 2)
    img.save(path)


def start_idle_timer():
    """Called once at startup: the cover comes up if nobody touches within
    IDLE_COVER_SEC. After that, every page re-arms it once it's drawn."""
    if ENABLED:
        _arm_cover()


def _arm_cover():
    gen = _page_gen

    async def after_idle():
        await asyncio.sleep(IDLE_COVER_SEC)
        if gen == _page_gen and not _busy:
            await _draw_cover(gen)

    asyncio.get_event_loop().create_task(after_idle())


async def _draw_cover(gen):
    try:
        await _push_cover(gen)
    except Exception as e:
        rospy.logwarn(f"eink_hook: could not draw the cover: {e!r}")


async def _push_cover(gen):
    loop = asyncio.get_event_loop()

    async def push(fn, *args, **kwargs):
        """Run one push unless a touch has started a page since."""
        if gen != _page_gen:
            return False
        try:
            await loop.run_in_executor(None, lambda: fn(*args, **kwargs))
        except Exception as e:
            rospy.logwarn(f"eink_hook: cover push failed: {e}")
        return gen == _page_gen

    try:
        orientation = await loop.run_in_executor(None, eink_memory_push.read_orientation)
    except Exception as e:
        rospy.logwarn(f"eink_hook: could not read tablet orientation ({e}); assuming portrait")
        orientation = "portrait"
    c = COVERS[orientation]
    w, h = c.screen
    m = c.frame_margin
    inner_x, inner_w = m + 40, w - 2 * (m + 40)
    title, tagline, body = memory_text.cover_texts()
    bear_center, bear_w = c.bear_center()
    common = dict(layout=orientation, idle=True)
    rospy.loginfo(f"eink_hook: nobody touching for {IDLE_COVER_SEC:.0f}s, drawing the cover ({orientation})")

    with tempfile.TemporaryDirectory() as tmp:
        frame = os.path.join(tmp, "frame.png")
        _frame_png(c, frame)
        # The frame first: its picture is blank inside, so drawn later it
        # would wipe out everything already written.
        steps = [
            (eink_memory_push.push_memory, (frame, w // 2, h // 2, w - 2 * m + 8),
             dict(event_id=f"{_event_id()}_cover_frame", max_h=h - 2 * m + 8,
                  clear_first=True, page_start=True)),
            (eink_memory_push.push_text, ([title], inner_x, c.title_y, inner_w),
             dict(px=c.title_px, event_id=f"{_event_id()}_cover_title", align="center",
                  pen_r=COVER_TITLE_PEN_R)),
            (eink_memory_push.push_text, ([tagline], inner_x, c.tagline_y, inner_w),
             dict(px=COVER_TAGLINE_PX, event_id=f"{_event_id()}_cover_tagline", align="center",
                  points_per_step=COVER_POINTS_PER_STEP)),
            (eink_memory_push.push_memory, (_COVER_BEAR, *bear_center, bear_w),
             dict(event_id=f"{_event_id()}_cover_bear", max_h=c.bear_size[1])),
            (eink_memory_push.push_text, (body, inner_x, c.body_y, inner_w),
             dict(px=c.body_px, event_id=f"{_event_id()}_cover_body", align="center",
                  points_per_step=COVER_POINTS_PER_STEP)),
            (eink_memory_push.push_page_end, (f"{_event_id()}_cover_end",), dict(idle=True)),
        ]
        for fn, args, kwargs in steps:
            if fn is not eink_memory_push.push_page_end:
                kwargs = {**common, **kwargs}
            if not await push(fn, *args, **kwargs):
                rospy.loginfo("eink_hook: a touch came in, cover stopped")
                return
