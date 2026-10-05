"""The picture-book text written under each touch's illustrations on the
e-ink display: a pre-written "memory" for the touched part, and a line
recording this touch itself ("...your hand touched the head") — the
concept's "the moment you touched it is kept as a memory, too".

A part's memory is always three sentences: a fixed opening (it has been
touched there many times), one episode picked at random from several, and
a fixed closing that points at the trace it left. They live in
data/memory_texts.json, one entry per part (keys match
touch_image_camera_new.PARTS), every sentence a {"ja", "en"} pair so an
English version only needs LANG switched. Japanese is written with spaces between
words (分かち書き): the viewer wraps lines on spaces.
"""
import datetime
import json
import os
import random

LANG = "ja"  # "ja" | "en"

_TEXTS_PATH = os.path.join(os.path.dirname(__file__), "../data/memory_texts.json")

# Day-of-month readings that aren't just "<n>にち".
_JA_DAYS = {
    1: "ついたち", 2: "ふつか", 3: "みっか", 4: "よっか", 5: "いつか",
    6: "むいか", 7: "なのか", 8: "ようか", 9: "ここのか", 10: "とおか",
    14: "じゅうよっか", 20: "はつか", 24: "にじゅうよっか",
}

_last_pick = {}


def _load():
    with open(_TEXTS_PATH, encoding="utf-8") as f:
        return json.load(f)


def memory_paragraphs(part, lang=None):
    """[opening, episode, evidence] for `part` — the episode never the same
    one twice in a row. Re-reads the JSON every call, so edits apply
    without restarting the node. Returns [] if the part has no entry."""
    lang = lang or LANG
    entry = _load().get(part)
    if not entry:
        return []
    episodes = entry["episodes"]
    choices = [i for i in range(len(episodes)) if i != _last_pick.get(part)] or [0]
    i = random.choice(choices)
    _last_pick[part] = i
    return [entry["opening"][lang], episodes[i][lang], entry["evidence"][lang]]


def touch_line(part, now=None, lang=None):
    """The closing lines for this touch: "<date>, your hand touched the
    <part>." and whatever follows it in the JSON — a list of paragraphs."""
    lang = lang or LANG
    now = now or datetime.datetime.now()
    spec = _load()["_touch_line"]
    part_ja, part_en = spec["parts"][part]
    hour12 = now.hour % 12 or 12
    fields = dict(
        year=now.year,
        month=now.month,
        month_en=now.strftime("%B"),
        day=now.day,
        day_ja=_JA_DAYS.get(now.day, f"{now.day}にち"),
        hour=hour12,
        ampm_ja="ごぜん" if now.hour < 12 else "ごご",
        ampm_en="am" if now.hour < 12 else "pm",
        part_ja=part_ja,
        part_en=part_en,
    )
    return [line.format(**fields) for line in spec[lang]]


def cover_texts(lang=None):
    """The cover shown while nobody is touching: (title, tagline, [body
    paragraphs]) from the JSON's "_cover". Each Japanese sentence of the
    body gets a line of its own (a line break after every 。)."""
    lang = lang or LANG
    spec = _load()["_cover"]
    body = [p[lang].replace("。", "。\n").strip() for p in spec["body"]]
    return spec["title"][lang], spec["tagline"][lang], body


if __name__ == "__main__":
    for p in ["head", "stomach", "larm", "rarm", "lleg", "rleg"]:
        for lang in ("ja", "en"):
            print("\n".join(memory_paragraphs(p, lang)))
            print("\n".join(touch_line(p, lang=lang)))
