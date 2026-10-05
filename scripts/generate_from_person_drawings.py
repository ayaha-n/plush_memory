#!/usr/bin/env python3
"""Generate styled illustrations from existing person drawings, without the
camera: for each part, pick person_drawing_<id>.png at random and write
data/images/<style>/generated_drawing_<id>_<part>.png — the same id, the
same bear/person composite and the same prompt draw_on_touch.py uses.

    python3 generate_from_person_drawings.py                  # 3 per part, shepard
    python3 generate_from_person_drawings.py --per-part 5 --parts head stomach
    python3 generate_from_person_drawings.py --ids 516 1543 --parts larm

Needs OPENAI_API_KEY. Outputs that already exist are skipped.
"""
import argparse
import os
import random
import re
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

import illustration_and_combine_new as ic

PARTS = ["larm", "rarm", "lleg", "rleg", "head", "stomach"]


def person_drawing_ids():
    pat = re.compile(r"person_drawing_(\d+)\.png$")
    return sorted(int(m.group(1)) for f in os.listdir(ic.path_to_dir) if (m := pat.match(f)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--style", default="shepard", choices=sorted(ic.STYLES))
    ap.add_argument("--parts", nargs="+", default=PARTS, choices=PARTS)
    ap.add_argument("--per-part", type=int, default=3)
    ap.add_argument("--ids", nargs="+", type=int,
                    help="use these person_drawing ids (for every part) instead of sampling")
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    if not ic.api_key:
        sys.exit("OPENAI_API_KEY is not set")

    if args.ids:
        jobs = [(part, pid) for part in args.parts for pid in args.ids
                if not os.path.exists(ic.styled_output_path(pid, part, args.style))]
    else:
        # Distinct ids across all parts, and none that already has an image
        # for that part in this style.
        pool = person_drawing_ids()
        random.shuffle(pool)
        jobs = []
        for part in args.parts:
            n = 0
            while n < args.per_part and pool:
                pid = pool.pop()
                if not os.path.exists(ic.styled_output_path(pid, part, args.style)):
                    jobs.append((part, pid))
                    n += 1
    for part, pid in jobs:
        print(f"plan: {part} <- person_drawing_{pid}", flush=True)

    with tempfile.TemporaryDirectory() as tmp:
        def run(job):
            part, pid = job
            # Built fresh per part rather than reusing combined_drawing_<id>.png:
            # that one was made for whichever part was touched, and larm needs
            # the mirrored bear.
            combined = os.path.join(tmp, f"combined_{pid}_{part}.png")
            ic.combine_with_bear(os.path.join(ic.path_to_dir, f"person_drawing_{pid}.png"), part).save(combined)
            return part, pid, ic.generate_styled(combined, pid, part, args.style)

        failed = 0
        with ThreadPoolExecutor(args.workers) as ex:
            for part, pid, out in ex.map(run, jobs):
                print(f"{'done' if out else 'FAILED'}: {part} {pid}", flush=True)
                failed += out is None
    print(f"{len(jobs) - failed}/{len(jobs)} generated")


if __name__ == "__main__":
    main()
