# plush_memory — working rules

- **Never run `git push` (or `git push --force`) without an explicit, direct instruction to push, given in that same turn.** Do not infer permission to push from an ambiguous question, from a prior unrelated push approval, or from "that seems like the next step." Confirm before every single push.
- **Commit messages are a single line.** No body, no `Co-Authored-By` trailer.
- **Avoid full-panel flashes on the e-ink display.** Don't reach for `request_full_refresh()` (or any GC16-style whole-screen flash) to fix ghosting or clearing; look for a partial-update / UFAST-based approach first, and only propose a full flash as a last resort, with the user's OK.
