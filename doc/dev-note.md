# Dev Notes — Delta Robot

> **Maintained by the human developer.** AI assistants may read this for context but must
> **not** edit it unless explicitly asked to.
>
> This is a personal scratchpad: hardware quirks, hunches, things to remember at the bench.
> It is deliberately *not* the project's issue tracker any more —
> **open problems live in [`open-issues.md`](open-issues.md)** (which AI agents may update),
> and superseded designs live in [`decision-log.md`](decision-log.md).

---

## Bench notes

* **Rotation axis.** The `[ROTATE]` log line prints the whole chain per pick
  (`vision_angle / board_heading / rotate_cmd / rotate_at_gate / rotate_at_end`). Read it
  after any change to the cup marker or the mounting — the offsets do not survive a
  re-mount. Probe with `python3 -m modules.test_rotate`; `rotate_sweep_sim` covers the maths
  without touching hardware.
* **Gate latency.** `dispatch_to_contact_s` in the `[GATE]` log is the number to watch; it is
  what `robot_movement_delay_s` is supposed to model. `latency_probe --target siemens`
  isolates the wire portion of it.
* **Belt speed control is coarse.** The Siemens speed command is imprecise at the low end and
  the encoder quantisation adds noise — that is what `conveyor.velocity_ema_alpha` smooths.
  `conveyor_position_scale_mm = 1.0` because the PLC already reports millimetres.
* **Camera.** Auto-exposure must stay off or FPS collapses and motion blur breaks tracking;
  disable auto-exposure *before* writing the manual exposure value. Frames are captured with
  PyAV, not OpenCV's V4L2 backend — that backend was the real 30 FPS bottleneck.
* **Local archive.** `../Delta_robot_git_backup_2026-07-11.git` is the full pre-purge mirror
  of the repository history. Keep it; never push it.

## Where things go

| I want to record… | Put it in |
|---|---|
| a problem that is still unresolved | `open-issues.md` |
| a design that was tried and dropped, and why | `decision-log.md` |
| how something currently works | `basis-theory.md` / `basis-programming.md` |
| a bench hunch, a quirk, a reminder to myself | here |
