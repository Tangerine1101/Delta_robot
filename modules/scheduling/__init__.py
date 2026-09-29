"""Which part next, and how fast the belt runs — the research surface of the cell.

Plugins are single decorated functions in `rules.py`, `planners.py`, `speed_laws.py` and
`gates.py`; config selects them by name. The contracts, with worked examples, are in
`README.md`; `python3 -m modules.scheduling` lists what is registered.

Nothing here knows about threads, PLCs or the camera: the realtime loop (modules/runtime) and
the offline bench (sandbox/) call the same functions.
"""
