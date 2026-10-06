"""Geometry and time models of the cell: pure functions and frozen data.

Nothing here sleeps, locks, prints or does I/O, and nothing here imports the runtime, the
scheduling plugins or a PLC link — which is what lets the sandbox and the tests use it as is.
"""
