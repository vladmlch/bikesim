"""Deferred-SIGINT shield for evidence-critical publication sections (R04).

Seal+finalize is a critical section: a second Ctrl-C delivered mid-write
can strand journaled input events against an unsealed episode or corrupt
the staged publication — the launch then loses its entire owned
recording. On the main thread, ``InterruptShield`` swaps the SIGINT
handler for a flag-setting one for the scope of its ``with`` block, so an
interrupt is recorded in ``deferred`` and honored after the section
completes instead of interrupting the write. ``signal.signal`` cannot
install handlers off the main thread; there the shield degrades to a
flag holder, and ``note()`` is the injected-hook seam a caller (or a
test without access to real signal delivery) uses to record a deferred
interrupt.
"""
from __future__ import annotations

import signal
import threading


class InterruptShield:
    """One shielded section's deferred-interrupt record.

    ``deferred`` counts interrupts received while the section was
    shielded. The previous handler is always restored on exit; the
    caller decides how a nonzero count is honored (re-raise
    ``KeyboardInterrupt`` where the run continues, journal/note it where
    an exception is already propagating).
    """

    def __init__(self):
        self.deferred = 0
        self._previous = None

    def note(self, signum=None, frame=None):
        """Record one deferred operator interrupt."""
        self.deferred += 1

    def __enter__(self):
        if threading.current_thread() is threading.main_thread():
            try:
                self._previous = signal.signal(signal.SIGINT, self.note)
            except (ValueError, OSError, RuntimeError):
                self._previous = None
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._previous is not None:
            signal.signal(signal.SIGINT, self._previous)
            self._previous = None
        return False
