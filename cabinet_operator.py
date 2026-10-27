"""
cabinet_operator.py - the human in the loop, for the examples.

On a real cabinet the operator pulls the drawer, shuts the lids and pushes
the drawer home; the examples print a prompt and poll Drawer Status until
the switches show it done (there is no firmware timeout for any of these).
On the software cabinet the same steps happen automatically after a short
delay, so the examples run unattended.
"""
import threading
import time

from el980_aux import DrawerPos


class Operator:
    def __init__(self, sim=None, delay=1.0):
        self.sim = sim
        self.delay = delay

    def _later(self, fn, *a):
        if self.sim is not None:
            threading.Timer(self.delay, fn, a).start()

    def prompt(self, text):
        print("  OPERATOR: " + text)

    def pull_out(self, m, drawer):
        """Drawer released: wait until the operator has it fully out (OPEN)."""
        self.prompt("pull drawer %d fully out until it locks" % drawer)
        self._later(self.sim.pull_out, drawer) if self.sim else None
        m.wait_drawer(drawer, DrawerPos.OPEN, timeout=None, on_tick=_tick)
        print("  drawer %d reads OPEN (held by the out-lock)" % drawer)

    def close_lids(self, m, drawer):
        """Items taken: wait until the firmware frees the out-lock (PARTIAL)."""
        self.prompt("take the items and shut every open lid of drawer %d" % drawer)
        self._later(self.sim.close_lids, drawer) if self.sim else None
        m.wait_drawer(drawer, DrawerPos.PARTIAL, timeout=None, on_tick=_tick)
        print("  all lids shut: the firmware released the out-lock (drawer %d reads PARTIAL)" % drawer)

    def push_home(self, m, drawer):
        self.prompt("push drawer %d home" % drawer)
        self._later(self.sim.push_home, drawer) if self.sim else None
        m.wait_drawer(drawer, DrawerPos.CLOSED, timeout=None, on_tick=_tick)
        print("  drawer %d reads CLOSED: transaction complete" % drawer)


_last = {"pos": None, "t": 0.0}


def _tick(pos):
    now = time.monotonic()
    if pos != _last["pos"] or now - _last["t"] > 5.0:
        print("    ... drawer position %s" % pos.name)
        _last["pos"], _last["t"] = pos, now
