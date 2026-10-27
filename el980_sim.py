"""
el980_sim.py - a software model of an EL980 drawer cabinet speaking the AUX
2.4.0 protocol, so the examples run on a desk with nothing attached.

It models what the firmware does, at the level the host can observe:
  - 8 drawer positions (closed / partial / open / indeterminate), the
    single-drawer interlock, the out-lock that holds a pulled-out drawer
  - 7 rows x 6 lids per drawer, each with a latch switch
  - the Vend Any queue (3 deep, one drawer per batch), lid release one after
    another with the 3 s lid-switch timeout, one result per batch
  - the deferred Drawer Open reply (~0.4 s, 5 s latch timeout)
  - Init abandoning an opened drawer
Timing is real wall-clock, so the examples wait as they would on hardware.

The operator is simulated with Cabinet.pull_out(d), close_lids(d) and
push_home(d). Faults are injected with Cabinet.offline, stuck_lids and
latch_fail. Nothing here talks to a real EL980.
"""
from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Set, Tuple

from el980_aux import (HDR, checksum, DrawerPos, DRAWERS, ROWS_PER_DRAWER, COLS_PER_ROW,
                       QUEUE_DEPTH, Link, CMD_INIT, CMD_POLL, CMD_VERSION, CMD_VEND_ANY,
                       CMD_DOOR_STATUS, CMD_MACHINE_STATUS, CMD_DRAWER_OPEN, CMD_DRAWER_STATUS,
                       CMD_RESET_ALL, CMD_SET_TYPE)

BUILD = 1963
LATCH_RELEASE_S = 0.4       # relay delay + pulse + switch
LATCH_FAIL_S = 5.0          # DRAWER_UNLOCK_TIMEOUT_MS: firmware gives up waiting for the closed switch
LID_RELEASE_S = 0.2         # a healthy lid reads released this long after its pulse
LID_TIMEOUT_S = 3.0         # DRAWER_LID_OPEN_TIMEOUT_MS
LID_GAP_S = 0.25            # DRAWER_FIRE_GAP_MS between lids of a batch


class Cabinet:
    def __init__(self, fitted: int = 8):
        self.lock = threading.RLock()
        self.fitted: Set[int] = set(range(1, fitted + 1))
        self.offline: Set[int] = set()
        self.stuck_lids: Set[Tuple[int, int, int]] = set()   # (drawer,row,col) that never release
        self.latch_fail: Set[int] = set()                     # drawers whose lock never releases
        self.pos: Dict[int, DrawerPos] = {d: DrawerPos.CLOSED for d in range(1, DRAWERS + 1)}
        self.lid_closed: Dict[int, List[List[bool]]] = {
            d: [[True] * COLS_PER_ROW for _ in range(ROWS_PER_DRAWER)] for d in range(1, DRAWERS + 1)}
        self.held_out: Dict[int, bool] = {d: False for d in range(1, DRAWERS + 1)}  # out-lock engaged
        self.awaiting_lid: Optional[int] = None    # drawer opened by 0x0A, no lid released yet
        self.queue: List[Tuple[int, int, int]] = []
        # vend engine
        self.vend_in_progress = False
        self.vend_good = False
        self.vend_failed = False
        self.vend_error = False
        self.vend_drawer: Optional[int] = None
        self.vend_items: List[Tuple[int, int, int]] = []
        self.vend_phase = "idle"        # idle | unlock | wait_out | lids | tail
        self.vend_t = 0.0
        self.vend_fired: Optional[Tuple[int, int, int]] = None
        self.lids_ok = True
        self.open_pending: Optional[Tuple[int, float]] = None   # (drawer, t_started)
        self.events: List[str] = []

    # ---- operator actions ----------------------------------------------------
    def pull_out(self, d: int) -> None:
        """Operator pulls a released drawer fully out; the out-lock holds it."""
        with self.lock:
            if self.pos[d] == DrawerPos.PARTIAL:
                self.pos[d] = DrawerPos.OPEN
                self.held_out[d] = True
                self._ev("operator pulled drawer %d fully out (out-lock engaged)" % d)

    def close_lids(self, d: int) -> None:
        """Operator shuts every open lid of the drawer."""
        with self.lock:
            self.lid_closed[d] = [[True] * COLS_PER_ROW for _ in range(ROWS_PER_DRAWER)]
            self._ev("operator closed the lids of drawer %d" % d)
            self._tick()

    def push_home(self, d: int) -> None:
        """Operator pushes the drawer home (only possible once the out-lock is free)."""
        with self.lock:
            self._tick()
            if self.pos[d] == DrawerPos.OPEN and self.held_out[d]:
                self._ev("drawer %d is held by the out-lock: cannot push home yet" % d)
                return
            if self.pos[d] in (DrawerPos.OPEN, DrawerPos.PARTIAL):
                self.pos[d] = DrawerPos.CLOSED
                self._ev("operator pushed drawer %d home (closed switch made)" % d)
                self._tick()

    def _ev(self, s: str) -> None:
        self.events.append("%7.2f  %s" % (time.monotonic() - _T0, s))

    # ---- firmware model -------------------------------------------------------
    def _position(self, d: int) -> DrawerPos:
        if d not in self.fitted:
            return DrawerPos.INDETERMINATE
        if self.pos[d] == DrawerPos.CLOSED:
            return DrawerPos.CLOSED              # closed switch is read by the EL980 itself
        if d in self.offline:
            return DrawerPos.INDETERMINATE       # out-lock switch lives on the drawer board
        return self.pos[d]

    def _interlock_ok(self, d: int) -> bool:
        return all(self._position(x) == DrawerPos.CLOSED for x in self.fitted if x != d)

    def _available(self, d: int) -> bool:
        return d in self.fitted and d not in self.offline

    def _all_lids_closed(self, d: int) -> bool:
        return all(all(r) for r in self.lid_closed[d])

    def _tick(self) -> None:
        """Advance the vend engine and the out-lock release; called on every frame."""
        now = time.monotonic()
        # out-lock: free the drawer once a lid has been released on it and all lids are shut
        for d in self.fitted:
            if self.held_out[d] and self.awaiting_lid != d and self.vend_drawer != d \
                    and self._all_lids_closed(d) and d not in self.offline:
                self.held_out[d] = False
                self.pos[d] = DrawerPos.PARTIAL
                self._ev("firmware released the out-lock of drawer %d (lids shut)" % d)
        ph = self.vend_phase
        d = self.vend_drawer
        if ph == "unlock":
            if now - self.vend_t >= LATCH_RELEASE_S:
                if d in self.latch_fail:
                    self._finish(False, error=True)
                else:
                    self.pos[d] = DrawerPos.PARTIAL
                    self.vend_phase = "wait_out"
                    self._ev("vend: drawer %d lock released, waiting for the pull" % d)
        elif ph == "wait_out":
            if self.pos[d] == DrawerPos.OPEN:
                self._start_lids()
        elif ph == "lids":
            if d in self.offline:
                self._finish(False)
                return
            f = self.vend_fired
            if f is not None:
                dd, r, c = f
                if f not in self.stuck_lids and now - self.vend_t >= LID_RELEASE_S \
                        and self.lid_closed[dd][r - 1][c - 1]:
                    self.lid_closed[dd][r - 1][c - 1] = False
                    self._ev("lid %d-%d-%d released (switch made)" % (dd, r, c))
                if not self.lid_closed[dd][r - 1][c - 1]:
                    self.vend_fired = None
                    self.vend_t = now
                elif now - self.vend_t >= LID_TIMEOUT_S:
                    self._ev("lid %d-%d-%d did NOT release within %.0f s" % (dd, r, c, LID_TIMEOUT_S))
                    self.lids_ok = False
                    self.vend_fired = None
                    self.vend_t = now
            if self.vend_fired is None:
                if self.vend_items and now - self.vend_t >= LID_GAP_S:
                    self.vend_fired = self.vend_items.pop(0)
                    self.vend_t = now
                    self._ev("lid %d-%d-%d fired" % self.vend_fired)
                elif not self.vend_items:
                    self._finish(self.lids_ok)
        # deferred 0x0A
        op = self.open_pending
        if op is not None:
            od, t0 = op
            if now - t0 >= LATCH_RELEASE_S:
                self.open_pending = None
                self.pos[od] = DrawerPos.PARTIAL
                self.awaiting_lid = od
                self._ev("drawer %d lock released (Drawer Open)" % od)

    def _start_lids(self) -> None:
        self.vend_phase = "lids"
        self.lids_ok = True
        self.vend_fired = None
        self.vend_t = time.monotonic() - LID_GAP_S
        self.awaiting_lid = None
        self._ev("vend: drawer %d is out, releasing %d lid(s)" % (self.vend_drawer, len(self.vend_items)))

    def _finish(self, ok: bool, error: bool = False) -> None:
        self.vend_in_progress = False
        self.vend_good = ok
        self.vend_failed = not ok
        self.vend_error = error
        self.vend_phase = "idle"
        self._ev("vend result: %s" % ("SUCCESS" if ok else "DID NOT SUCCEED"))
        self.vend_drawer = None
        self.vend_items = []

    def _busy(self) -> bool:
        return self.vend_phase != "idle"

    def _out_of_service(self) -> bool:
        if self._busy():
            return False
        return any(self._position(d) != DrawerPos.CLOSED for d in self.fitted)

    def _status_byte(self) -> int:
        b = 0x30     # drawer cabinet
        if self.vend_in_progress: b |= 0x80
        if self.vend_good:        b |= 0x40
        if self.vend_error:       b |= 0x02
        if self.vend_failed:      b |= 0x01
        return b

    # ---- frame handling -------------------------------------------------------
    def handle(self, cmd: int, d1: int, d2: int) -> Tuple[int, int, int, float]:
        """Returns (resp, d1, d2, delay_seconds)."""
        with self.lock:
            self._tick()
            m = d1 & 0x03
            if cmd in (CMD_INIT, CMD_RESET_ALL):
                self.queue = []
                self.vend_good = self.vend_failed = self.vend_error = False
                if self.awaiting_lid is not None:
                    self._ev("Init: abandoning opened drawer %d" % self.awaiting_lid)
                    self.awaiting_lid = None
                self.open_pending = None
                if not self._busy():
                    self.vend_in_progress = False
                self._tick()
                return cmd | 0x80, self._status_byte(), 0, 0.0
            if cmd == CMD_POLL:
                return 0x82, self._status_byte(), m, 0.0
            if cmd == CMD_VERSION:
                return 0x85, BUILD >> 8, BUILD & 0xFF, 0.0
            if cmd in (CMD_MACHINE_STATUS, CMD_SET_TYPE):
                b = 0x60 | 0x10 | 0x02 | 0x01
                if self._out_of_service():
                    b |= 0x04
                return cmd | 0x80, m, b, 0.0
            if cmd == CMD_DRAWER_STATUS:
                if m != 0:
                    return 0xFF, cmd, 0, 0.0
                word = 0
                for d in range(1, DRAWERS + 1):
                    word |= int(self._position(d)) << (2 * (d - 1))
                return 0x8B, word & 0xFF, word >> 8, 0.0
            if cmd == CMD_DOOR_STATUS:
                d = ((d1 >> 4) & 0x07) + 1
                row = ((d2 >> 4) & 0x0F) + 1
                if m != 0 or row > ROWS_PER_DRAWER or not self._available(d):
                    return 0x88, d1 & 0x73, 0x80, 0.0
                byte = sum(1 << c for c in range(COLS_PER_ROW) if self.lid_closed[d][row - 1][c])
                return 0x88, d1 & 0x73, byte, 0.0
            if cmd == CMD_DRAWER_OPEN:
                if m != 0:
                    return 0xFF, cmd, 0, 0.0
                return self._drawer_open(d1)
            if cmd == CMD_VEND_ANY:
                return self._vend_any(d1, d2)
            return 0xFF, cmd, 0, 0.0

    def _drawer_open(self, d1: int) -> Tuple[int, int, int, float]:
        d = ((d1 >> 4) & 0x07) + 1
        pos = int(self._position(d))
        echo = d1 & 0x73
        if d not in self.fitted:
            return 0x8A, echo, pos | 0x08, 0.0
        if d in self.offline:
            return 0x8A, echo, pos | 0x08 | 0x04, 0.0
        if pos in (DrawerPos.PARTIAL, DrawerPos.OPEN):
            return 0x8A, echo, pos, 0.0                       # harmless no-op
        if self.vend_in_progress or self.open_pending is not None:
            return 0x8A, echo, pos | 0x08 | 0x80, 0.0
        if not self._interlock_ok(d) or self._busy():
            return 0x8A, echo, pos | 0x08 | 0x40, 0.0
        if pos != DrawerPos.CLOSED:
            return 0x8A, echo, pos | 0x08, 0.0
        self._ev("Drawer Open %d: lock pulse" % d)
        if d in self.latch_fail:
            # the pulse goes out, the closed switch never opens: refusal after the 5 s wait
            self._ev("drawer %d latch did not release" % d)
            return 0x8A, echo, int(DrawerPos.CLOSED) | 0x08, LATCH_FAIL_S
        self.open_pending = (d, time.monotonic())
        return 0x8A, echo, int(DrawerPos.PARTIAL), LATCH_RELEASE_S

    def _vend_any(self, d1: int, d2: int) -> Tuple[int, int, int, float]:
        queue = not (d1 & 0x04)
        vend = not (d1 & 0x08)
        d = ((d1 >> 4) & 0x07) + 1
        row = ((d2 >> 4) & 0x0F) + 1
        col = (d2 & 0x0F) + 1
        base = 0x30

        def reject(code: int) -> Tuple[int, int, int, float]:
            b = base | 0x08
            if code == 2: b |= 0x04
            if code == 3: b |= 0x80
            return 0x87, b, 0, 0.0

        if self.vend_in_progress or self._busy():
            return reject(3)
        if (d1 & 0x03) != 0:
            return reject(1)

        def valid(dd, r, c) -> int:
            # firmware: the selection exists for every drawer 1..8, so a drawer that is
            # not fitted is reported like an offline one (bit 3 + bit 2); only a row or
            # column outside the layout is 'invalid selection' (bit 3 alone)
            if not (1 <= r <= ROWS_PER_DRAWER and 1 <= c <= COLS_PER_ROW):
                return 1
            return 0 if self._available(dd) else 2

        if queue:
            if len(self.queue) >= QUEUE_DEPTH:
                return reject(1)
            v = valid(d, row, col)
            if v:
                return reject(v)
            if self.queue and self.queue[0][0] != d:
                return reject(1)                               # batch would span two drawers
            self.queue.append((d, row, col))
            self._ev("staged compartment %d-%d-%d (queue %d)" % (d, row, col, len(self.queue)))
        if vend:
            if not self.queue:
                v = valid(d, row, col)
                if v:
                    return reject(v)
                items = [(d, row, col)]
            else:
                items = list(self.queue)
            vd = items[0][0]
            pos = self._position(vd)
            if pos == DrawerPos.OPEN:
                self.queue = []
                self._begin(vd, items, open_already=True)
            elif pos == DrawerPos.CLOSED and not self._busy() and self._interlock_ok(vd):
                self.queue = []
                self._begin(vd, items, open_already=False)
            else:
                self.queue = []                                # a refused commit clears the queue
                return reject(1)
            return 0x87, self._status_byte(), 0, 0.0
        return 0x87, base, 0, 0.0

    def _begin(self, d: int, items, open_already: bool) -> None:
        self.vend_in_progress = True
        self.vend_good = self.vend_failed = self.vend_error = False
        self.vend_drawer = d
        self.vend_items = items
        self.vend_t = time.monotonic()
        if open_already:
            self._start_lids()
        else:
            self.vend_phase = "unlock"
            self._ev("vend: drawer %d is closed, releasing its lock first" % d)


_T0 = time.monotonic()


class SimLink(Link):
    """Link adapter: frames go to Cabinet.handle(), replies come back with the
    modelled delay."""

    def __init__(self, cab: Cabinet):
        self.cab = cab
        self.pending: Optional[Tuple[float, bytes]] = None   # (ready_at, frame)

    def write(self, data: bytes) -> None:
        if len(data) != 5 or data[0] != HDR or checksum(data[1], data[2], data[3]) != data[4]:
            return
        resp, d1, d2, delay = self.cab.handle(data[1], data[2], data[3])
        fr = bytes([HDR, resp, d1 & 0xFF, d2 & 0xFF, checksum(resp, d1, d2)])
        self.pending = (time.monotonic() + delay + 0.005, fr)

    def read(self, n: int, timeout: float) -> bytes:
        if self.pending is None:
            time.sleep(min(timeout, 0.01))
            return b""
        ready, fr = self.pending
        wait = ready - time.monotonic()
        if wait > timeout:
            time.sleep(timeout)
            return b""
        if wait > 0:
            time.sleep(wait)
        out, self.pending = fr[:n], (fr[n:] and (ready, fr[n:])) or None
        return out

    def flush_input(self) -> None:
        self.pending = None
