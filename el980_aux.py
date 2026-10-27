"""
el980_aux.py - host-side library for the Seaga EL980 AUX serial interface
(spec "Seaga EL980 AUX interface 2.4.0", drawer-cabinet commands included).

Pure Python 3, one optional dependency: pyserial (pip install pyserial) for
the real serial link. The examples can also run against el980_sim.py, a
software model of a drawer cabinet, with no hardware attached.

Wire format (every command and every response):

    A5  cmd  d1  d2  ck        ck = (cmd + d1 + d2) & 0xFF

The EL980 answers every command within 20 ms, with one exception: Drawer Open
(0x0A) is answered when the drawer latch has been seen to release, about 0.4 s
after the command and about 5 s when the latch fails. All drawer, row and
column numbers in this API are 1-based (drawer 1..8, row 1..7, column 1..6)
and converted to the 0-based wire values here.
"""
from __future__ import annotations

import enum
import logging
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence

log = logging.getLogger("el980")

HDR = 0xA5

# ---- commands --------------------------------------------------------------
CMD_INIT            = 0x01
CMD_POLL            = 0x02
CMD_VERSION         = 0x05
CMD_VEND_ANY        = 0x07
CMD_DOOR_STATUS     = 0x08
CMD_MACHINE_STATUS  = 0x09
CMD_DRAWER_OPEN     = 0x0A
CMD_DRAWER_STATUS   = 0x0B
CMD_RESET_ALL       = 0x15
CMD_SET_TYPE        = 0x16
RESP_UNKNOWN        = 0xFF

DRAWERS = 8
ROWS_PER_DRAWER = 7     # v2.4.0 text says 6; VMC builds 1945 and later accept rows 1..7
COLS_PER_ROW = 6
QUEUE_DEPTH = 3         # Vend Any item queue (staged compartments per batch)

MACHINE_TYPE_COIL   = 0
MACHINE_TYPE_LOCKER = 1
MACHINE_TYPE_DRAWER = 3


# ---- framing ---------------------------------------------------------------
def checksum(cmd: int, d1: int, d2: int) -> int:
    return (cmd + d1 + d2) & 0xFF


def frame(cmd: int, d1: int = 0, d2: int = 0) -> bytes:
    return bytes([HDR, cmd & 0xFF, d1 & 0xFF, d2 & 0xFF, checksum(cmd, d1, d2)])


def hexs(b: bytes) -> str:
    return " ".join("%02X" % x for x in b)


class AuxError(Exception):
    """Base class for protocol errors."""


class AuxTimeout(AuxError):
    """No complete, valid response within the allowed time."""


class AuxUnknownCommand(AuxError):
    """The EL980 answered 0xFF: command not recognised in this mode/type."""


class AuxRejected(AuxError):
    """A Vend Any or Drawer Open request was refused; .result holds the decode."""

    def __init__(self, result):
        super().__init__(str(result))
        self.result = result


@dataclass
class Reply:
    cmd: int
    resp: int
    d1: int
    d2: int
    tx: bytes
    rx: bytes
    elapsed: float


# ---- decoders --------------------------------------------------------------
class DrawerPos(enum.IntEnum):
    """Two-bit drawer position from Drawer Status (0x8B) / Drawer Open (0x8A)."""
    INDETERMINATE = 0   # not fitted, board not answering while not closed, or switches disagree
    CLOSED = 1          # fully home and latched (positive reading from the closed switch)
    PARTIAL = 2         # released but not fully out, in motion, or pushed back part way
    OPEN = 3            # fully out and held by the out-lock


class OpType(enum.IntEnum):
    COIL = 0
    LOCKER = 2
    DRAWER = 3


@dataclass
class VendStatus:
    """The 'Poll-shaped' status byte returned by 0x81, 0x82, 0x87 and 0x95."""
    raw: int

    @property
    def in_progress(self) -> bool:      # bit 7
        return bool(self.raw & 0x80)

    @property
    def success(self) -> bool:          # bit 6 - valid only when not in_progress
        return bool(self.raw & 0x40)

    @property
    def op_type(self) -> OpType:        # bits 5-4
        v = (self.raw >> 4) & 0x03
        return OpType(v) if v in (0, 2, 3) else OpType.COIL

    @property
    def rejected(self) -> bool:         # bit 3 - only in the immediate Vend Any reply
        return bool(self.raw & 0x08)

    @property
    def no_motor(self) -> bool:         # bit 2 - with bit 3: target drawer offline
        return bool(self.raw & 0x04)

    @property
    def vend_error(self) -> bool:       # bit 1
        return bool(self.raw & 0x02)

    @property
    def failed(self) -> bool:           # bit 0 "Vend Did Not Succeed"
        return bool(self.raw & 0x01)

    def reject_reason(self) -> Optional[str]:
        """Why a Vend Any was refused (None when it was accepted)."""
        if not self.rejected:
            return None
        if self.no_motor:
            return "target drawer offline (not responding on the drawer bus)"
        if self.in_progress:
            return "machine busy: a vend is already in progress"
        return ("selection out of range, queue full, service mode, batch would span "
                "two drawers, or commit for a drawer that is not open")

    def __str__(self) -> str:
        bits = []
        if self.in_progress: bits.append("IN_PROGRESS")
        if self.success:     bits.append("SUCCESS")
        if self.rejected:    bits.append("REJECTED")
        if self.no_motor:    bits.append("NO_MOTOR/OFFLINE")
        if self.vend_error:  bits.append("VEND_ERROR")
        if self.failed:      bits.append("DID_NOT_SUCCEED")
        return "0x%02X %s [%s]" % (self.raw, self.op_type.name, " ".join(bits) or "idle")


@dataclass
class MachineStatus:
    """Response 0x89 / 0x96."""
    machine: int
    raw: int

    @property
    def is_drawer_cabinet(self) -> bool:
        return (self.raw & 0x60) == 0x60

    @property
    def is_locker(self) -> bool:
        return (self.raw & 0x60) == 0x20

    @property
    def enabled(self) -> bool:          # bit 4: reset complete
        return bool(self.raw & 0x10)

    @property
    def no_vendable_motors(self) -> bool:
        return bool(self.raw & 0x08)

    @property
    def out_of_service(self) -> bool:   # bit 2: on a drawer cabinet, a fitted drawer is not home
        return bool(self.raw & 0x04)

    @property
    def present(self) -> bool:
        return bool(self.raw & 0x02)

    @property
    def ever_present(self) -> bool:
        return bool(self.raw & 0x01)

    def __str__(self) -> str:
        t = "drawer cabinet" if self.is_drawer_cabinet else ("locker" if self.is_locker else "coil")
        return ("machine %d: %s, %s, %s, present=%s" % (
            self.machine, t,
            "enabled" if self.enabled else "not enabled",
            "OUT OF SERVICE" if self.out_of_service else "in service",
            self.present))


@dataclass
class DrawerOpenResult:
    """Response 0x8A."""
    drawer: int
    raw: int

    @property
    def position(self) -> DrawerPos:
        return DrawerPos(self.raw & 0x03)

    @property
    def accepted(self) -> bool:
        return not (self.raw & 0x08)

    @property
    def reason(self) -> Optional[str]:
        if self.accepted:
            return None
        if self.raw & 0x80:
            return "machine busy: a lid vend or another Drawer Open is in progress"
        if self.raw & 0x40:
            return "interlock blocked: another drawer is not closed"
        if self.raw & 0x04:
            return "target drawer offline (not responding on the drawer bus)"
        return "drawer out of range / not fitted, or the latch did not release within 5 s"

    def __str__(self) -> str:
        if self.accepted:
            return "drawer %d released, position now %s" % (self.drawer, self.position.name)
        return "drawer %d REFUSED (%s), position %s" % (self.drawer, self.reason, self.position.name)


@dataclass
class DoorRow:
    """Response 0x88 for one (drawer, row) on a drawer cabinet."""
    drawer: int
    row: int
    raw: int

    @property
    def offline(self) -> bool:          # bit 7: drawer offline or row invalid
        return bool(self.raw & 0x80)

    @property
    def closed(self) -> List[bool]:     # index 0 = column 1 ... index 5 = column 6
        return [bool(self.raw & (1 << c)) for c in range(COLS_PER_ROW)]

    def is_closed(self, col: int) -> bool:
        return bool(self.raw & (1 << (col - 1)))

    def __str__(self) -> str:
        if self.offline:
            return "drawer %d row %d: OFFLINE / row invalid" % (self.drawer, self.row)
        pic = "".join("C" if c else "o" for c in self.closed)
        return "drawer %d row %d: 0x%02X  cols 1..6 = %s" % (self.drawer, self.row, self.raw, pic)


# ---- transport -------------------------------------------------------------
class Link:
    """Byte transport. Subclass: write(bytes), read(n, timeout) -> bytes, flush_input()."""

    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def read(self, n: int, timeout: float) -> bytes:
        raise NotImplementedError

    def flush_input(self) -> None:
        pass

    def close(self) -> None:
        pass


class SerialLink(Link):
    """pyserial transport: 9600 N81, no flow control."""

    def __init__(self, port: str):
        import serial  # pyserial
        self.ser = serial.Serial(port, 9600, bytesize=8, parity="N", stopbits=1, timeout=0.05)

    def write(self, data: bytes) -> None:
        self.ser.write(data)
        self.ser.flush()

    def read(self, n: int, timeout: float) -> bytes:
        buf = b""
        t_end = time.monotonic() + timeout
        while len(buf) < n and time.monotonic() < t_end:
            buf += self.ser.read(n - len(buf))
        return buf

    def flush_input(self) -> None:
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()


# ---- the client ------------------------------------------------------------
class EL980:
    """One EL980 on one link. Every public method sends one command and
    returns a decoded response; nothing here blocks longer than the
    documented response time unless a wait_* helper is called."""

    REPLY_TIMEOUT = 0.25            # spec: 20 ms; allow for USB adapters
    OPEN_TIMEOUT = 6.0              # 0x0A is answered when the latch releases (~0.4 s) or has failed to (5 s)

    def __init__(self, link: Link, machine: int = 0):
        self.link = link
        self.machine = machine & 0x03
        self.trace: Callable[[str], None] = lambda s: log.debug(s)

    def close(self) -> None:
        self.link.close()

    # -- raw transaction
    def xfer(self, cmd: int, d1: int = 0, d2: int = 0, timeout: Optional[float] = None,
             retries: int = 1) -> Reply:
        """Send one frame, return the validated reply. A lost reply is retried
        `retries` times for idempotent commands; the caller passes retries=0
        for Vend Any (a repeated stage request would queue the item twice)."""
        tx = frame(cmd, d1, d2)
        timeout = timeout if timeout is not None else self.REPLY_TIMEOUT
        last_err = "no response"
        for attempt in range(retries + 1):
            self.link.flush_input()
            t0 = time.monotonic()
            self.link.write(tx)
            rx = self._read_frame(timeout)
            dt = time.monotonic() - t0
            if rx is None:
                last_err = "timeout after %.2f s" % timeout
                self.trace("TX %s  RX -- (%s)" % (hexs(tx), last_err))
                continue
            self.trace("TX %s  RX %s  (%.3f s)" % (hexs(tx), hexs(rx), dt))
            resp, rd1, rd2 = rx[1], rx[2], rx[3]
            if resp == RESP_UNKNOWN:
                raise AuxUnknownCommand("command 0x%02X not handled (reply FF %02X)" % (cmd, rd1))
            if resp != (cmd | 0x80):
                last_err = "unexpected response 0x%02X to command 0x%02X" % (resp, cmd)
                continue
            return Reply(cmd, resp, rd1, rd2, tx, rx, dt)
        raise AuxTimeout("cmd 0x%02X: %s" % (cmd, last_err))

    def _read_frame(self, timeout: float) -> Optional[bytes]:
        """Read one 5-byte frame, resynchronising on the 0xA5 header and
        rejecting a bad checksum."""
        t_end = time.monotonic() + timeout
        buf = b""
        while True:
            remaining = t_end - time.monotonic()
            if remaining <= 0:
                return None
            chunk = self.link.read(5 - len(buf), remaining)
            if not chunk:
                return None
            buf += chunk
            # drop anything before the header
            i = buf.find(bytes([HDR]))
            buf = buf[i:] if i >= 0 else b""
            if len(buf) >= 5:
                fr = buf[:5]
                if checksum(fr[1], fr[2], fr[3]) == fr[4]:
                    return fr
                self.trace("bad checksum in %s, resyncing" % hexs(fr))
                buf = buf[1:]

    # -- commands
    def init(self) -> VendStatus:
        """0x01: reset and initialise, clear errors and the vend queue. On a
        drawer cabinet it also abandons an opened drawer (the drawer is not
        moved or re-latched)."""
        r = self.xfer(CMD_INIT)
        return VendStatus(r.d1)

    def reset_all(self, machine_type: Optional[int] = None) -> VendStatus:
        """0x15: as Init, plus restart the AUX item count. machine_type
        0 coil / 1 locker / 3 drawer; a CHANGE of type restarts the board."""
        d1 = 0 if machine_type is None else (machine_type & 0x03)
        r = self.xfer(CMD_RESET_ALL, d1)
        return VendStatus(r.d1)

    def poll(self) -> VendStatus:
        """0x02: vend progress / result."""
        r = self.xfer(CMD_POLL, self.machine)
        return VendStatus(r.d1)

    def version(self) -> int:
        """0x05: firmware build number (0xFFFF = bootloader with no valid app)."""
        r = self.xfer(CMD_VERSION, self.machine)
        return (r.d1 << 8) | r.d2

    def machine_status(self) -> MachineStatus:
        """0x09."""
        r = self.xfer(CMD_MACHINE_STATUS, self.machine)
        return MachineStatus(r.d1 & 0x0F, r.d2)

    def set_machine_type(self, machine_type: int) -> MachineStatus:
        """0x16: declare the type. Re-declaring the current type is a no-op.
        A change restarts the board ~2 s after the reply; allow 20 s."""
        d1 = ((machine_type & 0x03) << 4) | self.machine
        r = self.xfer(CMD_SET_TYPE, d1)
        return MachineStatus(r.d1 & 0x0F, r.d2)

    def drawer_status(self) -> List[DrawerPos]:
        """0x0B: position of all 8 drawers; index 0 = drawer 1."""
        r = self.xfer(CMD_DRAWER_STATUS, self.machine)
        word = r.d1 | (r.d2 << 8)
        return [DrawerPos((word >> (2 * i)) & 0x03) for i in range(DRAWERS)]

    def drawer_open(self, drawer: int) -> DrawerOpenResult:
        """0x0A: release the lock of one drawer. Returns when the latch has
        released (0.4 s) or failed to (5 s). Raises AuxRejected when
        refused."""
        _check_drawer(drawer)
        d1 = ((drawer - 1) << 4) | self.machine
        r = self.xfer(CMD_DRAWER_OPEN, d1, timeout=self.OPEN_TIMEOUT)
        res = DrawerOpenResult(drawer, r.d2)
        if not res.accepted:
            raise AuxRejected(res)
        return res

    def door_status(self, drawer: int, row: int) -> DoorRow:
        """0x08: lid status of one row (6 columns) of one drawer. 1 = closed."""
        _check_drawer(drawer)
        if not 1 <= row <= 8:
            raise ValueError("row 1..%d (8 = out-lock switch row)" % ROWS_PER_DRAWER)
        d1 = ((drawer - 1) << 4) | self.machine
        r = self.xfer(CMD_DOOR_STATUS, d1, (row - 1) << 4)
        return DoorRow(drawer, row, r.d2)

    def vend_any(self, drawer: int = 1, row: int = 1, col: int = 1,
                 queue: bool = True, vend: bool = True) -> VendStatus:
        """0x07 raw: queue the compartment (queue=True) and/or vend what is
        queued (vend=True). Raises AuxRejected with the decoded status when
        bit 3 comes back set. Not retried: a repeat could queue twice."""
        _check_drawer(drawer)
        _check_compartment(row, col)
        d1 = ((drawer - 1) << 4) | (0 if vend else 0x08) | (0 if queue else 0x04) | self.machine
        d2 = ((row - 1) << 4) | (col - 1)
        r = self.xfer(CMD_VEND_ANY, d1, d2, retries=0)
        st = VendStatus(r.d1)
        if st.rejected:
            raise AuxRejected(st)
        return st

    def vend_stage(self, drawer: int, row: int, col: int) -> VendStatus:
        """Queue one compartment without vending (Vend 1, Queue 0)."""
        return self.vend_any(drawer, row, col, queue=True, vend=False)

    def vend_commit(self) -> VendStatus:
        """Release the queued lids (Vend 0, Queue 1). Drawer/row/col are
        ignored by the firmware for a commit."""
        return self.vend_any(1, 1, 1, queue=False, vend=True)

    def vend_now(self, drawer: int, row: int, col: int) -> VendStatus:
        """Queue one compartment and vend it in the same request (Vend 0, Queue 0)."""
        return self.vend_any(drawer, row, col, queue=True, vend=True)

    # -- waits
    def wait_vend_done(self, timeout: float = 30.0, period: float = 0.25) -> VendStatus:
        """Poll until Vend in progress clears; returns the final status."""
        t_end = time.monotonic() + timeout
        while True:
            st = self.poll()
            if not st.in_progress:
                return st
            if time.monotonic() >= t_end:
                raise AuxTimeout("vend still in progress after %.0f s" % timeout)
            time.sleep(period)

    def wait_drawer(self, drawer: int, want: DrawerPos, timeout: Optional[float],
                    period: float = 0.2, on_tick: Optional[Callable[[DrawerPos], None]] = None
                    ) -> DrawerPos:
        """Poll Drawer Status until `drawer` reads `want`. timeout=None waits
        for ever (operator-paced steps have no firmware timeout)."""
        _check_drawer(drawer)
        t_end = None if timeout is None else time.monotonic() + timeout
        while True:
            pos = self.drawer_status()[drawer - 1]
            if on_tick:
                on_tick(pos)
            if pos == want:
                return pos
            if t_end is not None and time.monotonic() >= t_end:
                raise AuxTimeout("drawer %d is %s, wanted %s after %.0f s"
                                 % (drawer, pos.name, want.name, timeout))
            time.sleep(period)


def _check_drawer(drawer: int) -> None:
    if not 1 <= drawer <= DRAWERS:
        raise ValueError("drawer 1..%d" % DRAWERS)


def _check_compartment(row: int, col: int) -> None:
    if not 1 <= row <= ROWS_PER_DRAWER:
        raise ValueError("row 1..%d" % ROWS_PER_DRAWER)
    if not 1 <= col <= COLS_PER_ROW:
        raise ValueError("column 1..%d" % COLS_PER_ROW)


# ---- example helpers -------------------------------------------------------
def open_from_args(argv: Sequence[str] = None):
    """Shared command-line handling for the examples:
         --port COM3        real cabinet on that serial port
         --sim              software cabinet (el980_sim.py), no hardware
         --trace            print every frame
    Returns (el980, args, sim_or_None)."""
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    p.add_argument("--sim", action="store_true", help="run against the software cabinet")
    p.add_argument("--trace", action="store_true", help="print every frame on the wire")
    p.add_argument("--drawer", type=int, default=1)
    p.add_argument("--row", type=int, default=1)
    p.add_argument("--col", type=int, default=1)
    args = p.parse_args(argv)
    sim = None
    if args.sim or not args.port:
        import el980_sim
        sim = el980_sim.Cabinet()
        link = el980_sim.SimLink(sim)
        if not args.sim:
            print("(no --port given: using the software cabinet, --sim)")
    else:
        link = SerialLink(args.port)
    m = EL980(link)
    if args.trace:
        m.trace = lambda s: print("    " + s)
    return m, args, sim
