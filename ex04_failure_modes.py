"""
ex04_failure_modes.py - every refusal and failure the host can meet, each
provoked on purpose, with the bytes the EL980 answers and how to decode them.

    python ex04_failure_modes.py --sim                 all scenarios, unattended
    python ex04_failure_modes.py --sim --only C,E,G    a selection
    python ex04_failure_modes.py --port COM3 --only A,C,J,L
         (on a real cabinet only the scenarios that move nothing are sensible
          unless an operator is standing at it; see each docstring)

Scenario    What is provoked                              Expected reply
  A  Drawer that is not fitted: Drawer Open                   0x8A bit 3 alone
     ... and Vend Any for the same drawer                     bit 3 + bit 2 (as offline)
  B  Vend Any / Drawer Open for an offline drawer board       bit 3 + bit 2
  C  Batch that would span two drawers                        bit 3 alone, queue kept
  D  Commit while the drawer is only partly out               bit 3 alone, queue cleared
  E  Drawer Open while another drawer is out (interlock)      0x8A bit 3 + bit 6
  F  Vend Any / Drawer Open while a vend is in progress       bit 3 + bit 7
  G  Lid that does not release (stuck lid / dead switch)      vend DID_NOT_SUCCEED after 3 s
  H  Drawer latch that does not release on Drawer Open        0x8A ~5 s later, bit 3, still CLOSED
  I  Opened drawer abandoned with Init                        out-lock freed, drawer can go home
  J  Queue full (4th compartment staged)                      bit 3 alone
  K  Machine (not 0) in a drawer-only command                 0xFF unknown-command reply
  L  Drawer left open with no transaction: Out of service     0x89 bit 2 set
"""
import sys
import time

from el980_aux import (open_from_args, DrawerPos, AuxRejected, AuxUnknownCommand, AuxError,
                       EL980, QUEUE_DEPTH)
from cabinet_operator import Operator

SCENARIOS = {}


def scenario(key, title):
    def deco(fn):
        SCENARIOS[key] = (title, fn)
        return fn
    return deco


def expect_reject(what, fn, *a):
    try:
        st = fn(*a)
        print("   %s -> ACCEPTED (unexpected): %s" % (what, st))
        return None
    except AuxRejected as e:
        r = e.result
        reason = r.reason if hasattr(r, "reason") else r.reject_reason()
        print("   %s -> REFUSED: %s\n      decode: %s" % (what, r, reason))
        return r


def not_fitted_drawer(m):
    pos = m.drawer_status()
    for d in range(8, 0, -1):
        if pos[d - 1] == DrawerPos.INDETERMINATE:
            return d
    return None


# ---------------------------------------------------------------------------
@scenario("A", "drawer not fitted / out of range")
def sc_a(m, sim, op):
    """Moves nothing. Needs a drawer position that is not fitted. Note the two
    commands word it differently: Drawer Open says 'not fitted' (bit 3 alone),
    Vend Any says 'drawer offline' (bit 3 + bit 2) because the selection
    itself exists and it is the drawer that is not answering."""
    if sim:
        sim.fitted.discard(8)
    d = not_fitted_drawer(m)
    if d is None:
        print("   every drawer is fitted on this cabinet: nothing to show")
        return
    expect_reject("vend %d-1-1" % d, m.vend_now, d, 1, 1)
    expect_reject("drawer open %d" % d, m.drawer_open, d)
    m.init()


@scenario("B", "drawer board offline")
def sc_b(m, sim, op):
    """Moves nothing. On hardware: unplug one drawer board's bus cable first
    and pass --drawer N."""
    d = 3 if sim else None
    if sim:
        sim.offline.add(d)
    else:
        print("   (hardware: use --only B --drawer N with drawer N's board disconnected)")
        return
    expect_reject("vend %d-1-1" % d, m.vend_now, d, 1, 1)
    expect_reject("drawer open %d" % d, m.drawer_open, d)
    print("   door status       : %s" % m.door_status(d, 1))
    print("   retrying the same selection fails until the board answers again")
    sim.offline.discard(d)
    m.init()


@scenario("C", "batch spanning two drawers")
def sc_c(m, sim, op):
    """Moves nothing (stage only, then Init clears the queue)."""
    print("   stage 1-1-1       : %s" % m.vend_stage(1, 1, 1))
    expect_reject("stage 2-1-1", m.vend_stage, 2, 1, 1)
    print("   the queue still holds 1-1-1; Init clears it: %s" % m.init())


@scenario("D", "commit while the drawer is only partly out")
def sc_d(m, sim, op):
    """Releases drawer 1's lock; the operator must NOT pull it out, then push it home."""
    print("   drawer open 1     : %s" % m.drawer_open(1))
    print("   stage 1-1-1       : %s" % m.vend_stage(1, 1, 1))
    expect_reject("commit (drawer PARTIAL)", m.vend_commit)
    print("   a refused commit clears the queue; staging again is accepted:")
    print("   stage 1-1-1       : %s" % m.vend_stage(1, 1, 1))
    print("   init              : %s" % m.init())
    op.push_home(m, 1)


@scenario("E", "interlock: a second drawer while one is out")
def sc_e(m, sim, op):
    """Drawer 1 is opened and pulled out, then drawer 2 is requested."""
    print("   drawer open 1     : %s" % m.drawer_open(1))
    op.pull_out(m, 1)
    expect_reject("drawer open 2", m.drawer_open, 2)
    expect_reject("vend 2-1-1 (commit for a drawer that is not open)", m.vend_now, 2, 1, 1)
    print("   drawer open 1 again is a harmless no-op: %s" % m.drawer_open(1))
    print("   finishing drawer 1 with a vend so it can be closed")
    print("   vend 1-1-1        : %s" % m.vend_now(1, 1, 1))
    print("   result            : %s" % m.wait_vend_done())
    op.close_lids(m, 1)
    op.push_home(m, 1)


@scenario("F", "machine busy: request during a vend")
def sc_f(m, sim, op):
    """A vend from a closed drawer runs the whole sequence (lock, pull, lid),
    so the machine is busy while the operator pulls; requests then get bit 3 + bit 7."""
    print("   vend 1-1-1 from closed: %s" % m.vend_now(1, 1, 1))
    expect_reject("vend 1-1-2 while busy", m.vend_now, 1, 1, 2)
    expect_reject("drawer open 2 while busy", m.drawer_open, 2)
    op.pull_out(m, 1)
    print("   result            : %s" % m.wait_vend_done())
    op.close_lids(m, 1)
    op.push_home(m, 1)


@scenario("G", "lid does not release")
def sc_g(m, sim, op):
    """The vend reports DID_NOT_SUCCEED 3 s after the lid was fired. The drawer
    stays out until its lids read shut, then is freed as usual."""
    d = 2
    if sim:
        sim.stuck_lids.add((d, 1, 1))
    else:
        print("   (hardware: hold lid %d-1-1 shut by hand when it fires)" % d)
    print("   drawer open %d     : %s" % (d, m.drawer_open(d)))
    op.pull_out(m, d)
    print("   stage %d-1-1       : %s" % (d, m.vend_stage(d, 1, 1)))
    print("   stage %d-1-2       : %s" % (d, m.vend_stage(d, 1, 2)))
    print("   commit            : %s" % m.vend_commit())
    t0 = time.monotonic()
    st = m.wait_vend_done(timeout=20)
    print("   result after %.1f s: %s" % (time.monotonic() - t0, st))
    print("   lid row           : %s" % m.door_status(d, 1))
    print("   host: one lid of the batch did not open; ask the operator to check %d-1-1" % d)
    if sim:
        sim.stuck_lids.clear()
    op.close_lids(m, d)
    op.push_home(m, d)


@scenario("H", "drawer latch does not release")
def sc_h(m, sim, op):
    """0x8A arrives about 5 s after the command with bit 3 set and the
    position still CLOSED. Read timeout for 0x0A must be 6 s."""
    d = 4
    if sim:
        sim.latch_fail.add(d)
    else:
        print("   (hardware: hold drawer %d's lock engaged, or disconnect its lock coil)" % d)
    t0 = time.monotonic()
    expect_reject("drawer open %d" % d, m.drawer_open, d)
    print("   reply took %.2f s" % (time.monotonic() - t0))
    if sim:
        sim.latch_fail.clear()


@scenario("I", "opened drawer abandoned with Init")
def sc_i(m, sim, op):
    """Drawer Open, operator pulls it out, host never commits. Init ends the
    wait: the out-lock is released once the lids are shut (they are), so
    the drawer reads PARTIAL and can be pushed home. Nothing is re-latched."""
    print("   drawer open 1     : %s" % m.drawer_open(1))
    op.pull_out(m, 1)
    print("   (host decides to abandon)")
    print("   init              : %s" % m.init())
    m.wait_drawer(1, DrawerPos.PARTIAL, timeout=5)
    print("   drawer 1 reads PARTIAL: out-lock released")
    op.push_home(m, 1)


@scenario("J", "queue full")
def sc_j(m, sim, op):
    """Moves nothing."""
    for c in range(1, QUEUE_DEPTH + 1):
        print("   stage 1-1-%d       : %s" % (c, m.vend_stage(1, 1, c)))
    expect_reject("stage 1-1-%d (4th)" % (QUEUE_DEPTH + 1), m.vend_stage, 1, 1, QUEUE_DEPTH + 1)
    print("   init              : %s" % m.init())


@scenario("K", "drawer command addressed to an AUX machine")
def sc_k(m, sim, op):
    """Moves nothing. 0x0A / 0x0B with Machine != 0 answer 0xFF."""
    aux1 = EL980(m.link, machine=1)
    aux1.trace = m.trace
    try:
        aux1.drawer_status()
        print("   accepted (unexpected)")
    except AuxUnknownCommand as e:
        print("   drawer status for machine 1 -> %s" % e)


@scenario("L", "drawer left open: Out of service")
def sc_l(m, sim, op):
    """A fitted drawer that is not home with no transaction attached takes
    the machine out of service (0x89 bit 2); the display says 'Close Drawer N'."""
    print("   drawer open 1     : %s" % m.drawer_open(1))
    print("   init (abandon)    : %s" % m.init())
    m.wait_drawer(1, DrawerPos.PARTIAL, timeout=5)
    print("   machine status    : %s" % m.machine_status())
    expect_reject("vend 2-1-1 while drawer 1 is out", m.vend_now, 2, 1, 1)
    op.push_home(m, 1)
    print("   machine status    : %s" % m.machine_status())


# ---------------------------------------------------------------------------
def main():
    only = None
    for i, a in enumerate(sys.argv):
        if a == "--only":
            only = [k.strip().upper() for k in sys.argv[i + 1].split(",")]
            del sys.argv[i:i + 2]
            break
    m, args, sim = open_from_args()
    op = Operator(sim, delay=0.8)
    keys = only or sorted(SCENARIOS)
    failures = 0
    try:
        print("init: %s" % m.init())
        for k in keys:
            title, fn = SCENARIOS[k]
            print("\n== %s: %s ==" % (k, title))
            try:
                fn(m, sim, op)
            except AuxError as e:
                failures += 1
                print("   !! %s: %s" % (type(e).__name__, e))
            pos = m.drawer_status()
            bad = [i + 1 for i, p in enumerate(pos) if p in (DrawerPos.OPEN, DrawerPos.PARTIAL)]
            if bad:
                print("   (drawers still out: %s)" % bad)
        return 1 if failures else 0
    finally:
        m.close()


if __name__ == "__main__":
    sys.exit(main())
