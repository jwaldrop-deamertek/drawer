"""
ex01_identify.py - power-up handshake and a read-only picture of the cabinet.

    python ex01_identify.py --port COM3        (real cabinet)
    python ex01_identify.py --sim              (software cabinet)

Sequence (spec FAQ 7 / 10): version, machine status, declare the drawer
type (a no-op when it already is one), Init, Drawer Status, then the lid
status of every row of one drawer.
"""
import sys
from el980_aux import (open_from_args, MACHINE_TYPE_DRAWER, DrawerPos, ROWS_PER_DRAWER,
                       AuxError)


def main():
    m, args, sim = open_from_args()
    try:
        build = m.version()
        print("firmware build      : %d" % build)

        ms = m.machine_status()
        print("machine status      : %s" % ms)
        if not ms.is_drawer_cabinet:
            print("  -> not a drawer cabinet. Set Machine Type (0x16) would switch it;")
            print("     that restarts the board, so this example does not send it.")
            return 1

        # Re-declaring the current type is a no-op; good practice once per power-up.
        ms = m.set_machine_type(MACHINE_TYPE_DRAWER)
        print("set type (no-op)    : %s" % ms)

        st = m.init()
        print("init                : %s" % st)

        pos = m.drawer_status()
        print("drawer status       :")
        for d, p in enumerate(pos, start=1):
            note = "" if p != DrawerPos.INDETERMINATE else "  (not fitted, or board not answering)"
            print("   drawer %d: %-13s%s" % (d, p.name, note))

        d = args.drawer
        print("lid status, drawer %d (C = closed, o = open):" % d)
        for row in range(1, ROWS_PER_DRAWER + 1):
            print("   " + str(m.door_status(d, row)))
        return 0
    except AuxError as e:
        print("protocol error: %s" % e)
        return 2
    finally:
        m.close()


if __name__ == "__main__":
    sys.exit(main())
