"""
ex05_door_scan.py - lid status of every compartment (0x08 per drawer/row),
as a grid, once or continuously.

    python ex05_door_scan.py --sim
    python ex05_door_scan.py --port COM3 --watch     (redraw on every change)

C = closed, o = open, '-' = drawer offline / row invalid. Drawers that
read INDETERMINATE in Drawer Status while closed are not fitted and are
skipped. Row 8 is the out-lock switch row (not a lid row).
"""
import sys
import time

from el980_aux import open_from_args, DrawerPos, ROWS_PER_DRAWER, DRAWERS, AuxError


def snapshot(m, fitted):
    grid = {}
    for d in fitted:
        for row in range(1, ROWS_PER_DRAWER + 1):
            dr = m.door_status(d, row)
            grid[(d, row)] = "------" if dr.offline else "".join("C" if c else "o" for c in dr.closed)
    return grid


def show(grid, fitted, pos):
    print("        " + "  ".join("drawer%d" % d for d in fitted))
    print("        " + "  ".join("%-7s" % pos[d - 1].name[:7] for d in fitted))
    for row in range(1, ROWS_PER_DRAWER + 1):
        print("row %d   " % row + "  ".join("%-7s" % grid[(d, row)] for d in fitted))


def main():
    watch = "--watch" in sys.argv
    if watch:
        sys.argv.remove("--watch")
    m, args, sim = open_from_args()
    try:
        pos = m.drawer_status()
        fitted = [d for d in range(1, DRAWERS + 1) if pos[d - 1] != DrawerPos.INDETERMINATE]
        if not fitted:
            print("no drawer reads a position: nothing fitted, or the boards are not answering yet")
            return 1
        last = None
        while True:
            pos = m.drawer_status()
            grid = snapshot(m, fitted)
            if grid != last:
                print(time.strftime("%H:%M:%S"))
                show(grid, fitted, pos)
                last = grid
            if not watch:
                return 0
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    except AuxError as e:
        print("protocol error: %s" % e)
        return 2
    finally:
        m.close()


if __name__ == "__main__":
    sys.exit(main())
