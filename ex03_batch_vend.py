"""
ex03_batch_vend.py - several compartments of ONE drawer in a single
transaction: stage up to 3 with "queue, don't vend", then one commit.
The firmware releases the lids one after another (250 ms apart) and
reports one result for the batch: SUCCESS only when every lid released.

    python ex03_batch_vend.py --sim --drawer 3
    python ex03_batch_vend.py --port COM3 --drawer 3
The compartments are rows 1..3, column 1 of that drawer (edit ITEMS).
"""
import sys

from el980_aux import open_from_args, AuxRejected, AuxError, ROWS_PER_DRAWER
from cabinet_operator import Operator


def main():
    m, args, sim = open_from_args()
    op = Operator(sim)
    d = args.drawer
    items = [(d, r, args.col) for r in range(1, min(3, ROWS_PER_DRAWER) + 1)]
    try:
        print("init                : %s" % m.init())
        print("drawer open         : %s" % m.drawer_open(d))
        op.pull_out(m, d)

        for (dd, r, c) in items:
            st = m.vend_stage(dd, r, c)             # Vend 1, Queue 0
            print("stage %d-%d-%d         : %s" % (dd, r, c, st))

        st = m.vend_commit()                        # Vend 0, Queue 1: drawer/row/col ignored
        print("commit              : %s" % st)
        st = m.wait_vend_done(timeout=20)
        print("batch result        : %s" % st)
        for (dd, r, c) in items:
            print("   " + str(m.door_status(dd, r)))

        op.close_lids(m, d)
        op.push_home(m, d)
        return 0 if st.success else 1
    except AuxRejected as e:
        print("REFUSED: %s" % e.result)
        return 3
    except AuxError as e:
        print("protocol error: %s" % e)
        return 2
    finally:
        if sim:
            print("\n-- cabinet model events --")
            print("\n".join(sim.events))
        m.close()


if __name__ == "__main__":
    sys.exit(main())
