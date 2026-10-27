"""
ex02_single_vend.py - one complete drawer transaction, the way the spec
(FAQ 10) describes it:

  Init -> Drawer Status (all closed) -> Drawer Open -> operator pulls it out
  -> Vend Any (one compartment) -> Poll until done -> operator shuts the lid
  -> firmware frees the out-lock -> operator pushes the drawer home.

    python ex02_single_vend.py --port COM3 --drawer 2 --row 1 --col 1
    python ex02_single_vend.py --sim --drawer 2 --row 3 --col 4
"""
import sys
import time

from el980_aux import open_from_args, DrawerPos, AuxRejected, AuxError
from cabinet_operator import Operator


def main():
    m, args, sim = open_from_args()
    op = Operator(sim)
    d, r, c = args.drawer, args.row, args.col
    try:
        print("init                : %s" % m.init())
        pos = m.drawer_status()
        print("drawer status       : %s" % " ".join("%d=%s" % (i + 1, p.name[:4]) for i, p in enumerate(pos)))
        if pos[d - 1] != DrawerPos.CLOSED:
            print("drawer %d is not closed (%s); the sequence below starts from a closed drawer." % (d, pos[d - 1].name))

        # 1. release the drawer lock. The reply comes when the latch has released (~0.4 s).
        t0 = time.monotonic()
        res = m.drawer_open(d)
        print("drawer open         : %s  (%.2f s)" % (res, time.monotonic() - t0))

        # 2. operator-paced: no firmware timeout. The host decides when to nag.
        op.pull_out(m, d)

        # 3. release the lid. Queue + vend in one request. The reply says IN_PROGRESS.
        st = m.vend_now(d, r, c)
        print("vend %d-%d-%d          : %s" % (d, r, c, st))

        # 4. poll until the lid has released (or the 3 s lid timeout expired).
        st = m.wait_vend_done(timeout=15)
        print("vend result         : %s" % st)
        if st.success:
            print("  compartment %d-%d-%d is open" % (d, r, c))
        else:
            print("  the lid did not read released within 3 s: ask the operator to check it")
        print("  lid row now       : %s" % m.door_status(d, r))

        # 5. items taken, lids shut -> out-lock freed -> drawer home.
        op.close_lids(m, d)
        op.push_home(m, d)
        print("machine status      : %s" % m.machine_status())
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
