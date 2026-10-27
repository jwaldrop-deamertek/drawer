# EL980 drawer cabinet - AUX interface examples (Python)

Example host code for the Seaga EL980 AUX serial interface, spec
"Seaga EL980 AUX interface 2.4.0", drawer-cabinet commands. Written against
VMC firmware build 1963 with drawer boards on build 1029.

Everything runs on plain Python 3.8+ (tested with 3.11). The real serial
link needs pyserial:

    pip install pyserial

Every example takes `--port COM3` (or `/dev/ttyUSB0`) for a cabinet on that
serial port, or `--sim` to run against the software cabinet in
`el980_sim.py`, which needs no hardware. `--trace` prints every frame on
the wire with its timing.

## Files

| File | What it is |
|---|---|
| `el980_aux.py` | The protocol library: framing, checksum, resync, one method per command, decoders for every response byte, `wait_*` helpers. Drawer, row and column are 1-based in the API (drawer 1..8, row 1..7, column 1..6) and converted to the 0-based wire values. |
| `el980_sim.py` | A software cabinet: 8 drawers, 7 x 6 lids each, interlock, out-lock, vend queue, lid timeout, deferred Drawer Open reply, Init semantics. Operator actions and fault injection are methods on `Cabinet`. |
| `cabinet_operator.py` | The human steps (pull the drawer out, shut the lids, push it home): prompts on hardware, automatic on the software cabinet. |
| `ex01_identify.py` | Power-up handshake and a read-only picture: version, machine status, type declaration, Init, Drawer Status, lid status of one drawer. |
| `ex02_single_vend.py` | One complete transaction: Drawer Open, pull, Vend Any, Poll to the result, lids shut, out-lock freed, drawer home. |
| `ex03_batch_vend.py` | Three compartments of one drawer staged then committed as one vend with one result. |
| `ex04_failure_modes.py` | Twelve refusals and failures provoked on purpose, each with the bytes returned and the decode. |
| `ex05_door_scan.py` | The lid grid of every fitted drawer, once or live (`--watch`). |

    python ex01_identify.py --sim
    python ex02_single_vend.py --sim --drawer 2 --row 3 --col 4
    python ex03_batch_vend.py --sim --drawer 3
    python ex04_failure_modes.py --sim
    python ex04_failure_modes.py --port COM3 --only A,C,J,K
    python ex05_door_scan.py --port COM3 --watch

## The transaction (ex02)

What the host sends, in order, and what it waits for. Every reply is
immediate (within 20 ms) except Drawer Open.

| Step | Command | Wait for |
|---|---|---|
| 1 | `A5 01 00 00 01` Init | reply `81 30 ..` (drawer cabinet, idle) |
| 2 | `A5 0B 00 00 0B` Drawer Status | every fitted drawer `01` closed |
| 3 | `A5 0A 10 00 1A` Drawer Open (drawer 2) | reply `8A 10 02 ..` arrives when the latch has released, about 0.4 s, or with bit 3 set after 5 s when it did not. Read timeout 6 s. |
| 4 | Drawer Status, repeated | drawer 2 reads `11` open. Operator-paced, no firmware timeout. |
| 5 | `A5 07 10 23 3A` Vend Any drawer 2 row 3 col 4, queue + vend | reply `87 B0 ..` IN_PROGRESS |
| 6 | `A5 02 00 00 02` Poll, repeated | bit 7 clears; bit 6 = SUCCESS, bit 0 = DID NOT SUCCEED (lid not released within 3 s) |
| 7 | Drawer Status, repeated | drawer 2 reads `10` partial: all its lids read shut and the firmware released the out-lock |
| 8 | Drawer Status, repeated | drawer 2 reads `01` closed. Done; the next drawer may be opened. |

Steps 4, 7 and 8 depend on the operator. The firmware never times them
out; the host decides when to prompt. An Init at any point abandons an
opened drawer (its out-lock is released once the lids are shut); nothing
is ever re-latched by firmware.

A batch (ex03) replaces step 5 with up to three "queue, don't vend"
requests (`A5 07 18 rc ..`, D1 bit 3 set) followed by one commit
(`A5 07 14 00 ..`, D1 bit 2 set, drawer/row/column ignored). All staged
compartments must be in the same drawer. One result is reported: SUCCESS
only when every lid released.

## Refusals and failures (ex04)

Vend Any replies carry the Poll-shaped status byte; bit 3 set means the
request was not accepted and nothing was queued or started.

| Scenario | Request | Reply | Decode |
|---|---|---|---|
| A | Drawer Open, drawer not fitted | `8A 70 08` | bit 3 alone: not fitted / out of range |
| A | Vend Any, drawer not fitted | `87 3C` | bit 3 + bit 2: the selection exists, the drawer is not answering |
| B | Vend Any / Drawer Open, board offline | `87 3C` / `8A x0 0D` | bit 3 + bit 2; 0x08 for that drawer answers `80` |
| C | Second stage in a different drawer | `87 38` | bit 3 alone; the first item stays queued |
| D | Commit while the drawer is partial | `87 38` | bit 3 alone; the queue is cleared |
| E | Drawer Open while another drawer is out | `8A x0 49` | bit 3 + bit 6: interlock |
| F | Vend Any during a vend | `87 B8` | bit 3 + bit 7: busy, poll until bit 7 clears |
| F | Drawer Open during a vend | `8A x0 89` | bit 3 + bit 7 |
| G | Lid does not release | Poll `82 31` about 3 s after the commit | DID NOT SUCCEED; 0x08 shows which lid is still closed |
| H | Latch does not release on Drawer Open | `8A x0 09` after about 5 s | bit 3, position still closed |
| I | Opened drawer abandoned | Init, then Drawer Status reads partial | out-lock released, drawer can go home |
| J | Fourth compartment staged | `87 38` | queue full (3) |
| K | 0x0A / 0x0B with Machine 1..3 | `FF 0B` | unknown command: drawer commands are Machine 0 only |
| L | Drawer left out, no transaction | 0x89 byte 3 bit 2 set | out of service until it is home; display says "Close Drawer N" |

Bytes shown are the response code and D1 (and D2 for 0x8A); `x0` is the
drawer echo.

## Notes for an implementation

- **Retries.** Replies to Poll, Drawer Status, Door Status, Machine Status
  and Version are idempotent: resend on a lost reply. Do not resend a Vend
  Any stage request blindly, a repeat queues the item twice; re-read with
  Poll/Init instead. Drawer Open for a drawer that already reads partial or
  open is a harmless no-op, so it is safe to repeat.
- **Timeouts.** 250 ms covers every reply except Drawer Open (6 s: the
  firmware waits 5 s for the closed switch before giving up, build 1963;
  the 2.4.0 text says 2 s). The
  library resynchronises on the `A5` header and drops a frame with a bad
  checksum.
- **Rows.** The 2.4.0 text says 6 rows per drawer. VMC builds 1945 and later
  lay out 7 rows (42 compartments per drawer); `ROWS_PER_DRAWER` in
  `el980_aux.py` is 7. Row 8 in Door Status (0x08) returns the out-lock
  switch row for diagnosis, not lids.
- **Lid polarity.** 0x08 bits 5..0 are 1 = closed, bit 0 = column 1; bit 7
  set means the drawer board is not answering (or the row is invalid), so
  `00` is a real reading (all six lids open) and `80` is not.
- **Out of service.** On a drawer cabinet, 0x89 bit 2 is set while a
  fitted drawer is not home with no transaction attached to it. A vend
  request then is refused (bit 3) and the cabinet display reads "Close
  Drawer N".
- **Type declaration.** Re-declaring the drawer type (`A5 16 30 00 46`) is
  a no-op and good practice at power-up. Changing the type restarts the
  board about 2 s after the reply; allow 20 s. The examples never change
  it.
