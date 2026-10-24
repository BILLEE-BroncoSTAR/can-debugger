# ODrive CANSimple Candump Decoder

A dependency-free Linux desktop viewer for turning SocketCAN `candump` output into ODrive CANSimple messages.

It reads normal `candump` lines such as:

```text
(1700000000.000001) can0 029#0000484100002040
```

and displays the original CAN ID, derived ODrive node ID, command code/name, raw bytes, and decoded fields. CANSimple uses an 11-bit ID where `node_id = can_id >> 5` and `cmd_id = can_id & 0x1f`.

The decoder follows the ODrive [CAN protocol documentation](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html) for firmware 0.6.12.

## Run

Python 3.11+ and Tk are required. `candump` is only necessary for live capture; pasted text and log files work without it.

```bash
uv run odesc-can-debugger
```

or:

```bash
python -m odesc_can_debugger.app
```

To capture live traffic, configure your SocketCAN device normally and enter its interface name (for example `can0`) in the app, then choose **Start candump**. The app invokes `candump <interface>` without a shell.

## Input formats

The viewer accepts standard compact `candump` output and the byte-separated `candump -x` format, including optional timestamps. It also accepts CAN-FD `##` frame notation, while decoding the CANSimple payload itself (which uses at most eight bytes).

Unknown command IDs, malformed frames, incomplete payloads, and non-CANSimple extended/error IDs are called out instead of being misinterpreted.
