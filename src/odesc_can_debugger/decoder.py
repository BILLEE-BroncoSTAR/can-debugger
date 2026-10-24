"""Decode SocketCAN ``candump`` frames using the ODrive CANSimple protocol."""

from __future__ import annotations

from dataclasses import dataclass
import re
import struct


COMMANDS: dict[int, str] = {
    0x00: "Get_Version", 0x01: "Heartbeat", 0x02: "Estop",
    0x03: "Get_Error", 0x04: "RxSdo", 0x05: "TxSdo", 0x06: "Address",
    0x07: "Set_Axis_State", 0x09: "Get_Encoder_Estimates",
    0x0B: "Set_Controller_Mode", 0x0C: "Set_Input_Pos",
    0x0D: "Set_Input_Vel", 0x0E: "Set_Input_Torque", 0x0F: "Set_Limits",
    0x11: "Set_Traj_Vel_Limit", 0x12: "Set_Traj_Accel_Limits",
    0x13: "Set_Traj_Inertia", 0x14: "Get_Iq", 0x15: "Get_Temperature",
    0x16: "Reboot", 0x17: "Get_Bus_Voltage_Current",
    0x18: "Clear_Errors", 0x19: "Set_Absolute_Position",
    0x1A: "Set_Pos_Gain", 0x1B: "Set_Vel_Gains", 0x1C: "Get_Torques",
    0x1D: "Get_Powers", 0x1F: "Enter_DFU_Mode",
}

AXIS_STATES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE", 3: "FULL_CALIBRATION_SEQUENCE",
    4: "MOTOR_CALIBRATION", 6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND",
    11: "HOMING", 12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION",
}
CONTROL_MODES = {0: "VOLTAGE_CONTROL", 1: "TORQUE_CONTROL", 2: "VELOCITY_CONTROL", 3: "POSITION_CONTROL"}
INPUT_MODES = {0: "INACTIVE", 1: "PASSTHROUGH", 2: "VEL_RAMP", 3: "POS_FILTER", 4: "MIX_CHANNELS", 5: "TRAP_TRAJ", 6: "TORQUE_RAMP", 7: "MIRROR"}
PROCEDURE_RESULTS = {0: "SUCCESS", 1: "BUSY", 2: "CANCELLED", 3: "DISARMED", 4: "NO_RESPONSE", 5: "POLE_PAIR_CPR_MISMATCH", 6: "PHASE_RESISTANCE_OUT_OF_RANGE", 7: "PHASE_INDUCTANCE_OUT_OF_RANGE", 8: "UNBALANCED_PHASES", 9: "INVALID_MOTOR_TYPE", 10: "ILLEGAL_HALL_STATE", 11: "INDEX_NOT_FOUND", 12: "ABS_SPI_TIMEOUT", 13: "NOT_CONVERGING"}
REBOOT_ACTIONS = {0: "reboot", 1: "save configuration", 2: "erase configuration", 3: "enter DFU mode"}

# Standard candump: "(timestamp) can0 123#deadbeef". The bracket form is
# emitted by candump -x, while CAN-FD uses "##" (possibly with flag bits).
FRAME_RE = re.compile(
    r"^\s*(?:\((?P<timestamp>[^)]+)\)\s+)?(?P<interface>\S+)\s+"
    r"(?P<can_id>[0-9A-Fa-f]+)(?P<rtr>R?)(?:#(?P<data>[0-9A-Fa-f]*))?\s*$"
)
FD_FRAME_RE = re.compile(
    r"^\s*(?:\((?P<timestamp>[^)]+)\)\s+)?(?P<interface>\S+)\s+"
    r"(?P<can_id>[0-9A-Fa-f]+)##[0-9A-Fa-f](?P<data>[0-9A-Fa-f]*)\s*$"
)
BRACKET_FRAME_RE = re.compile(
    r"^\s*(?:\((?P<timestamp>[^)]+)\)\s+)?(?P<interface>\S+)\s+"
    r"(?P<can_id>[0-9A-Fa-f]+)\s+\[(?P<length>\d+)]\s*(?P<data>(?:[0-9A-Fa-f]{2}\s*)*)$"
)


@dataclass(frozen=True)
class CanFrame:
    raw: str
    timestamp: str
    interface: str
    can_id: int
    data: bytes
    is_rtr: bool = False


@dataclass(frozen=True)
class DecodedFrame:
    frame: CanFrame
    node_id: int
    command_id: int
    command_name: str
    description: str


def parse_candump_line(line: str) -> CanFrame:
    """Parse one normal or ``candump -x`` line into a SocketCAN frame."""
    raw = line.rstrip("\n")
    bracket = BRACKET_FRAME_RE.match(raw)
    match = bracket or FD_FRAME_RE.match(raw) or FRAME_RE.match(raw)
    if not match:
        raise ValueError("Not a recognized candump frame")
    fields = match.groupdict()
    can_id = int(fields["can_id"], 16)
    if can_id > 0x7FF:
        raise ValueError("Extended/error CAN IDs are not CANSimple 11-bit IDs")
    data_text = (fields.get("data") or "").replace(" ", "")
    if len(data_text) % 2:
        raise ValueError("Payload has an odd number of hex digits")
    try:
        data = bytes.fromhex(data_text)
    except ValueError as exc:
        raise ValueError("Payload is not hexadecimal") from exc
    return CanFrame(raw, fields.get("timestamp") or "", fields["interface"], can_id, data, bool(fields.get("rtr")))


def _need(data: bytes, length: int) -> bool:
    return len(data) >= length


def _u8(data: bytes, offset: int = 0) -> int:
    return data[offset]


def _u16(data: bytes, offset: int = 0) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def _i16(data: bytes, offset: int = 0) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def _u32(data: bytes, offset: int = 0) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def _f32(data: bytes, offset: int = 0) -> float:
    return struct.unpack_from("<f", data, offset)[0]


def _number(value: float) -> str:
    return f"{value:.6g}"


def _named(value: int, names: dict[int, str]) -> str:
    return f"{value} ({names.get(value, 'UNKNOWN')})"


def _float_fields(data: bytes, fields: list[tuple[str, str]]) -> str:
    if not _need(data, 4 * len(fields)):
        return "Incomplete payload"
    return "; ".join(f"{name}: {_number(_f32(data, index * 4))} {unit}".rstrip() for index, (name, unit) in enumerate(fields))


def decode_frame(frame: CanFrame) -> DecodedFrame:
    """Decode a CANSimple frame. Unknown commands preserve the raw payload."""
    node_id, command_id = frame.can_id >> 5, frame.can_id & 0x1F
    name = COMMANDS.get(command_id, "Unknown command")
    data = frame.data
    if frame.is_rtr:
        description = "Remote transmission request (RTR); payload requested from ODrive"
    elif command_id in (0x02, 0x1F):
        description = "Empty payload" if not data else f"Expected empty payload; got {data.hex(' ').upper()}"
    elif command_id == 0x00:
        description = (f"Protocol: {_u8(data, 0)}; Hardware: {_u8(data, 1)}.{_u8(data, 2)} variant {_u8(data, 3)}; Firmware: {_u8(data, 4)}.{_u8(data, 5)}.{_u8(data, 6)}" + (" (unreleased)" if _u8(data, 7) else "")) if _need(data, 8) else "Incomplete payload"
    elif command_id == 0x01:
        description = (f"Axis errors: 0x{_u32(data):08X}; State: {_named(_u8(data, 4), AXIS_STATES)}; Procedure: {_named(_u8(data, 5), PROCEDURE_RESULTS)}; Trajectory done: {'yes' if _u8(data, 6) else 'no'}") if _need(data, 7) else "Incomplete payload"
    elif command_id == 0x03:
        description = f"Active errors: 0x{_u32(data):08X}; Disarm reason: 0x{_u32(data, 4):08X}" if _need(data, 8) else "Incomplete payload"
    elif command_id in (0x04, 0x05):
        if not _need(data, 8):
            description = "Incomplete payload"
        elif command_id == 0x04:
            description = f"{'Write' if _u8(data) else 'Read'}; Endpoint ID: {_u16(data, 1)}; Value (raw uint32): 0x{_u32(data, 4):08X}"
        else:
            description = f"Endpoint ID: {_u16(data, 1)}; Value (raw uint32): 0x{_u32(data, 4):08X}"
    elif command_id == 0x06:
        description = f"Node ID: {_u8(data)}; Serial number: {int.from_bytes(data[1:7], 'little')}; Connection ID: {_u8(data, 7)}" if _need(data, 8) else "Incomplete payload"
    elif command_id == 0x07:
        description = f"Requested state: {_named(_u32(data), AXIS_STATES)}" if _need(data, 4) else "Incomplete payload"
    elif command_id == 0x09:
        description = _float_fields(data, [("Position", "rev"), ("Velocity", "rev/s")])
    elif command_id == 0x0B:
        description = f"Control mode: {_named(_u32(data), CONTROL_MODES)}; Input mode: {_named(_u32(data, 4), INPUT_MODES)}" if _need(data, 8) else "Incomplete payload"
    elif command_id == 0x0C:
        description = (f"Position: {_number(_f32(data))} rev; Velocity feed-forward: {_number(_i16(data, 4) / 1000)} rev/s; Torque feed-forward: {_number(_i16(data, 6) / 1000)} Nm") if _need(data, 8) else "Incomplete payload"
    elif command_id == 0x0D:
        description = _float_fields(data, [("Velocity", "rev/s"), ("Torque feed-forward", "Nm")])
    elif command_id == 0x0E:
        description = _float_fields(data, [("Torque", "Nm")])
    elif command_id == 0x0F:
        description = _float_fields(data, [("Velocity limit", "rev/s"), ("Current limit", "A")])
    elif command_id == 0x11:
        description = _float_fields(data, [("Trajectory velocity limit", "rev/s")])
    elif command_id == 0x12:
        description = _float_fields(data, [("Trajectory acceleration limit", "rev/s²"), ("Trajectory deceleration limit", "rev/s²")])
    elif command_id == 0x13:
        description = _float_fields(data, [("Trajectory inertia", "Nm/(rev/s²)")])
    elif command_id == 0x14:
        description = _float_fields(data, [("Iq setpoint", "A"), ("Iq measured", "A")])
    elif command_id == 0x15:
        description = _float_fields(data, [("FET temperature", "°C"), ("Motor temperature", "°C")])
    elif command_id == 0x16:
        description = f"Action: {_named(_u8(data), REBOOT_ACTIONS)}" if _need(data, 1) else "Incomplete payload"
    elif command_id == 0x17:
        description = _float_fields(data, [("Bus voltage", "V"), ("Bus current", "A")])
    elif command_id == 0x18:
        description = f"Identify LED: {'on' if _u8(data) else 'off'}" if _need(data, 1) else "Incomplete payload"
    elif command_id == 0x19:
        description = _float_fields(data, [("Absolute position", "rev")])
    elif command_id == 0x1A:
        description = _float_fields(data, [("Position gain", "(rev/s)/rev")])
    elif command_id == 0x1B:
        description = _float_fields(data, [("Velocity gain", "Nm/(rev/s)"), ("Velocity integrator gain", "Nm/rev")])
    elif command_id == 0x1C:
        description = _float_fields(data, [("Torque target", "Nm"), ("Torque estimate", "Nm")])
    elif command_id == 0x1D:
        description = _float_fields(data, [("Electrical power", "W"), ("Mechanical power", "W")])
    else:
        description = f"No CANSimple definition. Raw payload: {data.hex(' ').upper() or '(empty)'}"
    return DecodedFrame(frame, node_id, command_id, name, description)


def decode_line(line: str) -> DecodedFrame:
    """Convenience function for parsing and decoding one candump line."""
    return decode_frame(parse_candump_line(line))
