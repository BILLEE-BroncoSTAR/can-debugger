import struct
import unittest

from odesc_can_debugger.decoder import decode_line, parse_candump_line


class DecoderTests(unittest.TestCase):
    def test_decodes_encoder_estimates_and_id_parts(self) -> None:
        # Node 1, Get_Encoder_Estimates (0x09): (1 << 5) | 0x09 = 0x029
        result = decode_line("(1700000000.010001) can0 029#0000484100002040")
        self.assertEqual(result.node_id, 1)
        self.assertEqual(result.command_id, 0x09)
        self.assertEqual(result.command_name, "Get_Encoder_Estimates")
        self.assertIn("Position: 12.5 rev", result.description)
        self.assertIn("Velocity: 2.5 rev/s", result.description)

    def test_decodes_bracket_candump_format(self) -> None:
        result = decode_line("can0 02E [4] 00 00 C0 3F")
        self.assertEqual(result.node_id, 1)
        self.assertEqual(result.command_name, "Set_Input_Torque")
        self.assertEqual(result.description, "Torque: 1.5 Nm")

    def test_heartbeat_uses_little_endian_fields(self) -> None:
        payload = struct.pack("<IBBB", 0x80000001, 8, 0, 1)
        result = decode_line(f"can0 021#{payload.hex()}")
        self.assertEqual(result.command_name, "Heartbeat")
        self.assertIn("Axis errors: 0x80000001", result.description)
        self.assertIn("8 (CLOSED_LOOP_CONTROL)", result.description)
        self.assertIn("Trajectory done: yes", result.description)

    def test_rejects_extended_can_id(self) -> None:
        with self.assertRaisesRegex(ValueError, "Extended"):
            parse_candump_line("can0 18FF50E5#00112233")
