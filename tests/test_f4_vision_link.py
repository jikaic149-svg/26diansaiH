"""PC端验证MaixCAM2<->F407 Packet V1闭环与实时调参。"""

import unittest

from f4_vision_link import (
    F4VisionLink,
    decode_vpa_packet,
    decode_vst_packet,
    encode_packet,
    parse_packet,
)


class FakePort:
    def __init__(self):
        self.written = bytearray()
        self.rx = bytearray()
        self.closed = False

    def write(self, data):
        self.written.extend(data)
        return len(data)

    def read(self, limit=256):
        chunk = bytes(self.rx[:limit])
        del self.rx[:limit]
        return chunk

    def close(self):
        self.closed = True


class F4VisionLinkTest(unittest.TestCase):
    def test_vtp_vpa_and_vst(self):
        port = FakePort()
        now = [100]
        link = F4VisionLink(
            port_factory=lambda _device, _baud: port,
            clock_ms=lambda: now[0],
        )
        self.assertTrue(link.start())
        self.assertTrue(link.send_pid_gains(1.2, 0.15, 1.8))
        packet_type, sequence, payload = parse_packet(
            bytes(port.written).rstrip(b"\n"))
        self.assertEqual(packet_type, "VTP")
        self.assertEqual(sequence, 0)
        self.assertEqual(payload, "1200,150,1800")

        port.written.clear()
        port.rx.extend(encode_packet(
            "VPA", 0, "0,1,1200,150,1800"))
        port.rx.extend(encode_packet(
            "VST", 0,
            "4,4,25,-12,34,0,-456,54329,54340,-91,325,-450,49151,3,6"))
        self.assertTrue(link.service(now[0]))
        self.assertTrue(link.last_vpa["accepted"])
        self.assertEqual(link.last_vpa["request_sequence"], 0)
        self.assertEqual(link.last_vst["estimate_mm"], -12)
        self.assertEqual(link.last_vst["theta_cmd_mdeg"], -456)
        self.assertEqual(link.last_vst["motor_target_raw"], 54329)
        self.assertEqual(link.last_vst["motor_feedback_raw"], 54340)
        self.assertEqual(link.last_vst["motor_command_ma"], -91)
        self.assertEqual(link.last_vst["motor_speed_centi_rpm"], 325)
        self.assertEqual(link.last_vst["stiction_kick_mdeg"], -450)
        self.assertEqual(link.last_vst["competition_mode"], 6)

    def test_decoders_reject_reserved_or_malformed_fields(self):
        accepted = decode_vpa_packet(1, "2,1,1200,150,1800")
        self.assertTrue(accepted["accepted"])
        with self.assertRaises(ValueError):
            decode_vpa_packet(1, "2,2,1200,150,1800")
        with self.assertRaises(ValueError):
            decode_vst_packet(
                1, "0,0,0,0,0,0,0,0,0,0,0,0,16384,0,0")
        with self.assertRaises(ValueError):
            decode_vst_packet(
                1, "0,0,0,0,0,0,0,0,0,0,0,0,0,0,1")

    def test_ball_target_packet_and_bounds(self):
        port = FakePort()
        link = F4VisionLink(
            port_factory=lambda _device, _baud: port,
            clock_ms=lambda: 200,
        )
        self.assertTrue(link.start())
        self.assertTrue(link.send_ball_target(37.0))
        packet_type, sequence, payload = parse_packet(
            bytes(port.written).rstrip(b"\n"))
        self.assertEqual(packet_type, "VBT")
        self.assertEqual(sequence, 0)
        self.assertEqual(payload, "370")
        self.assertEqual(link.ball_target_tx_count, 1)

        port.written.clear()
        self.assertTrue(link.send_ball_target(-120.0))
        packet_type, sequence, payload = parse_packet(
            bytes(port.written).rstrip(b"\n"))
        self.assertEqual(packet_type, "VBT")
        self.assertEqual(sequence, 1)
        self.assertEqual(payload, "-1200")
        with self.assertRaises(ValueError):
            link.send_ball_target(120.1)


if __name__ == "__main__":
    unittest.main()
