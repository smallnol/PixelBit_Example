"""離線單元測試：不需硬體。"""

from __future__ import annotations

import struct
import unittest

from aoa_protocol import (
    FOOTER,
    HEADER,
    decide_aoa_command,
    parse_json_line,
    try_parse_binary_frame,
)


def _build_binary(angle: int, dist_cm: int, sn: int = 1, addr: int = 0x2E5C) -> bytes:
  payload = bytearray()
  payload.append(sn & 0xFF)
  payload += struct.pack('<H', addr)
  payload += struct.pack('<i', angle)
  payload += struct.pack('<i', dist_cm)
  payload += struct.pack('<H', 0)  # user_cmd
  payload += struct.pack('<f', 0.0)  # F_Path
  payload += struct.pack('<f', 0.0)  # RX
  payload += struct.pack('<HHH', 0, 0, 0)  # Acc
  assert len(payload) == 27
  length = len(payload)
  xor = 0
  for b in payload:
    xor ^= b
  return bytes([HEADER, length]) + bytes(payload) + bytes([xor, FOOTER])


class AoaProtocolTest(unittest.TestCase):
  def test_binary_frame(self):
    frame = _build_binary(angle=-15, dist_cm=230)
    sample, rest = try_parse_binary_frame(frame)
    self.assertIsNotNone(sample)
    assert sample is not None
    self.assertEqual(rest, b'')
    self.assertAlmostEqual(sample.distance_m, 2.30)
    self.assertEqual(sample.angle_deg, -15.0)

  def test_json_line(self):
    line = (
        'JS0062{"TWR": {"a16":"2E5C","R":3,"T":8605,'
        '"D":185,"P":100,"Xcm":180,"Ycm":40,"O":1,"V":0,"X":0,"Y":0,"Z":0}}'
    )
    sample = parse_json_line(line)
    self.assertIsNotNone(sample)
    assert sample is not None
    self.assertAlmostEqual(sample.distance_m, 1.85)
    self.assertIsNotNone(sample.angle_deg)

  def test_decide(self):
    from aoa_protocol import AoaSample

    sample = AoaSample(distance_m=2.5, angle_deg=-20.0)
    cmd, _, _ = decide_aoa_command(
        following=True,
        sample=sample,
        age_s=0.1,
        stale_s=1.0,
        deadzone_deg=10,
        start_dist=2.0,
        stop_dist=1.8,
        angle_sign=1.0,
        prev_forward=False,
    )
    self.assertEqual(cmd, 'L')


if __name__ == '__main__':
  unittest.main()
