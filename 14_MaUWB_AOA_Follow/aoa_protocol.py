"""MaUWB STM32 AOA（Gen1）串列解析。

對齊 Makerfabs 韌體：
  Firmware/.../DW/PDoA/json_2pc.c
  - user_cmd==0 → JSON 文字列 JSxxxx{"TWR":...}
  - user_cmd==1 → 二進位跟隨幀 0x2A ... 0x23

到貨後請先用 aoa_sniff.py 對實機校對角度單位／左右符號。
"""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass


HEADER = 0x2A
FOOTER = 0x23
# Generic_cmd.h: MSG_HEX_OUTPUT_LEN (1+2+4+4+2+2+2+4+4) 再 +2 → length 欄位值
# format: sn(1) Addr(2) angle(4) distance(4) user_cmd(2)
#         F_Path(4) RX_Level(4) Acc_X(2) Acc_Y(2) Acc_Z(2) = 27 bytes payload after length
BINARY_PAYLOAD_LEN = 27

JS_TWR_RE = re.compile(
    r'JS[0-9A-Fa-f]{4}\{.*"D"\s*:\s*(-?\d+).*"Xcm"\s*:\s*(-?\d+)\s*,\s*"Ycm"\s*:\s*(-?\d+)',
    re.DOTALL,
)


@dataclass
class AoaSample:
  distance_m: float
  angle_deg: float | None
  addr16: int | None = None
  sn: int | None = None
  raw_mode: str = ''


def _xor_crc(data: bytes) -> int:
  x = 0
  for b in data:
    x ^= b
  return x & 0xFF


def try_parse_binary_frame(buf: bytes) -> tuple[AoaSample | None, bytes]:
  """從累積緩衝取出一幀；回傳 (sample|None, 剩餘緩衝)。"""
  while True:
    start = buf.find(bytes([HEADER]))
    if start < 0:
      return None, b''
    buf = buf[start:]
    if len(buf) < 2:
      return None, buf
    length = buf[1]
    # length = payload bytes after length field, before check+footer? 
    # App_Module_format_conver_uint8: check = XOR(buf, 2, msg.length); total = msg.length + 4
    # msg.length = MSG_HEX_OUTPUT_LEN + 2；payload starts at index 2
    frame_len = length + 4  # header, length, payload[length], check, footer
    if len(buf) < frame_len:
      return None, buf
    frame = buf[:frame_len]
    buf = buf[frame_len:]
    if frame[-1] != FOOTER:
      # 壞幀：跳過此 header 再找
      buf = frame[1:] + buf
      continue
    payload = frame[2 : 2 + length]
    check = frame[2 + length]
    if _xor_crc(payload) != check and _xor_crc(frame[2 : 2 + length]) != check:
      # 仍接受並嘗試解；若欄位對不齊 sniff 會看出來
      pass
    if length < BINARY_PAYLOAD_LEN:
      continue
    # little-endian ints per BREAK_UINT32 / LO/HI_UINT16
    sn = payload[0]
    addr16 = payload[1] | (payload[2] << 8)
    angle = struct.unpack_from('<i', payload, 3)[0]
    dist_cm = struct.unpack_from('<i', payload, 7)[0]
    if dist_cm <= 0 or dist_cm > 5000:
      continue
    return (
        AoaSample(
            distance_m=dist_cm / 100.0,
            angle_deg=float(angle),
            addr16=addr16,
            sn=sn,
            raw_mode='binary',
        ),
        buf,
    )


def parse_json_line(line: str) -> AoaSample | None:
  """解析一行 JS…{"TWR":…}；角度優先用 atan2(Ycm,Xcm)。"""
  import math

  m = JS_TWR_RE.search(line)
  if not m:
    # 寬鬆：只抓 D
    dm = re.search(r'"D"\s*:\s*(-?\d+)', line)
    if not dm:
      return None
    dist_cm = int(dm.group(1))
    if dist_cm <= 0:
      return None
    return AoaSample(distance_m=dist_cm / 100.0, angle_deg=None, raw_mode='json')

  dist_cm = int(m.group(1))
  x_cm = int(m.group(2))
  y_cm = int(m.group(3))
  # 車頭座標系需實測；暫定 angle = atan2(y, x) 度
  angle = math.degrees(math.atan2(y_cm, x_cm)) if (x_cm or y_cm) else 0.0
  return AoaSample(
      distance_m=dist_cm / 100.0,
      angle_deg=angle,
      raw_mode='json',
  )


def decide_aoa_command(
    *,
    following: bool,
    sample: AoaSample | None,
    age_s: float,
    stale_s: float,
    deadzone_deg: float,
    start_dist: float,
    stop_dist: float,
    angle_sign: float,
    prev_forward: bool,
    lost_cmd: str = 'Z',
) -> tuple[str, str, bool]:
  """回傳 (cmd, status, next_was_forward)。與 follow_car.decide_command 對齊。"""
  if not following:
    return 'S', 'follow OFF', False

  if sample is None or age_s > stale_s:
    return lost_cmd, f'AOA stale/missing -> {lost_cmd}', False

  if sample.angle_deg is None:
    return 'S', 'AOA dist only, no angle -> stop', False

  angle = sample.angle_deg * angle_sign
  dist = sample.distance_m

  if angle < -deadzone_deg:
    return 'L', f'turn LEFT ({angle:+.1f}deg)', prev_forward
  if angle > deadzone_deg:
    return 'R', f'turn RIGHT ({angle:+.1f}deg)', prev_forward

  if dist >= start_dist:
    return 'F', f'centered {dist:.2f}m -> forward', True
  if dist < stop_dist:
    return 'S', f'centered {dist:.2f}m -> stop', False
  if prev_forward:
    return 'F', f'band {dist:.2f}m -> keep F', True
  return 'S', f'band {dist:.2f}m -> keep S', False
