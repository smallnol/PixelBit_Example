#!/usr/bin/env python3
"""套件到貨後：在樹莓派上 sniff AOA Anchor 輸出，校對距離／角度。

用法：
  pip install pyserial python-dotenv
  cp .env.example .env   # 填 AOA_SERIAL_PORT
  python aoa_sniff.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

from dotenv import load_dotenv

from aoa_protocol import parse_json_line, try_parse_binary_frame

BASE = Path(__file__).resolve().parent
load_dotenv(BASE / '.env')

PORT = os.getenv('AOA_SERIAL_PORT', '/dev/ttyACM0')
BAUD = int(os.getenv('AOA_BAUD', '115200'))
MODE = os.getenv('AOA_MODE', 'binary').strip().lower()


def main() -> int:
  try:
    import serial
  except ImportError:
    print('需要 pyserial：pip install pyserial', file=sys.stderr)
    return 1

  print(f'開啟 {PORT} @ {BAUD} mode={MODE}')
  try:
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
  except Exception as e:
    print(f'無法開啟序列埠: {e}', file=sys.stderr)
    return 1

  buf = b''
  try:
    while True:
      chunk = ser.read(256)
      if not chunk:
        continue
      if MODE == 'json':
        for line in chunk.decode('utf-8', errors='ignore').splitlines():
          line = line.strip()
          if not line:
            continue
          sample = parse_json_line(line)
          if sample:
            print(
                f'JSON dist={sample.distance_m:.2f}m '
                f'angle={sample.angle_deg}'
            )
          else:
            print(f'RAW {line[:120]}')
      else:
        buf += chunk
        while True:
          sample, buf = try_parse_binary_frame(buf)
          if sample is None:
            break
          print(
              f'BIN sn={sample.sn} addr={sample.addr16} '
              f'dist={sample.distance_m:.2f}m angle={sample.angle_deg}'
          )
        # 緩衝過長則丟掉（避免壞資料卡死）
        if len(buf) > 4096:
          buf = buf[-512:]
      time.sleep(0.01)
  except KeyboardInterrupt:
    print('\n停止')
  finally:
    ser.close()
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
