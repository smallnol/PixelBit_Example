"""
在樹莓派上測試：經車上 ESP32 USB 轉發馬達命令到 UNO。

用法：
  python test_motor_relay.py
  python test_motor_relay.py /dev/ttyUSB0

預期：
  - 終端機會印「已送 Z/F/L/...」
  - UNO 用 USB 接電腦開 Serial Monitor(115200) 應出現 CMD Z / CMD F ...
  - （可選）車上 ESP32 若暫時插電腦測，應出現 FWD->UNO Z
"""

from __future__ import annotations

import os
import sys
import time

import serial
from dotenv import load_dotenv
from pathlib import Path

BASE = Path(__file__).resolve().parent
load_dotenv(BASE / '.env')

port = (sys.argv[1] if len(sys.argv) > 1 else os.getenv('UWB_SERIAL_PORT', '/dev/ttyUSB0')).strip()
baud = int(os.getenv('UWB_BAUD', '115200'))

print(f'開啟 {port} @ {baud}')
ser = serial.Serial(port, baud, timeout=0.3)
time.sleep(0.5)

# 清掉開機緩衝
ser.reset_input_buffer()

cmds = ['S', 'Z', 'Z', 'Z', 'Z', 'Z', 'L', 'L', 'L', 'R', 'R', 'R', 'F', 'F', 'F', 'S']
for c in cmds:
  ser.write(f'{c}\n'.encode('ascii'))
  ser.flush()
  print(f'已送: {c}')
  # 同時印出 ESP32 回傳（距離或 FWD->UNO）
  t0 = time.time()
  while time.time() - t0 < 0.35:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
      print(f'  << {line}')
  time.sleep(0.15)

ser.write(b'S\n')
ser.close()
print('結束。請看 UNO Serial Monitor 有無 CMD。')
