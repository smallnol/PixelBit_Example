#!/usr/bin/env python3
"""把 MaUWB AOA Anchor 切到跟隨二進位輸出（USER_CMD 1）並 SAVE。

韌體命令格式（cmd.c）：`USER_CMD 1\\r\\n`、`SAVE\\r\\n`
- 0 = JSON（JSxxxx{"TWR":...}）
- 1 = Hex 跟隨幀（0x2A ... 0x23）

用法：
  python aoa_setup_usercmd.py
  python aoa_setup_usercmd.py --mode 0   # 改回 JSON
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import serial
from dotenv import load_dotenv

load_dotenv(Path(__file__).resolve().parent / '.env')


def main() -> int:
  p = argparse.ArgumentParser()
  p.add_argument('--port', default=os.getenv('AOA_SERIAL_PORT', '/dev/ttyACM0'))
  p.add_argument('--baud', type=int, default=int(os.getenv('AOA_BAUD', '115200')))
  p.add_argument('--mode', type=int, choices=(0, 1), default=1)
  args = p.parse_args()

  print(f'連線 {args.port} @ {args.baud}')
  ser = serial.Serial(args.port, args.baud, timeout=0.5)
  time.sleep(0.3)

  for cmd in (f'USER_CMD {args.mode}', 'SAVE', 'GETCFG'):
    line = cmd + '\r\n'
    print(f'>>> {cmd}')
    ser.write(line.encode('ascii'))
    ser.flush()
    time.sleep(0.4)
    resp = ser.read(512)
    if resp:
      print('<<<', resp.decode('utf-8', errors='replace').strip())

  ser.close()
  print('完成。請用 aoa_sniff.py 確認有距離／角度輸出。')
  print('提醒：Qt 需先綁定唯一 Tag；一 Anchor 同時只配一 Tag。')
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
