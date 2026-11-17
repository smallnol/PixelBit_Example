#!/usr/bin/env python3
"""
樹莓派 5｜MaUWB STM32 AOA 小車跟隨

流程：
  AOA Anchor（距離 cm + 角度）→ 本程式決策 → USB-TTL → UNO D10（F/L/R/Z/S/X）

預設只預覽狀態；按「開啟跟隨」才送馬達命令。
提供 /estop 給跌倒偵測（12_Fall_Detection）確認跌倒時呼叫急停。
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

from dotenv import load_dotenv
from flask import Flask, jsonify, render_template_string

from aoa_protocol import decide_aoa_command
from aoa_reader import AoaReader
from motor_serial import MotorCommander

BASE_DIR = Path(__file__).resolve().parent
load_dotenv(BASE_DIR / '.env')
APP_VERSION = 'aoa-follow-2026-03-17'

HOST = os.getenv('HOST', '0.0.0.0')
PORT = int(os.getenv('PORT', '5001'))

AOA_SERIAL_PORT = os.getenv('AOA_SERIAL_PORT', '/dev/ttyACM0').strip()
AOA_BAUD = int(os.getenv('AOA_BAUD', '115200'))
AOA_MODE = os.getenv('AOA_MODE', 'binary').strip().lower()
AOA_STALE_SECONDS = float(os.getenv('AOA_STALE_SECONDS', '1.0'))
AOA_ANGLE_DEADZONE_DEG = float(os.getenv('AOA_ANGLE_DEADZONE_DEG', '10'))
AOA_ANGLE_SIGN = float(os.getenv('AOA_ANGLE_SIGN', '1'))
AOA_LOST_CMD = os.getenv('AOA_LOST_CMD', 'Z').strip().upper()[:1] or 'Z'

MOTOR_SERIAL_PORT = os.getenv('MOTOR_SERIAL_PORT', '/dev/ttyUSB0').strip()
MOTOR_BAUD = int(os.getenv('MOTOR_BAUD', '9600'))
CMD_INTERVAL_SECONDS = float(os.getenv('CMD_INTERVAL_SECONDS', '0.15'))

START_DIST = float(os.getenv('START_DIST', '2.0'))
STOP_DIST = float(os.getenv('STOP_DIST', '1.8'))

app = Flask(__name__)
state_lock = threading.Lock()
is_following = False
fall_estop_latched = False  # 跌倒急停後需手動解除才繼續跟隨
latest_cmd = 'S'
latest_status = '待機'
was_forward = False
control_stop = threading.Event()

aoa = AoaReader(AOA_SERIAL_PORT, AOA_BAUD, AOA_MODE)
motor = MotorCommander(MOTOR_SERIAL_PORT, MOTOR_BAUD, CMD_INTERVAL_SECONDS)


def control_loop() -> None:
  global latest_cmd, latest_status, was_forward

  print('=== AOA CONTROL LOOP RUNNING ===', flush=True)
  while not control_stop.is_set():
    try:
      sample, age = aoa.snapshot()
      with state_lock:
        following = is_following and not fall_estop_latched
        prev_forward = was_forward
        latched = fall_estop_latched

      if latched:
        motor.send('X', force=True)
        with state_lock:
          latest_cmd = 'X'
          latest_status = 'FALL ESTOP latched — press 解除急停 / 開啟跟隨'
        time.sleep(0.2)
        continue

      cmd, status, next_forward = decide_aoa_command(
          following=following,
          sample=sample,
          age_s=age,
          stale_s=AOA_STALE_SECONDS,
          deadzone_deg=AOA_ANGLE_DEADZONE_DEG,
          start_dist=START_DIST,
          stop_dist=STOP_DIST,
          angle_sign=AOA_ANGLE_SIGN,
          prev_forward=prev_forward,
          lost_cmd=AOA_LOST_CMD,
      )

      if following:
        motor.send(cmd)
      else:
        motor.send('S')
        next_forward = False
        if not is_following:
          status = 'follow OFF'

      with state_lock:
        latest_cmd = cmd if following else 'S'
        latest_status = status
        was_forward = next_forward

      time.sleep(0.05)
    except Exception as e:
      print(f'控制迴圈例外: {e}')
      time.sleep(0.2)


INDEX_HTML = """
<!doctype html>
<html lang="zh-Hant">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>AOA 小車跟隨</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 0; background: #111; color: #eee; }
    main { max-width: 720px; margin: 0 auto; padding: 16px; }
    h1 { font-size: 1.25rem; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 12px; }
    button { border: 0; border-radius: 8px; padding: 10px 14px; font-size: 1rem; cursor: pointer; }
    .on { background: #1f8f4e; color: #fff; }
    .off { background: #a33; color: #fff; }
    .ghost { background: #333; color: #eee; }
    .warn { background: #b36b00; color: #fff; }
    #status {
      margin-top: 10px; padding: 12px; background: #1b1b1b; border-radius: 8px;
      font-family: ui-monospace, monospace; white-space: pre-wrap; line-height: 1.45;
    }
    .hint { opacity: .75; font-size: .9rem; }
  </style>
</head>
<body>
  <main>
    <h1>MaUWB AOA 跟隨（Pi）</h1>
    <div class="row">
      <button class="on" onclick="api('/start_follow')">開啟跟隨</button>
      <button class="off" onclick="api('/stop_follow')">停止跟隨</button>
      <button class="warn" onclick="api('/estop')">緊急停止 X</button>
      <button class="ghost" onclick="api('/clear_estop')">解除急停</button>
    </div>
    <div id="status">載入中…</div>
    <p class="hint">
      角度死區 ±{{ deadzone }}°｜前進 ≥ {{ start }}m｜停止 &lt; {{ stop }}m<br/>
      AOA: {{ aoa }} ({{ mode }})｜Motor: {{ motor }} @ {{ mbaud }}
    </p>
  </main>
  <script>
    async function api(path) {
      await fetch(path);
      refresh();
    }
    async function refresh() {
      const r = await fetch('/status');
      const j = await r.json();
      const ang = j.angle_deg == null ? '--' : j.angle_deg.toFixed(1) + '°';
      const dist = j.distance == null ? '--' : j.distance.toFixed(2) + ' m';
      document.getElementById('status').textContent =
        `跟隨: ${j.following ? 'ON' : 'OFF'}  急停鎖: ${j.fall_estop ? 'YES' : 'no'}\\n` +
        `命令: ${j.cmd}\\n` +
        `距離: ${dist}  (age ${j.distance_age.toFixed(2)}s)\\n` +
        `角度: ${ang}\\n` +
        `AOA port: ${j.aoa_ok ? 'OK' : 'FAIL'} ${j.aoa_error}\\n` +
        `Motor port: ${j.motor_ok ? 'OK' : 'FAIL'} ${j.motor_error}\\n` +
        `狀態: ${j.status}`;
    }
    refresh();
    setInterval(refresh, 400);
  </script>
</body>
</html>
"""


@app.route('/')
def index():
  return render_template_string(
      INDEX_HTML,
      deadzone=AOA_ANGLE_DEADZONE_DEG,
      start=START_DIST,
      stop=STOP_DIST,
      aoa=AOA_SERIAL_PORT,
      mode=AOA_MODE,
      motor=MOTOR_SERIAL_PORT,
      mbaud=MOTOR_BAUD,
  )


@app.route('/start_follow')
def start_follow():
  global is_following, fall_estop_latched
  with state_lock:
    fall_estop_latched = False
    is_following = True
  motor.send('S', force=True)
  print('▶️ 開啟 AOA 跟隨')
  return jsonify({'ok': True, 'following': True})


@app.route('/stop_follow')
def stop_follow():
  global is_following
  with state_lock:
    is_following = False
  motor.send('S', force=True)
  print('⏹️ 停止跟隨')
  return jsonify({'ok': True, 'following': False})


@app.route('/estop')
def estop():
  """跌倒偵測或其他模組呼叫：急停並鎖住跟隨。"""
  global is_following, fall_estop_latched
  with state_lock:
    is_following = False
    fall_estop_latched = True
  motor.estop()
  print('🛑 ESTOP latched')
  return jsonify({'ok': True, 'cmd': 'X', 'fall_estop': True})


@app.route('/clear_estop')
def clear_estop():
  global fall_estop_latched
  with state_lock:
    fall_estop_latched = False
  motor.send('S', force=True)
  print('✅ 急停鎖已解除')
  return jsonify({'ok': True, 'fall_estop': False})


@app.route('/status')
def status():
  sample, age = aoa.snapshot()
  with state_lock:
    return jsonify(
        {
            'following': is_following,
            'fall_estop': fall_estop_latched,
            'cmd': latest_cmd,
            'status': latest_status,
            'distance': None if sample is None else sample.distance_m,
            'distance_age': age,
            'angle_deg': None if sample is None else sample.angle_deg,
            'aoa_ok': aoa.ok,
            'aoa_error': aoa.last_error,
            'motor_ok': motor.ok,
            'motor_error': motor.last_error,
            'version': APP_VERSION,
        }
    )


if __name__ == '__main__':
  print(f'=== APP_VERSION={APP_VERSION} ===', flush=True)
  if AOA_SERIAL_PORT == MOTOR_SERIAL_PORT:
    print(
        '⚠️ AOA_SERIAL_PORT 與 MOTOR_SERIAL_PORT 相同：'
        'AOA 架構應分開（Anchor=ACM，馬達=USB-TTL）',
        flush=True,
    )
  aoa.start()
  motor.open()
  t = threading.Thread(target=control_loop, name='aoa_control', daemon=True)
  t.start()
  print(f'🌐 http://<樹莓派IP>:{PORT}', flush=True)
  try:
    app.run(host=HOST, port=PORT, threaded=True, use_reloader=False)
  finally:
    control_stop.set()
    motor.close()
    aoa.close()
