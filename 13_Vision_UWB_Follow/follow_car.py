"""
樹莓派 5｜視覺 + UWB 跟隨小車

控制邏輯（Pi 當大腦）：
1. YOLO-pose 找人體 → 用 bbox 中心判斷在畫面左／中／右
2. 車上 ESP32-UWB 報距離（Serial 行：D:1.85）
3. 綜合決策後對 UNO 下馬達命令：
   - 找不到人 → Z（自轉搜尋）
   - 人在左側 → L（左轉直到置中）
   - 人在右側 → R（右轉直到置中）
   - 人在中央且距離 ≥ START_DIST → F（前進）
   - 人在中央且距離 < STOP_DIST → S（停止）
   - 距離失效／逾時 → S（安全停止；搜尋中仍可自轉）

預設只開預覽；按「開啟跟隨」才會送馬達命令。
"""

from __future__ import annotations

import os
import re
import threading
import time
from pathlib import Path

import cv2
import serial
from dotenv import load_dotenv
from flask import Flask, Response, jsonify, render_template_string
from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
load_dotenv(BASE_DIR / '.env')
APP_VERSION = 'follow-2026-03-16-ctrl-loop'

# --- 硬體 / 網路 ---
CAMERA_INDEX = int(os.getenv('CAMERA_INDEX', '0'))
HOST = os.getenv('HOST', '0.0.0.0')
PORT = int(os.getenv('PORT', '5001'))

# 車上 ESP32 USB（距離 + 馬達命令轉發），例如 /dev/ttyUSB0
UWB_SERIAL_PORT = os.getenv('UWB_SERIAL_PORT', '/dev/ttyUSB0').strip()
UWB_BAUD = int(os.getenv('UWB_BAUD', '115200'))

# 馬達命令出口：
# - 建議（無 USB-TTL）：填與 UWB 相同，例如 /dev/ttyUSB0
#   → Pi 經 USB 把 F/L/R/Z 寫給車上 ESP32，再轉 Serial2 → UNO
# - 或填 uwb / same 代表共用 UWB 口
# - 進階：另開 /dev/ttyAMA0 或 USB-TTL
_motor_port_raw = os.getenv('MOTOR_SERIAL_PORT', 'uwb').strip()
# Pi 5 的 serial0/ttyAMA0 常不是接到 UNO；無 USB-TTL 時一律走 UWB USB 轉發
_pi5_trap = {'/dev/serial0', '/dev/ttyama0', '/dev/ttyama10', 'serial0', 'ttyama0'}
if _motor_port_raw.lower() in {'', 'uwb', 'same', 'auto'} or _motor_port_raw.lower() in _pi5_trap:
  if _motor_port_raw and _motor_port_raw.lower() not in {'', 'uwb', 'same', 'auto'}:
    print(
        f'⚠️ MOTOR_SERIAL_PORT={_motor_port_raw} 在無 USB-TTL 架構下不可靠，'
        f'改走 UWB USB 轉發 ({UWB_SERIAL_PORT})'
    )
  MOTOR_SERIAL_PORT = UWB_SERIAL_PORT
else:
  MOTOR_SERIAL_PORT = _motor_port_raw
MOTOR_BAUD = int(
    os.getenv(
        'MOTOR_BAUD',
        str(UWB_BAUD if MOTOR_SERIAL_PORT == UWB_SERIAL_PORT else 9600),
    )
)
MOTOR_VIA_UWB = MOTOR_SERIAL_PORT == UWB_SERIAL_PORT

# --- 跟隨參數 ---
START_DIST = float(os.getenv('START_DIST', '2.0'))
STOP_DIST = float(os.getenv('STOP_DIST', '1.8'))
# 畫面中央死區：±比例內視為「置中」（0.12 = 左右各 12%）
CENTER_DEADZONE = float(os.getenv('CENTER_DEADZONE', '0.12'))
UWB_STALE_SECONDS = float(os.getenv('UWB_STALE_SECONDS', '1.0'))
CMD_INTERVAL_SECONDS = float(os.getenv('CMD_INTERVAL_SECONDS', '0.15'))
FRAME_WIDTH = int(os.getenv('FRAME_WIDTH', '640'))
FRAME_HEIGHT = int(os.getenv('FRAME_HEIGHT', '480'))
YOLO_IMGSZ = int(os.getenv('YOLO_IMGSZ', '320'))

DIST_LINE_RE = re.compile(r'D:\s*([0-9]+(?:\.[0-9]+)?)', re.IGNORECASE)
# 相容舊格式 [CAR] 1.85 m
DIST_LEGACY_RE = re.compile(r'\[CAR\]\s*([0-9]+(?:\.[0-9]+)?)\s*m', re.IGNORECASE)

app = Flask(__name__)

state_lock = threading.Lock()
is_following = False
latest_distance: float | None = None
last_distance_time = 0.0
latest_cmd = 'S'
latest_status = '待機'
person_offset = 0.0  # -1 左 … 0 中 … +1 右
person_seen = False
was_forward = False  # 1.8～2.0m 緩衝帶用：維持上一前進／停止
latest_jpeg: bytes | None = None
jpeg_lock = threading.Lock()
control_stop = threading.Event()

print('正在載入 YOLO-pose...')
model = YOLO('yolov8n-pose.pt')
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


class SerialReader:
  """背景讀取 UWB 距離行；可與馬達命令共用同一 USB 口。"""

  def __init__(self, port: str, baud: int):
    self.port = port
    self.baud = baud
    self._ser: serial.Serial | None = None
    self._io_lock = threading.Lock()
    self._stop = threading.Event()
    self._thread: threading.Thread | None = None
    self.ok = False
    self.last_error = ''

  def start(self) -> None:
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def close(self) -> None:
    self._stop.set()
    with self._io_lock:
      if self._ser and self._ser.is_open:
        try:
          self._ser.close()
        except Exception:
          pass

  def _open(self) -> bool:
    try:
      self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
      self.ok = True
      self.last_error = ''
      print(f'✅ UWB Serial 已開: {self.port} @ {self.baud}')
      return True
    except Exception as e:
      self.ok = False
      self.last_error = str(e)
      print(f'⚠️ 無法開啟 UWB Serial ({self.port}): {e}')
      return False

  def write_cmd(self, cmd: str) -> bool:
    """寫入單字母馬達命令（供共用 USB 轉發）。"""
    payload = f'{cmd}\n'.encode('ascii')
    with self._io_lock:
      if self._ser is None or not self._ser.is_open:
        if not self._open():
          return False
      try:
        self._ser.write(payload)
        self._ser.flush()
        return True
      except Exception as e:
        self.ok = False
        self.last_error = str(e)
        print(f'⚠️ UWB Serial 寫入命令失敗: {e}')
        try:
          if self._ser:
            self._ser.close()
        except Exception:
          pass
        self._ser = None
        return False

  def _run(self) -> None:
    global latest_distance, last_distance_time
    while not self._stop.is_set():
      with self._io_lock:
        need_open = self._ser is None or not self._ser.is_open
      if need_open:
        with self._io_lock:
          if not self._open():
            pass
        if not self.ok:
          time.sleep(2.0)
          continue
      try:
        with self._io_lock:
          raw = self._ser.readline() if self._ser else b''
        if not raw:
          continue
        line = raw.decode('utf-8', errors='ignore').strip()
        if not line:
          continue
        # ESP32 轉發成功時會印 FWD->UNO X；顯示在 Pi 終端方便除錯
        if 'FWD' in line.upper() or line.upper().startswith('CMD'):
          print(f'📥 ESP32: {line}', flush=True)
        m = DIST_LINE_RE.search(line) or DIST_LEGACY_RE.search(line)
        if not m:
          continue
        dist = float(m.group(1))
        if 0.05 < dist < 40.0:
          with state_lock:
            latest_distance = dist
            last_distance_time = time.time()
      except Exception as e:
        self.ok = False
        self.last_error = str(e)
        print(f'⚠️ UWB Serial 讀取錯誤: {e}')
        with self._io_lock:
          try:
            if self._ser:
              self._ser.close()
          except Exception:
            pass
          self._ser = None
        time.sleep(1.0)


class MotorCommander:
  """送馬達命令：可走獨立口，或共用 UWB USB（經 ESP32 轉 UNO）。"""

  def __init__(self, port: str, baud: int, shared_uwb: SerialReader | None = None):
    self.port = port
    self.baud = baud
    self._shared_uwb = shared_uwb
    self._ser: serial.Serial | None = None
    self.ok = False
    self.last_error = ''
    self._last_sent = ''
    self._last_sent_at = 0.0
    self.via_uwb = shared_uwb is not None

  def open(self) -> bool:
    if self._shared_uwb is not None:
      # 等 UWB 口開啟；以寫入測試視為就緒
      ok = self._shared_uwb.write_cmd('S')
      self.ok = ok
      self.last_error = '' if ok else self._shared_uwb.last_error
      if ok:
        print(f'✅ Motor 經 UWB USB 轉發: {self.port} @ {self.baud}')
      else:
        print(f'⚠️ Motor 經 UWB 轉發尚未就緒: {self.last_error}')
      return ok
    try:
      self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
      self.ok = True
      self.last_error = ''
      print(f'✅ Motor Serial 已開: {self.port} @ {self.baud}')
      self.send('S', force=True)
      return True
    except Exception as e:
      self.ok = False
      self.last_error = str(e)
      print(f'⚠️ 無法開啟 Motor Serial ({self.port}): {e}')
      return False

  def close(self) -> None:
    try:
      self.send('X', force=True)
    except Exception:
      pass
    if self._shared_uwb is not None:
      return
    if self._ser and self._ser.is_open:
      try:
        self._ser.close()
      except Exception:
        pass

  def send(self, cmd: str, force: bool = False) -> None:
    cmd = (cmd or 'S').upper()[:1]
    now = time.time()
    if (
        not force
        and cmd == self._last_sent
        and (now - self._last_sent_at) < CMD_INTERVAL_SECONDS
    ):
      return

    if self._shared_uwb is not None:
      if self._shared_uwb.write_cmd(cmd):
        # 只在命令變化時印，避免刷屏
        if cmd != self._last_sent:
          print(f'📤 經 USB 送馬達命令 -> ESP32: {cmd}')
        self._last_sent = cmd
        self._last_sent_at = now
        self.ok = True
        self.last_error = ''
      else:
        self.ok = False
        self.last_error = self._shared_uwb.last_error
      return

    if self._ser is None or not self._ser.is_open:
      if not self.open():
        return
    try:
      self._ser.write(f'{cmd}\n'.encode('ascii'))
      self._ser.flush()
      self._last_sent = cmd
      self._last_sent_at = now
      self.ok = True
    except Exception as e:
      self.ok = False
      self.last_error = str(e)
      print(f'⚠️ 送馬達命令失敗: {e}')
      try:
        if self._ser:
          self._ser.close()
      except Exception:
        pass
      self._ser = None


uwb_reader = SerialReader(UWB_SERIAL_PORT, UWB_BAUD)
motor = MotorCommander(
    MOTOR_SERIAL_PORT,
    MOTOR_BAUD,
    shared_uwb=uwb_reader if MOTOR_VIA_UWB else None,
)
print(
    f'🔧 馬達出口: {MOTOR_SERIAL_PORT}'
    + ('（經車上 ESP32 USB→Serial2→UNO）' if MOTOR_VIA_UWB else '（獨立序列埠）')
)


def decide_command(
    has_person: bool,
    offset: float,
    distance: float | None,
    dist_age: float,
    following: bool,
    prev_forward: bool,
) -> tuple[str, str, bool]:
  """回傳 (cmd, status_text, next_was_forward)。

  status_text 用英文（OpenCV 畫面字型不支援中文，避免 ??? 重影）。
  """
  if not following:
    return 'S', 'follow OFF (preview)', False

  dist_ok = distance is not None and dist_age <= UWB_STALE_SECONDS

  if not has_person:
    # 找不到人：自轉搜尋（不依賴距離）；結束前進狀態
    return 'Z', 'search: no person -> spin', False

  if offset < -CENTER_DEADZONE:
    return 'L', f'turn LEFT (offset={offset:+.2f})', prev_forward
  if offset > CENTER_DEADZONE:
    return 'R', f'turn RIGHT (offset={offset:+.2f})', prev_forward

  # 已置中 → 用 UWB 決定前進／停止
  if not dist_ok:
    return 'S', 'centered but UWB stale -> stop', False

  assert distance is not None
  if distance >= START_DIST:
    return 'F', f'centered dist {distance:.2f}m -> forward', True
  if distance < STOP_DIST:
    return 'S', f'centered dist {distance:.2f}m -> stop', False
  # 1.8～2.0：維持上一前進／停止（與舊 UWB 直線跟隨相同）
  if prev_forward:
    return 'F', f'centered band {distance:.2f}m -> keep F', True
  return 'S', f'centered band {distance:.2f}m -> keep S', False


def person_from_result(result) -> tuple[bool, float, tuple[int, int, int, int] | None]:
  """回傳 (seen, offset[-1..1], bbox xyxy int)。"""
  if result.boxes is None or len(result.boxes) == 0:
    return False, 0.0, None

  # 取信心最高的一個人
  boxes = result.boxes
  conf = boxes.conf
  best_i = int(conf.argmax().item()) if conf is not None and len(conf) else 0
  xyxy = boxes.xyxy[best_i].tolist()
  x1, y1, x2, y2 = xyxy
  cx = (x1 + x2) / 2.0
  frame_w = float(FRAME_WIDTH)
  # -1 = 最左, 0 = 正中, +1 = 最右
  offset = (cx / max(frame_w, 1.0)) * 2.0 - 1.0
  bbox = (int(x1), int(y1), int(x2), int(y2))
  return True, offset, bbox


def draw_hud(
    frame,
    has_person: bool,
    offset: float,
    bbox: tuple[int, int, int, int] | None,
    distance: float | None,
    dist_age: float,
    cmd: str,
    status: str,
    following: bool,
):
  h, w = frame.shape[:2]
  cx = w // 2
  # CENTER_DEADZONE = |offset| 閾值；offset ∈ [-1,1] → 像素半寬 = deadzone * (w/2)
  half = int(CENTER_DEADZONE * (w / 2))

  # 中央死區
  cv2.line(frame, (cx - half, 0), (cx - half, h), (80, 80, 80), 1)
  cv2.line(frame, (cx + half, 0), (cx + half, h), (80, 80, 80), 1)
  cv2.line(frame, (cx, 0), (cx, h), (0, 255, 255), 1)

  if has_person and bbox is not None:
    x1, y1, x2, y2 = bbox
    color = (0, 255, 0) if abs(offset) <= CENTER_DEADZONE else (0, 165, 255)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    pcx = (x1 + x2) // 2
    cv2.circle(frame, (pcx, (y1 + y2) // 2), 5, color, -1)

  dist_txt = 'UWB: --'
  if distance is not None:
    stale = ' (stale)' if dist_age > UWB_STALE_SECONDS else ''
    dist_txt = f'UWB: {distance:.2f}m{stale}'

  lines = [
      f'FOLLOW: {"ON" if following else "OFF"}  CMD: {cmd}',
      dist_txt,
      f'offset: {offset:+.2f}' if has_person else 'person: none',
      status[:52],
  ]
  # 單一底色條 + 單層白字，避免黑白雙描邊造成「重影」
  bar_h = 18 + 20 * len(lines)
  overlay = frame.copy()
  cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
  cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
  y = 22
  for line in lines:
    cv2.putText(
        frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA
    )
    y += 20
  return frame


def control_loop() -> None:
  """獨立控制迴圈：不依賴瀏覽器有沒有在看影像，都會送馬達命令。"""
  global latest_cmd, latest_status, person_offset, person_seen, was_forward, latest_jpeg

  print('=== CONTROL LOOP THREAD ENTER ===', flush=True)
  if not cap.isOpened():
    print('=== CAMERA NOT OPEN — control loop exit ===', flush=True)
    return

  print('=== CONTROL LOOP RUNNING (motor cmds independent of video_feed) ===', flush=True)
  while not control_stop.is_set():
    ok, frame = cap.read()
    if not ok:
      time.sleep(0.05)
      continue

    try:
      if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

      results = model.predict(source=frame, imgsz=YOLO_IMGSZ, conf=0.5, verbose=False)
      annotated = results[0].plot(labels=False, conf=False)
      has_person, offset, bbox = person_from_result(results[0])

      with state_lock:
        following = is_following
        distance = latest_distance
        dist_age = time.time() - last_distance_time if last_distance_time else 999.0
        prev_forward = was_forward

      cmd, status, next_forward = decide_command(
          has_person, offset, distance, dist_age, following, prev_forward
      )

      if following:
        motor.send(cmd)
      else:
        motor.send('S')
        next_forward = False

      with state_lock:
        latest_cmd = cmd
        latest_status = status
        person_offset = offset
        person_seen = has_person
        was_forward = next_forward

      hud = draw_hud(
          annotated,
          has_person,
          offset,
          bbox,
          distance,
          dist_age,
          cmd,
          status,
          following,
      )
      ret, jpg = cv2.imencode('.jpg', hud)
      if ret:
        with jpeg_lock:
          latest_jpeg = jpg.tobytes()
    except Exception as e:
      print(f'控制迴圈例外: {e}')
      time.sleep(0.1)


def generate_frames():
  """只負責把控制迴圈產出的畫面推給瀏覽器。"""
  while not control_stop.is_set():
    with jpeg_lock:
      frame = latest_jpeg
    if frame is None:
      time.sleep(0.05)
      continue
    yield (
        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
    )
    time.sleep(0.05)


INDEX_HTML = """
<!doctype html>
<html lang="zh-Hant">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>視覺 + UWB 跟隨</title>
  <style>
    :root { color-scheme: light; }
    body { font-family: system-ui, sans-serif; margin: 0; background: #111; color: #eee; }
    main { max-width: 960px; margin: 0 auto; padding: 16px; }
    h1 { font-size: 1.25rem; margin: 0 0 12px; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 12px; }
    button {
      border: 0; border-radius: 8px; padding: 10px 14px; font-size: 1rem; cursor: pointer;
    }
    .on { background: #1f8f4e; color: #fff; }
    .off { background: #a33; color: #fff; }
    .ghost { background: #333; color: #eee; }
    img { width: 100%; background: #000; border-radius: 8px; }
    #status {
      margin-top: 10px; padding: 10px 12px; background: #1b1b1b; border-radius: 8px;
      font-family: ui-monospace, monospace; white-space: pre-wrap; line-height: 1.45;
    }
    .hint { opacity: .75; font-size: .9rem; margin-top: 8px; }
  </style>
</head>
<body>
  <main>
    <h1>視覺 + UWB 跟隨小車</h1>
    <div class="row">
      <button class="on" onclick="api('/start_follow')">開啟跟隨</button>
      <button class="off" onclick="api('/stop_follow')">停止跟隨</button>
      <button class="ghost" onclick="refresh()">重新整理狀態</button>
    </div>
    <img src="/video_feed" alt="camera"/>
    <div id="status">載入中…</div>
    <p class="hint">
      置中死區 ±{{ deadzone }}｜前進 ≥ {{ start }}m｜停止 &lt; {{ stop }}m<br/>
      UWB: {{ uwb }}｜Motor: {{ motor }}
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
      document.getElementById('status').textContent =
        `跟隨: ${j.following ? 'ON' : 'OFF'}\\n` +
        `命令: ${j.cmd}\\n` +
        `距離: ${j.distance == null ? '--' : j.distance.toFixed(2) + ' m'} (age ${j.distance_age.toFixed(2)}s)\\n` +
        `人體: ${j.person_seen ? 'yes' : 'no'}  offset=${j.offset.toFixed(2)}\\n` +
        `UWB port: ${j.uwb_ok ? 'OK' : 'FAIL'} ${j.uwb_error}\\n` +
        `Motor port: ${j.motor_ok ? 'OK' : 'FAIL'} ${j.motor_error}\\n` +
        `狀態: ${j.status}`;
    }
    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>
"""


@app.route('/')
def index():
  return render_template_string(
      INDEX_HTML,
      deadzone=CENTER_DEADZONE,
      start=START_DIST,
      stop=STOP_DIST,
      uwb=UWB_SERIAL_PORT,
      motor=MOTOR_SERIAL_PORT,
  )


@app.route('/video_feed')
def video_feed():
  return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/start_follow')
def start_follow():
  global is_following
  with state_lock:
    is_following = True
  # 立刻送幾次，方便 UNO Monitor 確認有收到
  for _ in range(5):
    motor.send('Z', force=True)
    time.sleep(0.05)
  print(
      f'▶️ 開啟跟隨｜馬達出口={MOTOR_SERIAL_PORT}'
      f'｜via_uwb={MOTOR_VIA_UWB}｜已送測試 Z x5'
  )
  return jsonify(
      {
          'ok': True,
          'following': True,
          'motor_port': MOTOR_SERIAL_PORT,
          'motor_via_uwb': MOTOR_VIA_UWB,
      }
  )


@app.route('/stop_follow')
def stop_follow():
  global is_following
  with state_lock:
    is_following = False
  motor.send('S', force=True)
  print('⏹️ 停止跟隨')
  return jsonify({'ok': True, 'following': False})


@app.route('/status')
def status():
  with state_lock:
    dist = latest_distance
    age = time.time() - last_distance_time if last_distance_time else 999.0
    return jsonify(
        {
            'following': is_following,
            'cmd': latest_cmd,
            'status': latest_status,
            'distance': dist,
            'distance_age': age,
            'person_seen': person_seen,
            'offset': person_offset,
            'uwb_ok': uwb_reader.ok,
            'uwb_error': uwb_reader.last_error,
            'motor_ok': motor.ok,
            'motor_error': motor.last_error,
            'motor_via_uwb': motor.via_uwb,
            'motor_port': MOTOR_SERIAL_PORT,
        }
    )


if __name__ == '__main__':
  print(f'=== APP_VERSION={APP_VERSION} ===', flush=True)
  print(f'=== FILE={Path(__file__).resolve()} ===', flush=True)
  uwb_reader.start()
  motor.open()
  print('=== STARTING CONTROL THREAD ===', flush=True)
  control_thread = threading.Thread(target=control_loop, name='control_loop', daemon=True)
  control_thread.start()
  time.sleep(0.5)
  print(f'=== CONTROL THREAD ALIVE={control_thread.is_alive()} ===', flush=True)
  print(f'🌐 開啟 http://<樹莓派IP>:{PORT}', flush=True)
  try:
    app.run(host=HOST, port=PORT, threaded=True, use_reloader=False)
  finally:
    control_stop.set()
    motor.close()
    uwb_reader.close()
    cap.release()
