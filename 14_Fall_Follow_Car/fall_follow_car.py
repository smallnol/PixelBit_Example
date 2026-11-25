"""
樹莓派 5｜跌倒偵測 + 視覺跟隨小車（整合版）

功能：
1. YOLO-pose + ByteTrack：鎖定特定人跟隨／跌倒（路人不觸發）
2. ESP32-UWB 距離 + 4×VL53L0X 避障遙測（O:）
3. 確認跌倒 → 緊急煞停 X + LINE
4. 網頁：跟隨／跌倒開關、鎖定目標、手動遙控、感測儀表板
"""

from __future__ import annotations

import json
import os
import re
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import serial
from dotenv import load_dotenv
from flask import Flask, Response, jsonify, render_template_string, request, send_file
from ultralytics import YOLO

BASE_DIR = Path(__file__).resolve().parent
load_dotenv(BASE_DIR / '.env')
import line_media as fall  # noqa: E402

APP_VERSION = 'fall-follow-2026-11-25-nocam-telemetry'

# --- 硬體 / 網路 ---
CAMERA_INDEX = int(os.getenv('CAMERA_INDEX', '0'))
HOST = os.getenv('HOST', '0.0.0.0')
PORT = int(os.getenv('PORT', '5001'))

UWB_SERIAL_PORT = os.getenv('UWB_SERIAL_PORT', '/dev/ttyUSB0').strip()
UWB_BAUD = int(os.getenv('UWB_BAUD', '115200'))

_motor_port_raw = os.getenv('MOTOR_SERIAL_PORT', 'uwb').strip()
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
CENTER_DEADZONE = float(os.getenv('CENTER_DEADZONE', '0.12'))
UWB_STALE_SECONDS = float(os.getenv('UWB_STALE_SECONDS', '1.0'))
TOF_STALE_SECONDS = float(os.getenv('TOF_STALE_SECONDS', '1.0'))
CMD_INTERVAL_SECONDS = float(os.getenv('CMD_INTERVAL_SECONDS', '0.15'))
FRAME_WIDTH = int(os.getenv('FRAME_WIDTH', str(fall.FRAME_WIDTH)))
FRAME_HEIGHT = int(os.getenv('FRAME_HEIGHT', str(fall.FRAME_HEIGHT)))
YOLO_IMGSZ = int(os.getenv('YOLO_IMGSZ', '320'))
STOP_FOLLOW_ON_FALL = os.getenv('STOP_FOLLOW_ON_FALL', '1').strip() not in {'0', 'false', 'False'}
ENABLE_GESTURE_LOCK = os.getenv('ENABLE_GESTURE_LOCK', '1').strip() not in {'0', 'false', 'False'}
GESTURE_LOCK_SECONDS = float(os.getenv('GESTURE_LOCK_SECONDS', '2.0'))
HSV_REID_THRESHOLD = float(os.getenv('HSV_REID_THRESHOLD', '0.60'))
# 目標遺失搜尋中提高 Re-ID 門檻，降低誤跟路人
HSV_REID_SEARCH = float(os.getenv('HSV_REID_SEARCH', '0.72'))
AVOID_MM = int(os.getenv('AVOID_MM', '250'))
SIDE_AVOID_MM = int(os.getenv('SIDE_AVOID_MM', '220'))
TOF_OPEN_MARGIN = int(os.getenv('TOF_OPEN_MARGIN', '30'))
LOST_GRACE_SECONDS = float(os.getenv('LOST_GRACE_SECONDS', '1.0'))
SEARCH_BIAS_SECONDS = float(os.getenv('SEARCH_BIAS_SECONDS', '3.0'))
Z_TIMEOUT_SECONDS = float(os.getenv('Z_TIMEOUT_SECONDS', '12.0'))
STUCK_SECONDS = float(os.getenv('STUCK_SECONDS', '4.0'))
STUCK_DIST_EPS = float(os.getenv('STUCK_DIST_EPS', '0.15'))
CORRIDOR_NUDGE_MM = int(os.getenv('CORRIDOR_NUDGE_MM', '400'))
CORRIDOR_BIAS_MM = int(os.getenv('CORRIDOR_BIAS_MM', '80'))
TRACKER_CFG = os.getenv('TRACKER_CFG', 'bytetrack.yaml').strip() or 'bytetrack.yaml'

DIST_LINE_RE = re.compile(r'D:\s*([0-9]+(?:\.[0-9]+)?)', re.IGNORECASE)
DIST_LEGACY_RE = re.compile(r'\[CAR\]\s*([0-9]+(?:\.[0-9]+)?)\s*m', re.IGNORECASE)
# O:左前,正前,右前,正後
TOF_LINE_RE = re.compile(
    r'O:\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)',
    re.IGNORECASE,
)
TOF_LABELS = ('左前', '正前', '右前', '正後')

app = Flask(__name__)

state_lock = threading.Lock()
is_following = False
is_detecting = False
fall_start_time: float | None = None
last_alert_time = 0.0
active_event = None
fall_status = 'fall OFF'
fall_suspect_elapsed = 0.0
fall_alert_active = False
fall_alert_ts = ''
latest_distance: float | None = None
last_distance_time = 0.0
latest_tof_mm = [8190, 8190, 8190, 8190]
last_tof_time = 0.0
latest_cmd = 'S'
latest_status = '待機'
person_offset = 0.0
person_seen = False
was_forward = False
latest_jpeg: bytes | None = None
jpeg_lock = threading.Lock()
control_stop = threading.Event()
last_buffer_time = 0.0

# --- 特定人鎖定 ---
locked_target_id: int | None = None
locked_target_hist: np.ndarray | None = None
lock_request_center = False
gesture_raise_since: dict[int, float] = {}
manual_cmd: str | None = None
manual_cmd_until = 0.0

# --- 跟隨復原／避障狀態（對應走道、遮擋、轉角等情境）---
last_seen_offset = 0.0
target_lost_since: float | None = None
search_spin_since: float | None = None
search_exhausted = False
stuck_forward_since: float | None = None
stuck_dist_ref: float | None = None
follow_recovery_note = ''

print('正在載入 YOLO-pose...')
model = YOLO('yolov8n-pose.pt')
cap = cv2.VideoCapture(CAMERA_INDEX)
if cap.isOpened():
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
  # 縮短取像等待，避免沒鏡頭／壞裝置時卡住整支程式
  try:
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
  except Exception:
    pass
  print(f'✅ 相機已開啟 index={CAMERA_INDEX}', flush=True)
else:
  print(
      f'⚠️ 相機無法開啟 index={CAMERA_INDEX}：'
      f'網頁仍顯示 UWB／ToF；跟隨／跌倒需接回鏡頭',
      flush=True,
  )


def _placeholder_frame(msg: str = 'NO CAMERA') -> np.ndarray:
  frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
  cv2.putText(
      frame, msg, (20, FRAME_HEIGHT // 2),
      cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2, cv2.LINE_AA,
  )
  with state_lock:
    dist = latest_distance
    tof = list(latest_tof_mm)
    cmd = latest_cmd
  y = 40
  cv2.putText(
      frame, f'UWB: {dist if dist is not None else "--"} m  CMD: {cmd}',
      (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
  )
  y += 28
  cv2.putText(
      frame, 'ToF: ' + ','.join(str(v) for v in tof),
      (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1, cv2.LINE_AA,
  )
  return frame


def _publish_jpeg(frame: np.ndarray) -> None:
  global latest_jpeg
  ok_jpg, jpg_buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
  if ok_jpg:
    with jpeg_lock:
      latest_jpeg = jpg_buf.tobytes()


class SerialReader:
  """背景讀取 UWB 距離與 VL53 O:；可與馬達命令共用同一 USB 口。"""

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
    global latest_distance, last_distance_time, latest_tof_mm, last_tof_time
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
        if 'FWD' in line.upper() or line.upper().startswith('CMD'):
          print(f'📥 ESP32: {line}', flush=True)

        m_tof = TOF_LINE_RE.search(line)
        if m_tof:
          vals = [int(m_tof.group(i)) for i in range(1, 5)]
          with state_lock:
            latest_tof_mm = vals
            last_tof_time = time.time()
          if not hasattr(self, '_tof_log_n'):
            self._tof_log_n = 0
          self._tof_log_n += 1
          if self._tof_log_n <= 5 or self._tof_log_n % 25 == 0:
            print(f'📥 O: {vals}', flush=True)
          continue

        m = DIST_LINE_RE.search(line) or DIST_LEGACY_RE.search(line)
        if not m:
          continue
        dist = float(m.group(1))
        if 0.05 < dist < 40.0:
          with state_lock:
            latest_distance = dist
            last_distance_time = time.time()
          if not hasattr(self, '_dist_log_n'):
            self._dist_log_n = 0
          self._dist_log_n += 1
          if self._dist_log_n <= 5 or self._dist_log_n % 25 == 0:
            print(f'📥 D: {dist:.2f}', flush=True)
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


def upper_hsv_hist(frame: np.ndarray, bbox: tuple[int, int, int, int]) -> np.ndarray | None:
  x1, y1, x2, y2 = bbox
  h = max(1, y2 - y1)
  w = max(1, x2 - x1)
  uy2 = y1 + max(1, h // 2)
  x1c, y1c = max(0, x1), max(0, y1)
  x2c, y2c = min(frame.shape[1], x2), min(frame.shape[0], uy2)
  if x2c <= x1c or y2c <= y1c:
    return None
  roi = frame[y1c:y2c, x1c:x2c]
  if roi.size < 50:
    return None
  hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
  hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
  cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
  return hist


def hist_similarity(a: np.ndarray | None, b: np.ndarray | None) -> float:
  if a is None or b is None:
    return -1.0
  return float(cv2.compareHist(a, b, cv2.HISTCMP_CORREL))


def extract_tracks(result, frame: np.ndarray) -> list[dict]:
  """解析 YOLO track 結果為人物列表。"""
  people: list[dict] = []
  if result.boxes is None or len(result.boxes) == 0:
    return people
  boxes = result.boxes
  n = len(boxes)
  ids = boxes.id
  for i in range(n):
    xyxy = boxes.xyxy[i].tolist()
    x1, y1, x2, y2 = [int(v) for v in xyxy]
    cx = (x1 + x2) / 2.0
    offset = (cx / max(float(FRAME_WIDTH), 1.0)) * 2.0 - 1.0
    tid = int(ids[i].item()) if ids is not None else None
    conf = float(boxes.conf[i].item()) if boxes.conf is not None else 0.0
    kpts = None
    if result.keypoints is not None and i < len(result.keypoints.data):
      kpts = result.keypoints.data[i]
    bbox = (x1, y1, x2, y2)
    people.append(
        {
            'index': i,
            'track_id': tid,
            'bbox': bbox,
            'offset': offset,
            'conf': conf,
            'kpts': kpts,
            'hist': upper_hsv_hist(frame, bbox),
        }
    )
  return people


def hand_raised(kpts) -> bool:
  """單手手腕高於同側肩膀。"""
  if kpts is None:
    return False
  # 5/6 肩, 9/10 腕
  for sh_i, wr_i in ((5, 9), (6, 10)):
    sh = fall._kp_xy(kpts, sh_i, min_conf=0.3)
    wr = fall._kp_xy(kpts, wr_i, min_conf=0.3)
    if sh and wr and wr[1] < sh[1] - 20:
      return True
  return False


def resolve_locked_target(people: list[dict], frame: np.ndarray) -> tuple[dict | None, str]:
  """處理鎖定／手勢／HSV 重連。回傳 (target_person, lock_note)。"""
  global locked_target_id, locked_target_hist, lock_request_center, gesture_raise_since

  note = 'unlocked'
  now = time.time()

  # 網頁：鎖定畫面中央
  if lock_request_center:
    lock_request_center = False
    if people:
      best = min(people, key=lambda p: abs(p['offset']))
      if best['track_id'] is not None:
        locked_target_id = best['track_id']
        locked_target_hist = best['hist']
        note = f'locked center id={locked_target_id}'
        print(f'🎯 鎖定中央目標 ID={locked_target_id}', flush=True)
      else:
        note = 'lock failed: no track id'
    else:
      note = 'lock failed: no person'

  # 手勢自動鎖定（尚未鎖定時）
  if ENABLE_GESTURE_LOCK and locked_target_id is None:
    active_ids = set()
    for p in people:
      tid = p['track_id']
      if tid is None:
        continue
      active_ids.add(tid)
      if hand_raised(p['kpts']):
        gesture_raise_since.setdefault(tid, now)
        if now - gesture_raise_since[tid] >= GESTURE_LOCK_SECONDS:
          locked_target_id = tid
          locked_target_hist = p['hist']
          note = f'gesture lock id={tid}'
          print(f'✋ 手勢鎖定目標 ID={tid}', flush=True)
          gesture_raise_since.clear()
          break
      else:
        gesture_raise_since.pop(tid, None)
    for tid in list(gesture_raise_since.keys()):
      if tid not in active_ids:
        gesture_raise_since.pop(tid, None)

  if locked_target_id is None:
    return None, note if note != 'unlocked' else 'unlocked'

  # 直接命中 track_id
  for p in people:
    if p['track_id'] == locked_target_id:
      if p['hist'] is not None:
        locked_target_hist = p['hist']
      return p, f'locked id={locked_target_id}'

  # ByteTrack 換 ID → HSV 重連（搜尋中用更高門檻，避免誤跟路人）
  if locked_target_hist is not None and people:
    thr = HSV_REID_SEARCH if target_lost_since is not None else HSV_REID_THRESHOLD
    scored = []
    for p in people:
      sim = hist_similarity(locked_target_hist, p['hist'])
      if p['track_id'] is not None and sim >= thr:
        scored.append((sim, p))
    if scored:
      scored.sort(key=lambda x: x[0], reverse=True)
      best_sim, best_p = scored[0]
      old = locked_target_id
      locked_target_id = best_p['track_id']
      locked_target_hist = best_p['hist']
      print(
          f'🔁 HSV Re-ID: {old} → {locked_target_id} '
          f'(sim={best_sim:.2f}, thr={thr:.2f})',
          flush=True,
      )
      return best_p, f're-id {old}->{locked_target_id}'

  return None, f'locked id={locked_target_id} LOST'


def _tof_fresh(tof_age: float) -> bool:
  return tof_age <= TOF_STALE_SECONDS


def _front_avoid_cmd(fl: int, ff: int, fr: int) -> str | None:
  """前方過近時回傳覆寫命令；無須覆寫則 None。"""
  blocked = (fl < AVOID_MM) or (ff < AVOID_MM) or (fr < AVOID_MM)
  if not blocked:
    return None
  if fl < AVOID_MM and ff < AVOID_MM and fr < AVOID_MM:
    return 'S'
  if fl > fr + TOF_OPEN_MARGIN:
    return 'L'
  if fr > fl + TOF_OPEN_MARGIN:
    return 'R'
  return 'S'


def _apply_tof_motion_guard(
    cmd: str,
    tof_mm: list[int],
    tof_age: float,
) -> tuple[str, str]:
  """
  Pi 端 ToF 防護：走道側牆、前方矮障、自轉後方。
  與 UNO 底層雙保險；不用 AOA。
  """
  if cmd in {'S', 'X'} or not _tof_fresh(tof_age) or len(tof_mm) < 4:
    return cmd, ''

  fl, ff, fr, rear = (int(tof_mm[0]), int(tof_mm[1]), int(tof_mm[2]), int(tof_mm[3]))
  note = ''

  if cmd == 'Z' and rear < AVOID_MM:
    return 'S', 'tof: rear blocked stop Z'

  # 禁止往近牆側轉（情境1）
  if cmd == 'L' and fl < SIDE_AVOID_MM:
    if fr > fl + TOF_OPEN_MARGIN:
      return 'R', 'tof: left wall -> bias R'
    return 'S', 'tof: left wall hold'
  if cmd == 'R' and fr < SIDE_AVOID_MM:
    if fl > fr + TOF_OPEN_MARGIN:
      return 'L', 'tof: right wall -> bias L'
    return 'S', 'tof: right wall hold'

  if cmd == 'F':
    avoid = _front_avoid_cmd(fl, ff, fr)
    if avoid is not None:
      return avoid, f'tof: front avoid -> {avoid}'
    # 走道置中微偏：一側明顯較近牆則往另一側靠
    if fl < CORRIDOR_NUDGE_MM and fl < fr - CORRIDOR_BIAS_MM:
      return 'R', 'tof: corridor nudge R'
    if fr < CORRIDOR_NUDGE_MM and fr < fl - CORRIDOR_BIAS_MM:
      return 'L', 'tof: corridor nudge L'

  return cmd, note


def _recover_when_target_lost(
    now: float,
    tof_mm: list[int],
    tof_age: float,
) -> tuple[str, str, bool]:
  """目標短暫消失／轉角／遮擋：寬限 → 依最後方位搜 → 自轉 → 逾時停止。"""
  global target_lost_since, search_spin_since, search_exhausted, follow_recovery_note

  if target_lost_since is None:
    target_lost_since = now
    search_spin_since = None

  if search_exhausted:
    follow_recovery_note = 'search exhausted: hold S (re-acquire or unlock)'
    return 'S', follow_recovery_note, False

  lost_for = now - target_lost_since

  # 情境7：寬限內先停，給 YOLO／Re-ID 機會
  if lost_for < LOST_GRACE_SECONDS:
    follow_recovery_note = f'lost grace {lost_for:.1f}s hold'
    return 'S', follow_recovery_note, False

  # 情境3／4：先往「最後看到的方位」轉，避免盲目原地轉錯邊
  bias_for = lost_for - LOST_GRACE_SECONDS
  if bias_for < SEARCH_BIAS_SECONDS:
    if last_seen_offset < -0.05:
      cmd, note = _apply_tof_motion_guard('L', tof_mm, tof_age)
      follow_recovery_note = f'search bias L ({note or "last offset"})'
      return cmd, follow_recovery_note, False
    if last_seen_offset > 0.05:
      cmd, note = _apply_tof_motion_guard('R', tof_mm, tof_age)
      follow_recovery_note = f'search bias R ({note or "last offset"})'
      return cmd, follow_recovery_note, False

  # 全域自轉；後方過近則停；逾時停止避免無限轉（無 AOA）
  if search_spin_since is None:
    search_spin_since = now
  spun = now - search_spin_since
  if spun >= Z_TIMEOUT_SECONDS:
    search_exhausted = True
    follow_recovery_note = f'Z timeout {spun:.0f}s -> S'
    print(f'⚠️ {follow_recovery_note}', flush=True)
    return 'S', follow_recovery_note, False

  cmd, note = _apply_tof_motion_guard('Z', tof_mm, tof_age)
  follow_recovery_note = f'search spin {spun:.1f}s' + (f' ({note})' if note else '')
  return cmd, follow_recovery_note, False


def _stuck_recover(
    cmd: str,
    distance: float | None,
    dist_ok: bool,
    tof_mm: list[int],
    tof_age: float,
    now: float,
) -> tuple[str, str]:
  """
  情境5：無碰觸開關時，用「一直 F 但 UWB 距離沒拉近」推估卡住／細障。
  """
  global stuck_forward_since, stuck_dist_ref, follow_recovery_note

  if cmd != 'F' or not dist_ok or distance is None:
    stuck_forward_since = None
    stuck_dist_ref = None
    return cmd, ''

  if stuck_forward_since is None:
    stuck_forward_since = now
    stuck_dist_ref = distance
    return cmd, ''

  if now - stuck_forward_since < STUCK_SECONDS:
    return cmd, ''

  moved_closer = distance < (stuck_dist_ref - STUCK_DIST_EPS)
  if moved_closer:
    stuck_forward_since = now
    stuck_dist_ref = distance
    return cmd, ''

  # 卡住：往較空側偏轉再開；ToF 不可用則停
  stuck_forward_since = None
  stuck_dist_ref = None
  if _tof_fresh(tof_age) and len(tof_mm) >= 3:
    fl, fr = int(tof_mm[0]), int(tof_mm[2])
    if fl > fr + TOF_OPEN_MARGIN:
      follow_recovery_note = 'stuck: UWB not closing -> L'
      return 'L', follow_recovery_note
    if fr > fl + TOF_OPEN_MARGIN:
      follow_recovery_note = 'stuck: UWB not closing -> R'
      return 'R', follow_recovery_note
  follow_recovery_note = 'stuck: UWB not closing -> S'
  return 'S', follow_recovery_note


def decide_command(
    has_person: bool,
    offset: float,
    distance: float | None,
    dist_age: float,
    following: bool,
    prev_forward: bool,
    fall_emergency: bool = False,
    need_lock: bool = False,
    tof_mm: list[int] | None = None,
    tof_age: float = 999.0,
    now: float | None = None,
) -> tuple[str, str, bool]:
  global last_seen_offset, target_lost_since, search_spin_since
  global search_exhausted, stuck_forward_since, stuck_dist_ref, follow_recovery_note

  if now is None:
    now = time.time()
  if tof_mm is None:
    tof_mm = [8190, 8190, 8190, 8190]

  if fall_emergency:
    follow_recovery_note = ''
    return 'X', 'FALL confirmed -> E-STOP', False

  if not following:
    target_lost_since = None
    search_spin_since = None
    search_exhausted = False
    stuck_forward_since = None
    follow_recovery_note = ''
    return 'S', 'follow OFF (preview)', False

  if need_lock:
    return 'S', 'follow ON but no TARGET lock', False

  dist_ok = distance is not None and dist_age <= UWB_STALE_SECONDS

  # 鎖定中但畫面丟目標 → 復原流程（非立刻 Z）
  if not has_person:
    stuck_forward_since = None
    stuck_dist_ref = None
    return _recover_when_target_lost(now, tof_mm, tof_age)

  # 重新看到指定人：清搜尋狀態
  last_seen_offset = float(offset)
  target_lost_since = None
  search_spin_since = None
  search_exhausted = False
  follow_recovery_note = ''

  if offset < -CENTER_DEADZONE:
    cmd, status, nxt = 'L', f'turn LEFT (offset={offset:+.2f})', prev_forward
  elif offset > CENTER_DEADZONE:
    cmd, status, nxt = 'R', f'turn RIGHT (offset={offset:+.2f})', prev_forward
  elif not dist_ok:
    cmd, status, nxt = 'S', 'centered but UWB stale -> stop', False
  else:
    assert distance is not None
    if distance >= START_DIST:
      cmd, status, nxt = 'F', f'centered dist {distance:.2f}m -> forward', True
    elif distance < STOP_DIST:
      cmd, status, nxt = 'S', f'centered dist {distance:.2f}m -> stop', False
    elif prev_forward:
      cmd, status, nxt = 'F', f'centered band {distance:.2f}m -> keep F', True
    else:
      cmd, status, nxt = 'S', f'centered band {distance:.2f}m -> keep S', False

  # 情境1／2：ToF 走道與前方防護
  guarded, gnote = _apply_tof_motion_guard(cmd, tof_mm, tof_age)
  if gnote:
    status = f'{status} | {gnote}'
    follow_recovery_note = gnote
  cmd = guarded
  if cmd != 'F':
    nxt = False if cmd in {'S', 'X', 'Z'} else prev_forward

  # 情境5：前進卡住推估
  stuck_cmd, snote = _stuck_recover(cmd, distance, dist_ok, tof_mm, tof_age, now)
  if snote:
    status = f'{status} | {snote}'
    follow_recovery_note = snote
    cmd = stuck_cmd
    nxt = cmd == 'F'

  return cmd, status, nxt


def handle_fall_logic(pose, annotated_frame, detecting: bool) -> tuple[str, bool, float]:
  """回傳 (fall_status_en, fall_emergency_now, suspect_elapsed)."""
  global fall_start_time, last_alert_time, active_event, is_following
  global fall_alert_active, fall_alert_ts

  emergency = False
  status = 'fall OFF'
  elapsed = 0.0
  if not detecting:
    if active_event is not None:
      fall.discard_suspect_event(active_event, '偵測已關閉')
      active_event = None
    fall_start_time = None
    return status, False, 0.0

  if pose is None:
    if active_event is not None:
      fall.discard_suspect_event(active_event, '畫面中失去鎖定目標')
      active_event = None
    fall_start_time = None
    return 'fall: no locked target', False, 0.0

  is_falling, is_fall_lateral, is_fall_frontal, *_rest = pose
  now = time.time()

  if is_falling:
    if fall_start_time is None:
      fall_start_time = now
      if active_event is None:
        active_event = fall.start_suspect_event(annotated_frame.copy())
    else:
      if active_event is not None:
        fall.append_event_frame(active_event, annotated_frame)

    elapsed = now - fall_start_time
    if elapsed >= fall.FALL_CONFIRM_SECONDS:
      status = f'FALL confirmed {elapsed:.1f}s'
      if (now - last_alert_time) > fall.ALERT_COOLDOWN_SECONDS:
        print('🚨 確認跌倒 → 急停 + LINE')
        event_to_finalize = active_event
        active_event = None
        original_path = preview_path = video_path = None
        if event_to_finalize is not None:
          original_path, preview_path, video_path = fall.finalize_confirmed_event(
              event_to_finalize, annotated_frame.copy()
          )
        else:
          stamp = fall._now_stamp()
          dest = fall.CONFIRMED_DIR / stamp
          dest.mkdir(parents=True, exist_ok=True)
          original_path, preview_path = fall.save_line_images(
              annotated_frame, 'fall', dest
          )

        fall_type = '側向跌倒' if is_fall_lateral else '正面/背面跌倒'
        alert_msg = (
            '【🚨 緊急安全通報】\n'
            '跟隨小車系統偵測到「鎖定目標」跌倒倒地！\n'
            f'倒地型態: {fall_type}\n'
            f'持續時間: {round(elapsed, 1)} 秒\n'
            f'影片含跌倒前約 {fall.ROLLING_BUFFER_SECONDS:g} 秒前情＋倒地過程\n'
            '小車已緊急煞停，請立即確認現場！'
        )
        motor.send('X', force=True)
        if STOP_FOLLOW_ON_FALL:
          with state_lock:
            is_following = False
        emergency = True
        fall_alert_active = True
        fall_alert_ts = time.strftime('%Y-%m-%d %H:%M:%S')
        threading.Thread(
            target=fall.send_fall_alert,
            args=(alert_msg, original_path, preview_path, video_path),
            daemon=True,
        ).start()
        last_alert_time = now
        active_event = fall.start_suspect_event(annotated_frame.copy())
      else:
        emergency = True
        status = f'FALL cooldown ({elapsed:.1f}s)'
    else:
      status = f'fall suspect {elapsed:.1f}s'
  else:
    if active_event is not None:
      fall.discard_suspect_event(active_event, '恢復站立/坐姿')
      active_event = None
    fall_start_time = None
    status = 'fall: normal pose'
    elapsed = 0.0

  return status, emergency, elapsed


def draw_people(
    frame: np.ndarray,
    people: list[dict],
    locked_id: int | None,
    target: dict | None,
) -> np.ndarray:
  for p in people:
    x1, y1, x2, y2 = p['bbox']
    tid = p['track_id']
    is_tgt = target is not None and p is target
    if is_tgt:
      cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
      label = f'TARGET LOCKED [ID:{tid if tid is not None else "?"}]'
      cv2.putText(
          frame, label, (x1, max(18, y1 - 8)),
          cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA,
      )
      # 畫鎖定目標骨架
      kpts = p['kpts']
      if kpts is not None:
        for i in range(len(kpts)):
          xy = fall._kp_xy(kpts, i, min_conf=0.3)
          if xy:
            cv2.circle(frame, (int(xy[0]), int(xy[1])), 3, (0, 255, 0), -1)
        # 簡化骨架連線
        for a, b in (
            (5, 6), (5, 11), (6, 12), (11, 12),
            (5, 9), (6, 10), (11, 13), (12, 14),
            (0, 5), (0, 6),
        ):
          pa, pb = fall._kp_xy(kpts, a, 0.3), fall._kp_xy(kpts, b, 0.3)
          if pa and pb:
            cv2.line(
                frame,
                (int(pa[0]), int(pa[1])),
                (int(pb[0]), int(pb[1])),
                (0, 200, 0),
                2,
            )
    else:
      cv2.rectangle(frame, (x1, y1), (x2, y2), (120, 120, 120), 1)
      cv2.putText(
          frame,
          f'IGNORED[ID:{tid if tid is not None else "?"}]',
          (x1, max(18, y1 - 6)),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.45,
          (160, 160, 160),
          1,
          cv2.LINE_AA,
      )
  if locked_id is not None and target is None:
    cv2.putText(
        frame,
        f'TARGET ID:{locked_id} LOST - searching',
        (10, FRAME_HEIGHT - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 165, 255),
        2,
        cv2.LINE_AA,
    )
  return frame


def draw_hud(
    frame,
    has_person: bool,
    offset: float,
    distance: float | None,
    dist_age: float,
    cmd: str,
    status: str,
    following: bool,
    detecting: bool,
    fall_txt: str,
    lock_txt: str,
    tof_mm: list[int],
):
  h, w = frame.shape[:2]
  cx = w // 2
  half = int(CENTER_DEADZONE * (w / 2))

  cv2.line(frame, (cx - half, 0), (cx - half, h), (80, 80, 80), 1)
  cv2.line(frame, (cx + half, 0), (cx + half, h), (80, 80, 80), 1)
  cv2.line(frame, (cx, 0), (cx, h), (0, 255, 255), 1)

  dist_txt = 'UWB: --'
  if distance is not None:
    stale = ' (stale)' if dist_age > UWB_STALE_SECONDS else ''
    dist_txt = f'UWB: {distance:.2f}m{stale}'

  tof_txt = 'ToF: ' + ','.join(str(v) for v in tof_mm)

  lines = [
      f'FOLLOW: {"ON" if following else "OFF"}  FALL: {"ON" if detecting else "OFF"}  CMD: {cmd}',
      dist_txt,
      tof_txt[:52],
      lock_txt[:52],
      f'offset: {offset:+.2f}' if has_person else 'target: none',
      status[:52],
      fall_txt[:52],
  ]
  bar_h = 18 + 20 * len(lines)
  overlay = frame.copy()
  cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
  cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
  y = 22
  for line in lines:
    cv2.putText(
        frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
    )
    y += 20
  return frame


def control_loop() -> None:
  global latest_cmd, latest_status, person_offset, person_seen, was_forward
  global latest_jpeg, fall_status, last_buffer_time, fall_suspect_elapsed
  global manual_cmd, manual_cmd_until

  print('=== CONTROL LOOP THREAD ENTER ===', flush=True)
  camera_ok = bool(cap is not None and cap.isOpened())
  if not camera_ok:
    print(
        '=== CONTROL LOOP RUNNING (NO CAMERA: telemetry + manual only) ===',
        flush=True,
    )
  else:
    print('=== CONTROL LOOP RUNNING (follow + fall + lock + tof) ===', flush=True)
  frame_interval = 1.0 / fall.CLIP_FPS

  while not control_stop.is_set():
    # 無相機：仍刷新佔位畫面與手動／急停，讓網頁 UWB／ToF 可看
    if not camera_ok:
      now = time.time()
      with state_lock:
        following = is_following
        man_cmd = manual_cmd
        man_until = manual_cmd_until
        distance = latest_distance
        dist_age = now - last_distance_time if last_distance_time else 999.0
        tof_snapshot = list(latest_tof_mm)
        tof_age = now - last_tof_time if last_tof_time else 999.0

      cmd, status = 'S', 'no camera: follow vision disabled'
      next_forward = False
      if man_cmd and now <= man_until:
        cmd, status, next_forward = man_cmd, f'manual {man_cmd}', man_cmd == 'F'
      elif following:
        status = 'no camera: auto-follow paused (UWB/ToF live)'

      # 無相機時仍套用 ToF 防護（手動前進不會硬撞）
      if cmd in {'F', 'L', 'R', 'Z'}:
        guarded, gnote = _apply_tof_motion_guard(cmd, tof_snapshot, tof_age)
        if gnote:
          status = f'{status} | {gnote}'
        cmd = guarded

      motor.send(cmd, force=True)
      with state_lock:
        latest_cmd = cmd
        latest_status = status
        person_seen = False
        was_forward = next_forward
        fall_status = 'fall: need camera'
      _publish_jpeg(_placeholder_frame('NO CAMERA — UWB/ToF only'))
      time.sleep(0.15)
      continue

    ok, frame = cap.read()
    if not ok:
      _publish_jpeg(_placeholder_frame('CAMERA READ FAIL'))
      time.sleep(0.05)
      continue

    try:
      if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

      results = model.track(
          source=frame,
          persist=True,
          tracker=TRACKER_CFG,
          imgsz=YOLO_IMGSZ,
          conf=0.5,
          verbose=False,
      )
      result = results[0]
      people = extract_tracks(result, frame)
      target, lock_note = resolve_locked_target(people, frame)

      annotated = frame.copy()
      annotated = draw_people(annotated, people, locked_target_id, target)

      has_person = target is not None
      offset = float(target['offset']) if target else 0.0
      bbox = target['bbox'] if target else None

      pose = None
      if target is not None and target['kpts'] is not None:
        x1, y1, x2, y2 = target['bbox']
        pose = fall.analyze_pose_xywh_kpts(
            (x1 + x2) / 2.0,
            (y1 + y2) / 2.0,
            float(x2 - x1),
            float(y2 - y1),
            target['kpts'],
        )

      now = time.time()
      if now - last_buffer_time >= frame_interval:
        ok_jpg, jpg_buf = cv2.imencode(
            '.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        )
        if ok_jpg:
          fall.rolling_buffer.append(jpg_buf.tobytes())
        last_buffer_time = now

      with state_lock:
        following = is_following
        detecting = is_detecting
        distance = latest_distance
        dist_age = time.time() - last_distance_time if last_distance_time else 999.0
        prev_forward = was_forward
        tof_snapshot = list(latest_tof_mm)
        tof_age = time.time() - last_tof_time if last_tof_time else 999.0
        man_cmd = manual_cmd
        man_until = manual_cmd_until
        locked = locked_target_id

      # 跌倒只看鎖定目標
      if detecting and locked is None:
        f_status, fall_emergency, suspect_el = (
            'fall: need TARGET lock',
            False,
            0.0,
        )
      else:
        f_status, fall_emergency, suspect_el = handle_fall_logic(
            pose, annotated, detecting
        )

      need_lock = locked is None
      cmd, status, next_forward = decide_command(
          has_person,
          offset,
          distance,
          dist_age,
          following,
          prev_forward,
          fall_emergency=fall_emergency,
          need_lock=need_lock,
          tof_mm=tof_snapshot,
          tof_age=tof_age,
          now=now,
      )

      # 手動遙控：跟隨 OFF 時才生效；X 永遠可急停
      if man_cmd and now <= man_until:
        if man_cmd == 'X' or not following:
          cmd = man_cmd
          status = f'manual {man_cmd}'
          next_forward = man_cmd == 'F'
      elif man_cmd and now > man_until:
        with state_lock:
          if manual_cmd == man_cmd:
            manual_cmd = None

      if fall_emergency:
        motor.send('X', force=True)
        next_forward = False
      elif following:
        motor.send(cmd)
      elif man_cmd and now <= man_until and man_cmd in {'F', 'L', 'R', 'S', 'X', 'Z'}:
        motor.send(cmd, force=True)
        if man_cmd in {'S', 'X'}:
          next_forward = False
      else:
        motor.send('S')
        next_forward = False

      lock_txt = (
          f'TARGET ID:{locked}' if locked is not None else 'TARGET: none'
      ) + f' | {lock_note}'

      with state_lock:
        latest_cmd = cmd
        latest_status = status
        fall_status = f_status
        fall_suspect_elapsed = suspect_el
        person_offset = offset
        person_seen = has_person
        was_forward = next_forward

      hud = draw_hud(
          annotated,
          has_person,
          offset,
          distance,
          dist_age,
          cmd,
          status,
          following,
          detecting,
          f_status,
          lock_txt,
          tof_snapshot,
      )
      ret, jpg = cv2.imencode('.jpg', hud)
      if ret:
        with jpeg_lock:
          latest_jpeg = jpg.tobytes()
    except Exception as e:
      print(f'控制迴圈例外: {e}')
      time.sleep(0.1)


def generate_frames():
  while not control_stop.is_set():
    with jpeg_lock:
      frame = latest_jpeg
    if frame is None:
      ok_jpg, jpg_buf = cv2.imencode(
          '.jpg',
          _placeholder_frame('WAITING'),
          [int(cv2.IMWRITE_JPEG_QUALITY), 60],
      )
      frame = jpg_buf.tobytes() if ok_jpg else None
      if frame is None:
        time.sleep(0.1)
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
  <title>跌倒偵測 + 跟隨小車</title>
  <style>
    :root {
      --bg: #0f1419; --panel: #1a222c; --text: #e8eef4; --muted: #93a4b5;
      --ok: #1f8f4e; --danger: #c0392b; --warn: #b86b00; --accent: #2d7dd2;
    }
    * { box-sizing: border-box; }
    body { margin: 0; font-family: "Segoe UI", "Noto Sans TC", sans-serif;
           background: linear-gradient(160deg, #0f1419, #182230 50%, #12181f); color: var(--text); }
    main { max-width: 1100px; margin: 0 auto; padding: 16px; }
    h1 { font-size: 1.3rem; margin: 0 0 4px; }
    .sub { color: var(--muted); font-size: .9rem; margin-bottom: 14px; }
    .banner {
      display: none; background: #8b0000; color: #fff; padding: 12px 14px;
      border-radius: 10px; margin-bottom: 12px; font-weight: 700;
      animation: pulse 1.2s ease-in-out infinite;
    }
    .banner.show { display: flex; gap: 12px; align-items: center; justify-content: space-between; flex-wrap: wrap; }
    @keyframes pulse { 50% { filter: brightness(1.15); } }
    .grid { display: grid; grid-template-columns: 1.4fr 1fr; gap: 14px; }
    @media (max-width: 860px) { .grid { grid-template-columns: 1fr; } }
    .card {
      background: var(--panel); border: 1px solid #2a3542; border-radius: 12px; padding: 12px;
    }
    .row { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 10px; }
    button {
      border: 0; border-radius: 8px; padding: 10px 12px; font-size: .95rem; cursor: pointer; color: #fff;
    }
    button:disabled { opacity: .4; cursor: not-allowed; }
    .on { background: var(--ok); }
    .off { background: var(--danger); }
    .warn { background: var(--warn); }
    .ghost { background: #334155; }
    .accent { background: var(--accent); }
    img.feed { width: 100%; background: #000; border-radius: 8px; display: block; }
    .pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; max-width: 240px; }
    .pad .spacer { visibility: hidden; }
    .tof-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; }
    .tof {
      background: #111820; border-radius: 8px; padding: 8px 6px; text-align: center; border: 1px solid #2a3542;
    }
    .tof.danger { border-color: #e74c3c; background: #3a1515; color: #ffb4b4; }
    .tof .v { font-size: 1.05rem; font-weight: 700; font-variant-numeric: tabular-nums; }
    .tof .l { font-size: .75rem; color: var(--muted); }
    .meter {
      height: 8px; background: #243040; border-radius: 99px; overflow: hidden; margin-top: 6px;
    }
    .meter > span { display: block; height: 100%; background: #3cb371; }
    .tof.danger .meter > span { background: #e74c3c; }
    #status {
      margin-top: 8px; padding: 10px; background: #111820; border-radius: 8px;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace; white-space: pre-wrap;
      font-size: .85rem; line-height: 1.45; color: #d7e2ec;
    }
    .hint { opacity: .75; font-size: .85rem; margin-top: 10px; }
    h2 { font-size: 1rem; margin: 0 0 8px; }
  </style>
</head>
<body>
  <main>
    <div id="alertBanner" class="banner">
      <span id="alertText">已確認跌倒！</span>
      <button class="ghost" onclick="api('/api/clear_alert','POST')">解除警報狀態</button>
    </div>
    <h1>跌倒偵測 + 視覺跟隨（鎖定目標版）</h1>
    <p class="sub">version {{ version }}｜UWB + 4×VL53L0X｜ByteTrack 特定人鎖定</p>

    <div class="grid">
      <section class="card">
        <div class="row">
          <button class="on" onclick="api('/start_follow')">開啟跟隨</button>
          <button class="off" onclick="api('/stop_follow')">停止跟隨</button>
          <button class="warn" onclick="api('/start_detection')">開啟跌倒偵測</button>
          <button class="off" onclick="api('/stop_detection')">關閉跌倒偵測</button>
        </div>
        <div class="row">
          <button class="accent" onclick="api('/api/lock_center','POST')">鎖定畫面中央目標</button>
          <button class="ghost" onclick="api('/api/unlock_target','POST')">解除目標鎖定</button>
        </div>
        <img class="feed" src="/video_feed" alt="camera"/>
        <div id="status">載入中…</div>
      </section>

      <aside>
        <section class="card" style="margin-bottom:12px">
          <h2>手動遙控</h2>
          <p class="sub" style="margin:0 0 8px">跟隨 ON 時方向鍵停用；緊急煞停永遠可用</p>
          <div class="pad">
            <span class="spacer"></span>
            <button id="btnF" class="on" onmousedown="hold('F')" onmouseup="releaseHold()" ontouchstart="hold('F')" ontouchend="releaseHold()">前進</button>
            <span class="spacer"></span>
            <button id="btnL" class="accent" onmousedown="hold('L')" onmouseup="releaseHold()" ontouchstart="hold('L')" ontouchend="releaseHold()">左轉</button>
            <button id="btnS" class="ghost" onclick="tap('S')">停止</button>
            <button id="btnR" class="accent" onmousedown="hold('R')" onmouseup="releaseHold()" ontouchstart="hold('R')" ontouchend="releaseHold()">右轉</button>
            <span class="spacer"></span>
            <button class="off" onclick="tap('X')">緊急煞停</button>
            <span class="spacer"></span>
          </div>
        </section>

        <section class="card" style="margin-bottom:12px">
          <h2>UWB 距離</h2>
          <div id="uwbBox">-- m</div>
          <div class="sub" id="uwbAge">freshness: --</div>
        </section>

        <section class="card">
          <h2>VL53L0X（mm）左前／正前／右前／正後</h2>
          <div class="tof-grid" id="tofGrid"></div>
          <div class="sub" id="tofAge" style="margin-top:8px">ToF age: --</div>
        </section>
      </aside>
    </div>

    <p class="hint">
      置中死區 ±{{ deadzone }}｜前進 ≥ {{ start }}m｜停止 &lt; {{ stop }}m｜跌倒確認 {{ confirm }}s｜靈敏度 {{ sens }}｜避障 {{ avoid }}mm<br/>
      UWB/Motor: {{ uwb }}｜手勢鎖定: {{ gesture }}｜{{ version }}
    </p>
  </main>
  <script>
    const AVOID = {{ avoid }};
    const LABELS = {{ labels_json }};
    let holdTimer = null;
    let following = false;

    async function api(path, method='GET', body=null) {
      const opt = { method };
      if (body) {
        opt.headers = {'Content-Type': 'application/json'};
        opt.body = JSON.stringify(body);
      }
      await fetch(path, opt);
      refresh();
    }

    async function tap(cmd) {
      await api('/api/manual_cmd', 'POST', {cmd, hold_ms: 200});
    }
    function hold(cmd) {
      if (following && cmd !== 'X') return;
      api('/api/manual_cmd', 'POST', {cmd, hold_ms: 400});
      if (holdTimer) clearInterval(holdTimer);
      holdTimer = setInterval(() => {
        api('/api/manual_cmd', 'POST', {cmd, hold_ms: 400});
      }, 250);
    }
    function releaseHold() {
      if (holdTimer) { clearInterval(holdTimer); holdTimer = null; }
      api('/api/manual_cmd', 'POST', {cmd: 'S', hold_ms: 150});
    }

    function setManualEnabled(on) {
      following = on;
      ['btnF','btnL','btnR'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = on;
      });
    }

    function renderTof(vals, age) {
      const root = document.getElementById('tofGrid');
      root.innerHTML = '';
      (vals || [8190,8190,8190,8190]).forEach((v, i) => {
        const danger = v < AVOID;
        const pct = Math.max(0, Math.min(100, (v / 1000) * 100));
        const div = document.createElement('div');
        div.className = 'tof' + (danger ? ' danger' : '');
        div.innerHTML = `<div class="l">${LABELS[i]}</div>
          <div class="v">${v}</div>
          <div class="meter"><span style="width:${pct}%"></span></div>`;
        root.appendChild(div);
      });
      document.getElementById('tofAge').textContent =
        `ToF age: ${age.toFixed(2)}s` + (age > 1 ? ' (stale)' : '');
    }

    async function refresh() {
      try {
      const r = await fetch('/api/status');
      if (!r.ok) throw new Error('HTTP ' + r.status);
      const j = await r.json();
      setManualEnabled(!!j.following);

      const banner = document.getElementById('alertBanner');
      if (j.fall_alert_active) {
        banner.classList.add('show');
        document.getElementById('alertText').textContent =
          `已確認跌倒！ ${j.fall_alert_ts || ''}`;
      } else {
        banner.classList.remove('show');
      }

      const uwbStale = j.distance_age > j.uwb_stale_seconds;
      document.getElementById('uwbBox').textContent =
        j.distance == null ? '-- m' : `${j.distance.toFixed(2)} m`;
      document.getElementById('uwbAge').textContent =
        `freshness: ${j.distance_age.toFixed(2)}s (${uwbStale ? '逾時 stale' : '正常'})`;

      renderTof(j.tof_mm, j.tof_age);

      let fallLine = j.fall_status;
      if (j.fall_suspect_elapsed > 0 && j.fall_suspect_elapsed < j.fall_confirm_seconds) {
        fallLine = `疑似跌倒計時中: ${j.fall_suspect_elapsed.toFixed(1)} / ${j.fall_confirm_seconds.toFixed(1)}s`;
      } else if ((j.fall_status || '').startsWith('FALL confirmed')) {
        fallLine = '已確認跌倒！';
      } else if ((j.fall_status || '').includes('normal')) {
        fallLine = '正常';
      }

      document.getElementById('status').textContent =
        `跟隨: ${j.following ? 'ON' : 'OFF'}\\n` +
        `跌倒偵測: ${j.detecting ? 'ON' : 'OFF'}\\n` +
        `鎖定: ${j.locked_target_id == null ? '未鎖定（等待目標）' : '已鎖定 ID: ' + j.locked_target_id}\\n` +
        `命令: ${j.cmd}\\n` +
        `人體目標: ${j.person_seen ? 'yes' : 'no'}  offset=${j.offset.toFixed(2)}\\n` +
        `UWB: ${j.uwb_ok ? 'OK' : 'FAIL'} ${j.uwb_error || ''}\\n` +
        `Motor: ${j.motor_ok ? 'OK' : 'FAIL'} ${j.motor_error || ''}\\n` +
        `跟隨狀態: ${j.status}\\n` +
        `復原: ${j.recovery || '-'}\\n` +
        `跌倒狀態: ${fallLine}`;
      } catch (e) {
        document.getElementById('status').textContent =
          '狀態更新失敗: ' + e + '\\n請確認程式仍在執行，並硬重新整理頁面';
      }
    }

    refresh();
    setInterval(refresh, 300);
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
      confirm=fall.FALL_CONFIRM_SECONDS,
      sens=fall.FALL_SENSITIVITY,
      avoid=AVOID_MM,
      uwb=UWB_SERIAL_PORT,
      gesture='ON' if ENABLE_GESTURE_LOCK else 'OFF',
      version=APP_VERSION,
      labels_json=json.dumps(list(TOF_LABELS), ensure_ascii=False),
  )


@app.route('/video_feed')
def video_feed():
  return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/media/<path:subpath>')
def media_files(subpath: str):
  target = (BASE_DIR / subpath).resolve()
  events_root = (BASE_DIR / 'events').resolve()
  if not str(target).startswith(str(events_root)) or not target.is_file():
    return 'Not Found', 404
  return send_file(target)


@app.route('/start_follow')
def start_follow():
  global is_following
  with state_lock:
    is_following = True
  for _ in range(5):
    motor.send('Z', force=True)
    time.sleep(0.05)
  print(f'▶️ 開啟跟隨｜via_uwb={MOTOR_VIA_UWB}', flush=True)
  return jsonify({'ok': True, 'following': True})


@app.route('/stop_follow')
def stop_follow():
  global is_following, manual_cmd
  with state_lock:
    is_following = False
    manual_cmd = None
  motor.send('S', force=True)
  print('⏹️ 停止跟隨', flush=True)
  return jsonify({'ok': True, 'following': False})


@app.route('/start_detection')
def start_detection():
  global is_detecting
  with state_lock:
    is_detecting = True
  print('▶️ 開啟跌倒偵測', flush=True)
  return jsonify({'ok': True, 'detecting': True})


@app.route('/stop_detection')
def stop_detection():
  global is_detecting, active_event, fall_start_time
  with state_lock:
    is_detecting = False
  if active_event is not None:
    fall.discard_suspect_event(active_event, '手動關閉偵測')
    active_event = None
  fall_start_time = None
  print('⏹️ 關閉跌倒偵測', flush=True)
  return jsonify({'ok': True, 'detecting': False})


@app.route('/api/lock_center', methods=['POST', 'GET'])
def api_lock_center():
  global lock_request_center
  with state_lock:
    lock_request_center = True
  return jsonify({'ok': True, 'msg': 'lock center requested'})


@app.route('/api/unlock_target', methods=['POST', 'GET'])
def api_unlock_target():
  global locked_target_id, locked_target_hist, gesture_raise_since
  global target_lost_since, search_spin_since, search_exhausted
  global stuck_forward_since, stuck_dist_ref, follow_recovery_note
  with state_lock:
    locked_target_id = None
    locked_target_hist = None
    gesture_raise_since.clear()
    target_lost_since = None
    search_spin_since = None
    search_exhausted = False
    stuck_forward_since = None
    stuck_dist_ref = None
    follow_recovery_note = ''
  print('🔓 已解除目標鎖定', flush=True)
  return jsonify({'ok': True, 'locked_target_id': None})


@app.route('/api/manual_cmd', methods=['POST'])
def api_manual_cmd():
  global manual_cmd, manual_cmd_until
  data = request.get_json(silent=True) or {}
  cmd = str(data.get('cmd', 'S')).upper()[:1]
  if cmd not in {'F', 'L', 'R', 'Z', 'S', 'X'}:
    return jsonify({'ok': False, 'error': 'invalid cmd'}), 400
  hold_ms = float(data.get('hold_ms', 300))
  hold_ms = max(80.0, min(hold_ms, 2000.0))

  with state_lock:
    following = is_following

  # 安全互鎖：跟隨 ON 時禁用方向鍵；X 永遠允許
  if following and cmd not in {'X', 'S'}:
    return jsonify({'ok': False, 'error': 'follow ON: manual steer disabled'}), 409

  with state_lock:
    manual_cmd = cmd
    manual_cmd_until = time.time() + (hold_ms / 1000.0)

  if cmd == 'X':
    motor.send('X', force=True)
  return jsonify({'ok': True, 'cmd': cmd})


@app.route('/api/clear_alert', methods=['POST', 'GET'])
def api_clear_alert():
  global fall_alert_active, fall_alert_ts
  with state_lock:
    fall_alert_active = False
    fall_alert_ts = ''
  return jsonify({'ok': True})


@app.route('/status')
@app.route('/api/status')
def status():
  with state_lock:
    dist = latest_distance
    age = time.time() - last_distance_time if last_distance_time else 999.0
    tof_age = time.time() - last_tof_time if last_tof_time else 999.0
    return jsonify(
        {
            'following': is_following,
            'detecting': is_detecting,
            'cmd': latest_cmd,
            'status': latest_status,
            'fall_status': fall_status,
            'fall_suspect_elapsed': fall_suspect_elapsed,
            'fall_confirm_seconds': fall.FALL_CONFIRM_SECONDS,
            'fall_alert_active': fall_alert_active,
            'fall_alert_ts': fall_alert_ts,
            'distance': dist,
            'distance_age': age,
            'uwb_stale_seconds': UWB_STALE_SECONDS,
            'tof_mm': list(latest_tof_mm),
            'tof_age': tof_age,
            'tof_labels': list(TOF_LABELS),
            'avoid_mm': AVOID_MM,
            'recovery': follow_recovery_note,
            'search_exhausted': search_exhausted,
            'person_seen': person_seen,
            'offset': person_offset,
            'locked_target_id': locked_target_id,
            'uwb_ok': uwb_reader.ok,
            'uwb_error': uwb_reader.last_error,
            'motor_ok': motor.ok,
            'motor_error': motor.last_error,
            'motor_via_uwb': motor.via_uwb,
            'motor_port': MOTOR_SERIAL_PORT,
            'version': APP_VERSION,
        }
    )


if __name__ == '__main__':
  print(f'=== APP_VERSION={APP_VERSION} ===', flush=True)
  print(f'=== FILE={Path(__file__).resolve()} ===', flush=True)
  print(f'=== FALL_SENSITIVITY={fall.FALL_SENSITIVITY:g} ===', flush=True)
  print(f'=== TRACKER={TRACKER_CFG} AVOID_MM={AVOID_MM} ===', flush=True)
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
