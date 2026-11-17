"""
樹莓派 5｜YOLO-pose 跌倒偵測 + LINE 文字／照片／影片通報

================================================================================
【本次「照片／影片」附加功能介紹】
--------------------------------------------------------------------------------
目的：
  確認跌倒後，除文字警報外，讓家屬在 LINE 直接看到現場照片與短片
  （短片含跌倒前約 30 秒前情 + 倒地過程）。

原理（為何要上傳圖床）：
  LINE Messaging API 不能直接收樹莓派本機檔案，必須先變成 HTTPS 網址，
  LINE 伺服器再去抓取，聊天室才能顯示。

主力管道（請維持）：
  • 照片　　→ ImgBB（可設過期秒數，例如 600＝約 10 分鐘後圖床刪檔）
  • 影片　　→ litterbox（可播；僅支援 1h/12h/24h/72h，最短 1 小時）
  • 影片封面 → 同用 ImgBB 預覽圖（讓 LINE 影片氣泡有縮圖）

相關流程：
  循環緩衝 30 秒 → 疑似跌倒開始組片 → 滿約 10 秒確認
  → 存本機 → ImgBB 上傳照片／litterbox 上傳影片 → LINE Push

【照片／影片相關程式位置（函式名）】
--------------------------------------------------------------------------------
  設定變數
    IMGBB_API_KEY / IMGBB_EXPIRATION_SECONDS / LITTERBOX_KEEP
    ROLLING_BUFFER_SECONDS

  照片
    save_line_safe_jpeg / save_line_images
    upload_to_imgbb
    resolve_image_urls

  影片（錄製／轉檔／上傳）
    rolling_buffer（循環緩衝）
    decode_rolling_frame / start_suspect_event / append_event_frame
    discard_suspect_event / finalize_confirmed_event
    ensure_line_h264_mp4
    upload_to_litterbox / resolve_video_url

  組裝並推播 LINE
    send_line_messages / send_fall_alert
    （在 generate_frames 確認跌倒處呼叫 send_fall_alert）
================================================================================

主流程：
1. 預設僅骨架預覽（不警報）
2. 鏡頭持續保留最近約 30 秒（行車紀錄器式；逾時自動覆蓋）
3. 開啟偵測後姿勢異常 → 用「跌倒前 30 秒 + 倒地過程」組影片
4. 持續異常滿 FALL_CONFIRM_SECONDS → LINE：文字 + ImgBB 照片 + litterbox 影片
5. 確認前站起來／關閉偵測 → 刪除暫存
"""

from __future__ import annotations

import base64
import os
import shutil
import subprocess
import threading
import time
import uuid
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import requests
from dotenv import load_dotenv
from flask import Flask, Response, render_template_string
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# 環境與路徑
# ---------------------------------------------------------------------------
BASE_DIR = Path(__file__).resolve().parent
load_dotenv(BASE_DIR / '.env')

PENDING_DIR = BASE_DIR / 'events' / 'pending'
CONFIRMED_DIR = BASE_DIR / 'events' / 'confirmed'
PENDING_DIR.mkdir(parents=True, exist_ok=True)
CONFIRMED_DIR.mkdir(parents=True, exist_ok=True)

LINE_CHANNEL_ACCESS_TOKEN = os.getenv('LINE_CHANNEL_ACCESS_TOKEN', '').strip()
LINE_USER_ID = os.getenv('LINE_USER_ID', '').strip()
IMGBB_API_KEY = os.getenv('IMGBB_API_KEY', '').strip()
try:
  IMGBB_EXPIRATION_SECONDS = int(os.getenv('IMGBB_EXPIRATION_SECONDS', '600'))
except ValueError:
  IMGBB_EXPIRATION_SECONDS = 600
IMGBB_EXPIRATION_SECONDS = max(60, min(IMGBB_EXPIRATION_SECONDS, 15_552_000))

LITTERBOX_KEEP = os.getenv('LITTERBOX_KEEP', '1h').strip() or '1h'
if LITTERBOX_KEEP not in {'1h', '12h', '24h', '72h'}:
  print(f'⚠️ LITTERBOX_KEEP={LITTERBOX_KEEP} 無效，改用 1h')
  LITTERBOX_KEEP = '1h'

CAMERA_INDEX = int(os.getenv('CAMERA_INDEX', '0'))
HOST = os.getenv('HOST', '0.0.0.0')
PORT = int(os.getenv('PORT', '5000'))

# 確認跌倒時呼叫 AOA 跟隨服務急停（預設本機 follow_aoa.py :5001）
FOLLOW_ESTOP_URL = os.getenv('FOLLOW_ESTOP_URL', 'http://127.0.0.1:5001/estop').strip()

FALL_CONFIRM_SECONDS = 10.0
ALERT_COOLDOWN_SECONDS = 30.0
ROLLING_BUFFER_SECONDS = float(os.getenv('ROLLING_BUFFER_SECONDS', '30'))
CLIP_FPS = 10.0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

app = Flask(__name__)
state_lock = threading.Lock()
is_detecting = False
fall_start_time = None
last_alert_time = 0.0
active_event = None

print('正在載入 YOLO 模型...')
model = YOLO('yolov8n-pose.pt')
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

rolling_maxlen = max(1, int(ROLLING_BUFFER_SECONDS * CLIP_FPS))
rolling_buffer: deque[bytes] = deque(maxlen=rolling_maxlen)
print(
    f'🎥 循環錄影緩衝約 {ROLLING_BUFFER_SECONDS:g} 秒'
    f'（最多 {rolling_maxlen} 幀 @ {CLIP_FPS:g} fps）'
)
print(f'☁️ 照片主力 ImgBB｜影片主力 litterbox（保存 {LITTERBOX_KEEP}）')


def _now_stamp() -> str:
  return datetime.now().strftime('%Y%m%d_%H%M%S')


def _line_configured() -> bool:
  return bool(LINE_CHANNEL_ACCESS_TOKEN and LINE_USER_ID)


# ===========================================================================
# 照片：縮圖、存檔、ImgBB 上傳
# ===========================================================================
def resize_for_line(image: np.ndarray, max_side: int) -> np.ndarray:
  h, w = image.shape[:2]
  scale = min(max_side / max(h, w), 1.0)
  if scale >= 1.0:
    return image
  return cv2.resize(
      image,
      (max(1, int(w * scale)), max(1, int(h * scale))),
      interpolation=cv2.INTER_AREA,
  )


def encode_jpeg(image: np.ndarray, quality: int = 85) -> bytes:
  ok, buf = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
  if not ok:
    raise RuntimeError('JPEG 編碼失敗')
  return buf.tobytes()


def save_line_safe_jpeg(
    frame: np.ndarray, path: Path, max_side: int, max_bytes: int = 900_000
) -> Path:
  """存成符合 LINE 限制的 JPEG（邊長 + 檔案大小）。"""
  image = resize_for_line(frame, max_side)
  data = encode_jpeg(image, 85)
  for quality in range(85, 35, -5):
    data = encode_jpeg(image, quality)
    if len(data) <= max_bytes:
      break
  path.write_bytes(data)
  print(f'🖼️ 已存 {path.name}: {path.stat().st_size} bytes, max_side={max_side}')
  return path


def save_line_images(frame: np.ndarray, stem: str, folder: Path) -> tuple[Path, Path]:
  folder.mkdir(parents=True, exist_ok=True)
  original_path = folder / f'{stem}_original.jpg'
  preview_path = folder / f'{stem}_preview.jpg'
  save_line_safe_jpeg(frame, original_path, max_side=1024, max_bytes=900_000)
  save_line_safe_jpeg(frame, preview_path, max_side=240, max_bytes=200_000)
  return original_path, preview_path


def upload_to_imgbb(image_path: Path) -> str | None:
  """照片主力：ImgBB（可 expiration 自動刪）。"""
  if not IMGBB_API_KEY:
    return None
  try:
    with open(image_path, 'rb') as f:
      payload = {
          'key': IMGBB_API_KEY,
          'image': base64.b64encode(f.read()).decode('utf-8'),
          'expiration': str(IMGBB_EXPIRATION_SECONDS),
      }
    res = requests.post('https://api.imgbb.com/1/upload', data=payload, timeout=20)
    data = res.json()
    if res.status_code == 200 and data.get('success'):
      url = (
          data.get('data', {}).get('image', {}).get('url')
          or data.get('data', {}).get('display_url')
          or data.get('data', {}).get('url')
      )
      if url and url.startswith('https://'):
        print(
            f'☁️ ImgBB 上傳成功（約 {IMGBB_EXPIRATION_SECONDS / 60:g} 分鐘後刪）: {url}'
        )
        return url
    print(f'❌ ImgBB 上傳失敗: {res.status_code} {res.text[:300]}')
  except Exception as e:
    print(f'❌ ImgBB 上傳例外: {e}')
  return None


def resolve_image_urls(original_path: Path, preview_path: Path) -> tuple[str | None, str | None]:
  preview_url = upload_to_imgbb(preview_path)
  original_url = upload_to_imgbb(original_path)
  if original_url and preview_url:
    return original_url, preview_url
  if preview_url:
    return preview_url, preview_url
  if original_url:
    return original_url, original_url
  print('❌ 照片 ImgBB 上傳失敗；請確認 IMGBB_API_KEY')
  return None, None


def line_can_embed_image(url: str | None) -> bool:
  if not url:
    return False
  u = url.lower()
  return ('i.ibb.co/' in u) or ('ibb.co/' in u)


# ===========================================================================
# 影片：循環緩衝、錄製、H.264、litterbox 上傳
# ===========================================================================
def decode_rolling_frame(jpg_bytes: bytes, frame_size: tuple[int, int]) -> np.ndarray | None:
  frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
  if frame is None:
    return None
  w, h = frame_size
  if frame.shape[1] != w or frame.shape[0] != h:
    frame = cv2.resize(frame, (w, h))
  return frame


def open_video_writer(
    event_dir: Path, frame_size: tuple[int, int]
) -> tuple[cv2.VideoWriter | None, Path | None]:
  event_dir.mkdir(parents=True, exist_ok=True)
  w, h = frame_size
  for out_path, fourcc_name in ((event_dir / 'clip.mp4', 'mp4v'), (event_dir / 'clip.avi', 'XVID')):
    writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*fourcc_name), CLIP_FPS, (w, h))
    if writer.isOpened():
      print(f'ℹ️ VideoWriter 使用: {out_path.name} ({fourcc_name})')
      return writer, out_path
    writer.release()
    if out_path.exists():
      out_path.unlink(missing_ok=True)
  print('❌ 無法建立 VideoWriter')
  return None, None


def start_suspect_event(first_frame: np.ndarray) -> dict:
  """疑似跌倒：先寫入跌倒前循環緩衝，再繼續錄倒地過程。"""
  event_id = f'{_now_stamp()}_{uuid.uuid4().hex[:6]}'
  event_dir = PENDING_DIR / event_id
  event_dir.mkdir(parents=True, exist_ok=True)
  h, w = first_frame.shape[:2]
  writer, video_path = open_video_writer(event_dir, (w, h))

  pre_count = 0
  if writer is not None:
    for jpg in list(rolling_buffer):
      frame = decode_rolling_frame(jpg, (w, h))
      if frame is None:
        continue
      writer.write(frame)
      pre_count += 1
    writer.write(first_frame)

  original_path, preview_path = save_line_images(first_frame, 'suspect', event_dir)
  print(
      f'⚠️ 疑似跌倒，開始錄影: {event_id}'
      f'｜前情約 {pre_count / CLIP_FPS:.1f}s'
  )
  return {
      'id': event_id,
      'dir': event_dir,
      'writer': writer,
      'video_path': video_path,
      'photo_original': original_path,
      'photo_preview': preview_path,
      'frame_size': (w, h),
  }


def append_event_frame(event: dict, frame: np.ndarray) -> None:
  writer = event.get('writer')
  if writer is None:
    return
  w, h = event['frame_size']
  if frame.shape[1] != w or frame.shape[0] != h:
    frame = cv2.resize(frame, (w, h))
  writer.write(frame)


def discard_suspect_event(event: dict | None, reason: str) -> None:
  if not event:
    return
  writer = event.get('writer')
  if writer is not None:
    writer.release()
    event['writer'] = None
  event_dir = event.get('dir')
  if event_dir and Path(event_dir).exists():
    shutil.rmtree(event_dir, ignore_errors=True)
    print(f'🗑️ 已刪除疑似片段（{reason}）: {event.get("id")}')


def finalize_confirmed_event(
    event: dict, annotated_frame: np.ndarray
) -> tuple[Path, Path, Path | None]:
  writer = event.get('writer')
  if writer is not None:
    writer.release()
    event['writer'] = None

  original_path, preview_path = save_line_images(annotated_frame, 'fall', event['dir'])
  video_path = event.get('video_path')
  if video_path is None or not Path(video_path).exists() or Path(video_path).stat().st_size <= 0:
    video_path = None
    for name in ('clip.mp4', 'clip.avi'):
      candidate = event['dir'] / name
      if candidate.exists() and candidate.stat().st_size > 0:
        video_path = candidate
        break

  dest_dir = CONFIRMED_DIR / event['id']
  if dest_dir.exists():
    shutil.rmtree(dest_dir, ignore_errors=True)
  shutil.move(str(event['dir']), str(dest_dir))

  confirmed_video = dest_dir / Path(video_path).name if video_path else None
  print(f'💾 已保留確認跌倒媒體: {dest_dir}')
  return dest_dir / original_path.name, dest_dir / preview_path.name, confirmed_video


def ensure_line_h264_mp4(video_path: Path) -> Path | None:
  """轉成 LINE 較相容的 H.264 MP4。"""
  if video_path is None or not video_path.exists():
    return None
  out_path = video_path.with_name(video_path.stem + '_h264.mp4')
  cmd = [
      'ffmpeg', '-y', '-i', str(video_path),
      '-c:v', 'libx264', '-profile:v', 'baseline', '-level', '3.0',
      '-pix_fmt', 'yuv420p', '-vf', 'scale=640:-2', '-r', '15',
      '-movflags', '+faststart', '-an', str(out_path),
  ]
  try:
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if result.returncode == 0 and out_path.exists() and out_path.stat().st_size > 0:
      print(f'🎬 已轉成 H.264: {out_path.name}')
      return out_path
    print(f'⚠️ ffmpeg 轉檔失敗，改傳原影片。{result.stderr[-200:]}')
  except FileNotFoundError:
    print('⚠️ 未安裝 ffmpeg（建議: sudo apt install -y ffmpeg）')
  except Exception as e:
    print(f'⚠️ ffmpeg 例外: {e}')
  return video_path


def upload_to_litterbox(file_path: Path) -> str | None:
  """影片主力：litterbox（自動過期）。"""
  try:
    with open(file_path, 'rb') as f:
      res = requests.post(
          'https://litterbox.catbox.moe/resources/internals/api.php',
          data={'reqtype': 'fileupload', 'time': LITTERBOX_KEEP},
          files={'fileToUpload': (file_path.name, f, 'video/mp4')},
          headers={'User-Agent': 'FallDetection/1.0'},
          timeout=180,
      )
    text = (res.text or '').strip()
    if res.status_code == 200 and text.startswith('https://'):
      print(f'☁️ litterbox 上傳成功（{LITTERBOX_KEEP} 後刪）: {text}')
      return text
    print(f'❌ litterbox 上傳失敗: {res.status_code} {text[:300]!r}')
  except Exception as e:
    print(f'❌ litterbox 上傳例外: {e}')
  return None


def resolve_video_url(
    video_path: Path, preview_url: str | None
) -> tuple[str | None, str | None]:
  """回傳 (video_url, preview_url)；影片=litterbox，封面=ImgBB。"""
  ready = ensure_line_h264_mp4(video_path)
  if ready is None:
    return None, preview_url

  try:
    print(f'📦 影片大小約 {ready.stat().st_size / (1024 * 1024):.2f} MB')
  except OSError:
    pass

  video_url = upload_to_litterbox(ready)
  if not video_url:
    return None, preview_url

  if not line_can_embed_image(preview_url):
    for name in ('fall_preview.jpg', 'fall_original.jpg'):
      candidate = ready.parent / name
      if candidate.exists():
        preview_url = upload_to_imgbb(candidate)
        if line_can_embed_image(preview_url):
          break

  if not line_can_embed_image(preview_url):
    print('⚠️ 影片預覽圖需要 ImgBB')
    return video_url, None
  return video_url, preview_url


# ===========================================================================
# LINE 推播
# ===========================================================================
def send_line_messages(messages: list[dict]) -> bool:
  if not _line_configured():
    print('❌ 未設定 LINE 憑證，略過推播')
    return False
  try:
    print('正在嘗試連線至 LINE 伺服器...')
    res = requests.post(
        'https://api.line.me/v2/bot/message/push',
        headers={
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {LINE_CHANNEL_ACCESS_TOKEN}',
        },
        json={'to': LINE_USER_ID, 'messages': messages},
        timeout=10,
    )
    if res.status_code == 200:
      print('✅ 成功發送 LINE 訊息！')
      return True
    print(f'❌ LINE 發送失敗 ({res.status_code}): {res.text}')
  except Exception as e:
    print(f'❌ LINE 發送例外: {e}')
  return False


def trigger_follow_estop() -> None:
  """通知 14_MaUWB_AOA_Follow/follow_aoa.py 送 X 並鎖住跟隨。"""
  if not FOLLOW_ESTOP_URL:
    return
  try:
    r = requests.get(FOLLOW_ESTOP_URL, timeout=2.0)
    print(f'🛑 已請求小車急停: {FOLLOW_ESTOP_URL} -> HTTP {r.status_code}')
  except Exception as e:
    print(f'⚠️ 小車急停請求失敗（跟隨服務可能未開）: {e}')


def send_fall_alert(
    message_text: str,
    original_path: Path,
    preview_path: Path,
    video_path: Path | None = None,
) -> None:
  """文字 + ImgBB 照片 + litterbox 影片。"""
  messages: list[dict] = [{'type': 'text', 'text': message_text}]

  original_url, preview_url = resolve_image_urls(original_path, preview_path)
  if original_url and preview_url:
    messages[0]['text'] = message_text + f'\n\n📷 照片連結:\n{original_url}'
    if line_can_embed_image(original_url) and line_can_embed_image(preview_url):
      messages.append(
          {
              'type': 'image',
              'originalContentUrl': original_url,
              'previewImageUrl': preview_url,
          }
      )
      print(f'🖼️ LINE 圖片訊息: {original_url}')
    else:
      print('⚠️ 照片僅放文字連結（請確認 IMGBB_API_KEY）')
  else:
    print('⚠️ 無法上傳照片；已保存在本機')

  if video_path and video_path.exists():
    video_url, video_preview = resolve_video_url(video_path, preview_url)
    if video_url and video_preview:
      messages.append(
          {
              'type': 'video',
              'originalContentUrl': video_url,
              'previewImageUrl': video_preview,
          }
      )
      messages[0]['text'] += f'\n🎬 影片連結:\n{video_url}'
      print(f'🎬 LINE 影片氣泡: {video_url}')
    else:
      print('⚠️ 無法建立 LINE 影片氣泡（檢查 litterbox／ImgBB）')

  for i, msg in enumerate(messages[1:], start=1):
    print(f'📎 LINE 媒體[{i}]: {msg.get("type")} -> {msg.get("originalContentUrl")}')
  send_line_messages(messages[:5])


# ===========================================================================
# 跌倒判定與主迴圈
# ===========================================================================
def analyze_pose(result) -> tuple[bool, bool, bool, float, float, float] | None:
  if len(result.boxes) == 0 or result.keypoints is None or len(result.keypoints.data) == 0:
    return None

  box = result.boxes.xywh[0]
  w, h = float(box[2]), float(box[3])
  if h <= 1e-6:
    return None
  aspect_ratio = round(w / h, 2)

  keypoints = result.keypoints.data[0]
  confs = [float(keypoints[i][2]) for i in (5, 6, 11, 12)]
  if min(confs) < 0.3:
    is_fall_lateral = aspect_ratio > 1.1
    return is_fall_lateral, is_fall_lateral, False, aspect_ratio, -1.0, -1.0

  avg_shoulder_y = (float(keypoints[5][1]) + float(keypoints[6][1])) / 2
  avg_hip_y = (float(keypoints[11][1]) + float(keypoints[12][1])) / 2
  torso_height = abs(avg_hip_y - avg_shoulder_y)
  is_fall_lateral = aspect_ratio > 1.1
  is_fall_frontal = (torso_height < 40) and (avg_shoulder_y > 220)
  return (
      is_fall_lateral or is_fall_frontal,
      is_fall_lateral,
      is_fall_frontal,
      aspect_ratio,
      torso_height,
      avg_shoulder_y,
  )


def generate_frames():
  global fall_start_time, last_alert_time, active_event

  if not cap.isOpened():
    print('錯誤：無法開啟攝影機！')
    return

  frame_interval = 1.0 / CLIP_FPS
  last_buffer_time = 0.0

  while True:
    success, frame = cap.read()
    if not success:
      print('⚠️ 讀取攝影機失敗，稍後重試...')
      time.sleep(0.2)
      continue

    try:
      results = model.predict(source=frame, imgsz=320, conf=0.5, verbose=False)
      annotated_frame = results[0].plot()

      now = time.time()
      if now - last_buffer_time >= frame_interval:
        ok_jpg, jpg_buf = cv2.imencode(
            '.jpg', annotated_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        )
        if ok_jpg:
          rolling_buffer.append(jpg_buf.tobytes())
        last_buffer_time = now

      status_msg = '--> 【等待偵測人體】'
      pose = analyze_pose(results[0])

      with state_lock:
        detecting = is_detecting
        event = active_event

      if pose is not None:
        (
            is_currently_falling,
            is_fall_lateral,
            _is_fall_frontal,
            aspect_ratio,
            torso_height,
            avg_shoulder_y,
        ) = pose
        current_time = time.time()

        if detecting:
          if is_currently_falling:
            if fall_start_time is None:
              fall_start_time = current_time
              with state_lock:
                if active_event is None:
                  active_event = start_suspect_event(annotated_frame.copy())
                  event = active_event
            elif event is not None:
              append_event_frame(event, annotated_frame)

            elapsed = current_time - fall_start_time
            if elapsed >= FALL_CONFIRM_SECONDS:
              status_msg = f'--> 【🚨 確認跌倒！已持續 {round(elapsed, 1)}s】'
              if (current_time - last_alert_time) > ALERT_COOLDOWN_SECONDS:
                print('🚨 觸發 LINE 發送機制！')
                with state_lock:
                  event_to_finalize = active_event
                  active_event = None

                if event_to_finalize is not None:
                  original_path, preview_path, video_path = finalize_confirmed_event(
                      event_to_finalize, annotated_frame.copy()
                  )
                else:
                  dest = CONFIRMED_DIR / _now_stamp()
                  dest.mkdir(parents=True, exist_ok=True)
                  original_path, preview_path = save_line_images(
                      annotated_frame, 'fall', dest
                  )
                  video_path = None

                fall_type = '側向跌倒' if is_fall_lateral else '正面/背面跌倒'
                alert_msg = (
                    '【🚨 緊急安全通報】\n'
                    '樹莓派系統偵測到人體跌倒倒地！\n'
                    f'倒地型態: {fall_type}\n'
                    f'持續時間: {round(elapsed, 1)} 秒\n'
                    f'影片含跌倒前約 {ROLLING_BUFFER_SECONDS:g} 秒前情＋倒地過程\n'
                    '請立即確認現場狀況！'
                )
                trigger_follow_estop()
                send_fall_alert(alert_msg, original_path, preview_path, video_path)
                last_alert_time = current_time
                with state_lock:
                  active_event = start_suspect_event(annotated_frame.copy())
            else:
              status_msg = (
                  f'--> 【⚠️ 疑似跌倒，確認中... ({round(elapsed, 1)}s)】'
              )
          else:
            if event is not None:
              with state_lock:
                to_discard = active_event
                active_event = None
              discard_suspect_event(to_discard, '恢復站立/坐姿')
            fall_start_time = None
            status_msg = '--> 【正常站立/坐姿】'
        else:
          if event is not None:
            with state_lock:
              to_discard = active_event
              active_event = None
            discard_suspect_event(to_discard, '偵測已關閉')
          fall_start_time = None
          status_msg = '--> 【骨架測試中（偵測關閉）】'

        torso_txt = 'n/a' if torso_height < 0 else str(round(torso_height, 1))
        shoulder_txt = 'n/a' if avg_shoulder_y < 0 else str(round(avg_shoulder_y, 1))
        print(
            f'寬高比: {aspect_ratio} | 軀幹長: {torso_txt} |'
            f' 肩膀Y: {shoulder_txt} {status_msg}'
        )
      else:
        with state_lock:
          detecting = is_detecting
          event = active_event
        if detecting and event is not None:
          with state_lock:
            to_discard = active_event
            active_event = None
          discard_suspect_event(to_discard, '畫面中失去人體')
          fall_start_time = None

      ret, buffer = cv2.imencode('.jpg', annotated_frame)
      if not ret:
        continue
      yield (
          b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n'
      )
    except Exception as e:
      print(f'發生例外錯誤: {e}')
      time.sleep(0.1)
      continue


# ===========================================================================
# Flask 網頁
# ===========================================================================
@app.route('/start_detection')
def start_detection():
  global is_detecting, fall_start_time, active_event
  with state_lock:
    is_detecting = True
    fall_start_time = None
    if active_event is not None:
      discard_suspect_event(active_event, '重新開始偵測')
      active_event = None
  print('▶️ 開啟跌倒偵測')
  return 'STARTED'


@app.route('/stop_detection')
def stop_detection():
  global is_detecting, fall_start_time, active_event
  with state_lock:
    is_detecting = False
    fall_start_time = None
    if active_event is not None:
      discard_suspect_event(active_event, '手動停止偵測')
      active_event = None
  print('⏸️ 停止跌倒偵測')
  return 'STOPPED'


@app.route('/')
def index():
  return render_template_string(
      """
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="utf-8">
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>YOLO-pose 跌倒通報系統</title>
      <style>
        body { text-align: center; background: #222; color: #fff; font-family: Arial, sans-serif; }
        .btn { padding: 12px 24px; font-size: 18px; font-weight: bold; margin: 10px;
               border: none; border-radius: 8px; cursor: pointer; }
        .btn-start { background: #28a745; color: #fff; }
        .btn-stop { background: #dc3545; color: #fff; }
        #status-text { font-size: 20px; font-weight: bold; color: #ffc107; margin-top: 10px; }
        .hint { color: #bbb; font-size: 14px; max-width: 640px; margin: 12px auto; line-height: 1.5; }
        img { max-width: 100%; height: auto; }
      </style>
      <script>
        function startDetection() {
          fetch('/start_detection').then(() => {
            const el = document.getElementById('status-text');
            el.innerText = '當前狀態：🟢 跌倒偵測中 (會發送 LINE)';
            el.style.color = '#28a745';
          });
        }
        function stopDetection() {
          fetch('/stop_detection').then(() => {
            const el = document.getElementById('status-text');
            el.innerText = '當前狀態：🟡 僅骨架測試 (不會發送 LINE)';
            el.style.color = '#ffc107';
          });
        }
      </script>
    </head>
    <body>
      <h2>雙重判定跌倒偵測與 LINE 通報系統</h2>
      <div>
        <button class="btn btn-start" onclick="startDetection()">▶️ 開啟跌倒偵測</button>
        <button class="btn btn-stop" onclick="stopDetection()">⏸️ 停止跌倒偵測</button>
      </div>
      <div id="status-text">當前狀態：🟡 僅骨架測試 (不會發送 LINE)</div>
      <p class="hint">
        鏡頭保留最近約 30 秒畫面（無跌倒自動覆蓋）。
        確認後 LINE 收到 ImgBB 照片＋litterbox 前情短片。
      </p>
      <img src="/video_feed" width="640" alt="video feed">
    </body>
    </html>
    """
  )


@app.route('/video_feed')
def video_feed():
  return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
  env_path = BASE_DIR / '.env'
  print(f'📂 .env 路徑: {env_path} （存在: {env_path.exists()}）')
  if _line_configured():
    print(
        f'🔐 LINE 已載入｜Token 長度={len(LINE_CHANNEL_ACCESS_TOKEN)}｜'
        f'UserID 開頭={LINE_USER_ID[:4]}...'
    )
  else:
    print('⚠️ 尚未讀到 LINE 憑證：請確認 .env 已存檔後重開')
  if IMGBB_API_KEY:
    print(f'☁️ ImgBB 已設定｜約 {IMGBB_EXPIRATION_SECONDS / 60:g} 分鐘後圖床刪照片')
  else:
    print('⚠️ 未設定 IMGBB_API_KEY：LINE 照片氣泡可能無法顯示')
  print(f'🎬 影片主力 litterbox（{LITTERBOX_KEEP} 後刪）')
  print(f'伺服器啟動：http://{HOST}:{PORT}')
  app.run(host=HOST, port=PORT, debug=False, threaded=True)
