"""
樹莓派 5｜YOLO-pose 跌倒偵測 + LINE 文字/照片/影片通報

流程：
1. 預設僅骨架預覽（不警報）
2. 開啟偵測後，若姿勢異常 → 開始錄「疑似跌倒」短片
3. 持續異常滿 FALL_CONFIRM_SECONDS → 確認跌倒：保留媒體、推播 LINE（文字+照片，可選影片）
4. 若在確認前站起來 → 自動刪除疑似短片與暫存照片
"""

from __future__ import annotations

import base64
import os
import shutil
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
from flask import Flask, Response, render_template_string, send_file
from ultralytics import YOLO

# 固定從「這個 .py 檔所在資料夾」讀 .env（避免工作目錄不同而讀不到）
BASE_DIR = Path(__file__).resolve().parent
load_dotenv(BASE_DIR / '.env')

PENDING_DIR = BASE_DIR / 'events' / 'pending'
CONFIRMED_DIR = BASE_DIR / 'events' / 'confirmed'
PENDING_DIR.mkdir(parents=True, exist_ok=True)
CONFIRMED_DIR.mkdir(parents=True, exist_ok=True)

# --- LINE / 公開網址設定（請用 .env，不要把 Token 寫進程式）---
LINE_CHANNEL_ACCESS_TOKEN = os.getenv('LINE_CHANNEL_ACCESS_TOKEN', '').strip()
LINE_USER_ID = os.getenv('LINE_USER_ID', '').strip()
PUBLIC_BASE_URL = os.getenv('PUBLIC_BASE_URL', '').rstrip('/')
IMGBB_API_KEY = os.getenv('IMGBB_API_KEY', '').strip()
# ImgBB 自動刪除秒數（最短 60，最長 15552000）；預設 600 = 10 分鐘
try:
  IMGBB_EXPIRATION_SECONDS = int(os.getenv('IMGBB_EXPIRATION_SECONDS', '600'))
except ValueError:
  IMGBB_EXPIRATION_SECONDS = 600
IMGBB_EXPIRATION_SECONDS = max(60, min(IMGBB_EXPIRATION_SECONDS, 15_552_000))

CAMERA_INDEX = int(os.getenv('CAMERA_INDEX', '0'))
HOST = os.getenv('HOST', '0.0.0.0')
PORT = int(os.getenv('PORT', '5000'))

# --- 偵測參數 ---
FALL_CONFIRM_SECONDS = 10.0
ALERT_COOLDOWN_SECONDS = 30.0
PRE_ROLL_SECONDS = 3.0  # 跌倒前多留幾秒畫面進影片
CLIP_FPS = 10.0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

app = Flask(__name__)

# --- 執行期狀態（加鎖避免網頁按鈕與串流執行緒互相踩）---
state_lock = threading.Lock()
is_detecting = False
fall_start_time = None
last_alert_time = 0.0
active_event = None  # 進行中的疑似跌倒事件

print('正在載入 YOLO 模型...')
model = YOLO('yolov8n-pose.pt')
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# 跌倒前畫面緩衝（約 PRE_ROLL_SECONDS）
pre_roll_maxlen = max(1, int(PRE_ROLL_SECONDS * CLIP_FPS))
pre_roll_buffer: deque[np.ndarray] = deque(maxlen=pre_roll_maxlen)


def _now_stamp() -> str:
  return datetime.now().strftime('%Y%m%d_%H%M%S')


def _line_configured() -> bool:
  return bool(LINE_CHANNEL_ACCESS_TOKEN and LINE_USER_ID)


def resize_for_line(image: np.ndarray, max_side: int) -> np.ndarray:
  """縮放 JPEG 影像以符合 LINE 尺寸限制。"""
  h, w = image.shape[:2]
  scale = min(max_side / max(h, w), 1.0)
  if scale >= 1.0:
    return image
  new_size = (max(1, int(w * scale)), max(1, int(h * scale)))
  return cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)


def encode_jpeg(image: np.ndarray, quality: int = 85) -> bytes:
  ok, buf = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
  if not ok:
    raise RuntimeError('JPEG 編碼失敗')
  return buf.tobytes()


def save_line_safe_jpeg(
    frame: np.ndarray, path: Path, max_side: int, max_bytes: int = 900_000
) -> Path:
  """存成 LINE 較不易失敗的 JPEG（尺寸 + 檔案大小雙重限制）。"""
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
  """產出 original（≤1024）與 preview（≤240）兩張 JPEG。"""
  folder.mkdir(parents=True, exist_ok=True)
  original_path = folder / f'{stem}_original.jpg'
  preview_path = folder / f'{stem}_preview.jpg'
  save_line_safe_jpeg(frame, original_path, max_side=1024, max_bytes=900_000)
  save_line_safe_jpeg(frame, preview_path, max_side=240, max_bytes=200_000)
  return original_path, preview_path


def public_media_url(path: Path) -> str | None:
  """把本機檔案轉成 HTTPS URL（需設定 PUBLIC_BASE_URL）。照片請勿依賴此路徑給 LINE。"""
  if not PUBLIC_BASE_URL:
    return None
  try:
    rel = path.resolve().relative_to(BASE_DIR.resolve()).as_posix()
  except ValueError:
    return None
  return f'{PUBLIC_BASE_URL}/media/{rel}'


def is_direct_https_media_url(url: str) -> bool:
  """上傳成功回傳的直連網址即可採用。"""
  if not url or not url.startswith('https://'):
    return False
  lower = url.lower()
  if any(
      host in lower
      for host in (
          'telegra.ph/',
          'i.ibb.co/',
          'ibb.co/',
          'files.catbox.moe/',
          'litter.catbox.moe/',
          '0x0.st/',
      )
  ):
    return True
  return lower.endswith(('.jpg', '.jpeg', '.png', '.mp4'))


def upload_to_imgbb(image_path: Path) -> str | None:
  """ImgBB 上傳照片（需 API key）— 對 LINE 顯示通常最穩；可設定自動過期刪除。"""
  if not IMGBB_API_KEY:
    return None
  try:
    with open(image_path, 'rb') as f:
      payload = {
          'key': IMGBB_API_KEY,
          'image': base64.b64encode(f.read()).decode('utf-8'),
          # 10 分鐘 = 600 秒；讓第三方圖床自動刪，降低長期留存
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
        mins = IMGBB_EXPIRATION_SECONDS / 60
        print(f'☁️ ImgBB 上傳成功（約 {mins:g} 分鐘後自動刪）: {url}')
        return url
      print(f'⚠️ ImgBB 回傳網址不可用: {url}')
    print(f'❌ ImgBB 上傳失敗: {res.status_code} {res.text[:300]}')
  except Exception as e:
    print(f'❌ ImgBB 上傳例外: {e}')
  return None


def upload_to_telegraph(image_path: Path) -> str | None:
  """
  Telegram Telegraph 圖床（免 API key）。
  人用瀏覽器看 catbox 沒問題，但 LINE 伺服器常抓不到 catbox；
  telegra.ph 對 LINE 圖片訊息通常較可用。
  """
  try:
    with open(image_path, 'rb') as f:
      res = requests.post(
          'https://telegra.ph/upload',
          files={'file': ('image.jpg', f, 'image/jpeg')},
          timeout=30,
      )
    data = res.json()
    if isinstance(data, list) and data and isinstance(data[0], dict) and data[0].get('src'):
      src = data[0]['src']
      url = src if src.startswith('http') else f'https://telegra.ph{src}'
      print(f'☁️ Telegraph 上傳成功: {url}')
      return url
    print(f'❌ Telegraph 上傳失敗: {res.status_code} {str(data)[:300]}')
  except Exception as e:
    print(f'❌ Telegraph 上傳例外: {e}')
  return None


def upload_to_catbox(file_path: Path) -> str | None:
  """匿名上傳到 catbox（適合影片；照片給 LINE 常顯示空白，僅作備援）。"""
  try:
    with open(file_path, 'rb') as f:
      res = requests.post(
          'https://catbox.moe/user/api.php',
          data={'reqtype': 'fileupload'},
          files={'fileToUpload': (file_path.name, f, 'application/octet-stream')},
          timeout=120,
      )
    text = (res.text or '').strip()
    if res.status_code == 200 and text.startswith('https://'):
      print(f'☁️ catbox 上傳成功: {text}')
      return text
    print(f'❌ catbox 上傳失敗: {res.status_code} {text[:300]}')
  except Exception as e:
    print(f'❌ catbox 上傳例外: {e}')
  return None


def upload_image_for_line(image_path: Path) -> str | None:
  """
  照片給 LINE 官方帳號顯示用：
  優先 ImgBB / Telegraph（LINE 伺服器較讀得到）
  catbox 人點連結看得到，但 LINE 氣泡常空白，故放最後且會警告。
  """
  for uploader in (upload_to_imgbb, upload_to_telegraph):
    url = uploader(image_path)
    if url and is_direct_https_media_url(url):
      return url

  # 最後才 catbox：文字連結可用，image message 可能仍空白
  url = upload_to_catbox(image_path)
  if url:
    print('⚠️ catbox 照片網址已取得，但 LINE 圖片氣泡可能仍無法顯示；建議設定 IMGBB_API_KEY')
    return url
  return None


def ensure_line_h264_mp4(video_path: Path) -> Path | None:
  """LINE 影片較吃 H.264；若系統有 ffmpeg 就轉檔，否則回傳原檔讓後續嘗試。"""
  if video_path is None or not video_path.exists():
    return None
  if video_path.suffix.lower() not in ('.mp4', '.avi'):
    return video_path

  out_path = video_path.with_name(video_path.stem + '_h264.mp4')
  cmd = [
      'ffmpeg',
      '-y',
      '-i',
      str(video_path),
      '-c:v',
      'libx264',
      '-pix_fmt',
      'yuv420p',
      '-movflags',
      '+faststart',
      '-an',
      str(out_path),
  ]
  try:
    import subprocess

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if result.returncode == 0 and out_path.exists() and out_path.stat().st_size > 0:
      print(f'🎬 已轉成 H.264: {out_path.name}')
      return out_path
    print(f'⚠️ ffmpeg 轉檔失敗，改傳原影片。stderr: {result.stderr[-300:]}')
  except FileNotFoundError:
    print('⚠️ 未安裝 ffmpeg，影片可能無法在 LINE 播放（建議: sudo apt install -y ffmpeg）')
  except Exception as e:
    print(f'⚠️ ffmpeg 轉檔例外: {e}')
  return video_path


def line_can_embed_image(url: str | None) -> bool:
  # LINE 伺服器需能抓圖；catbox 常失敗，telegra.ph / imgbb 較穩
  if not url:
    return False
  u = url.lower()
  return ('telegra.ph/' in u) or ('i.ibb.co/' in u) or ('ibb.co/' in u)


def resolve_image_urls(original_path: Path, preview_path: Path) -> tuple[str | None, str | None]:
  """照片走 ImgBB（LINE 氣泡才穩）；不走 cloudflared。"""
  preview_url = upload_image_for_line(preview_path)
  original_url = upload_image_for_line(original_path)

  if original_url and preview_url:
    return original_url, preview_url
  if preview_url:
    print('ℹ️ 僅 preview 上傳成功，original/preview 共用同一網址')
    return preview_url, preview_url
  if original_url:
    print('ℹ️ 僅 original 上傳成功，original/preview 共用同一網址')
    return original_url, original_url

  print('❌ 照片圖床上傳失敗；請確認 IMGBB_API_KEY')
  return None, None


def resolve_video_url(video_path: Path, preview_url: str | None) -> tuple[Path | None, str | None, str | None]:
  """
  影片給 LINE 氣泡播放：
  - 檔案本身走 PUBLIC_BASE_URL（cloudflared）
  - 預覽圖走 ImgBB
  回傳 (ready_path, video_url, preview_url)
  """
  ready = ensure_line_h264_mp4(video_path)
  if ready is None:
    return None, None, preview_url

  video_url = public_media_url(ready)
  if video_url:
    print(f'🔗 影片使用 cloudflared 網址: {video_url}')
  else:
    print(
        '⚠️ 未設定 PUBLIC_BASE_URL：無法產生 LINE 影片氣泡。'
        '請先開 cloudflared 並把 https://... 填進 .env 後重開程式'
    )
    return ready, None, preview_url

  if not preview_url or not line_can_embed_image(preview_url):
    sibling_preview = ready.parent / 'fall_preview.jpg'
    sibling_original = ready.parent / 'fall_original.jpg'
    for candidate in (sibling_preview, sibling_original):
      if candidate.exists():
        preview_url = upload_image_for_line(candidate)
        if preview_url and line_can_embed_image(preview_url):
          break

  if not preview_url or not line_can_embed_image(preview_url):
    print('⚠️ 影片預覽圖無法用 ImgBB 產生，LINE 影片訊息可能失敗')
    return ready, video_url, None

  return ready, video_url, preview_url


def send_line_messages(messages: list[dict]) -> bool:
  if not _line_configured():
    print('❌ 未設定 LINE_CHANNEL_ACCESS_TOKEN / LINE_USER_ID，略過推播')
    return False

  url = 'https://api.line.me/v2/bot/message/push'
  headers = {
      'Content-Type': 'application/json',
      'Authorization': f'Bearer {LINE_CHANNEL_ACCESS_TOKEN}',
  }
  payload = {'to': LINE_USER_ID, 'messages': messages}

  try:
    print('正在嘗試連線至 LINE 伺服器...')
    res = requests.post(url, headers=headers, json=payload, timeout=10)
    if res.status_code == 200:
      print('✅ 成功發送 LINE 訊息！')
      return True
    print(f'❌ LINE 發送失敗 ({res.status_code}): {res.text}')
  except Exception as e:
    print(f'❌ LINE 發送時發生例外錯誤: {e}')
  return False


def send_fall_alert(
    message_text: str,
    original_path: Path,
    preview_path: Path,
    video_path: Path | None = None,
) -> None:
  """推播文字 + ImgBB 照片 + cloudflared 影片氣泡（可點開播放）。"""
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
      print(f'🖼️ 將以 LINE 圖片訊息發送: {original_url}')
    else:
      print(
          '⚠️ 照片網址人點得到，但 LINE 氣泡可能顯示空白，'
          '改只放文字連結（建議設定 IMGBB_API_KEY）'
      )
  else:
    print('⚠️ 無法上傳照片到圖床；照片已保存在本機。建議設定 IMGBB_API_KEY')

  if video_path and video_path.exists():
    _ready, video_url, video_preview = resolve_video_url(video_path, preview_url)
    if video_url and video_preview:
      messages.append(
          {
              'type': 'video',
              'originalContentUrl': video_url,
              'previewImageUrl': video_preview,
          }
      )
      messages[0]['text'] += f'\n🎬 影片連結:\n{video_url}'
      print(f'🎬 將以 LINE 影片氣泡發送: {video_url}')
    else:
      print(
          '⚠️ 無法建立 LINE 影片氣泡。請確認：'
          '1) cloudflared 有在跑  2) .env 的 PUBLIC_BASE_URL 是最新 https 網址  '
          '3) 程式已重開讀到新網址'
      )

  if len(messages) > 1:
    for i, msg in enumerate(messages[1:], start=1):
      print(f'📎 LINE 媒體[{i}]: {msg.get("type")} -> {msg.get("originalContentUrl")}')

  # LINE 單次最多 5 則
  send_line_messages(messages[:5])


def open_video_writer(
    event_dir: Path, frame_size: tuple[int, int]
) -> tuple[cv2.VideoWriter | None, Path | None]:
  """優先 mp4v；失敗再試 XVID avi。回傳 (writer, path)。"""
  event_dir.mkdir(parents=True, exist_ok=True)
  w, h = frame_size

  candidates = [
      (event_dir / 'clip.mp4', 'mp4v'),
      (event_dir / 'clip.avi', 'XVID'),
  ]
  for out_path, fourcc_name in candidates:
    fourcc = cv2.VideoWriter_fourcc(*fourcc_name)
    writer = cv2.VideoWriter(str(out_path), fourcc, CLIP_FPS, (w, h))
    if writer.isOpened():
      print(f'ℹ️ VideoWriter 使用: {out_path.name} ({fourcc_name})')
      return writer, out_path
    writer.release()
    if out_path.exists():
      out_path.unlink(missing_ok=True)

  print('❌ 無法建立 VideoWriter')
  return None, None


def start_suspect_event(first_frame: np.ndarray) -> dict:
  """疑似跌倒開始：建立暫存目錄、先寫入 pre-roll，再持續錄影。"""
  event_id = f'{_now_stamp()}_{uuid.uuid4().hex[:6]}'
  event_dir = PENDING_DIR / event_id
  event_dir.mkdir(parents=True, exist_ok=True)

  h, w = first_frame.shape[:2]
  writer, video_path = open_video_writer(event_dir, (w, h))

  if writer is not None:
    for buffered in list(pre_roll_buffer):
      if buffered.shape[1] != w or buffered.shape[0] != h:
        buffered = cv2.resize(buffered, (w, h))
      writer.write(buffered)
    writer.write(first_frame)

  original_path, preview_path = save_line_images(first_frame, 'suspect', event_dir)

  print(f'⚠️ 疑似跌倒，開始錄影事件: {event_id}')
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
  """使用者站起來 / 停止偵測：關閉錄影並刪除暫存。"""
  if not event:
    return

  writer = event.get('writer')
  if writer is not None:
    writer.release()
    event['writer'] = None

  event_dir = event.get('dir')
  if event_dir and Path(event_dir).exists():
    shutil.rmtree(event_dir, ignore_errors=True)
    print(f'🗑️ 已刪除疑似跌倒片段（{reason}）: {event.get("id")}')


def finalize_confirmed_event(event: dict, annotated_frame: np.ndarray) -> tuple[Path, Path, Path | None]:
  """確認跌倒：關閉錄影、更新照片、搬到 confirmed。"""
  writer = event.get('writer')
  if writer is not None:
    writer.release()
    event['writer'] = None

  # 以確認當下畫面更新要傳送的照片
  original_path, preview_path = save_line_images(annotated_frame, 'fall', event['dir'])
  event['photo_original'] = original_path
  event['photo_preview'] = preview_path

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

  confirmed_original = dest_dir / original_path.name
  confirmed_preview = dest_dir / preview_path.name
  confirmed_video = dest_dir / Path(video_path).name if video_path else None
  print(f'💾 已保留確認跌倒媒體: {dest_dir}')
  return confirmed_original, confirmed_preview, confirmed_video


def analyze_pose(result) -> tuple[bool, bool, bool, float, float, float] | None:
  """回傳 (is_fall, lateral, frontal, aspect_ratio, torso_height, avg_shoulder_y)。"""
  if len(result.boxes) == 0 or result.keypoints is None or len(result.keypoints.data) == 0:
    return None

  box = result.boxes.xywh[0]
  w, h = float(box[2]), float(box[3])
  if h <= 1e-6:
    return None
  aspect_ratio = round(w / h, 2)

  keypoints = result.keypoints.data[0]
  # COCO: 5/6 肩膀, 11/12 髖
  confs = [float(keypoints[i][2]) for i in (5, 6, 11, 12)]
  if min(confs) < 0.3:
    # 關鍵點太不可靠時，僅用寬高比做側向判斷
    is_fall_lateral = aspect_ratio > 1.1
    is_fall_frontal = False
    return (
        is_fall_lateral or is_fall_frontal,
        is_fall_lateral,
        is_fall_frontal,
        aspect_ratio,
        -1.0,
        -1.0,
    )

  l_shoulder_y = float(keypoints[5][1])
  r_shoulder_y = float(keypoints[6][1])
  l_hip_y = float(keypoints[11][1])
  r_hip_y = float(keypoints[12][1])

  avg_shoulder_y = (l_shoulder_y + r_shoulder_y) / 2
  avg_hip_y = (l_hip_y + r_hip_y) / 2
  torso_height = abs(avg_hip_y - avg_shoulder_y)

  is_fall_lateral = aspect_ratio > 1.1
  is_fall_frontal = (torso_height < 40) and (avg_shoulder_y > 220)
  is_currently_falling = is_fall_lateral or is_fall_frontal
  return (
      is_currently_falling,
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

      # 維護跌倒前緩衝（降採樣到 CLIP_FPS，避免記憶體爆）
      now = time.time()
      if now - last_buffer_time >= frame_interval:
        pre_roll_buffer.append(annotated_frame.copy())
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
            is_fall_frontal,
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
            else:
              if event is not None:
                append_event_frame(event, annotated_frame)

            elapsed_fall_time = current_time - fall_start_time

            if elapsed_fall_time >= FALL_CONFIRM_SECONDS:
              status_msg = (
                  f'--> 【🚨 確認為跌倒事件！已持續 {round(elapsed_fall_time, 1)}s】'
              )

              if (current_time - last_alert_time) > ALERT_COOLDOWN_SECONDS:
                print('🚨 觸發 LINE 發送機制！')
                with state_lock:
                  event_to_finalize = active_event
                  active_event = None

                original_path = preview_path = video_path = None
                if event_to_finalize is not None:
                  original_path, preview_path, video_path = finalize_confirmed_event(
                      event_to_finalize, annotated_frame.copy()
                  )
                else:
                  # 保底：至少存一張確認照片
                  stamp = _now_stamp()
                  dest = CONFIRMED_DIR / stamp
                  dest.mkdir(parents=True, exist_ok=True)
                  original_path, preview_path = save_line_images(
                      annotated_frame, 'fall', dest
                  )

                fall_type = '側向跌倒' if is_fall_lateral else '正面/背面跌倒'
                alert_msg = (
                    '【🚨 緊急安全通報】\n'
                    '樹莓派系統偵測到人體跌倒倒地！\n'
                    f'倒地型態: {fall_type}\n'
                    f'持續時間: {round(elapsed_fall_time, 1)} 秒\n'
                    '請立即確認現場狀況！'
                )
                send_fall_alert(alert_msg, original_path, preview_path, video_path)
                last_alert_time = current_time
                # 若人仍倒地，立刻開下一段錄影，供冷卻後再次通報使用
                with state_lock:
                  active_event = start_suspect_event(annotated_frame.copy())
            else:
              status_msg = (
                  '--> 【⚠️ 疑似跌倒，確認計時中...'
                  f' ({round(elapsed_fall_time, 1)}s)｜片段暫存中】'
              )
          else:
            # 站起來 / 恢復 → 刪除疑似片段
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
          status_msg = '--> 【骨架測試中（跌倒偵測已關閉）】'

        torso_txt = 'n/a' if torso_height < 0 else str(round(torso_height, 1))
        shoulder_txt = 'n/a' if avg_shoulder_y < 0 else str(round(avg_shoulder_y, 1))
        print(
            f'寬高比: {aspect_ratio} | 軀幹長: {torso_txt} |'
            f' 肩膀Y: {shoulder_txt} {status_msg}'
        )
      else:
        # 畫面中沒人：若正在疑似跌倒，視為中斷並刪除
        with state_lock:
          detecting = is_detecting
          event = active_event
        if detecting and event is not None:
          with state_lock:
            to_discard = active_event
            active_event = None
          discard_suspect_event(to_discard, '畫面中失去人體')
          fall_start_time = None
        elif not detecting:
          status_msg = '--> 【骨架測試中（跌倒偵測已關閉）】'

      ret, buffer = cv2.imencode('.jpg', annotated_frame)
      if not ret:
        continue

      frame_bytes = buffer.tobytes()
      yield (
          b'--frame\r\n'
          b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
      )

    except Exception as e:
      print(f'發生例外錯誤: {e}')
      time.sleep(0.1)
      continue


@app.route('/media/<path:subpath>')
def media_files(subpath: str):
  """提供 events 內影片給 LINE 下載（需搭配 PUBLIC_BASE_URL / cloudflared）。"""
  target = (BASE_DIR / subpath).resolve()
  events_root = (BASE_DIR / 'events').resolve()
  if not str(target).startswith(str(events_root)) or not target.is_file():
    return 'Not Found', 404

  suffix = target.suffix.lower()
  mime = {
      '.jpg': 'image/jpeg',
      '.jpeg': 'image/jpeg',
      '.png': 'image/png',
      '.mp4': 'video/mp4',
      '.avi': 'video/x-msvideo',
  }.get(suffix, 'application/octet-stream')
  # conditional=True 支援 Range，較利於 LINE 拉影片
  return send_file(target, mimetype=mime, conditional=True, as_attachment=False)


@app.route('/start_detection')
def start_detection():
  global is_detecting, fall_start_time, active_event
  with state_lock:
    is_detecting = True
    fall_start_time = None
    if active_event is not None:
      discard_suspect_event(active_event, '重新開始偵測')
      active_event = None
  print('▶️ 網頁觸發：開啟跌倒偵測')
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
  print('⏸️ 網頁觸發：停止跌倒偵測')
  return 'STOPPED'


@app.route('/')
def index():
  html_content = """
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <title>YOLO-pose 跌倒通報系統</title>
        <style>
            body { text-align: center; background-color: #222; color: white; font-family: Arial, sans-serif; }
            .btn {
                padding: 12px 24px;
                font-size: 18px;
                font-weight: bold;
                margin: 10px;
                border: none;
                border-radius: 8px;
                cursor: pointer;
            }
            .btn-start { background-color: #28a745; color: white; }
            .btn-stop { background-color: #dc3545; color: white; }
            .btn:hover { opacity: 0.8; }
            #status-text { font-size: 20px; font-weight: bold; color: #ffc107; margin-top: 10px; }
            .hint { color: #bbb; font-size: 14px; max-width: 640px; margin: 12px auto; line-height: 1.5; }
            img { max-width: 100%; height: auto; }
        </style>
        <script>
            function startDetection() {
                fetch('/start_detection')
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status-text').innerText = "當前狀態：🟢 跌倒偵測中 (會發送 LINE)";
                    document.getElementById('status-text').style.color = "#28a745";
                });
            }

            function stopDetection() {
                fetch('/stop_detection')
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status-text').innerText = "當前狀態：🟡 僅骨架測試 (不會發送 LINE)";
                    document.getElementById('status-text').style.color = "#ffc107";
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
          疑似跌倒會先暫存短片；若在確認時間內站起來會自動刪除。
          確認跌倒後會推播 LINE 文字＋照片（有設定公開 HTTPS 網址時可再附影片）。
        </p>
        <img src="/video_feed" width="640" alt="video feed">
    </body>
    </html>
    """
  return render_template_string(html_content)


@app.route('/video_feed')
def video_feed():
  return Response(
      generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame'
  )


if __name__ == '__main__':
  env_path = BASE_DIR / '.env'
  print(f'📂 .env 路徑: {env_path} （存在: {env_path.exists()}）')
  if _line_configured():
    print(
        f'🔐 LINE 設定已載入｜Token 長度={len(LINE_CHANNEL_ACCESS_TOKEN)}｜'
        f'UserID 開頭={LINE_USER_ID[:4]}...'
    )
  else:
    print('⚠️ 尚未讀到 LINE 憑證：請確認 .env 已存檔，且與此 .py 同一資料夾後重新啟動')
  if IMGBB_API_KEY:
    print(
        f'☁️ ImgBB 已設定｜照片約 {IMGBB_EXPIRATION_SECONDS / 60:g} 分鐘後由圖床自動刪除'
        '（LINE 聊天多半仍可看已快取的圖）'
    )
  if not PUBLIC_BASE_URL and not IMGBB_API_KEY:
    print(
        '⚠️ 未設定 IMGBB_API_KEY：'
        '確認跌倒時仍會存本機照片/影片，但 LINE 圖片氣泡可能無法顯示'
    )
  elif PUBLIC_BASE_URL and not PUBLIC_BASE_URL.startswith('https://'):
    print('⚠️ PUBLIC_BASE_URL 必須是 https:// ，LINE 才收得到媒體')

  print(f'伺服器啟動：http://{HOST}:{PORT}')
  app.run(host=HOST, port=PORT, debug=False, threaded=True)
