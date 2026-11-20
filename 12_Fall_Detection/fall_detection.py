"""
樹莓派 5｜YOLO-pose 跌倒偵測 + LINE 文字/照片/影片通報

流程：
1. 預設僅骨架預覽（不警報）
2. 鏡頭持續保留最近約 30 秒畫面（行車紀錄器式循環緩衝；逾時自動丟掉）
3. 開啟偵測後，若姿勢異常 → 用「跌倒前 30 秒 + 倒地過程」開始組影片
4. 持續異常滿 FALL_CONFIRM_SECONDS → 確認跌倒：推播 LINE（ImgBB 照片 + 短效影片圖床）
5. 若在確認前站起來 → 自動刪除疑似短片與暫存照片
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
# 行車紀錄器式：持續保留最近 N 秒；沒跌倒就隨時間自動覆蓋刪除
ROLLING_BUFFER_SECONDS = float(os.getenv('ROLLING_BUFFER_SECONDS', '30'))
# 跌倒靈敏度：1.0=偏嚴（幾乎要躺平），預設 1.45 可抓蹲下／半倒；越大越敏感
try:
  FALL_SENSITIVITY = float(os.getenv('FALL_SENSITIVITY', '1.45'))
except ValueError:
  FALL_SENSITIVITY = 1.45
FALL_SENSITIVITY = max(0.8, min(FALL_SENSITIVITY, 2.5))
CLIP_FPS = 10.0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# litterbox 只支援 1h / 12h / 24h / 72h（沒有 10 分鐘）；預設用最短 1h
LITTERBOX_KEEP = os.getenv('LITTERBOX_KEEP', '1h').strip() or '1h'
if LITTERBOX_KEEP not in {'1h', '12h', '24h', '72h'}:
  print(f'⚠️ LITTERBOX_KEEP={LITTERBOX_KEEP} 無效，改用 1h')
  LITTERBOX_KEEP = '1h'

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

# 循環緩衝：存 JPEG，比存原始幀省記憶體；超過时长自動丟掉最舊畫面
rolling_maxlen = max(1, int(ROLLING_BUFFER_SECONDS * CLIP_FPS))
rolling_buffer: deque[bytes] = deque(maxlen=rolling_maxlen)
print(
    f'🎥 循環錄影緩衝約 {ROLLING_BUFFER_SECONDS:g} 秒'
    f'（最多 {rolling_maxlen} 幀 @ {CLIP_FPS:g} fps；無跌倒會自動覆蓋）'
)
print(
    f'☁️ 影片上傳優先 litterbox（{LITTERBOX_KEEP}）；'
    '若掛掉則改 uguu/pomf（約數小時自動刪）'
)
print(
    f'🎯 跌倒靈敏度 FALL_SENSITIVITY={FALL_SENSITIVITY:g} '
    '（越大越敏感；蹲下也較容易觸發，桌前坐姿仍會排除）'
)


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
          'uguu.se/',
          'pomf2.lain.la/',
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


def _browser_ua() -> str:
  return (
      'Mozilla/5.0 (X11; Linux aarch64) AppleWebKit/537.36 '
      '(KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36'
  )


def line_can_embed_video(url: str | None) -> bool:
  """LINE 影片氣泡可用的直連 HTTPS mp4 主機。"""
  if not url:
    return False
  u = url.lower()
  if not u.startswith('https://'):
    return False
  hosts = (
      'litter.catbox.moe/',
      'files.catbox.moe/',
      'uguu.se/',
      'pomf2.lain.la/',
      'lain.la/',
      'trycloudflare.com/',
  )
  if any(h in u for h in hosts):
    return True
  # 自架 cloudflared / PUBLIC_BASE_URL：路徑需是 .mp4
  if PUBLIC_BASE_URL and u.startswith(PUBLIC_BASE_URL.lower()) and u.endswith('.mp4'):
    return True
  return False


def upload_to_catbox(file_path: Path) -> str | None:
  """
  litterbox 姊妹站。目前常回 412 Invalid uploader（對方擋 IP／VPN／機房網段），
  不是本機參數能修好的；失敗就改走 uguu/pomf。
  """
  headers = {
      'User-Agent': _browser_ua(),
      'Referer': 'https://catbox.moe/',
  }
  data = {'reqtype': 'fileupload'}
  userhash = os.getenv('CATBOX_USERHASH', '').strip()
  if userhash:
    data['userhash'] = userhash
  try:
    with open(file_path, 'rb') as f:
      res = requests.post(
          'https://catbox.moe/user/api.php',
          data=data,
          files={'fileToUpload': (file_path.name, f, 'application/octet-stream')},
          headers=headers,
          timeout=180,
      )
    text = (res.text or '').strip()
    print(f'☁️ catbox 回應: status={res.status_code} body={text[:200]!r}')
    if res.status_code == 200 and text.startswith('https://'):
      print(f'☁️ catbox 上傳成功: {text}')
      return text
    if res.status_code == 412 or 'invalid uploader' in text.lower():
      print('⚠️ catbox 拒絕上傳（Invalid uploader＝對方擋來源 IP，非程式錯誤）')
    else:
      print(f'❌ catbox 上傳失敗: {res.status_code} {text[:300]!r}')
  except Exception as e:
    print(f'❌ catbox 上傳例外: {e}')
  return None


def upload_to_litterbox(file_path: Path, keep: str | None = None) -> str | None:
  """影片理想主力（會過期）；目前常 500，失敗就改走其他圖床。"""
  keep = keep or LITTERBOX_KEEP
  headers = {
      'User-Agent': _browser_ua(),
      'Referer': 'https://litterbox.catbox.moe/',
  }
  for attempt in range(1, 3):
    try:
      with open(file_path, 'rb') as f:
        res = requests.post(
            'https://litterbox.catbox.moe/resources/internals/api.php',
            data={'reqtype': 'fileupload', 'time': keep},
            files={'fileToUpload': (file_path.name, f, 'application/octet-stream')},
            headers=headers,
            timeout=180,
        )
      text = (res.text or '').strip()
      print(f'☁️ litterbox 回應: status={res.status_code} body={text[:200]!r}')
      if res.status_code == 200 and text.startswith('https://'):
        print(f'☁️ litterbox 上傳成功（{keep} 後刪）: {text}')
        return text
      print(f'❌ litterbox 上傳失敗 (第 {attempt}/2): {res.status_code}')
      if res.status_code == 500 and attempt == 1:
        time.sleep(1.5)
        continue
    except Exception as e:
      print(f'❌ litterbox 上傳例外: {e}')
  return None


def upload_to_uguu(file_path: Path) -> str | None:
  """uguu.se：直連 video/mp4，檔案約數小時後自動刪（適合短效通報）。"""
  headers = {'User-Agent': _browser_ua()}
  try:
    with open(file_path, 'rb') as f:
      res = requests.post(
          'https://uguu.se/upload',
          files={'files[]': (file_path.name, f, 'video/mp4')},
          headers=headers,
          timeout=180,
      )
    data = res.json()
    if res.status_code == 200 and data.get('success'):
      files = data.get('files') or []
      if files and isinstance(files[0], dict):
        url = (files[0].get('url') or '').strip()
        if url.startswith('https://'):
          print(f'☁️ uguu 上傳成功（短效自動刪）: {url}')
          return url
    print(f'❌ uguu 上傳失敗: {res.status_code} {str(data)[:300]!r}')
  except Exception as e:
    print(f'❌ uguu 上傳例外: {e}')
  return None


def upload_to_pomf(file_path: Path) -> str | None:
  """pomf.lain.la：直連 video/mp4 備援。"""
  headers = {'User-Agent': _browser_ua()}
  try:
    with open(file_path, 'rb') as f:
      res = requests.post(
          'https://pomf.lain.la/upload.php',
          files={'files[]': (file_path.name, f, 'video/mp4')},
          headers=headers,
          timeout=180,
      )
    data = res.json()
    if res.status_code == 200 and data.get('success'):
      files = data.get('files') or []
      if files and isinstance(files[0], dict):
        url = (files[0].get('url') or '').strip()
        if url.startswith('https://'):
          print(f'☁️ pomf 上傳成功: {url}')
          return url
    print(f'❌ pomf 上傳失敗: {res.status_code} {str(data)[:300]!r}')
  except Exception as e:
    print(f'❌ pomf 上傳例外: {e}')
  return None


def upload_video_for_line(video_path: Path) -> str | None:
  """
  影片上傳順序：
  1) litterbox（會過期；目前常 500）
  2) catbox（目前常 412 Invalid uploader＝對方擋 IP）
  3) uguu（短效自動刪，直連 mp4）
  4) pomf（直連 mp4 備援）
  5) cloudflared PUBLIC_BASE_URL（需隧道網址新鮮）
  """
  for uploader in (
      upload_to_litterbox,
      upload_to_catbox,
      upload_to_uguu,
      upload_to_pomf,
  ):
    url = uploader(video_path)
    if url and line_can_embed_video(url):
      return url

  tunnel = public_media_url(video_path)
  if tunnel and line_can_embed_video(tunnel):
    print(f'🔗 圖床皆失敗，改用 cloudflared 備援: {tunnel}')
    return tunnel

  print(
      '⚠️ 無法取得 LINE 可播影片網址：'
      'litterbox=500 / catbox=412 時請依賴 uguu/pomf；'
      '或設定最新 PUBLIC_BASE_URL'
  )
  return None


def decode_rolling_frame(jpg_bytes: bytes, frame_size: tuple[int, int]) -> np.ndarray | None:
  arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
  frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
  if frame is None:
    return None
  w, h = frame_size
  if frame.shape[1] != w or frame.shape[0] != h:
    frame = cv2.resize(frame, (w, h))
  return frame


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
      '-profile:v',
      'baseline',
      '-level',
      '3.0',
      '-pix_fmt',
      'yuv420p',
      '-vf',
      'scale=640:-2',
      '-r',
      '15',
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
  - 照片／預覽：ImgBB（可直接顯示）
  - 影片本體：catbox（點氣泡播放；封面仍用 ImgBB）
  """
  ready = ensure_line_h264_mp4(video_path)
  if ready is None:
    return None, None, preview_url

  try:
    size_mb = ready.stat().st_size / (1024 * 1024)
    print(f'📦 影片大小約 {size_mb:.2f} MB')
  except OSError:
    pass

  video_url = upload_video_for_line(ready)
  if not video_url:
    print('⚠️ litterbox/catbox 都無法提供影片網址')
    return ready, None, preview_url
  print(f'🎬 影片網址: {video_url}')

  if not preview_url or not line_can_embed_image(preview_url):
    sibling_preview = ready.parent / 'fall_preview.jpg'
    sibling_original = ready.parent / 'fall_original.jpg'
    for candidate in (sibling_preview, sibling_original):
      if candidate.exists():
        preview_url = upload_image_for_line(candidate)
        if preview_url and line_can_embed_image(preview_url):
          break

  if not preview_url or not line_can_embed_image(preview_url):
    print('⚠️ 影片預覽圖需要 ImgBB；請確認 IMGBB_API_KEY')
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
  """推播文字 + ImgBB 照片 + litterbox/catbox 影片氣泡（可點開播放）。"""
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
    if (
        video_url
        and video_preview
        and line_can_embed_video(video_url)
        and line_can_embed_image(video_preview)
    ):
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
          '⚠️ 無法建立可播放的 LINE 影片氣泡。'
          'litterbox/catbox 目前常掛；已改試 uguu/pomf。'
          '照片仍會正常發送。'
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
  """疑似跌倒開始：寫入循環緩衝（約跌倒前 30 秒），再繼續錄倒地過程。"""
  event_id = f'{_now_stamp()}_{uuid.uuid4().hex[:6]}'
  event_dir = PENDING_DIR / event_id
  event_dir.mkdir(parents=True, exist_ok=True)

  h, w = first_frame.shape[:2]
  writer, video_path = open_video_writer(event_dir, (w, h))

  pre_count = 0
  if writer is not None:
    # 先倒出「跌倒前」循環緩衝，讓家屬看到前因
    for jpg in list(rolling_buffer):
      frame = decode_rolling_frame(jpg, (w, h))
      if frame is None:
        continue
      writer.write(frame)
      pre_count += 1
    writer.write(first_frame)

  original_path, preview_path = save_line_images(first_frame, 'suspect', event_dir)
  pre_seconds = pre_count / CLIP_FPS if CLIP_FPS else 0
  print(
      f'⚠️ 疑似跌倒，開始錄影事件: {event_id}'
      f'｜已附上跌倒前約 {pre_seconds:.1f}s 循環緩衝'
  )
  return {
      'id': event_id,
      'dir': event_dir,
      'writer': writer,
      'video_path': video_path,
      'photo_original': original_path,
      'photo_preview': preview_path,
      'frame_size': (w, h),
      'pre_roll_frames': pre_count,
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


def _kp_xy(keypoints, idx: int, min_conf: float = 0.25) -> tuple[float, float] | None:
  """讀取 COCO keypoint；信心不足則回 None。"""
  x, y, c = float(keypoints[idx][0]), float(keypoints[idx][1]), float(keypoints[idx][2])
  if c < min_conf:
    return None
  return x, y


def _avg_point(
    a: tuple[float, float] | None,
    b: tuple[float, float] | None,
) -> tuple[float, float] | None:
  if a and b:
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
  return a or b


def analyze_pose(result) -> tuple[bool, bool, bool, float, float, float] | None:
  """
  回傳 (is_fall, lateral, frontal, aspect_ratio, torso_height, avg_shoulder_y)。

  目標：
  - 桌前坐姿／低頭打字 → 不報
  - 蹲下、半倒、躺平 → 依 FALL_SENSITIVITY 可報（預設偏敏感）
  """
  if len(result.boxes) == 0 or result.keypoints is None or len(result.keypoints.data) == 0:
    return None

  box = result.boxes.xywh[0]
  cx, cy = float(box[0]), float(box[1])
  w, h = float(box[2]), float(box[3])
  if h <= 1e-6:
    return None
  aspect_ratio = round(w / h, 2)
  sens = FALL_SENSITIVITY

  keypoints = result.keypoints.data[0]
  # COCO: 0鼻 5/6肩 9/10腕 11/12髖 13/14膝
  nose = _kp_xy(keypoints, 0)
  l_sh = _kp_xy(keypoints, 5)
  r_sh = _kp_xy(keypoints, 6)
  l_hip = _kp_xy(keypoints, 11)
  r_hip = _kp_xy(keypoints, 12)
  l_wr = _kp_xy(keypoints, 9)
  r_wr = _kp_xy(keypoints, 10)
  l_knee = _kp_xy(keypoints, 13)
  r_knee = _kp_xy(keypoints, 14)

  shoulder = _avg_point(l_sh, r_sh)
  hip = _avg_point(l_hip, r_hip)
  wrist = _avg_point(l_wr, r_wr)
  knee = _avg_point(l_knee, r_knee)

  # 關鍵點不可靠：寬扁 + 人在畫面偏下才當側摔（避免桌前側坐）
  if shoulder is None or hip is None:
    box_low = cy > (FRAME_HEIGHT * (0.58 - 0.08 * (sens - 1.0)))
    is_fall_lateral = aspect_ratio > (1.30 - 0.12 * (sens - 1.0)) and box_low
    return (is_fall_lateral, is_fall_lateral, False, aspect_ratio, -1.0, -1.0)

  avg_shoulder_y = shoulder[1]
  avg_hip_y = hip[1]
  torso_height = abs(avg_hip_y - avg_shoulder_y)
  head_y = nose[1] if nose else avg_shoulder_y
  head_to_hip = abs(avg_hip_y - head_y)

  # --- 桌前坐姿排除（只擋「操作桌面」，不擋蹲下）---
  desk_pose = False
  if wrist is not None:
    wrist_near_hip = abs(wrist[1] - avg_hip_y) < max(80.0, 0.40 * h)
    head_above_wrist = (wrist[1] - head_y) > max(35.0, 0.15 * h)
    # 肩膀仍偏畫面上方＝坐著，不是蹲在地上
    shoulders_mid_high = avg_shoulder_y < (FRAME_HEIGHT * 0.42)
    desk_pose = wrist_near_hip and head_above_wrist and shoulders_mid_high

  if desk_pose:
    return (False, False, False, aspect_ratio, torso_height, avg_shoulder_y)

  # --- 跌倒／蹲下條件（數值隨 FALL_SENSITIVITY 放寬）---
  # 軀幹「夠扁」門檻：敏感度越高，允許越大的 torso_height 仍算倒地
  torso_limit = max(70.0, 0.36 * h) * sens
  head_hip_limit = max(95.0, 0.48 * h) * sens
  shoulder_low_y = FRAME_HEIGHT * (0.40 - 0.06 * (sens - 1.0))
  aspect_lateral = 1.28 - 0.15 * (sens - 1.0)

  torso_compressed = torso_height < torso_limit
  head_compressed = head_to_hip < head_hip_limit
  body_low = avg_shoulder_y > shoulder_low_y

  knees_tucked = False
  if knee is not None:
    knees_tucked = abs(knee[1] - avg_hip_y) < (max(55.0, 0.32 * h) * sens)

  # 側摔：框偏扁 + 人偏低 + 軀幹有壓縮
  is_fall_lateral = (
      aspect_ratio > aspect_lateral and body_low and torso_compressed
  )
  # 正面倒／半倒：人偏低 + 頭髖距離壓縮
  is_fall_frontal = (
      body_low and head_compressed and torso_compressed and aspect_ratio < 1.85
  )
  # 蹲下：膝蓋靠近髖 + 肩膀偏低（不算桌前坐姿）
  is_fall_crouch = body_low and knees_tucked and head_compressed
  if is_fall_crouch:
    is_fall_frontal = True

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

      # 行車紀錄器式循環緩衝（JPEG；逾時自動覆蓋最舊畫面）
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
                    f'影片含跌倒前約 {ROLLING_BUFFER_SECONDS:g} 秒前情＋倒地過程\n'
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
          鏡頭會持續保留最近約 30 秒畫面（無跌倒自動覆蓋）。
          確認跌倒後，LINE 會收到照片＋含前情的短片；未確認的暫存會刪除。
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
    )
  else:
    print('⚠️ 未設定 IMGBB_API_KEY：LINE 照片氣泡可能無法顯示')

  print('🎬 影片將優先上傳 catbox；照片維持 ImgBB 直接顯示')
  if PUBLIC_BASE_URL:
    print(f'ℹ️ PUBLIC_BASE_URL 僅作影片備援: {PUBLIC_BASE_URL}')

  print(f'伺服器啟動：http://{HOST}:{PORT}')
  app.run(host=HOST, port=PORT, debug=False, threaded=True)
