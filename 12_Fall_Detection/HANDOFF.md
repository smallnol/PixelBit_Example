# 交接說明｜樹莓派跌倒偵測（可貼到新聊天）

把下面整段複製到另一個 Cursor／Agent 聊天即可。

---

## 這專案在做什麼

在 **Raspberry Pi 5** 上跑 **YOLOv8-pose**，用鏡頭即時看人姿勢。  
預設只顯示骨架；網頁按「開啟跌倒偵測」後，若判定跌倒並持續約 **10 秒**，就推播 **LINE**：

1. 文字警報  
2. **照片**（ImgBB，可約 10 分鐘自動刪）  
3. **短片**（含跌倒前約 30 秒循環緩衝 + 倒地過程；H.264 mp4）

未確認的疑似事件（中途站起來／關偵測／畫面沒人）會自動刪暫存。

## 程式位置

| 項目 | 值 |
|------|-----|
| Repo | `https://github.com/smallnol/PixelBit_Example` |
| Branch / PR | `cursor/fall-detection-line-media-5500`／[PR #1](https://github.com/smallnol/PixelBit_Example/pull/1) |
| 主程式 | `12_Fall_Detection/fall_detection.py`（約 1100+ 行） |
| 設定範例 | `12_Fall_Detection/.env.example` |
| Pi 指令速查 | `12_Fall_Detection/PI_COMMANDS.md` |
| 相依套件 | `12_Fall_Detection/requirements.txt`（flask, opencv, ultralytics, requests, dotenv, numpy） |

Pi 上實際跑的路徑通常是：`~/test/fall_detection.py` + `~/test/.env` + venv `yolo_env`。

## 核心流程

```
鏡頭 → YOLO-pose 骨架
     → 永遠保留最近 ~30s JPEG 循環緩衝
     → 開啟偵測後：姿勢異常 → pending 錄影（先寫入前情緩衝）
     → 異常滿 10s → confirmed + LINE（圖+影）
     → 未滿 10s 就恢復 → 刪 pending
```

網頁：`http://樹莓派IP:5000`（Flask MJPEG 預覽 + 開關按鈕）。

## 跌倒判定（重點）

函式：`analyze_pose()`  

- **不要**只靠「框寬高比 > 1.1」（舊版會把桌前側坐當跌倒）  
- **桌前坐姿排除**：手腕像在桌面 + 頭高於手腕 + 肩膀偏畫面上方 → 不報  
- **會報**：側倒／半倒／躺平，以及 **蹲下**（膝蓋靠近髖 + 人偏低）  
- 靈敏度：`.env` 的 `FALL_SENSITIVITY`（預設 `1.45`；越大越敏感，`1.6~1.8` 更易抓蹲下；誤報就降到 `1.2`）

確認秒數：`FALL_CONFIRM_SECONDS = 10`；冷卻：`ALERT_COOLDOWN_SECONDS = 30`。

## LINE 媒體策略（現況）

LINE 的 image／video 氣泡需要 **公開 HTTPS 直連**。

| 類型 | 作法 |
|------|------|
| 照片 | **ImgBB**（必備 `IMGBB_API_KEY`；`IMGBB_EXPIRATION_SECONDS` 預設 600） |
| 影片上傳順序 | litterbox → catbox → **uguu** → pomf → cloudflared(`PUBLIC_BASE_URL`) |
| 已知雷點 | litterbox 常 **HTTP 500**；catbox 常 **412 Invalid uploader**（對方擋 IP，不是參數寫錯）→ 目前實戰靠 **uguu／pomf** |
| 影片編碼 | `ensure_line_h264_mp4()`（ffmpeg → H.264）才給 LINE |

相關函式：`upload_video_for_line`、`upload_to_imgbb`、`send_fall_alert`、`resolve_video_url`。

## `.env` 最少要有

```bash
LINE_CHANNEL_ACCESS_TOKEN=...
LINE_USER_ID=...
IMGBB_API_KEY=...
```

常用可選：

```bash
FALL_SENSITIVITY=1.45
ROLLING_BUFFER_SECONDS=30
LITTERBOX_KEEP=1h
# PUBLIC_BASE_URL=https://xxxx.trycloudflare.com   # 圖床全掛時最後備援
```

## Pi 更新／啟動（常打）

```bash
cd ~/test
curl -fsSL -o fall_detection.py \
  https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-detection-line-media-5500/12_Fall_Detection/fall_detection.py

source yolo_env/bin/activate
python fall_detection.py
```

## 已知限制／注意

- 桌前低頭打字不應警報；若又誤報 → 降 `FALL_SENSITIVITY`  
- 蹲下抓不到 → 升 `FALL_SENSITIVITY`  
- litterbox／catbox 不穩時，影片走 uguu（短效自動刪）或 pomf  
- `README.md` 部分內容偏舊（還寫 3 秒前情／主要靠 cloudflared）；以本 HANDOFF 與程式為準  
- 使用者之後可能想把這套與 UWB 跟隨車（`github.com/smallnol/uwb-follow-car`）合併；**目前跌倒偵測獨立優先**

## 給下一個 Agent 的建議任務方向（若被要求改）

1. 先讀 `fall_detection.py` 的 `analyze_pose` / `upload_video_for_line` / `send_fall_alert`  
2. 改判定或上傳後，push 到 branch `cursor/fall-detection-line-media-5500`，並更新 `PI_COMMANDS.md`  
3. 給使用者 Pi 的 `curl` 更新指令，不要假設他會 git pull 整 repo  

---
