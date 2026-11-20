# 樹莓派常用指令（跌倒偵測）

在 Pi 的 `~/test` 目錄操作。

## 1) 從 GitHub 更新程式

```bash
cd ~/test
curl -fsSL -o fall_detection.py \
  https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-detection-line-media-5500/12_Fall_Detection/fall_detection.py
```

## 2) 啟動程式（依你的 venv 名稱調整）

```bash
cd ~/test
source yolo_env/bin/activate
python fall_detection.py
```

## 3) 另開一個 terminal：cloudflared（影片最後備援時才需要）

```bash
cloudflared tunnel --url http://127.0.0.1:5000
```

把印出的 `https://xxxx.trycloudflare.com` 寫進 `~/test/.env` 的 `PUBLIC_BASE_URL=`，再重開程式。

## 4) 預覽網頁

瀏覽器開：`http://樹莓派IP:5000`

- 先骨架測試 → 再按「開啟跌倒偵測」
- 坐著不應警報；真的躺下約 10 秒才通報 LINE

## 5) `.env` 最少要有

```bash
LINE_CHANNEL_ACCESS_TOKEN=...
LINE_USER_ID=...
IMGBB_API_KEY=...
```

可選：

```bash
LITTERBOX_KEEP=1h
ROLLING_BUFFER_SECONDS=30
# PUBLIC_BASE_URL=https://xxxx.trycloudflare.com
```
