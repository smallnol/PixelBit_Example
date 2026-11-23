# 跌倒偵測 + 視覺跟隨（打包版）

把最新 `12_Fall_Detection` 核心與 `13_Vision_UWB_Follow` 合成一支程式（**暫不含 MaUWB AOA**）。

`line_media.py` 同步自分支 `cursor/fall-detection-line-media-5500`（含 `FALL_SENSITIVITY`、桌前坐姿排除、uguu／pomf 影片備援）。

## 功能

| 功能 | 說明 |
|------|------|
| 視覺跟隨 | YOLO 判左／中／右；ESP32-UWB 距離；經 USB 轉發命令到 UNO |
| 跌倒偵測 | 約 10 秒確認；桌前坐姿不報；蹲下／半倒／躺平可報；LINE 文字＋照片＋影片 |
| 跌倒急停 | 確認跌倒 → 送 `X` 緊急煞停，並可自動關閉跟隨 |

網頁按鈕可**分別**開關跟隨／跌倒偵測。

## 樹莓派第一次安裝

```bash
mkdir -p ~/fall_follow && cd ~/fall_follow
curl -L -o fall_follow_car.py "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-combine-1509/14_Fall_Follow_Car/fall_follow_car.py"
curl -L -o line_media.py "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-combine-1509/14_Fall_Follow_Car/line_media.py"
curl -L -o requirements.txt "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-combine-1509/14_Fall_Follow_Car/requirements.txt"
curl -L -o .env.example "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-combine-1509/14_Fall_Follow_Car/.env.example"

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
nano .env   # 填 LINE / IMGBB / serial
mkdir -p events/pending events/confirmed
```

## 每次更新／執行

```bash
cd ~/fall_follow
source .venv/bin/activate
curl -L -o fall_follow_car.py "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-combine-1509/14_Fall_Follow_Car/fall_follow_car.py"
curl -L -o line_media.py "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-combine-1509/14_Fall_Follow_Car/line_media.py"
python fall_follow_car.py
```

瀏覽器：`http://樹莓派IP:5001`

## 接線（與目前跟隨相同）

- 車上 ESP32 USB → Pi（`/dev/ttyUSB0`）
- ESP32 **GPIO17 → UNO D10**，GND 共地
- 18650 只接 BMP73T104 馬達電；UNO 用行充 5V；**電池 GND 共 UNO GND**
- UNO 燒 `uwb-follow-car` 的 `uno_car`（Pi-slave）

## 檔案

- `fall_follow_car.py`：主程式（Flask + 控制迴圈）
- `line_media.py`：跌倒錄影／LINE 上傳
- `test_motor_relay.py`：可選，測馬達轉發
