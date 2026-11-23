# 跌倒偵測 + 視覺跟隨（整合版）

把最新跌倒核心、視覺／UWB 跟隨、**5×VL53L0X 避障**、**特定人鎖定**、**網頁遙控／儀表板**合成一支程式（**暫不含 MaUWB AOA**）。

## 功能一覽

| 功能 | 說明 |
|------|------|
| 視覺跟隨 | ByteTrack 鎖定目標 → 左右置中；UWB 距離 → 前進／停 |
| 特定人鎖定 | 網頁鎖中央／舉手 2 秒；HSV Re-ID 抗遮擋換 ID |
| 跌倒偵測 | 只分析鎖定目標；約 10 秒確認；LINE 文字＋照片＋影片 |
| 跌倒急停 | 確認 → `X`；可自動關跟隨 |
| 底層避障 | UNO 讀 5×VL53；前進遇障強制 L／R／S；`X` 最高優先 |
| 網頁儀表板 | 手動遙控、UWB／ToF 即時顯示、跌倒警報橫幅 |

網頁：`http://樹莓派IP:5001`

## 韌體（必燒）

見 [`firmware/README.md`](firmware/README.md)：

- `firmware/uno_car_vl53/` — UNO + VL53 + Soft-start
- `firmware/uwb_car_responder/` — ESP32 距離 `D:` + 轉發命令 + 橋接 `O:`

新增接線：**UNO D11 → ESP32 GPIO16**（避障封包回傳）。

## 樹莓派安裝

```bash
mkdir -p ~/fall_follow && cd ~/fall_follow
curl -L -o fall_follow_car.py "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/fall_follow_car.py"
curl -L -o line_media.py "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/line_media.py"
curl -L -o requirements.txt "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/requirements.txt"
curl -L -o .env.example "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/.env.example"

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
nano .env
mkdir -p events/pending events/confirmed
python fall_follow_car.py
```

## 操作順序（建議）

1. 開網頁預覽，確認 UWB／ToF 有數值  
2. **鎖定畫面中央目標**（或舉手 2 秒）→ 綠框 `TARGET LOCKED`  
3. 開啟跟隨／跌倒偵測  
4. 跟隨 OFF 時可用手動方向鍵；跟隨 ON 時方向鍵禁用（急停仍可用）

## 檔案

- `PROJECT_HANDBOOK.md` — 專題說明書
- `fall_follow_car.py` — 主程式
- `line_media.py` — 跌倒核心
- `firmware/` — UNO／ESP32 韌體
- `test_motor_relay.py` — 可選馬達轉發測試
