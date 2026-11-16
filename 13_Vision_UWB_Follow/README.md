# 視覺 + UWB 跟隨小車（樹莓派 5）

Pi 當大腦：**YOLO-pose 判斷人在畫面左／中／右**，**UWB 判斷距離**，再對 UNO 下馬達命令。

跌倒偵測仍在 `12_Fall_Detection/`（本資料夾先專心做跟隨）。

## 行為

| 條件 | 馬達命令 |
|------|----------|
| 跟隨關閉 | `S` 停止 |
| 畫面沒人 | `Z` 自轉搜尋 |
| 人在畫面左側 | `L` 左轉直到置中 |
| 人在畫面右側 | `R` 右轉直到置中 |
| 置中且距離 ≥ 2.0 m | `F` 前進 |
| 置中且距離 < 1.8 m | `S` 停止 |
| 置中且 1.8～2.0 m | 維持上一前進／停止（緩衝帶） |
| 置中但 UWB 逾時 | `S` 停止 |

## 硬體接線（建議）

```
人身上 ESP32-UWB (Tag)  --UWB-->  車上 ESP32-UWB (Responder)
                                         │ USB
                                         ▼
                                    樹莓派 5
                                    ├─ 相機（跟車頭同向）
                                    └─ USB-TTL TX ──► UNO D10
                                       共地 GND ──► UNO GND

UNO + BMP73T104：只聽 Pi 命令，不再直接讀 UWB
```

1. **車上 ESP32 USB → 樹莓派**（預設 `/dev/ttyUSB0`）  
   讀距離行：`D:1.85`
2. **馬達命令接到 UNO D10**（二選一）  
   - **沒有 USB-TTL（建議 Pi 5）**：GPIO **腳 8 TX → UNO D10**，**腳 6 GND → UNO GND**  
     `.env` 填 `MOTOR_SERIAL_PORT=/dev/ttyAMA0`  
     並在 `/boot/firmware/config.txt` 加一行 `dtparam=uart0=on` 後重開  
     （**不要**用 `/dev/serial0`：Pi 5 常連到 `ttyAMA10` 除錯座，不是腳 8）  
   - **有 USB-TTL**：TX → UNO D10，GND 共地，`.env` 填 `/dev/ttyUSB1`
3. 燒錄 `uwb-follow-car` 的新版：
   - `uwb_car_responder/`（輸出 `D:xx.xx`）
   - `uno_car/`（Pi-slave 馬達）
   - `uwb_tag_initiator/`（人身上，不變）

用 `ls /dev/ttyUSB*` / `ls /dev/ttyAMA0` 確認裝置後再寫進 `.env`。

## 安裝

```bash
cd 13_Vision_UWB_Follow
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
# 編輯 .env：UWB_SERIAL_PORT / MOTOR_SERIAL_PORT
```

Pi 使用者加入 dialout（若尚未）：

```bash
sudo usermod -aG dialout $USER
# 重新登入後生效
```

## 執行

```bash
python follow_car.py
```

瀏覽器：`http://樹莓派IP:5001`

1. 預設僅預覽（不驅動馬達）
2. 按「開啟跟隨」才會送 `F/L/R/Z`
3. 「停止跟隨」立刻 `S`

## 與跌倒偵測的關係

| 專案 | Port | 職責 |
|------|------|------|
| `12_Fall_Detection` | 5000 | 跌倒確認 + LINE |
| `13_Vision_UWB_Follow` | 5001 | 跟隨控制 |

之後若要「確認跌倒 → 急停小車」，可在跌倒流程呼叫本程式的停止，或合併成同一 Flask 服務。壁障（VL53L0X）尚未接入。

## 調參

| 變數 | 預設 | 說明 |
|------|------|------|
| `CENTER_DEADZONE` | `0.12` | 置中寬度；太小會左右晃 |
| `START_DIST` / `STOP_DIST` | `2.0` / `1.8` | 與原本 UWB 直線跟隨一致 |
| `UWB_STALE_SECONDS` | `1.0` | 距離過舊則不前進 |
