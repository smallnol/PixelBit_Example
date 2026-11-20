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
                                         │ USB（距離 + 馬達命令）
                                         ▼
                                    樹莓派 5 + 相機
                                         │
                    車上 ESP32 GPIO17 ───┘（Serial2 轉發）
                                         ▼
                                    UNO D10（馬達命令 F/L/R/Z/S/X）
```

**建議接線（無 USB-TTL）**

1. **車上 ESP32 USB → 樹莓派**（`/dev/ttyUSB0`）  
   - Pi 讀距離 `D:1.85`  
   - Pi 寫馬達命令 `F/L/R/Z/S/X`（同一 USB）
2. **車上 ESP32 GPIO17（TX2）→ UNO D10**，**GND 共地**  
   - ESP32 把 USB 收到的命令轉到 Serial2
3. **拔掉** 樹莓派腳 8 接到 UNO 的杜邦線（改走 ESP32 轉發）
4. 燒錄：
   - `uwb_car_responder/`（距離 + 轉發命令）
   - `uno_car/`（Pi-slave 聽 F/L/R/Z）
   - `uwb_tag_initiator/`（不變）

`.env`：
```env
UWB_SERIAL_PORT=/dev/ttyUSB0
MOTOR_SERIAL_PORT=/dev/ttyUSB0
MOTOR_BAUD=115200
```

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
