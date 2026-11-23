# 部署手冊：接線／供電／燒錄／樹莓派指令

分支：`cursor/fall-follow-vl53-lock-dash-1509`  
目錄：`14_Fall_Follow_Car/`

---

## 一、系統元件一覽

| 元件 | 角色 | 要燒的程式 |
|------|------|------------|
| **人身上 ESP32-UWB（Tag）** | 發出 UWB POLL，給車上測距 | `uwb-follow-car` 的 `uwb_tag_initiator`（不變，沿用舊版即可） |
| **車上 ESP32-UWB（Responder）** | 算距離 `D:`；USB↔Pi；轉發馬達命令；橋接 `O:` | **本 repo** `firmware/uwb_car_responder/uwb_car_responder.ino` |
| **Arduino UNO R3 + BMP73T104** | 聽 `F/L/R/Z/S/X`、讀 5×VL53、避障覆寫、馬達 | **本 repo** `firmware/uno_car_vl53/uno_car_vl53.ino` |
| **5× VL53L0X** | 左前／正前／右前／左側／右側距離（mm） | 無韌體（由 UNO 驅動） |
| **樹莓派 5 + 相機** | YOLO 跟隨／跌倒／網頁／送馬達命令 | **不燒韌體**；跑 Python：`fall_follow_car.py` + `line_media.py` |
| **18650／動力電池** | 只給馬達驅動板（BMP73T104） | — |
| **行充／5V** | 建議給 UNO 邏輯電（勿把 14.8V 灌進 UNO 5V） | — |

---

## 二、供電（務必分清楚）

```
┌─────────────────────┐         ┌──────────────────────┐
│ 18650 / 動力電池     │         │ 行充或 5V USB        │
│ （例：約 14.8V）      │         │                      │
└─────────┬───────────┘         └──────────┬───────────┘
          │ VIN / 馬達電                   │ 5V（UNO）
          ▼                                ▼
   ┌──────────────┐                  ┌──────────┐
   │ BMP73T104    │◄── I2C ──────────│ UNO R3   │
   │ 馬達驅動     │                  └────┬─────┘
   └──────┬───────┘                       │
          │ 馬達輸出                      │
          ▼                               │
        底盤馬達                           │
                                          │
   ★ 電池 GND ──共地── UNO GND ──共地── ESP32 GND ──共地──（可選）Pi GND
```

### 規則

1. **動力電池只接 BMP73T104 馬達電**，**絕對不要**接到 UNO 的 5V 腳。  
2. **UNO** 用獨立 5V（行充／USB 上傳時的供電均可），但 **GND 必須與電池／驅動板共地**。  
3. **車上 ESP32** 由 **樹莓派 USB** 供電即可（同時當通訊口 `/dev/ttyUSB0`）。  
4. **人身上 Tag ESP32** 用行動電源或電池模組獨立供電。  
5. **VL53L0X** 由 UNO 的 **3.3V 或 5V（依模組規格）+ GND + SDA/SCL** 供電；多數 Pololu／相容板用 **VIN 接 UNO 5V 或 3.3V**，請對照你手邊模組標示。  
6. 劇烈原地轉可能觸發 BMP73T104 **OCP**；韌體已做弧線轉＋Soft-start，仍堵轉時需清保護／重上電。

---

## 三、訊號接線（完整）

### 3.1 總圖

```
人身上 ESP32-UWB (Tag)
        │  UWB 無線測距
        ▼
車上 ESP32-UWB (Responder)
        │ USB（115200）──────────────────────────► 樹莓派 5
        │   ↑ Pi 讀：D:1.85（距離 m）
        │   ↑ Pi 讀：O:320,450,...（避障 mm）
        │   ↓ Pi 寫：F/L/R/Z/S/X
        │
        │ GPIO17 (TX2) ──► UNO D10     （馬達命令）
        │ GPIO16 (RX2) ◄── UNO D11     （O: 回傳）★新增
        │ GND 共地
        ▼
Arduino UNO
        │ I2C SDA/SCL + XSHUT
        ▼
5× VL53L0X
        │
UNO ──I2C──► BMP73T104 ──► 馬達
```

### 3.2 車上 ESP32 ↔ UNO

| 從 | 到 | 用途 |
|----|----|------|
| ESP32 **GPIO17** | UNO **D10** | 馬達命令 TX→RX |
| UNO **D11** | ESP32 **GPIO16** | 避障 `O:` 回傳 ★必接 |
| ESP32 **GND** | UNO **GND** | 共地 |
| ESP32 **USB** | 樹莓派 **USB** | 供電＋`/dev/ttyUSB0` |

> 拔掉舊實驗用「Pi 腳位直接到 UNO」的杜邦線；馬達命令一律走 ESP32 轉發。

### 3.3 VL53L0X ×5（接在 UNO）

共用：**SDA、SCL、GND、VCC**；每顆各一根 **XSHUT**：

| 方位 | XSHUT 腳 | 開機 I2C 位址 |
|------|----------|----------------|
| [0] 左前 | **D2** | `0x30` |
| [1] 正前 | **D3** | `0x31` |
| [2] 右前 | **D4** | `0x32` |
| [3] 左側 | **D5** | `0x33` |
| [4] 右側 | **D6** | `0x34` |

UNO 開機流程：五根 XSHUT 先全 LOW → 依序拉高並 `setAddress()`，避免大家都是預設 `0x29`。

### 3.4 人身上 Tag

- 燒錄 `uwb_tag_initiator`（`smallnol/uwb-follow-car` 原專案即可）  
- 獨立供電，與車上 Responder 在 UWB 通訊範圍內即可（無杜邦線接到車）

### 3.5 樹莓派

| 接線 | 說明 |
|------|------|
| USB → 車上 ESP32 | 通常是 `/dev/ttyUSB0`（CP2102／CH340） |
| CSI／USB 相機 | `CAMERA_INDEX=0` |
| 網路 Wi-Fi／網線 | 開網頁 `:5001`、上傳 LINE 媒體 |

---

## 四、哪些檔要燒錄（Arduino IDE）

### 4.1 車上 ESP32-UWB

- **檔案**：  
  `14_Fall_Follow_Car/firmware/uwb_car_responder/uwb_car_responder.ino`
- **板子**：ESP32 Dev Module（依你的 UWB 板型）
- **函式庫**：`dw3000`（與原本 uwb-follow-car 相同）
- **鮑率監看**：115200  
- 開機應看到類似：`O: bridge`、`USB: D:xx.xx | O:mm,...`

### 4.2 Arduino UNO

- **檔案**：  
  `14_Fall_Follow_Car/firmware/uno_car_vl53/uno_car_vl53.ino`
- **板子**：Arduino UNO
- **函式庫**：
  - `BMP73T104`
  - `VL53L0X`（**Pololu** 版，有 `setAddress`）
- **鮑率監看**：115200（USB 除錯）  
- SoftSerial 對 ESP32 為 **9600**
- 開機應看到：`UNO Pi-slave + VL53L0X`、各 idx 的 `VL53 OK`，並週期印 `O:...`

### 4.3 人身上 Tag（若尚未燒過）

- `https://github.com/smallnol/uwb-follow-car` → `uwb_tag_initiator/uwb_tag_initiator.ino`  
- **這次整合沒改 Tag**，舊的可繼續用。

### 4.4 樹莓派：不燒 .ino

跑 Python（下一節指令）。需要兩個檔在同一目錄：

- `fall_follow_car.py`
- `line_media.py`

---

## 五、樹莓派終端機指令

### 5.1 第一次安裝

```bash
# 使用者加入 dialout（若尚未），然後重新登入
sudo usermod -aG dialout $USER
# 重新 SSH / 重開機後再生效

mkdir -p ~/fall_follow && cd ~/fall_follow

curl -fsSL -o fall_follow_car.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/fall_follow_car.py"
curl -fsSL -o line_media.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/line_media.py"
curl -fsSL -o requirements.txt \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/requirements.txt"
curl -fsSL -o .env.example \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/.env.example"

python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt

cp .env.example .env
nano .env   # 至少填 LINE／IMGBB；序列埠確認為 /dev/ttyUSB0

mkdir -p events/pending events/confirmed

# 確認 USB 裝置
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

`.env` 建議關鍵項：

```env
UWB_SERIAL_PORT=/dev/ttyUSB0
MOTOR_SERIAL_PORT=/dev/ttyUSB0
CAMERA_INDEX=0
PORT=5001
FALL_SENSITIVITY=1.45
AVOID_MM=250
```

### 5.2 每次更新程式後執行

```bash
cd ~/fall_follow
source .venv/bin/activate

curl -fsSL -o fall_follow_car.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/fall_follow_car.py"
curl -fsSL -o line_media.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/line_media.py"

pkill -f fall_follow_car.py || true
python3 fall_follow_car.py
```

### 5.3 背景跑（可選）

```bash
cd ~/fall_follow
source .venv/bin/activate
pkill -f fall_follow_car.py || true
nohup python3 fall_follow_car.py > fall_follow.log 2>&1 &
tail -f fall_follow.log
```

### 5.4 開機成功應看到

```
=== APP_VERSION=fall-follow-2026-11-23-vl53-lock-dash ===
=== FALL_SENSITIVITY=1.45 ===
=== TRACKER=bytetrack.yaml AVOID_MM=250 ===
✅ UWB Serial 已開: /dev/ttyUSB0
=== CONTROL LOOP RUNNING ...
🌐 開啟 http://<樹莓派IP>:5001
```

瀏覽器：`http://樹莓派IP:5001`

### 5.5 快速自檢指令

```bash
# 看 ESP32 是否在吐 D: / O:（先停掉 Python，避免佔用序列埠）
pkill -f fall_follow_car.py || true
python3 - <<'PY'
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
t0 = time.time()
while time.time() - t0 < 8:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print(line)
ser.close()
PY
```

預期會出現 `D:x.xx` 與／或 `O:...,...,...,...,...`（人身上 Tag 開著才有穩定 `D:`；UNO 燒好且 D11↔GPIO16 接好才有 `O:`）。

---

## 六、上電與操作順序

1. 接好 GND 共地、訊號線、VL53  
2. 動力電池 → 驅動板；UNO 5V；Pi 開機  
3. 車上 ESP32 插上 Pi USB；身上 Tag 開機  
4. 燒錄確認：UNO／車上 ESP32 為本 repo `firmware/` 版本  
5. Pi 跑 `python3 fall_follow_car.py`  
6. 網頁：**先鎖定目標** → 再開跟隨／跌倒偵測  
7. 跟隨 OFF 可手動遙控；跟隨 ON 方向鍵禁用（急停 `X` 仍可用）

---

## 七、常見問題

| 現象 | 檢查 |
|------|------|
| 沒有 `D:` | Tag 有無開機／距離；ESP32 是否本版韌體 |
| 沒有 `O:` | UNO 是否本版；**D11→GPIO16**；VL53 供電／XSHUT |
| 馬達完全不轉 | GND 共地？GPIO17→D10？Pi 是否經 USB 送出命令（log 有 `FWD->UNO`） |
| 轉一下就死 | OCP：降速、清過流、避免高負載原地對拉 |
| 跟錯人／路人報跌倒 | 是否已鎖定？綠框 `TARGET LOCKED` |
| `/dev/ttyUSB0` 權限 | `dialout` 群組並重新登入 |
