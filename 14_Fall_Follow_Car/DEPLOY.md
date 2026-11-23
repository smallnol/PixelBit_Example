# 部署手冊：接線／供電／燒錄／樹莓派指令

分支：`cursor/fall-follow-vl53-lock-dash-1509`  
目錄：`14_Fall_Follow_Car/`  
雷射：**4× VL53L0X**（左前、正前、右前、**前端下視防墜**）

---

## 一、要燒哪些程式

| 板子 | 燒錄檔案 | 說明 |
|------|----------|------|
| **車上 ESP32-UWB** | `firmware/uwb_car_responder/uwb_car_responder.ino` | 測距 `D:`；USB↔Pi；轉發馬達命令；橋接 `O:` |
| **Arduino UNO** | `firmware/uno_car_vl53/uno_car_vl53.ino` | 馬達、4×VL53 避障／防墜、回傳 `O:` |
| **身上 Tag ESP32** | `uwb-follow-car` → `uwb_tag_initiator`（沿用舊版） | 人身上測距標籤 |
| **樹莓派 5** | **不燒 .ino** | 跑 `fall_follow_car.py` + `line_media.py` |

### Arduino IDE 函式庫

- UNO：`BMP73T104`、**VL53L0X（Pololu）**
- 車上 ESP32：`dw3000`（與原本 UWB 專案相同）

---

## 二、供電（雙電池，只共地）

```
【電池 A｜邏輯電】2S 18650（約 7.2V）
    → XL4005 → 5.15V → 樹莓派 5（Type-C）
         → Pi USB → 相機、車上 ESP32-UWB、Arduino UNO

【電池 B｜馬達電】獨立電池（常見 3S／4S，依 BMP73T104）
    → 只接 BMP73T104 馬達電／VIN
    → 跳線帽不要接：高壓不灌進 UNO 5V

★ 電池 A 負極 ──短接── 電池 B 負極（單點共地）
★ 正極絕對不要併在一起
```

### 規則

1. 馬達高壓 **只**給驅動板；**不要**接 UNO／Pi 的 5V。  
2. UNO／ESP32／USB 相機由 **Pi USB** 供電即可。  
3. BM 擴充板插在 UNO 上時，**GND 已與 UNO 相通**；電池 B 負極進 BM 電源 GND 即與 UNO 共地。  
4. 再經 USB → Pi → XL4005 → 電池 A 負極，整車邏輯地就連起來。

---

## 三、訊號接線

### 3.1 總圖

```
身上 Tag ESP32-UWB  --UWB-->  車上 ESP32-UWB
                                   │ USB 115200
                                   ▼
                              樹莓派 5 + USB 相機
                                   │
              GPIO17 ──────────────┘──► UNO D10   （命令 F/L/R/Z/S/X）
              GPIO16 ◄──────────────── UNO D11   （O: 四顆距離）
              GND 共地

UNO + BMP73T104（插上擴充板）
  │ I2C SDA/SCL + XSHUT
  ▼
4× VL53L0X
```

### 3.2 ESP32 ↔ UNO

| 從 | 到 | 用途 |
|----|----|------|
| ESP32 **GPIO17** | UNO **D10** | 馬達命令 |
| UNO **D11** | ESP32 **GPIO16** | `O:` 回傳（必接） |
| ESP32 **GND** | UNO **GND** | 共地 |
| ESP32 **USB** | 樹莓派 **USB** | `/dev/ttyUSB0` |

> 板子上數字腳就是 `2`～`5`，不一定印 `D`。

### 3.3 四顆 VL53L0X（接在 UNO）

共用：**SDA、SCL、VCC、GND**；每顆一根 **XSHUT**：

| 編號 | 方位 | 朝向 | UNO XSHUT | I2C 位址 | 用途 |
|------|------|------|-----------|----------|------|
| [0] | **左前** | 水平朝前偏左 | **2** | `0x30` | 前進避障 |
| [1] | **正前** | 水平朝前 | **3** | `0x31` | 前進避障 |
| [2] | **右前** | 水平朝前偏右 | **4** | `0x32` | 前進避障 |
| [3] | **前端下視** | **朝地面** | **5** | `0x33` | **防墜（樓梯／落差）** |

開機：四根 XSHUT 先全 LOW → 依序拉高並改址，避免都搶預設 `0x29`。

### 3.4 避障／防墜行為（UNO 底層）

| 情況 | 行為 |
|------|------|
| 收到 `F`，左前／正前／右前任一 **&lt; 250mm** | 強制停或往較空側微轉 |
| **下視距離 ≥ 150mm**（或超量程） | 視為前方無地板 → **強制 `S`**（防掉下樓梯） |
| 收到 `X` | 急停，不被覆寫 |

> 下視安裝高度會影響門檻：地板正常讀數應明顯 **小於** `CLIFF_MM`（預設 150）。請實測微調。

### 3.5 協定字串

- Pi → ESP32 → UNO：`F` / `L` / `R` / `Z` / `S` / `X` + 換行  
- UNO → ESP32 → Pi：`O:左前,正前,右前,下視`（mm）  
- ESP32 → Pi：`D:1.85`（UWB 公尺）

---

## 四、樹莓派終端機指令

### 4.1 第一次安裝

```bash
sudo usermod -aG dialout $USER
# 重新登入後再生效

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
nano .env

mkdir -p events/pending events/confirmed
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

`.env` 建議：

```env
UWB_SERIAL_PORT=/dev/ttyUSB0
MOTOR_SERIAL_PORT=/dev/ttyUSB0
CAMERA_INDEX=0
PORT=5001
AVOID_MM=250
CLIFF_MM=150
FALL_SENSITIVITY=1.45
```

（可選）提高 Pi 5 USB 電流：

```bash
# /boot/firmware/config.txt 或 /boot/config.txt
usb_max_current_enable=1
sudo reboot
```

### 4.2 每次更新後執行

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

瀏覽器：`http://樹莓派IP:5001`

### 4.3 自檢序列埠（先停 Python）

```bash
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

預期：`D:x.xx`（Tag 開著）以及 `O:a,b,c,d` 四個數字（下視在平地應明顯小於 150）。

---

## 五、上電順序（建議）

1. 接好共地、訊號、四顆 VL53（含朝下那顆）  
2. 電池 B → 馬達板；電池 A → XL4005 → Pi  
3. 車上 ESP32 插 Pi USB；身上 Tag 開機  
4. 確認已燒本文件列出的 UNO／ESP32 韌體  
5. Pi 執行 `python3 fall_follow_car.py`  
6. 網頁先**鎖定目標** → 再開跟隨／跌倒偵測  

---

## 六、常見問題

| 現象 | 檢查 |
|------|------|
| 沒有 `O:` 四個數 | UNO 韌體、D11↔GPIO16、VL53 供電／XSHUT |
| 平地一直被當懸崖煞停 | 下視安裝高度；把 `CLIFF_MM` 調大；確認鏡頭朝下看得到地板 |
| 樓梯前不煞車 | 下視是否朝下；`CLIFF_MM` 是否太大；讀數是否有變大 |
| 沒有 `D:` | Tag 有無開機 |
| 馬達不轉 | GPIO17→D10、共地、Pi log 是否有 `FWD->UNO` |
