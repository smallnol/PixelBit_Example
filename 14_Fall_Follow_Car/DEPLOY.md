# 部署手冊：接線／供電／燒錄／樹莓派指令

分支：`cursor/remove-vl53-1509`（基底可對 `cursor/fall-follow-combine-1509`）  
目錄：`14_Fall_Follow_Car/`  

> **※ 2026-11-25 註解：VL53L0X 已全部拔除。**  
> - 不燒 `uno_car_vl53`；改燒 **`firmware/uno_car/uno_car.ino`**  
> - 無 `O:` 遙測、網頁無 ToF、無近距避障覆寫  
> - 舊 VL53 接線表保留於下方「附錄」僅供日後復用  

完整功能＋腳位總表見 **`FEATURES_PINOUT.txt`**。

---

## 一、要燒哪些程式

| 板子 | 燒錄檔案 | 說明 |
|------|----------|------|
| **車上 ESP32-UWB** | `firmware/uwb_car_responder/uwb_car_responder.ino` | 測距 `D:`；USB↔Pi；轉發馬達命令（**不轉 O:**） |
| **Arduino UNO** | `firmware/uno_car/uno_car.ino` | **純馬達**；執行 F/L/R/Z/S/X |
| **身上 Tag ESP32** | `uwb-follow-car` → `uwb_tag_initiator`（沿用） | 人身上測距標籤 |
| **樹莓派 5** | **不燒 .ino** | 跑 `fall_follow_car.py` + `line_media.py` |

### Arduino IDE 函式庫

- UNO：`BMP73T104`（**不需** Pololu VL53L0X）
- 車上 ESP32：`dw3000`（與原本 UWB 專案相同）

---

## 二、供電（雙電池，只共地）

```
【電池 A｜邏輯電】3S 18650（約 11.1V）
    → DC 頭 → XY3606 降壓模組 → USB → Type-C → 樹莓派 5
         → Pi USB → 相機、車上 ESP32-UWB、Arduino UNO

【電池 B｜馬達電】獨立電池（常見 3S／4S，依 BMP73T104）
    → 只接 BMP73T104 馬達電／VIN
    → 跳線帽不要接：高壓不灌進 UNO 5V

★ 電池 A 負極 ──短接── 電池 B 負極（單點共地）
★ 正極絕對不要併在一起
★ XY3606 輸出請調到樹莓派 5 安全範圍（約 5.1V），再插 Type-C
```

### 規則

1. 馬達高壓 **只**給驅動板；**不要**接 UNO／Pi 的 5V。  
2. UNO／ESP32／USB 相機由 **Pi USB** 供電即可。  
3. BM 擴充板插在 UNO 上時，**GND 已與 UNO 相通**；電池 B 負極進 BM 電源 GND 即與 UNO 共地。  
4. 再經 USB → Pi → XY3606 → 電池 A 負極，整車邏輯地就連起來。

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
              GPIO16 ◄──────────────── UNO D11   （保留；目前不送 O:）
              GND 共地

UNO + BMP73T104（插上擴充板）
  ※ VL53 本輪可不接
```

### 3.2 ESP32 ↔ UNO

| 從 | 到 | 用途 |
|----|----|------|
| ESP32 **GPIO17** | UNO **D10** | 馬達命令 |
| UNO **D11** | ESP32 **GPIO16** | 保留（VL53 停用期間可留線） |
| ESP32 **GND** | UNO **GND** | 共地 |
| ESP32 **USB** | 樹莓派 **USB** | `/dev/ttyUSB0` |

### 3.3 協定字串（目前）

- Pi → ESP32 → UNO：`F` / `L` / `R` / `Z` / `S` / `X` + 換行  
- ESP32 → Pi：`D:1.85`（UWB 公尺）  
- ~~UNO → ESP32 → Pi：`O:...`~~ **已停用**

### 3.4 行為（無 VL53）

| 情況 | 行為 |
|------|------|
| 收到 `F/L/R/Z/S` | UNO 原樣執行（**無避障覆寫**） |
| 收到 `X` | 急停 |
| 命令逾時 | 自動停 |

**請人工監看障礙；無雷射近距防護。**

---

## 四、樹莓派終端機指令

### 4.1 第一次安裝

```bash
sudo usermod -aG dialout $USER
# 重新登入後再生效

mkdir -p ~/fall_follow && cd ~/fall_follow

curl -fsSL -o fall_follow_car.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/remove-vl53-1509/14_Fall_Follow_Car/fall_follow_car.py"
curl -fsSL -o line_media.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/remove-vl53-1509/14_Fall_Follow_Car/line_media.py"
curl -fsSL -o requirements.txt \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/remove-vl53-1509/14_Fall_Follow_Car/requirements.txt"
curl -fsSL -o .env.example \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/remove-vl53-1509/14_Fall_Follow_Car/.env.example"

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
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/remove-vl53-1509/14_Fall_Follow_Car/fall_follow_car.py"
curl -fsSL -o line_media.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/remove-vl53-1509/14_Fall_Follow_Car/line_media.py"

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

預期：`D:x.xx`（Tag 開著）。**不應再期待 `O:a,b,c,d`。**

---

## 五、上電順序（建議）

1. 接好共地、訊號（**VL53 可不接**）  
2. 電池 B → 馬達板；電池 A → DC 頭 → XY3606 → USB-C → Pi  
3. 車上 ESP32 插 Pi USB；身上 Tag 開機  
4. 確認已燒 **`uno_car.ino`** 與更新後的 `uwb_car_responder.ino`  
5. Pi 執行 `python3 fall_follow_car.py`  
6. 網頁先**鎖定目標** → 再開跟隨／跌倒偵測  

---

## 六、常見問題

| 現象 | 檢查 |
|------|------|
| 沒有 `O:` | **正常**（VL53 已停用） |
| 沒有 `D:` | Tag 有無開機 |
| 馬達不轉 | GPIO17→D10、共地、Pi log 是否有 `FWD->UNO` |
| Pi 無法開機／重開 | XY3606 輸出電壓是否約 5.1V；電流是否夠 |
| 撞到障礙 | **無近距避障**；請降低速度／人工監看，或日後接回 VL53 |

---

## 附錄｜舊 VL53 接線（已停用，僅供日後復用）

> 以下**目前程式不會使用**。若要復用：燒回 `uno_car_vl53`、恢復 Pi／ESP32 的 `O:` 解析。

| 編號 | 方位 | UNO XSHUT | I2C 位址 |
|------|------|-----------|----------|
| [0] | 左前 | D2 | `0x30` |
| [1] | 正前 | D3 | `0x31` |
| [2] | 右前 | D4 | `0x32` |
| [3] | 正後 | D5 | `0x33` |

共用 SDA/SCL/VCC(建議 5V)/GND；門檻舊值約 `AVOID_MM=250`。
