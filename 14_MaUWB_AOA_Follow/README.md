# MaUWB STM32 AOA 小車跟隨＋跌倒聯動

## 兩大功能

| 功能 | 程式 | 說明 |
|------|------|------|
| **AOA 跟隨** | 本目錄 `follow_aoa.py` + `uno_car/` | Tag 距離＋角度 → Pi 決策 → UNO 開車 |
| **跌倒偵測** | `../12_Fall_Detection/fall_detection.py` | YOLO-pose + LINE；確認跌倒呼叫 `/estop` |

AOA **套件本身**使用 Makerfabs 原廠韌體（不必自己燒跟隨邏輯）；Pi 用序列埠讀距離／角度。

---

## 資料流

```
身上 Tag（電池／USB-TTL 供電）
        │ UWB
        ▼
車上 Anchor ──USB-Native──► 樹莓派 5  follow_aoa.py :5001
                              │              ▲
                              │ F/L/R/Z/S/X  │ GET /estop
                              ▼              │
                         USB-TTL TX     fall_detection.py :5000
                              │              （確認跌倒）
                              ▼
                         UNO D10 + BMP73T104
```

---

## 1. UNO（小車）

檔案：[`uno_car/uno_car.ino`](uno_car/uno_car.ino)

1. Arduino IDE 選 **Arduino Uno**，安裝函式庫 **BMP73T104**
2. 燒錄本 sketch
3. 接線：
   - BMP 盾**直接插在** UNO 上
   - 4S 18650 ≈ 14.8V → 擴充板馬達 **V+/V-**（板內隔離，不灌 UNO 邏輯電）
   - UNO 邏輯電：行充／USB 5V
   - USB-TTL **TX → D10**，**GND → GND**，鮑率 **9600**

| 命令 | 動作 |
|------|------|
| F | 前進 |
| L / R | 弧線左轉／右轉 |
| Z | 慢速自轉搜尋 |
| S | 軟停 |
| X | 緊急煞停 |

逾約 1.2s 無命令會自動急停；運動中會清 OCP，降低堵轉假死。

---

## 2. AOA Kit（原廠）

硬體：[MaUWB_STM32 AOA Development Kit](https://www.makerfabs.com/mauwb-stm32-aoa-development-kit.html)  
韌體開源：[Makerfabs/UWB-AOA-with-Display-STM32F103C8T6](https://github.com/Makerfabs/UWB-AOA-with-Display-STM32F103C8T6)

1. Anchor **USB-Native** → Pi（驅動後多為 `/dev/ttyACM0`）
2. Tag 電池或 USB-TTL 供電（人身上）
3. 先用官方 Qt「AOA System」綁定**唯一** Tag（一 Anchor 同時一 Tag）
4. 切跟隨二進位輸出：

```bash
cd 14_MaUWB_AOA_Follow
cp .env.example .env   # 編輯埠號
python aoa_setup_usercmd.py          # USER_CMD 1 + SAVE
python aoa_sniff.py                  # 應看到 dist / angle
```

| 模式 | 設定 | 輸出 |
|------|------|------|
| JSON | `USER_CMD 0` | `JSxxxx{"TWR":{"D":cm,...}}` |
| **Binary（建議）** | `USER_CMD 1` | `0x2A` … angle + dist_cm … `0x23` |

角度約 ±60°；超出視角會失效 → 預設送 `Z` 搜尋。

---

## 3. 樹莓派 5｜跟隨

```bash
cd 14_MaUWB_AOA_Follow
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
# AOA_SERIAL_PORT=/dev/ttyACM0
# MOTOR_SERIAL_PORT=/dev/ttyUSB0
# MOTOR_BAUD=9600
sudo usermod -aG dialout $USER   # 一次即可，重新登入
python follow_aoa.py
```

瀏覽：`http://<樹莓派IP>:5001`

- 開啟跟隨／停止跟隨
- 緊急停止 X（並鎖住，需「解除急停」）
- 控制在背景 thread，不靠瀏覽器才開車

決策摘要：

| 條件 | 命令 |
|------|------|
| 跟隨關／急停鎖 | S / X |
| AOA 逾時 | Z（可改 `AOA_LOST_CMD=S`） |
| 角度偏左／右 | L / R |
| 置中且 ≥ 2.0 m | F |
| 置中且 < 1.8 m | S |

左右反了：`.env` 設 `AOA_ANGLE_SIGN=-1`。

---

## 4. 樹莓派 5｜跌倒（已整合急停）

```bash
cd ../12_Fall_Detection
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env   # LINE / ImgBB 等
# FOLLOW_ESTOP_URL=http://127.0.0.1:5001/estop
python fall_detection.py
```

`http://<樹莓派IP>:5000` — 確認跌倒（約 10s）時會：

1. `GET` 跟隨服務 `/estop` → 小車 `X` 並鎖跟隨  
2. LINE 文字＋照片＋影片（原有流程）

建議**兩個 process 同時開**：先 `follow_aoa.py`，再 `fall_detection.py`。相機給跌倒用；跟隨不佔用相機。

---

## 目錄一覽

| 檔案 | 角色 |
|------|------|
| `uno_car/uno_car.ino` | UNO 馬達 Pi-slave |
| `follow_aoa.py` | Pi 跟隨主程式 |
| `aoa_reader.py` / `aoa_protocol.py` | AOA 讀取／解析／決策 |
| `motor_serial.py` | 對 UNO 送命令 |
| `aoa_setup_usercmd.py` | 切 Binary／JSON |
| `aoa_sniff.py` | 到貨校對 |
| `test_aoa_protocol.py` | 離線單元測試 |

---

## 離線測試（無硬體）

```bash
cd 14_MaUWB_AOA_Follow
python3 -m unittest test_aoa_protocol.py -v
```

---

## 供電（務必）

| 區塊 | 作法 |
|------|------|
| BMP 馬達電 | 14.8V → V+/V-；盾插在 UNO，**板內隔離** |
| UNO 邏輯 | 行充／USB 5V |
| AOA | USB 5V，**勿接 14.8V** |
| Pi 5 | 原廠 27W |
