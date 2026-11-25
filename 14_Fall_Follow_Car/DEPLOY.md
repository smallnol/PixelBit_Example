# 部署手冊：接線／供電／燒錄／樹莓派指令

分支：`cursor/vl53-front-only-1509`  
目錄：`14_Fall_Follow_Car/`  

> **※ 2026-11-25：僅 1×VL53L0X 正前。** 左前／右前／正後不接。

完整腳位見 **`FEATURES_PINOUT.txt`**。

---

## 一、要燒哪些程式

| 板子 | 燒錄檔案 | 說明 |
|------|----------|------|
| **車上 ESP32-UWB** | `firmware/uwb_car_responder/uwb_car_responder.ino` | `D:` + 轉發馬達 + 橋接 `O:` |
| **Arduino UNO** | `firmware/uno_car/uno_car.ino` | 馬達 + **正前 VL53** + `O:mm` |
| **身上 Tag ESP32** | `uwb-follow-car` → `uwb_tag_initiator`（沿用） | 測距標籤 |
| **樹莓派 5** | **不燒 .ino** | `fall_follow_car.py` + `line_media.py` |

函式庫：UNO 需 `BMP73T104` + **Pololu VL53L0X**；ESP32 需 `dw3000`。

---

## 二、供電（雙電池，只共地）

```
【電池 A｜邏輯】3S → DC → XY3606（~5.1V）→ USB-C → Pi 5
                 → Pi USB → 相機、ESP32、UNO

【電池 B｜馬達】獨立 → BMP73T104（跳帽勿灌 UNO 5V）

★ 只共地，正極勿併
```

---

## 三、訊號接線

```
Tag --UWB--> 車上 ESP32 --USB--> Pi
                GPIO17 → UNO D10（命令）
                GPIO16 ← UNO D11（O:正前）
                GND 共地

VL53 正前×1：
  VCC→UNO 5V  GND  SDA→A4  SCL→A5  XSHUT→D3
```

### 協定

- Pi → UNO：`F/L/R/Z/S/X`
- UNO → Pi：`O:320`（正前 mm）
- ESP32 → Pi：`D:1.85`（UWB m）

### 避障

| 情況 | 行為 |
|------|------|
| `F` 且正前 &lt; 250mm | 強制 `S`（UNO＋Pi） |
| `L`／`R`／`Z` | 不覆寫（無側／後感測） |
| `X` | 急停 |

---

## 四、樹莓派指令

```bash
mkdir -p ~/fall_follow && cd ~/fall_follow
curl -fsSL -o fall_follow_car.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/vl53-front-only-1509/14_Fall_Follow_Car/fall_follow_car.py"
curl -fsSL -o line_media.py \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/vl53-front-only-1509/14_Fall_Follow_Car/line_media.py"
curl -fsSL -o requirements.txt \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/vl53-front-only-1509/14_Fall_Follow_Car/requirements.txt"
curl -fsSL -o .env.example \
  "https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/vl53-front-only-1509/14_Fall_Follow_Car/.env.example"

python3 -m venv .venv && source .venv/bin/activate
pip install -U pip && pip install -r requirements.txt
cp .env.example .env && nano .env
mkdir -p events/pending events/confirmed
python3 fall_follow_car.py
```

`.env`：`AVOID_MM=250`、`UWB_SERIAL_PORT=/dev/ttyUSB0`。

自檢序列埠應見 `D:x.xx` 與 **`O:數字`（單一）**。

---

## 五、上電順序

1. 接共地、GPIO17/16、**一顆正前 VL53**  
2. 電池 B → 馬達；電池 A → XY3606 → Pi  
3. 燒好 UNO／ESP32；Tag 開機  
4. `python3 fall_follow_car.py` → 網頁鎖定 → 跟隨  

---

## 六、常見問題

| 現象 | 檢查 |
|------|------|
| 沒有 `O:` | D11↔GPIO16、VL53 供電／XSHUT D3、Pololu 庫 |
| `O:` 一直 8190 | 遮擋／接線／init FAIL |
| 沒有 `D:` | Tag 開機 |
| 側撞／後撞 | **預期**：單顆只管正前 |
