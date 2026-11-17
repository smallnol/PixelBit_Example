# MaUWB STM32 AOA 小車跟隨整合（架構規劃）

> 目標：以 **MaUWB_STM32 AOA Development Kit** 取代 ESP32-UWB DS-TWR，作為跟隨主測距／方位來源。  
> 過渡相機方案（`13_Vision_UWB_Follow`）保留為備用／融合，**不是刪除**。

對應舊方案：

| 項目 | 舊（過渡） | 新（AOA） |
|------|------------|-----------|
| 方位 | YOLO 畫面左／中／右 | Anchor PDOA 角度（±60°） |
| 距離 | ESP32-UWB `D:1.85` | Anchor ToF `dist_cm` |
| 馬達 | Pi → ESP32 USB → GPIO17 → UNO D10 | Pi →（獨立 Serial）→ UNO D10 |
| 大腦 | 樹莓派 5 | **仍建議樹莓派 5** |

---

## 1. 建議總架構（Pi 當大腦）

```
身上 Tag（MaUWB AOA Tag，鋰電／USB-TTL 供電）
        │  UWB PDOA + ToF
        ▼
車上 Anchor（MaUWB AOA Anchor）
        │  USB-Native（STM32 VCP）→ 距離 + 角度
        ▼
   樹莓派 5（follow 決策 + 跌倒偵測可並存）
        │  馬達命令 F/L/R/Z/S/X
        ▼
   USB-TTL（或既有轉接板）TX → UNO D10（SoftwareSerial）
        │
        ▼
   UNO R3 + BMP73T104（維持現有 Pi-slave 韌體）
```

### 為什麼仍用 Pi 當大腦（不建議 Anchor 直接控車）

1. **跌倒偵測已在 Pi**（`12_Fall_Detection`）；同一大腦才能做「確認跌倒 → `X` 急停」。
2. UNO／BMP73T104 命令介面已穩定；少改韌體風險最低。
3. AOA 官方輸出面向 host（PC／Pi），Qt demo 同路徑；到貨後用 Pi 解析最快落地。
4. 之後壁障（5× VL53L0X）也適合在 Pi 融合後再下命令。

### 不建議的替代

| 方案 | 問題 |
|------|------|
| Anchor 韌體直接算 F/L/R 打 UNO | 需改 STM32 韌體＋失去與跌倒／相機融合 |
| 繼續用 ESP32-UWB 當距離源 | 與本目標衝突，應卸除 |
| 保留 ESP32 只做 USB→Serial2 轉發 | 可當過渡，但多一顆板；最終應改 USB-TTL |

---

## 2. 硬體／供電（務必保留既有規則）

| 區塊 | 供電 | 注意 |
|------|------|------|
| BMP73T104 馬達電 | 4S 18650 ≈ 14.8V → **只接**馬達板 V+/V- | **不可**接 UNO 5V |
| UNO | 行充／USB 5V | |
| 共地 | 18650 GND **必須**共 UNO GND | |
| Pi 5 | 原廠 27W | |
| AOA Anchor | USB 5V（接 Pi 或獨立 5V） | **不可**接 14.8V |
| AOA Tag | 板載鋰電／USB-TTL 5V | 人身上攜帶 |

載重＋原地對拉易觸發 BMP73T104 過流鎖死 → 斷馬達電復位；UNO 維持弧線轉向＋清 OCP。

### 建議接線表

| 從 | 到 | 說明 |
|----|----|------|
| Anchor **USB-Native** | Pi USB | 讀距離／角度（VCP；Linux 多為 `/dev/ttyACM*`） |
| Tag **USB-TTL** 或電池 | — | 只供電；不必接 Pi |
| USB-TTL TX（3.3V／5V 皆可試，UNO 為 5V 容限） | UNO **D10** | 馬達命令；鮑率與 UNO 一致（現為 **9600**） |
| USB-TTL GND | UNO GND | 共地 |
| （可選）USB-TTL 接 Pi USB | — | 出現第二個 `/dev/ttyUSB*` |

拔掉舊線：

- 車上 ESP32-UWB USB／GPIO17 轉發鏈
- Pi 腳 8 → UNO 舊杜邦線（若還在）

### AOA 安裝方位

- Anchor 雙天線朝向車頭正前方；0° ≈ 正前方，左負／右正（實測後以 `AOA_ANGLE_SIGN` 校正）。
- 規格：**角度約 ±60°**、誤差約 ±5°；距離約 30 m @ 6.8M、誤差 < 10 cm。
- 超出視角時角度不可靠 → 應用 `Z` 自轉搜尋（或切相機輔助）。

---

## 3. 資料格式（Anchor → Pi）

來源：[Makerfabs UWB-AOA STM32 韌體](https://github.com/Makerfabs/UWB-AOA-with-Display-STM32F103C8T6)  
`json_2pc.c` 依 `user_cmd` 分流：

### Mode A（預設 `user_cmd == 0`）：JSON 文字

```
JSxxxx{"TWR": {"a16":"...","R":n,"T":...,"D":343,"P":...,"Xcm":...,"Ycm":...,"O":...,"V":...,"X":...,"Y":...,"Z":...}}
```

| 欄位 | 意義 |
|------|------|
| `D` | 距離 **cm**（整數） |
| `P` | 原始 PDOA（韌體註解：milli-radians；實測需對照 Qt） |
| `Xcm` / `Ycm` | 相對 Node 的直角座標（cm）→ 可用 `atan2(Y,X)` 推角度 |

### Mode B（`user_cmd == 1`）：跟隨二進位（建議主用）

函式 `send_to_pc_custom_twr_carfollow()`：

| 欄位 | 內容 |
|------|------|
| header | `0x2A`（`*`） |
| length | payload 長度 |
| sn | range 序號 |
| Addr | tag addr16 |
| angle | `int`（度，來自 `Res.angle`） |
| distance | `int`（**cm**，來自 `Res.dist_cm`） |
| user_cmd / 功率 / Acc | 其餘狀態 |
| XOR | 從 length 後開始 XOR |
| footer | `0x23`（`#`） |

到貨後第一件事：用 Qt「AOA System」確認 Tag 綁定（**一 Anchor 同時只配一 Tag**），再用 `UserCmd=1`（或韌體對應指令）切到 Mode B，於 Pi `cat`／自寫 parser 驗證。

---

## 4. 控制邏輯（對齊現有 UNO 命令）

**UNO 介面不改**：仍聽 `F/L/R/Z/S/X`（`uwb-follow-car` → `uno_car/` Pi-slave）。

| 條件 | 命令 |
|------|------|
| 跟隨關閉 | `S` |
| AOA 逾時／無有效幀 | `Z`（搜尋）或 `S`（保守；建議可調） |
| 角度 < −DEADZONE | `L` |
| 角度 > +DEADZONE | `R` |
| \|角度\| ≤ DEADZONE 且距離 ≥ START_DIST | `F` |
| \|角度\| ≤ DEADZONE 且距離 < STOP_DIST | `S` |
| 緩衝帶（例如 1.8～2.0 m） | 維持上一 F／S |

建議預設（可與舊視覺方案共用）：

- `START_DIST=2.0` m、`STOP_DIST=1.8` m  
- `AOA_ANGLE_DEADZONE_DEG=10`（約對應舊 `CENTER_DEADZONE=0.12`）  
- `AOA_STALE_SECONDS=1.0`

---

## 5. 與現有程式如何銜接

### PixelBit：`13_Vision_UWB_Follow/follow_car.py`

保留檔案，演進為 **模式切換**（建議）：

```text
FOLLOW_SOURCE=aoa | vision | fused
```

| 模式 | 方位 | 距離 | 用途 |
|------|------|------|------|
| `vision` | YOLO offset | （可選）無／手動 | 套件未到／室內光好 |
| `aoa` | AOA angle | AOA dist | **主目標** |
| `fused` | AOA 優先；AOA 失效才 YOLO | AOA | 室外／遮擋備援 |

實作要點：

1. 新增 `AoaReader`（解析 Mode B 或 JSON），取代只認 `D:1.85` 的 `SerialReader`。
2. `decide_command()` 改吃 `angle_deg`／`has_fix`，視覺路徑改呼叫同一決策。
3. **馬達出口與 AOA 口分離**：`MOTOR_SERIAL_PORT` ≠ `AOA_SERIAL_PORT`（不再假設「同一 USB 轉發」）。
4. 控制迴圈維持背景 thread（不依賴瀏覽器 `video_feed`）。

### `uwb-follow-car`

| 資料夾 | 處置 |
|--------|------|
| `uno_car/` | **保留**，幾乎不改 |
| `uwb_car_responder/`、`uwb_tag_initiator/` | 標記 deprecated；不再燒錄 |
| （可選）新 `serial_relay/` | 若暫時沒 USB-TTL，可用閒置 ESP32 **只轉發**字元 |

本機舊路徑 `C:\Users\a0907\Documents\uwb-follow-car` 以 GitHub `cursor/pi-vision-follow-1509` 為準；AOA 落地後再開 `cursor/aoa-follow-…` 分支清掉 UWB 角色說明。

---

## 6. 過渡相機方案：要不要保留？

**要保留。** 理由：

1. 套件未到前仍可實車驗證馬達／UNO／過流行為。  
2. AOA 只有 ±60°；人跑出視角時相機或 `Z` 搜尋仍有價值。  
3. 跌倒偵測本來就靠相機；跟隨視覺模組可共用相機管線（注意 CPU：跟隨與跌倒建議分時或分 process）。  
4. 刪除成本高、保留成本低（`FOLLOW_SOURCE=vision` 即可）。

建議：AOA 穩定後預設 `aoa`，`vision`／`fused` 當開關，不刪目錄。

---

## 7. 到貨後實作順序

1. Qt 驗證 Anchor↔Tag（距離／角度跳動是否合理）。  
2. Pi 辨識埠：`ls /dev/ttyACM* /dev/ttyUSB*`；寫死到 `.env`。  
3. 切 Mode B，跑 `aoa_sniff.py`（見同目錄）確認 angle／cm。  
4. 接 USB-TTL → UNO，只用鍵盤／腳本送 `F/L/R/S` 確認馬達。  
5. 接上 `decide_command` AOA 路徑，先關相機。  
6. 再打開 `fused`／與跌倒急停聯動。  
7. 最後規劃 5× VL53L0X 壁障（未做）。

---

## 8. `.env` 草案

見 `.env.example`。重點：

```env
FOLLOW_SOURCE=aoa
AOA_SERIAL_PORT=/dev/ttyACM0
AOA_BAUD=115200
AOA_MODE=binary          # binary | json
MOTOR_SERIAL_PORT=/dev/ttyUSB0
MOTOR_BAUD=9600
START_DIST=2.0
STOP_DIST=1.8
AOA_ANGLE_DEADZONE_DEG=10
AOA_STALE_SECONDS=1.0
```

---

## 9. 參考連結

- Kit：[MaUWB_STM32 AOA Development Kit](https://www.makerfabs.com/mauwb-stm32-aoa-development-kit.html)  
- 韌體／Qt：[Makerfabs/UWB-AOA-with-Display-STM32F103C8T6](https://github.com/Makerfabs/UWB-AOA-with-Display-STM32F103C8T6)  
- 視覺過渡：`13_Vision_UWB_Follow`（分支 `cursor/vision-uwb-follow-1509`）  
- 車控：`https://github.com/smallnol/uwb-follow-car`（`uno_car` Pi-slave）  
- 跌倒：`12_Fall_Detection`（分支 `cursor/fall-detection-line-media-5500`）  

> 注意：Makerfabs **AOA Gen2** 與本 Gen1 kit **不相通**；請確認到貨為 STM32F103 + Display 這套 Gen1。
