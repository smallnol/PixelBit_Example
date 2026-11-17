# MaUWB STM32 AOA 小車跟隨整合（架構規劃）

## 專題當前兩大功能

| # | 功能 | 主感測／主算力 | 狀態 |
|---|------|----------------|------|
| 1 | **小車跟隨使用者** | MaUWB AOA 的**距離 + 角度** | 硬體改 AOA；套件未到前可用相機過渡 |
| 2 | **跌倒偵測 + LINE 通報** | **樹莓派 5** 相機／YOLOv8-pose | 已大致完成（`12_Fall_Detection`） |

> Pi 的**主責是跌倒偵測**。跟隨以 AOA 為準；若把 AOA 接上 Pi 做決策／與跌倒聯動（例如確認跌倒 → 小車 `X` 急停），或之後用相機輔助讓跟隨更穩，**可以留 Pi 在跟隨鏈路上**——不是要用 YOLO 取代 AOA。

對應舊方案：

| 項目 | 舊（過渡） | 新（目標） |
|------|------------|-----------|
| 跟隨方位 | YOLO 畫面左／中／右 | **AOA 角度** |
| 跟隨距離 | ESP32-UWB `D:1.85` | **AOA ToF** |
| 馬達 | Pi → ESP32 USB → GPIO17 → UNO D10 | Pi（或過渡轉發）→ UNO D10 |
| Pi 主責 | 曾兼視覺跟隨大腦 | **跌倒偵測**；跟隨決策可掛在同一台 Pi |

---

## 1. 建議總架構

```
[功能 1 跟隨]
  身上 Tag ──UWB──► 車上 Anchor ──USB（距離+角度）──► 樹莓派 5
                                                      │ 解析 AOA → F/L/R/Z/S/X
                                                      ▼
                                              USB-TTL → UNO D10 → BMP73T104

[功能 2 跌倒]
  相機 ──► 樹莓派 5（YOLOv8-pose + Flask + LINE）
              │ 確認跌倒時可順便送 X 急停（與跟隨共用命令口）
```

### 角色怎麼分

| 角色 | 做什麼 | 不做什麼 |
|------|--------|----------|
| AOA Anchor | 算出 Tag **距離／角度**並上報 | 不必改韌體直接控馬達（第一版） |
| 樹莓派 5 | **跌倒偵測**；讀 AOA、下馬達命令；可選相機輔助跟隨 | 不以 YOLO 當跟隨主方位 |
| UNO + BMP | 聽 `F/L/R/Z/S/X` 開車 | 不算 AOA |

### 為什麼跟隨決策仍建議經過 Pi（第一版）

1. 跌倒已在 Pi；同一台最容易做「跌倒確認 → 急停」。
2. AOA 官方本來就是報給 host（PC／Pi）；Pi 解析最快落地。
3. UNO 命令介面已穩，少改韌體。

之後若證明 AOA→UNO 直連更單純、且跌倒急停另有通路，再評估精簡亦可。

### 不建議

| 方案 | 問題 |
|------|------|
| 繼續用 ESP32-UWB 當距離源 | 已決定改 AOA |
| 用 YOLO 當跟隨主方案、AOA 當配角 | 與目標相反 |
| 第一版就大改 Anchor 韌體直控馬達 | 拖慢「兩功能先跑起來」 |

---

## 2. 硬體／供電（務必保留既有規則）

| 區塊 | 供電 | 注意 |
|------|------|------|
| BMP73T104 馬達電 | 4S 18650 ≈ 14.8V → 接擴充板馬達 **V+/V-** | BMP 為 **UNO 相容擴充板，直接插在 UNO 上**；板內已隔離，**14.8V 不會灌進 UNO 邏輯電** |
| UNO 邏輯電 | 行充／USB 5V（經 UNO 本體供電） | 與馬達 14.8V 分開；插著 BMP 盾也不要把 14.8V 當 UNO 的 5V 來源 |
| 共地 | 盾疊插後邏輯 GND 已與 UNO 相連；馬達電 V-／GND 依 BMP 接線 | 外接 USB-TTL／AOA 時仍要與 UNO GND 共地 |
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

### 跌倒（主責，已大致完成）

- `12_Fall_Detection/`：維持獨立服務（port 5000）即可。  
- 後續小加分：確認跌倒時對馬達口送 `X`／`S`（與跟隨共用 `MOTOR_SERIAL_PORT`）。

### 跟隨：`13_Vision_UWB_Follow/follow_car.py` → 演進／或新腳本在本目錄

```text
FOLLOW_SOURCE=aoa          # 目標預設：只靠 AOA 距離+角度
# FOLLOW_SOURCE=vision     # 套件未到前的過渡
# FOLLOW_SOURCE=fused      # 可選：AOA 為主，相機只在能提高精度／補 ±60° 外時用
```

實作要點：

1. 新增 `AoaReader`（Mode B 或 JSON），取代只認 `D:1.85`。  
2. `decide_command()` 以 **angle_deg + distance_m** 為主。  
3. **馬達口與 AOA 口分離**。  
4. 控制迴圈背景 thread；不依賴瀏覽器。

### `uwb-follow-car`

| 資料夾 | 處置 |
|--------|------|
| `uno_car/` | **保留**，幾乎不改 |
| `uwb_car_responder/`、`uwb_tag_initiator/` | deprecated |
| （可選）`serial_relay/` | 暫無 USB-TTL 時用閒置 ESP32 只轉發 |

---

## 6. 相機在跟隨裡的角色

| 用途 | 要不要 |
|------|--------|
| **跌倒偵測** | **要**（Pi 主責，必留相機） |
| 跟隨主方位 | **不要**；改 AOA |
| 套件未到前過渡跟隨 | 可暫留 `vision` |
| 讓跟隨更準／補 AOA 視角外 | **可選** `fused`；有實測增益再留，不是必做 |

AOA 穩定後預設 `aoa`。壁障（5× VL53L0X）仍列之後，不在當前兩大功能內。

---

## 7. 實作順序（對準兩大功能）

1. **跌倒**：維持現況可演示（必要時再接急停）。  
2. AOA 到貨：Qt 驗證距離／角度。  
3. Pi：`aoa_sniff.py` 校對 → USB-TTL 驗馬達 → AOA `decide` 實車跟隨。  
4. （可選）跌倒確認 → 小車急停。  
5. （可選）`fused` 若實測更穩再加。  
6. 之後才做 VL53L0X 壁障。

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
