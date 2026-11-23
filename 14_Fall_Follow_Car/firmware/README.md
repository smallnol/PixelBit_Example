# 韌體：VL53L0X 避障 + O: 雙向轉發

## 燒錄

| 板子 | 檔案 | 說明 |
|------|------|------|
| Arduino UNO | `uno_car_vl53/uno_car_vl53.ino` | Pi-slave 馬達 + 5×VL53 + 避障覆寫 + Soft-start |
| 車上 ESP32-UWB | `uwb_car_responder/uwb_car_responder.ino` | DS-TWR 距離 `D:` + 馬達轉發 + `O:` 橋接 |

UNO 需安裝函式庫：**BMP73T104**、**VL53L0X（Pololu）**。  
ESP32 需與原本相同的 **dw3000** 函式庫。

## 新增接線（相對舊版）

```
ESP32 GPIO17 (TX2) → UNO D10   （馬達命令，原本就有）
UNO D11 (SoftSerial TX) → ESP32 GPIO16 （新增：O: 避障回傳）
GND 共地

VL53L0X ×5 共用 UNO I2C（SDA/SCL）
XSHUT: D2 左前, D3 正前, D4 右前, D5 左側, D6 右側
開機改址：0x30～0x34
```

## 協定

- Pi → ESP32 USB → Serial2 → UNO：`F/L/R/Z/S/X\n`
- UNO → ESP32 Serial2 → USB → Pi：`O:左前,正前,右前,左側,右側\n`（mm，約 200ms）
- ESP32 → Pi USB：`D:1.85\n`（UWB 公尺）

## 避障行為（UNO）

收到 `F` 且前方三顆任一 \< 250mm：

- 左較開闊 → 強制 `L`
- 右較開闊 → 強制 `R`
- 前方全堵／難判斷 → 強制 `S`
- `X` 永不被覆寫
