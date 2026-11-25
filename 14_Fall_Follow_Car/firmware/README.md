# 韌體：馬達 + 1×VL53 正前

> **※ 2026-11-25：改為僅 1 顆正前 VL53L0X。**  
> 左前／右前／正後不接、不讀。舊 4 顆參考見 `uno_car_vl53/`（勿燒）。

## 燒錄

| 板子 | 檔案 |
|------|------|
| Arduino UNO | `uno_car/uno_car.ino`（馬達 + 正前 VL53） |
| 車上 ESP32-UWB（Anchor） | `uwb_car_responder/uwb_car_responder.ino` |
| 身上 Tag ESP32 | 外掛 `uwb-follow-car` → `uwb_tag_initiator/`（沿用） |

UNO 函式庫：`BMP73T104`、Pololu `VL53L0X`  
ESP32 函式庫：`dw3000`

## 接線

```
ESP32 GPIO17 → UNO D10   （馬達命令）
UNO D11 → ESP32 GPIO16   （O: 正前 mm）
GND 共地

VL53×1 正前：
  VCC→UNO 5V  GND→GND  SDA→A4  SCL→A5  XSHUT→D3
  位址：預設 0x29（單顆不必改址）
```

## 行為

- 前進 `F` 且正前 **&lt; 250mm** → 強制 `S`（UNO + Pi 雙保險）  
- `L`／`R`／`Z`：**不**做側／後避障（無對應感測器）  
- `X` 最高優先  
- 遙測：`O:320`（單一正前 mm）
