# 韌體：馬達轉發（VL53 已停用）

> **※ 2026-11-25 註解：4×VL53L0X 避障與 `O:` 遙測已從程式拔除。**  
> 請燒 **`uno_car/uno_car.ino`**。舊檔 `uno_car_vl53/` 僅供日後復用，勿再燒。

## 燒錄

| 板子 | 檔案 |
|------|------|
| Arduino UNO | `uno_car/uno_car.ino`（純馬達） |
| 車上 ESP32-UWB（Anchor） | `uwb_car_responder/uwb_car_responder.ino` |
| 身上 Tag ESP32 | 外掛 `uwb-follow-car` → `uwb_tag_initiator/`（沿用） |

UNO 函式庫：`BMP73T104`（**不需** Pololu VL53L0X）  
ESP32 函式庫：`dw3000`

## 接線

```
ESP32 GPIO17 → UNO D10   （馬達命令）
UNO D11 → ESP32 GPIO16   （保留；目前不送 O:）
GND 共地

※ VL53×4／XSHUT D2～D5／I2C：本輪可不接
```

## 行為

- UNO 原樣執行 Pi 下達的 `F/L/R/Z/S/X`（**無避障覆寫**）  
- `X` 緊急煞停；命令逾時自動停  
- ESP32：USB 收馬達命令轉 Serial2；輸出 `D:x.xx`；**不轉傳 O:**
