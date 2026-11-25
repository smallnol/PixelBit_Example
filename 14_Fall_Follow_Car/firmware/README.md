# 韌體：4×VL53L0X 避障 + O: 雙向轉發

## 燒錄

| 板子 | 檔案 |
|------|------|
| Arduino UNO | `uno_car_vl53/uno_car_vl53.ino` |
| 車上 ESP32-UWB | `uwb_car_responder/uwb_car_responder.ino` |

UNO 函式庫：`BMP73T104`、Pololu `VL53L0X`  
ESP32 函式庫：`dw3000`

## 接線

```
ESP32 GPIO17 → UNO D10   （馬達命令）
UNO D11 → ESP32 GPIO16   （O: 回傳）
GND 共地

VL53×4 共用 SDA/SCL/VCC/GND
XSHUT: 2=左前, 3=正前, 4=右前, 5=正後
位址：0x30～0x33
```

## 行為

- 前方三顆 &lt; 250mm：前進時強制停／微偏  
- **正後 &lt; 250mm**：自轉搜尋 `Z` 時強制停  
- `X` 最高優先  
- 遙測：`O:左前,正前,右前,正後`
