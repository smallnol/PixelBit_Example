# 運作邏輯｜1×正前 VL53（無 AOA）

版本：`fall-follow-2026-11-25-vl53-front`  
實作：`fall_follow_car.py` + `uno_car/uno_car.ino` + `uwb_car_responder.ino`

---

## 一、硬體資料流（每迴圈）

```
相機 → Pi YOLO-pose + ByteTrack → 鎖定目標左右偏移
身上 Tag --UWB--> 車上 ESP32 → USB → Pi（D:公尺）
正前 VL53 → UNO I2C → SoftSerial O:mm → ESP32 GPIO16 → USB → Pi
Pi 命令 F/L/R/Z/S/X → USB → ESP32 GPIO17 → UNO D10 → 馬達
```

接線要點：
- VL53：VCC/GND/SDA/SCL；**XSHUT 不接**
- GPIO17→D10（馬達）、**D11→GPIO16（O:）**、共地

---

## 二、Pi 跟隨決策順序（跟隨 ON 且已鎖定）

1. **跌倒確認** → `X` 急停（最高）  
2. **畫面沒有鎖定目標** → 復原鏈（見下）  
3. **有目標**：  
   - offset 偏左 → `L`；偏右 → `R`  
   - 置中：看 UWB → ≥`START_DIST`(2.0m) 則 `F`；&lt;`STOP_DIST`(1.8m) 則 `S`；中間帶保持上一態  
4. **正前 ToF**：若命令是 `F` 且 `tof_mm < AVOID_MM`(250) 且資料新鮮 → 改 `S`  
5. **卡住推估**：持續 `F` 且 UWB 新鮮，但 ≥4s 距離沒靠近 0.15m → 改 `S`

手動遙控：跟隨 OFF 時可用；跟隨 ON 時方向鍵鎖，`X` 永遠可。手動 `F` 同樣受正前 ToF 擋。

---

## 三、丟目標復原鏈（情境 3／4／7）

| 階段 | 時間 | 命令 |
|------|------|------|
| 寬限 | 0～1s | `S`（給 Re-ID） |
| 偏轉搜 | 接著 3s | 依 `last_seen_offset` → `L` 或 `R` |
| 自轉 | 再最多 12s | `Z` |
| 放棄 | 逾時後 | `S`（`search_exhausted`；需人入畫或解除鎖定） |

路人：不相符 HSV → **不跟**（搜尋門檻 0.72 較嚴）。

---

## 四、UNO 底層（雙保險）

- 收到 `F` 且正前 &lt;250mm → 強制 `S`（即使 Pi 漏擋）  
- `L`／`R`／`Z`／`S`：不覆寫  
- `X`：急停最高優先  
- 每 ~200ms 送 `O:mm`

---

## 五、七情境對照（現況）

| # | 情境 | 現況 |
|---|------|------|
| 1 | 走道貼牆 | 視覺轉；**無側 ToF**，可能貼牆 |
| 2 | 正前矮障 | **有**：F→S |
| 3 | 箱子擋人 | 正前停 + 復原鏈；**無後防** |
| 4 | 轉角死角 | 復原鏈（最後方位） |
| 5 | 椅腳卡住 | UWB 卡住→S；不繞行 |
| 6 | 出畫跟路人？ | 鎖定+HSV，不跟路人 |
| 7 | 骨架短暫丟 | 1s 寬限再搜 |

參數：`AVOID_MM=250`、`LOST_GRACE=1`、`SEARCH_BIAS=3`、`Z_TIMEOUT=12`、`STUCK=4s/0.15m`。
