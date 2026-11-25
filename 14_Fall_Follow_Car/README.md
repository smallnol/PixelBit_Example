# 跌倒偵測 + 視覺跟隨（整合版）

把最新跌倒核心、視覺／UWB 跟隨、**1×VL53L0X 正前避障**、**特定人鎖定**、**網頁遙控／儀表板**合成一支程式（**暫不含 MaUWB AOA**）。

> **※ 2026-11-25：VL53 改為僅正前 1 顆**（左前／右前／正後已拔除）。

## 功能一覽

| 功能 | 說明 |
|------|------|
| 視覺跟隨 | ByteTrack 鎖定目標 → 左右置中；UWB 距離 → 前進／停 |
| 特定人鎖定 | 網頁鎖中央／舉手 2 秒；HSV Re-ID 抗遮擋換 ID |
| 跌倒偵測 | 只分析鎖定目標；約 10 秒確認；LINE 文字＋照片＋影片 |
| 跌倒急停 | 確認 → `X`；可自動關跟隨 |
| 正前避障 | 1×VL53；前進 `F` 且 &lt;250mm → 停（UNO＋Pi） |
| 網頁儀表板 | 手動遙控、UWB／正前 ToF、跌倒警報橫幅 |

網頁：`http://樹莓派IP:5001`

## 接線／供電／燒錄／Pi 指令（完整）

請看 **[`DEPLOY.md`](DEPLOY.md)**。  
功能＋腳位總表：**[`FEATURES_PINOUT.txt`](FEATURES_PINOUT.txt)**。

簡表：

| 板子 | 燒錄檔 |
|------|--------|
| 車上 ESP32-UWB | `firmware/uwb_car_responder/uwb_car_responder.ino` |
| Arduino UNO | `firmware/uno_car/uno_car.ino`（馬達 + **1×正前 VL53**） |
| 身上 Tag ESP32 | `uwb-follow-car` 的 `uwb_tag_initiator`（沿用） |
| 樹莓派 | 不燒韌體；跑 `fall_follow_car.py` + `line_media.py` |

★ UNO D10 ← ESP32 GPIO17（馬達）  
★ UNO D11 → ESP32 GPIO16（`O:mm`，**要接**；馬達命令才是 D10）  
★ VL53 正前：VCC/GND/SDA/SCL；**XSHUT 不接**  
★ 邏輯電：**3S 18650 → DC → XY3606 → USB-C → Pi 5**

## 操作順序（建議）

1. 開網頁預覽，確認 UWB 與正前 ToF 有數值  
2. **鎖定畫面中央目標**（或舉手 2 秒）→ 綠框 `TARGET LOCKED`  
3. 開啟跟隨／跌倒偵測  
4. 注意：僅正前有雷射防護，側／後方仍需人工注意

## 檔案

- `PROJECT_HANDBOOK.md` — 專題說明書
- `專題問答.txt` — 口試／報告備稿
- `FEATURES_PINOUT.txt` — 功能＋腳位總表
- `FOLLOW_SCENARIOS.md` — 跟隨情境（單顆正前）
- `DEPLOY.md` — 接線／燒錄／Pi 指令
- `fall_follow_car.py` — 主程式
- `line_media.py` — 跌倒核心
- `firmware/` — UNO／ESP32 韌體
