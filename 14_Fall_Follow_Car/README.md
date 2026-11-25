# 跌倒偵測 + 視覺跟隨（整合版）

把最新跌倒核心、視覺／UWB 跟隨、**特定人鎖定**、**網頁遙控／儀表板**合成一支程式（**暫不含 MaUWB AOA**）。

> **※ 2026-11-25 註解：VL53L0X 已全部拔除（程式＋韌體）。**  
> 目前無近距雷射避障、無 `O:` 遙測、網頁不再顯示 ToF。  
> 日後若要接回，舊參考：`firmware/uno_car_vl53/`（請勿直接燒，見該目錄 README）。

## 功能一覽

| 功能 | 說明 |
|------|------|
| 視覺跟隨 | ByteTrack 鎖定目標 → 左右置中；UWB 距離 → 前進／停 |
| 特定人鎖定 | 網頁鎖中央／舉手 2 秒；HSV Re-ID 抗遮擋換 ID |
| 跌倒偵測 | 只分析鎖定目標；約 10 秒確認；LINE 文字＋照片＋影片 |
| 跌倒急停 | 確認 → `X`；可自動關跟隨 |
| ~~底層避障~~ | **已停用**（原 4×VL53 左前／正前／右前／正後） |
| 網頁儀表板 | 手動遙控、UWB 即時顯示、跌倒警報橫幅 |

網頁：`http://樹莓派IP:5001`

## 接線／供電／燒錄／Pi 指令（完整）

請看 **[`DEPLOY.md`](DEPLOY.md)**。  
功能＋腳位總表：**[`FEATURES_PINOUT.txt`](FEATURES_PINOUT.txt)**。

簡表：

| 板子 | 燒錄檔 |
|------|--------|
| 車上 ESP32-UWB | `firmware/uwb_car_responder/uwb_car_responder.ino` |
| Arduino UNO | `firmware/uno_car/uno_car.ino`（**純馬達，無 VL53**） |
| 身上 Tag ESP32 | `uwb-follow-car` 的 `uwb_tag_initiator`（沿用） |
| 樹莓派 | 不燒韌體；跑 `fall_follow_car.py` + `line_media.py` |

★ UNO D10 ← ESP32 GPIO17（馬達命令）  
★ UNO D11 → ESP32 GPIO16（接線可留；目前不送 `O:`）  
★ 邏輯電：**3S 18650 → DC → XY3606 → USB-C → Pi 5**  
★ **VL53／XSHUT D2～D5／I2C：本輪可不接**

## 操作順序（建議）

1. 開網頁預覽，確認 UWB 有數值  
2. **鎖定畫面中央目標**（或舉手 2 秒）→ 綠框 `TARGET LOCKED`  
3. 開啟跟隨／跌倒偵測  
4. 跟隨 OFF 時可用手動方向鍵；跟隨 ON 時方向鍵禁用（急停仍可用）  
5. **注意：無 VL53 時請自行看路，勿高速撞障**

## 檔案

- `PROJECT_HANDBOOK.md` — 專題說明書
- `專題問答.txt` — 口試／報告備稿
- `FEATURES_PINOUT.txt` — 功能＋腳位總表（核對用）
- `FOLLOW_SCENARIOS.md` — 七種跟隨情境（已註明 VL53 停用後行為）
- `DEPLOY.md` — 接線／供電／燒錄／Pi 指令
- `web_preview/` — **網頁版面預覽**（靜態 HTML，不連 Pi）
- `fall_follow_car.py` — 主程式
- `line_media.py` — 跌倒核心
- `firmware/` — UNO／ESP32 韌體
- `test_motor_relay.py` — 可選馬達轉發測試
