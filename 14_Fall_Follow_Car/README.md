# 跌倒偵測 + 視覺跟隨（整合版）

把最新跌倒核心、視覺／UWB 跟隨、**4×VL53L0X（含下視防墜）**、**特定人鎖定**、**網頁遙控／儀表板**合成一支程式（**暫不含 MaUWB AOA**）。

## 功能一覽

| 功能 | 說明 |
|------|------|
| 視覺跟隨 | ByteTrack 鎖定目標 → 左右置中；UWB 距離 → 前進／停 |
| 特定人鎖定 | 網頁鎖中央／舉手 2 秒；HSV Re-ID 抗遮擋換 ID |
| 跌倒偵測 | 只分析鎖定目標；約 10 秒確認；LINE 文字＋照片＋影片 |
| 跌倒急停 | 確認 → `X`；可自動關跟隨 |
| 底層避障／防墜 | 左前／正前／右前避障；**前端下視**遇落差強制停 |
| 網頁儀表板 | 手動遙控、UWB／ToF 即時顯示、跌倒警報橫幅 |

網頁：`http://樹莓派IP:5001`

## 接線／供電／燒錄／Pi 指令（完整）

請看 **[`DEPLOY.md`](DEPLOY.md)**（含供電規則、每根訊號線、要燒哪些 `.ino`、樹莓派 curl／venv 指令）。

簡表：

| 板子 | 燒錄檔 |
|------|--------|
| 車上 ESP32-UWB | `firmware/uwb_car_responder/uwb_car_responder.ino` |
| Arduino UNO | `firmware/uno_car_vl53/uno_car_vl53.ino`（4×VL53） |
| 身上 Tag ESP32 | `uwb-follow-car` 的 `uwb_tag_initiator`（沿用） |
| 樹莓派 | 不燒韌體；跑 `fall_follow_car.py` + `line_media.py` |

★ **UNO D11 → ESP32 GPIO16**（`O:` 回傳）  
★ VL53：腳 **2/3/4/5** = 左前／正前／右前／**下視防墜**

## 操作順序（建議）

1. 開網頁預覽，確認 UWB／ToF 有數值  
2. **鎖定畫面中央目標**（或舉手 2 秒）→ 綠框 `TARGET LOCKED`  
3. 開啟跟隨／跌倒偵測  
4. 跟隨 OFF 時可用手動方向鍵；跟隨 ON 時方向鍵禁用（急停仍可用）

## 檔案

- `PROJECT_HANDBOOK.md` — 專題說明書
- `DEPLOY.md` — 接線／供電／燒錄／Pi 指令
- `web_preview/` — **網頁版面預覽**（靜態 HTML，不連 Pi）
- `fall_follow_car.py` — 主程式
- `line_media.py` — 跌倒核心
- `firmware/` — UNO／ESP32 韌體
- `test_motor_relay.py` — 可選馬達轉發測試
