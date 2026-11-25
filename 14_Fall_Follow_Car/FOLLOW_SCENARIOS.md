# 七種跟隨情境｜改寫後判定（無 AOA）

版本：`fall-follow-2026-11-25-follow-recovery`  
實作：`fall_follow_car.py`（Pi）+ `uno_car_vl53.ino`（UNO 側牆）

| 參數 | 預設 | 意義 |
|------|------|------|
| `SIDE_AVOID_MM` | 220 | 側牆過近不可往該側轉 |
| `AVOID_MM` | 250 | 前方／正後過近 |
| `LOST_GRACE_SECONDS` | 1.0 | 丟目標先停、不立刻自轉 |
| `SEARCH_BIAS_SECONDS` | 3.0 | 往「最後看到的左右」轉 |
| `Z_TIMEOUT_SECONDS` | 12 | 自轉逾時改停，避免無限轉 |
| `STUCK_SECONDS` / `STUCK_DIST_EPS` | 4.0 / 0.15m | F 但距離沒拉近 → 卡住 |
| `HSV_REID_THRESHOLD` / `HSV_REID_SEARCH` | 0.60 / 0.72 | 正常／搜尋中 Re-ID 門檻 |

## 1. 走道人偏左／右，車往牆靠

1. 視覺仍依 offset 下 `L`／`R`／`F`。  
2. **Pi + UNO**：若往左側轉且左前 ToF `< SIDE/AVOID` → 改 `S` 或往較空的 `R`（右側對稱）。  
3. 前進時若一側明顯較近牆（`< CORRIDOR_NUDGE` 且差 `> CORRIDOR_BIAS`）→ 微偏往通道中心。

## 2. 矮障 VL53 有、相機沒有

1. Pi 仍可能因人置中而下 `F`。  
2. **Pi `_apply_tof_motion_guard`**：前方過近立刻改 `L`／`R`／`S`（與 UNO 同邏輯雙保險）。  
3. 不會因「相機沒看到障礙」就硬撞；卡在障前則可能進卡住推估（見 5）。

## 3. 中途箱子；自轉找不到人

1. 前方堵：ToF 把 `F` 改彎／停。  
2. 人出畫／被擋：`LOST` → **先 `S` 寬限 1s** → **往 `last_seen_offset` 偏轉 3s** → 再 `Z`。  
3. `Z` 滿 **12s** 仍找不到 → **`search_exhausted`，改 `S` 停住**（網頁 `recovery` 提示）；需人再入畫 Re-ID 或按解除鎖定。  
4. 自轉時正後過近 → 強制 `S`（不背撞）。

## 4. 轉角人進死角

與 3 相同復原鏈（寬限 → 最後方位 → 自轉 → 逾時停）。  
無 AOA，不依 UWB 轉向；靠「最後畫面左右」提高轉對邊機率。

## 5. 細長物／椅腳，VL53 沒打到

無保險桿硬體。軟體推估：  
持續下 `F` 且 UWB 新鮮，但 **≥4s 距離未比基準靠近 0.15m** → 視為卡住 → 往 ToF 較空側 `L`／`R`，否則 `S`。  
（真撞上且輪子空轉／距離不變時才有機會觸發；非完美碰觸偵測。）

## 6. 鎖定人出畫，自轉會不會跟別人？

1. `locked_target_id` 保留；路人仍 IGNORED。  
2. 僅 **HSV ≥ 門檻** 才 Re-ID；搜尋中用 **`HSV_REID_SEARCH=0.72`**（較嚴）。  
3. 不相符 → 繼續搜尋／逾時停，**不會改跟路人**（誤綁風險降低但仍非零）。

## 7. 骨架暫時丟鎖定

1. `target is None` **不再立刻 `Z`**。  
2. **1s 寬限 `S`**，期間仍可 HSV Re-ID 接回。  
3. 逾寬限才進入「最後方位 → 自轉 → 逾時停」。

## 網頁

`/api/status` 新增：`recovery`、`search_exhausted`；狀態列顯示「復原」字串。
