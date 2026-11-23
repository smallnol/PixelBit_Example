# 網頁版面預覽（不連樹莓派）

純靜態 HTML，用瀏覽器直接打開即可看版面與按鈕互動（模擬狀態）。

## 開啟方式

1. 下載或 clone 後，用瀏覽器打開：
   ```
   14_Fall_Follow_Car/web_preview/index.html
   ```
2. 或 GitHub 上點進檔案後用 Raw／本地另存再開。

線上 Raw（可另存）：  
https://raw.githubusercontent.com/smallnol/PixelBit_Example/cursor/fall-follow-vl53-lock-dash-1509/14_Fall_Follow_Car/web_preview/index.html

> GitHub 網頁上直接預覽 HTML 有限；建議下載後雙擊用 Chrome／Edge 開啟。

## 與實機差異

| 預覽 | 實機（Pi :5001） |
|------|------------------|
| SVG 示意畫面 | 真實 MJPEG `video_feed` |
| 按鈕改本地 mock | 呼叫 Flask API／馬達 |
| 上方紫色「模擬」按鈕 | 無（實機靠真實感測） |
