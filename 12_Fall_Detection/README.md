# 樹莓派 5 跌倒偵測系統（YOLO-pose + LINE）

即時骨架預覽、雙重跌倒判定、LINE 文字／照片通報，並支援「疑似跌倒短片」：

- **疑似跌倒**：開始暫存短片（含跌倒前約 30 秒前情）
- **確認跌倒**（持續滿 10 秒）：保留媒體，推播 LINE（文字 + 照片 + 影片）；並呼叫 `FOLLOW_ESTOP_URL`（預設 `http://127.0.0.1:5001/estop`）讓 AOA 跟隨服務送 `X` 急停
- **中途站起來／關閉偵測／畫面失去人體**：自動刪除暫存短片與暫存照片

AOA 跟隨程式見 `../14_MaUWB_AOA_Follow/`。

## 安裝

```bash
cd 12_Fall_Detection
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
# 編輯 .env 填入 LINE Token、User ID
```

## 設定 `.env`

| 變數 | 說明 |
|------|------|
| `LINE_CHANNEL_ACCESS_TOKEN` | LINE Messaging API Channel Access Token |
| `LINE_USER_ID` | 接收通報的 User ID |
| `PUBLIC_BASE_URL` | 對外 HTTPS 網址（cloudflared / ngrok），LINE 才能抓照片／影片 |
| `IMGBB_API_KEY` | 可選；沒有公開網址時用來上傳**照片** |

> 安全提醒：若 Token 曾貼到聊天室或程式碼，請到 LINE Developers **立刻重發**，不要再用舊 Token。

## 讓 LINE 收得到照片／影片

LINE 的 image／video 訊息只接受 **HTTPS 公開 URL**，不能直接塞本機檔案。

### 方式 A（建議）：cloudflared 隧道

```bash
# 另開一個終端
cloudflared tunnel --url http://127.0.0.1:5000
```

把顯示的 `https://xxxx.trycloudflare.com` 填進 `.env` 的 `PUBLIC_BASE_URL`，再啟動本程式。

### 方式 B：ImgBB（僅照片）

到 [api.imgbb.com](https://api.imgbb.com/) 申請 key，填入 `IMGBB_API_KEY`。  
此方式可推播照片，但**無法**靠 ImgBB 推播影片。

## 執行

```bash
python fall_detection.py
```

瀏覽器開啟：`http://樹莓派IP:5000`

1. 預設為骨架測試（不發 LINE）
2. 按「開啟跌倒偵測」後才會計時與通報
3. 確認事件保存在 `events/confirmed/`
4. 未確認的暫存位於 `events/pending/`（恢復後會刪除）

## 關於「能不能拍跌倒當下的影片？」

可以。本程式在疑似跌倒時就開始錄短片（含跌倒前緩衝），確認後保留並可經 `PUBLIC_BASE_URL` 推到 LINE。

LINE 影片限制概要：HTTPS、MP4、需預覽圖；請保持短片精簡（本系統約為確認秒數 + 3 秒 pre-roll）。

## 關於「快速爬起來，疑似片段會不會自動刪除？」

會。若在 `FALL_CONFIRM_SECONDS`（預設 10 秒）內恢復站立／坐姿、手動停止偵測，或畫面失去人體，會關閉錄影並刪除 `events/pending/` 內該次暫存。
