# Tutorials

> 完成 [O-Week](../oweek/) 後，請依序閱讀以下教學。  
> 內容涵蓋實驗室開發慣例、專案結構、機器人連線與 ROS 2 跨機通訊設定。

!!! abstract "學習路徑"

    教學編號反映建議閱讀順序：先建立 Git 與 Repository 習慣，再進行實機網路與 Middleware 設定。

| # | 主題 | 說明 |
|---|------|------|
| 1 | [Commit](1.commit/) | Git commit 格式與撰寫慣例 |
| 2 | [Gitignore](2.gitignore/) | 專案中應忽略哪些檔案 |
| 3 | [Project-based Repository](3.repo/) | 實驗室標準專案目錄結構 |
| 4 | [ETH over SSH](4.eth-over-ssh/) | 乙太網路直連與靜態 IP 設定 |
| 5 | [Zenoh Middleware](5.zenoh-middleware/) | 以 rmw_zenoh 進行跨機 ROS 2 通訊 |

!!! tip "與 O-Week 的關係"

    - **§1–§3** 延伸 O-Week Git / Docker 內容，適用於開始 contribute 或建立新專案時。
    - **§4–§5** 面向機器人實驗場景，建議在需要連線至實機或跨機 ROS 2 時再閱讀。
