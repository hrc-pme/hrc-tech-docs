# OpenDuck

![](../assets/robot/duck-thumbnail.png){width="200px"}

## 1. Hardware

### 1-1. Power

使用行充供電給樹莓派 Rpi5 即可，馬達控制板由樹莓派 USB 線供電。

---

## 2. Software

由於 Stretch3 使用者多，每人開發習慣不同，也使用不同 repo。  
以下開啟方式為 @pomelo925 維護，如果須特定功能請找相關的開發者哦。

> Repository: https://github.com/hrc-pme/openduck-isaac.git

開啟步驟：

1. 將樹莓派開機，並使用 SSH 遠端登入。  
2. 找空的資料夾路徑(如 `/home/myname/` )，下載此 repo。  
   ```
   git clone https://github.com/hrc-pme/openduck-isaac.git
   cd openduck-isaac
   ```
3. 在 repo 根目錄執行 `source run.sh`，會跳出頁面如下。
   
    ![duck-run](../assets/robot/duck-run.png)

    不同的功能、運算平台已經被封裝成獨立的容器，其中：
    
    * openduck_robot: 控制馬達、訓練用環境。
    * social_navigation: 社交導航環境。
  
        ??? info "選單後續選項"

            可決定環境是否使用 gpu 加速運算。  
            cu129 表示支援 CUDA 12.9，須注意硬體是否支援。
      
            ![](../assets/robot/duck-run2.png/)