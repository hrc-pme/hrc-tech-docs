# Koch Robot Arm v1.1

![](../assets/robot/koch-thumbnail.png){width="200px"}

## 1. Hardware

### 1-1. Power
1. 檢查ttyACM設備`ls -la /dev/ttyACM*`
2. 確認`single_follower.yaml`文件中配置的`port`是否正確

---

## 2. Software

> Repository: https://github.com/hrc-pme/Lerobot_system.git

開啟步驟：

1. 手臂藉由ROS2控制，請先建構ROS2環境
2. 找空的資料夾路徑(如 `/home/myname/` )，下載此 repo。  
   ```
   git clone https://github.com/hrc-pme/Lerobot_system.git
   cd Lerobot_system
   ```
3. 執行校正腳本
    ```
    cd /Lerobot_system/ros2_ws/src/koch_control/scripts
    ./calibrate_koch.sh
    ```
4. 校正完成後，檔案會儲存到 `/home/Lerobot_system/calibration/koch/follower/ koch_right_follower_calibration.json`
5. 啟動控制腳本
    ```
    # Source ROS2 環境
    source /opt/ros/humble/setup.bash
    source /Lerobot_system/ros2_ws/install/setup.bash

    # 啟動 Follower 控制節點
    ros2 run koch_control koch_leader_follower_control 