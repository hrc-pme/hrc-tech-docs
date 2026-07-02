# OpenArm

![](../assets/robot/openarm4.png){width="300px"}

## 1. Hardware

### 1-1. Power

將電源開啟，即可啟動所有馬達

![](../assets/robot/openarm1.jpg){width="400px"}

???+ info "燈顯"

    如圖中馬達常亮紅色，代表機器在待機狀態，
    接收指令後會轉成綠色。

---

### 1-2. Emergency Stop

緊急開關位於機器底座旁，拍下去只會將馬達斷電，電腦及相機等感測器仍正常運作。

![](../assets/robot/openarm2.jpg){width="300px"}
---

## 2. Software

<!-- 描述 OpenArm 的軟體設置 -->

> Repository: https://github.com/hrc-pme/openarm.git


開啟步驟：

1. Clone repository
   ```bash
   git clone https://github.com/hrc-pme/openarm.git
   cd openarm
   ```

2. 啟動控制腳本
   ```bash
   # Source ROS2 環境
   cd ~/ros2_ws && colcon build
   source ~/ros2_ws/install/setup.bash
   # 啟動控制節點
   ros2 launch openarm_bringup openarm.launch.py arm_type:=v10 use_fake_hardware:=true
   ```

---

