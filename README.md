# 机械臂抓取流程
## 流程
- 入柜 —— 抓取
- 入柜 —— 放置
- 出柜 —— 抓取
- 出柜 —— 放置
- demo —— 打开夹爪
- demo —— 关闭夹爪

## 启动事项
1. 如果要直接启动项目
```bash
# 在ros2_ws打开终端
. launch.sh
```
<br />

2. 修改点位然后启动项目
```bash
# 在ros2_ws打开终端
python3 fair_ros.py
# 在里面右下角有一个launch的选项
```
<br />

3. 如果点位修改后异常怎么办
```bash
# 在ros2_ws打开终端，然后修改点位
gedit /ros2_ws/fairino_bringup/config/custom_points_config.yaml
# WARNING：这里面的 item_above_car_point 和 item_in_car_point 都必须是double类型
```
<br />


## 测试指令
### 物品入柜  抓取
```bash
ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"pick_up\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_store\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"
```

### 物品入柜  放置
```bash
ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"place\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_store\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"
```

### 物品出柜  抓取
```bash
ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"pick_up\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_outbound\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"
```

### 物品出柜  放置
```bash
ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"place\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{\"task_type\":\"item_outbound\",\"item_id\":5,\"cabinet_id\":[1,4]}}'}"
```

### 演示demo  张开夹爪
```bash
ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"open_gripper\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{}}'}"
```

### 演示demo  关闭夹爪
```bash
ros2 topic pub /tcp_to_ros_cmd std_msgs/msg/String "{data: '{\"code\":200,\"action\":\"close_gripper\",\"message\":\"夹取\",\"datetime\":\"2025-11-22 17:13:00\",\"data\":{}}'}"
```


## 各种物品网络IP
### 路由器
```bash
192.168.58.100
```
### 左A 主机（黑色）
```bash
# 用户名：zekeep
# 密码：1
192.168.58.141
```
### 左A 机械臂
```bash
192.168.58.3
```
### 右B 主机（黑色）
```bash
# 用户名：zeekeep
# 密码：1
192.168.58.142
```
### 右B 机械臂
```bash
192.168.58.2
```


# 调试（杂项）
## 摄像头
```bash
//ros2 launch orbbec_camera gemini.launch.xml depth_registration:=true enable_d2c_viewer:=true
```
```bash
#带深度
ros2 launch orbbec_camera gemini.launch.xml \
    depth_registration:=true \
    enable_d2c_viewer:=true \
    color_width:=1280 \
    color_height:=720 \
    depth_width:=1280 \
    depth_height:=720 \
    color_fps:=30 \
    depth_fps:=distortion...30 \
    depth_format:=Y16
```
```bash
#不带深度
ros2 launch orbbec_camera gemini.launch.xml \
    enable_depth:=false \
    color_width:=1920 \
    color_height:=1080 \
    color_fps:=30
```
## 手眼标定
```bash
ros2 launch easy_handeye2 calibrate.launch.py \
name:=eye_in_hand_calibrate \
calibration_type:=eye_in_hand \
tracking_base_frame:=camera_link \
tracking_marker_frame:=aruco_marker_frame \
robot_base_frame:=base_link \
robot_effector_frame:=wrist3_link
```

```bash
ros2 launch easy_handeye2_demo calibrate.launch.py calibration_type:=eye_in_hand \
tracking_base_frame:=tr_base \
tracking_marker_frame:=tr_marker \
robot_base_frame:=panda_link0 \
robot_effector_frame:=panda_link8
```