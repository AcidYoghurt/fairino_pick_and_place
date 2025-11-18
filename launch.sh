## 摄像头
ros2 launch orbbec_camera gemini.launch.xml depth_registration:=true enable_d2c_viewer:=true

## 手眼标定
ros2 launch easy_handeye2 calibrate.launch.py \
name:=eye_in_hand_calibrate \
calibration_type:=eye_in_hand \
tracking_base_frame:=camera_link \
tracking_marker_frame:=aruco_marker_frame \
robot_base_frame:=base_link \
robot_effector_frame:=wrist3_link


ros2 launch easy_handeye2_demo calibrate.launch.py calibration_type:=eye_in_hand \
tracking_base_frame:=tr_base \
tracking_marker_frame:=tr_marker \
robot_base_frame:=panda_link0 \
robot_effector_frame:=panda_link8
