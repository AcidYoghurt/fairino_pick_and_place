# rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

colcon build --symlink-install

# 编译特定包
# colcon build --symlink-install --packages-select
