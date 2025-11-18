cmake_minimum_required(VERSION 3.16)
project(item_recognition)
set(USE_SCOPED_HEADER_INSTALL_DIR TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ros2 部分
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()  # 这里的依赖全都写在package.xml

ament_auto_add_executable(
        point_cloud_extractor_qr src/point_cloud_extractor_qr.cpp
)

ament_auto_add_executable(
        wechat_qr_detection src/wechat_qr_detection.cpp
)

ament_auto_add_executable(
        collision_object_manage src/collision_object_manage.cpp
)

# 如果启用了测试，则寻找测试依赖
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 禁用版权和cpplint检查
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        config
        launch
)