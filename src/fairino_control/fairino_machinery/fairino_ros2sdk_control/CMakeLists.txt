cmake_minimum_required(VERSION 3.10)
project(fairino_ros2sdk_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
        fairino_ros2sdk_control src/fairino_ros2sdk_control.cpp
)

target_link_libraries(fairino_ros2sdk_control yaml-cpp)

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
