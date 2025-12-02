cmake_minimum_required(VERSION 3.8)
project(fairino_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

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
        rviz
        config
        launch
)

