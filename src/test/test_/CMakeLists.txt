cmake_minimum_required(VERSION 3.16)
project(test_)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()  # 这里的依赖全都写在package.xml

# 链接法奥CPP sdk
set(FAIRINO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/test_/fairino_cpp_sdk")
# 包含头文件路径
include_directories(
        ${FAIRINO_SDK_PATH}/include
)
# 链接库路径
link_directories(
        ${FAIRINO_SDK_PATH}/lib
)

ament_auto_add_executable(
        test_tf src/test_tf.cpp
)

ament_auto_add_executable(
        test_faiirino_mtc_control src/test_faiirino_mtc_control.cpp
)

ament_auto_add_executable(
        aaa src/aaa.cpp
)

ament_auto_add_executable(
        test_ros2_sdk src/test_ros2_sdk.cpp
)

ament_auto_add_executable(
        test_fairino_cpp_sdk src/test_fairino_cpp_sdk.cpp
)

target_link_libraries(test_fairino_cpp_sdk
        fairino
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
        urdf
        meshes
        launch
)