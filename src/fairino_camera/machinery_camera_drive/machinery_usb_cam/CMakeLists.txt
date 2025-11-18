cmake_minimum_required(VERSION 3.10)
project(machinery_usb_cam)
set(USE_SCOPED_HEADER_INSTALL_DIR TRUE)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
        rectify_node src/rectify_node.cpp
)

target_link_libraries(rectify_node ${OpenCV_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_auto_package(
        INSTALL_TO_SHARE
        config
        launch
)

