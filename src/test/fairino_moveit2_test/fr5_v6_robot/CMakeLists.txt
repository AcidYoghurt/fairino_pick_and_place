cmake_minimum_required(VERSION 3.16)
project(fr5_v6_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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
        launch
        config
        meshes
        urdf
)
