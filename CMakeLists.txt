cmake_minimum_required(VERSION 3.10.2)

add_definitions("-std=c++17")

project(bias_calibration)


find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(${PROJECT_NAME}
	main.cpp 
	imu_tk/io_utils.cpp 
	imu_tk/filters.cpp
	imu_tk/calibration.cpp
	)

target_link_libraries(${PROJECT_NAME}
	${CERES_LIBRARIES}
	stdc++fs)

target_include_directories(${PROJECT_NAME} PUBLIC
	${Ceres_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR})

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
       COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_SOURCE_DIR}
       )