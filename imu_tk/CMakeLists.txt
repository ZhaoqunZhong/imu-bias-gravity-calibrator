cmake_minimum_required(VERSION 3.10.2)

project(imu_tk)

set(imu_tk_SRCS
        ${CMAKE_SOURCE_DIR}/platform/native_debug.cpp
        calibration.cpp
        filters.cpp
        io_utils.cpp
        )

add_library(imu_tk SHARED
        ${imu_tk_SRCS})

target_include_directories(imu_tk PUBLIC
        ${CMAKE_SOURCE_DIR}/lib-includes
        ${CMAKE_SOURCE_DIR}/platform
        )

#set(jniLibs_DIR ${CMAKE_SOURCE_DIR}/../jniLibs/${ANDROID_ABI})

target_link_libraries(imu_tk
        log
        android
        ${jniLibs_DIR}/libceres.so
        )