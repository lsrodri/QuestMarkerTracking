cmake_minimum_required(VERSION 3.10)
project(aruco_unity_plugin)

# Set standard C++ version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
# For Windows: Make sure OpenCV_DIR is set to the path containing OpenCVConfig.cmake
# For Android: Pass -DOpenCV_DIR=path/to/OpenCV-android-sdk/sdk/native/jni
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Create shared library
add_library(aruco_unity_plugin SHARED ArucoUnityPlugin.cpp)

# Link OpenCV
target_link_libraries(aruco_unity_plugin ${OpenCV_LIBS})

# For Android cross-compiling, link Android log if necessary (optional)
if(ANDROID)
    find_library(log-lib log)
    target_link_libraries(aruco_unity_plugin ${log-lib})
endif()
