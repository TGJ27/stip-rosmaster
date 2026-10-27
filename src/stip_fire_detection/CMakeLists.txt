cmake_minimum_required(VERSION 3.0.2)
project(stip_fire_detection)

add_compile_options(-O3 -std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(OpenCV REQUIRED)
find_package(CUDA 11.4 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

# Jetson Orin / CUDA 11.4. The CUDA kernels are built only for the Orin GPU.
list(APPEND CUDA_NVCC_FLAGS "-O3" "-std=c++14" "-gencode=arch=compute_87,code=sm_87")

# JetPack 5 / TensorRT 8.5 locations.
find_path(TENSORRT_INCLUDE_DIR
  NvInfer.h
  HINTS
    /usr/include/aarch64-linux-gnu
    /usr/include
)

find_library(NVINFER_LIB
  nvinfer
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib
)

find_library(NVINFER_PLUGIN_LIB
  nvinfer_plugin
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib
)

find_path(CUDA_INCLUDE_DIR
  cuda_runtime_api.h
  HINTS
    /usr/local/cuda/include
)

find_library(CUDART_LIB
  cudart
  HINTS
    /usr/local/cuda/lib64
    /usr/local/cuda/targets/aarch64-linux/lib
)

if(NOT TENSORRT_INCLUDE_DIR)
  message(FATAL_ERROR "TensorRT NvInfer.h not found")
endif()

if(NOT NVINFER_LIB)
  message(FATAL_ERROR "TensorRT nvinfer library not found")
endif()

if(NOT NVINFER_PLUGIN_LIB)
  message(FATAL_ERROR "TensorRT nvinfer_plugin library not found")
endif()

if(NOT CUDA_INCLUDE_DIR)
  message(FATAL_ERROR "CUDA headers not found")
endif()

if(NOT CUDART_LIB)
  message(FATAL_ERROR "CUDA cudart library not found")
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

link_directories(${GST_LIBRARY_DIRS})
add_definitions(${GST_CFLAGS_OTHER})

cuda_add_executable(
  fire_detector_cpp
  src/fire_detector_cpp.cpp
  src/fire_hw_jpeg.cpp
  src/fire_cuda.cu
)

target_link_libraries(
  fire_detector_cpp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${NVINFER_LIB}
  ${NVINFER_PLUGIN_LIB}
  ${CUDART_LIB}
  ${CUDA_LIBRARIES}
  ${GST_LIBRARIES}
)

add_dependencies(
  fire_detector_cpp
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS fire_detector_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/fire_viewer.py
  tools/test_engine.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
