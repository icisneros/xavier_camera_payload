cmake_minimum_required (VERSION 3.1)
project(argus_multicam)
#add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)



set(ARGUS_DIR /usr/src/jetson_multimedia_api/argus)
set(JETSON_MM_DIR /usr/src/jetson_multimedia_api/)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake "${CMAKE_MODULE_PATH}")
set(CMAKE_CXX_FLAGS "-O3")
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

pkg_check_modules(GSTREAMER QUIET gstreamer-0.10)
if(NOT GSTREAMER_FOUND)
  set(GSTREAMER_VERSION_1_x TRUE)
endif()
if(GSTREAMER_VERSION_1_x)
  message(STATUS "gst 1.0")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
  pkg_check_modules(GSTAPP REQUIRED gstreamer-app-1.0)
  pkg_check_modules(GSTPB REQUIRED gstreamer-pbutils-1.0>=1.4)

else()
  message(STATUS "gst 0.1")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-0.10)
endif()

find_package(CUDA REQUIRED)
find_package(Argus REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge message_generation geometry_msgs nvidia_to_ros)

include_directories(
  include
  /usr/include/libdrm
  ${GSTREAMER_INCLUDE_DIRS}

  )

add_message_files(
  FILES
  MultiStamps.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_library(classes_lib
  ${JETSON_MM_DIR}/samples/common/classes/NvApplicationProfiler.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvElementProfiler.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvV4l2Element.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvBuffer.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvJpegDecoder.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvV4l2ElementPlane.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvDrmRenderer.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvJpegEncoder.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvVideoConverter.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvEglRenderer.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvLogging.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvVideoDecoder.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvElement.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvUtils.cpp
  ${JETSON_MM_DIR}/samples/common/classes/NvVideoEncoder.cpp
)

add_library(utils_lib
  ${ARGUS_DIR}/samples/utils/ArgusHelpers.cpp
  ${ARGUS_DIR}/samples/utils/CUDAHelper.cpp
  ${ARGUS_DIR}/samples/utils/EGLGlobal.cpp
  ${ARGUS_DIR}/samples/utils/Observed.cpp
  ${ARGUS_DIR}/samples/utils/Thread.cpp
  ${ARGUS_DIR}/samples/utils/WindowBase.cpp
  ${ARGUS_DIR}/samples/utils/gtk/GuiElement.cpp
  ${ARGUS_DIR}/samples/utils/gtk/Window.cpp
)

find_library(NVBUFF_LIB REQUIRED NAMES nvbuf_utils HINTS /usr/lib/aarch64-linux-gnu/tegra )



target_link_libraries(utils_lib
  ${GTK3_LIBRARIES}
  classes_lib
)

target_include_directories(utils_lib
  PUBLIC
  ${ARGUS_DIR}/samples/utils
  ${ARGUS_DIR}/../include
  PRIVATE
  ${GTK3_INCLUDE_DIRS}
)

set(SOURCES src/argus_main.cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ARGUS_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${JETSON_MM_DIR}/include/libjpeg-8b
  ${JETSON_MM_DIR}/include/
  ${OpenCV_INCLUDE_DIRS}
  ${GSTAPP_INCLUDE_DIRS}
  ${nvidia_to_ros_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS message_runtime)
cuda_add_executable(${PROJECT_NAME} ${SOURCES})
#cuda_add_executable(${PROJECT_NAME}_dual ${OTHER_SOURCES})


#link_directories(
#${JETSON_MM_DIR}/../../lib/aarch64-linux-gnu/tegra/
#)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ARGUS_LIBRARIES}
  ${CUDA_LIBRARIES}
  utils_lib
  cuda
  classes_lib
  ${JETSON_MM_DIR}/../../lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so
  # /usr/lib/aarch64-linux-gnu/libdrm.so
  # /usr/lib/aarch64-linux-gnu/libv4l2.so
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
   ${GSTAPP_LIBRARIES}
   ${GSTPB_LIBRARIES}
  )


cuda_add_executable(${PROJECT_NAME}_fast src/argus_main_fast.cpp)

target_link_libraries(${PROJECT_NAME}_fast
  ${catkin_LIBRARIES}
  ${ARGUS_LIBRARIES}
  ${CUDA_LIBRARIES}
  utils_lib
  cuda
  classes_lib
  ${JETSON_MM_DIR}/../../lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so
  # /usr/lib/aarch64-linux-gnu/libdrm.so
  # /usr/lib/aarch64-linux-gnu/libv4l2.so
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
   ${GSTAPP_LIBRARIES}
   ${GSTPB_LIBRARIES}
   ${nvidia_to_ros_LIBRARIES}
   )


include(CPack)
