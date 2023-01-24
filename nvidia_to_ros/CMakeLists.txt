cmake_minimum_required (VERSION 3.1)
project(nvidia_to_ros)

if(EXISTS "/etc/nv_tegra_release")

message(WARNING "nvidia_to_ros JETSON VERSION")
  
set(ARGUS_DIR /usr/src/jetson_multimedia_api/argus)
set(JETSON_MM_DIR /usr/src/jetson_multimedia_api/)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake "${CMAKE_MODULE_PATH}")
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)


find_package(CUDA REQUIRED)
find_package(Argus REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge std_msgs rosbag)

include_directories(/usr/include/libdrm include)

find_library(NVBUFF_LIB REQUIRED NAMES nvbuf_utils HINTS /usr/lib/aarch64-linux-gnu/tegra )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ARGUS_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${JETSON_MM_DIR}/include/libjpeg-8b
  ${JETSON_MM_DIR}/include/
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS std_msgs
               INCLUDE_DIRS include ${ARGUS_INCLUDE_DIR}
               LIBRARIES nv2ros)

add_library(nv2ros src/nv_server_client.cpp src/nv2ros.cpp)
target_link_libraries(nv2ros
  ${catkin_LIBRARIES}
  ${ARGUS_LIBRARIES}
  ${JETSON_MM_DIR}/../../lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so
  ${OpenCV_LIBRARIES}
  )





cuda_add_executable(nv_consumer src/nv_consumer.cpp)
target_link_libraries(nv_consumer
  ${catkin_LIBRARIES}
  nv2ros
  )

cuda_add_executable(nv_log src/nv_log.cpp)
target_link_libraries(nv_log
  ${catkin_LIBRARIES}
  nv2ros
  )

cuda_add_executable(nv_log_to_bag src/nv_log_to_bag.cpp)
target_link_libraries(nv_log_to_bag
  ${catkin_LIBRARIES}
  nv2ros
  )

cuda_add_executable(hz src/hz.cpp)
target_link_libraries(hz
  ${catkin_LIBRARIES}
  nv2ros
  )


cuda_add_executable(nv_producer src/nv_producer.cpp)
target_link_libraries(nv_producer
  ${catkin_LIBRARIES}
  nv2ros
  )

add_executable(nv_transform src/nv_transform.cpp)
target_link_libraries(nv_transform
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nv2ros
  )


cuda_add_executable(nv2gst src/nv2gst.cpp)
target_include_directories(nv2gst
  PUBLIC
  ${ARGUS_DIR}/samples/utils
  ${ARGUS_DIR}/../include
  PRIVATE
  ${GTK3_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  /usr/include/gstreamer-1.0/
)
target_link_libraries(nv2gst
  ${catkin_LIBRARIES}
  ${ARGUS_LIBRARIES}
  ${CUDA_LIBRARIES}
  cuda
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
  "EGL" "GLESv2"
  nv2ros
  )

cuda_add_executable(gst2nv src/gst2nv.cpp)
target_include_directories(gst2nv
  PUBLIC
  ${ARGUS_DIR}/samples/utils
  ${ARGUS_DIR}/../include
  PRIVATE
  ${GTK3_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  /usr/include/gstreamer-1.0/
)
target_link_libraries(gst2nv
  ${catkin_LIBRARIES}
  ${ARGUS_LIBRARIES}
  ${CUDA_LIBRARIES}
  #utils_lib
  cuda
  #classes_lib
  ${JETSON_MM_DIR}/../../lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so
  /usr/lib/aarch64-linux-gnu/libdrm.so
  /usr/lib/aarch64-linux-gnu/libv4l2.so
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
  "EGL" "GLESv2"
  nv2ros
  )


cuda_add_executable(gst2bag src/gst2bag.cpp)
target_include_directories(gst2bag
  PUBLIC
  ${ARGUS_DIR}/samples/utils
  ${ARGUS_DIR}/../include
  PRIVATE
  ${GTK3_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  /usr/include/gstreamer-1.0/
)
target_link_libraries(gst2bag
  ${catkin_LIBRARIES}
  ${ARGUS_LIBRARIES}
  ${CUDA_LIBRARIES}
  #utils_lib
  cuda
  #classes_lib
  ${JETSON_MM_DIR}/../../lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so
  /usr/lib/aarch64-linux-gnu/libdrm.so
  /usr/lib/aarch64-linux-gnu/libv4l2.so
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
  "EGL" "GLESv2"
  nv2ros
  )


cuda_add_executable(ros2nv src/ros2nv.cpp)
target_include_directories(ros2nv
  PUBLIC
  ${ARGUS_DIR}/samples/utils
  ${ARGUS_DIR}/../include
  PRIVATE
  ${GTK3_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  /usr/include/gstreamer-1.0/
)
target_link_libraries(ros2nv
  ${catkin_LIBRARIES}
  nv2ros
  )


include(CPack)

else()


message(WARNING "nvidia_to_ros NON-JETSON VERSION")
  
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge std_msgs rosbag message_generation message_filters)

include_directories(include
  include/nvidia_to_ros/non_jetson
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

add_definitions(-DNON_JETSON)

add_message_files(FILES
  ImageAndCameraInfo.msg
  )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  )

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
               INCLUDE_DIRS include
               LIBRARIES nv2ros)

	     
add_library(nv2ros src/nv_server_client.cpp src/nv2ros_non_jetson.cpp)
target_link_libraries(nv2ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )



add_executable(nv_consumer src/nv_consumer.cpp)
target_link_libraries(nv_consumer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nv2ros
  )

add_executable(nv_producer src/nv_producer.cpp)
target_link_libraries(nv_producer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nv2ros
  )

add_executable(nv_transform src/nv_transform.cpp)
target_link_libraries(nv_transform
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nv2ros
  )

add_executable(ros2nv src/ros2nv.cpp)
target_link_libraries(ros2nv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nv2ros
  )

message(WARNING "DSFJHSKHFJFKSDHKHKDSFKFHDKSHFJKSHFJDSHFKS")

endif()
