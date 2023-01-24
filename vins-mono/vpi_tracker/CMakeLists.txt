cmake_minimum_required(VERSION 3.0.2)
include(CheckLanguage)
check_language(CUDA)
project(vpi_tracker LANGUAGES CXX CUDA)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	cv_bridge
	image_transport
	camera_model
	nvidia_to_ros
	)
find_package(vpi 1.0 REQUIRED)
find_package(OpenCV 4 REQUIRED)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
find_package(CUDA REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${nvidia_to_ros_INCLUDE_DIRS}
)

add_executable(vpi_tracker
	src/vpi_feature_tracker.cpp
    src/vpi_feature_tracker_node.cpp
	src/vpi_parameters.cpp
	src/RANSAC_cuda_tools.cu
    )

target_link_libraries(vpi_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS} vpi opencv_core opencv_imgproc)
target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES})
if(OpenCV_VERSION VERSION_LESS 3)
    target_link_libraries(${PROJECT_NAME} opencv_highgui)
else()
    target_link_libraries(${PROJECT_NAME} opencv_imgcodecs opencv_videoio)
endif()



add_executable(vpi_tracker_fast
  src/vpi_feature_tracker_fast.cpp
  src/vpi_feature_tracker_node_fast.cpp
  src/vpi_parameters.cpp
  src/RANSAC_cuda_tools.cu
  )

target_link_libraries(vpi_tracker_fast
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  vpi
  opencv_core
  opencv_imgproc
  ${CUDA_LIBRARIES}
  ${nvidia_to_ros_LIBRARIES})
if(OpenCV_VERSION VERSION_LESS 3)
    target_link_libraries(vpi_tracker_fast opencv_highgui)
else()
    target_link_libraries(vpi_tracker_fast opencv_imgcodecs opencv_videoio)
endif()
