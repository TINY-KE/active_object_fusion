cmake_minimum_required(VERSION 3.10.2)
project(active_eao)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE release)
ENDIF()
SET(CMAKE_BUILD_TYPE Debug)
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#[ERROR]: no matching function for call to ‘_IplImage::_IplImage(cv::Mat&)’
set(CMAKE_CXX_FLAGS -DCV__ENABLE_C_API_CTORS)
# yolox
ADD_DEFINITIONS("-w -O2 -pthread")
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 14)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
# O0 -Wall -g -ggdb  前三个是做什么的？
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}    ")   #-march=native -Wno-return-type -Wall  -O3   -Wno-unused-function  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   ") #-march=native -Wno-return-type -Wall   -O3   -Wno-unused-function
 
# set carv needed flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__STDC_CONSTANT_MACROS -DGL_GLEXT_PROTOTYPES -D_LINUX -D_REENTRANT -DCGAL_USE_F2C -DCGAL_DISABLE_ROUNDING_MATH_CHECK")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__STDC_CONSTANT_MACROS -DGL_GLEXT_PROTOTYPES -D_LINUX -D_REENTRANT -DCGAL_USE_F2C -DCGAL_DISABLE_ROUNDING_MATH_CHECK")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake_modules)

set(OpenCV_DIR "/home/zhjd/thirdparty/opencv3.4.10/opencv-3.4.10/build")
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()
message(STATUS "Using opencv version ${OpenCV_VERSION}")
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# # glog
# include(../cmake_modules/glog.cmake)

# find openmp
find_package(OpenMP)
if (OPENMP_FOUND)
    message("-- OpenMP found")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find CGAL  sudo apt-get install libcgal-dev
set(CMAKE_BUILD_TYPE_BACKUP ${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL REQUIRED COMPONENTS Core)
include( ${CGAL_USE_FILE} )
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_BACKUP})

# Find Boost
find_package(Boost  REQUIRED COMPONENTS filesystem thread date_time iostreams serialization chrono system)
# find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

# Find PCL
set(PCL_DIR "/home/zhjd/Thirdparty/pcl-1.8.0/pcl-pcl-1.8.0/build")
find_package( PCL 1.7 REQUIRED)
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

# BLAS and LAPACK  科学计算包 sudo apt-get install libblas-dev liblapack-dev liblapacke-dev
find_package(LAPACK REQUIRED)
link_directories(${LAPACK_LIBRARY_DIR})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
set(CMAKE_CXX_STANDARD 17)
set(cv_bridge_DIR /home/zhjd/thirdparty/vision_opencv-indigo/cv_bridge/cmake)
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    message_generation
    std_srvs
    cv_bridge
    image_transport
    roslib

    # yolo
    darknet_ros_msgs
)


## Generate services in the 'srv' folder
add_service_files(FILES
  saveOdometry.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    /home/zhjd/ws_active/src/kinect/EAO-Fusion/ros_test/include
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../Thirdparty/DBoW2/DBoW2
    ${PROJECT_SOURCE_DIR}/../Thirdparty/DBoW2/
    ${PROJECT_SOURCE_DIR}/../Thirdparty/g2o/g2o
    ${PROJECT_SOURCE_DIR}/../src/g2oAddition
    ${PROJECT_SOURCE_DIR}/../include/PEAC
    ${PROJECT_SOURCE_DIR}/../src/DepthFilter
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../lib)


add_library(${PROJECT_NAME}_lib SHARED

    ../src/System.cc
    # base
    ../src/System.cc
    ../src/Tracking.cc
    ../src/LocalMapping.cc
    ../src/LoopClosing.cc
    ../src/ORBextractor.cc
    ../src/ORBmatcher.cc
    ../src/FrameDrawer.cc
    ../src/Converter.cc
    ../src/MapPoint.cc
    ../src/KeyFrame.cc
    ../src/Map.cc
    ../src/MapDrawer.cc
    ../src/Optimizer.cc
    ../src/PnPsolver.cc
    ../src/Frame.cc
    ../src/KeyFrameDatabase.cc
    ../src/Sim3Solver.cc
    ../src/Initializer.cc
    ../src/Viewer.cc
    ../src/Object.cc      # NOTE [EAO-SLAM]

    #rviz
    ../src/MapPublisher.cc
    
    # plane
    ../src/MapPlane.cc
    ../src/g2oAddition/EdgePlane.cc
    ../src/g2oAddition/VertexPlane.cc

)


target_link_libraries(${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/../Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/../Thirdparty/g2o/lib/libg2o.so
    boost_system boost_filesystem boost_serialization
    lapack blas

    # dw
    /opt/ros/melodic/lib/libxmlrpcpp.so
)


# my 版本
add_executable(sync_yolomessage src/real_topic_test.cpp)
add_dependencies(sync_yolomessage ${catkin_EXPORTED_TARGETS})
target_link_libraries(sync_yolomessage    ${catkin_LIBRARIES}     ${PROJECT_NAME}_lib )


