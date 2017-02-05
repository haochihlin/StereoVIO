cmake_minimum_required(VERSION 2.8.3)
project(stereo_vio)

# Support c++11 compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
  tf_conversions
)

## System dependencies are found with CMake's conventions
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)
FIND_PACKAGE( OpenCV REQUIRED )
## For OpenCV 3.X
#set(OpenCV_DIR "/media/jim/Data_Storage/opencv-3.1.0/release")
#FIND_PACKAGE( OpenCV 3.1 EXACT REQUIRED )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} include)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# VO_test
ADD_EXECUTABLE( VO_test src/VO_test.cpp )
TARGET_LINK_LIBRARIES(VO_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# ESKF_test
ADD_EXECUTABLE( ESKF_test src/ESKF_test.cpp )
TARGET_LINK_LIBRARIES(ESKF_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# VIO_test
ADD_EXECUTABLE( VIO_test src/VIO_test.cpp )
TARGET_LINK_LIBRARIES(VIO_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# Container_Test
ADD_EXECUTABLE( Container_Test src/Container_Test.cpp )
TARGET_LINK_LIBRARIES(Container_Test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )
