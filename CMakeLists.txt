cmake_minimum_required(VERSION 2.8)

project(maker_binocular)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	sensor_msgs
	cv_bridge
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(visensor_driver 
	src/main.cpp
	src/ViSensorDriver.cpp
)

target_link_libraries(visensor_driver
	-lusb-1.0
	-lpthread
	${OpenCV_LIBS}
	${GLOG_LIBRARIES}
	${catkin_LIBRARIES}
)
