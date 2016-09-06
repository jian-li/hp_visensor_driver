cmake_minimum_required(VERSION 2.8)

project(maker_binocular)

find_package(OpenCV REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${OpenCV_INCLUDE_DIRS}
)

add_executable(binocular_driver 
	src/main.cpp
	src/maker_binocular.cpp
)

target_link_libraries(binocular_driver
	-lusb-1.0
	${OpenCV_LIBS}
)