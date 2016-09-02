cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

add_executable(binocular_driver src/main.cpp)

target_link_libraries(binocular_driver
	-lusb-1.0
	${OpenCV_LIBS}
)