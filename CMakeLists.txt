cmake_minimum_required(VERSION 2.8.3)

project(opencv_test)
set(CMAKE_CXX_FLAGS "-stdlib=libc++")
find_package(OpenCV REQUIRED)
include_directories(
	${OpenCV_INCLUDE_DIRS}
)
add_executable(run main.cpp)
target_link_libraries(run
	${OpenCV_LIBRARIES}
)