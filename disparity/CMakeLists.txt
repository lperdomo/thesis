cmake_minimum_required(VERSION 3.5)
project(disparity)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization registration filters features)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(disparity main.cpp
	disparity.cpp disparity.h
        descriptor.cpp descriptor.h
        elas.cpp elas.h
        filter.cpp filter.h
        image.h
        matrix.cpp matrix.h
        timer.h
        triangle.cpp triangle.h)

target_link_libraries(disparity
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})
