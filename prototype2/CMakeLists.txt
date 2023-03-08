cmake_minimum_required(VERSION 3.5)
project(prototype2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O2")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization registration filters features)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(prototype2 main.cc
	pipeline.cc pipeline.h
	util.cc util.h
	config.cc config.h
	normal.cc normal.h
        descriptor.cc descriptor.h
	matcher.h matcher.cc
        dataset.cc dataset.h
	pch.cc pch.h
    output.cc output.h
	distances_widget.cc distances_widget.h
	trajectory_widget.cc trajectory_widget.h
	m2dp.cc m2dp.h
	shot.cc shot.h
	scene.cc scene.h)
qt5_use_modules(prototype2 Widgets OpenGL)

target_link_libraries(prototype2
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
		${QT_LIBRARIES})
