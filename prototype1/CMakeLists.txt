cmake_minimum_required(VERSION 3.9)
project(prototype1)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization features filters segmentation)
find_package(OpenCV REQUIRED)

include_directories("peac")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(prototype1 pipeline.cc pipeline.h
        io.cc io.h
        segmentation.cc segmentation.h
        feature.cc feature.h
        ahc.cc ahc.h
        omp.cc omp.h
        ./peac/AHCParamSet.hpp
        ./peac/AHCPlaneFitter.hpp
        ./peac/AHCPlaneSeg.hpp
        ./peac/AHCTypes.hpp
        ./peac/AHCUtils.hpp
        ./peac/DisjointSet.hpp
        ./peac/eig33sym.hpp)

target_link_libraries(prototype1
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${PCL_VISUALIZATION_LIBRARIES}
        ${PCL_FILTERS_LIBRARIES}
        ${PCL_FEATURES_LIBRARIES}
        ${PCL_SEGMENTATION_LIBRARIES}
        ${OpenCV_LIBS})
