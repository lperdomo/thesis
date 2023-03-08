cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project ( dat2pcd_batch )
#edit line below to give cmake a hint where to find a PCLConfig.cmake in case of manually built and not installed PCL
#set(PCL_DIR "/path/to/PCLConfig.cmake")
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters)
include_directories(${PCL_INCLUDE_DIRS})
add_executable( dat2pcd_batch dat2pcd_batch.cpp)
target_link_libraries( dat2pcd_batch ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} )
