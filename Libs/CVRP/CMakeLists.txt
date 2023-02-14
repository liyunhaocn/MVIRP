cmake_minimum_required (VERSION 3.18)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
#set(LIBRARY_OUTPUT_PATH ../bin)

include_directories(../../Source)

aux_source_directory(. LIB_SRCS)
add_library (cvrp ${LIB_SRCS} ../../Source/Parameters.cpp)