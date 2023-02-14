cmake_minimum_required (VERSION 3.18)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
aux_source_directory(. LIB_SRCS)

add_library(hustGurobi ${LIB_SRCS} ${gurobi_static} ${gurobi_dynamic})