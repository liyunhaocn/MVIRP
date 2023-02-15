
cmake_minimum_required (VERSION 3.18)

find_package(GTest REQUIRED)

include_directories(/home/liyunhao/gurobi1001/linux64/include)
find_library(gurobi_static NAMES gurobi_g++4.8 PATHS /home/liyunhao/gurobi1001/linux64/lib)
find_library(gurobi_dynamic NAMES gurobi100 PATHS /home/liyunhao/gurobi1001/linux64/lib)

include_directories(${GTEST_INCLUDE_DIRS})
aux_source_directory(. ALL_SRCS)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../Deploy")

add_executable(IRPTest ${ALL_SRCS})

target_link_libraries(IRPTest ${GTEST_BOTH_LIBRARIES} ${gurobi_static} ${gurobi_dynamic} pthread)

gtest_discover_tests(IRPTest)