cmake_minimum_required (VERSION 3.18)

add_subdirectory(../Libs Libs)
#add_subdirectory(../Libs/CVRP CVRP)
#add_subdirectory(../Libs/Gurobi Gurobi)
#add_subdirectory(../Libs/Delaunator Delaunator)

aux_source_directory(. ALL_SRCS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../Deploy")

add_executable(IRPSolver ${ALL_SRCS})

target_link_libraries(IRPSolver cvrp delaunator hustGurobi ${gurobi_static} ${gurobi_dynamic})
