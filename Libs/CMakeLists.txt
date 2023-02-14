
cmake_minimum_required (VERSION 3.18)

include_directories(/home/liyunhao/gurobi1001/linux64/include)
find_library(gurobi_static NAMES gurobi_g++5.2 PATHS /home/liyunhao/gurobi1001/linux64/lib)
find_library(gurobi_dynamic NAMES gurobi100 PATHS /home/liyunhao/gurobi1001/linux64/lib)

#add_subdirectory(.)
add_subdirectory(CVRP)
add_subdirectory(Delaunator)
add_subdirectory(Gurobi)
