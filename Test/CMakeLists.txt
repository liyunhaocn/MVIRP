cmake_minimum_required (VERSION 3.18)
project (MPIRPTest)

include_directories(/home/liyunhao/gurobi1001/linux64/include)
find_library(gurobi_static NAMES gurobi_c++ PATHS /home/liyunhao/gurobi1001/linux64/lib)
find_library(gurobi_dynamic NAMES gurobi100 PATHS /home/liyunhao/gurobi1001/linux64/lib)

include_directories(../Libs)

add_subdirectory(Source)