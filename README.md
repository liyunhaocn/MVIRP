# TSMHA: An Open-source Matheuristic-based Solver for the Multi-vehicle Inventory Routing Problem


TSMHA, an open-source C++ implementation of a three-stage matheuristic algorithm that combines metaheuristics with mixed integer programming formulations for solving the MIRP [^1]. Tested on 1,038 commonly used benchmark instances, our TSMHA solver took the third place in the 12th DIMACS implementation challenge [^2]. In addition, our proposed TSMHA algorithm includes a solution repair procedure via solving the capacitated vehicle routing problem (CVRP), which can be  conveniently used as an independent CVRP solver. 



[^1]:https://github.com/liyunhaocn/MVIRP
[^2]:http://dimacs.rutgers.edu/programs/challenge/vrp/results/





Once the solver is installed, you can run it in the command line. It is clear that the three required parameters simply give the directory or path to the input, output and configuration files, respectively. If the format of the command line arguments is incorrect, the solver will print the help information as below:

```shell
Usage: ./Solver instance output parameters 

Positionals:
instance    string  required    the instance file path			
output      string  required    the solution saved directory
parameters  string  required    the parameters file path

Options:
-h,--help                     print this help message and exit
```





The solver is built with `CMake`. `CMake 3.18` or a newer version is required.
After installing `CMake`, the commands shown as below.
are used to generate the `TSMHA` solver. Finally, the solver will be saved in the directory `Deploy`.

```shell
mkdir Deploy
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```



In order to verify the reliability of the solver, we have embedded unit tests into the solver.
To be specific, we used open-source Google Test framework [^3]. The Test project is also built by `CMake`. We tested the optimization procedure for each stage individually. The code of the test cases are saved in directory `Test/Source`. Google Test shall be installed on your computer before building the test project. Then running commands below will install the `Test` project.

```shell
cd Test
mkdir Deploy
mkdir build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```



[^3]:https://github.com/google/googletest

