#include "Parameters.h"
#include "Solver.h"
#include "Solution.h"

using namespace hsh::mvirp;


int main(int argc, char* argv[]) {
	
    
    try{
        Solver solver;
        solver.run(argc, argv);
	}
    catch (const Str &e) { std::cout << "EXCEPTION | " << e << std::endl; }
    catch (std::exception &e) { std::cout << "Exception occurred: " << e.what() << std::endl; }
}