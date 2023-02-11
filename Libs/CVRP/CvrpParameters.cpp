#include "CvrpParameters.h"

void hsh::cvrp::Parameters::print()
{
	using namespace std;
	cout << "NODE INFO" << endl;
	for ( auto &n : nodeInfo ) {
		cout << n.ID << " " << n.demand << " " << n.coordX << " " << n.coordY << " " << n.serviceDuration << endl;
	}
	cout << "END" << endl;
}
