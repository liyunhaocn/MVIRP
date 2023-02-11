#include "Individual.h"

void hsh::cvrp::Individual::copyAllFrom( Individual & src )
{
	// TODO[params不需要自身赋值, 当前MVIRP场景不会有第二个params对象]
	//params = src.params;
	//std::cout << "FIRST" << std::endl;
	for ( int ID = 0; ID < params.numNodes; ++ID ) { nodes[ID] = src.nodes[ID]; }
	for ( int ID = 0; ID < src.numDepots; ++ID ) { depots[ID] = src.depots[ID]; }
	numRoutes = src.numRoutes;
	numSubtours = src.numSubtours;
	numDepots = src.numDepots;

	distance = src.distance;
	capacityExcess = src.capacityExcess;
	durationExcess = src.durationExcess;
	isFeasible = src.isFeasible;

	cycleID = src.cycleID;
	numAccumulation = src.numAccumulation;
	numPrivateEdges = src.numPrivateEdges;
	baseIndivID = src.baseIndivID;
	goalIndivID = src.goalIndivID;
	tGenID = src.tGenID;
}

void hsh::cvrp::Individual::copyLinksFrom( const Individual & src )
{ 
	params = src.params;
	for ( int ID = 1; ID < params.numNodes; ++ID )
	{
		nodes[ID].links[0] = src.nodes[ID].links[0];
		nodes[ID].links[1] = src.nodes[ID].links[1];
	}
}

void hsh::cvrp::Individual::calculateCompletely()
{ 
	distance = getDistance();
	capacityExcess = getCapacityExcess();
	durationExcess = getDurationExcess();
	isFeasible = getFeasibleStatus();
}

double hsh::cvrp::Individual::getDistanceOf( int depotID )
{
	double distanceRoute = 0.;
	for ( int start = depots[depotID].ID , cur = start , succ = depots[depotID].links[1];
		  ;
		  cur = succ , succ = nodes[cur].links[1] )
	{
		distanceRoute += params.distanceMat[cur][succ];
		if ( succ == start ) { break; }
	}
	return distanceRoute;
}

double hsh::cvrp::Individual::getDistance()
{
	double distanceIndiv = 0.;
	for ( int ID = 0; ID < numDepots; ++ID ) { distanceIndiv += getDistanceOf( ID ); }
	return distanceIndiv;
}

double hsh::cvrp::Individual::getLoadOf( int depotID )
{
	double loadRoute = 0.;
	for ( int start = depots[depotID].ID , cur = start , succ = depots[depotID].links[1];
		  ;
		  cur = succ , succ = nodes[cur].links[1] )
	{
		loadRoute += params.nodeInfo[cur].demand;
		if ( succ == start ) { break; }
	}
	return loadRoute;
}

double hsh::cvrp::Individual::getCapacityExcessOf( int depotID )
{
	double loadRoute = getLoadOf( depotID );
    //std::cout << depotID << ": load " << loadRoute << std::endl;
	return getExcessOf( params.capacityLimit , loadRoute );
}

double hsh::cvrp::Individual::getCapacityExcess()
{
	double capacityExcessIndiv = 0.;
	for ( int ID = 0; ID < numDepots; ++ID ) { capacityExcessIndiv += getCapacityExcessOf( ID ); }
	return capacityExcessIndiv;
}

double hsh::cvrp::Individual::getDurationOf( int depotID )
{
	double durationRoute = 0.;
	for ( int start = depots[depotID].ID , cur = start , succ = depots[depotID].links[1];
		  ;
		  cur = succ , succ = nodes[cur].links[1] )
	{
		durationRoute += params.distanceMat[cur][succ] + params.nodeInfo[cur].serviceDuration;
		if ( succ == start ) { break; }
	}
	return durationRoute;
}

double hsh::cvrp::Individual::getDurationExcessOf( int depotID )
{
	double durationRoute = getDurationOf( depotID );
	return getExcessOf( params.durationLimit , durationRoute );
}

double hsh::cvrp::Individual::getDurationExcess()
{
	double durationExcessIndiv = 0.;
	for ( int ID = 0; ID < numDepots; ++ID ) { durationExcess += getDurationExcessOf( ID ); }
	return durationExcessIndiv;
}

void hsh::cvrp::Individual::reset()
{
	numRoutes = 0;
	numSubtours = 0;
	numDepots = 0;
	distance = 0;
	capacityExcess = 0;
	durationExcess = 0.0;
	isFeasible = false;
	cycleID = -1;
	numAccumulation = -1;
	numPrivateEdges = 0;
	baseIndivID = -1;
	goalIndivID = -1;
	tGenID = -1;
}

hsh::cvrp::Node & hsh::cvrp::Individual::addRouteDepot()
{
	numDepots++;
	return depots[numRoutes++];
}

hsh::cvrp::Node & hsh::cvrp::Individual::addSubtDepot( int expectedNumRoutes )
{
	numDepots++;
	return depots[expectedNumRoutes + numSubtours++];
}

void hsh::cvrp::Individual::done( int expectedNumRoutes )
{
	if ( numRoutes != expectedNumRoutes )
	{
		throw String( "Incorrect number of routes: " + numRoutes );
	}
	numDepots = numRoutes + numSubtours;
}

void hsh::cvrp::Individual::delDepot( int deletedDepotID )
{
	for ( int ID = deletedDepotID + 1; ID < numDepots; ++ID )
	{
		depots[ID - 1] = depots[ID];
	}
	--numDepots;
	deletedDepotID < numRoutes ? --numRoutes : --numSubtours;
}

double hsh::cvrp::Individual::getExcessOf( double limit , double value)
{
	if ( value < limit + params.error )
	{
		return 0.;
	}
	else
	{
		return value - limit;
	}
}

bool hsh::cvrp::Individual::getFeasibleStatus()
{
	return  ( capacityExcess < params.error && durationExcess < params.error );
}

bool hsh::cvrp::Individual::inspectLinks( bool showErrorMsg )
{
	using namespace std;
	Vec<bool> visited( params.numNodes , false ); 

	for ( int routeId = 0; routeId < numDepots; ++routeId )
	{
		auto &depot = depots[routeId];
		if ( depot.ID != 0 ) { return false; }
		for ( int startNodeId = depot.ID , curNodeId = depot.links[1] , succNodeId = nodes[curNodeId].links[1];
			  ;
			  curNodeId = succNodeId , succNodeId = nodes[curNodeId].links[1] )
		{
			if ( curNodeId == 0 || visited[curNodeId] ) { return false; }
			else { visited[curNodeId] = true; }
			if ( succNodeId == startNodeId ) { break; }
		}
	}
	if ( showErrorMsg ) { cout << "ALL DEPOT IDS ARE 0, AND THERE IS NO DUPLICATE ONE." << endl; }
	for ( int i = 1; i < params.numNodes; i++ )
	{
		if ( !visited[i] )
		{
			cout << "MISSING NODE ID " << i << endl;
			system("pause"); // TODO[dbg]
			return false;
		}
	}
	if ( showErrorMsg ) { cout << "ALL NODES ARE COVERED, AND THERE IS NO DUPLICATE ONE." << endl; }
	return true;
}
