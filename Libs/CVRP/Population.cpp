#include "Population.h"

void hsh::cvrp::Population::initPopulation()
{ 
	/* ������Ⱥ */
	indivs = Vec<Individual>( params.population.size , Individual( params ) ); // Ԥ�ڵ���Ⱥ��С
	int failedCnt = 0, maxFailedCnt = 200;
	int indivID = 0;
	for (; indivID < params.population.size; ++indivID )
	{
		indivs[indivID].reset();
		if ( generateInitialIndiv( indivs[indivID] , nagataLS , repair ) != true ) {
			--indivID;
			++failedCnt;
			if (failedCnt == maxFailedCnt) {  // ����ʧ�ܼ�������, ֻ�������и�������
				for (int i = 0; i < params.population.size; ++i) {
					// ע��ʧ���в����кͿ��е���·����������Ҫ������
					if (!indivs[i].isFeasible || indivs[i].numRoutes != params.critical.numRoutes) { // ��Ⱥ�ض�Ҫ�����·����
						if (params.dbg.lk)std::cout << i << " fea stat: " << indivs[i].isFeasible << ", num rou: " << indivs[i].numRoutes << std::endl;
						params.population.size = i;
						break;
					}
				}
				//if (params.dbg.lk)std::cout << "POPULATION READY: " << params.population.size << std::endl;
				break; // ����������߼�����ִ��, ������return
			}
		}
        else { /*std::cout << ".";*/ }
	}
	/*for ( int id = 0; id < indivID; ++id ) 
	{
		std::cout << "[" << id << ": " << indivs[id].numDepots << ": " << indivs[id].isFeasible << "]"
			<< indivs[id].distance << " ";
	}std::cout << std::endl;*/

	/* ����洢�ռ�; Ԥ��һ��λ��, ���Դ���Ӵ�������Ϣ */
	numPrivateEdges = Vec<Vec<int>>(params.population.size + 1, Vec<int>(params.population.size + 1, -1));
	minPrivateEdges = Vec<int>(params.population.size + 1, -1);
	backup = Vec<int>(params.population.size + 1, -1);
	maxPrivateEdges = Vec<int>(params.population.size + 1, -1);
	backup2 = Vec<int>(params.population.size + 1, -1);
	numCycles = Vec<Vec<int>>(params.population.size + 1, Vec<int>(params.population.size + 1, -1));
	minCycles = Vec<int>(params.population.size + 1, -1);
	backup3 = Vec<int>(params.population.size + 1, -1);
	goodness = Vec<double>(params.population.size + 1, -1);

	/* ��ʼ����Ⱥ������Ϣ */
	for ( int indivID = 0; indivID < params.population.size; ++indivID )
	{
		for ( int otherIndivID = indivID + 1; otherIndivID < params.population.size; ++otherIndivID )
		{
			eax.prepareDataStructures();
			eax.classifyEdges( indivs[indivID] , indivs[otherIndivID] );
			numPrivateEdges[indivID][otherIndivID] = numPrivateEdges[otherIndivID][indivID] = eax.privateEdgesMap.size();
			if ( params.population.useMinCycles )
			{
				eax.generateCycles( params.population.timesGenerateCycles );
				numCycles[indivID][otherIndivID] = numCycles[otherIndivID][indivID] = eax.cycles.size();
			}
		}
		int minGap = INT_MAX , maxGap = -1, minCycs = INT_MAX;
		for ( int otherIndivID = 0; otherIndivID < params.population.size; ++otherIndivID )
		{
			if ( otherIndivID == indivID ) { continue; }
			minGap = std::min( minGap , numPrivateEdges[indivID][otherIndivID] );
			maxGap = std::max( maxGap , numPrivateEdges[indivID][otherIndivID] );
			minCycs = std::min( minCycs , numCycles[indivID][otherIndivID] );
		}
		minPrivateEdges[indivID] = minGap;
		maxPrivateEdges[indivID] = maxGap;
		minCycles[indivID] = minCycs;
	}
}

bool hsh::cvrp::Population::generateInitialIndiv( Individual &indiv , NagataLocalSearch & nagataLS , Repair & repair )
{
	/* ÿ���ͻ�һ��·�� */
	for ( int routeID = 0; routeID < params.numClients; ++routeID )
	{
		Node &client = indiv.nodes[routeID + 1];
		Node &depot = indiv.addRouteDepot();
		
		client.ID = routeID + 1;
		client.routeID = routeID;
		client.links[0] = client.links[1] = 0;

		depot.ID = 0;
		depot.routeID = routeID;
		depot.links[0] = depot.links[1] = client.ID;
	}
	//std::cout << "LINKS SHOULD BE RIGHT." << std::endl;
	//indiv.inspectLinks( true );

	/* �Ż�·���� */
	indiv.calculateCompletely();
	nagataLS.setSearchMode( NagataLocalSearch::SearchMode::PENAL ).run( indiv );
	//std::cout << "PENAL NAGATA LS SHOULD BE RIGHT." << std::endl;
	if (indiv.numRoutes > params.critical.maxNumRoutes ||
		(params.critical.numRoutes != -1 && indiv.numRoutes != params.critical.numRoutes)) {
        //std::cout << " " << indiv.numRoutes << " ";
		return false;
	}
	//std::cout << "NUMBER OF ROUTES SHOULD BE RIGHT." << std::endl;

	/* ����Υ���� */
	if ( indiv.isFeasible != true )
	{
		repair.setProximityMode( Repair::ProximityMode::FULL ).eliminateExcess( indiv );
		if ( indiv.isFeasible != true ) {
			return false;
		}
	}
	//std::cout << "THERE IS NO EXCESS ." << std::endl;

	/* ��һ���Ż� */
    //std::cout << "capa excess " << indiv.getCapacityExcess() << std::endl;
	nagataLS.setSearchMode( NagataLocalSearch::SearchMode::FEASIBLE ).run( indiv );
	//if (!indiv.inspectLinks(true)) { // TODO[dbg]
	//	int k = 100;
	//}
	//std::cout << "FINAL GENERATE DISTANCE " << indiv.distance << ", FEASIBLE" << indiv.isFeasible << std::endl;
	return true;
}

void hsh::cvrp::Population::updatePopulation( Individual & offspIndiv, int baseIndivID, int goalIndivID )
{ 
	//if (params.dbg.lk)std::cout << "upd pop "  << std::endl;
	/* ������С˽�б�����Ϣ */
	for ( int indivID = 0; indivID < params.population.size; ++indivID )
	{
		backup[indivID] = minPrivateEdges[indivID];
		backup2[indivID] = maxPrivateEdges[indivID];
		backup3[indivID] = minCycles[indivID];
	}
	//if (params.dbg.lk)std::cout << "upd pop1 " << std::endl;
	/* �Ӵ�������Ϣ */
	int offspIndivID = params.population.size; // ����Ԥ����λ��
	int minGap = INT_MAX, maxGap = -1, minCycs = INT_MAX;
	for ( int otherIndivID = 0; otherIndivID < params.population.size; ++otherIndivID )
	{
		//if (params.dbg.lk)std::cout << "upd pop2.1 " << std::endl;
		eax.prepareDataStructures();
		//if (params.dbg.lk)std::cout << "upd pop2.2 " << std::endl;
		indivs[otherIndivID];
		//if (params.dbg.lk)std::cout << "upd pop2.3 " << std::endl;
		eax.classifyEdges( offspIndiv , indivs[otherIndivID] );
		//if (params.dbg.lk)std::cout << "upd pop2 " << std::endl;
		numPrivateEdges[offspIndivID][otherIndivID] = numPrivateEdges[otherIndivID][offspIndivID] = eax.privateEdgesMap.size();
		minGap = std::min( minGap , numPrivateEdges[offspIndivID][otherIndivID] );
		maxGap = std::max( maxGap , numPrivateEdges[offspIndivID][otherIndivID] );
		minPrivateEdges[otherIndivID] = std::min( numPrivateEdges[otherIndivID][offspIndivID] , minPrivateEdges[otherIndivID] );
		maxPrivateEdges[otherIndivID] = std::max( numPrivateEdges[otherIndivID][offspIndivID] , maxPrivateEdges[otherIndivID] );
		if ( params.population.useMinCycles )
		{
			//if (params.dbg.lk)std::cout << "upd pop3 " << std::endl;
			eax.generateCycles( params.population.timesGenerateCycles );
			//if (params.dbg.lk)std::cout << "upd pop4 " << std::endl;
			numCycles[offspIndivID][otherIndivID] = numCycles[otherIndivID][offspIndivID] = eax.cycles.size();
			minCycs = std::min( minCycs , numCycles[offspIndivID][otherIndivID] );
			minCycles[otherIndivID] = std::min( numCycles[otherIndivID][offspIndivID] , minCycles[otherIndivID] );
		}
	}
	minPrivateEdges[offspIndivID] = minGap;
	maxPrivateEdges[offspIndivID] = maxGap;
	minCycles[offspIndivID] = minCycs;
	//if (params.dbg.lk)std::cout << "upd pop5 " << std::endl;
	/* ��������Ŷȷ��� */
	double minDistance = Inf , maxDistance = -1;
	for ( int indivID = 0; indivID < params.population.size; ++indivID )
	{
		if ( indivs[indivID].distance < minDistance ) { minDistance = indivs[indivID].distance; }
		if ( indivs[indivID].distance > maxDistance ) { maxDistance = indivs[indivID].distance; }
	}
	minDistance = offspIndiv.distance < minDistance ? offspIndiv.distance : minDistance;
	maxDistance = offspIndiv.distance > maxDistance ? offspIndiv.distance : maxDistance;
	double minMinPrivateEdges = (double)( *( std::min_element( minPrivateEdges.begin() , minPrivateEdges.end() ) ) );
	double maxMinPrivateEdges = (double)( *( std::max_element( minPrivateEdges.begin() , minPrivateEdges.end() ) ) );
	double minMaxPrivateEdges = (double)( *( std::min_element( maxPrivateEdges.begin() , maxPrivateEdges.end() ) ) );
	double maxMaxPrivateEdges = (double)( *( std::max_element( maxPrivateEdges.begin() , maxPrivateEdges.end() ) ) );
	int minMinCycles = ( *std::min_element( minCycles.begin() , minCycles.end() ) );
	int maxMinCylces = ( *std::max_element( minCycles.begin() , minCycles.end() ) );
	
	auto normalizedDistance = [ & ] ( double val , double minVal , double maxVal )->double { return ( val - minVal ) / ( maxVal - minVal + 1.0 ); };
	auto normalizedMinPrivEdges = [ & ] ( double val , double minVal , double maxVal )->double { return ( maxVal - val ) / ( maxVal - minVal + 1.0 ); };
	//if (params.dbg.lk)std::cout << "upd pop6 " << std::endl;
	double factor = params.population.balanceFactor; // Ȩ���������������������
	for ( int indivID = 0; indivID < params.population.size; ++indivID )
	{
		goodness[indivID] = factor * normalizedDistance( indivs[indivID].distance , minDistance , maxDistance )
			+ ( 1.0 - factor ) * normalizedMinPrivEdges( minCycles[indivID] , minMinCycles , maxMinCylces );
	}
	goodness[offspIndivID] = factor * normalizedDistance( offspIndiv.distance , minDistance , maxDistance )
		+ ( 1.0 - factor ) * normalizedMinPrivEdges( minCycles[offspIndivID] , minMinCycles , maxMinCylces );
	
	/* �����Ŷȷ���, ������Ⱥ */
	if ( baseIndivID == goalIndivID && baseIndivID != -1 )
	{ 
		// �滻ָ������
		replaceIndividual( offspIndiv , baseIndivID );
	}
	else
	{ 
		// �滻�Ŷȷ����ϲ����
		Vec<int> orderByGoodness;
		for ( int indivID = 0; indivID <= params.population.size; ++indivID ) { orderByGoodness.push_back( indivID ); }
		random_shuffle( orderByGoodness.begin() , orderByGoodness.end() );
		sort( orderByGoodness.begin() , orderByGoodness.end() , [ & ] ( int indivID1 , int indivID2 )->bool
			  {
				  if ( goodness[indivID1] < goodness[indivID2] - params.error )
				  {
					  return true;
				  }
				  else { return false; }
			  } );
		int worstIndivID = orderByGoodness.back() , secondWorstIndivID = orderByGoodness[orderByGoodness.size() - 2];

		if ( worstIndivID != offspIndivID )
		{
			replaceIndividual( offspIndiv , worstIndivID );
		}
		else
		{
			if ( ( rand() % 100 ) < params.population.replaceSecondWorst )
			{
				replaceIndividual( offspIndiv , secondWorstIndivID );
			}
			else
			{
				std::swap( backup , minPrivateEdges );
				std::swap( backup2 , maxPrivateEdges );
				std::swap( backup3 , minCycles );
			}
		}
	}
	//if (params.dbg.lk)std::cout << "upd pop done. " << std::endl;
}

void hsh::cvrp::Population::replaceIndividual( Individual & offspIndiv , int replacedIndivID )
{ 
	/* �滻���� */
	indivs[replacedIndivID].copyAllFrom( offspIndiv ); // std::swap( offspIndiv , indivs[replacedIndivID] );

	/* ������Ⱥ������Ϣ */
	for ( int otherIndivID = 0; otherIndivID < params.population.size; ++otherIndivID )
	{
		if ( otherIndivID == replacedIndivID ) { numPrivateEdges[replacedIndivID][otherIndivID] = -1; }
		else
		{
			numPrivateEdges[replacedIndivID][otherIndivID] = ( numPrivateEdges.back() )[otherIndivID];
			numPrivateEdges[otherIndivID][replacedIndivID] = ( numPrivateEdges.back() )[otherIndivID];
			numCycles[replacedIndivID][otherIndivID] = ( numCycles.back() )[otherIndivID];
			numCycles[otherIndivID][replacedIndivID] = ( numCycles.back() )[otherIndivID];
		}
	}
	
	for ( int indivID = 0; indivID < params.population.size; ++indivID )
	{
		int minGap = INT_MAX , maxGap = -1 , minCycs = INT_MAX;
		for ( int otherIndivID = 0; otherIndivID < params.population.size; ++otherIndivID )
		{
			if ( indivID == otherIndivID ) { continue; }
			else
			{
				if ( numPrivateEdges[indivID][otherIndivID] < minGap ) { minGap = numPrivateEdges[indivID][otherIndivID]; }
				if ( numPrivateEdges[indivID][otherIndivID] > maxGap ) { maxGap = numPrivateEdges[indivID][otherIndivID]; }
				if ( numCycles[indivID][otherIndivID] < minCycs ) { minCycs = numCycles[indivID][otherIndivID]; }
			}
		}
		//std::cout << replacedIndivID << std::endl;
		minPrivateEdges[indivID] = minGap;
		maxPrivateEdges[indivID] = maxGap;
		minCycles[indivID] = minCycs;
	}
}

hsh::cvrp::Population::Population( Parameters &params , Eax &eax , NagataLocalSearch &nagataLS , Repair &repair ) 
	:params( params ) , eax( eax ) , nagataLS( nagataLS ) , repair( repair )
{ }
