#include "Repair.h"

void hsh::cvrp::Repair::mergeSubtours( Individual & indiv )
{
	//system( "pause" );
	//std::cout << "MERGE INSIDE BEGINS." << std::endl;
	while ( indiv.numDepots >  indiv.numRoutes)
	{
		/* �� ID ��ʾ��Ӧ�ڵ��·�� */

		int subtourV = rand() % indiv.numSubtours; // ���ѡ��һ���ӻ�·
		auto &depotSubtourV = indiv.depots[subtourV + indiv.numRoutes];
		
		double bestDelta = Inf;
		Vec<Move> bestMoves;
		for ( int startSubtourV = depotSubtourV.ID , V = startSubtourV , succV = depotSubtourV.links[1];
			  ;
			  V = succV , succV = indiv.nodes[V].links[1] )
		{
			// ���ӻ�·��ÿ���ڵ� `V`, ��������·����ÿ���ڵ� `W`, �Լ�����������
			for ( int routeW = 0; routeW < params.critical.numRoutes; ++routeW )
			{
				if ( routeW == depotSubtourV.routeID ) { continue; }
				for ( int startRouteW = 0 , W = startRouteW , succW = indiv.depots[routeW].links[1];
					  ;
					  W = succW , succW = indiv.nodes[W].links[1] )
				{
					Move v_to_w( Move::Type::OPT2STARTYPE1 , V , W , subtourV + indiv.numRoutes , routeW );
					Move v_to_wSucc( Move::Type::OPT2STARTYPE2 , V , W , subtourV + indiv.numRoutes , routeW );

					double delta1 = deltaDistance( v_to_w , indiv );
					double delta2 = deltaDistance( v_to_wSucc , indiv );
					
					if ( abs( delta1 - bestDelta ) > params.error )
					{
						if ( delta1 < bestDelta )
						{
							bestMoves.clear();
							bestMoves.emplace_back( v_to_w );
							bestDelta = delta1;
						}
					}
					else
					{
						bestMoves.emplace_back( v_to_w );
					}

					if ( abs( delta2 - bestDelta ) > params.error )
					{
						if ( delta1 < bestDelta )
						{
							bestMoves.clear();
							bestMoves.emplace_back( v_to_wSucc );
							bestDelta = delta2;
						}
					}
					else
					{
						bestMoves.emplace_back( v_to_wSucc );
					}
					if ( succW == startRouteW ) { break; }
				}
			}
			if ( succV == startSubtourV ) { break; }
		}
		//std::cout << "? " << bestMoves.size() << std::endl;
		/* ѡ��һ����õĺϲ�����ִ�� */
		//Individual backup( params );
		//backup.copyAllFrom( indiv );
		updateLinks(  bestMoves[rand() % bestMoves.size()], indiv);
		//if ( indiv.inspectLinks( true ) == false ) { system( "pause" ); }
	}
}

bool hsh::cvrp::Repair::eliminateExcess( Individual & indiv )
{
	//double timeStart = clock();
	typeOrder = {
		Move::INSKIND1,Move::INSKIND2,
		Move::SWAPKIND1,Move::SWAPKIND2,
		Move::INSKIND3, Move::INSKIND4,
		Move::OPT2KIND1,Move::OPT2KIND2,Move::OPT2KIND3,Move::OPT2KIND4
	};
	int rounds = 0;
	prepareAdditionalInfo( indiv );
	do
	{
		rounds++;
		// Ѱ��Υ�����ػ� ( �� ) Υ������ʱ����·������
		infeasibleRoutes.clear();
		for ( int route = 0; route < indiv.numRoutes; ++route )
		{
			if ( capacityExcesses[route] > params.error || durationExcesses[route] > params.error ) 
			{ 
				infeasibleRoutes.push_back( route );
			}
		}

		// û��Υ����·��, �˳�
		if ( infeasibleRoutes.empty() == true ) { break; }

		// ���ѡ��һ��Υ����·��
		int routeV = infeasibleRoutes[rand() % infeasibleRoutes.size()];
		auto &depotRouteV = indiv.depots[routeV];

		// ��������·�������нڵ��������
		bestImprovingMoves.clear();

		for ( int startRouteV = depotRouteV.ID , V = depotRouteV.links[1] , succV = indiv.nodes[V].links[1];
			  ;
			  V = succV , succV = indiv.nodes[V].links[1] )
		{
			if ( V == 0 ) { throw String( "Can not construct neighborhood of a depot." ); }
			
			//std::random_shuffle( typeOrder.begin() , typeOrder.end() ); // �����������������, �������岻��, ��Ϊ���еĶ������ᱻ����

			switch ( proximityMode )
			{
			case ProximityMode::FULL:
				{
                    rangeProximityDepot = params.numClients - 1;
                    rangeProximity = std::min(10, params.numClients - 1);
					break;
				}
			case ProximityMode::GRANULAR:
				{
					rangeProximityDepot = std::min(10, params.numClients - 1);
					rangeProximity = std::min(10, params.numClients - 1);
					break;
				}
			}
			//std::cout << rangeProximity << ", " << rangeProximityDepot << std::endl;
			//std::cout << params.numClients << ", " << params.numClients << std::endl;
			if ( indiv.nodes[V].links[0] == 0 || indiv.nodes[V].links[1] == 0 
				 || indiv.nodes[indiv.nodes[V].links[0]].links[0] == 0 || indiv.nodes[indiv.nodes[V].links[1]].links[1] == 0 )
			{
				constructNeighborhoodOf( V , rangeProximityDepot , typeOrder , indiv );
			}
			else
			{
				constructNeighborhoodOf( V , rangeProximity , typeOrder , indiv ); 
			}
			//system( "pause" );
			if ( succV == startRouteV ) { break; }
		}
		
		/* ѭ���˳�����ж� */
		if ( bestImprovingMoves.empty() )
		{
			break;
		}
		else
		{
			//std::cout << "best improving moves size " << bestImprovingMoves.size() << std::endl;
			//std::cout << "perform" << std::endl;
			performMove( bestImprovingMoves[rand() % bestImprovingMoves.size()] , indiv );
			//std::cout << "perform ends" << std::endl;
		}
		//std::cout << "INFEASIBLE ROUTES " << infeasibleRoutes.size() << ", ROUNDS " << rounds << std::endl;
	} while ( infeasibleRoutes.empty() == false );

	//std::cout << "INFEASIBLE ROUTES " << infeasibleRoutes.size() << ", ROUNDS " << rounds << std::endl;
	//std::cout << "ELIMINATE EXCESS TAKES " << ( clock() - timeStart ) / CLOCKS_PER_SEC << std::endl;
	return indiv.isFeasible;
}

bool hsh::cvrp::Repair::evaluateMove( Move & move , Individual & indiv )
{
	int dtNumRoutes = deltaNumRoutes( move , indiv );
	if ( dtNumRoutes != 0 ) { return false; } //  �����Ǹı�·�������Ķ���

	double dtDistance = deltaDistance( move , indiv );
	double dtCapacityExcess = deltaCapacityExcess( move , indiv );
	double dtDurationExcess = params.isDurationLimit ? deltaDurationExcess( move , dtDistance , indiv ) : 0;
	double dtPenaltyTermValue = params.critical.penaltyCapacityExcess*dtCapacityExcess
		+ params.critical.penaltyDurationExcess*dtDurationExcess;
	
	if ( dtPenaltyTermValue > -params.error ) { return false; } // �����ǳͷ����������ڵ��� 0 �Ķ���
	double penaltyFuncValue = ( indiv.distance + dtDistance )
		+ params.critical.penaltyCapacityExcess*( indiv.capacityExcess + dtCapacityExcess )
		+ params.critical.penaltyDurationExcess*( indiv.durationExcess + dtDurationExcess );

	if ( bestImprovingMoves.empty() || penaltyFuncValue < bestImprobingValue - params.error )
	{
		bestImprovingMoves.clear(); // �����ȵ�ǰ��ø��õĶ���, ��ռ���
		bestImprobingValue = penaltyFuncValue;
		bestImprovingMoves.emplace_back( move );
		return false; // ������������
	}
	else
	{
		if ( penaltyFuncValue > bestImprobingValue + params.error ) { return false; } // �ȵ�ǰ��õĲ�, ������
		else 
		{ 
			bestImprovingMoves.emplace_back( move );// �͵�ǰ��õ���ͬ, ���뼯��, ������������
			return false;
		} 
	}

}

int hsh::cvrp::Repair::extendRangeWithCurrentProximity( int V , double looseFactor , int minRangeProximity , int maxRangeProximity , Individual &indiv )
{
	if ( indiv.nodes[V].links[0] == 0 || indiv.nodes[V].links[1] == 0 ) { return maxRangeProximity; }
	auto average = [ ] ( int a , int b )->int { return ( a + b ) / 2; };
	int prevProximityV = nodeAdditionalInfo[V].prevProximity;
	int succProximityV = nodeAdditionalInfo[V].succProximity;
	int	curAverProximity = average( prevProximityV , succProximityV );
	//std::cout << nodeAdditionalInfo[V].prevProximity << ", "<< nodeAdditionalInfo[V].succProximity << std::endl;
	//std::cout << "V " << V << std::endl;
	double radius = looseFactor * params.proximityMat[V][curAverProximity].distance;
	//system( "pause" );
	for ( int k = 0; k < params.numNodes; ++k )
	{
		if ( params.distanceMat[V][k] > radius + params.error )
		{
			return std::min( minRangeProximity , std::max( maxRangeProximity , k ) );
		}
	}

	return maxRangeProximity;
}

hsh::cvrp::Repair & hsh::cvrp::Repair::setProximityMode( ProximityMode proximityMode )
{
	this->proximityMode = proximityMode;
	return *this;
}






