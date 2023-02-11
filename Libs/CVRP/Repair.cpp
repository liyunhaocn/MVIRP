#include "Repair.h"

void hsh::cvrp::Repair::mergeSubtours( Individual & indiv )
{
	//system( "pause" );
	//std::cout << "MERGE INSIDE BEGINS." << std::endl;
	while ( indiv.numDepots >  indiv.numRoutes)
	{
		/* 以 ID 表示对应节点或路径 */

		int subtourV = rand() % indiv.numSubtours; // 随机选择一条子回路
		auto &depotSubtourV = indiv.depots[subtourV + indiv.numRoutes];
		
		double bestDelta = Inf;
		Vec<Move> bestMoves;
		for ( int startSubtourV = depotSubtourV.ID , V = startSubtourV , succV = depotSubtourV.links[1];
			  ;
			  V = succV , succV = indiv.nodes[V].links[1] )
		{
			// 对子回路上每个节点 `V`, 考虑所有路径上每个节点 `W`, 以及两种邻域动作
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
		/* 选择一个最好的合并动作执行 */
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
		// 寻找违反载重或 ( 和 ) 违反服务时长的路径集合
		infeasibleRoutes.clear();
		for ( int route = 0; route < indiv.numRoutes; ++route )
		{
			if ( capacityExcesses[route] > params.error || durationExcesses[route] > params.error ) 
			{ 
				infeasibleRoutes.push_back( route );
			}
		}

		// 没有违反的路径, 退出
		if ( infeasibleRoutes.empty() == true ) { break; }

		// 随机选择一条违反的路径
		int routeV = infeasibleRoutes[rand() % infeasibleRoutes.size()];
		auto &depotRouteV = indiv.depots[routeV];

		// 考察这条路径上所有节点的邻域动作
		bestImprovingMoves.clear();

		for ( int startRouteV = depotRouteV.ID , V = depotRouteV.links[1] , succV = indiv.nodes[V].links[1];
			  ;
			  V = succV , succV = indiv.nodes[V].links[1] )
		{
			if ( V == 0 ) { throw String( "Can not construct neighborhood of a depot." ); }
			
			//std::random_shuffle( typeOrder.begin() , typeOrder.end() ); // 随机打乱邻域动作类型, 但是意义不大, 因为所有的动作都会被考虑

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
		
		/* 循环退出情况判断 */
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
	if ( dtNumRoutes != 0 ) { return false; } //  不考虑改变路径数量的动作

	double dtDistance = deltaDistance( move , indiv );
	double dtCapacityExcess = deltaCapacityExcess( move , indiv );
	double dtDurationExcess = params.isDurationLimit ? deltaDurationExcess( move , dtDistance , indiv ) : 0;
	double dtPenaltyTermValue = params.critical.penaltyCapacityExcess*dtCapacityExcess
		+ params.critical.penaltyDurationExcess*dtDurationExcess;
	
	if ( dtPenaltyTermValue > -params.error ) { return false; } // 不考虑惩罚项增量大于等于 0 的动作
	double penaltyFuncValue = ( indiv.distance + dtDistance )
		+ params.critical.penaltyCapacityExcess*( indiv.capacityExcess + dtCapacityExcess )
		+ params.critical.penaltyDurationExcess*( indiv.durationExcess + dtDurationExcess );

	if ( bestImprovingMoves.empty() || penaltyFuncValue < bestImprobingValue - params.error )
	{
		bestImprovingMoves.clear(); // 遇到比当前最好更好的动作, 清空集合
		bestImprobingValue = penaltyFuncValue;
		bestImprovingMoves.emplace_back( move );
		return false; // 继续构造邻域
	}
	else
	{
		if ( penaltyFuncValue > bestImprobingValue + params.error ) { return false; } // 比当前最好的差, 不考虑
		else 
		{ 
			bestImprovingMoves.emplace_back( move );// 和当前最好的相同, 加入集合, 继续构造邻域
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






