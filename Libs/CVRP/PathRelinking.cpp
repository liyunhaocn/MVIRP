#include "PathRelinking.h"

void hsh::cvrp::PathRelinking::resetRawIndivs()
{
	sizeRawIndivs = 0;
}

void hsh::cvrp::PathRelinking::resetPickedRawIndivs()
{
	sizePickedRawIndivs = 0;
}

void hsh::cvrp::PathRelinking::resetTempRawIndivs()
{
	sizeTempRawIndivs = 0;
}

hsh::cvrp::Individual & hsh::cvrp::PathRelinking::addRawIndiv()
{
	rawIndivs[sizeRawIndivs].reset();
	return rawIndivs[sizeRawIndivs++];
}

hsh::cvrp::Individual & hsh::cvrp::PathRelinking::addTempIndiv()
{
	tempRawIndivs[sizeTempRawIndivs].reset();
	return tempRawIndivs[sizeTempRawIndivs++];
}

hsh::cvrp::Individual & hsh::cvrp::PathRelinking::addPickedRawIndiv()
{
	pickedRawIndivs[sizePickedRawIndivs].reset();
	return pickedRawIndivs[sizePickedRawIndivs++];
}

void hsh::cvrp::PathRelinking::core( Individual & base , int baseIndivID , Individual & goal , int goalIndivID )
{
	resetRawIndivs(); // 清空上次的状态
	eax.prepareDataStructures();
	eax.classifyEdges( base , goal );
	if ( eax.privateEdgesMap.size() == 0 ) { return; }
	eax.generateCycles( params.pathRelinking.generateCycleTimes );
	
	Individual *lastRawIndiv = &base;
	lastRawIndiv->numAccumulation = 0;

	Vec<int> used( eax.cycles.size() , -1 ); // ` used[cycleID] ` 表示使用 ` cycleID ` 对应 AB-Cycle 的原始个体 ID
	//std::cout << "cycle size " << eax.cycles.size() << std::endl;
	for (int rawIndivID = 0; rawIndivID < used.size(); ++rawIndivID) // 一共有 AB-Cycle 集合大小数量的原始个体
	{
		/* 试探所有未使用的 AB-Cycle, 应用到已确定的最后一个原始个体上 */
		resetTempRawIndivs();
		for (int cycleID = 0; cycleID < used.size(); ++cycleID)
		{
			if (used[cycleID] != -1) { continue; }

			Individual& tempIndiv = addTempIndiv();
			tempIndiv.copyLinksFrom(*lastRawIndiv);
			eax.applyCycle(eax.cycles[cycleID], tempIndiv);
		}

		/* 从试探所产生的临时原始个体中确定一个原始个体 */
		Individual* determinedRawIndiv = nullptr;

		Vec<int> order(sizeTempRawIndivs);
		for (int index = 0; index < order.size(); ++index) { order[index] = index; }
		auto sortingRule = [&](int tempIndivID1, int tempIndivID2) -> bool
		{
			// 子回路最少优先
			if (tempRawIndivs[tempIndivID1].numSubtours < tempRawIndivs[tempIndivID2].numSubtours) { return true; }
			else
			{
				if (tempRawIndivs[tempIndivID1].numSubtours > tempRawIndivs[tempIndivID2].numSubtours) { return false; }
				else
				{
					// 容量违反和服务时长违反加权最少优先
					double balanceFactor = 0.5; // 容量违反和服务时长违反权衡因子
					double penaltyTermVal1 = balanceFactor * tempRawIndivs[tempIndivID1].capacityExcess
						+ (1 - balanceFactor) * tempRawIndivs[tempIndivID1].durationExcess;
					double penaltyTermVal2 = balanceFactor * tempRawIndivs[tempIndivID2].capacityExcess
						+ (1 - balanceFactor) * tempRawIndivs[tempIndivID2].durationExcess;
					if (penaltyTermVal1 < penaltyTermVal2 - params.error) { return true; }
					else
					{
						if (penaltyTermVal1 > penaltyTermVal2 + params.error) { return false; }
						else
						{
							// 个体距离最短优先
							if (tempRawIndivs[tempIndivID1].distance < tempRawIndivs[tempIndivID2].distance - params.error) { return true; }
							else { return false; } // 个体距离大于或等于
						}
					}
				}
			}
		};
		std::sort(order.begin(), order.end(), sortingRule);

		int M = std::min((rand() % 3) + 3, sizeTempRawIndivs);
		int minSubtours = tempRawIndivs[order[0]].numSubtours;
		int totalWeights = 0;
		while (tempRawIndivs[order[M - 1]].numSubtours != minSubtours) { M--; }

		constexpr int weights[6] = { 60,30,20,15,12,10 }; // { 60,30,20,15,12,10 };{ 2520,1260,840,630,504,420,360,315,280,252 }
		for (int m = 0; m < M; m++) { totalWeights += weights[m]; }
		int pickedNum = rand() % totalWeights;
		for (int m = 0; pickedNum >= 0; pickedNum -= weights[m], m++) { determinedRawIndiv = &tempRawIndivs[order[m]]; }

		if (determinedRawIndiv != nullptr)
		{
			used[determinedRawIndiv->cycleID] = rawIndivID;
			Individual& rawIndiv = addRawIndiv();
			rawIndiv.copyAllFrom(*determinedRawIndiv);// std::swap( *determinedRawIndiv , rawIndiv );
			rawIndiv.numAccumulation = lastRawIndiv->numAccumulation + eax.cycles[rawIndiv.cycleID].size();
			rawIndiv.numPrivateEdges = eax.privateEdgesMap.size();
			rawIndiv.baseIndivID = baseIndivID;
			rawIndiv.goalIndivID = goalIndivID;
			/*std::cout << "new gen rawIndiv num routes " << rawIndiv.numRoutes
                << ", baseid " << rawIndiv.baseIndivID << ", goalid " << rawIndiv.goalIndivID << ", num routes " << base.numRoutes << std::endl;*/
			lastRawIndiv = &rawIndiv;
		}
	}
}

const hsh::cvrp::Vec<int>& hsh::cvrp::PathRelinking::run( Population &pop )
{
	const int times = params.pathRelinking.sizeGoals;

	resetPickedRawIndivs();
	//std::cout << "init sizePickedRawIndivs" << sizePickedRawIndivs << std::endl;
	/* 种群个体按距离从小到大排序 */
	if (params.population.size != orderPop.size())
	{
		orderPop.resize(params.population.size);
		for ( int index = 0; index < orderPop.size(); ++index ) { orderPop[index] = index; }
	}
	std::sort( orderPop.begin() , orderPop.end() , [ & ] ( int indivID1 , int indviID2 ) { return pop.indivs[indivID1].distance < pop.indivs[indviID2].distance; } );
    //std::cout << "Pathrelinking..." << std::endl;
	/* 选择一个 base, 以及指定个数 goal, 进行路径重连 */
	int baseIndivIndex = rand() % orderPop.size();
	int baseIndivID = orderPop[baseIndivIndex];
	for ( int t = 0; t < times; ++t )
	{
		auto pickGoalIndiv = [ & ] ()
		{
			int goalIndivIndex = -1;
			if ( ( rand() % 100 ) < ( double( baseIndivIndex ) / ( orderPop.size() - 1 ) * 100 * params.pathRelinking.goalControlFactor ) 
				 || baseIndivIndex == orderPop.size() - 1 )
			{
				goalIndivIndex = rand() % baseIndivIndex;
			}
			else { goalIndivIndex = ( rand() % ( orderPop.size() - baseIndivIndex - 1 ) ) + baseIndivIndex + 1; }
			return orderPop[goalIndivIndex];
		};
		int goalIndivID = pickGoalIndiv();
		Individual &base = pop.indivs[baseIndivID] , &goal = pop.indivs[goalIndivID];
        //std::cout << "t: " << t << "Base ID: " << baseIndivID << ", Goal ID: " << goalIndivID << std::endl;
		core( base , baseIndivID , goal , goalIndivID );
		int begin = 0 , end = sizeRawIndivs;
		//std::cout << "sizeRawIndivs" << sizeRawIndivs << std::endl;
		if ( sizeRawIndivs > 4 )
		{
			begin = round( sizeRawIndivs*params.pathRelinking.beginFactor );
			end = round( sizeRawIndivs*params.pathRelinking.endFactor );
		}
        //std::cout << "begin: " << begin << ", end: " << end << std::endl;
        for (int rawIndivIndex = begin; rawIndivIndex <= end && rawIndivIndex < sizeRawIndivs - 1; ++rawIndivIndex)
		{
            //std::cout << "rawIndivIndex, :" << rawIndivs[rawIndivIndex].baseIndivID << ", " << rawIndivs[rawIndivIndex].goalIndivID << std::endl;
			Individual &pickedRawIndiv = addPickedRawIndiv();
			pickedRawIndiv.copyAllFrom( rawIndivs[rawIndivIndex] );
            //std::cout << "sizePickedRawIndivs" << sizePickedRawIndivs << std::endl;
		}
	}

	/* 对所有选出的原始个体, 进行一次排序 */
	orderPickedRawIndivs.clear();
	for (int pickedIndivID = 0; pickedIndivID < sizePickedRawIndivs; ++pickedIndivID)
	{
		orderPickedRawIndivs.push_back( pickedIndivID );
	}
    std::sort(orderPickedRawIndivs.begin(), orderPickedRawIndivs.end(), [&] (int id1, int id2) {
        auto &i1 = pickedRawIndivs[id1];
        auto &i2 = pickedRawIndivs[id2];
        return i1.distance + params.critical.penaltyCapacityExcess*i1.capacityExcess <
            i2.distance + params.critical.penaltyCapacityExcess*i2.capacityExcess - params.error;
              });
	return orderPickedRawIndivs;
}

hsh::cvrp::PathRelinking::PathRelinking( Parameters &params, Eax & eax ) :params(params), eax( eax )
{
	sizeRawIndivs = 0;
	rawIndivs = Vec<Individual>( params.pathRelinking.singleTimeUB, Individual( params ) );
	sizeTempRawIndivs = 0;
	tempRawIndivs = Vec<Individual>( params.pathRelinking.singleTimeUB , Individual( params ) );
	sizePickedRawIndivs = 0;
	pickedRawIndivs = Vec<Individual>( params.pathRelinking.singleTimeUB*params.pathRelinking.sizeGoals , Individual( params ) );
	orderPickedRawIndivs = Vec<int>( params.pathRelinking.singleTimeUB*params.pathRelinking.sizeGoals );
}
