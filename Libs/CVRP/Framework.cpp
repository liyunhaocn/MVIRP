#include "Framework.h"

void hsh::cvrp::PathRelinkingWithEax::reset()
{ 
	overallCountPathRelinking = 0;
	overallCountRawIndivs = 0;
	overallCountMatureIndivs = 0;
}

void hsh::cvrp::PathRelinkingWithEax::run()
{
	reset();
	double timeStart = clock();

	/* 初始化种群 */
	int origPopSz = params.population.size; // 用于恢复默认的种群大小
	pop.initPopulation();
	//if (params.dbg.lk)std::cout << "Pop ready." << std::endl;
	overallBest.reset();
	overallBest.distance = Inf;
	//if (params.dbg.lk)std::cout << "Pop ready done." << std::endl;
	if (params.population.size < 2) {
		params.population.size = origPopSz;
		return;
	}

	/* 开始进化 */
	for ( int countPathRelinking = 0; countPathRelinking < params.pathRelinkingWithEax.countStopRunning; ++countPathRelinking )
	{
		if ( ( clock() - timeStart ) / CLOCKS_PER_SEC > params.runTimeLimit ) { break; }
		//if (params.dbg.lk) std::cout << "PR STARTS." << std::endl;
		const Vec<int> &order = pr.run( pop ); // 由路径重连提供备选原始个体的排序结果
		++overallCountPathRelinking;
		//if (params.dbg.lk) std::cout << "PR FINISHED." << std::endl;

		int bestOffspIndivIndex = -1;
		double bestOffspIndivDistance = Inf;
		for (int index = 0; index < (int)order.size(); ++index)
		{
			Individual &offspIndiv = pr.pickedRawIndivs[order[index]];
			++overallCountRawIndivs;

			if ( offspIndiv.numDepots != params.critical.numRoutes ) // 可能路径数相同, 但是存在子回路
			{
				if (offspIndiv.numSubtours == 0) {
					std::cout << "offsp num routes: " << offspIndiv.numRoutes 
						<< ", pop num routes: " << params.critical.numRoutes << std::endl;
					system("pause"); // TODO[dbg]
				}
				//std::cout << "MERGE BEGINS." << std::endl;
				repair.mergeSubtours( offspIndiv );
				//offspIndiv.inspectLinks( true );
				//std::cout << "OFFSP MERGE FINISHED." << std::endl;
			}
			else
			{
				//std::cout << "OFFSP MERGE PASSED." << std::endl;
			}
			offspIndiv.calculateCompletely();
			if ( offspIndiv.isFeasible != true )
			{
				repair.setProximityMode( Repair::ProximityMode::GRANULAR ).eliminateExcess( offspIndiv );
				//offspIndiv.inspectLinks( true );
				//std::cout << "OFFSP ELIMINATE EXCESS FINISHED." << std::endl;
				if ( offspIndiv.isFeasible != true )
				{
					continue; //TODO[或者添加一个非可行解池]
				}
				++overallCountMatureIndivs;
			}
			else
			{
				//offspIndiv.inspectLinks( true );
				//std::cout << "OFFSP ELIMINATE EXCESS PASSED." << std::endl;
			}
			//offspIndiv.inspectLinks( true );
			nagataLS.setSearchMode( NagataLocalSearch::SearchMode::FEASIBLE ).run( offspIndiv );
			//std::cout << "OFFSP LS FINISHED." << std::endl;
			if ( offspIndiv.distance < bestOffspIndivDistance - params.error )
			{
				bestOffspIndivDistance = offspIndiv.distance;
				bestOffspIndivIndex = index;
			}
		}
		//if (params.dbg.lk)std::cout << "PR FINISHED." << std::endl;
		/* 控制种群的更新 */
		if ( bestOffspIndivIndex == -1 )
		{
			continue;
		}
		else
		{
			//if (params.dbg.lk)std::cout << "order[bestOffspIndivIndex]:" << order[bestOffspIndivIndex] << std::endl;
			Individual &bestOffsp = pr.pickedRawIndivs[order[bestOffspIndivIndex]];
			//if (params.dbg.lk)std::cout << "base id " << bestOffsp.baseIndivID << ", goal id " << bestOffsp.goalIndivID << std::endl;
			Individual &base = pop.indivs[bestOffsp.baseIndivID];
			Individual &goal = pop.indivs[bestOffsp.goalIndivID];
			//if (params.dbg.lk)std::cout << "base " << base.distance << ", goal " << goal.distance << std::endl;
			if ( bestOffsp.distance < overallBest.distance - params.error )
			{
				overallBest.copyAllFrom( bestOffsp );
				countPathRelinking = -1;
				/*std::cout << "| DISTANCE " << overallBest.distance
					<< " | FEASIBLE " << overallBest.isFeasible
					<< " | TIME " << ( clock() - timeStart ) / CLOCKS_PER_SEC << "s |"
					<< std::endl;*/
			}

			bool replaceBase = ( bestOffsp.distance < base.distance - params.error );
			bool replaceGoal = ( bestOffsp.distance < goal.distance - params.error );
			if ( replaceBase && replaceGoal )
			{
				if ( bestOffsp.numAccumulation <= bestOffsp.numPrivateEdges / 2 )
				{
					pop.updatePopulation( bestOffsp , bestOffsp.baseIndivID , bestOffsp.baseIndivID );
				}
				else
				{
					pop.updatePopulation( bestOffsp , bestOffsp.goalIndivID , bestOffsp.goalIndivID );
				}
			}
			else
			{
				if ( replaceBase )
				{
					pop.updatePopulation( bestOffsp , bestOffsp.baseIndivID , bestOffsp.baseIndivID );
				}
				else
				{
					if ( replaceGoal )
					{
						pop.updatePopulation( bestOffsp , bestOffsp.goalIndivID , bestOffsp.goalIndivID );
					}
					else
					{
						pop.updatePopulation( bestOffsp , bestOffsp.baseIndivID , bestOffsp.goalIndivID );
					}
				}
			}
		}
	}

	/*for ( int indivID = 0; indivID < params.population.size; ++indivID ) {
		std::cout << "[" << indivID <<": "<<pop.indivs[indivID].isFeasible << "]" << pop.indivs[indivID].distance << " ";
	}std::cout << std::endl;*/

	//std::cout << overallBest.distance << std::endl;
	//std::cout << "FINISHED AT " << ( clock() - timeStart ) / CLOCKS_PER_SEC << "s" << std::endl;

	//historicalResults.push_back( overallBest.distance );
	/*std::cout << "HISTORY: " << std::endl;
	for ( int runID = 0; runID < historicalResults.size(); ++runID ) {
		std::cout << "[" << runID << "]" << historicalResults[runID] << " "; 
	}std::cout << std::endl;*/

	params.population.size = origPopSz;
}

hsh::cvrp::PathRelinkingWithEax::PathRelinkingWithEax( Parameters & params ,
													   Population & pop , 
													   Eax & eax , 
													   PathRelinking & pr ,
													   NagataLocalSearch & nagataLS ,
													   Repair & repair )
	:params(params), pop(pop), eax(eax), pr(pr), nagataLS(nagataLS), repair(repair), overallBest(params)
{ }
