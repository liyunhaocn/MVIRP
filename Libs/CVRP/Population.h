#pragma once
#include "Individual.h"
#include "CrossOver.h"
#include "NagataLocalSearch.h"
#include "Repair.h"

namespace hsh {
	namespace cvrp {

		class Population {
		public:

			Parameters & params;
			Eax &eax;
			NagataLocalSearch &nagataLS;
			Repair &repair;

				/* 种群管理信息 */
			Vec<Individual> indivs; // 种群所有个体
			Vec<Vec<int>> numPrivateEdges;
			Vec<int> minPrivateEdges , backup;
			Vec<int> maxPrivateEdges , backup2;
			Vec<Vec<int>> numCycles;
			Vec<int> minCycles , backup3;
			Vec<double> goodness;

			/* 初始化种群 */
			void initPopulation();

			/* 产生初始解个体 */
			bool generateInitialIndiv( Individual &indiv, NagataLocalSearch &nagataLS , Repair &repair );

			/* 更新种群 */
			void updatePopulation( Individual &offspIndiv , int baseIndivID = -1, int goalIndivID = -1 );

			/* 替换个体 */
			void replaceIndividual( Individual &offspIndiv , int replacedIndivID );
		
			/* 构造函数 */
			Population(Parameters &params, Eax &eax, NagataLocalSearch &nagataLS, Repair &repair);
		};
	}
}