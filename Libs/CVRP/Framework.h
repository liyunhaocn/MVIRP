#pragma once
#include "Population.h"
#include "PathRelinking.h"
#include "NagataLocalSearch.h"
#include "Repair.h"

namespace hsh {
	namespace cvrp {

		class Framework {
		public:
			
		};

		class PathRelinkingWithEax :public Framework
		{
		public:

			Parameters &params;
			Population &pop;
			Eax &eax;
			PathRelinking &pr;
			NagataLocalSearch &nagataLS;
			Repair &repair;

				/* 进化过程信息 */
			int overallCountPathRelinking = 0; // 全局路径重连 ID
			int overallCountRawIndivs = 0;
			int overallCountMatureIndivs = 0;

				/* 历史信息 */
			Vec<double> historicalResults;
			Individual overallBest;

			void reset();

			void run();

			PathRelinkingWithEax( Parameters &params ,
								  Population &pop ,
								  Eax &eax,
								  PathRelinking &pr,
								  NagataLocalSearch &nagataLS ,
								  Repair &repair );
		};

	}
}