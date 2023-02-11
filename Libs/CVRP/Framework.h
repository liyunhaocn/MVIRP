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

				/* ����������Ϣ */
			int overallCountPathRelinking = 0; // ȫ��·������ ID
			int overallCountRawIndivs = 0;
			int overallCountMatureIndivs = 0;

				/* ��ʷ��Ϣ */
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