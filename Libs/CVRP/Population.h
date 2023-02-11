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

				/* ��Ⱥ������Ϣ */
			Vec<Individual> indivs; // ��Ⱥ���и���
			Vec<Vec<int>> numPrivateEdges;
			Vec<int> minPrivateEdges , backup;
			Vec<int> maxPrivateEdges , backup2;
			Vec<Vec<int>> numCycles;
			Vec<int> minCycles , backup3;
			Vec<double> goodness;

			/* ��ʼ����Ⱥ */
			void initPopulation();

			/* ������ʼ����� */
			bool generateInitialIndiv( Individual &indiv, NagataLocalSearch &nagataLS , Repair &repair );

			/* ������Ⱥ */
			void updatePopulation( Individual &offspIndiv , int baseIndivID = -1, int goalIndivID = -1 );

			/* �滻���� */
			void replaceIndividual( Individual &offspIndiv , int replacedIndivID );
		
			/* ���캯�� */
			Population(Parameters &params, Eax &eax, NagataLocalSearch &nagataLS, Repair &repair);
		};
	}
}