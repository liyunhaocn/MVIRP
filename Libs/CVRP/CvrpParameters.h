#pragma once
#include "CvrpTypeDef.h"
#include "../Source/Parameters.h"

namespace hsh
{
	namespace cvrp
	{
		/* �����вֿ��ͻ���ԭʼ��Ϣ */
		struct NodeInfo { 
			int ID;
            // OPTIMIZE[hsh][5]: �������������ʹ��, �����������ɾ��. 
            double coordX;
            double coordY;
			double serviceDuration;
			double demand;
		};

		/* ��k�ӽ��Ľڵ���Ϣ */
		struct Near {
			int ID; // �ڵ� ID
			int reversedProximity; // ����
			double distance;
		};

		class Parameters {
		public:

				/* �������� */
            bool isDurationLimit = false;
            Second runTimeLimit = INT_MAX;
            int numClients;
            int maxClients;
            int numNodes;
            int maxNodes;
			double serviceTime;
			double durationLimit;
			double capacityLimit;
			Vec<NodeInfo> nodeInfo;
			Vec<Vec<double>> distanceMat;
			Vec<Vec<Near>> proximityMat;
			Vec<Vec<int>> lookupProximityMat; // �����±� `[i][j]` ��ѯ�ڵ� j �ǽڵ� i �ĵڼ��ӽ�

				/* ������ */
			double error = 0.00001;

				/* �㷨���ò��� */
			struct
			{
				int maxNumRoutes = 5;
				int numRoutes = -1;
				double penaltyCapacityExcess = .5;
				double penaltyDurationExcess = .5;
			}critical;

			struct
			{
				int size = 10;
				int replaceSecondWorst = 10;
				double balanceFactor = 0.6;
				bool useMinCycles = true;
				int timesGenerateCycles = 5;
			}population;

			struct
			{
				int singleTimeUB = 100;
				int sizeGoals = 3;
				int generateCycleTimes = 10;
				double goalControlFactor = 1.0;
				double beginFactor = 0.4;
				double endFactor = 0.6;
			}pathRelinking;

			struct
			{
				int proximity = 20;
			}nagataLS;

			struct
			{

			}repair;

			struct
			{
				int countStopRunning = 10 * 10 / 2;
			}pathRelinkingWithEax;

			struct {
				bool lk = false;
			}dbg;

			/* ��ӡ������Ϣ */
			void print();

            Parameters() {}
		};
	}
}