#pragma once
#include "CvrpTypeDef.h"
#include "../Source/Parameters.h"

namespace hsh
{
	namespace cvrp
	{
		/* 算例中仓库或客户的原始信息 */
		struct NodeInfo { 
			int ID;
            // OPTIMIZE[hsh][5]: 点坐标仅供测试使用, 待测试无误后删除. 
            double coordX;
            double coordY;
			double serviceDuration;
			double demand;
		};

		/* 第k接近的节点信息 */
		struct Near {
			int ID; // 节点 ID
			int reversedProximity; // 反向
			double distance;
		};

		class Parameters {
		public:

				/* 算例参数 */
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
			Vec<Vec<int>> lookupProximityMat; // 根据下标 `[i][j]` 查询节点 j 是节点 i 的第几接近

				/* 误差参数 */
			double error = 0.00001;

				/* 算法配置参数 */
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

			/* 打印算例信息 */
			void print();

            Parameters() {}
		};
	}
}