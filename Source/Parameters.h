#pragma once
#include "CmdLineParser.h"

namespace hsh {
	namespace mvirp {
        struct Node {
			ID id;
			Coordinate x, y;
			ProdCons prodCons; // 每个节点的生产量/消费量, 消费量为负值, 生产量为正值
			InvenCost holdingCost;
			InvenLevel initLevel, minLevel, maxLevel;
		};

		class Parameters {
		public:
			int numNodes;
			int numClients;
			int numPeriods;
			Capacity vehCapacity;
			int numVehicles;
			Vec<Node> nodes;
			Vec<Vec<Distance>> distances;

			int totTimeLimit = INT_MAX;
			TimeSeed timeSeed = 0;
			double error = 0.00001; // 误差

			Str instDir, instName, instExtName;
			Str slnDir, slnName, slnExtName, slnTag;

			enum  class EliminationPolicy {
				NoSubTour,
				AllSubTours,
				FirstSubTour,
				MinSubTour,
				MinMinSubTour,
				SVRandSubTour, // 随机添加给定长度范围内的单车辆的子回路
				MVRandSubTour // 随机添加给定长度范围内的多车辆的子回路
			};
            struct {
                Second timeOnce = 300;
                int maxThreads = 1;
				double capaUsage = 0.9;
				int szRandSubTour = 10;
				EliminationPolicy policy = EliminationPolicy::MVRandSubTour;
            }finding;
            struct {
                Second timeOnce = 60;
				int maxThreads = 1;
				double capaUsage = 0.9;
				int szRandSubTour = 10;
				EliminationPolicy policy = EliminationPolicy::MVRandSubTour;
            }adjusting;
            struct {
				int szLayer1 = 0;
				int szLayer2 = 0;
				bool delDelivZero = true;
				int maxTimesPtb = 100, timesPtb = 0;
            }refining;
			struct {
				ProdCons maxCons = 0;
			}statistics;

		public:
			bool loadInstance(Str instPath);
			bool setTimeSeed(TimeSeed timeSeed = 0);

			Parameters();
		};
	}
}