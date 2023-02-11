#pragma once
#include "CmdLineParser.h"

namespace hsh {
	namespace mvirp {
        struct Node {
			ID id;
			Coordinate x, y;
			ProdCons prodCons; // ÿ���ڵ��������/������, ������Ϊ��ֵ, ������Ϊ��ֵ
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
			double error = 0.00001; // ���

			Str instDir, instName, instExtName;
			Str slnDir, slnName, slnExtName, slnTag;

			enum  class EliminationPolicy {
				NoSubTour,
				AllSubTours,
				FirstSubTour,
				MinSubTour,
				MinMinSubTour,
				SVRandSubTour, // �����Ӹ������ȷ�Χ�ڵĵ��������ӻ�·
				MVRandSubTour // �����Ӹ������ȷ�Χ�ڵĶ೵�����ӻ�·
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