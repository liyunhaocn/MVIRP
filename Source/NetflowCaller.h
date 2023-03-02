#pragma once
#include "Parameters.h"
#include "Solution.h"

#include "../Libs/lemon/network_simplex.h"
#include "../Libs/lemon/cost_scaling.h"
#include "../Libs/lemon/capacity_scaling.h"
#include "../Libs/lemon/cycle_canceling.h"
#include "../Libs/lemon/smart_graph.h"
#include "../Libs/lemon/full_graph.h"

namespace hsh {
	namespace mvirp {
		class NetflowCaller {
		public:
			using Graph = typename lemon::SmartDigraph;
			using ValType = int;
			using CostType = double;
			using NetSplex = typename lemon::NetworkSimplex<Graph, ValType, CostType>;
			using CostScal = typename lemon::CostScaling<Graph, ValType, CostType>;
			using CapaMap = typename Graph::template ArcMap<int>; // 这里的泛型是值的类型
			using CostMap = typename Graph::template ArcMap<int>;
			using DemaMap = typename Graph::template ArcMap<int>;
			Parameters &params;
			Graph g;
			CostMap *costPtr = nullptr;
			CapaMap *capaPtr = nullptr;
			DemaMap *demaPtr = nullptr;
			int flowAmount = 0;
			NetSplex* netSplexPtr;
			CostScal *costScalPtr;
			int precise = 1000;
			HashSet<ID> restoreList; // 恢复到完整图
			bool firstRun = true;
			bool check = false;

			// 为网络流模型中的点和边编号
			int numPer = params.numPeriods;
			int numNd = params.numNodes;
			int numCli = params.numClients;
			int numVeh = params.numVehicles; // 这里是每周期车辆数的最大值
			int totCli = numCli * numPer;
			int totVeh = numVeh * numPer;
			int totNd = 2 + numPer + totVeh + totCli;
			int totArc = numPer // 每周期仓库产生量
				+ numNd // 初始库存
				+ numNd * numPer // 每周期库存
				+ totVeh // 每周期仓库到车辆
				+ numCli * totVeh // 每辆车的配送量
				+ numCli * numPer; // 每周期客户的消耗量
			int delivOffset = numPer // 每周期仓库产生量
				+ numNd // 初始库存
				+ numNd * numPer // 每周期库存
				+ totVeh; // 每周期仓库到车辆
			int consOffset = numPer // 每周期仓库产生量
				+ numNd // 初始库存
				+ numNd * numPer // 每周期库存
				+ totVeh // 每周期仓库到车辆
				+ numCli * totVeh; // 每辆车的配送量

		public:
			NetflowCaller(Parameters &params):params(params) {
				init();
			}
			~NetflowCaller() { delete costPtr; delete capaPtr; delete demaPtr; delete netSplexPtr; }

			void init();
			void setCheck(bool b) { check = b; }
			InvenCost run(Solution &sln, bool assignDeliv = false);
		};
	}
}