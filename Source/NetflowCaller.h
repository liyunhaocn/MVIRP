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
			using CapaMap = typename Graph::template ArcMap<int>; // ����ķ�����ֵ������
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
			HashSet<ID> restoreList; // �ָ�������ͼ
			bool firstRun = true;
			bool check = false;

			// Ϊ������ģ���еĵ�ͱ߱��
			int numPer = params.numPeriods;
			int numNd = params.numNodes;
			int numCli = params.numClients;
			int numVeh = params.numVehicles; // ������ÿ���ڳ����������ֵ
			int totCli = numCli * numPer;
			int totVeh = numVeh * numPer;
			int totNd = 2 + numPer + totVeh + totCli;
			int totArc = numPer // ÿ���ڲֿ������
				+ numNd // ��ʼ���
				+ numNd * numPer // ÿ���ڿ��
				+ totVeh // ÿ���ڲֿ⵽����
				+ numCli * totVeh // ÿ������������
				+ numCli * numPer; // ÿ���ڿͻ���������
			int delivOffset = numPer // ÿ���ڲֿ������
				+ numNd // ��ʼ���
				+ numNd * numPer // ÿ���ڿ��
				+ totVeh; // ÿ���ڲֿ⵽����
			int consOffset = numPer // ÿ���ڲֿ������
				+ numNd // ��ʼ���
				+ numNd * numPer // ÿ���ڿ��
				+ totVeh // ÿ���ڲֿ⵽����
				+ numCli * totVeh; // ÿ������������

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