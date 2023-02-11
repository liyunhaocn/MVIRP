/*
	完成将子图转换成 CVRP 求解器的输入, 200个点的矩阵计算和邻近排序需要0.009s完成
*/

#pragma once
#include "TypeDef.h"
#include "Solution.h"
#include "../Libs/CVRP/CvrpParameters.h"
#include "../Libs/CVRP/NagataLocalSearch.h"
#include "../Libs/CVRP/Repair.h"
#include "../Libs/CVRP/Population.h"
#include "../Libs/CVRP/Framework.h"

namespace hsh {
	namespace mvirp {

		class CvrpCaller {
		public:
            mvirp::Parameters &mvirpParams;
            cvrp::Parameters cvrpParams;

            Vec<int> nodeMap; // 将节点 Id 从小到大的顺序编号, 索引即为算例中的节点 Id
            Vec<int> revNdMap; // 反向查找

            enum class Mode { GenInit, AsInit, Pop };
            
            // TODO[hsh][5]: 增加运行时间限制.

        public:
            /*求解给定周期的 CVRP 子问题 */
			bool solve(ID period, Solution &sln, Mode mode, int specNumRou, double replacedThres);
            /* 将给定周期的子图转换为 CVRP 算例 */
            void subgraphToInstance(ID periodId, Solution &sln, int specNumRou);
            /* 求解算例 */
            bool runGenInit(ID periodId, Solution &sln, int specNumRou, double repalcedThres);
            // 以当前解的路由作为CVRP初始解
            bool runAsInitSln(ID periodId, Solution &sln, int specNumRou, double repalcedThres);
            // 生成种群进行CVRP优化
            bool runWithPop(ID periodId, Solution &sln, int specNumRou, double repalcedThres);
            /* 将求解结果填入 Routes. */
            void indivToRoutes(cvrp::Individual &indiv, Routes &routes, Vec<Visit> &visits);
		    
            /* 构造 CVRP 调用器, 初始化相关数据 */
            CvrpCaller(mvirp::Parameters &params);
        };
	}
}