#pragma once
#include "NagataBase.h"

namespace hsh
{
	namespace cvrp
	{
		class NagataLocalSearch :public NagataBase
		{
		public:

			enum SearchMode { FEASIBLE , PENAL };

				/* 局部搜索过程信息 */
			SearchMode searchMode = FEASIBLE; // 邻域动作评估的模式
			//int rangeProximity = 10; // 构造客户节点邻域时所考虑的接近范围
			Move firstImproving; // 记录首次改进的邻域动作
			Vec<int> clientOrder; // 构造邻域的节点次序
			Vec<Move::Type> typeOrder; // 构造邻域的动作类型次序
			Vec<int> orderProximity;

			/* 评估邻域动作, 返回动作是否改进个体 */
			bool evaluateMove( Move &move , Individual &indiv ) override;

			/* 执行局部搜索, 指定评估模式 */
			void run( Individual & indiv );

			/* 设置评估模式 */
			NagataLocalSearch & setSearchMode( SearchMode searchMode );

			NagataLocalSearch( Parameters &params );
		};
	}
}
